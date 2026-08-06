"""
udp_telemetry.py  —  Continuous Slave → Master Telemetry over UDP
==================================================================
Architecture
------------
• Every slave drone broadcasts a compact JSON packet at ~5 Hz (every 200 ms)
  to a single well-known UDP port on the master.
• The master runs ONE asyncio DatagramProtocol server that receives ALL
  slave packets on that single port — no per-slave socket needed.
• The drone_id field inside each packet demuxes which slave sent it.
• On receiving a packet the handler:
    1. Validates and parses the JSON.
    2. Updates SwarmState (async, lock-protected).
    3. Posts the updated snapshot to the Flask ground server via HTTP
       (non-blocking — fires and forgets if the server is busy).
    4. Logs anomalies (stale drones, sudden altitude drops, etc.).

Why one port / one socket?
  With N=3 slaves each at 5 Hz you have 15 packets/s.  A single asyncio
  event loop handles this with near-zero CPU.  Adding more slaves costs
  zero additional OS resources.

Packet format (slave → master, every 200 ms)
--------------------------------------------
{
  "drone_id":  "slave_1",          # "slave_1" | "slave_2" | "slave_3"
  "lat":       12.971599,
  "lng":       77.594563,
  "altitude":  3.2,                # metres AGL
  "heading":   47.3,               # degrees 0-360
  "speed":     2.1,                # m/s horizontal
  "armed":     true,
  "airborne":  true,
  "bat_pct":   82,                 # battery %
  "sensor":    0.04,               # metal-detector reading (0-1)
  "seq":       1042                # monotonic packet counter (drop stale)
}

Mine detection logic (server-side, in this file)
-------------------------------------------------
sensor > SENSOR_THRESH  for  PERSIST_N  consecutive packets
→ emit mine_detected event → STATE.add_mine() → forward to ground server

Stale drone detection
---------------------
A watchdog coroutine runs every second.  If a drone has not sent a
packet in > STALE_TIMEOUT_S it is flagged as stale in the snapshot,
which makes the map dim that drone and the collision checker ignore it.
"""

import asyncio
import json
import logging
import math
import time
from collections import defaultdict

import aiohttp   # pip install aiohttp

from swarm_state import STATE, DRONE_IDS

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ════════════════════════════════════════════════════════════════════════════
UDP_HOST         = "0.0.0.0"
UDP_PORT         = 14550          # all slaves send here
GROUND_SERVER    = "https://localhost"   # Flask app.py (same Pi)
STALE_TIMEOUT_S  = 2.0            # drone gone silent → flag stale
SENSOR_THRESH    = 0.65           # metal-detector reading threshold
PERSIST_N        = 5              # N consecutive hits → confirmed mine
WATCHDOG_HZ      = 1.0            # stale-check interval (seconds)

log = logging.getLogger("udp_telemetry")
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)


# ════════════════════════════════════════════════════════════════════════════
#  MINE PERSISTENCE TRACKER
# ════════════════════════════════════════════════════════════════════════════
class MinePersistenceTracker:
    """
    Counts consecutive above-threshold sensor readings per drone.
    Once a drone hits PERSIST_N readings, a mine is confirmed and the
    counter resets so the same spot isn't double-counted.

    Separate instance per drone, held in a dict keyed by drone_id.
    """
    def __init__(self):
        self._count:    dict[str, int]   = defaultdict(int)
        self._last_pos: dict[str, tuple] = {}   # last (lat, lng) at trigger

    def feed(self, drone_id: str, sensor: float, lat: float, lng: float) -> bool:
        """
        Returns True (once) when a mine is confirmed.
        """
        if sensor >= SENSOR_THRESH:
            self._count[drone_id]    += 1
            self._last_pos[drone_id]  = (lat, lng)
        else:
            # Reading dropped — reset streak
            self._count[drone_id] = 0

        if self._count[drone_id] >= PERSIST_N:
            self._count[drone_id] = 0   # reset — don't fire twice
            return True
        return False


_mine_tracker = MinePersistenceTracker()


# ════════════════════════════════════════════════════════════════════════════
#  UDP PROTOCOL HANDLER
# ════════════════════════════════════════════════════════════════════════════
class TelemetryProtocol(asyncio.DatagramProtocol):
    """
    asyncio DatagramProtocol — one instance handles ALL slave packets.

    asyncio guarantees that datagram_received() is called serially inside
    the event loop — there is NO parallel execution here even with 3 slaves
    sending simultaneously.  The event loop queues the callbacks and fires
    them one at a time, so we never need a thread lock in this handler.

    The STATE.update_drone() call DOES acquire the async lock internally,
    which is correct because other coroutines (TCP commander, watchdog)
    may also be accessing state.
    """

    def __init__(self):
        self.transport   = None
        self._seq: dict[str, int] = {}   # last seen seq per drone

    def connection_made(self, transport):
        self.transport = transport
        log.info(f"UDP telemetry listening on {UDP_HOST}:{UDP_PORT}")

    def datagram_received(self, data: bytes, addr: tuple):
        """Called by the event loop for every incoming UDP datagram."""
        try:
            pkt = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            log.warning(f"Bad packet from {addr}: {e}")
            return

        did = pkt.get("drone_id")
        if did not in DRONE_IDS:
            log.warning(f"Unknown drone_id '{did}' from {addr}")
            return

        # ── Stale-packet rejection ──────────────────────────────────────
        seq = pkt.get("seq", 0)
        if did in self._seq and seq <= self._seq[did]:
            # Out-of-order or duplicate — discard silently
            return
        self._seq[did] = seq

        # ── Schedule async processing on the event loop ─────────────────
        # We can't await here (sync callback), so we schedule coroutines.
        loop = asyncio.get_event_loop()
        loop.create_task(_process_telemetry(pkt, addr))

    def error_received(self, exc):
        log.error(f"UDP error: {exc}")

    def connection_lost(self, exc):
        log.warning("UDP socket closed")


# ════════════════════════════════════════════════════════════════════════════
#  ASYNC TELEMETRY PROCESSOR
# ════════════════════════════════════════════════════════════════════════════
async def _process_telemetry(pkt: dict, addr: tuple) -> None:
    """
    Coroutine scheduled for each incoming packet.

    Steps:
      1. Update shared SwarmState (acquires internal async lock).
      2. Check metal-detector sensor for mine confirmation.
      3. Forward telemetry to Flask ground server (fire-and-forget HTTP POST).
    """
    did    = pkt["drone_id"]
    lat    = float(pkt.get("lat",      0))
    lng    = float(pkt.get("lng",      0))
    sensor = float(pkt.get("sensor",   0))

    # 1 ── Update position in shared state ──────────────────────────────
    await STATE.update_drone(pkt)

    # 2 ── Mine detection ────────────────────────────────────────────────
    if _mine_tracker.feed(did, sensor, lat, lng):
        log.info(f"[MINE CONFIRMED] by {did} at {lat:.6f},{lng:.6f}")
        mine = await STATE.add_mine(lat, lng, detected_by=did)
        # Forward mine event to Flask ground server
        asyncio.create_task(
            _post_to_ground({
                "type":     "mine_detected",
                "lat":      lat,
                "lng":      lng,
                "drone_id": did,
            })
        )

    # 3 ── Forward position to Flask ground server ───────────────────────
    asyncio.create_task(
        _post_to_ground({
            "type":      "drone_position",
            "drone_id":  did,
            "lat":       lat,
            "lng":       lng,
            "heading":   float(pkt.get("heading",  0)),
            "altitude":  float(pkt.get("altitude", 0)),
        })
    )


# ════════════════════════════════════════════════════════════════════════════
#  HTTP FORWARD TO FLASK GROUND SERVER  (fire and forget)
# ════════════════════════════════════════════════════════════════════════════
_http_session: aiohttp.ClientSession | None = None

async def _get_http_session() -> aiohttp.ClientSession:
    global _http_session
    if _http_session is None or _http_session.closed:
        # verify=False because we use a self-signed cert on the Pi
        connector = aiohttp.TCPConnector(ssl=False)
        _http_session = aiohttp.ClientSession(connector=connector)
    return _http_session

async def _post_to_ground(payload: dict) -> None:
    """
    Non-blocking POST to Flask /api/drone_update.
    If the ground server is unreachable, we log and move on —
    telemetry never waits for the HTTP response.
    """
    try:
        session = await _get_http_session()
        async with session.post(
            f"{GROUND_SERVER}/api/drone_update",
            json=payload,
            timeout=aiohttp.ClientTimeout(total=0.5),
        ) as resp:
            if resp.status != 200:
                log.warning(f"Ground server returned {resp.status}")
    except Exception as e:
        # Ground server unreachable — not fatal for the swarm
        log.debug(f"Ground server post failed: {e}")


# ════════════════════════════════════════════════════════════════════════════
#  STALE DRONE WATCHDOG
# ════════════════════════════════════════════════════════════════════════════
async def _watchdog() -> None:
    """
    Runs every second.  Logs a warning for any drone that has gone silent.
    The state itself is updated lazily — DroneState.is_stale() checks age
    dynamically, so no mutation needed here.

    If a drone goes stale while airborne, we push an emergency LAND command
    onto its command queue so the TCP commander sends it immediately.
    """
    while True:
        await asyncio.sleep(1.0 / WATCHDOG_HZ)
        async with STATE.lock:
            snap = STATE.snapshot()   # cheap read while holding lock
        for did, d in snap["drones"].items():
            if did == "master":
                continue
            if d["stale"] and d["airborne"]:
                log.warning(f"[WATCHDOG] {did} STALE + AIRBORNE — queuing LAND")
                await STATE.cmd_queues[did].put({
                    "cmd":    "LAND",
                    "reason": "stale_telemetry",
                })


# ════════════════════════════════════════════════════════════════════════════
#  SLAVE-SIDE: broadcast loop (runs ON each slave drone)
# ════════════════════════════════════════════════════════════════════════════
async def slave_broadcast_loop(
    drone_id:    str,
    master_host: str,
    master_port: int = UDP_PORT,
    hz:          float = 5.0,
) -> None:
    """
    Run this coroutine on each slave Pi.
    Reads MAVLink/MAVSDK telemetry (replace the stub below with real reads)
    and broadcasts a JSON packet to the master at `hz` Hz.

    Sequence numbers are monotonically increasing so the master can
    detect and discard out-of-order packets.
    """
    interval = 1.0 / hz
    seq      = 0

    # Create a UDP socket (sendto only — no bind needed on slave)
    loop      = asyncio.get_event_loop()
    transport, _ = await loop.create_datagram_endpoint(
        asyncio.DatagramProtocol,
        remote_addr=(master_host, master_port),
    )

    log.info(f"[{drone_id}] Broadcasting telemetry → {master_host}:{master_port} @ {hz} Hz")

    try:
        while True:
            t0 = time.monotonic()

            # ── Read real telemetry here (MAVSDK / MAVLink) ──────────────
            # Replace this stub with your actual MAVSDK telemetry reads.
            # Example with MAVSDK-Python:
            #   pos = await drone.telemetry.position().__aiter__().__anext__()
            #   hdg = await drone.telemetry.heading().__aiter__().__anext__()
            telemetry = _read_telemetry_stub(drone_id, seq)
            # ─────────────────────────────────────────────────────────────

            pkt = json.dumps({
                "drone_id":  drone_id,
                "lat":       telemetry["lat"],
                "lng":       telemetry["lng"],
                "altitude":  telemetry["alt"],
                "heading":   telemetry["heading"],
                "speed":     telemetry["speed"],
                "armed":     telemetry["armed"],
                "airborne":  telemetry["airborne"],
                "bat_pct":   telemetry["bat_pct"],
                "sensor":    telemetry["sensor"],
                "seq":       seq,
            }).encode("utf-8")

            transport.sendto(pkt)
            seq += 1

            # Sleep for the remainder of the interval
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, interval - elapsed))
    finally:
        transport.close()


def _read_telemetry_stub(drone_id: str, seq: int) -> dict:
    """
    STUB — replace with real MAVSDK reads on each slave.
    Returns a dict with the fields the packet expects.
    """
    import random
    t = seq * 0.2   # pretend 5 Hz
    offsets = {"slave_1": 0, "slave_2": 5, "slave_3": 10}
    off = offsets.get(drone_id, 0)
    return {
        "lat":      12.971599 + off * 0.00001 + math.sin(t * 0.1) * 0.00005,
        "lng":      77.594563 + math.cos(t * 0.1) * 0.00005,
        "alt":      3.0 + math.sin(t * 0.5) * 0.3,
        "heading":  (t * 10 + off * 120) % 360,
        "speed":    2.0 + random.uniform(-0.2, 0.2),
        "armed":    True,
        "airborne": True,
        "bat_pct":  max(0, 100 - seq // 50),
        "sensor":   0.9 if 30 <= seq <= 34 else random.uniform(0.0, 0.1),
    }


# ════════════════════════════════════════════════════════════════════════════
#  MASTER-SIDE ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════
async def start_udp_server() -> None:
    """
    Start the UDP telemetry server on the master.
    Call this from your main event loop alongside start_tcp_commander().

    Example (in master.py):
        await asyncio.gather(
            udp_telemetry.start_udp_server(),
            tcp_commander.start_tcp_commander(),
        )
    """
    loop = asyncio.get_event_loop()

    # Create the UDP endpoint — ONE socket, all drones send here
    transport, protocol = await loop.create_datagram_endpoint(
        TelemetryProtocol,
        local_addr=(UDP_HOST, UDP_PORT),
    )

    log.info(f"UDP telemetry server ready on {UDP_HOST}:{UDP_PORT}")

    # Start the stale-drone watchdog
    asyncio.create_task(_watchdog())

    # Keep running until cancelled
    try:
        await asyncio.Future()   # run forever
    finally:
        transport.close()
        if _http_session and not _http_session.closed:
            await _http_session.close()
