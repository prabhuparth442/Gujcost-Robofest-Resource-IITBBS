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
    2. Updates SwarmState (threading.Lock protected — sync call).
    3. Posts the updated snapshot to the Flask ground server via HTTP
       (non-blocking — fires and forgets if the server is busy).
    4. Logs anomalies (stale drones, sudden altitude drops, etc.).

Packet format (slave → master, every 200 ms)
--------------------------------------------
{
  "drone_id":  "slave_1",
  "lat":       23.077953,
  "lng":       72.495347,
  "altitude":  3.2,           # metres AGL
  "alt_amsl":  45.7,          # metres AMSL absolute
  "heading":   47.3,
  "speed":     2.1,
  "armed":     true,
  "airborne":  true,
  "bat_pct":   82,
  "sensor":    0.04,          # metal-detector 0.0-1.0
  "seq":       1042
}

FIXES vs original
-----------------
  FIX — _process_telemetry() called  await STATE.update_drone()  and
         await STATE.add_mine() .  After swarm_state.py was fixed to use
         threading.Lock, those methods became plain synchronous  def .
         Awaiting a non-coroutine returns the value immediately but wastes
         a scheduler cycle; worse, if Python ever rejects awaiting a non-
         awaitable it raises TypeError at runtime.
         Fix: removed all  await  prefixes from STATE calls.
         Same fix applied in  _handle_mine_connection()  (anomaly_report
         and sector_result branches).
         _watchdog() used  async with STATE.lock  — replaced with a plain
         synchronous  with STATE.lock  block since lock is now threading.Lock.
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
UDP_HOST        = "0.0.0.0"
UDP_PORT        = 14550
GROUND_SERVER   = "https://10.42.0.1"   # Flask app.py on master Pi
STALE_TIMEOUT_S = 2.0
SENSOR_THRESH   = 0.65
PERSIST_N       = 5
WATCHDOG_HZ     = 1.0

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
    Fires exactly ONCE per confirmed mine then resets.
    """
    def __init__(self):
        self._count:    dict[str, int]   = defaultdict(int)
        self._last_pos: dict[str, tuple] = {}

    def feed(self, drone_id: str, sensor: float, lat: float, lng: float) -> bool:
        if sensor >= SENSOR_THRESH:
            self._count[drone_id]   += 1
            self._last_pos[drone_id] = (lat, lng)
        else:
            self._count[drone_id] = 0

        if self._count[drone_id] >= PERSIST_N:
            self._count[drone_id] = 0
            return True
        return False


_mine_tracker = MinePersistenceTracker()


# ════════════════════════════════════════════════════════════════════════════
#  UDP PROTOCOL HANDLER
# ════════════════════════════════════════════════════════════════════════════
class TelemetryProtocol(asyncio.DatagramProtocol):
    """
    asyncio DatagramProtocol — one instance handles ALL slave packets.
    datagram_received() is a sync callback; we schedule async processing
    via loop.create_task() so HTTP forwards don't block packet receipt.
    """

    def __init__(self):
        self.transport = None
        self._seq: dict[str, int] = {}

    def connection_made(self, transport):
        self.transport = transport
        log.info(f"UDP telemetry listening on {UDP_HOST}:{UDP_PORT}")

    def datagram_received(self, data: bytes, addr: tuple):
        try:
            pkt = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            log.warning(f"Bad packet from {addr}: {e}")
            return

        did = pkt.get("drone_id")
        if did not in DRONE_IDS:
            log.warning(f"Unknown drone_id '{did}' from {addr}")
            return

        # Stale-packet rejection
        seq = pkt.get("seq", 0)
        if did in self._seq and seq <= self._seq[did]:
            return
        self._seq[did] = seq

        # Schedule async HTTP forward; STATE update happens synchronously here
        # so it is always written before any other coroutine sees this packet.
        _update_state(pkt)
        loop = asyncio.get_running_loop()
        loop.create_task(_async_forward(pkt, addr))

    def error_received(self, exc):
        log.error(f"UDP error: {exc}")

    def connection_lost(self, exc):
        log.warning("UDP socket closed")


# ════════════════════════════════════════════════════════════════════════════
#  SYNCHRONOUS STATE UPDATE  (called directly in datagram_received)
# ════════════════════════════════════════════════════════════════════════════
def _update_state(pkt: dict) -> None:
    """
    Write telemetry into SwarmState and check for mine confirmation.
    Both STATE calls are plain synchronous functions (threading.Lock inside).

    FIX: original code scheduled these as coroutines with `await STATE.*`
         After swarm_state.py was fixed to use threading.Lock, those methods
         are no longer coroutines — awaiting them would TypeError at runtime.
         They are now called directly here in the sync datagram_received path.
    """
    did    = pkt.get("drone_id", "")
    lat    = float(pkt.get("lat",    0))
    lng    = float(pkt.get("lng",    0))
    sensor = float(pkt.get("sensor", 0))

    # FIX: was  await STATE.update_drone(pkt)
    STATE.update_drone(pkt)

    log.debug(
        f"[UDP] {did} seq={pkt.get('seq')} "
        f"lat={lat:.5f} lng={lng:.5f} "
        f"alt={pkt.get('altitude',0):.1f}m sensor={sensor:.2f}"
    )

    if _mine_tracker.feed(did, sensor, lat, lng):
        log.info(f"[MINE CONFIRMED] by {did} at {lat:.6f},{lng:.6f}")
        # FIX: was  mine = await STATE.add_mine(...)
        mine = STATE.add_mine(lat, lng, detected_by=did)
        log.info(f"[MINE] Added #{mine.mine_id} at ({mine.x:.1f}m, {mine.y:.1f}m)")
        # HTTP forward scheduled by caller (_async_forward sees sensor value)


# ════════════════════════════════════════════════════════════════════════════
#  ASYNC HTTP FORWARD  (fire-and-forget, runs in event loop)
# ════════════════════════════════════════════════════════════════════════════
async def _async_forward(pkt: dict, addr: tuple) -> None:
    """
    Forward position (and mine event if sensor triggered) to Flask app.py.
    Runs as a background task — never blocks packet receipt.
    """
    did    = pkt.get("drone_id", "")
    lat    = float(pkt.get("lat",    0))
    lng    = float(pkt.get("lng",    0))
    sensor = float(pkt.get("sensor", 0))

    # Always forward position
    await _post_to_ground({
        "type":      "drone_position",
        "drone_id":  did,
        "lat":       lat,
        "lng":       lng,
        "heading":   float(pkt.get("heading",  0)),
        "altitude":  float(pkt.get("altitude", 0)),
    })

    # Forward mine event if this packet completed a persistence run
    # (mine was already added to STATE in _update_state above)
    if sensor >= SENSOR_THRESH:
        # Check if a mine was just confirmed (count reset to 0 means it fired)
        # We re-check STATE.mines for a recent entry at this position
        with STATE.lock:
            recent = [
                m for m in STATE.mines
                if abs(m.lat - lat) < 0.0001 and abs(m.lng - lng) < 0.0001
            ]
        if recent:
            await _post_to_ground({
                "type":     "mine_detected",
                "lat":      lat,
                "lng":      lng,
                "drone_id": did,
            })


# ════════════════════════════════════════════════════════════════════════════
#  HTTP FORWARD TO FLASK GROUND SERVER
# ════════════════════════════════════════════════════════════════════════════
_http_session: aiohttp.ClientSession | None = None


async def _get_http_session() -> aiohttp.ClientSession:
    global _http_session
    if _http_session is None or _http_session.closed:
        connector = aiohttp.TCPConnector(ssl=False)   # self-signed cert on Pi
        _http_session = aiohttp.ClientSession(connector=connector)
    return _http_session


async def _post_to_ground(payload: dict) -> None:
    """Non-blocking POST to Flask /api/drone_update."""
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
        log.debug(f"Ground server post failed: {e}")


# ════════════════════════════════════════════════════════════════════════════
#  STALE DRONE WATCHDOG
# ════════════════════════════════════════════════════════════════════════════
async def _watchdog() -> None:
    """
    Checks every second for drones that have gone silent.
    If a drone is stale AND airborne, queues an emergency LAND.

    FIX: original used  async with STATE.lock  — lock is now threading.Lock,
         which must be acquired with plain  with STATE.lock  (not async with).
    """
    while True:
        await asyncio.sleep(1.0 / WATCHDOG_HZ)

        # FIX: was  async with STATE.lock — now plain threading.Lock
        with STATE.lock:
            snap = STATE.snapshot()

        for did, d in snap["drones"].items():
            if did == "master":
                continue
            if d["stale"] and d["airborne"]:
                log.warning(f"[WATCHDOG] {did} STALE + AIRBORNE — queuing LAND")
                STATE.cmd_queues[did].put_nowait({
                    "cmd":    "LAND",
                    "reason": "stale_telemetry",
                })


# ════════════════════════════════════════════════════════════════════════════
#  MINE TCP LISTENER  (DroneTunnel anomaly_report packets)
# ════════════════════════════════════════════════════════════════════════════
MINE_TCP_PORT = 5000


async def _handle_mine_connection(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
) -> None:
    """
    Handles one DroneTunnel connection from a slave.
    Wire format: 4-byte big-endian length prefix + JSON body.

    FIX: all  await STATE.add_mine()  calls replaced with plain
         STATE.add_mine()  (sync method with threading.Lock).
    """
    peer = writer.get_extra_info("peername")
    log.info(f"[MINE TCP] Connection from {peer}")
    try:
        len_bytes   = await asyncio.wait_for(reader.readexactly(4), timeout=10.0)
        payload_len = int.from_bytes(len_bytes, "big")

        if payload_len > 5_000_000:
            log.warning(f"[MINE TCP] Oversized packet {payload_len}B — dropping")
            return

        raw = await asyncio.wait_for(reader.readexactly(payload_len), timeout=10.0)
        pkt = json.loads(raw.decode("utf-8"))

        pkt_type = pkt.get("type", "unknown")
        log.info(f"[MINE TCP] type='{pkt_type}'  size={payload_len}B  from {peer}")

        if pkt_type == "anomaly_report":
            lat      = float(pkt.get("latitude",  0))
            lng      = float(pkt.get("longitude", 0))
            drone_id = pkt.get("drone_id", str(peer[0]))

            if abs(lat) < 0.001 and abs(lng) < 0.001:
                log.warning(f"[MINE TCP] Zeroed GPS from {drone_id} — ignoring")
                return

            log.info(f"[MINE TCP] Mine report from {drone_id}: ({lat:.6f},{lng:.6f})")
            # FIX: was  mine = await STATE.add_mine(...)
            mine = STATE.add_mine(lat, lng, detected_by=drone_id)
            log.info(f"[MINE TCP] Added to STATE as mine #{mine.mine_id}")

            asyncio.create_task(_post_to_ground({
                "type":     "mine_detected",
                "lat":      lat,
                "lng":      lng,
                "drone_id": drone_id,
            }))

        elif pkt_type == "sector_result":
            found    = pkt.get("mine_found", False)
            drone_id = pkt.get("drone_id", "?")
            sector   = pkt.get("sector_id", "?")
            conf     = pkt.get("confidence", 0.0)
            log.info(
                f"[MINE TCP] sector_result from {drone_id}: "
                f"sector={sector} mine_found={found} conf={conf*100:.1f}%"
            )
            if found:
                mlat = pkt.get("mine_lat")
                mlng = pkt.get("mine_lon")
                if mlat and mlng:
                    # FIX: was  mine = await STATE.add_mine(...)
                    mine = STATE.add_mine(float(mlat), float(mlng), detected_by=drone_id)
                    asyncio.create_task(_post_to_ground({
                        "type":     "mine_detected",
                        "lat":      mlat,
                        "lng":      mlng,
                        "drone_id": drone_id,
                    }))
                    log.info(
                        f"[MINE TCP] sector_result mine #{mine.mine_id} added "
                        f"({mlat:.6f},{mlng:.6f})"
                    )
        else:
            log.warning(f"[MINE TCP] Unknown packet type '{pkt_type}' — ignored")

    except asyncio.IncompleteReadError:
        log.warning(f"[MINE TCP] {peer} closed before full packet arrived")
    except asyncio.TimeoutError:
        log.warning(f"[MINE TCP] Timeout reading from {peer}")
    except json.JSONDecodeError as e:
        log.error(f"[MINE TCP] JSON parse error from {peer}: {e}")
    except Exception as e:
        log.error(f"[MINE TCP] Unexpected error from {peer}: {e}")
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def _mine_tcp_server() -> None:
    server = await asyncio.start_server(
        _handle_mine_connection, "0.0.0.0", MINE_TCP_PORT,
    )
    addrs = [str(s.getsockname()) for s in server.sockets]
    log.info(f"[MINE TCP] Server listening on {addrs}")
    async with server:
        await server.serve_forever()


# ════════════════════════════════════════════════════════════════════════════
#  MASTER-SIDE ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════
async def start_udp_server() -> None:
    """
    Start the UDP telemetry server AND mine TCP listener on the master.

    Example (master main):
        await asyncio.gather(
            udp_telemetry.start_udp_server(),
            tcp_commander.start_tcp_commander(),
        )
    """
    loop = asyncio.get_running_loop()

    transport, _ = await loop.create_datagram_endpoint(
        TelemetryProtocol,
        local_addr=(UDP_HOST, UDP_PORT),
    )
    log.info(f"[UDP] Telemetry server ready on {UDP_HOST}:{UDP_PORT}")

    asyncio.create_task(_watchdog(),         name="watchdog")
    asyncio.create_task(_mine_tcp_server(),  name="mine_tcp_server")
    log.info(f"[MINE TCP] Listener starting on port {MINE_TCP_PORT}")

    try:
        await asyncio.Future()
    finally:
        transport.close()
        if _http_session and not _http_session.closed:
            await _http_session.close()
