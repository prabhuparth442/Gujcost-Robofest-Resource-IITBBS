"""
udp_channel.py  —  Continuous Telemetry Channel (UDP)
======================================================
Mirrors the DroneTunnel pattern but purpose-built for high-frequency,
continuous, text-only telemetry from slave drones to the master.

DESIGN PHILOSOPHY
-----------------
Like DroneTunnel, this file exposes two usable classes:

  UDPSender    — runs on each SLAVE drone.
                 Call sender.send(payload_dict) in a loop.
                 That's the entire slave-side API.

  UDPReceiver  — runs on the MASTER drone.
                 Instantiate once, call receiver.start().
                 Every valid packet is automatically written into
                 SwarmState via STATE.update_drone() or STATE.add_mine().
                 No further plumbing needed.

WHY ONE PORT FOR ALL SLAVES
----------------------------
A single UDPReceiver binds to one port.  All slaves send to that same
port.  The drone_id field inside each packet demuxes which slave it came
from.  Adding a 4th slave costs zero extra sockets on the master.

STALE-PACKET REJECTION
-----------------------
Every packet carries a monotonically increasing `seq` number.
The receiver tracks the last seen seq per drone and silently discards
anything that arrives out-of-order (common on lossy Wi-Fi).
This means SwarmState always holds the FRESHEST data, never old data.

PACKET SCHEMA  (slave → master, JSON text, every ~200 ms)
----------------------------------------------------------
{
    "drone_id" : "slave_1",     # identifies the sender
    "seq"      : 1042,          # monotonic counter — stale-drop guard
    "lat"      : 12.971599,
    "lng"      : 77.594563,
    "altitude" : 3.2,           # metres AGL
    "heading"  : 47.3,          # degrees 0-360
    "speed"    : 2.1,           # m/s ground speed
    "armed"    : true,
    "airborne" : true,
    "bat_pct"  : 82,            # battery %
    "sensor"   : 0.04           # metal-detector reading 0.0-1.0
}

No images, no binary blobs — text/JSON only, exactly as requested.
"""

import json
import logging
import math
import socket
import threading
import time
from typing import Callable, Optional

from swarm_state import STATE, DRONE_IDS

log = logging.getLogger("udp_channel")

# ─────────────────────────────────────────────────────────────────────────────
#  CONSTANTS  (change these to match your network)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_MASTER_IP   = "10.42.0.1"   # master's IP — slaves send here
DEFAULT_UDP_PORT    = 14550            # single shared port on master
SEND_HZ             = 5               # slave broadcast rate
SENSOR_THRESH       = 0.65            # metal-detector trigger level
MINE_PERSIST_COUNT  = 5               # consecutive hits → confirmed mine
STALE_TIMEOUT_S     = 2.0             # drone silent for this long → stale flag


# ═════════════════════════════════════════════════════════════════════════════
#  UDPSender  —  runs on each SLAVE
# ═════════════════════════════════════════════════════════════════════════════
class UDPSender:
    """
    Slave-side UDP broadcaster.

    Usage (on each slave Pi):

        from udp_channel import UDPSender

        sender = UDPSender(
            drone_id   = "slave_1",
            master_ip  = "10.42.0.1",
            master_port= 14550,
        )

        # In your telemetry loop (e.g. alongside MAVSDK reads):
        while flying:
            sender.send({
                "lat": pos.latitude_deg,
                "lng": pos.longitude_deg,
                "altitude": pos.absolute_altitude_m,
                "heading":  heading.heading_deg,
                "speed":    ground_speed,
                "armed":    is_armed,
                "airborne": is_in_air,
                "bat_pct":  battery.remaining_percent * 100,
                "sensor":   read_metal_detector(),
            })
            time.sleep(1 / SEND_HZ)

    The sender automatically stamps drone_id and seq onto every packet —
    you never manage those manually.
    """

    def __init__(
        self,
        drone_id:    str,
        master_ip:   str  = DEFAULT_MASTER_IP,
        master_port: int  = DEFAULT_UDP_PORT,
    ):
        if drone_id not in DRONE_IDS:
            raise ValueError(f"drone_id must be one of {DRONE_IDS}, got '{drone_id}'")

        self.drone_id    = drone_id
        self.master_ip   = master_ip
        self.master_port = master_port

        self._seq    = 0
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # UDP — no connect() needed, but calling connect() here lets the OS
        # pick the best source interface automatically.
        self._sock.connect((master_ip, master_port))

        log.info(f"[{drone_id}] UDPSender ready → {master_ip}:{master_port}")

    def send(self, telemetry: dict) -> bool:
        """
        Stamp and broadcast one telemetry packet.

        telemetry dict keys:  lat, lng, altitude, heading, speed,
                              armed, airborne, bat_pct, sensor
        Returns True on success, False on socket error.
        The call is NON-BLOCKING — UDP sendto returns immediately.
        """
        self._seq += 1
        packet = {
            "drone_id": self.drone_id,
            "seq":      self._seq,
            **telemetry,                 # spread caller's fields in
        }

        try:
            raw = json.dumps(packet).encode("utf-8")
            self._sock.send(raw)
            return True
        except OSError as e:
            log.warning(f"[{self.drone_id}] UDP send failed: {e}")
            return False

    def close(self):
        self._sock.close()
        log.info(f"[{self.drone_id}] UDPSender closed")


# ═════════════════════════════════════════════════════════════════════════════
#  _MinePersistenceTracker  —  internal helper, not exported
# ═════════════════════════════════════════════════════════════════════════════
class _MinePersistenceTracker:
    """
    Counts consecutive above-threshold sensor readings per drone.
    Fires exactly ONCE per detection event then resets, so the same
    mine isn't added to STATE multiple times.
    """

    def __init__(self, threshold: float = SENSOR_THRESH, persist: int = MINE_PERSIST_COUNT):
        self._threshold = threshold
        self._persist   = persist
        self._counts: dict[str, int] = {}

    def feed(self, drone_id: str, sensor_val: float) -> bool:
        """
        Returns True exactly once when a mine is confirmed.
        After returning True the counter resets for that drone.
        """
        if sensor_val >= self._threshold:
            self._counts[drone_id] = self._counts.get(drone_id, 0) + 1
        else:
            self._counts[drone_id] = 0   # reading dropped — reset streak

        if self._counts.get(drone_id, 0) >= self._persist:
            self._counts[drone_id] = 0   # reset so same spot isn't double-counted
            return True
        return False


# ═════════════════════════════════════════════════════════════════════════════
#  UDPReceiver  —  runs on the MASTER
# ═════════════════════════════════════════════════════════════════════════════
class UDPReceiver:
    """
    Master-side UDP listener.  One instance handles ALL slaves.

    Usage (on the master Pi):

        from udp_channel import UDPReceiver

        receiver = UDPReceiver(port=14550)
        receiver.start()          # launches background thread, non-blocking
        # ... rest of master logic continues ...

    Every incoming packet is:
      1. Validated (drone_id known, seq not stale).
      2. Written into SwarmState via STATE.update_drone().
      3. Checked for mine persistence — confirmed mines go into STATE too.
      4. Optionally passed to an on_packet callback you can provide.

    You can also register a callback for mine events:
        receiver.on_mine = lambda mine_record: forward_to_ground_server(mine_record)
    """

    def __init__(
        self,
        port:     int = DEFAULT_UDP_PORT,
        host:     str = "0.0.0.0",
        on_packet: Optional[Callable[[dict], None]] = None,
        on_mine:   Optional[Callable]               = None,
    ):
        self.port      = port
        self.host      = host
        self.on_packet = on_packet   # called after every valid packet
        self.on_mine   = on_mine     # called when a mine is confirmed

        self._mine_tracker = _MinePersistenceTracker()
        self._last_seq: dict[str, int] = {}   # stale-drop state per drone
        self._running  = False
        self._thread:  Optional[threading.Thread] = None
        self._sock:    Optional[socket.socket]    = None

    # ── public API ────────────────────────────────────────────────────────

    def start(self) -> None:
        """
        Start listening in a background daemon thread.
        Returns immediately — listening happens in the background.
        Call stop() to shut down cleanly.
        """
        if self._running:
            log.warning("UDPReceiver already running")
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(1.0)   # allows clean shutdown check

        self._running = True
        self._thread  = threading.Thread(
            target  = self._listen_loop,
            name    = "UDPReceiver",
            daemon  = True,           # dies automatically when master exits
        )
        self._thread.start()
        log.info(f"UDPReceiver listening on {self.host}:{self.port}")

    def stop(self) -> None:
        """Signal the background thread to stop and close the socket."""
        self._running = False
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=3.0)
        log.info("UDPReceiver stopped")

    # ── internals ─────────────────────────────────────────────────────────

    def _listen_loop(self) -> None:
        """
        Blocking receive loop — runs in the background thread.

        Threading safety note:
        STATE.update_drone() and STATE.add_mine() both acquire
        STATE.lock internally (it's a threading.Lock in the sync version).
        So even if the TCP commander thread also touches STATE at the same
        time, there are no races.
        """
        while self._running:
            try:
                raw, addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue        # just check _running flag and loop
            except OSError:
                break           # socket was closed by stop()

            try:
                packet = json.loads(raw.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                log.warning(f"Bad UDP packet from {addr}: {e}")
                continue

            self._handle_packet(packet, addr)

    def _handle_packet(self, packet: dict, addr: tuple) -> None:
        """
        Validate → stale-drop → update STATE → mine check → callback.
        This is called serially in the single receiver thread, so no
        internal locking is needed here beyond what STATE methods do.
        """
        # ── 1. Validate required fields ───────────────────────────────────
        drone_id = packet.get("drone_id")
        if drone_id not in DRONE_IDS:
            log.warning(f"Unknown drone_id '{drone_id}' from {addr} — dropped")
            return

        # ── 2. Stale-packet rejection ─────────────────────────────────────
        seq      = int(packet.get("seq", 0))
        last_seq = self._last_seq.get(drone_id, -1)
        if seq <= last_seq:
            # Out-of-order or duplicate — silently discard
            return
        self._last_seq[drone_id] = seq

        # ── 3. Update shared SwarmState ───────────────────────────────────
        #    STATE.update_drone is the ONLY place we write drone positions.
        #    SwarmState is the single source of truth — nothing else stores
        #    a copy of position data.
        STATE.update_drone(packet)

        log.debug(
            f"[UDP] {drone_id} seq={seq} "
            f"lat={packet.get('lat',0):.5f} lng={packet.get('lng',0):.5f} "
            f"alt={packet.get('altitude',0):.1f}m "
            f"sensor={packet.get('sensor',0):.2f}"
        )

        # ── 4. Mine persistence check ─────────────────────────────────────
        sensor   = float(packet.get("sensor", 0.0))
        lat      = float(packet.get("lat", 0.0))
        lng      = float(packet.get("lng", 0.0))

        if self._mine_tracker.feed(drone_id, sensor):
            mine = STATE.add_mine(lat, lng, detected_by=drone_id)
            log.info(
                f"[MINE CONFIRMED] #{mine.mine_id} by {drone_id} "
                f"at ({mine.x:.1f}m, {mine.y:.1f}m)"
            )
            # Fire optional mine callback (e.g. forward to Flask ground server)
            if self.on_mine:
                try:
                    self.on_mine(mine)
                except Exception as e:
                    log.error(f"on_mine callback error: {e}")

        # ── 5. Fire optional general packet callback ──────────────────────
        if self.on_packet:
            try:
                self.on_packet(packet)
            except Exception as e:
                log.error(f"on_packet callback error: {e}")


# ═════════════════════════════════════════════════════════════════════════════
#  STALE DRONE WATCHDOG  —  optional, call start_watchdog() on master
# ═════════════════════════════════════════════════════════════════════════════
def start_watchdog(
    on_stale: Optional[Callable[[str], None]] = None,
    check_hz: float = 1.0,
) -> threading.Thread:
    """
    Starts a background thread that checks every (1/check_hz) seconds for
    drones that have gone silent (no packet for > STALE_TIMEOUT_S).

    on_stale(drone_id) is called when a drone first goes stale.
    Typical use: push an emergency LAND onto that drone's command queue.

    Returns the thread so you can join() it if needed.

    Example:
        def handle_stale(drone_id):
            tcp_channel.enqueue(drone_id, {"cmd": "LAND", "reason": "stale"})

        start_watchdog(on_stale=handle_stale)
    """
    already_flagged: set[str] = set()

    def _loop():
        while True:
            time.sleep(1.0 / check_hz)
            snap = STATE.snapshot()
            for did, d in snap["drones"].items():
                if did == "master":
                    continue
                if d["stale"] and d["airborne"] and did not in already_flagged:
                    already_flagged.add(did)
                    log.warning(f"[WATCHDOG] {did} is STALE and AIRBORNE")
                    if on_stale:
                        try:
                            on_stale(did)
                        except Exception as e:
                            log.error(f"on_stale callback error: {e}")
                elif not d["stale"] and did in already_flagged:
                    already_flagged.discard(did)   # drone came back online

    t = threading.Thread(target=_loop, name="UDPWatchdog", daemon=True)
    t.start()
    return t
