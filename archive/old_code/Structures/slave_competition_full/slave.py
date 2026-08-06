"""
slave.py  —  S.A.F.E. Slave Drone Startup Script
=================================================
Runs on each slave Pi (slave_1, slave_2, slave_3).
Does NOT run on the master Pi.

What it does
------------
1. Connects to the flight controller via MAVSDK (serial or TCP).
2. Starts a TCPCommandServer on port 14560 — master connects here.
3. Registers handlers for: ARM, TAKEOFF, GOTO, HOLD, PAUSE, RESUME, LAND, STATUS.
4. Broadcasts telemetry (position, altitude, heading, sensor) to master via UDP
   at 5 Hz using an inline UDPSender (no swarm_state dependency).
5. Falls back to StubFC if the FC connection fails — TCP server still starts
   so the master can reach this slave in tests.

Serial ports (SpeedyBee + ArduPilot):
  slave_1, slave_3, master : /dev/ttyACM0  @ 115200  (USB CDC-ACM)
  slave_2                  : /dev/ttyAMA0  @ 115200  (UART direct)
  Note: ttyAMA0 requires Bluetooth disabled in /boot/config.txt:
        dtoverlay=disable-bt
        Then: sudo raspi-config nonint do_serial_hw 0
              sudo raspi-config nonint do_serial_cons 1
              reboot

Run commands:
  # slave_1 / slave_3 (USB)
  python3 slave.py --id slave_1 --master 10.42.0.1 --mavsdk serial:///dev/ttyACM0:115200

  # slave_2 (UART direct)
  python3 slave.py --id slave_2 --master 10.42.0.1 --mavsdk serial:///dev/ttyAMA0:115200

  # Via mavproxy (if mavproxy is bridging the FC on port 5760):
  python3 slave.py --id slave_1 --master 10.42.0.1 --mavsdk tcp://:5760

  # Stub mode (no FC — for testing TCP channel only)
  python3 slave.py --id slave_1 --master 10.42.0.1 --stub

IMPORTANT — does NOT import swarm_state or udp_channel.
  Both of those modules import STATE = SwarmState() at module level.
  On Python 3.12 this crashes because asyncio.Lock() is created outside
  an event loop.  slave.py uses an inline UDPSender (30 lines below)
  with zero external dependencies beyond the stdlib socket module.

AGL → AMSL altitude conversion:
  MAVSDK goto_location(lat, lng, alt, yaw) requires AMSL absolute altitude.
  RealFC reads both pos.relative_altitude_m (AGL) and pos.absolute_altitude_m
  (AMSL) from telemetry and computes:
      ground_amsl   = alt_amsl - alt_agl
      target_amsl   = ground_amsl + desired_agl
"""

import argparse
import asyncio
import json
import logging
import math
import socket
import sys
import threading
import time

# ── tcp_channel is safe to import on slave ──────────────────────────────────
# tcp_channel.py no longer imports swarm_state (Bug #6 fixed), so it won't
# trigger the asyncio.Lock crash on this Pi.
from tcp_channel import TCPCommandServer

log = logging.getLogger("slave")
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ════════════════════════════════════════════════════════════════════════════
CMD_PORT     = 14560     # TCPCommandServer listens here
UDP_PORT     = 14550     # master listens for telemetry here
TELEM_HZ     = 5         # telemetry broadcast rate


# ════════════════════════════════════════════════════════════════════════════
#  INLINE UDP SENDER  — no swarm_state dependency
# ════════════════════════════════════════════════════════════════════════════
class UDPSender:
    """
    Minimal standalone UDP telemetry broadcaster.
    Intentionally does NOT import swarm_state or udp_channel to avoid the
    asyncio.Lock crash on Python 3.12.
    """
    def __init__(self, drone_id: str, master_ip: str, master_port: int = UDP_PORT):
        self.drone_id = drone_id
        self._seq     = 0
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.connect((master_ip, master_port))
        log.info(f"[{drone_id}] UDPSender → {master_ip}:{master_port}")

    def send(self, telemetry: dict) -> bool:
        self._seq += 1
        packet = {"drone_id": self.drone_id, "seq": self._seq, **telemetry}
        try:
            self._sock.send(json.dumps(packet).encode("utf-8"))
            return True
        except OSError as e:
            log.warning(f"UDP send error: {e}")
            return False

    def close(self):
        self._sock.close()


# ════════════════════════════════════════════════════════════════════════════
#  STUB FC  — used when --stub flag is passed or FC connection fails
# ════════════════════════════════════════════════════════════════════════════
class StubFC:
    """
    Simulated flight controller.  Accepts all commands, logs them,
    updates local state so telemetry looks plausible in stub mode.
    """
    def __init__(self, drone_id: str):
        self.drone_id  = drone_id
        self.lat       = 23.077953
        self.lng       = 72.495347
        self.alt_agl   = 0.0
        self.alt_amsl  = 0.0
        self.heading   = 0.0
        self.speed     = 0.0
        self.armed     = False
        self.airborne  = False
        self.bat_pct   = 95.0
        log.info(f"[{drone_id}] StubFC ready")

    def arm(self):
        self.armed = True
        log.info(f"[STUB] ARM")
        return True

    def takeoff(self, alt_agl: float):
        self.airborne = True
        self.alt_agl  = alt_agl
        self.alt_amsl = alt_agl   # stub: treat AMSL ≈ AGL
        log.info(f"[STUB] TAKEOFF → {alt_agl}m AGL")
        return True

    def goto(self, lat: float, lng: float, alt_agl: float):
        self.lat     = lat
        self.lng     = lng
        self.alt_agl = alt_agl
        log.info(f"[STUB] GOTO ({lat:.6f}, {lng:.6f}) @ {alt_agl}m AGL")
        return True

    def hold(self):
        self.speed = 0.0
        log.info(f"[STUB] HOLD")
        return True

    def land(self):
        self.airborne = False
        self.alt_agl  = 0.0
        self.armed    = False
        log.info(f"[STUB] LAND")
        return True

    def get_telemetry(self) -> dict:
        return {
            "lat":      self.lat,
            "lng":      self.lng,
            "altitude": self.alt_agl,
            "alt_amsl": self.alt_amsl,
            "heading":  self.heading,
            "speed":    self.speed,
            "armed":    self.armed,
            "airborne": self.airborne,
            "bat_pct":  self.bat_pct,
            "sensor":   read_sensor(),
        }


# ════════════════════════════════════════════════════════════════════════════
#  REAL FC  — wraps MAVSDK-Python
# ════════════════════════════════════════════════════════════════════════════
class RealFC:
    """
    Wraps MAVSDK-Python for ArduPilot (SpeedyBee).

    Key design decisions:
    • Uses asyncio internally; command handlers call asyncio.run_coroutine_
      threadsafe() to bridge from the TCPCommandServer's sync thread into
      the MAVSDK event loop running in a background thread.
    • AGL → AMSL: goto_location() requires AMSL absolute altitude.
      We read both pos.relative_altitude_m (AGL) and pos.absolute_altitude_m
      (AMSL) from live telemetry to compute ground_amsl, then add desired AGL.
    """

    def __init__(self, drone_id: str, mavsdk_address: str):
        self.drone_id       = drone_id
        self.mavsdk_address = mavsdk_address

        # Shared telemetry state — written by MAVSDK loop, read by telem thread
        self._lock     = threading.Lock()
        self._lat      = 0.0
        self._lng      = 0.0
        self._alt_agl  = 0.0
        self._alt_amsl = 0.0
        self._heading  = 0.0
        self._speed    = 0.0
        self._armed    = False
        self._airborne = False
        self._bat_pct  = 0.0

        self._loop   = None   # asyncio loop running in background thread
        self._drone  = None   # mavsdk.System instance
        self._ready  = threading.Event()

        # Start MAVSDK event loop in background thread
        t = threading.Thread(target=self._run_loop, name=f"MAVSDK-{drone_id}", daemon=True)
        t.start()
        if not self._ready.wait(timeout=30.0):
            raise RuntimeError(f"[{drone_id}] MAVSDK connect timeout on {mavsdk_address}")

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connect_and_stream())

    async def _connect_and_stream(self):
        from mavsdk import System
        self._drone = System()
        log.info(f"[{self.drone_id}] Connecting MAVSDK → {self.mavsdk_address}")
        await self._drone.connect(system_address=self.mavsdk_address)

        # Wait for GPS fix
        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                log.info(f"[{self.drone_id}] GPS fix OK")
                break

        self._ready.set()

        # Stream telemetry into shared state
        await asyncio.gather(
            self._stream_position(),
            self._stream_attitude(),
            self._stream_battery(),
            self._stream_armed(),
            self._stream_in_air(),
        )

    async def _stream_position(self):
        async for pos in self._drone.telemetry.position():
            with self._lock:
                self._lat      = pos.latitude_deg
                self._lng      = pos.longitude_deg
                self._alt_agl  = pos.relative_altitude_m
                self._alt_amsl = pos.absolute_altitude_m

    async def _stream_attitude(self):
        async for att in self._drone.telemetry.attitude_euler():
            with self._lock:
                self._heading = att.yaw_deg % 360

    async def _stream_battery(self):
        async for bat in self._drone.telemetry.battery():
            with self._lock:
                self._bat_pct = bat.remaining_percent * 100

    async def _stream_armed(self):
        async for armed in self._drone.telemetry.armed():
            with self._lock:
                self._armed = armed

    async def _stream_in_air(self):
        async for in_air in self._drone.telemetry.in_air():
            with self._lock:
                self._airborne = in_air

    def _run_async(self, coro):
        """Bridge: run a coroutine on the MAVSDK loop from a sync thread."""
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=10.0)

    def arm(self):
        async def _arm():
            await self._drone.action.set_takeoff_altitude(3.0)
            await self._drone.action.arm()
        self._run_async(_arm())
        return True

    def takeoff(self, alt_agl: float):
        async def _takeoff():
            await self._drone.action.set_takeoff_altitude(alt_agl)
            await self._drone.action.takeoff()
        self._run_async(_takeoff())
        return True

    def goto(self, lat: float, lng: float, alt_agl: float):
        """
        AGL → AMSL conversion:
          ground_amsl = alt_amsl - alt_agl   (from live telemetry)
          target_amsl = ground_amsl + desired_agl
        goto_location() takes AMSL absolute altitude.
        """
        async def _goto():
            with self._lock:
                cur_agl  = self._alt_agl
                cur_amsl = self._alt_amsl
            ground_amsl  = cur_amsl - cur_agl
            target_amsl  = ground_amsl + alt_agl
            await self._drone.action.goto_location(
                lat, lng, target_amsl, float("nan")  # nan = keep heading
            )
        self._run_async(_goto())
        return True

    def hold(self):
        async def _hold():
            await self._drone.action.hold()
        self._run_async(_hold())
        return True

    def land(self):
        async def _land():
            await self._drone.action.land()
        self._run_async(_land())
        return True

    def get_telemetry(self) -> dict:
        with self._lock:
            return {
                "lat":      self._lat,
                "lng":      self._lng,
                "altitude": self._alt_agl,
                "alt_amsl": self._alt_amsl,
                "heading":  self._heading,
                "speed":    self._speed,
                "armed":    self._armed,
                "airborne": self._airborne,
                "bat_pct":  self._bat_pct,
                "sensor":   read_sensor(),
            }


# ════════════════════════════════════════════════════════════════════════════
#  METAL DETECTOR SENSOR READ
# ════════════════════════════════════════════════════════════════════════════
def read_sensor() -> float:
    """
    Read the metal detector GPIO pin.
    TODO: replace with real GPIO read.
    e.g.:  import RPi.GPIO as GPIO
           GPIO.setmode(GPIO.BCM)
           GPIO.setup(17, GPIO.IN)
           return 1.0 if GPIO.input(17) else 0.0
    """
    return 0.0


# ════════════════════════════════════════════════════════════════════════════
#  COMMAND HANDLERS
# ════════════════════════════════════════════════════════════════════════════
def register_handlers(server: TCPCommandServer, fc, drone_id: str):
    """Register all command handlers on the TCPCommandServer."""

    @server.on_command("ARM")
    def handle_arm(cmd):
        try:
            fc.arm()
            return {"status": "ok"}
        except Exception as e:
            log.error(f"ARM failed: {e}")
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("TAKEOFF")
    def handle_takeoff(cmd):
        alt = float(cmd.get("alt", 3.0))
        try:
            fc.takeoff(alt)
            return {"status": "ok"}
        except Exception as e:
            log.error(f"TAKEOFF failed: {e}")
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("GOTO")
    def handle_goto(cmd):
        lat = float(cmd["lat"])
        lng = float(cmd["lng"])
        alt = float(cmd.get("alt", 3.0))
        try:
            fc.goto(lat, lng, alt)
            return {"status": "ok"}
        except Exception as e:
            log.error(f"GOTO failed: {e}")
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("HOLD")
    def handle_hold(cmd):
        try:
            fc.hold()
            return {"status": "ok"}
        except Exception as e:
            log.error(f"HOLD failed: {e}")
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("PAUSE")
    def handle_pause(cmd):
        # PAUSE = HOLD for ArduPilot GUIDED mode
        try:
            fc.hold()
            return {"status": "ok"}
        except Exception as e:
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("RESUME")
    def handle_resume(cmd):
        # RESUME: master will send a new GOTO after this
        log.info(f"[{drone_id}] RESUME — awaiting next GOTO from master")
        return {"status": "ok"}

    @server.on_command("LAND")
    def handle_land(cmd):
        try:
            fc.land()
            return {"status": "ok"}
        except Exception as e:
            log.error(f"LAND failed: {e}")
            return {"status": "rejected", "reason": str(e)}

    @server.on_command("STATUS")
    def handle_status(cmd):
        """Ping handler — master uses this to verify the socket is alive."""
        telem = fc.get_telemetry()
        return {
            "status":   "ok",
            "drone_id": drone_id,
            "armed":    telem["armed"],
            "airborne": telem["airborne"],
            "alt":      telem["altitude"],
        }


# ════════════════════════════════════════════════════════════════════════════
#  TELEMETRY BROADCAST LOOP
# ════════════════════════════════════════════════════════════════════════════
def telemetry_loop(fc, sender: UDPSender, hz: float = TELEM_HZ):
    """
    Runs in a background daemon thread.
    Reads telemetry from the FC and broadcasts to master at `hz` Hz.
    """
    interval = 1.0 / hz
    log.info(f"Telemetry loop started @ {hz} Hz")
    while True:
        t0 = time.monotonic()
        try:
            telem = fc.get_telemetry()
            sender.send(telem)
        except Exception as e:
            log.warning(f"Telemetry read error: {e}")
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="S.A.F.E. Slave Drone")
    parser.add_argument("--id",     required=True,
                        help="Drone ID e.g. slave_1")
    parser.add_argument("--master", required=True,
                        help="Master Pi IP e.g. 10.42.0.1")
    parser.add_argument("--mavsdk", default=None,
                        help="MAVSDK address e.g. serial:///dev/ttyACM0:115200"
                             " or tcp://:5760")
    parser.add_argument("--stub",   action="store_true",
                        help="Use StubFC (no real flight controller)")
    parser.add_argument("--port",   type=int, default=CMD_PORT,
                        help=f"TCP command port (default {CMD_PORT})")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    drone_id = args.id

    # ── Build FC ──────────────────────────────────────────────────────────
    fc = None
    if args.stub:
        log.info(f"[{drone_id}] Stub mode — no real FC")
        fc = StubFC(drone_id)
    else:
        if not args.mavsdk:
            log.error("--mavsdk address required unless --stub is set")
            sys.exit(1)
        try:
            log.info(f"[{drone_id}] Connecting to FC: {args.mavsdk}")
            fc = RealFC(drone_id, args.mavsdk)
            log.info(f"[{drone_id}] FC connected ✓")
        except Exception as e:
            log.warning(f"[{drone_id}] FC connection failed: {e}")
            log.warning(f"[{drone_id}] Falling back to StubFC — TCP server will still start")
            fc = StubFC(drone_id)

    # ── UDP telemetry sender ──────────────────────────────────────────────
    sender = UDPSender(drone_id, master_ip=args.master)

    # ── Start telemetry thread ────────────────────────────────────────────
    t = threading.Thread(
        target  = telemetry_loop,
        args    = (fc, sender),
        name    = f"TelemetryLoop-{drone_id}",
        daemon  = True,
    )
    t.start()

    # ── Build and start TCP command server ────────────────────────────────
    server = TCPCommandServer(drone_id=drone_id, port=args.port)
    register_handlers(server, fc, drone_id)

    log.info(f"[{drone_id}] Ready — listening for master on port {args.port}")
    try:
        server.start(blocking=True)   # blocks forever
    except KeyboardInterrupt:
        log.info(f"[{drone_id}] Shutting down")
        try:
            fc.land()
        except Exception:
            pass
        sender.close()


if __name__ == "__main__":
    main()
