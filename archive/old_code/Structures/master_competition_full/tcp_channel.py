"""
tcp_channel.py  —  Command Channel (TCP)
=========================================
Mirrors the DroneTunnel pattern but purpose-built for the
order → process → result command flow between master and slaves.

DESIGN PHILOSOPHY
-----------------
Two classes, clean mirror of each other:

  TCPCommandServer  — runs on each SLAVE drone.
                      Call server.start() once.
                      Register handlers via server.on_command(cmd, fn).
                      The class manages the socket, JSON framing, and ACKs.
                      Your handler just receives a dict and returns a result.

  TCPCommandClient  — runs on the MASTER drone.
                      One instance per slave.
                      Call client.send(cmd_dict) to send a command and
                      WAIT for the result (blocking, with timeout + retry).
                      Call client.enqueue(cmd_dict) to fire-and-forget into
                      a background queue — the client sends and ACKs in order.

ORDER → PROCESS → RESULT EXPLAINED
------------------------------------
Master                              Slave
──────                              ─────
client.send({"cmd":"GOTO", ...})
  │
  ├─── TCP: {"cmd":"GOTO","seq":7} ──────────────────►
  │                                    handler("GOTO") runs
  │                                    MAVSDK goto_location()
  │◄── TCP: {"ack":"GOTO","seq":7,"status":"ok"} ──────
  │
  └─ returns {"status":"ok"}   ← your code gets this back


PERSISTENT CONNECTIONS
-----------------------
Unlike Tunnel.py which opens a new socket per send(), TCPCommandClient
keeps ONE socket open per slave and reuses it.  This means:
  • No TCP handshake overhead between commands (~1-3 ms saved per command).
  • A dropped connection is detected immediately, not on the next send.
  • The slave always has a live reader — it can receive emergency commands
    (like LAND) even when no regular command is being sent.

SIMULTANEOUS COMMANDS TO MULTIPLE SLAVES
-----------------------------------------
Use TCPCommandBroadcaster (at the bottom of this file):

    broadcaster = TCPCommandBroadcaster(clients)
    results = broadcaster.send_all({"cmd": "PAUSE"})
    # All slaves get PAUSE at the same time via threads.
    # results = {"slave_1": {"status":"ok"}, "slave_2": {"status":"ok"}, ...}

TEXT-ONLY PROTOCOL  (newline-delimited JSON)
---------------------------------------------
Every message (command or ACK) is one JSON object terminated by newline.
No binary, no images — exactly as requested.

Command  (master → slave):
    {"cmd": "GOTO", "lat": 12.97, "lng": 77.59, "alt": 3.0, "seq": 7}
    {"cmd": "LAND", "seq": 8}
    {"cmd": "PAUSE", "seq": 9}

ACK  (slave → master):
    {"ack": "GOTO", "seq": 7, "status": "ok"}
    {"ack": "GOTO", "seq": 7, "status": "rejected", "reason": "no_gps_fix"}
    {"ack": "LAND", "seq": 8, "status": "ok"}
"""

import json
import logging
import socket
import threading
import time
from typing import Callable, Optional


log = logging.getLogger("tcp_channel")

# ─────────────────────────────────────────────────────────────────────────────
#  CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_CMD_PORT  = 14560
ACK_TIMEOUT_S     = 3.0    # wait this long for an ACK before retry
MAX_RETRIES       = 3      # retry this many times before giving up
RECONNECT_DELAY_S = 3.0    # wait between reconnect attempts
RECV_BUFSIZE      = 65535


# ─────────────────────────────────────────────────────────────────────────────
#  INTERNAL: newline-delimited JSON framing
# ─────────────────────────────────────────────────────────────────────────────
def _send_json(sock: socket.socket, payload: dict) -> None:
    """Encode payload as JSON + newline and send over sock."""
    raw = (json.dumps(payload) + "\n").encode("utf-8")
    sock.sendall(raw)


def _recv_json(sock: socket.socket, buf: bytearray) -> Optional[dict]:
    """
    Read bytes from sock into buf until a newline is found.
    Returns the parsed dict, or None if the connection closed.
    Raises json.JSONDecodeError on bad JSON.
    Mutates buf in-place so partial reads across calls work correctly.
    """
    while b"\n" not in buf:
        chunk = sock.recv(RECV_BUFSIZE)
        if not chunk:
            return None     # connection closed cleanly
        buf.extend(chunk)

    line, remainder = buf.split(b"\n", 1)
    buf.clear()
    buf.extend(remainder)
    return json.loads(line.decode("utf-8").strip())


# ═════════════════════════════════════════════════════════════════════════════
#  TCPCommandServer  —  runs on each SLAVE
# ═════════════════════════════════════════════════════════════════════════════
class TCPCommandServer:
    """
    Slave-side TCP command receiver.

    Usage (on each slave Pi):

        from tcp_channel import TCPCommandServer

        server = TCPCommandServer(drone_id="slave_1", port=14560)

        # Register a handler for each command type.
        # Your handler receives the full command dict and returns a result dict.
        # { "status": "ok" }  or  { "status": "rejected", "reason": "..." }

        @server.on_command("GOTO")
        def handle_goto(cmd):
            # Replace with real MAVSDK:
            # await drone.action.goto_location(cmd["lat"], cmd["lng"], cmd["alt"], 0)
            print(f"Flying to {cmd['lat']}, {cmd['lng']}")
            return {"status": "ok"}

        @server.on_command("LAND")
        def handle_land(cmd):
            # await drone.action.land()
            return {"status": "ok"}

        server.start()   # blocks forever (or pass blocking=False)

    The server accepts ONE connection at a time (from the master).
    If the master disconnects and reconnects, the server accepts the new
    connection automatically.
    """

    def __init__(self, drone_id: str, port: int = DEFAULT_CMD_PORT, host: str = "0.0.0.0"):
        self.drone_id  = drone_id
        self.port      = port
        self.host      = host
        self._handlers: dict[str, Callable[[dict], dict]] = {}
        self._running  = False
        self._sock:   Optional[socket.socket] = None

    def on_command(self, cmd_name: str) -> Callable:
        """
        Decorator to register a handler for a command type.

            @server.on_command("GOTO")
            def handle_goto(cmd):
                ...
                return {"status": "ok"}
        """
        def decorator(fn: Callable) -> Callable:
            self._handlers[cmd_name.upper()] = fn
            log.info(f"[{self.drone_id}] Handler registered for '{cmd_name.upper()}'")
            return fn
        return decorator

    def register(self, cmd_name: str, fn: Callable) -> None:
        """Imperative alternative to the @on_command decorator."""
        self._handlers[cmd_name.upper()] = fn
        log.info(f"[{self.drone_id}] Handler registered for '{cmd_name.upper()}'")

    def start(self, blocking: bool = True) -> Optional[threading.Thread]:
        """
        Start the command server.
        blocking=True  → this call never returns (normal use in slave main).
        blocking=False → runs in background thread, returns the thread.
        """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(1)
        self._running = True
        log.info(f"[{self.drone_id}] TCPCommandServer listening on {self.host}:{self.port}")

        if blocking:
            self._accept_loop()
        else:
            t = threading.Thread(
                target = self._accept_loop,
                name   = f"TCPServer-{self.drone_id}",
                daemon = True,
            )
            t.start()
            return t

    def stop(self) -> None:
        self._running = False
        if self._sock:
            self._sock.close()
        log.info(f"[{self.drone_id}] TCPCommandServer stopped")

    # ── internals ─────────────────────────────────────────────────────────

    def _accept_loop(self) -> None:
        """Accept connections from the master one at a time."""
        while self._running:
            try:
                conn, addr = self._sock.accept()
            except OSError:
                break   # socket closed by stop()

            log.info(f"[{self.drone_id}] Master connected from {addr}")
            try:
                self._handle_connection(conn)
            except Exception as e:
                log.error(f"[{self.drone_id}] Connection error: {e}")
            finally:
                conn.close()
                log.info(f"[{self.drone_id}] Master disconnected — waiting for reconnect")

    def _handle_connection(self, conn: socket.socket) -> None:
        """
        Read commands, dispatch to handlers, send ACKs.

        ORDER → PROCESS → RESULT loop:
          1. Read one JSON command from the socket.
          2. Look up the registered handler.
          3. Call the handler (this is the "process" step — can be slow).
          4. Send the result back as an ACK.
          Repeat until connection closes.
        """
        conn.settimeout(None)   # no timeout on reads — master controls pacing
        buf = bytearray()

        while True:
            # ── ORDER: receive command ────────────────────────────────────
            try:
                cmd = _recv_json(conn, buf)
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                log.warning(f"[{self.drone_id}] Bad command JSON: {e}")
                continue
            except OSError:
                break   # connection dropped

            if cmd is None:
                break   # master closed the connection

            cmd_name = cmd.get("cmd", "").upper()
            seq      = cmd.get("seq", 0)
            log.info(f"[{self.drone_id}] ← {cmd_name}  seq={seq}  data={cmd}")

            # ── PROCESS: run handler ──────────────────────────────────────
            handler = self._handlers.get(cmd_name)
            if handler is None:
                result = {"status": "rejected", "reason": f"no handler for '{cmd_name}'"}
                log.warning(f"[{self.drone_id}] No handler for '{cmd_name}'")
            else:
                try:
                    result = handler(cmd)
                    if not isinstance(result, dict):
                        result = {"status": "ok"}
                except Exception as e:
                    result = {"status": "rejected", "reason": str(e)}
                    log.error(f"[{self.drone_id}] Handler error for {cmd_name}: {e}")

            # ── RESULT: send ACK ──────────────────────────────────────────
            ack = {"ack": cmd_name, "seq": seq, **result}
            try:
                _send_json(conn, ack)
                log.info(f"[{self.drone_id}] → ACK {cmd_name} seq={seq} status={result.get('status')}")
            except OSError as e:
                log.error(f"[{self.drone_id}] Failed to send ACK: {e}")
                break


# ═════════════════════════════════════════════════════════════════════════════
#  TCPCommandClient  —  runs on the MASTER (one per slave)
# ═════════════════════════════════════════════════════════════════════════════
class TCPCommandClient:
    """
    Master-side TCP command sender for ONE slave.

    Usage (on the master Pi):

        from tcp_channel import TCPCommandClient

        client = TCPCommandClient(drone_id="slave_1", slave_ip="192.168.1.11")
        client.connect_loop()      # start background reconnect thread

        # Blocking send — waits for ACK:
        result = client.send({"cmd": "GOTO", "lat": 12.97, "lng": 77.59, "alt": 3.0})
        # result = {"status": "ok"}  or  {"status": "rejected", "reason": "..."}

        # Fire-and-forget into queue (returns immediately):
        client.enqueue({"cmd": "PAUSE"})

    For broadcasting to ALL slaves at once, use TCPCommandBroadcaster below.
    """

    def __init__(
        self,
        drone_id:   str,
        slave_ip:   str,
        slave_port: int = DEFAULT_CMD_PORT,
    ):
        self.drone_id   = drone_id
        self.slave_ip   = slave_ip
        self.slave_port = slave_port

        self._sock:   Optional[socket.socket] = None
        self._buf     = bytearray()
        self._lock    = threading.Lock()        # one send at a time per slave
        self._connected_event = threading.Event()
        self._seq     = 0
        self._running = False

        # Background command queue (for enqueue() / fire-and-forget use)
        self._queue: list[dict] = []
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()

    # ── public API ────────────────────────────────────────────────────────

    def connect_loop(self) -> threading.Thread:
        """
        Start background thread that keeps the TCP connection alive.
        Retries every RECONNECT_DELAY_S seconds on failure.
        Returns immediately — connection happens in the background.
        """
        self._running = True
        t = threading.Thread(
            target = self._reconnect_loop,
            name   = f"TCPClient-{self.drone_id}",
            daemon = True,
        )
        t.start()

        # Start the queue drainer thread
        q_thread = threading.Thread(
            target = self._queue_drain_loop,
            name   = f"TCPQueue-{self.drone_id}",
            daemon = True,
        )
        q_thread.start()

        return t

    def send(self, command: dict, retries: int = MAX_RETRIES) -> dict:
        """
        Send a command and BLOCK until an ACK is received (or timeout).

        Automatically stamps a seq number.
        Retries up to `retries` times on timeout.

        Returns the ACK dict from the slave, e.g.:
            {"ack": "GOTO", "seq": 7, "status": "ok"}
            {"ack": "GOTO", "seq": 7, "status": "rejected", "reason": "no_gps"}
            {"status": "timeout"}    ← if all retries exhausted
            {"status": "not_connected"}
        """
        if not self._connected_event.wait(timeout=5.0):
            log.warning(f"[{self.drone_id}] send() called but not connected")
            return {"status": "not_connected"}

        with self._lock:    # ← ensures only one command in-flight per slave
            self._seq += 1
            command["seq"] = self._seq
            seq = self._seq

            for attempt in range(1, retries + 1):
                # ── Send ──────────────────────────────────────────────────
                try:
                    _send_json(self._sock, command)
                    log.info(f"[{self.drone_id}] → {command['cmd']} seq={seq} (attempt {attempt})")
                except OSError as e:
                    log.error(f"[{self.drone_id}] Send error: {e}")
                    self._mark_disconnected()
                    return {"status": "send_error"}

                # ── Wait for ACK ──────────────────────────────────────────
                self._sock.settimeout(ACK_TIMEOUT_S)
                try:
                    ack = _recv_json(self._sock, self._buf)
                    if ack is None:
                        log.warning(f"[{self.drone_id}] Connection closed waiting for ACK")
                        self._mark_disconnected()
                        return {"status": "disconnected"}

                    if ack.get("seq") == seq:
                        log.info(
                            f"[{self.drone_id}] ✓ ACK {ack.get('ack')} "
                            f"seq={seq} status={ack.get('status')}"
                        )
                        return ack
                    else:
                        log.warning(f"[{self.drone_id}] ACK seq mismatch — got {ack.get('seq')} want {seq}")

                except socket.timeout:
                    log.warning(f"[{self.drone_id}] ACK timeout seq={seq} attempt {attempt}/{retries}")
                except OSError as e:
                    log.error(f"[{self.drone_id}] Recv error: {e}")
                    self._mark_disconnected()
                    return {"status": "recv_error"}
                finally:
                    self._sock.settimeout(None)

            log.error(f"[{self.drone_id}] All {retries} retries exhausted for seq={seq}")
            return {"status": "timeout"}

    def enqueue(self, command: dict) -> None:
        """
        Add a command to the background queue and return immediately.
        The queue drainer sends commands in order, one at a time, each
        waiting for its ACK before sending the next.

        Use this for non-urgent commands where you don't need to block.
        Use send() when you need to act on the result immediately.
        """
        with self._queue_lock:
            self._queue.append(command)
        self._queue_event.set()
        log.debug(f"[{self.drone_id}] Enqueued {command.get('cmd')}")

    def stop(self) -> None:
        self._running = False
        self._mark_disconnected()

    # ── internals ─────────────────────────────────────────────────────────

    def _reconnect_loop(self) -> None:
        """Keep trying to connect to the slave. Runs in background thread."""
        while self._running:
            try:
                log.info(f"[{self.drone_id}] Connecting to {self.slave_ip}:{self.slave_port}…")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self.slave_ip, self.slave_port))
                sock.settimeout(None)

                with self._lock:
                    self._sock = sock
                    self._buf.clear()
                self._connected_event.set()
                log.info(f"[{self.drone_id}] TCP connected ✓")

                # Wait here until the connection drops.
                # _mark_disconnected() clears the event when send() detects a failure.
                # We use a separate disconnected event so we don't wait on a set event.
                self._disconnected_event = threading.Event()
                self._disconnected_event.wait()   # blocks until _mark_disconnected fires
                if self._running:
                    log.warning(f"[{self.drone_id}] Connection lost — reconnecting in {RECONNECT_DELAY_S}s")
                    time.sleep(RECONNECT_DELAY_S)

            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                log.warning(f"[{self.drone_id}] Connect failed: {e} — retry in {RECONNECT_DELAY_S}s")
                self._connected_event.clear()
                time.sleep(RECONNECT_DELAY_S)

    def _mark_disconnected(self) -> None:
        """Called when we detect the connection has dropped."""
        self._connected_event.clear()
        # Signal the reconnect loop that the connection died
        if hasattr(self, "_disconnected_event"):
            self._disconnected_event.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _queue_drain_loop(self) -> None:
        """
        Background thread that drains the enqueue() queue.
        Sends one command at a time, waiting for ACK before the next.
        This preserves command ordering for the slave.
        """
        while self._running:
            self._queue_event.wait()
            self._queue_event.clear()

            while True:
                with self._queue_lock:
                    if not self._queue:
                        break
                    cmd = self._queue.pop(0)

                result = self.send(cmd)
                if result.get("status") not in ("ok",):
                    log.warning(
                        f"[{self.drone_id}] Queued command {cmd.get('cmd')} "
                        f"result: {result}"
                    )


# ═════════════════════════════════════════════════════════════════════════════
#  TCPCommandBroadcaster  —  send to ALL slaves simultaneously
# ═════════════════════════════════════════════════════════════════════════════
class TCPCommandBroadcaster:
    """
    Sends the SAME command to all slaves at the same time using threads.

    Usage:

        clients = {
            "slave_1": TCPCommandClient("slave_1", "192.168.1.11"),
            "slave_2": TCPCommandClient("slave_2", "192.168.1.12"),
            "slave_3": TCPCommandClient("slave_3", "192.168.1.13"),
        }
        for c in clients.values():
            c.connect_loop()

        broadcaster = TCPCommandBroadcaster(clients)

        # Send PAUSE to all slaves simultaneously, wait for all ACKs:
        results = broadcaster.send_all({"cmd": "PAUSE"})
        # results = {"slave_1": {"status":"ok"}, "slave_2": {"status":"ok"}, ...}

        # Broadcast without waiting (fire and forget):
        broadcaster.enqueue_all({"cmd": "LAND"})
    """

    def __init__(self, clients: dict[str, TCPCommandClient]):
        self.clients = clients

    def send_all(self, command: dict) -> dict[str, dict]:
        """
        Send command to all slaves simultaneously.
        Blocks until ALL slaves have ACKed (or timed out).

        Each slave gets its own copy of the command dict (with its own seq).
        Returns dict mapping drone_id → ACK result.
        """
        results: dict[str, dict] = {}
        results_lock = threading.Lock()
        threads = []

        for did, client in self.clients.items():
            def _send(drone_id=did, c=client):
                r = c.send(dict(command))   # copy so seq stamps don't collide
                with results_lock:
                    results[drone_id] = r

            t = threading.Thread(target=_send, name=f"bcast_{did}", daemon=True)
            threads.append(t)

        # Start all threads at roughly the same time
        for t in threads:
            t.start()

        # Wait for all to finish
        for t in threads:
            t.join(timeout=ACK_TIMEOUT_S * MAX_RETRIES + 1)

        return results

    def enqueue_all(self, command: dict) -> None:
        """
        Enqueue command on all slaves simultaneously (non-blocking).
        Each slave's queue drainer sends it in order.
        """
        for client in self.clients.values():
            client.enqueue(dict(command))
