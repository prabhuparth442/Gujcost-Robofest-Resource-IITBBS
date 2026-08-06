#!/usr/bin/env python3
"""
pc_sector_viewer.py  —  Run this on your PC.
Receives sector scan results from the drone over WiFi and displays them
as a live gallery in your browser at http://localhost:9000

Each "card" shows:
  - Raw thermal image of the sector (colormapped)
  - Processed binary detection map
  - Final annotated image (mine circled, if found)
  - Sector number, timestamp, confidence, GPS coords

Data lives only in RAM — closing this script wipes everything.
No files written to disk.

On DRONE side: the orchestrator calls tunnel.send_sector_result(...)
after each sector completes (see 08_comms_link.py patch below).

Usage:
    python3 pc_sector_viewer.py            # default port 9000
    python3 pc_sector_viewer.py --port 9001
"""

import json
import base64
import socket
import threading
import argparse
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── In-memory store (wiped on exit) ─────────────────────────────────────────
_sectors = []          # list of sector result dicts, newest first
_store_lock = threading.Lock()
_total_mines = 0

DASHBOARD = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>DRONE SWARM · SECTOR RESULTS</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=Barlow+Condensed:wght@300;600;800&display=swap" rel="stylesheet">
<style>
:root {
  --bg:       #07090c;
  --surface:  #0d1117;
  --border:   #1c2333;
  --green:    #3ddc84;
  --red:      #ff4757;
  --amber:    #ffa502;
  --blue:     #70a5fd;
  --muted:    #4a5568;
  --text:     #cdd9e5;
  --mono:     'IBM Plex Mono', monospace;
  --ui:       'Barlow Condensed', sans-serif;
}
* { box-sizing: border-box; margin: 0; padding: 0; }

body {
  background: var(--bg);
  color: var(--text);
  font-family: var(--ui);
  min-height: 100vh;
}

/* subtle grid texture */
body::before {
  content:'';
  position:fixed; inset:0;
  background-image:
    linear-gradient(rgba(255,255,255,.012) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255,255,255,.012) 1px, transparent 1px);
  background-size: 40px 40px;
  pointer-events:none;
  z-index:0;
}

header {
  position: sticky; top: 0; z-index: 100;
  display: flex; align-items: center; justify-content: space-between;
  padding: 0 32px;
  height: 56px;
  background: rgba(7,9,12,0.95);
  backdrop-filter: blur(12px);
  border-bottom: 1px solid var(--border);
}

.brand {
  font-family: var(--mono);
  font-size: 11px;
  letter-spacing: 4px;
  color: var(--green);
  text-transform: uppercase;
}
.brand em { color: var(--muted); font-style: normal; }

.header-stats {
  display: flex; gap: 32px; align-items: center;
}
.hstat {
  font-family: var(--mono);
  font-size: 11px;
  color: var(--muted);
}
.hstat b { color: var(--text); font-size: 16px; }
.hstat.mines b { color: var(--red); }

.conn-dot {
  width: 8px; height: 8px; border-radius: 50%;
  background: var(--green);
  box-shadow: 0 0 8px var(--green);
  animation: pulse 1.8s infinite;
}
.conn-dot.waiting { background: var(--amber); box-shadow: 0 0 8px var(--amber); }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }

main {
  position: relative; z-index: 1;
  padding: 32px;
  max-width: 1600px;
  margin: 0 auto;
}

.empty-state {
  display: flex; flex-direction: column; align-items: center;
  justify-content: center; gap: 16px;
  min-height: 60vh;
  font-family: var(--mono);
}
.empty-state .big { font-size: 48px; opacity: .08; }
.empty-state p { color: var(--muted); font-size: 12px; letter-spacing: 2px; }

/* SECTOR CARDS */
.sector-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(480px, 1fr));
  gap: 20px;
}

.sector-card {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 4px;
  overflow: hidden;
  animation: slideIn .35s ease;
}
.sector-card.mine-found {
  border-color: rgba(255,71,87,.4);
  box-shadow: 0 0 24px rgba(255,71,87,.08);
}
.sector-card.clear {
  border-color: rgba(61,220,132,.15);
}

@keyframes slideIn {
  from { opacity:0; transform: translateY(12px); }
  to   { opacity:1; transform: none; }
}

.card-top {
  display: flex; justify-content: space-between; align-items: center;
  padding: 10px 16px;
  border-bottom: 1px solid var(--border);
}
.sector-label {
  font-family: var(--mono);
  font-size: 11px;
  letter-spacing: 3px;
  color: var(--muted);
}
.sector-num {
  font-family: var(--ui);
  font-weight: 800;
  font-size: 22px;
  color: var(--text);
  line-height: 1;
}
.result-badge {
  padding: 4px 12px;
  border-radius: 2px;
  font-family: var(--mono);
  font-size: 10px;
  letter-spacing: 2px;
  font-weight: 600;
}
.result-badge.mine  { background: rgba(255,71,87,.15); color: var(--red);   border: 1px solid rgba(255,71,87,.3); }
.result-badge.clear { background: rgba(61,220,132,.08); color: var(--green); border: 1px solid rgba(61,220,132,.2); }

/* Image strip */
.img-strip {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1px;
  background: var(--border);
}
.img-slot {
  position: relative;
  aspect-ratio: 4/3;
  background: #080c10;
  overflow: hidden;
}
.img-slot img {
  width: 100%; height: 100%;
  object-fit: cover;
  display: block;
}
.img-slot .slot-label {
  position: absolute; bottom: 0; left: 0; right: 0;
  padding: 3px 6px;
  background: rgba(0,0,0,.7);
  font-family: var(--mono);
  font-size: 9px;
  color: rgba(255,255,255,.5);
  letter-spacing: 1px;
}
.img-slot.no-img {
  display: flex; align-items: center; justify-content: center;
  font-family: var(--mono); font-size: 9px;
  color: var(--muted); letter-spacing: 1px;
}

/* Meta grid */
.card-meta {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 1px;
  background: var(--border);
  border-top: 1px solid var(--border);
}
.meta-cell {
  background: var(--surface);
  padding: 8px 14px;
}
.meta-cell .label {
  font-family: var(--mono);
  font-size: 9px;
  color: var(--muted);
  letter-spacing: 1px;
  margin-bottom: 2px;
}
.meta-cell .value {
  font-family: var(--mono);
  font-size: 12px;
  color: var(--text);
}
.meta-cell .value.green { color: var(--green); }
.meta-cell .value.red   { color: var(--red); }
.meta-cell .value.amber { color: var(--amber); }

/* GPS coords block */
.gps-block {
  padding: 8px 16px;
  border-top: 1px solid var(--border);
  display: flex; gap: 24px; align-items: center;
}
.gps-block .gps-label {
  font-family: var(--mono); font-size: 9px;
  color: var(--muted); letter-spacing: 1px;
}
.gps-block .gps-val {
  font-family: var(--mono); font-size: 11px;
  color: var(--blue);
}
.gps-block .no-gps {
  font-family: var(--mono); font-size: 10px;
  color: var(--muted);
}

/* Timestamp footer */
.card-footer {
  padding: 6px 16px;
  border-top: 1px solid var(--border);
  display: flex; justify-content: space-between;
  font-family: var(--mono); font-size: 9px;
  color: var(--muted);
}
</style>
</head>
<body>

<header>
  <div class="brand">◈ DRONE SWARM <em>//</em> SECTOR RESULTS</div>
  <div class="header-stats">
    <div class="hstat">SECTORS <b id="sectorCount">0</b></div>
    <div class="hstat mines">MINES FOUND <b id="mineCount">0</b></div>
    <div class="conn-dot waiting" id="connDot" title="Waiting for drone data"></div>
  </div>
</header>

<main>
  <div id="emptyState" class="empty-state">
    <div class="big">◈</div>
    <p>WAITING FOR SECTOR DATA FROM DRONE</p>
    <p style="opacity:.5">Results will appear here as each sector completes</p>
  </div>
  <div class="sector-grid" id="sectorGrid"></div>
</main>

<script>
let knownCount = 0;

function buildCard(s) {
  const hasMine = s.mine_found;
  const card = document.createElement('div');
  card.className = 'sector-card ' + (hasMine ? 'mine-found' : 'clear');
  card.id = 'sector-' + s.sector_id;

  const imgSlot = (b64, label) => b64
    ? `<div class="img-slot"><img src="data:image/jpeg;base64,${b64}" loading="lazy">
       <div class="slot-label">${label}</div></div>`
    : `<div class="img-slot no-img">${label}<br>N/A</div>`;

  const gpsBlock = hasMine
    ? `<div class="gps-block">
        <div><div class="gps-label">MINE LAT</div><div class="gps-val">${s.mine_lat ? s.mine_lat.toFixed(6) : '—'}°</div></div>
        <div><div class="gps-label">MINE LON</div><div class="gps-val">${s.mine_lon ? s.mine_lon.toFixed(6) : '—'}°</div></div>
        <div><div class="gps-label">CONFIDENCE</div><div class="gps-val" style="color:var(--amber)">${s.confidence ? (s.confidence*100).toFixed(1)+'%' : '—'}</div></div>
       </div>`
    : `<div class="gps-block"><span class="no-gps">No anomaly detected this sector</span></div>`;

  const hotPct = s.hot_pixel_pct ? s.hot_pixel_pct.toFixed(1)+'%' : '—';
  const confVal = s.confidence ? (s.confidence*100).toFixed(1)+'%' : '—';
  const framesVal = s.frames_captured || '—';

  card.innerHTML = `
    <div class="card-top">
      <div>
        <div class="sector-label">SECTOR</div>
        <div class="sector-num">${String(s.sector_id).padStart(2,'0')}</div>
      </div>
      <div class="result-badge ${hasMine ? 'mine' : 'clear'}">
        ${hasMine ? '⚑ MINE DETECTED' : '✓ AREA CLEAR'}
      </div>
    </div>
    <div class="img-strip">
      ${imgSlot(s.img_raw,    'RAW THERMAL')}
      ${imgSlot(s.img_binary, 'BINARY MAP')}
      ${imgSlot(s.img_final,  'DETECTION')}
    </div>
    <div class="card-meta">
      <div class="meta-cell">
        <div class="label">HOT PIXELS</div>
        <div class="value ${s.hot_pixel_pct > 0 ? 'amber' : ''}">${hotPct}</div>
      </div>
      <div class="meta-cell">
        <div class="label">CONFIDENCE</div>
        <div class="value ${s.mine_found ? 'red' : 'green'}">${confVal}</div>
      </div>
      <div class="meta-cell">
        <div class="label">FRAMES</div>
        <div class="value">${framesVal}</div>
      </div>
    </div>
    ${gpsBlock}
    <div class="card-footer">
      <span>DRONE: ${s.drone_id || 'drone3'}</span>
      <span>${s.timestamp || ''}</span>
    </div>
  `;
  return card;
}

async function poll() {
  try {
    const r = await fetch('/results');
    const data = await r.json();

    document.getElementById('sectorCount').textContent = data.sectors.length;
    document.getElementById('mineCount').textContent   = data.total_mines;

    const dot = document.getElementById('connDot');
    if (data.sectors.length > 0) {
      dot.className = 'conn-dot';
      dot.title = 'Receiving data';
    }

    if (data.sectors.length > knownCount) {
      document.getElementById('emptyState').style.display = 'none';
      const grid = document.getElementById('sectorGrid');
      // Add new cards at top
      for (let i = data.sectors.length - 1; i >= knownCount; i--) {
        const card = buildCard(data.sectors[i]);
        grid.insertBefore(card, grid.firstChild);
      }
      knownCount = data.sectors.length;
    }
  } catch(e) {}
  setTimeout(poll, 1500);
}
poll();
</script>
</body>
</html>"""


# ── HTTP server ──────────────────────────────────────────────────────────────
class _WebHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(DASHBOARD.encode())
        elif self.path == '/results':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            with _store_lock:
                self.wfile.write(json.dumps({
                    "sectors": _sectors,
                    "total_mines": _total_mines
                }).encode())
        else:
            self.send_response(404)
            self.end_headers()


# ── TCP receiver (drone sends data here) ────────────────────────────────────
def _tcp_receiver(port):
    """Listens for JSON packets from the drone's DroneTunnel."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.settimeout(0.5)
    server.listen(5)
    print(f"[RECEIVER] Listening for drone data on TCP port {port}...")

    global _total_mines

    while True:
        try:
            conn, addr = server.accept()
            data = b""
            conn.settimeout(3.0)
            try:
                while True:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    data += chunk
            except socket.timeout:
                pass
            conn.close()

            if not data:
                continue

            try:
                # Strip the 4-byte length prefix sent by DroneTunnel._transmit()
                if len(data) > 4:
                    data = data[4:]
                payload = json.loads(data.decode('utf-8'))
            except Exception as e:
                print(f"[RECEIVER] Bad JSON ({len(data)} bytes): {e}")
                continue

            if payload.get('type') == 'sector_result':
                with _store_lock:
                    _sectors.append(payload)
                    if payload.get('mine_found'):
                        _total_mines += 1
                label = "MINE FOUND" if payload.get('mine_found') else "clear"
                print(f"[RECEIVER] Sector {payload.get('sector_id')} → {label}")

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[RECEIVER] Error: {e}")


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',     type=int, default=9000, help='Web dashboard port')
    parser.add_argument('--tcp-port', type=int, default=5000, help='TCP receive port (must match drone)')
    args = parser.parse_args()

    # Start TCP receiver in background
    t = threading.Thread(target=_tcp_receiver, args=(args.tcp_port,), daemon=True)
    t.start()

    # Start web dashboard
    print(f"[VIEWER] Dashboard at http://localhost:{args.port}")
    print(f"[VIEWER] Data is RAM-only — closing this script wipes everything.")
    print(f"[VIEWER] Ctrl+C to stop.\n")
    try:
        HTTPServer(('0.0.0.0', args.port), _WebHandler).serve_forever()
    except KeyboardInterrupt:
        print("\n[VIEWER] Stopped. All data cleared.")
