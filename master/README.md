# Master Drone — S.A.F.E. Ground Server

The master drone runs a **Flask web server** that:
- Receives telemetry (GPS, mine reports, coverage grid) from all 3 slave drones
- Serves a live map UI to the operator's phone browser
- Accepts voice commands via the phone microphone (Vosk offline speech recognition)
- Runs an A* path planner to generate the safe path for the person-at-risk
- Sends movement commands (GOTO, PAUSE, LAND, SIDE_MOVE) to slaves via TCP

> The master uses the **same Raspberry Pi hardware** as the slaves — it just runs different software.

---

## File map

```
master/
│
├── app.py              ← ENTRY POINT
│                          Thread 1: Flask web server (HTTPS :443)
│                          Thread 2: Vosk offline speech recogniser
│
├── index.html          ← Phone browser UI
│                          Live map: drone positions, mines, coverage cells
│                          Streams phone microphone to /api/audio_chunk
│                          Shows Vosk transcript
│
├── fieldmap.py         ← Field geometry (identical to slave version)
│                          BURIED_MINES, FORBIDDEN_ZONES, PASS_LANES
│
├── grid_map.py         ← Master coverage grid
│                          Merges grid snapshots from all 3 slaves
│                          Provides coverage_pct() for mission monitoring
│
├── swarm_state.py      ← Shared swarm state
│
├── tcp_channel.py      ← TCP client → slaves :14560
│                          Sends GOTO / PAUSE / RESUME / LAND / SIDE_MOVE
│
├── tcp_commander.py    ← Command dispatch helpers
│
├── udp_channel.py      ← UDP receiver from slaves :14550
│                          Collects telemetry at 5 Hz
│
├── udp_telementry.py   ← Telemetry parsing and state update
│
├── requirements.txt
├── config/
│   └── origin_state.json
└── bin/
    ├── mlx_stdout
    └── camera_bridge
```

---

## Quick start

### 1. Install dependencies

```bash
sudo apt-get install -y python3-pip portaudio19-dev libatomic1
pip3 install flask vosk
```

### 2. Download Vosk model (one-time)

```bash
cd master/
# Download the small English model (~40 MB)
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
# Result: ./vosk-model-small-en-us-0.15/ directory
```

### 3. Generate TLS certificate (one-time)

Required so the phone browser allows microphone access over the local network:

```bash
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem \
  -days 365 -nodes -subj "/CN=localhost"
```

### 4. Start the server

```bash
sudo python3 app.py
# Starts on https://0.0.0.0:443
# Open on phone browser: https://<master-pi-ip>/
# (Accept the self-signed cert warning)
```

### 5. Set field origin in the UI

Before the mission starts, tap "Set Origin" in the UI to anchor the coordinate system. This tells the A* planner where (0, 0) is.

---

## API endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Serves `index.html` map UI |
| `/api/status` | GET | Returns drone positions, mines, coverage, transcript |
| `/api/set_origin` | POST | Sets field origin GPS + heading |
| `/api/drone_update` | POST | Receives slave telemetry and mine reports |
| `/api/audio_chunk` | POST | Phone browser streams raw PCM audio |
| `/api/voice_command` | POST | Fallback: browser sends pre-parsed command string |
| `/api/get_my_location` | POST | A* safe path from person's GPS to exit |
| `/api/pending_commands` | GET | Slaves poll this to consume queued commands |

---

## Voice commands (Vosk offline)

The phone browser captures microphone audio via `getUserMedia()` and POSTs 16-bit mono PCM to `/api/audio_chunk` every ~0.5 s. The Vosk worker decodes speech and maps to intents:

| Say | Maps to | Effect |
|-----|---------|--------|
| "start" / "go" / "begin" / "launch" / "fly" | `start` | Marks mission as started |
| "pause" / "stop" / "hold" / "wait" / "freeze" | `pause` | Sends PAUSE to all slaves |
| "resume" / "continue" / "proceed" / "go ahead" | `resume` | Sends RESUME to all slaves |
| "scan" / "check" / "rescan" / "search" | `scan` | Re-scan command |
| "forward" / "advance" / "move forward" | `forward` | Advance command |
| "land" / "abort" / "emergency" / "down" | `land` | Emergency land all drones |
| "status" / "report" / "where" | `status` | Status query |

> **Tip:** Speak clearly and pause between words. Vosk is keyword-based — it only needs to hear the trigger word anywhere in the sentence.

---

## How the A* path planner works

When the person-at-risk opens `https://<master-ip>/` on their phone, the browser sends their GPS location to `/api/get_my_location`. The server:

1. Converts person's GPS → local coordinates
2. Combines all mine detections + pre-known buried mines + forbidden zones into hazard circles (1 m radius each)
3. Runs A* from person's position to the exit zone (Y = +100 m)
4. Returns a list of 0.5 m grid waypoints forming the safe path
5. The phone map renders the path with arrows/highlights

The path re-computes on each call, so if new mines are detected mid-crossing, the path updates automatically.

---

## Network requirements

The master Pi acts as a **WiFi access point**:

```bash
# Example: set up Pi as AP (hostapd + dnsmasq)
# Master IP on AP interface: 10.42.0.1
# Slaves connect to this AP
# Phone connects to this AP
```

All communication is local — no internet required during the competition.

---

## Troubleshooting

**"No cert.pem — running HTTP. Browser may block getUserMedia."**  
Generate the TLS cert (see step 3 above). Without HTTPS, Chrome/Safari won't allow mic access.

**"vosk not installed"**  
Run `pip install vosk`. If Vosk model is missing, speech recognition thread silently exits but the rest of the server still works (voice commands will be unavailable).

**Slaves not appearing on map**  
Check all slaves have `MASTER_IP=<this-pi-ip>` set and are sending UDP to port 14550. Use `sudo netstat -ulnp | grep 14550` to verify the receiver is listening.

**Mine reports not appearing**  
Port 5000 must be open and accessible. Check with `sudo netstat -tlnp | grep 5000`.

**A* returns empty path**  
Too many overlapping hazard circles with 1 m radius each may block all paths through a narrow field. Reduce `SAFE_RADIUS` in `app.py` or verify the mine GPS coordinates are correct.
