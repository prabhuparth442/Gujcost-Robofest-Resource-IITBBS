# Concepts Reference — Technical Deep Dives

This file explains the *why* behind every major technical decision in the system.
It assumes you understand basic Python but may be new to drones, embedded systems,
or robotics. Each section links to further reading.

---

## 1. Why thermal cameras for mine detection?

Landmines are made of metal or plastic. Both materials have different **thermal inertia**
(how quickly they heat up and cool down) compared to the surrounding soil.

- **During the day:** Solar radiation heats the soil surface faster than the buried mine →
  the mine appears *cooler* than surrounding soil (cool anomaly)
- **At night / evening:** The surface soil cools faster than the mine →
  the mine appears *warmer* than surrounding soil (warm anomaly)

This creates a temperature difference (ΔT) that a thermal camera can detect. For buried
mines at typical depths (5–10 cm), this ΔT is very small: **±0.15 to ±1.25°C**.
Our `02_vision_filter.py` looks for exactly this bidirectional anomaly.

Surface discs (plastic/metal discs sitting on top of the ground) are much easier —
they heat up significantly in sunlight, creating ΔT of **+3 to +40°C**, detected by
`06_surface_filter.py`.

**Research basis:** arXiv:2410.23998 — UAV-based detection of landmines using infrared
thermography (88% detection accuracy, evening flights optimal)

---

## 2. Why a 32×24 sensor? Isn't that very low resolution?

The MLX90640 has only 768 pixels total. A standard phone camera has millions. Yet we
chose this sensor because:

1. **Cost** — ~₹3,000 vs ₹15,000+ for FLIR Lepton at 160×120
2. **Weight** — essential for staying under the 750g limit
3. **I²C interface** — directly supported by the BCM2835 library on Raspberry Pi
4. **Adequate for the task** — at 1.5m altitude with 55°×35° FOV, each pixel covers
   ~4.9 cm of ground. A buried mine (typically 10–20 cm diameter) covers 2–4 pixels,
   which is enough for statistical detection

The physics of thermal detection doesn't require high spatial resolution — it requires
**temperature resolution** (sensitivity), which the MLX90640 provides at ~0.1°C.

---

## 3. Why C++ for the thermal camera driver?

The MLX90640 requires precise I²C timing during EEPROM reads and per-frame DMA transfers.
The Melexis C++ driver handles this timing at the hardware level using BCM2835's direct
memory access.

When we tried Python's `smbus2` library instead, we got frequent frame drops and EEPROM
read failures because Python's garbage collector would pause execution at the wrong moment,
causing I²C timing violations.

The solution: run the C++ driver as a **subprocess** and stream the float data over stdout:

```python
proc = subprocess.Popen(["sudo", "./bin/mlx_stdout"], stdout=subprocess.PIPE)
raw  = proc.stdout.read(768 * 4)   # one frame = 3072 bytes
frame = np.frombuffer(raw, dtype=np.float32).reshape((24, 32))
```

The Python code just reads bytes — it never touches I²C directly.

---

## 4. What is Fixed Pattern Noise (FPN) and why must we remove it?

Every pixel in the MLX90640's focal plane array was fabricated slightly differently
due to manufacturing tolerances. This results in a fixed "fingerprint" — some pixels
always read slightly hotter, some always colder — regardless of what the camera is
actually looking at.

If left uncorrected, FPN creates fake hot/cold spots that our vision filter would
trigger on, producing false mine detections over bare soil.

**Calibration procedure** (done by `00_preflight_calib.py`):
1. Point camera at a uniform-temperature surface (or use lens cap)
2. Capture 30 frames
3. Average them → this average *is* the FPN pattern
4. Save to `slave/config/fpn_pattern.npy`
5. During flight, subtract FPN from every live frame before processing

FPN drifts with sensor temperature, so **recalibrate before every flight**.

---

## 5. How does A* path planning work?

A* (pronounced "A-star") is an algorithm that finds the shortest path between two points
on a grid while avoiding obstacles.

**How it works:**
1. The field is divided into a grid of 0.5m × 0.5m cells
2. Each cell is either free or blocked (hazard/forbidden)
3. A* starts from the drone's current position and explores cells outward
4. At each step it prioritises cells that are both close to the current position AND
   close to the destination (using a distance estimate called the **heuristic**)
5. When it reaches the destination, it traces back the path it took

**Why Euclidean heuristic?** We allow 8-directional movement (including diagonals), so
the Euclidean (straight-line) distance to the goal is a good estimate. Manhattan distance
(only horizontal/vertical) would overestimate and slow the search.

**Iteration limit (200,000):** If A* runs for too long, the field may be completely blocked
(e.g., hazard zones merged into a wall). The limit prevents infinite loops — the drone
will hover in place and report an error if path planning fails.

Further reading: https://www.redblobgames.com/pathfinding/a-star/introduction.html
(the best beginner explanation of A* on the internet)

---

## 6. Why no GPS during the mission?

The competition rules explicitly forbid GPS during the 10-minute mission window. This is
intentional — GPS is unavailable in real minefields (dense forest, GPS jamming in conflict
zones, indoor buildings).

Our solution: **origin-lock** at the start position.
1. Before the mission: record the GPS fix at the start point → `origin_state.json`
2. During the mission: all positions are expressed as metres offset from that origin
3. pymavlink `SET_POSITION_TARGET_GLOBAL_INT` converts local coordinates back to GPS using our math

The ArduPilot EKF3 (Extended Kalman Filter) inside the SpeedyBee continues to estimate position
by fusing barometer (altitude) + accelerometer + compass — without GPS. This drifts over
time (roughly 0.3–0.5 m/min), which is why we use a fuzzy footprint disc in the coverage
grid rather than exact cell marking.

---

## 7. How does the coverage grid work?

The grid (`slave/grid_map.py`) is a **sparse dictionary** — only cells that have been
visited are stored. The entire 60m×15m field at 0.5m resolution would be 3600 cells
maximum, but most missions will touch only a fraction.

Each cell has a **flag** (a bitfield — an integer where individual bits mean different things):

```
0  = UNVISITED  (not in dict)
1  = SCANNED    (drone flew nearby, sensor covered this cell)
2  = DETECTION  (surface disc detected here)
4  = HAZARD     (pre-known buried mine avoidance zone)
8  = FORBIDDEN  (pole / statue no-fly zone)
```

Flags are **OR'd** together — a cell can be both SCANNED and HAZARD (value = 5).
Once a flag is set, it is never cleared (cells only go up in state, never down).

When the drone moves, `mark_position(x, y)` paints a disc of radius 0.78m around the
actual GPS position — this "fuzzy footprint" absorbs GPS drift naturally.

---

## 8. How does TCP length-prefixing work?

TCP is a stream protocol — it delivers bytes in order, but doesn't know where one
"message" ends and the next begins. Without framing, the receiver can't tell if it got
half a packet or two packets glued together.

Our solution: every packet is prefixed with its length as a 4-byte big-endian integer.

```
Bytes 0-3:  length of payload (uint32, big-endian)
Bytes 4-N:  JSON payload (UTF-8)
```

The receiver reads 4 bytes first, converts to an integer, then reads exactly that many
more bytes. No ambiguity.

```python
# Sender (slave):
raw = json.dumps(packet).encode('utf-8')
sock.sendall(len(raw).to_bytes(4, 'big'))
sock.sendall(raw)

# Receiver (master):
length = int.from_bytes(sock.recv(4), 'big')
data   = sock.recv(length)
packet = json.loads(data)
```

---

## 9. How does offline speech recognition work?

Vosk is an offline speech recognition library. Instead of sending audio to Google/Amazon
servers, it runs a neural network model directly on the Raspberry Pi.

The model we use (`vosk-model-small-en-us-0.15`) is ~50 MB and fast enough to run on
a Pi 4 in near-real-time.

The audio path:
```
Phone microphone → getUserMedia() API in browser
    → 16-bit mono PCM at 16,000 Hz sample rate
    → POST to /api/audio_chunk on master Pi's Flask server
    → Vosk KaldiRecognizer.AcceptWaveform(pcm_bytes)
    → Returns recognised text: "start mission"
    → Master matches text to command table → sends TCP to all slaves
```

**Why HTTPS is required:** Browser security policy (`getUserMedia()`) only grants
microphone access on HTTPS pages. HTTP is rejected by the browser. That's why we
generate a self-signed TLS certificate — it's "insecure" in the browser's eyes
(you'll get a warning) but encrypts the audio stream.

Vosk documentation: https://alphacephei.com/vosk/

---

## 10. What is the master–slave architecture?

A **master–slave architecture** means one computer is in charge (master) and the others
take orders (slaves).

In our system:
- The **master drone** knows the full mission plan, keeps the mine map, does path planning,
  processes voice commands, and coordinates between all slaves
- Each **slave drone** is responsible only for its assigned scan lane: fly → detect → report

Advantages of centralisation:
- Simple to debug (one place has all state)
- No consensus problem (no voting between drones)
- Easy to add or remove drones

Disadvantages:
- Single point of failure (master crashes → mission stops)
- All communication goes through master (bottleneck at scale)

For 3 drones and a 10-minute mission, centralisation is the right trade-off.

---

## 11. How does mine deduplication work?

When two slaves scan overlapping lanes, they may both detect the same mine and both
report it. Without deduplication, the master's mine list would have duplicates.

The deduplication algorithm in `05_map_verifier.py`:
1. When a new mine report arrives at (lat, lon), convert to local coordinates (x, y)
2. Check all previously confirmed mines
3. If any existing mine is within `DEDUPLICATION_RADIUS_M = 1.5 m` of the new report,
   it's considered the same mine
4. Average the coordinates (fusion): `x_avg = (x_existing + x_new) / 2`
5. Discard the new report (return False = "not new, skip reporting")

1.5m was chosen based on GPS accuracy (~0.5m) + drone position error (~0.5m) + sensor
pixel-to-metre uncertainty (~0.5m) — the total system spatial error budget.

---

## 12. Why 4 passes and 3 drones?

The field is 15m wide. Each MLX90640 at 1.5m altitude has a ground footprint of ~1.56m wide.

With 3 drones at 1.4m spacing, one pass covers 3 × 1.56m ≈ 4.7m (with slight overlap).
Four passes × 4.7m ≈ 18.8m — enough to cover the 15m field with margin.

The `LANE_STEP_M = 1.4m` was tuned to ensure every point in the field is covered by
at least one drone's sensor footprint in at least one pass.

**Known issue:** Four full passes takes ~13 minutes at 0.3 m/s — slightly over the 10-minute
limit. Improvements: faster flight speed, larger step size, or 3 passes with 90% coverage.
See `research/04_path_planning.md` for options.
