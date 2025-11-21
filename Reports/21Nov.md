# Finding on Soil Simulation Plastic Detection

**Prompt:**  
See this file of possible detection sensors and based on realistic data simulate detectibility of plastic plates underground at depth of 15cm, use properties don't guess these will be values and that. remember the sensor will be 2m above ground.

**Answer:**  
Short version: if you plug real soil/plastic properties and real sensor specs into even a simple model, a **15 cm deep plastic plate is effectively invisible to any realistic low-cost sensor on a drone flying 2 m above the ground.** The only remotely plausible modality is thermal IR, and even there the predicted signal is way below the noise floor of parts like the AMG8833.

I'll walk through a conservative thermal-IR calculation and then quickly sanity-check the other sensor families in your 18Nov file.

---

## **1. Setup & physical assumptions**

I'll keep the assumptions explicit so you can tweak numbers later.

**Target**

* Plastic plate ≈ anti-personnel mine: say **diameter 20 cm, thickness 5 cm**.
* Material ~ PVC / generic plastic  
  - dielectric constant εᵣ ≈ 3–3.8 at 1 kHz  
  - thermal conductivity k ≈ 0.16–0.19 W/m·K

**Soil**

* Heavy clay / sand-loam type (similar to landmine papers).
* Typical **thermal diffusivity α ≈ 0.5×10⁻⁶ m²/s** is widely used as a "standard soil" value.
* Thermal model & real data from Moukalled et al. (Heat & Mass Transfer in Moist Soil, Part II):  
  - TNT / Tetryl mines buried at **1 cm depth**: surface thermal contrast up to **4–9 °C** depending on soil moisture.  
  - As burial depth increases, **surface contrast falls sharply and is "nearly vanished" by 8 cm depth**.

**Drone & sensor**

* Height: **h = 2 m** above ground.
* Sensor: **AMG8833 (8×8 array, 60° FOV)**  
  - Accuracy: ±2.5 °C.  
  - Resolution: 0.25 °C steps.  
  - Typical board implementations advertise **NETD ≈ 0.05 °C (50 mK)** at 1 Hz.
* Each pixel FOV ≈ 60°/8 ≈ 7.5°.  
  At 2 m altitude, the projected ground footprint per pixel is  
  `w ≈ 2h·tan(Δθ/2) ≈ 2·2·tan(3.75°) ≈ 0.26 m` (26 cm) wide → pixel area ≈ 0.07 m².
* Plate area = π·(0.1 m)² ≈ 0.031 m² → **fill factor ≈ 0.44** of one pixel (best case, plate centred under a pixel).

---

## **2. Thermal IR "simulation" for 15 cm depth**

### **2.1. How fast does the thermal signal die with depth?**

For diurnal heating, soil behaves like a **thermal low-pass filter**.  
The temperature wave amplitude decays roughly as:

$$A(z) = A_0 \exp\left(-\frac{z}{\delta}\right), \quad \delta = \sqrt{\frac{2\alpha}{\omega}}$$

where:

* $\alpha$ is thermal diffusivity,
* $\omega = 2\pi / T$ with $T = 24\ \text{h}$.

With α = 0.5×10⁻⁶ m²/s, δ ≈ 0.117 m (11.7 cm). So the *general* temperature wave at 15 cm depth is only:

$$\exp(-0.15 / 0.117) \approx 0.28$$

of its surface amplitude.

But we care about **contrast** between "soil above mine" vs "normal soil", and for that we can anchor to Moukalled's results instead of guessing:

* For a plastic-cased explosive mine at **1 cm burial depth**, maximum surface contrast ΔT₁ ≈ **4 °C** (dry heavy clay).
* At **8 cm depth**, the paper states the surface contrast **"nearly vanishes"** – their plots show values well below ~0.5 °C and trending to zero by that depth.

Let's approximate the depth-dependence of the *maximum* surface contrast with a simple exponential:

$$\Delta T(z) \approx \Delta T_1 \exp\left(-\frac{z - z_1}{\lambda}\right)$$

Use two anchor points from the paper:

* At $z_1 = 0.01\ \text{m}$: $\Delta T_1 = 4\ ^\circ\text{C}$
* At $z_2 = 0.08\ \text{m}$: take a generous "nearly vanished" value of $\Delta T_2 = 0.2\ ^\circ\text{C}$

Solving for λ:

$$\lambda = \frac{z_2 - z_1}{\ln(\Delta T_1 / \Delta T_2)} \approx 2.3\ \text{cm}$$

Then at **z = 15 cm**:

$$\Delta T(0.15) \approx 4\ \exp\left(-\frac{0.15 - 0.01}{0.023}\right) \approx 0.01^\circ\text{C} = 10\ \text{mK}$$

If we're *very* generous and assume ΔT₂ at 8 cm was actually 0.5 °C, the same fit gives ≈ **0.063 °C (63 mK)** at 15 cm. Either way, the physically plausible range is something like:

**Predicted max surface contrast at 15 cm depth**  
≈ **2.5–60 mK**, depending on soil and how optimistic you are.

That's already *far* below the multi-degree contrasts you see in most "cool" thermal-mine demo videos, because those usually use shallow 1–4 cm burial.

### **2.2. Pixel averaging at 2 m altitude**

The mine only covers ≈44 % of a pixel footprint at 2 m. The pixel sees an *average* temperature:

$$\Delta T_\text{pixel} \approx \text{fill factor} \times \Delta T_\text{surface}$$

So:

* Optimistic case: 60 mK surface contrast → ≈ **26 mK per pixel**.
* More realistic case: 10 mK surface contrast → ≈ **4–5 mK per pixel**.
* Pessimistic / moist or noisy soil: a few mK or less.

### **2.3. Compare with real sensor noise**

From actual AMG8833 modules:

* Resolution: 0.25 °C steps.
* Accuracy: ±2.5 °C (typical).
* Some modules advertise **NETD ≈ 0.05 °C (50 mK) at 1 Hz** (best-case).

So even in the optimistic scenario (~26 mK pixel contrast), your signal is:

$$\text{SNR} \approx \frac{26\ \text{mK}}{50\ \text{mK}} \approx 0.5 \quad (\text{per frame})$$

and in realistic conditions (≈5–10 mK pixel contrast), SNR ≪ 0.2.

That's **before** you include:

* Sensor drift and calibration error (±2.5 °C).
* Non-uniform soil moisture & texture (which easily produce ≫0.1 °C natural variations).
* Wind, partial vegetation cover, clouds, flight altitude variations, etc.

With that much clutter, a 5–20 mK signal is essentially undetectable with an 8×8 grid camera. In the landmine literature, even high-end cooled IR cameras (NETD ≈ 20–30 mK) struggle beyond ~10 cm burial for anti-personnel-sized plastic mines under realistic conditions.

**Conclusion for thermal IR at 2 m:**

For a 15 cm deep plastic plate, surface thermal contrast is on the order of a few–tens of mK.  
After pixel averaging at 2 m altitude, the signal sits **at or below the noise floor** of AMG8833-class sensors. Robust detection is not realistically achievable.

---

## **3. Other sensor families from your 18Nov list**

Now, check the rest explicitly **with the 2 m stand-off constraint**.

### **3.1. Capacitive / dielectric sensing**

Physics:

$$C = \varepsilon_0 \varepsilon_r \frac{A}{d}$$

A remote capacitance sensor's field lines mostly travel through **air**; only a tiny fraction goes down into the soil and even less reaches 15 cm depth.

Relevant material properties:

* PVC / plastic εᵣ ≈ 3–4.
* Dry soil εᵣ ≲ 5; moist soil quickly rises to 10–20+.

Key real-world facts:

* Cheap "capacitive soil moisture" probes that *do* detect at 10–60 cm are **inserted into the soil**, with electrodes centimetres from the sensing volume.
* Industrial capacitive proximity sensors typically switch at **millimetres to a few centimetres** of stand-off; even highly tuned designs talk about ranges "up to 250 mm" (25 cm) for large targets, and that's still in *air*, not through 15 cm of soil.

In your geometry:

* Distance from electrode to plate centre ≈ **h + depth ≈ 2.15 m**.
* Very roughly, the relevant fraction of field lines that "touch" the buried plate scales like (target area / sphere area) ≈ A / (4πr²). Here that's ∼0.031 m² / (4π·(2.15 m)²) ≈ 0.0005.
* So the **relative change in capacitance** from inserting the plate is on the order of 10⁻⁴ of the already tiny soil-coupled component – far below what any practical capacitive front-end can resolve, especially on a moving drone in variable humidity.

At 2 m altitude, capacitive sensing will read "bulk air + ground" and the presence of a 20 cm plastic plate at 15 cm depth changes the apparent capacitance by an utterly negligible fraction. It's effectively **zero signal**.

Capacitive sensing only becomes meaningful if your electrodes are **within a few centimetres of the surface** and you're probing soil directly (as your 18Nov doc already says).

---

### **3.2. Metal detection (EMI / PI)**

Even if your "plastic plate" had *some* metal:

* Metal detectors are **electromagnetic induction** devices; sensitivity drops roughly with 1/r³ for dipole-like fields.
* Mine-clearance spec for hand-held systems: detect AT mines up to 300 mm depth and AP mines up to 100 mm depth **with detector head ≤10 cm above ground**.
* A UAV-borne GPR/EMI paper explicitly notes that detection "is limited by distance, since the sensor has to be as close as possible to the ground surface".

At 2 m altitude, the coil–target distance is ~2.1–2.2 m instead of 0.1 m. That's a ~20× distance increase, so the induction signal drops by **≈20³ ≈ 8000×**. You'd need an utterly different, very high-power radar-like system, not a hobby PI board.

For a **pure plastic plate (no metal)**, EMI is fundamentally blind anyway.

---

### **3.3. Ground-penetrating radar / microwave**

GPR can see plastic because it responds to dielectric contrast, but:

* Classic landmine GPR keeps antennas **4–5 cm above the ground** for stable signals.
* Reviews of landmine GPR emphasise that performance depends critically on **proximity to the ground**; hand-held and close-in systems are the only ones that meet operational requirements; stand-off airborne systems have "as yet some way to deliver useful performance" because of path loss and clutter.

With antennas 2 m in the air, a 1–2 GHz radar pulse must traverse:

* 2 m air down
* reflection + scattering at rough ground surface
* 0.15 m soil (strong attenuation, especially if moist)
* then the same path back up.

Even the **air path alone** adds >40 dB of free-space loss at these frequencies, and the tiny dielectric contrast between plastic (εᵣ~3–4) and surrounding soil (εᵣ~6–15) makes the reflection coefficient small. Combine that with your power, weight, and cost limits and the result is: **2 m-high GPR for 15 cm plastic targets is not realistic** for this competition.

---

### **3.4. Acoustic / seismic sensing**

Acoustic-to-seismic methods are actually *good* for plastic mines in research setups:

* They drive the ground with low-frequency sound, causing buried objects to resonate and modifying surface vibration.
* Successful experiments use **geophones or laser Doppler vibrometers** on or very near the surface; some systems place loudspeakers ~0.6 m above soil and a vibrometer just above or scanning centimeters over the ground.
* A JASA paper on using microphones instead of vibrometers notes that the acoustic field generated by Rayleigh waves is **evanescent in air**, and "can only be accurately measured well within a seismic wavelength of the surface" – typically meaning **centimetre-scale stand-off**, not metres.

At 2 m altitude, your microphone is essentially in *free air*, seeing direct speaker noise and wind, not the subtle Rayleigh-wave-induced pressure variations that encode the mine resonance. Again, the signal is essentially zero.

---

## **4. What this means for your project**

Putting it bluntly (because that's what you've been asking for):

1. **With the sensor at 2 m altitude, none of the realistic low-cost modalities (thermal, capacitive, EMI, acoustic) can reliably detect a 15 cm-deep plastic plate.**

   * Thermal IR: predicted pixel-level signals are **a few–tens of mK**, below camera noise and far below environmental clutter.
   * Capacitive / dielectric: effect on capacitance at 2 m is ≪10⁻³ of baseline – effectively zero.
   * EMI / metal detection: fundamentally blind to plastic and anyway distance-limited to ≪2 m.
   * Acoustic/seismic: requires near-surface sensors; 2 m is far outside the useful near-ground region.

2. The only technologies that *could* work at that depth are **close-in GPR or seismo-acoustic with laser vibrometer**, both of which:

   * need the sensing head **within centimetres of the surface**, and
   * are much too heavy / expensive for your 500 g, ₹40k-per-drone budget.

3. So, for *this* competition spec, "realistic simulation" basically tells you:

   * **Physical detection of 15 cm plastic mines from a 2 m-high micro-UAV is not feasible** with commodity sensors.
   * A defensible strategy is what you already started in the 18Nov report:
     * use the provided map + GPS + vision to predict mine zones,
     * maybe detect **very shallow or surface anomalies** (disturbed soil, vegetation, shallow objects) with cameras / shallow thermal,
     * and clearly state in your report that deeper plastic mines are outside the detection envelope given the altitude and budget constraints (backed with the kind of numbers above).

If you like, next step we can:

* Turn these calculations into a **small table / graph**: depth vs expected ΔT vs SNR for different sensors, so you can drop it straight into your report, and
* Explicitly write the "limitations" subsection explaining to judges why 2 m stand-off + 15 cm depth + plastic + low budget is physically contradictory.

---

## **Prompt 2:**  
Assume this the plastic plates are burried manually by orgnanizers few days before the competition and we need not detect just plastic plates but anything that could tell these is burial done here even when visually it doesn't seem so. do the simulation for these things after few days of burial and then create a report of this. be realistic and use real values and be professional.

## **Answer:**  
Here's the short version first:

* **Directly "seeing" a plastic plate at 15 cm depth from a 2 m-high drone is still essentially impossible with realistic sensors.** Thermal models and experiments show that for most soils you stop getting a significant *object* thermal signature at about **10 cm depth**.

* What *is* detectable for a few days to weeks after burial is **the disturbed soil itself**: slightly different bulk density, moisture, thermal conductivity, emissivity and micro-roughness compared to undisturbed ground. These changes create a **thermal IR anomaly of roughly 1–3 °C** at the surface under good conditions.

* A light drone-grade LWIR camera (e.g. MLX90640 or FLIR Lepton 3.5, NETD ≈0.05–0.1 °C) can easily resolve a 1 °C contrast at 2 m AGL, provided the disturbed patch is at least ~30×30 cm and the soil is mostly bare.

Below is a structured "report-style" answer with a simple but realistic simulation and practical conclusions.

---

## **1. Scenario and assumptions**

**Target / burial**

* Target: plastic "mine" plate, ~20–25 cm diameter, buried horizontally.
* Burial depth: **15 cm from surface to top of plate**.
* Hole: realistically ~30–40 cm diameter, ~20 cm deep, then backfilled and manually smoothed. That means **top ~10–15 cm of soil above the plate is fully remixed and loosened**.

**Environment**

* Terrain: bare or sparsely vegetated **sandy / silty agricultural soil** (similar to test plots used in mine-detection work).
* Time since burial: **2–7 days**.
* Weather: at least one clear-sky day with normal diurnal cycle, no heavy rain after burial (a strong rain flattens the disturbance signature much faster).

**Drone sensor**

To stay realistic and light, assume:

* **LWIR microbolometer** like MLX90640 (32×24 px, 55°×35° FOV, NETD ≈0.1 K)
  * At **2 m height**, this FOV gives ground coverage ~2.1 m × 1.3 m, so each pixel is ≈ **6.5 cm × 5.3 cm**.
* Optional high-res RGB camera (cheap phone-class module).
* Flight height: 2 m AGL.

A typical burial patch (≈30–40 cm across) therefore covers **5–8 thermal pixels**, not just one.

---

## **2. What changes in the soil after a fresh burial?**

### **2.1 Direct mine vs. disturbed soil**

Thermal landmine studies distinguish:

* **Volume effect** – the buried object's own thermophysical properties (heat capacity, conductivity) compared to soil.
* **Surface / disturbance effect** – the act of digging and backfilling changes soil structure, bulk density, porosity, moisture distribution and surface texture in the top few centimetres.

For mines deeper than about **10 cm**, experiments and modelling show the *volume effect* becomes too weak to generate useful surface contrast in most soils.

However, recent work from ERDC (TR-24-11) explicitly shows that **disturbed soil alone ("faux target") produces a measurable thermal IR signature that can persist for weeks**, and that this disturbance can equal or exceed the object's own signature.

So for your competition, **the thing we can plausibly detect is "recently disturbed patch of soil", not "plastic plate"**.

### **2.2 How large are the physical changes?**

From soil-physics studies:

* **Bulk density & porosity**
  * Remoulded / uncompacted soil has **20–50 % lower thermal conductivity** than fully compacted soil at the same water content.
  * Digging and backfilling lowers bulk density and increases air-filled porosity in the disturbed zone.

* **Thermal conductivity & heat capacity**
  * Typical λ for moist agricultural soils: ~0.7–1.5 W m⁻¹ K⁻¹; volumetric heat capacity C_vol ≈ 1.6–2.5 MJ m⁻³ K⁻¹.
  * Disturbed soil patch after burial is reasonably **20–30 % lower λ and 15–25 % lower C_vol** than undisturbed surroundings.

* **Moisture distribution**
  * Time-lapse electrical resistivity surveys of graves show marked resistivity changes (linked to moisture and porosity) in the disturbed zone within **1–2 days** of burial, which then evolve over weeks.

* **Surface micro-texture and emissivity**
  * Excavation redistributes fine and coarse particles; finer particles accumulate on the surface, raising emissivity and slightly changing albedo/roughness.

All of this pushes the disturbed patch toward:

* **Lower effective thermal mass** → heats up and cools down faster.
* Slightly different **emissivity and reflectance** → a few tenths of a degree offset in apparent LWIR temperature.

---

## **3. Thermal signature "simulation" for 2–7 days after burial**

I'll use a very simple energy-balance model, then cross-check against published experimental values.

### **3.1 Parameter choices**

Representative values (within ranges from soil-property literature):

| Parameter | Undisturbed soil | Disturbed soil (after digging) |
|-----------|------------------|--------------------------------|
| Bulk density ρ_b | 1.5 Mg/m³ | 1.2 Mg/m³ (20 % lower) |
| Thermal conductivity λ | 1.5 W/m·K | 1.1 W/m·K (~25 % lower) |
| Volumetric heat capacity C_vol | 2.0 MJ/m³·K | 1.5 MJ/m³·K (~25 % lower) |
| Emissivity ε (LWIR) | 0.94 | 0.96 (finer surface, slightly higher) |

Consider the top **L = 0.10 m** soil layer (this dominates diurnal response). Assume, for simplicity, a net absorbed solar + sensible heat flux of **q ≈ 500 W/m²** for a couple of afternoon hours on a clear day.

Very roughly, temperature rise ΔT over a heating period Δt if conduction downward is slower than the imposed flux:

$$\Delta T \approx \frac{q\,\Delta t}{C_\text{vol}\,L}$$

For **2 hours (7200 s)** of strong heating:

* Undisturbed:  
  $\Delta T_{\text{und}} \approx (500\times7200)/(2.0\times10^6\times0.1) \approx 1.8\ ^\circ\text{C}$

* Disturbed:  
  $\Delta T_{\text{dist}} \approx (500\times7200)/(1.5\times10^6\times0.1) \approx 2.4\ ^\circ\text{C}$

So after a couple of hours of sun, the disturbed patch is about **0.6 °C warmer** purely from lower heat capacity. Reduced λ also means the surface gradient is steeper, which increases peak difference slightly; combined with emissivity effects, you easily reach **≈1 °C of apparent contrast**.

This is consistent with:

* Landmine IR experiments that report **1–3 °C apparent temperature differences** associated with disturbed soil, often largely independent of burial depth.
* ERDC's disturbed-soil vs. "faux target" study, which finds a statistically significant, diurnally varying radiance difference between disturbed areas and background persisting for **weeks** after burial.

So for your "few days after burial" case, **assuming dry, bare soil and at least one sunny day, a 1–2 °C surface anomaly is realistic**.

The **plastic plate at 15 cm** adds very little on top of this. Depth is beyond the ~10 cm limit where the mine's own volume effect creates a significant signature; most of what you see is the disturbed soil, not the plate.

---

## **4. Sensor-level detectability from 2 m**

### **4.1 Thermal camera SNR**

Take MLX90640, 55°×35° FOV version:

* FOV width at 2 m: ≈ 2.1 m ⇒ **6.5 cm per pixel horizontally**.
* FOV height: ≈ 1.3 m ⇒ **5.3 cm per pixel vertically**.
* NETD (noise-equivalent temperature difference): **≈0.1 K RMS at 1 Hz**.

Assume:

* Disturbed patch diameter 0.35 m – realistic for a hand-dug hole.
  * That's roughly **5×6 pixels** of the thermal array.
* Apparent temperature contrast between disturbed patch and background: **ΔT ≈ 1.0–2.0 °C** at best diurnal times.

Then for a single pixel fully on disturbed soil:

* **Signal** S ≈ 1 K (conservative)
* **Noise** N ≈ 0.1 K

So **SNR ≈ 10** for a single frame. Averaging a few frames improves that further.

A cheaper 8×8 AMG8833 array has worse per-pixel accuracy (±2.5 °C absolute error) but still has sub-0.5 °C noise and clearly resolves gradients of 1–2 °C.

**Conclusion:** From 2 m, under good environmental conditions, the thermal disturbance signal is *comfortably above sensor noise* for realistic miniature LWIR modules.

### **4.2 Spatial resolution and localisation**

With ~5–8 hot pixels forming a cluster:

* You can fit an ellipse to the hot region and **localise the centre of the disturbed patch to within ~5–10 cm**, which is good enough to guide a person to the suspected mine location.
* Using a higher-resolution Lepton 3.5 (160×120 px, 57° HFOV) gives GSD ≈ 1.6 cm at 2 m, so one plastic-plate-sized anomaly spans >100 pixels, allowing much finer localisation.

Atmospheric attenuation over 2 m path in 8–14 µm is negligible, so you don't lose contrast through the air.

---

## **5. "Other things" that betray the burial**

You asked to consider *anything* that might indicate a burial, not just the plate itself.

### **5.1 Micro-topography (shape / height)**

Even when organisers "smooth" the soil, a hand-dug and backfilled hole almost always leaves:

* Slight **micro-depressions or mounds** (5–20 mm amplitude).
* Differences in **surface roughness**.

At 2 m with a 60° FOV and a 12-MP RGB camera, pixel size is on the order of 1–2 mm, so:

* These micro-height variations don't look like big holes but they can create **subtle shading differences** if the sun is low.
* Structure-from-motion / stereo from several viewpoints can reconstruct **centimetre-scale height changes**, but that increases flight time and complexity.

In practice, RGB topography can help reject weird shapes (e.g. vehicle ruts, footprints) and focus on patches roughly circular / square and the right size.

### **5.2 Spectral / moisture differences**

Hyperspectral and multispectral studies show disturbed soil often has:

* Different **spectral reflectance curves** due to redistribution of fines and changes in moisture; these differences can last days to weeks.

From a 2 m drone you could, in principle, use:

* Simple **NIR/RGB ratios** (if you have an NIR-sensitive camera) to see slightly darker or lighter soil where porosity and moisture differ.
* But the effect is usually smaller than the thermal signature and more sensitive to illumination angle.

### **5.3 What *isn't* practical from 2 m**

* **GPR / ground EM / resistivity** – require antennas or electrodes close to, or on, the ground; from 2 m with a tiny UAV they're not realistic.
* **Acoustic / seismic methods** – need coupling to the soil; airborne microphones won't see subtle differences in ground stiffness at that distance.

So your realistically usable "remote from 2 m" cues are:

1. Thermal IR disturbance signature.
2. RGB / possibly NIR texture & micro-relief.

---

## **6. Overall feasibility and limitations**

### **6.1 When this works**

The disturbed-soil-detection approach is **physically plausible** and can be very effective if:

* The field is mostly **bare soil** with few natural disturbances.
* Mines are buried **recently (days–weeks)** and the soil has not been compacted by machinery afterwards.
* You fly during **favourable thermal windows**:
  * Shortly **after sunrise** (disturbed soil may warm faster); or
  * In the **late afternoon / early evening** as the patch cools at a different rate.

In that regime, a 32×24 or 160×120 LWIR module on a 2 m-high drone has **very high SNR** for the disturbance signal.

### **6.2 Key failure modes**

Be honest about the weaknesses:

* **Cloudy, windy days** suppress diurnal heating differences → ΔT may drop below ~0.5 °C and approach the clutter level.
* **Heavy rain** can rapidly recompact the surface and wash fine particles, accelerating the decay of the disturbance signature. ERDC saw gradual reduction in edge intensity over weeks, with rain events speeding it up.
* **Other disturbances** (footprints, probe holes, earlier test pits) will also show up as thermal anomalies; you will need size/shape filters and maybe a "no-go region" around paths.
* If organisers *really* over-compact and groom the soil, the difference in density/porosity shrinks; then both thermal and spectral signatures weaken.

---
