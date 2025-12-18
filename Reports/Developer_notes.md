# SITL Simulation: Final Mathematical Specification
**Objective:** Physics-accurate generation of thermal signatures for buried mines and surface clutter.

---

## 1. The Core 3D Diffusion Engine
For every voxel $(i, j, k)$ that is NOT on the surface, use the **Energy Balance Method**:

$$\rho C_p \frac{\Delta V}{\Delta t} (T_{new} - T_{old}) = \sum Q_{neighbors}$$

Simplified for a uniform grid (Finite Difference):
$$T_{i,j,k}^{t+1} = T_{i,j,k}^t + \frac{\Delta t}{\rho C_p} \left[ \frac{k_x \Delta T_x}{\Delta x^2} + \frac{k_y \Delta T_y}{\Delta y^2} + \frac{k_z \Delta T_z}{\Delta z^2} \right]$$

* **Logic:** This allows you to have different $k$ values for a "Mine Voxel" vs a "Soil Voxel." If the current voxel is a mine, use $k_{mine}$ and $(\rho C_p)_{mine}$.



---

## 2. Surface Node Equations ($z=0$)
The temperature of the surface voxels is determined by the **Surface Energy Balance**. This is the most critical part for your drone's "camera" view.

$$Q_{net} = Q_{sun} - Q_{conv} - Q_{rad} - Q_{evap}$$

### 2.1 Solar Input ($Q_{sun}$)
$$Q_{sun} = (1 - \text{Albedo}) \cdot I_{solar} \cdot \cos(\theta)$$
* **Soil Albedo:** $\approx 0.2 - 0.3$
* **Rock Albedo:** $\approx 0.1 - 0.2$ (Rocks absorb more because they are darker).

### 2.2 Convective Cooling ($Q_{conv}$)
$$Q_{conv} = h \cdot (T_{surf} - T_{air})$$
* **$h$ (Heat Transfer Coeff):** $h = 5.7 + 3.8 \cdot v_{wind}$ (where $v$ is wind speed in m/s).

### 2.3 Sky Radiation ($Q_{rad}$)
$$Q_{rad} = \epsilon \sigma (T_{surf}^4 - T_{sky}^4)$$
* **Note:** $T_{sky}$ is usually $10^\circ C$ to $20^\circ C$ colder than $T_{air}$.

### 2.4 Evaporation ($Q_{evap}$) - Optional but Accurate
If you want to simulate **Wet Soil/Moisture Patches**:
$$Q_{evap} = L_v \cdot E$$
where $L_v$ is latent heat and $E$ is evaporation rate. This is why wet patches look like "Cold Spots."

---

## 3. Modeling Obstruction Signatures

| Feature | Equation Logic | Visual Result |
| :--- | :--- | :--- |
| **Buried Mine** | $k_{mine} \ll k_{soil}$ | **Reflective Boundary:** Heat "piles up" in the soil layer above the mine. Surface becomes a **Warm Blob**. |
| **Buried Rock** | $k_{rock} \gg k_{soil}$ | **Conductive Sink:** Heat is "wicked" away into the deep earth. Surface becomes a **Cold Blob**. |
| **Surface Rock** | $\text{Albedo}_{rock} < \text{Albedo}_{soil}$ | **Absorption Dominant:** Higher solar intake + high thermal mass = **Extreme Hot Spot**. |
| **Hollow Cavity** | $k_{air} \approx 0.026$ | **Insulator:** Even more extreme thermal block than plastic. Very high contrast. |



---

## 4. Stability & Simulation Parameters
To prevent your code from crashing (numerical instability):

1.  **Time Step ($dt$):** Must satisfy $\Delta t < \frac{\rho C_p \Delta z^2}{2k}$. For $1cm$ voxels, $dt = 1s$ is usually safe.
2.  **Convergence:** Run the simulation for **2 full diurnal cycles** (48 hours) before you start "flying" your drone. This ensures the deep-earth temperature is realistic.
3.  **The "Detector":** Your IR camera pixel value at $(x,y)$ is simply $T(x,y, z=0)$.
