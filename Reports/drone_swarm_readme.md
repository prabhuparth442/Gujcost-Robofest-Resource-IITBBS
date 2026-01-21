# Comprehensive Procedure for Master-Slave Drone Swarm Coordination

---

## Overview

This document provides the end-to-end procedure for coordinating two UAVs in a Master-Slave configuration. The Master drone calculates trajectories for both itself and the Slave, transmitting commands over a UDP-based MAVLink network.

---

## Step 1: Hardware Integration

Before software configuration, the physical communication link must be established on both drones.

1. **Companion Computer (CC):** Mount a Raspberry Pi or similar board to each drone.

2. **Telemetry Wiring:** Connect the Flight Controller (FC) `TELEMETRY 2` port to the CC's `GPIO UART` pins.
   - FC TX → CC RX
   - FC RX → CC TX
   - GND → GND

3. **Power:** Ensure the CC is powered by a stable 5V BEC (Battery Elimination Circuit).

---

## Step 2: Flight Controller Parameter Tuning

Using Mission Planner or QGroundControl, set the following parameters on **both** drones to allow external control:

- `SERIAL2_PROTOCOL` = 2 (MAVLink 2)
- `SERIAL2_BAUD` = 921 (921600 baud)
- `COM_RC_IN_MODE` = 1 (Required for Offboard/Guided control if RC is not detected)
- `EK3_SRC1_POSXY` = 3 (Ensure Primary GPS is the source for XY position)

---

## Step 3: Network Infrastructure

Both drones must join the same Local Area Network (LAN).

### Static IP Assignment

On the Master (`192.168.1.10`) and Slave (`192.168.1.11`), modify the network configuration:

```bash
# Edit /etc/dhcpcd.conf
interface wlan0
static ip_address=192.168.1.XX/24
static routers=192.168.1.1
```

---

## Step 4: MAVLink Routing (Bridge)

The Slave drone must "expose" its MAVLink stream so the Master can reach it. Run this on the Slave's CC:

```bash
# Use MAVProxy to bridge Serial to UDP
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:192.168.1.10:14540
```

---

## Step 5: Master Coordination Logic

This Python script runs on the Master Drone and controls both vehicles simultaneously.

```python
# Full Swarm Coordination Script
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

async def run_swarm():
    master = System()
    slave = System()

    # Connect to local Master and remote Slave
    await master.connect(system_address="udp://:14540")
    await slave.connect(system_address="udp://192.168.1.11:14540")

    print("Checking connection...")
    # Verify both systems are online
    async for state in master.core.connection_state():
        if state.is_connected: break
    async for state in slave.core.connection_state():
        if state.is_connected: break

    # Setup: Arm and Takeoff
    print("Arming Swarm...")
    await asyncio.gather(master.action.arm(), slave.action.arm())
    await asyncio.gather(master.action.takeoff(), slave.action.takeoff())
    await asyncio.sleep(10)

    # Enable Offboard for Slave
    await slave.offboard.set_position_ned(PositionNedYaw(0, 0, -2.5, 0))
    try:
        await slave.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # Coordination Loop
    async for telemetry in master.telemetry.position_velocity_ned():
        m_n = telemetry.position.north_m
        m_e = telemetry.position.east_m
        
        # Slave follows 3 meters behind (South) the Master
        target_n = m_n - 3.0
        target_e = m_e
        
        await slave.offboard.set_position_ned(PositionNedYaw(target_n, target_e, -2.5, 0))
        await asyncio.sleep(0.05) # 20Hz Update Rate

if __name__ == "__main__":
    asyncio.run(run_swarm())
```

---

## Step 6: Pre-Flight and Execution

1. **Calibration:** Ensure both drones have calibrated accelerometers and compasses.

2. **Manual Override:** Ensure the RC Transmitter is in "Position Hold" or "Loiter" mode as a safety fallback.

3. **Execution:** Start the Slave bridge first, then the Master script.

---

## Safety Considerations

⚠️ **Warning:** Always test in a controlled environment first. Ensure:
- Adequate GPS signal on both drones
- RC transmitter ready for manual override
- Clear communication between Master and Slave
- Safe flight area with no obstacles

---

## Troubleshooting

### Connection Issues
- Verify both drones are on the same network subnet
- Check firewall settings on companion computers
- Ensure MAVProxy is running on Slave before starting Master script

### Offboard Mode Failures
- Confirm `COM_RC_IN_MODE` parameter is set correctly
- Check that GPS has sufficient fix quality
- Verify telemetry connection is stable

---

## Dependencies

- Python 3.7+
- MAVSDK-Python (`pip install mavsdk`)
- MAVProxy (`pip install MAVProxy`)
- ArduPilot or PX4 firmware on flight controllers

---

## License

[Specify your license here, e.g., MIT, GPL, etc.]

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Contact

For questions or collaboration, please contact [your-email@example.com]