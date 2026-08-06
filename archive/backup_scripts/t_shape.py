import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- CONFIGURATION ---
# CRITICAL FIX: Changed from 14540 to 14551 to match your Pymavlink setup
CONNECTION_STRING = "serial:///dev/ttyACM0:115200" 

TAKEOFF_ALTITUDE = 5.0
DISTANCE_D = 5.0  # Distance "d"
SPEED_MPS = 5.0

async def run():
    drone = System()
    print(f"-- Connecting to {CONNECTION_STRING}...")
    
    # Connect to the specific port
    await drone.connect(system_address=CONNECTION_STRING)

    # 1. Connection Check
    print("-- Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to Drone!")
            break

    # 2. GPS Health Check (Required for Offboard/Guided)
    print("-- Checking GPS Health...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- GPS Ready.")
            break
        else:
            print(f"-- Waiting for GPS... (Global: {health.is_global_position_ok}, Home: {health.is_home_position_ok})", end='\r')
            await asyncio.sleep(1)

    # 3. Arm and Takeoff
    print("-- Arming")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"!! Arming Failed: {e}")
        return

    print(f"-- Taking off to {TAKEOFF_ALTITUDE}m")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.takeoff()

    # Wait for altitude
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > TAKEOFF_ALTITUDE * 0.95:
            print("-- Reached Altitude")
            break
        await asyncio.sleep(0.5)

    await asyncio.sleep(2) # Stabilize

    # 4. Initialize Offboard Mode (GUIDED)
    # ArduPilot requires a stream of setpoints BEFORE switching modes
    print("-- Initializing Offboard Mode...")
    initial_point = PositionNedYaw(0.0, 0.0, -TAKEOFF_ALTITUDE, 0.0)
    await drone.offboard.set_position_ned(initial_point)

    try:
        await drone.offboard.start()
        print("-- Offboard Mode Started")
    except OffboardError as error:
        print(f"!! Starting Offboard mode failed: {error._result.result}")
        print("!! Ensure your drone has GPS lock and is not in failsafe.")
        await drone.action.land()
        return

    # --- T-SHAPE FLIGHT LOGIC ---
    
    # Calculate travel times
    flight_time_d = DISTANCE_D / SPEED_MPS
    flight_time_2d = (2 * DISTANCE_D) / SPEED_MPS
    buffer = 4.0 # Give extra time for acceleration/deceleration

    # Leg 1: North (d)
    print(f"-- Moving North {DISTANCE_D}m")
    await drone.offboard.set_position_ned(
        PositionNedYaw(DISTANCE_D, 0.0, -TAKEOFF_ALTITUDE, 0.0))
    await asyncio.sleep(flight_time_d + buffer)

    # Leg 2: South (2d) -> From +d to -d is a distance of 2d
    print(f"-- Moving South {2 * DISTANCE_D}m")
    await drone.offboard.set_position_ned(
        PositionNedYaw(-DISTANCE_D, 0.0, -TAKEOFF_ALTITUDE, 0.0))
    await asyncio.sleep(flight_time_2d + buffer)

    # Leg 3: North (d) -> Return to 0
    print(f"-- Moving North {DISTANCE_D}m (Returning Home)")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -TAKEOFF_ALTITUDE, 0.0))
    await asyncio.sleep(flight_time_d + buffer)

    # 5. Land
    print("-- Stopping Offboard Mode")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
