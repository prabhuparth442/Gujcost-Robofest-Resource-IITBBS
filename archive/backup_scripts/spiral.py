import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- CONFIGURATION ---
CONNECTION_STRING = "serial:///dev/ttyACM0:115200"  # Matches your simulator port
TAKEOFF_ALTITUDE = 8.0
SPEED_MPS = 5.0

# Spiral Settings
MAX_RADIUS = 12.0     # How wide the spiral gets (meters)
LOOPS = 3             # How many full circles to spin
POINTS_PER_LOOP = 8  # Smoothness (higher = smoother circle)

async def run():
    drone = System()
    print(f"-- Connecting to {CONNECTION_STRING}...")
    await drone.connect(system_address=CONNECTION_STRING)

    print("-- Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    print("-- Arming")
    await drone.action.arm()

    print(f"-- Taking off to {TAKEOFF_ALTITUDE}m")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.takeoff()
    
    # Wait for altitude
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > TAKEOFF_ALTITUDE * 0.95:
            print("-- Altitude Reached")
            break
        await asyncio.sleep(0.5)
    
    await asyncio.sleep(2) # Stabilize

    # --- START OFFBOARD MODE ---
    print("-- Initializing Offboard Mode...")
    initial_point = PositionNedYaw(0.0, 0.0, -TAKEOFF_ALTITUDE, 0.0)
    await drone.offboard.set_position_ned(initial_point)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"!! Offboard failed: {error._result.result}")
        await drone.action.land()
        return

    # --- SPIRAL LOGIC ---
    print(f"-- Starting Spiral: {LOOPS} loops, max radius {MAX_RADIUS}m")
    
    total_points = LOOPS * POINTS_PER_LOOP
    
    for i in range(total_points):
        # Calculate percentage of mission complete (0.0 to 1.0)
        progress = i / total_points
        
        # Current Radius grows as we progress
        current_radius = progress * MAX_RADIUS
        
        # Current Angle (in radians)
        # 2 * PI is one full circle. We multiply by LOOPS.
        angle = progress * (2 * math.pi * LOOPS)
        
        # Calculate X (North) and Y (East)
        x = current_radius * math.cos(angle)
        y = current_radius * math.sin(angle)
        
        # Calculate Yaw to face the direction of travel (optional, looks cool)
        # Adding 90 degrees (pi/2) makes it face the tangent
        yaw_deg = math.degrees(angle + (math.pi/2)) 
        
        print(f" > Pt {i+1}/{total_points}: R={current_radius:.1f}m | x={x:.1f}, y={y:.1f}")
        
        await drone.offboard.set_position_ned(
            PositionNedYaw(x, y, -TAKEOFF_ALTITUDE, yaw_deg))
        
        # Wait time determines speed. 
        # Shorter sleep = faster drone.
        await asyncio.sleep(0.5) 

    # --- RETURN HOME ---
    print("-- Spiral Complete. Returning Home...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -TAKEOFF_ALTITUDE, 0.0))
    await asyncio.sleep(5)

    print("-- Landing")
    try:
        await drone.offboard.stop()
    except:
        pass # Ignore error if already stopped
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
