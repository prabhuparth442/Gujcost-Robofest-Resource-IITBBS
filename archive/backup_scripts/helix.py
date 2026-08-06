import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- CONFIGURATION ---
CONNECTION_STRING = "serial:///dev/ttyACM0:115200"
RADIUS = 5.0          # Radius of the circle (meters)
START_ALTITUDE = 3.0  # Start height
END_ALTITUDE = 10.0   # Top height
LOOPS = 3             # How many turns to make while climbing
SPEED_DELAY = 0.2     # Speed of update

async def run():
    drone = System()
    print(f"-- Connecting to {CONNECTION_STRING}...")
    await drone.connect(system_address=CONNECTION_STRING)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    print("-- Arming & Taking Off")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(START_ALTITUDE)
    await drone.action.takeoff()
    
    # Wait for takeoff
    await asyncio.sleep(5)

    # --- CORKSCREW MODE ---
    print("-- Starting Corkscrew Ascent...")
    
    # 1. Move to the edge of the circle first
    # (So we don't slash through the center)
    print("-- Moving to start position")
    start_pos = PositionNedYaw(RADIUS, 0.0, -START_ALTITUDE, 0.0)
    await drone.offboard.set_position_ned(start_pos)
    
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # 2. Calculate the Helix
    # Total vertical distance to climb
    climb_height = END_ALTITUDE - START_ALTITUDE
    
    # Resolution (points per loop)
    points_per_loop = 30
    total_points = LOOPS * points_per_loop
    
    for i in range(total_points):
        progress = i / total_points
        
        # ANGLE: 0 to 360 * LOOPS
        angle = progress * (2 * math.pi * LOOPS)
        
        # ALTITUDE: Linear interpolation from Start to End
        # Remember: Z is negative for UP
        current_alt = START_ALTITUDE + (climb_height * progress)
        z_down = -current_alt
        
        # POSITION: Standard Circle Math
        x = RADIUS * math.cos(angle)
        y = RADIUS * math.sin(angle)
        
        # YAW: Face the center (Classic "Orbit" look)
        # To face center, Yaw = Angle + 180 degrees
        yaw_deg = math.degrees(angle) + 180
        # Normalize to 0-360
        yaw_deg = yaw_deg % 360

        print(f" > Helix: Alt {current_alt:.1f}m | Angle {math.degrees(angle)%360:.0f}")
        
        await drone.offboard.set_position_ned(
            PositionNedYaw(x, y, z_down, yaw_deg))
        
        await asyncio.sleep(SPEED_DELAY)

    # --- HOVER AT TOP ---
    print("-- Reached Top! Enjoy the view.")
    await asyncio.sleep(5)

    # --- DESCEND STRAIGHT DOWN ---
    print("-- Descending through the center...")
    # Move to center (0,0) at top altitude
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -END_ALTITUDE, 0.0))
    await asyncio.sleep(4)
    
    # Land
    print("-- Landing")
    try:
        await drone.offboard.stop()
    except:
        pass
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
