import asyncio
from mavsdk import System

async def run():
    # --- CONFIGURATION ---
    # Real Drone Connection
    CONN_STRING = "serial:///dev/ttyACM0:115200"
    TARGET_ALTITUDE = 3.0
    HOVER_TIME = 20
    # ---------------------

    drone = System()
    print(f"Connecting to drone on {CONN_STRING}...")
    await drone.connect(system_address=CONN_STRING)

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    # 1. PRE-FLIGHT CHECKS
    print("Checking GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- GPS Position OK")
            break

    # 2. ARM AND TAKEOFF
    print("-- Arming")
    await drone.action.arm()

    print(f"-- Taking off to {TARGET_ALTITUDE}m")
    await drone.action.set_takeoff_altitude(TARGET_ALTITUDE)
    await drone.action.takeoff()

    # 3. HOVER
    print(f"-- Hovering for {HOVER_TIME} seconds...")
    await asyncio.sleep(HOVER_TIME)

    # 4. LAND
    print("-- Landing...")
    await drone.action.land()

    # 5. WAIT FOR LANDING
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed!")
            break
            
    # 6. DISARM
    print("-- Disarming")
    try:
        await drone.action.disarm()
    except:
        pass

if __name__ == "__main__":
    asyncio.run(run())
