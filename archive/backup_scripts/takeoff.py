import asyncio
from mavsdk import System

async def run():
    # 1. Connect to the Drone
    # If using USB: system_address="serial:///dev/ttyACM0:57600"
    # If using GPIO (TELEMETRY port): system_address="serial:///dev/ttyAMA0:57600"
    # Note: Change 57600 to 921600 if you configured your Pixhawk that way.
    
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    # 2. Check Connection
    print("Waiting for drone state...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # 3. Check GPS Health (Safety First!)
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # 4. Arm and Takeoff
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
