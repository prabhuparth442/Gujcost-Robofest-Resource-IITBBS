import asyncio
from mavsdk import System

async def run():
    # 1. Connect via USB
    # /dev/ttyACM0 is the standard port for Pixhawk via USB on Linux
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    # 2. Confirm Connection
    print("Waiting for drone state...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to Drone!")
            break

    # 3. Print Telemetry (Proof that data is flowing)
    print("Fetching battery status...")
    async for battery in drone.telemetry.battery():
        print(f"-- Battery Voltage: {battery.voltage_v}V")
        print(f"-- Battery Remaining: {battery.remaining_percent * 100}%")
        break

    # 4. Attempt to Arm (Indoor Test)
    print("-- Arming Motors...")
    try:
        await drone.action.arm()
        print("-- ARMING SUCCESS! Motors should be spinning.")
        
        # Wait 5 seconds to let you see them spin
        await asyncio.sleep(10)
        
        print("-- Disarming...")
        await drone.action.disarm()

    except Exception as e:
        print("\n!!! ARMING FAILED !!!")
        print(f"Error: {e}")
        print("\nPOSSIBLE FIXES for Indoors:")
        print("1. Your 'Pre-Arm Safety Checks' are blocking it because you have no GPS.")
        print("2. Connect QGroundControl/Mission Planner -> Safety -> Disable 'GPS Lock' check.")
        print("3. Or switch Flight Mode to 'STABILIZED' manually via RC controller before running script.")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
