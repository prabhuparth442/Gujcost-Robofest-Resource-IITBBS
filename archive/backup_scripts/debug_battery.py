import asyncio
from mavsdk import System

async def run():
    drone = System()
    print("Connecting to drone...")
    # Using the standard USB port
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    print("\n--- LISTENING FOR BATTERY DATA (Press Ctrl+C to stop) ---")
    print("If nothing prints below this line in 5 seconds, your Pixhawk is NOT sending battery data.")
    
    # We use a loop that prints EVERY update to see if it fluctuates
    async for battery in drone.telemetry.battery():
        print(f"Update Received -> Voltage: {battery.voltage_v:.2f}V | Remaining: {battery.remaining_percent * 100:.0f}%")

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("Stopping...")
