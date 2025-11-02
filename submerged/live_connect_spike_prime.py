import asyncio
from bleak import BleakClient

# Replace with your hub's Bluetooth address
hub_address = "XX:XX:XX:XX:XX:XX"  

# UUIDs for the LEGO Wireless Protocol (LWP3) services
UART_SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123"
UART_CHAR_UUID = "00001624-1212-efde-1623-785feabcd123"

# Commands to control motors (examples only, you may need to adjust)
MOVE_FORWARD = bytes([0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x00, 0x64])  # Move motor A forward at speed 100
TURN_LEFT = bytes([0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x00, 0x32])  # Turn left command example

async def send_command(command):
    async with BleakClient(hub_address) as client:
        await client.write_gatt_char(UART_CHAR_UUID, command)
        print("Command sent:", command)

async def main():
    await send_command(MOVE_FORWARD)  # Example: Move forward
    await asyncio.sleep(1)
    await send_command(TURN_LEFT)     # Example: Turn left

# Run the main function
asyncio.run(main())
