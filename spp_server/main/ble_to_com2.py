import asyncio
from bleak import BleakClient, BleakScanner
import serial

# BLE target
BLE_DEVICE_NAME = "nimble-s3"
# Correct characteristic UUID from your ESP32 code
SPP_CHAR_UUID = "10325476-98ba-dcfe-1032-547698badcfe"

# Local virtual COM port
SERIAL_PORT = "COM3"   # one end of the com0com pair
BAUDRATE = 115200


async def run():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    target_device = None
    for d in devices:
        if d.name and BLE_DEVICE_NAME in d.name:
            target_device = d
            break

    if target_device is None:
        print(f"❌ Device {BLE_DEVICE_NAME} not found!")
        return

    print(f"🔗 Connecting to {target_device.name} ({target_device.address})...")
    async with BleakClient(target_device.address) as client:
        print("✅ Connected!")

        # Open serial port
        ser = serial.Serial(SERIAL_PORT, BAUDRATE)
        print(f"📡 Forwarding BLE → {SERIAL_PORT}...")

        def notification_handler(sender, data):
            """Forward BLE data to serial port"""
            ser.write(data)
            print(f"➡️  Received {len(data)} bytes, sent to {SERIAL_PORT}: {data}")

        # Subscribe to notifications
        await client.start_notify(SPP_CHAR_UUID, notification_handler)

        print("Press Ctrl+C to stop...")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Stopping...")
            await client.stop_notify(SPP_CHAR_UUID)
            ser.close()
            print("🔒 Serial port closed.")


if __name__ == "__main__":
    asyncio.run(run())
