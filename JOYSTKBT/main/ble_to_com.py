import asyncio
from bleak import BleakClient, BleakScanner
import serial

# Adjust these:
DEVICE_NAME = "ESP32S3_HELLO"   # must match what your ESP32 advertises
COM_PORT = "COM9"             # virtual COM port for HyperTerminal
BAUDRATE = 115200

# NUS UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

async def main():
    # Scan for devices
    devices = await BleakScanner.discover()
    esp32 = next(d for d in devices if d.name == DEVICE_NAME)
    print(f"Connecting to {esp32.name} [{esp32.address}]")

    ser = serial.Serial(COM_PORT, BAUDRATE)

    def handle_rx(_, data: bytearray):
        text = data.decode(errors="ignore")
        print(f"BLE → {text}")
        ser.write(text.encode())

    async with BleakClient(esp32.address) as client:
        print("Connected, subscribing to TX notifications...")
        await client.start_notify(NUS_TX_UUID, handle_rx)

        print("Bridge running. Open HyperTerminal on", COM_PORT)
        while True:
            await asyncio.sleep(1)

asyncio.run(main())