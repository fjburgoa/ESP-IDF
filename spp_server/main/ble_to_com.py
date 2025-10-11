import asyncio
from bleak import BleakScanner, BleakClient

BLE_DEVICE_NAME = "nimble-ble-spp-srv"

async def run():
    devices = await BleakScanner.discover()
    target = None
    for d in devices:
        if BLE_DEVICE_NAME in d.name:
            target = d
            break

    if target is None:
        print("Device not found during scan.")
        return

    # Use the BleakScanner device object directly
    async with BleakClient(target) as client:
        print(f"Connected to {target.name}!")
        for service in client.services:
            print(service)
            for char in service.characteristics:
                print("  Char UUID:", char.uuid)
