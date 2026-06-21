import asyncio
from bleak import BleakClient

# Use your specific address
ADDRESS = "30:76:F5:B9:B7:C6"
# These MUST match your ESP32 code
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

def notification_handler(sender, data):
    try:
        decoded_data = data.decode('utf-8')
        vals = decoded_data.split(',')
        if len(vals) >= 6:
            print(f"Acc: {vals[0]:>6}, {vals[1]:>6}, {vals[2]:>6} | "
                  f"Angle: {vals[3]:>6}, {vals[4]:>6}, {vals[5]:>6}")
    except Exception as e:
        print(f"Decode error: {e}")

async def run():
    print(f"Connecting to {ADDRESS}...")
    
    # attempt connection
    try:
        async with BleakClient(ADDRESS) as client:
            if client.is_connected:
                print(f"Connected! Service Discovery started...")
                
                # Start notifications
                await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
                
                print("Receiving data... (Ctrl+C to stop)")
                while True:
                    await asyncio.sleep(1)
            else:
                print("Failed to connect.")
    except Exception as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    asyncio.run(run())

# import asyncio
# from bleak import BleakClient, BleakScanner

# # This MUST match the UUIDs in your Arduino code
# SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
# CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
# DEVICE_NAME = "ESP32_IMU_BLE"

# def notification_handler(sender, data):
#     # Decode the byte array to string
#     decoded_data = data.decode('utf-8')
#     # Split the CSV values
#     vals = decoded_data.split(',')
    
#     if len(vals) == 6:
#         print(f"Acc: {vals[0]:>6}, {vals[1]:>6}, {vals[2]:>6} | "
#               f"Angle: {vals[3]:>6}, {vals[4]:>6}, {vals[5]:>6}")

# async def run():
#     print(f"Searching for {DEVICE_NAME}...")
#     device = await BleakScanner.find_device_by_filter(
#         lambda d, ad: d.name and d.name == DEVICE_NAME
#     )

#     if not device:
#         print("Device not found.")
#         return

#     print(f"Connected to {device.address}")
#     async with BleakClient(device) as client:
#         # Start receiving notifications
#         await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        
#         print("Receiving data... (Ctrl+C to stop)")
#         while True:
#             await asyncio.sleep(1)

# if __name__ == "__main__":
#     try:
#         asyncio.run(run())
#     except KeyboardInterrupt:
#         print("\nStopped.")