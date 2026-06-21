import socket

# Use your device's Bluetooth MAC address
server_mac = '30:76:F5:B9:B7:C6' 
port = 1  # Standard RFCOMM port

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
try:
    s.connect((server_mac, port))
    print("Connected successfully.")
    while True:
        data = s.recv(1024)
        if data:
            print(f"Received: {data.decode('utf-8')}")
except Exception as e:
    print(f"Connection failed: {e}")
finally:
    s.close()
