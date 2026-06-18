import serial

port = "COM5"  # change this
baud = 115200

ser = serial.Serial(port, baud, timeout=1)

print("Listening...")

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print("Received:", line)

except KeyboardInterrupt:
    print("Stopped")
    ser.close()