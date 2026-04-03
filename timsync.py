import serial
import time
from datetime import datetime

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)

    now = datetime.now()
    sync_string = now.strftime("SYNC:%Y,%m,%d,%H,%M,%S\n")

    # Send it over serial
    print(f"Sending: {sync_string.strip()}")
    ser.write(sync_string.encode('utf-8'))

    # Read the response
    time.sleep(0.5)
    while ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        print(f"FED3 says: {response}")

    ser.close()

except Exception as e:
    print(f"Error: {e}")
