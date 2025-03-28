#!/usr/bin/env python3

import time, serial

try:
    ser = serial.Serial()
    ser.port = '/dev/ttyACM1'
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS
    ser.parity =serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 1
    ser.open()

    # Write commands into dwm1001c module to read data.
    # Necessary for the first time. After that, cutecom can show uwb data.
    ser.write(b'\r')
    time.sleep(0.5)
    ser.write(b'\r')
    time.sleep(0.5)
    ser.write(b'\r') # in case
    time.sleep(0.5)
    ser.write(b'lec\r')

    ser.close()

except Exception as e:
    print(e)
    pass
print(ser)

ser.open()

while True:
    try:
        serialData = ser.read_until()
        # data=str(ser.readline())
        data = [x.strip() for x in serialData.strip().split(b',')]

        if b"DIST" in data[0]:
            print("Raw data: ", data)

            # The number of elements should be 2 + 6*NUMBER_OF_ANCHORS + 5 (TAG POS)
            number_of_anchors = int((len(data) - 7)/6)

            for i in range(number_of_anchors):
                id = str(data[2 + 6*i], 'UTF8')
                dist = float(data[7 + 6*i])
                print("anchor_id: ", id, ", distance: ", dist)
                # print("\n distance: ", float(data[7+6]))

        time.sleep(0.01)

    except Exception as e:
        print(e)
        pass
    except KeyboardInterrupt:
        ser.close()