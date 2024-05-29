import time
import serial
import array
import numpy as np

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM13',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

ser.write(bytes("rq", "ascii"))
out = b''
# Give the device time to respond (can probably be shortened)
time.sleep(10)
while ser.in_waiting > 0:
    out += ser.read(1)
    
if out != '':
    print(f"captured {len(out)} bytes")
    data = array.array('H',out) # H
    print(data)
    arr_to_save = np.asarray(data)
    np.savetxt("foo.csv", arr_to_save, delimiter=",", fmt="%d")