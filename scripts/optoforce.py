"""
Run as follows:

    $ python scripts/optoforce.py

Afterwards, use `plot_optoforce.py` to visualize data

Force scale parameters are from:
    `OptoForce/Sensor datasheet/SensitivityReport-PFH0A052.pdf`

Comms stuff is from:
    `OpfoForce/Interfaces and Softwares/USB/_Protocol description/OptoForce General DAQ - USB,CAN,UART - v1.7.pdf`

If you get permission errors when trying to open the serial port, run:

    $ sudo chmod 666 /dev/ttyACM0

From: https://stackoverflow.com/questions/27858041/oserror-errno-13-permission-denied-dev-ttyacm0-using-pyserial-from-pyth
"""
# https://pyserial.readthedocs.io/en/latest/shortintro.html
import serial
from serial.tools.list_ports import comports
import struct
from datetime import datetime
import time

# constants
OPTO_PARAMS = {
    'baudrate': 1000_000,
    'stopbits': serial.STOPBITS_ONE,
    'parity': serial.PARITY_NONE,
    'bytesize': serial.EIGHTBITS,
}

FX_SCALE = 1/7925 * 300   # ~= 1/26.42
FY_SCALE = 1/8641 * 300   # ~= 1/28.8
FZ_SCALE = 1/6049 * 2000  # ~= 1/3.02

# get optoforce port
ports = comports()
for dev in ports:
    if dev.description == 'OptoForce DAQ':
        break
else:
    raise RuntimeError(f"Couldn't find the optoforce! Got: {ports}")

print('Connecting to optoforce...')
with open('data/raw-opto-log.csv', 'w+') as outfile, \
        serial.Serial(dev.device, **OPTO_PARAMS) as opt_ser:

    print('Writing optoforce setup code...')
    header = (170, 0, 50, 3)
    speed = 1   # 1 = 1000 Hz, 10 = 100 Hz
    filter = 0  # no filter
    zero = 255
    checksum = sum(header) + speed + filter + zero
    payload = (*header, speed, filter, zero, *divmod(checksum, 256))
    opt_ser.write(payload)

    # writing header for csv
    # outfile.write('time [H:M:S:f],Fx [N],Fy [N],Fz [N]\n')
    outfile.write('time,Fx [N],Fy [N],Fz [N]\n')

    print("Logging - press ctrl-c to stop...")

    opt_ser.flushInput()
    expected_header = bytes((170, 7, 8, 10))  # => b'\xaa\x07\x08\n'

    while True:
        try:
            opt_ser.read_until(expected_header)

            # https://docs.python.org/3/library/struct.html#format-characters
            count, status, fx, fy, fz, checksum = (
                struct.unpack('>HHhhhH', opt_ser.read(12))
            )
            fx *= FX_SCALE
            fy *= FY_SCALE
            fz *= FZ_SCALE
            # print(f'count={count:2}, status={status:2}, fx={fx: 8.3f} fy={fy: 8.3f} fz={fz: 8.3f}')

            # t = datetime.now().strftime('%H:%M:%S:%f')
            # outfile.write(f'{t},{fx},{fy},{fz}\n')
            outfile.write(f'{time.time()},{fx},{fy},{fz}\n')
        except KeyboardInterrupt:
            break
