from typing import List, Tuple
import csv
import serial
import struct
import time

HEADER = bytes([0xAA, 0x55])
DATAFMT = '<ff9f'

data: List[List[float]] = []

# # code to generate test data (floats from 0 to 11)
# data_bytes = struct.pack(DATAFMT, *map(float, range(11)))
# boom_pos, height, *imudata = struct.unpack(
#     DATAFMT, data_bytes,
# )

print('connecting... ', end='')
with serial.Serial('/dev/ttyACM0', 250_000) as ser:
    print('connected! Press ctrl+c to exit')
    ser.reset_input_buffer()

    while True:
        try:
            ser.reset_input_buffer()
            ser.read_until(HEADER)
            data_bytes = ser.read(struct.calcsize(DATAFMT))
            height, boom_pos, *imudata = struct.unpack(
                DATAFMT, data_bytes,
            )
            print(f'height = {1000*height:.4f} mm, '
                  f'boom pos = {boom_pos:.2f} m, '
                  f'imudata = {imudata[:6]}')

            data.append([time.time(), height, boom_pos, *imudata])

        except KeyboardInterrupt:
            break

print('disconnected. Writing file...')


def fmt(s: str):
    return [s.format(ax) for ax in 'xyz']


imu_header = \
    fmt('accel {} [m/s^2]') + \
    fmt('gyro {} [rad/s]') + \
    fmt('mag {} [uT]')

with open('data/height-data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['time', 'height [mm]', 'boom pos [m]'] + imu_header)
    writer.writerows(data)

if len(data) > 0:
    import matplotlib.pyplot as plt
    plt.style.use('seaborn')
    plt.title('Height data [mm]')
    plt.plot([1000*d[0] for d in data])
    plt.show()
