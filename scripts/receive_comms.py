from typing import List, Tuple
import csv
import serial
import struct

HEADER = bytes([0xAA, 0x55])
DATAFMT = '<ff'

data: List[Tuple[float, float]] = []

print('connecting... ', end='')
with serial.Serial('/dev/ttyACM0', 250_000) as ser:
    print('connected! Press ctrl+c to exit')
    ser.reset_input_buffer()

    while True:
        try:
            ser.reset_input_buffer()
            ser.read_until(HEADER)
            data_bytes = ser.read(struct.calcsize(DATAFMT))
            height_m, boom_pos_m = struct.unpack(
                DATAFMT, data_bytes,
            )
            print(f'height = {1000*height_m:.4f} mm, boom pos = {boom_pos_m:.2f} m')

            data.append((height_m, boom_pos_m))

        except KeyboardInterrupt:
            break

import matplotlib.pyplot as plt
plt.style.use('seaborn')
plt.title('Height data [mm]')
plt.plot([1000*d[0] for d in data])
plt.show()


# print('disconnected. Writing file...')

# with open('log.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(['global_yaw', 'global_pitch'])
#     writer.writerows(data)
