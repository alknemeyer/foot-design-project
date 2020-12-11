from typing import List, NamedTuple
import csv
import serial
import struct
import time


class Packet(NamedTuple):
    time: float
    height_m: float
    # boom_pos_m: float
    boom_encoder_val: int
    accel_x_mss: float
    accel_y_mss: float
    accel_z_mss: float
    gyro_x_rads: float
    gyro_y_rads: float
    gyro_z_rads: float
    mag_x_uT: float
    mag_y_uT: float
    mag_z_uT: float


HEADER = bytes([0xAA, 0x55])
# DATAFMT = '<ff9f'
DATAFMT = '<if9f'

data: List[Packet] = []

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
            boom_pos, height, *imudata = struct.unpack(
                DATAFMT, data_bytes,
            )
            acx, acy, acz = imudata[:3]  # accelerations
            print(f'height = {height:3.0f} mm, '
                  f'boom pos = {boom_pos:4.3f} m, '
                  f'imu acceleration = {acx:3.1f} {acy:3.1f} {acz:3.1f}')

            data.append(Packet(time.time(), height/1000, boom_pos, *imudata))

        except KeyboardInterrupt:
            break


def fmt(s: str):
    return [s.format(ax) for ax in 'xyz']


imu_header = \
    fmt('accel {} [m/s^2]') + \
    fmt('gyro {} [rad/s]') + \
    fmt('mag {} [uT]')


if len(data) > 0:
    print('Writing file...')
    with open('data/height-data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['time', 'height [mm]', 'boom pos [m]'] + imu_header)
        writer.writerows(data)

    print('Plotting...')
    import matplotlib.pyplot as plt
    plt.style.use('seaborn')
    fig, (ax0, ax1, ax2) = plt.subplots(3, 1)
    ax0.plot([d.height_m*1000 for d in data], label='height [mm]')
    ax0.legend()

    # ax1.plot([d.boom_pos_m for d in data], label='yaw [m]')
    ax1.plot([d.boom_encoder_val for d in data], label='encoder count')
    ax1.legend()

    ax2.plot([d.accel_x_mss for d in data], label='accel-x')
    ax2.plot([d.accel_y_mss for d in data], label='accel-y')
    ax2.plot([d.accel_z_mss for d in data], label='accel-z')
    ax2.legend()
    plt.show()
