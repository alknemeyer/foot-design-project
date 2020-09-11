from typing import List
import csv, numpy as np, serial, time

data: List[List[float]] = []

with serial.Serial('/dev/ttyACM1', 115200, timeout=1) as ser:
    print('waiting for start', end='')
    while ser.readline().decode().rstrip() != 'global_yaw,global_pitch':
        print('.', end='')
        time.sleep(0.5)

    print(' connected!')

    while (line := ser.readline().decode().rstrip()) != 'END LOGGING':
        line = line.split(',')
        yaw = float(line[0])
        pitch = float(line[1])
        data.append([yaw, pitch])

with open('log.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['global_yaw', 'global_pitch'])
    writer.writerows(data)

print('File contents:')
print(np.array(data))
