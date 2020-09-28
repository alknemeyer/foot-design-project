# # header:
# opt_ser.write((170, 0, 50, 3, 1, 0, 255, 1, 223))

# if opt_ser.inWaiting() >= 16:
#     temp_opt = opt_ser.readline()
# else:
#     continue

# if len(temp_opt) == 16:
#     outfile.write(datetime.now().strftime('%H:%M:%S:%f,'))

#     fx = int.from_bytes(
#         temp_opt[4:6], byteorder='big', signed=True) * FX_SCALE
#     fy = int.from_bytes(
#         temp_opt[6:8], byteorder='big', signed=True) * FY_SCALE
#     fz = int.from_bytes(
#         temp_opt[8:10], byteorder='big', signed=True) * FZ_SCALE
#     outfile.write(f'{fx},{fy},{fz}\n')