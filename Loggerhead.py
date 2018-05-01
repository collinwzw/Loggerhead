import serial
import time
import math
from math import log10, floor

ser = serial.Serial('/dev/cu.usbmodemFD121',9600)
count=0
gyro_x = 0
gyro_y = 0
gyro_z = 0
acc_x  = 0
acc_y  = 0
acc_z  = 0

while True:
    read_serial= ser.readline()
    if read_serial[0] != 'm':
        x = read_serial
 #       print (x.strip())
        
        gyro_x += (float(x[0:7]))/100
        gyro_y += (float(x[7:15]))/100
        gyro_z += (float(x[15:22]))/100
        acc_x  += (float(x[22:29]))/100
        acc_y  += (float(x[29:36]))/100
        acc_z  += (float(x[36:43]))/100
        count = count + 1

    if count == 5:
          
          gyro_x_avg = gyro_x/count
          f"{gyro_x_avg:{3}.{2}}"
          gyro_y_avg = gyro_y/count
          format(gyro_y_avg,'.2f')
          gyro_z_avg= gyro_z/count
          format(gyro_z_avg,'.2f')
          acc_x_avg= acc_x/count
          format(acc_x_avg,'.2f')
          acc_y_avg= acc_y/count
          format(acc_y_avg,'.2f')
          acc_z_avg= acc_z/count
          format(acc_z_avg,'.2f')
          
          print (gyro_x_avg,gyro_y_avg,gyro_z_avg,acc_x_avg,acc_y_avg,acc_z_avg)

          roll = math.atan2(acc_x_avg,acc_z_avg)
          format(roll,'.2f')
          pitch = math.atan2(acc_y_avg,acc_z_avg)
          format(pitch,'.2f')
          ser.write(b'm1090100\0')
          gyro_x = 0
          gyro_y = 0
          gyro_z = 0
          acc_x  = 0
          acc_y  = 0
          acc_z  = 0
          #read_serial= ser.readline()
          #print (read_serial)
 #         motor = []
 #         motor = [0,30,20]
 #         ser.write(motor)
#          speed = 20 #ms delay
#          ser.write(b'speed')
          count = 0
