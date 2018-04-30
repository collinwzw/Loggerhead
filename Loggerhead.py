import serial
import time

ser = serial.Serial('/dev/cu.usbmodemFD121',9600)
count=0

while True:
    read_serial= ser.readline()
    if read_serial[0] != 'm':
        x = read_serial
 #       print (x.strip())
        
        gyro_x = (float(x[0:7]))/100
        gyro_y = (float(x[7:15]))/100
        gyro_z = (float(x[15:22]))/100
        acc_x = (float(x[22:29]))/100
        acc_y = (float(x[29:36]))/100
        acc_z = (float(x[36:43]))/100
        print (gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z)
        count = count + 1
    if read_serial[0] =='m':
        print ("success")
    if count == 5:
          ser.write(b'm')
          count = 0

