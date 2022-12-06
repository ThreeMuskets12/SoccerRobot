import serial
import time
ad=serial.Serial('/dev/tty.usbmodem1101',115200)
time.sleep(2) #After opening a connection to the arduino the arduino will reboot - so you need to wait 2 seconds for it to boot up before sending data over!
print(ad.name)
CW = b'\x01\x55\x80\x00\x00\x00\x00\x00\x00\x00\x01\x55\x80\x00\x00\x00\x00\x00\x00\x00\x01\x55\x80\x00\x00\x00\x00\x00\x00\x00\x01\x55\x80\x00\x00\x00\x00\x00\x00\x00\x01\x55\x80\x00\x00\x00\x00\x00\x00\x00'
ad.write(CW)