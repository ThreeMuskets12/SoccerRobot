#This quaternion 3D visualization program was written by Paul McWhorter
#of https://toptechboy.com. He has a 26 episode long series on using
#the BNO055 with an Arduino Nano. This python program was a part of
#his "Lesson 21: Visualizing 3D Rotations in VPython using Quaternions"
#Some minor modifications were made to allow the visualizer to better
#represent the physical layout of the BNO085.

#Modifications made by Noah Page

#To get this to work on any machine, you will need to modify
#line #20 so that it points to the location where you can
#stream the serial data from the SAMV71 Xplained board
#on your own computer. On my Mac, this happens to be
#/dev/tty.usbmodem2102, but it could be different for other
#machines. Make sure to leave the baud rate at 38400!


from vpython import *
from time import *
import numpy as np
import math
import serial

ad=serial.Serial('/dev/tty.usbmodem2102',38400)
sleep(1)

scene.range=5
scene.background=color.yellow
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)

scene.width=1200
scene.height=1080

xarrow=arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

frontArrow=arrow(length=1,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

myObj=box(lenght=3,width=2,height=.1,pos=vector(0,.1,0),opacity=.5,color=color.black)
while (True):
    try:
        while (ad.inWaiting()==0):
            pass
        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        q0=float(splitPacket[0])
        q1=float(splitPacket[1])
        q2=float(splitPacket[2])
        q3=float(splitPacket[3])

        roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=math.asin(2*(q0*q2-q3*q1))
        yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
        print(str(roll) + " " + str(pitch) + " " + str(yaw))
        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=2
        frontArrow.length=1
        upArrow.length=1
    except:
        pass