#!/usr/bin/env python

PKG = 'microcontroller'
import roslib; roslib.load_manifest(PKG)

import serial
import io
import rospy
import sys
import array
import time

from microcontroller.srv import *

ser = None

def handlePutPen(req):
    print "put pen down : \r"
    print req.down
    
    return PutPenResponse()

def handleWriteToLCD(req):
    print "Write to LCD messsage : \r"
    print req.message
    
    return WriteToLCDResponse()

def handleDrawNumber(req):
    print "Draw number : \r"
    print req.number
    print "Draw it big? \r"
    print req.isBig
    
    return DrawNumberResponse()

def handleTurnLED(req):
    print "Turn LED on \r"
    print req.on
    
    return TurnLEDResponse()

def handleMove(req):
    print "Executing move with a distance of \r"
    print req.distance

    commande="00000000"
    if(req.distance!=0):
        sign = ' '
        if(req.distance < 0):
            sign = '-'
        dist_string = str(req.distance)
        while(len(dist_string)<7):
            dist_string = '0'+dist_string
        commande = ' 0000000'+sign+dist_string
    
    print(commande)
    sendCommandToController(commande)
    
    return MoveResponse()

def handleRotate(req):
    print "Executing rotation with angle \r"
    print req.angle
    
    commande="00000000"
    if(req.angle!=0):
        angle_abs = int(abs(req.angle))
        if(req.angle > 0): #Si angle positif
            if(abs(req.angle) > 99): #Si un angle de plus de 2 digits
                commande="TP"+str(angle_abs)
            else:
                print(angle_abs)
                commande="TP0"+str(angle_abs)
        else: 
            if(abs(req.angle) > 99):
                commande="TN"+str(angle_abs)
            else:
                commande="TN0"+str(angle_abs)
        commande+="000"
        
    print(commande)
    sendCommandToController(commande)
    
    return RotateResponse()

def handleDecodeAntenna(req):
    print "Decoding antenna \r"
    
    #Example de retour des informations
    response = DecodeAntennaResponse()
    response.isBig = False
    response.orientation = DecodeAntennaResponse.SOUTH #Il est aussi possible d'utiliser NORTH, WEST, EAST
    response.number = 8
    
    print "isBig:" + isBig
    print "orientation:" + orientation
    print "number:"
    
    return response

def sendCommandToController(commande):
    global ser
    
    ser.open()
    
    ser.write(bytes(commande))
    time.sleep(0.5) #le temps que le microcontrolleur recoive la commande
    
    response = ser.readline() #Boucle while ici?? 
    print(repr("read data:" + response))

    ser.close()
    
    return true

def Microcontroller():
    global ser
    
    rospy.init_node('microcontroller')
    
    #Init des services de ROS
    s = rospy.Service('microcontroller/putPen', PutPen, handlePutPen)
    s = rospy.Service('microcontroller/writeToLCD', WriteToLCD, handleWriteToLCD)
    s = rospy.Service('microcontroller/drawNumber', DrawNumber, handleDrawNumber)
    s = rospy.Service('microcontroller/turnLED', TurnLED, handleTurnLED)
    s = rospy.Service('microcontroller/move', Move, handleMove)
    s = rospy.Service('microcontroller/rotate', Rotate, handleRotate)
    s = rospy.Service('microcontroller/decodeAntenna', DecodeAntenna, handleDecodeAntenna)
    
    #Init du port serial
    ser = serial.Serial()
    ser.port = ('/dev/ttyUSB0') 
    ser.baudrate = 19200
    ser.parity = serial.PARITY_EVEN
    ser.stopbits = 1
    ser.timeout = 0
    ser.bytesize = serial.EIGHTBITS
    ser.writeTimeout = 2
    
    rospy.spin()

        
if __name__ == '__main__':
    Microcontroller()
