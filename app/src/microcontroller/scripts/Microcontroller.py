#!/usr/bin/env python

PKG = 'microcontroller'
import roslib; roslib.load_manifest(PKG)

import serial
import io
import rospy
import sys
from microcontroller.srv import *

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

def handleExecuteMove(req):
    print "Executing move with angle \r"
    print req.angle
    print "and distance of \r"
    print req.distance

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
    ser = serial.Serial()
    ser.port = ('/dev/ttyUSB0')
    ser.baudrate = 115200
    ser.parity = serial.PARITY_EVEN
    ser.stopbits = 1
    ser.timeout = 2
    ser.open()

    ser.write(commande)
    ser.close()
    
    return ExecuteMoveResponse()

def handleDecodeAntenna(req):
    print "Decoding antenna \r"
    
    request = DecodeAntennaResponse()
    request.isBig = False
    request.orientation = DecodeAntennaResponse.SOUTH
    request.number = 8
    
    return request

def Microcontroller():
    rospy.init_node('microcontroller')
    
    s = rospy.Service('microcontroller/putPen', PutPen, handlePutPen)
    s = rospy.Service('microcontroller/writeToLCD', WriteToLCD, handleWriteToLCD)
    s = rospy.Service('microcontroller/drawNumber', DrawNumber, handleDrawNumber)
    s = rospy.Service('microcontroller/turnLED', TurnLED, handleTurnLED)
    s = rospy.Service('microcontroller/executeMove', ExecuteMove, handleExecuteMove)
    s = rospy.Service('microcontroller/decodeAntenna', DecodeAntenna, handleDecodeAntenna)

    rospy.spin()

        
if __name__ == '__main__':
    Microcontroller()
