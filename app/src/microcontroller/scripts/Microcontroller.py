#!/usr/bin/env python

PKG = 'microcontroller'
import roslib; roslib.load_manifest(PKG)


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
