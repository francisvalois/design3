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

def Microcontroller():
    rospy.init_node('microcontroller')
    
    s = rospy.Service('microcontroller/putPen', PutPen, handlePutPen)
    s = rospy.Service('microcontroller/writeToLCD', WriteToLCD, handleWriteToLCD)
    s = rospy.Service('microcontroller/drawNumber', DrawNumber, handleDrawNumber)
    s = rospy.Service('microcontroller/turnLED', TurnLED, handleTurnLED)

    rospy.spin()

        
if __name__ == '__main__':
    Microcontroller()
