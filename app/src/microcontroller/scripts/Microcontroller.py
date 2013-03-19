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
    rospy.loginfo("Putting Pen Down ?:%s", req.down)
    
    return PutPenResponse()

def handleWriteToLCD(req):
    rospy.loginfo("Write Message To LCD:%s", req.message)
    
    return WriteToLCDResponse()

def handleDrawNumber(req):
    rospy.loginfo("Draw Number:%d Big?:%s ", req.number, req.isBig)
    
    return DrawNumberResponse()

def handleTurnLED(req):
    rospy.loginfo("Turn LED on?:%s", req.on)
    
    return TurnLEDResponse()

def handleTurnLED(req):
    rospy.loginfo("Turn LED on?:%s", req.on)
    
    return TurnLEDResponse()

def handleTranslate(req):
    rospy.loginfo("Translate Robot of x:%d y:%d", req.x, req.y)
    
    return TranslateResponse()

def handleRotateVerticallyCam(req):
    rospy.loginfo("Rotating cam of angle:%d", req.angle)
    
    return RotateVerticallyCamResponse()

def handleMove(req):
    rospy.loginfo("Executing Move Of:%f", req.distance) #req.distancex, req.distancey)

    commande = " 0000000"
    #req.distancex = req.distancex/21.7*6533
    #if(req.distancex != 0):
    #    sign = ' '
    #    if(req.distancex < 0):
    #        sign = '-'
    #    dist_string = str(req.distancex)
    #    while(len(dist_string) < 7):
    #        dist_string = '0' + dist_string
    #    commande = sign + dist_string
    
    sendCommandToController(commande)

    commande = " 0000000"
    distance = int(req.distance/21.7*6533)
    if(distance != 0):
        sign = ' '
        if(distance < 0):
            sign = '-'
        dist_string = str(abs(distance))
        while(len(dist_string) < 7):
            dist_string = '0' + dist_string
        commande = sign + dist_string
      
    sendCommandToController(commande)
      
    return MoveResponse()

def handleRotate(req):
    rospy.loginfo("Executing Rotation With Angle:%f", req.angle)
    
    commande = "00000000"
    if(req.angle != 0):
        angle_abs = int(abs(req.angle))
        if(req.angle > 0):  # Si angle positif
            if(abs(req.angle) > 99):  # Si un angle de plus de 2 digits
                commande = "TP" + str(angle_abs)
            else:
                print(angle_abs)
                commande = "TP0" + str(angle_abs)
        else: 
            if(abs(req.angle) > 99):
                commande = "TN" + str(angle_abs)
            else:
                commande = "TN0" + str(angle_abs)
        commande += "000"
        
    sendCommandToController(commande)
    
    return RotateResponse()

def handleDecodeAntenna(req):
    # Example de retour des informations
    response = DecodeAntennaResponse()
    response.isBig = False
    response.orientation = DecodeAntennaResponse.SOUTH  # Il est aussi possible d'utiliser NORTH, WEST, EAST
    response.number = 8
    
    rospy.loginfo("Decoded Antenna Param are... number:%d orientation:%d isBig?:%s ", response.number, response.orientation, response.isBig)
    
    return response

def sendCommandToController(commande):
    global ser
    
    rospy.loginfo("Sending command to microcontroller %s", commande)
    
    try: 
        ser.open()
    except Exception, e:
        rospy.logerr("IMPOSSIBLE D'OUVRIR LE PORT DU MICROCONTROLLEUR: %s", str(e))
        #exit()
    
    if ser.isOpen():
        try:
            ser.write(bytes(commande))
            response = None
            #time.sleep(0.5)  # le temps que le microcontrolleur recoive la commande
            while(response != '1'):
                response = ser.readline()  # Boucle while ici?? 
            print(repr("read data:" + response))
            
        except Exception, e1:
            rospy.logerr("LA COMMANDE %s NE S'EST PAS RENDU: %s", commande, str(e))
    
        ser.close()
    else:
        rospy.logerr("LE PORT DU MICROCONTROLLEUR N'EST PAS OUVERT")

def Microcontroller():
    global ser
    
    rospy.init_node('microcontroller')
    
    rospy.loginfo("Creating services for Microcontroller")
    s = rospy.Service('microcontroller/putPen', PutPen, handlePutPen)
    s = rospy.Service('microcontroller/writeToLCD', WriteToLCD, handleWriteToLCD)
    s = rospy.Service('microcontroller/drawNumber', DrawNumber, handleDrawNumber)
    s = rospy.Service('microcontroller/turnLED', TurnLED, handleTurnLED)
    s = rospy.Service('microcontroller/move', Move, handleMove)
    s = rospy.Service('microcontroller/rotate', Rotate, handleRotate)
    s = rospy.Service('microcontroller/decodeAntenna', DecodeAntenna, handleDecodeAntenna)
    s = rospy.Service('microcontroller/translate', Translate, handleTranslate)
    s = rospy.Service('microcontroller/rotateVerticallyCam', RotateVerticallyCam, handleRotateVerticallyCam)
    
    rospy.loginfo("Creating Serial Communication")
    ser = serial.Serial()
    ser.port = ('/dev/ttyUSB0') 
    ser.baudrate = 19200
    ser.parity = serial.PARITY_EVEN
    ser.stopbits = 1
    ser.timeout = 0
    ser.bytesize = serial.EIGHTBITS
    ser.writeTimeout = 0
    
    rospy.loginfo("Microcontroller initiated")
    
    rospy.spin()
        
if __name__ == '__main__':
    Microcontroller()
