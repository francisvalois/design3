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
    
    if(req.down):
        commande = "P0000000"
    else:
        commande = "M0000000"

    sendCommandToController(commande)
    
    return PutPenResponse()

def handleWriteToLCD(req):
    rospy.loginfo("Write Antenna Param To LCD  number:%d isBig:%s orientation:%s", req.sudocubeNo, req.size, req.orientation)

    sendCommandToController("{0}{1}{2}00000".format(), req.sudocubeNo, req.orientation, req.size)
    
    return WriteToLCDResponse()

def handleDrawNumber(req):
    rospy.loginfo("Draw Number:%d Big?:%s ", req.number, req.isBig)

    sendCommandToController("D{0}{1}00000".format(('P', 'G')[req.isBig], req.number))
    
    return DrawNumberResponse()

def handleTurnLED(req):
    rospy.loginfo("Turn LED on?:%s", req.on)

    if(req.on):
        commande = "O0000000"
    else:
        commande = "C0000000"

    sendCommandToController(commande)
    
    return TurnLEDResponse()

def handleTranslate(req):
    rospy.loginfo("Translate Robot of x:%d y:%d", req.x, req.y)

    commande = " 0000000"
    req.x = int(req.x/21.7*6533)
    if(req.x != 0):
        sign = ' '
        if(req.x < 0):
            sign = '-'
        dist_string = str(abs(req.x))
        while(len(dist_string) < 7):
            dist_string = '0' + dist_string
        commande = sign + dist_string
    
    sendCommandToController(commande)

    commande = " 0000000"
    req.y = int(req.y/21.7*6533)
    if(req.y != 0):
        sign = ' '
        if(req.y < 0):
            sign = '-'
        dist_string = str(abs(req.y))
        while(len(dist_string) < 7):
            dist_string = '0' + dist_string
        commande = sign + dist_string
      
    sendCommandToController(commande)
    
    return TranslateResponse()

def handleRotateCam(req):
    rospy.loginfo("Rotating cam of vAngle:%d  and hAngle:%d", req.vAngle, req.hAngle)

    serCam.open()
    if req.angle:
        commande = "8405702E".decode("hex")
        serCam.write(bytes(commande))
        time.sleep(0.1)
        commande = "84040031".decode("hex")
        serCam.write(bytes(commande))
    else:
        commande = "84053034".decode("hex")
        serCam.write(bytes(commande))
        time.sleep(0.1)
        commande = "84040031".decode("hex")
        serCam.write(bytes(commande))
    serCam.close()
    
    return RotateVerticallyCamResponse()

def handleMove(req):
    rospy.loginfo("Executing Move Of:%f", req.distance)

    commande = " 0000000"
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
    
    commande = "TN000000"
    if(req.angle != 0):
        angle_abs = int(abs(req.angle))
        if(req.angle > 0):  # Si angle positif
            if(abs(req.angle) > 99):  # Si un angle de plus de 2 digits
                commande = "TP" + str(angle_abs)
            elif (abs(req.angle) >=10 and abs(req.angle) <= 99):
                print(angle_abs)
                commande = "TP0" + str(angle_abs)
            else:
                print(angle_abs)
                commande = "TP00" + str(angle_abs)
        else: 
            if(abs(req.angle) > 99):
                commande = "TN" + str(angle_abs)
            elif (abs(req.angle) >=10 and abs(req.angle) <= 99):
                commande = "TN0" + str(angle_abs)
            else:
                commande = "TN00" + str(angle_abs)
                
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

def handleGetSonarXDistance(req):
    rospy.loginfo("Getting distance from sonar no:%d", req.sonarNo)
    
    #Exemple de reponse
    response = SonarXDistanceResponse(); 
    response.distance = 0.0;
    
    return response

def sendCommandToController(commande):
    global ser
    
    rospy.loginfo("Sending command to microcontroller %s", commande)
    
    try: 
        ser.open()
        
        if ser.isOpen():
            ser.write(bytes(commande))
            time.sleep(0.5)  # le temps que le microcontrolleur recoive la commande

            response = None
            while(response != 'E'):
                response = ser.readline()  # Boucle while ici?? 
            print(repr("read data:" + response))
    
            ser.close()
    except Exception, e:
        rospy.logerr("IMPOSSIBLE D'OUVRIR LE PORT DU MICROCONTROLLEUR: %s", str(e))

def Microcontroller():
    global ser
    global serCam
    
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
    s = rospy.Service('microcontroller/rotateCam', RotateCam, handleRotateCam)
    s = rospy.Service('microcontroller/getSonarXDistance', GetSonarXDistance, handleGetSonarXDistance)
    
    rospy.loginfo("Creating Serial Communication")
    ser = serial.Serial()
    #ser.port = ('/dev/ttyUSB0') 
    ser.port = ('/dev/serial/by-id/usb-TXI_Luminary_Micro_ICDI_Board_0B01015D-if01-port0')
    ser.baudrate = 19200
    ser.parity = serial.PARITY_EVEN
    ser.stopbits = 1
    ser.timeout = 0
    ser.bytesize = serial.EIGHTBITS
    ser.writeTimeout = 0

    rospy.loginfo("Creating Serial Communication with Camera")
    serCam = serial.Serial()
    serCam.port = ('/dev/ttyACM0') 
    serCam.baudrate = 19200
    serCam.parity = serial.PARITY_NONE
    serCam.stopbits = 1
    serCam.timeout = 0
    serCam.bytesize = serial.EIGHTBITS
    serCam.writeTimeout = 0   
    
    rospy.loginfo("Microcontroller initiated")
    
    rospy.spin()
    
    ser.close()
        
if __name__ == '__main__':
    Microcontroller()
