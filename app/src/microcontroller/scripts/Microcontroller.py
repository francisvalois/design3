#!/usr/bin/env python

PKG = 'microcontroller'
import roslib; roslib.load_manifest(PKG)

import serial
import io
import rospy
import sys
import array
import time
from antenna import *
from fileformat import *

from microcontroller.srv import *

ser = None
serCam = None

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

    sendCommandToController("L{0}{1}{2}0000".format(req.sudocubeNo, req.orientation, req.size))
    
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
    req.x = int(req.x / 21.7 * 6533)
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
    req.y = int(req.y / 21.7 * 6533)
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
    global serCam
    rospy.loginfo("Rotating cam of vAngle:%d  and hAngle:%d", req.vAngle, req.hAngle)

    #Limite entre 38.27 et -51.73 pour hAngle et entre 27.63 et 062.37  
    serCam.open()
    if serCam.isOpen(): 
    	anglex_res = int(146+float(req.hAngle)/45*127)
    	angley_res = int(176+float(req.vAngle)/45*127)
    	if(anglex_res> 254 or anglex_res < 0 or angley_res> 254 or angley_res < 0):
            print("!!!!!!!VALEUR DEPASSE 254 OU MOINS 0!!!!!!!!")
        else:
            commande = ("FF04{0:02X}".format(anglex_res)).decode('hex')
            serCam.write(bytes(commande))
            time.sleep(0.1)
            commande = ("FF05{0:02X}".format(angley_res)).decode('hex')
            serCam.write(bytes(commande))
        serCam.close()
    
    return RotateCamResponse()

def handleMove(req):
    rospy.loginfo("Executing Move Of:%f", req.distance)

    commande = " 0000000"
    sendCommandToController(commande)

    commande = " 0000000"
    distance = int(req.distance / 21.7 * 6533)
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
        angle_abs = abs(req.angle)
        if(req.angle > 0):  # Si angle positif
            if(abs(req.angle) > 99):  # Si un angle de plus de 2 digits
                commande = "TP" + str(angle_abs).substring(0, 6)
            elif (abs(req.angle) >= 10 and abs(req.angle) <= 99):
                print(angle_abs)
                commande = "TP0" + str(angle_abs).substring(0, 5)
            else:
                print(angle_abs)
                commande = "TP00" + str(angle_abs).substring(0, 4)
        else: 
            if(abs(req.angle) > 99):
                commande = "TN" + str(angle_abs).substring(0, 6)
            elif (abs(req.angle) >= 10 and abs(req.angle) <= 99):
                commande = "TN0" + str(angle_abs).substring(0, 5)
            else:
                commande = "TN00" + str(angle_abs).substring(0, 4)
                
        commande += "000"
        
    sendCommandToController(commande)
    
    return RotateResponse()

def handleDecodeAntenna(req):
    rospy.loginfo("Decoding antenna param...")

    response = DecodeAntennaResponse()
    test = sendCommandToController("A0000000")
    response.number = int(test[0])*pow(2,2)+int(test[1])*pow(2,1)+int(test[2]) + 1
    if int(test[3]) == 0 and int(test[4]) == 0:
        response.orientation = DecodeAntennaResponse.NORTH
    elif int(test[3]) == 0 and int(test[4]) == 1:
        response.orientation = DecodeAntennaResponse.EAST
    elif int(test[3]) == 1 and int(test[4]) == 0:
        response.orientation = DecodeAntennaResponse.SOUTH
    elif int(test[3]) == 1 and int(test[4]) == 1:
        response.orientation = DecodeAntennaResponse.WEST
    if int(test[5]) == 1:
        response.isBig = True
    else:
        response.isBig = False
    rospy.loginfo("Decoded Antenna Param are... number:%d orientation:%d isBig?:%s ", response.number, response.orientation, response.isBig)
    
    return response

def handleGetSonarXDistance(req):
    rep = sendCommandToController("S0000000")
    valeur = ord(bytes(rep[0]))*pow(2,24) + ord(bytes(rep[1]))*pow(2,16) + ord(bytes(rep[2]))*pow(2,8) + ord(bytes(rep[3]))
    # Exemple de reponse
    response = GetSonarXDistanceResponse(); 
    response.distance = float(valeur)/16/58;
    
    rospy.loginfo("Getting distance from sonar no:%d  distance:%f", req.sonarNo, response.distance)
    
    return response

def sendCommandToController(commande):
    global ser
    
    rospy.loginfo("Sending command to microcontroller %s", commande)
    
    response = ""
    try: 
        ser.open()
        
        if ser.isOpen():
            ser.write(bytes(commande))
            time.sleep(0.5)  # le temps que le microcontrolleur recoive la commande

            while('E' not in response):
                response = ser.readline()  # Boucle while ici?? 
            print(repr("read data:" + response))
    
            ser.close()
    except Exception, e:
        rospy.logerr("IMPOSSIBLE D'OUVRIR LE PORT DU MICROCONTROLLEUR: %s", str(e))
    return response

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
    # ser.port = ('/dev/ttyUSB0') 
    ser.port = ('/dev/serial/by-id/usb-TXI_Luminary_Micro_ICDI_Board_0B010140-if01-port0')
    ser.baudrate = 115200
    ser.parity = serial.PARITY_EVEN
    ser.stopbits = 1
    ser.timeout = 0
    ser.bytesize = serial.EIGHTBITS
    ser.writeTimeout = 0

    rospy.loginfo("Creating Serial Communication with Camera")
    serCam = serial.Serial()
    #serCam.port = ('/dev/ttyACM0') 
    serCam.port = ('/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00044759-if00')
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
