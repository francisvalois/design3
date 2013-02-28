#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Terminal serie 
# 

import serial
import io
import array
import sys

# Formattage des fichiers : '$' est le caractère sentinelle pour une colonne, '\n' pour fin de fichier

def fichierCVS(donnees, nomFichier):

# --------------- Commenter / Decommenter selon systeme d'exploitation -------------
	# Linux
	nom = "./" + nomFichier

	# Windows
	#nom = ".\\" + nomFichier
# ----------------------------------------------------------------------------------

	f = open(nom, 'w')

	for i in range(len(donnees)):
		f.write(donnees[i][0].lstrip('0') + ',' + donnees[i][1].lstrip('0') +'\n')
	
	f.close()

def formatDonnees(rawData):

	print("{0:d} bytes de donnees lues".format(len(rawData)))
	nbColonnes = rawData[0] - 48
	#print(rawData)

	Data = rawData[1:len(rawData) -1]
	DataString = Data.decode('ascii')
	#print(DataString)

	dataInt = []

	for i in range(10):
		dataInt.append(DataString[i*10:i*10 + 10])

	#print(dataInt)

	if (len(rawData) - 2)%4 != 0:
		return 0

	dataArray = []

	nbLignes = int((len(dataInt))/nbColonnes)
	#print(nbLignes)

	for x in range(nbLignes):
		ligne = []
		for y in range(nbColonnes):
			ligne.append(dataInt[y*nbLignes + x])
		print(ligne)
		dataArray.append(ligne)

	#print(dataArray)

	return dataArray


def main(argv=None):
	fini = False
	while fini == False:
		print('Entrer un nom de fichier:')
# --------------- Commenter / Decommenter selon systeme d'exploitation -------------
		# Windows
		#nomFichier = input()
		# Linux
		nomFichier = raw_input() 

# ----------------------------------------------------------------------------------
		
		print('Entrer une commande (Usage : opcode arg1 arg2)')
		print('Commande:')

# --------------- Commenter / Decommenter selon systeme d'exploitation -------------
		# Windows
		#commande = input()
		# Linux
		commande = raw_input()
# ----------------------------------------------------------------------------------
		
		while len(commande) != 8:
			print('Usage : entrer opcode arg1 arg2')
			print('Commande: ')
			# Windows
			#commande = input()
			# Linux
			commande = raw_input()

		com = commande.rsplit(' ')


		ser = serial.Serial()

# --------------- Commenter / Decommenter selon systeme d'exploitation -------------

		# Linux
		ser.port = ('/dev/ttyUSB0')

		# Windows
    	# pour ourvrir le port COM6 sous windows - no port = no device - 1
		#ser.port = 5

# ----------------------------------------------------------------------------------
		ser.baudrate = 115200
		ser.parity = serial.PARITY_EVEN
		ser.stopbits = 1
		ser.timeout = 2

		#if not(com[2].isalnum) or int(com[2]) <= 0:
		#	ser.timeout = 1
		#else:
		#	ser.timeout = int(com[2]) + 1

		ser.open()

# --------------- Commenter / Decommenter selon systeme d'exploitation -------------

		# Windows
		#ser.write(bytes(commande, encoding = "ascii"))

		# Linux
		ser.write(commande)
# ----------------------------------------------------------------------------------

		byte = ' '.encode('ascii')
		donnees = bytearray()
		while byte != '\n'.encode('ascii'):
			byte = ser.read(1);
			donnees = donnees + byte

		#donnees = ser.read(100)
		#donnees = ser.readline()

		dataArray = formatDonnees(donnees)
		if dataArray != 0:
			fichierCVS(dataArray, nomFichier)
		else:
			print("Erreur de formattage des donnees\n")
		ser.close()

		print('Continuer (y/n) ?')
		reponse = input()
		while reponse != 'y' and reponse != 'n':
			print("Taper 'y' pour oui, 'n' pour non.")
			reponse = input()

		if 'y' in reponse:
			fini = False
		if 'n' in reponse:
			fini = True

if __name__ == "__main__":
    sys.exit(main())

