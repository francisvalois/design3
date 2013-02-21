# Terminal série 
# 

import serial
import io
import array
import sys

# Formattage des fichiers : '$' est le caractère sentinelle pour une colonne, '\n' pour fin de fichier

def fichierCVS(donnees, nomFichier):

	#ouverture de fichier sous linux
	#f = open('./output.cvs', 'w')
	#nom = "./" + nomFichier
	#ouverture de fichier sous windows
	nom = ".\\" + nomFichier
	f = open(nom, 'w')

	for i in range(len(donnees)):
		f.write(str(donnees[i][0]) + ',' + str(donnees[i][1]) +'\n')
	
	f.close()

def formatDonnees(rawData):

	print("{0:d} bytes de donnees lues".format(len(rawData)))
	nbColonnes = rawData[0] - 48

	if (len(rawData) - 2)%4 != 0:
		return 0

	dataArray = []

	Data = rawData[1:len(rawData) -1]

	print(len(Data))
	print(rawData)
	print(rawData[0])
	print(nbColonnes)
	nbLignes = int((len(Data)/4)/nbColonnes)
	print(nbLignes)

	for x in range(nbLignes):
		colonne = []
		for y in range(nbColonnes):
			s = Data[x + y*nbLignes: x + y*nbLignes + 4]
			z = int.from_bytes(s, 'big')
			print(z)
			colonne.append(z)
		dataArray.append(colonne)

	return dataArray


def main(argv=None):
	fini = False
	while fini == False:
		print('Entrer un nom de fichier:')
		nomFichier = input() 
		print('Entrer une commande (Usage : opcode arg1 arg2)')
		print('Commande:')
		commande = input()

		while len(commande) != 8 or commande[2] != ' ' or commande[5] != ' ':
			print('Usage : entrer opcode arg1 arg2')
			print('Commande: ')
			commande = input()

		com = commande.rsplit(' ')


		ser = serial.Serial()

		# pour ouvrir un port sous linux
		#ser.port = ('/dev/ttyUSB0')

    	# pour ourvrir le port COM6 sous windows - no port = no device - 1
		ser.port = 5

		ser.baudrate = 115200
		ser.parity = serial.PARITY_EVEN
		ser.stopbits = 1

		if not(com[2].isalnum) or int(com[2]) <= 0:
			ser.timeout = 1
		else:
			ser.timeout = int(com[2]) + 1

		ser.open()
		ser.write(bytes(commande, encoding = "ascii"))
		donnees = ser.read(42)
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

