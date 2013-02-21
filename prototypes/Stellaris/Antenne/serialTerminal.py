# Terminal série 
# 

import serial
import io
import array

# Formattage des fichiers : '$' est le caractère sentinelle pour une colonne, '\n' pour fin de fichier

def fichierCVS(donnees):

	#ouverture de fichier sous linux
	f = open('/home/diane/output.cvs', 'w')
	#ouverture de fichier sous windows
	#f = open('C:\Users\diane\Documents\output.cvs')

	for i in xrange(len(donnees[0])):
		f.write(str[0][i])
		for j in xrange(1, len(donnees)):
			f.write(',', str(d[j][i]))
		f.write('\n') 
	f.close()

def formatDonnees(rawData):

	nbColonnes = int(rawData[0])
	#if(len(rawData) - 2)%2 == 0:
	#	m = (len(rawData) - 2)/2
	#	nbColonnes = 2
	#	nbLignes = m/4
	#if(len(rawData) - 3)%3 == 0:
	#	m = (len(rawData) - 3)/3
	#	nbColonnes = 3
	#	nbLignes = m/4

	#if rawData[m] != '$' and m%4 !=0:
	#	return 0
	if (len(rawData) - 2)%4 != 0:
		return 0

	dataArray = []
	#if nbColonnes == 2:
		#Data = rawData[0:m] + rawData[m+1:len(rawData)-1]
	#else:
	#	Data = rawData[0:m] + rawData[m+1:m*2+1] + rawData[m*2+2:len(rawData)-1]

	Data = rawData[1:len(rawData) -1]

	nbLignes = (len(Data)/4)/nbColonnes 

	for x in xrange(nbLignes):
		colonne = []
		for y in xrange(nbColonnes):
			s = Data[x + y*nbLignes: x + y*nbLignes + 4]
			z = s.encode('hex')
			colonne.append(int(z,16))
		dataArray.append(colonne)

	return dataArray


def main(argv=None):
	print('Entrer une commande (Usage : opcode arg1 arg2')
	commande = print input('Commande: ')

	while commande.count() != 8 or commande[2] != ' ' or commande[5] != ' ':
		print('Usage : entrer opcode arg1 arg2')
		print input('Commande: ')

	com = commande.rsplit(' ')


	ser = serial.Serial()

	# pour ouvrir un port sous linux
	ser.port = ('dev/ttyUSB0')

    # pour ourvrir un port sous windows
    #ser.port('COM6')

	ser.baudrate = 19200
	ser.parity = serial.PARITY_EVEN

	if !com[2].isalnum or com[2] <= 0:
		ser.timeout = 1
	else:
		ser.timeout = int(com[2]) + 1

	ser.open()
	ser.write(commande)
	donnees = ser.readlines()
	ser.close()

	dataArray = formatDonnees(donnees)
	if dataArray != 0:
		fichierCVS(dataArray)
	else:
		print("Erreur de formattage des donnees\n")

if __name__ == "__main__":
    sys.exit(main())

