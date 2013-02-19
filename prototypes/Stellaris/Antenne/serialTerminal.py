# Terminal série 
# 

import serial
import io

# Formattage des fichiers : '$' est le caractère sentinelle pour une colonne, '\n' pour fin de fichier

def fichierCVS(donnees):

	i = 0
	j = 0
	x = 0
	d = []
	d1 = []
	for i in range(len(donnees)):
		if cmp(donnees[i],'$') != 0 and cmp(donnees[i],'\n') != 0:
			x +=1
			d1.append(donnees[i])
		else:
			d.append(d1)
			d1 = []
			j +=1

	#ouverture de fichier sous linux
	f = open('/home/diane/output.cvs', 'w')
	#ouverture de fichier sous windows
	#f = open('C:\')

	l = 0
	k = 0
	m = x/j
	while l < m:
		f.write(str(d[l][k]))
		while k < j:
			l +=1
			f.write(',', str(d[l][k]))
		f.write('\n') 
		k +=1
	f.close()



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
	donnees = ser.readline()
	ser.close()

	fichierCVS(donnees)

if __name__ == "__main__":
    sys.exit(main())

