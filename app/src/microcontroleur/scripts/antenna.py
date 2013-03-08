import io
import array
import sys

def main(argv=None):

	print('test')

	print('Entrer un nom de fichier entrant:')
	nomFichier = input() 

	print('Entrer un nom de fichier sortant:')
	output = input()

	nom = ".\\" + nomFichier
	f = open(nomFichier, 'r')

	data = f.readlines()
	#print(data)

	f.close()
	dataInt = []
	smallest_0 = 65535
	s_indx = 0

	data.pop(0)
	# construction de l'array, repérage du plus petit et du plus grand élément
	for x in range(len(data)):
		current = int(data[x])
		if current < smallest_0 and current > 2000:
			smallest_0 = current
			s_indx = x
		dataInt.append(current)

	#array qui contient les timings associés à leur valeur 1 ou 0
	ValuedData = []
	# nettoyage de l'array, attribution des 1 et des zéros
	i = s_indx
	toggle = True
	while i >= 0:
		ligne = []
		ligne.append(dataInt[i])
		if toggle:
			ligne.append(0)
		else:
			ligne.append(1)
		if dataInt[i] > 2000:
			ValuedData.append(ligne)
			toggle = not(toggle)
		i = i - 1

	print(smallest_0)

	ValuedData.reverse()

	i = s_indx + 1
	toggle = False
	while i <= (len(dataInt) - 1):
		ligne = []
		ligne.append(dataInt[i])
		if toggle == True:
			ligne.append(0)
		else:
			ligne.append(1)
		if dataInt[i] > 2000:
			ValuedData.append(ligne)
			toggle = not(toggle)
		i = i + 1

	print(ValuedData)

	# trouver le plus petit "1"
	smallest_1 = 65535
	for j in range(len(ValuedData)):
		if ValuedData[j][1] == 1:
			if ValuedData[j][0] < smallest_1:
				smallest_1 = ValuedData[j][0]

	print(smallest_1)

	altValuedData = []
	j = 0
	for j in range(len(ValuedData)):
		ligne = []
		ligne.append(ValuedData[j][0])
		if(ValuedData[j][1] == 0):
			ligne.append(1)
		else:
			ligne.append(0)
		altValuedData.append(ligne)
	print(altValuedData)

	# sequençage: 10 - un seul zéro; 11 - un seul '1'; 20 - deux zéros consécutifs; 21 - deux '1' consécutifs
	seqData = []
	for k in range(len(ValuedData)):
		if ValuedData[k][1] == 0:
			if ValuedData[k][0] > smallest_0*2.7: # scale factor pour zéro
				seqData.append(20)
			else:
				seqData.append(10)
		else:
			if ValuedData[k][0] > smallest_1*2.5: # scale factor pour un
				seqData.append(21)
			else:
				seqData.append(11)

	altSeqData = []
	k = 0
	alt_smallest_0 = smallest_1
	alt_smallest_1 = smallest_0
	for k in range(len(altValuedData)):
		if altValuedData[k][1] == 0:
			if altValuedData[k][0] > alt_smallest_0*1.8: # scale factor pour zéro
				altSeqData.append(20)
			else:
				altSeqData.append(10)
		else:
			if altValuedData[k][0] > alt_smallest_1*4: # scale factor pour un
				altSeqData.append(21)
			else:
				altSeqData.append(11)


	print(seqData)
	print(altSeqData)
	nom = ".\\" + output
	g = open(output, 'w')
	for l in range(len(ValuedData)):
		g.write(str(ValuedData[l][0]) + ',' + str(seqData[l]) + ',' + str(altSeqData[l]) + '\r\n')	
	g.close()



	# algorithme de Knuth-Morris-Pratt (modifié)

	T = []

	# 1 - Conctruction du tableau des décalages

	for y in range(len(seqData) - 1, 0, -1):
		if seqData[y] == 20 and y < len(seqData) - 14:
			T.append(y)


	P = [10,11,10,11,10,11,10,11,10,11,10,11,10,11,20] # chaîne recherchée -> bits d'arrêt suivis du bit de départ

	# 2 - Recherche
	debut = -1
	ok = 0
	altOk = 0
	
	m = 0
	i = len(P) - 1

	while (T[m] - len(P) + i + 1) >= 0 and i >= 0 and m < len(T):
		if seqData[T[m] - len(P) + i + 1] == P[i]:
			i = i - 1
		else:
			m = m + 1
			i = len(P) - 1

	if i == -1:
		debut = T[m]
		print(str(T[m]))
		ok = 1
	else:
		debut = -1
		print("Erreur")

	# Si erreur, essai avec l'assignation alternative
	if debut == -1:
		altT = []
		for y in range(len(altSeqData) - 1, 0, -1):
			if altSeqData[y] == 20 and y < len(altSeqData) - 14:
				altT.append(y)
		print(str(altT))
		altM = 0
		i = len(P) - 1
		print(str(altT[0]))
		while (altT[altM] - len(P) + i + 1) >= 0 and i >= 0 and altM < len(altT):
			if altSeqData[altT[altM] - len(P) + i + 1] == P[i]:
				i = i - 1
			else:
				altM = altM + 1
				i = len(P) - 1

		if i == -1:
			debut = altT[altM]
			print(str(altT[altM]))
			altOk = 1
		else:
			debut = -1
			print("Erreur")	

	# Décodage

	if debut != -1:
		currentBit = 0
		reponse = []
		idx = debut + 1
		previousBit = 0

		if ok == 1:
			while currentBit < 7:
				if previousBit == 0 and seqData[idx] == 11:
					reponse.append(0)
					currentBit = currentBit + 1
					idx = idx + 2
					previousBit = 0
				elif previousBit == 0 and seqData[idx] == 21:
					reponse.append(1)
					currentBit = currentBit + 1
					idx = idx + 1
					previousBit = 1
				elif previousBit == 1 and seqData[idx] == 10:
					reponse.append(1)
					currentBit = currentBit + 1
					idx = idx + 2
					previousBit = 1
				elif previousBit == 1 and seqData[idx] == 20:
					reponse.append(0)
					currentBit = currentBit + 1
					idx = idx + 1
					previousBit = 0
		elif altOk == 1:
			while currentBit < 7:
				if previousBit == 0 and altSeqData[idx] == 11:
					reponse.append(0)
					currentBit = currentBit + 1
					idx = idx + 2
					previousBit = 0
				elif previousBit == 0 and altSeqData[idx] == 21:
					reponse.append(1)
					currentBit = currentBit + 1
					idx = idx + 1
					previousBit = 1
				elif previousBit == 1 and altSeqData[idx] == 10:
					reponse.append(1)
					currentBit = currentBit + 1
					idx = idx + 2
					previousBit = 1
				elif previousBit == 1 and altSeqData[idx] == 20:
					reponse.append(0)
					currentBit = currentBit + 1
					idx = idx + 1
					previousBit = 0

		print(str(reponse))

if __name__ == "__main__":
    sys.exit(main())