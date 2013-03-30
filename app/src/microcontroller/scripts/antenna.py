import io
import array
import sys

def traiterDonneesAntennes(nomFichier):

	print('********************Test de decodeur antenne*************************')
	sumary = nomFichier + ".dec"
	h = open(sumary, 'w')
	intro = "Resultats : " + nomFichier + '\r\n'
	h.write(intro)

	v = 0
	for v in range(5):

		nom = ".\\" + nomFichier + '.' + str(v)
		f = open(nom, 'r')

		data = f.readlines()
	#print(data)

		f.close()
		dataInt = []
		smallest_0 = 65535
		s_indx = 0

		data.pop(0)
	# construction de l'array, reperage du plus petit et du plus grand element
		for x in range(len(data)):
			current = int(data[x])
			if current < smallest_0 and current > 2000:
				smallest_0 = current
				s_indx = x
			dataInt.append(current)

	#array qui contient les timings associes a leur valeur 1 ou 0
		ValuedData = []
	# nettoyage de l'array, attribution des 1 et des zeros
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

		#print(smallest_0)

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

	# trouver le plus petit "1"
		smallest_1 = 65535
		for j in range(len(ValuedData)):
			if ValuedData[j][1] == 1:
				if ValuedData[j][0] < smallest_1:
					smallest_1 = ValuedData[j][0]

	############################# Sequencage ################################################################
	# On attribue une valeur digitale a chaque timing selon les correspondances suivantes:
	# 10 - un seul zero; 11 - un seul '1'; 20 - deux zeros consecutifs; 21 - deux '1' consecutifs
		seqData = []
		for k in range(len(ValuedData)):
			if ValuedData[k][1] == 0:
				if ValuedData[k][0] > smallest_0*2.7: # scale factor pour zero
					seqData.append(20)
				else:
					seqData.append(10)
			else:
				if ValuedData[k][0] > smallest_1*1.75: # scale factor pour un
					seqData.append(21)
				else:
					seqData.append(11)


		#print(seqData)
	#print(altSeqData)
		output = nomFichier + ".seq"
		nom = ".\\" + output
		g = open(output, 'w')
		for l in range(len(ValuedData)):
			g.write(str(ValuedData[l][0]) + ',' + str(seqData[l]) + '\r\n')	
		g.close()



	######################## algorithme de Knuth-Morris-Pratt (modifie) ###############################################
	# On recherche la sequence P des bits d'arret et de depart dans l'echantillon seqData

		T = []
		P = [10,11,10,11,10,11,10,11,10,11,10,11,10,11,20] # chaine recherchee -> bits d'arret suivis du bit de depart

	# 1 - Conctruction du tableau des decalages

		for y in range(len(seqData) - 1, 0, -1):
			if seqData[y] == 20 and y < len(seqData) - 14 and y >= len(P) + 14 :
				T.append(y)
	# 2 - Recherche
		debut = -1
		ok = 0
	#altOk = 0
	
		m = 0
		i = len(P) - 1

		if len(T) > 0:
			while (T[m] - len(P) + i + 1) >= 0 and i >= 0 and m < len(T):
				if seqData[T[m] - len(P) + i + 1] == P[i]:
					i = i - 1
				else:
					m = m + 1
					i = len(P) - 1
					if m >= len(T):
						break

			if i == -1:
				debut = T[m]
				#print(str(T[m]))
				ok = 1
			else:
				debut = -1
				print("Erreur, sequence d'arret - depart non trouvee")
		else:
			debut = -1
			print("Erreur")

	########################################## Decodage #######################################################

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

			# verification (on va voir si la sequence de bits utiles de l'autre cote de la sequence d'arret correspond a celle trouvee)

				currentBit = 0
				idxRev = debut - 15 # on commence a la fin de la sequence et on recule
				reponseConf = []
				previousBit = 1
				while currentBit < 7:
					if previousBit == 1 and seqData[idxRev] == 21:
						reponseConf.append(0)
						idxRev = idxRev - 1
						previousBit = 0
						currentBit = currentBit + 1
					elif previousBit == 1 and seqData[idxRev] == 11:
						reponseConf.append(1)
						idxRev = idxRev - 2
						previousBit = 1
						currentBit = currentBit + 1
					elif previousBit == 0 and seqData[idxRev] == 10:
						reponseConf.append(0)
						idxRev = idxRev - 2
						previousBit = 0
						currentBit = currentBit + 1
					elif previousBit == 0 and seqData[idxRev] == 20:
						reponseConf.append(1)
						idxRev = idxRev - 1
						previousBit = 1
						currentBit = currentBit + 1

				reponseConf.reverse()

				if str(reponse) == str(reponseConf):
					print("Success ! :" + str(reponse))
					q = "Essai" + str(v) + " :" + "SUCCES " + str(reponseConf) + '\n' 
					h.write(q)
					return reponseConf
				else:
					qf = "Essai" + str(v) + " :" + "ECHEC\n"
					h.write(qf)
					print("Echec")
		else:
			print("Echec")
			qff = "Essai" + str(v) + " :" + "ECHEC (Sequence arret non trouvee)\n"
			h.write(qff)
			
	h.close()
	return False

if __name__ == "__main__":
    sys.exit(main())
