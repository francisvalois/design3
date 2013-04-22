#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Terminal serie 
# 

import serial
import io
import array
import sys

# Formattage des fichiers : '$' est le caractere sentinelle pour une colonne, '\n' pour fin de fichier

def fichierCVS(donnees, nomFichier):

# --------------- Commenter / Decommenter selon systeme d'exploitation -------------
    # Linux
    nom = "./" + nomFichier

    # Windows
    #nom = ".\\" + nomFichier
# ----------------------------------------------------------------------------------
    f = open(nomFichier, 'w')
    if(len(donnees[0]) == 1):
        for i in range(len(donnees)):
            if(donnees[i][0] != '0000000000'):
                f.write(donnees[i][0].lstrip('0') + '\n')
            else:
                f.write('0' + '\n')
    else:
        for i in range(len(donnees)):
            if donnees[i][0] != "0000000000" and donnees[i][1] != "0000000000":
                f.write(donnees[i][0].lstrip('0') + ',' + donnees[i][1].lstrip('0') +'\n')
            elif donnees[i][0] == "0000000000" and donnees[i][1] != "0000000000":
                f.write("0" + ',' + donnees[i][1].lstrip('0') + '\n')
            elif donnees[i][0] != "0000000000" and donnees[i][1] == "0000000000" :
                f.write(donnees[i][0].lstrip('0') + ',' + "0" + '\n')
            else:
                f.write("0,0" + '\n')
    
    f.close()

def formatDonnees(rawData):

    print("{0:d} bytes de donnees lues".format(len(rawData)))
    nbColonnes = rawData[0] - 48
    #print(rawData)

    Data = rawData[1:len(rawData) -1]
    DataString = Data.decode("ascii")
    #print(DataString)

    dataInt = []

    longueur = int(len(DataString)/10)

    for i in range(longueur):
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
        #print(ligne)
        dataArray.append(ligne)

    #print(dataArray)

    return dataArray



if __name__ == "__main__":
    sys.exit(main())

