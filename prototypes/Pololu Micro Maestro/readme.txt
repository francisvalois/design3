//Auteur : PL

Pour obtenir les settings du Pololu, seulement brancher le micro à l'ordi et ouvrir Maestro Control Center.

POur obtenir mes settings, faire open setting et choisir maestro_settings.

//Aperçu mes settings dans PololuMaestro Control Center
Maestro
Servo4
    Go to
    Position=1598.25
Servo5
    Go to
    Position=1683.50

/* Note si alimenté et fonctionnel LED ROUGE SUR MICRO FLASH */
/* ATTENTION SVP PAS POUSSER/BOUGER LA CAMERA PENDANT MOTEURS ALIMENTÉ SINON FAIT FORCER MOTEURS ET PEUT FAIRE PETER LES ENTRÉES ET L'ALIM*/


Avec le Command Protocol du Pololu:
pour position normal (straight), transmettre au COM 4 (en hex):
84 05 30 34 && 84 04 00 31
pour position penché, transmettre( en hex):
84 05 70 2E && 84 04 00 31
