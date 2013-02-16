#include "inc/hw_types.h"
//Fichier qui contient des fonctions pour traiter les différents types de commandes à effectuer

/*
 * COMMAND
 */

// Fonction qui gere les commandes
// \param command : commande à traiter
// \return booléen qui dit si la commande a été traitée avec succès.
tBoolean handleCommand(volatile unsigned char* command){
	return true;
}

/*
 * PID
 */

//Filtre PID, ref: http://www.telecom-robotics.org/node/326
/*
volatile unsigned long PIDHandler(long Ci,long R)
{
  long P,D;
  double C;
  e = Ci-R;
  FR=filter(R);//On filtre le retour
  P=e;//Terme Proportionnel
  I = I+e;//Terme Integral
  D = FR-Old_R;//Terme D&eacute;riv&eacute;
  C = Kp*P+Ki*I+Kd*D;// oops on a fini
  Old_R = FR;//On sauvegarde
 
  return C;
}
*/

/*
 * MOTEUR
 */

//Initilisation des ports et pins pour transmettre des commandes aux moteurs
void initMotorCommand(void){
}

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorTurnCCW(unsigned short mnumber){
}

void motorTurnCW(unsigned short mnumber){
}

void motorBrake(unsigned short mnumber){
}


/*
 * LED
 */
 
void initLED(void){
}

void openLED(void){
}

void closeLED(void){
}
