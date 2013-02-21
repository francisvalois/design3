#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

//Variables globales externes
extern volatile unsigned long periodPWM; //Période du PWM
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile long speed, position;

//Variables globales
volatile long pos0, pos1, pos2, pos3;
volatile long previous_error0, previous_error1, previous_error2, previous_error3;
volatile long I0, I1, I2, I3;
volatile long consigne0, consigne1, consigne2, consigne3; 
volatile long output0, output1, output2, outpu3;
volatile long Kd, Ki, Kp;
volatile long slow_brake_pente, slow_start_pente;
long tolerancePID;
tBoolean slow_brake, slow_start;
//Fichier qui contient des fonctions pour traiter les différents types de commandes à effectuer

//Declaration de fonctions
//commande.c
void motorTurnCCW(unsigned short mnumber);
void motorTurnCW(unsigned short mnumber);
void motorBrake(unsigned short mnumber);

/*
 * COMMANDE
 */

// Fonction qui gere les commandes
// \param command : commande à traiter
// \return booléen qui dit si la commande a été traitée avec succès.
tBoolean handleCommand(volatile unsigned char* command){
	return true;
}

/*
 * PID/Asservissement
 */

//Filtre PID, ref: http://www.telecom-robotics.org/node/326

long PIDHandler(volatile long *consigne, volatile long *measured_value, volatile long *I, volatile long *previous_error, long dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  long D = (error - *previous_error)/dt;
  long output = Kp*error + Ki*(*I) + Kd*D;
  *previous_error = error;
  return output;
}
  
/*  double C;
  e = Ci-R;
  FR=filter(R);//On filtre le retour
  P=e;//Terme Proportionnel
  I = I+e;//Terme Integral
  D = FR-Old_R;//Terme D&eacute;riv&eacute;
  C = Kp*P+Ki*I+Kd*D;// oops on a fini
  Old_R = FR;//On sauvegarde
  return new_consigne;
}*/

void asservirMoteurs(void){
	
	//Motor 0
	pos0 =  QEIPositionGet(QEI0_BASE);
	//if(QEIDirectionGet(QEI0_BASE)==-1) pos0 = -(1000000-pos0);
	consigne0 = PIDHandler(&consigne0, &pos1, &I0, &previous_error0, 1/10); // TODO ou =output0
	//Motor 1
	pos1 = QEIPositionGet(QEI1_BASE);
	//if(QEIDirectionGet(QEI1_BASE)==-1) pos1 = -(1000000-pos1);
	consigne1 = PIDHandler(&consigne1, &pos1, &I1, &previous_error1, 1/10);
	//Motor 2
	pos2 = position_m2;
	consigne2 = PIDHandler(&consigne2, &pos2, &I2, &previous_error2, 1/10);
	//Motor 3
	pos3 = position_m3;
	consigne3 = PIDHandler(&consigne3, &pos3, &I3, &previous_error3, 1/10);
	
	//Vérifier si atteint la consigne
	if(pos0 > consigne0-tolerancePID && pos0 < consigne0+tolerancePID){
		motorBrake(0);
	}
	if(pos1 > consigne1-tolerancePID && pos1 < consigne1+tolerancePID){
		motorBrake(1);
	}
	if(pos2 > consigne2-tolerancePID && pos2 < consigne2+tolerancePID){
		motorBrake(2);
	}
	if(pos3 > consigne3-tolerancePID && pos3 < consigne3+tolerancePID){
		motorBrake(3);
	}
	
	//Vérifier si dépassé la consigne
	if(previous_error0 > 0){
		motorTurnCW(0);
	}
	else{
		motorTurnCCW(0);
	}
	if(previous_error1 > 0){
		motorTurnCW(1);
	}
	else{
		motorTurnCCW(1);
	}
	if(previous_error2 > 0){
		motorTurnCW(2);
	}
	else{
		motorTurnCCW(2);
	}
	if(previous_error3 > 0){
		motorTurnCW(3);
	}
	else{
		motorTurnCCW(3);
	}
	
}


/*
 * MOTEUR
 */

//Initilisation des ports et pins pour transmettre des commandes aux moteurs
void initMotorCommand(void){
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_5 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
	GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4 | GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
	
}

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorTurnCCW(unsigned short mnumber){
	switch(mnumber){
		case 0: //M1
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4);
		case 1: //M2
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6);
		case 2: //M3
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_7); //Inversé
		case 3: //M4
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_5); //Inversé
		default:
			return;
	}
}

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorTurnCW(unsigned short mnumber){
	switch(mnumber){
		case 0: //M1
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_5);
		case 1: //M2
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_7);
		case 2: //M3
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_5); //Inversé
		case 3: //M4
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4); //Inversé
		default:
			return;
	}
}

// Commande permet au moteur de changer de direction
/*void motorGoOpposite(unsigned short mnumber){
	switch(mnumber){
		case 0: //M1
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, ^(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5)));
		case 1: //M2
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, ^(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7)));
		case 2: //M3
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, ^(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7))); //Inversé
		case 3: //M4
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, ^(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5))); //Inversé
		default:
			return;
	}
}*/

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorBrake(unsigned short mnumber){
	switch(mnumber){
		case 0: //M1
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
			consigne0=0;
			I0=0;
			previous_error0=0;
			QEIPositionSet(QEI0_BASE, 0);
		case 1: //M2
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0x00);
			consigne1=0;
			I1=0;
			previous_error1=0;
			QEIPositionSet(QEI1_BASE, 0);
		case 2: //M3
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
			consigne2=0;
			I2=0;
			previous_error2=0;
			position_m2=0;
		case 3: //M4
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
			consigne3=0;
			I3=0;
			previous_error3=0;
			position_m3=0;
		default:
			return;
	}
}

// Freinage sec du moteur
// \param mnumber : numero du moteur
void motorHardBrake(unsigned short mnumber){
	switch(mnumber){
		case 0: //M1
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0xFF);
		case 1: //M2
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
		case 2: //M3
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xFF);
		case 3: //M4
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0xFF);
		default:
			return;
	}
}

// Freinage en pente du moteur
// \param pente : pente
void motorSlowBrake(unsigned long pente){
	slow_brake_pente = pente;
	slow_brake = true;
}

// Depart en pente du moteur
// \param pente : position
void motorSlowStart(unsigned long pente){
	slow_start_pente = pente;
	slow_start = true;
}

void moveLateral(long distance){
	if(distance > 0){ //A droite
		consigne1 = distance;
		consigne2 = -distance; //La rotation est inversé par rapport à l'autre roue et donc la distance est opposée
		motorTurnCW(1);
		motorTurnCCW(2);
	}
	if(distance < 0){ //A gauche
		consigne1 = distance;
		consigne2 = -distance;
		motorTurnCCW(1);
		motorTurnCW(2);
	}
}
void moveFront(long distance){
	if(distance > 0){ //Avance
		consigne0 = distance;
		consigne3 = -distance;
		motorTurnCW(0);
		motorTurnCCW(3);
	}
	else if(distance < 0){ //Recule
		consigne0 = distance;
		consigne3 = -distance;
		motorTurnCCW(0);
		motorTurnCW(3);
	}
}
void turn(long distance){
	if(distance > 0){//Si distance positive tourner dans le sens horaire
		motorTurnCW(0);
		motorTurnCW(1);
		motorTurnCW(2);
		motorTurnCW(3);
		consigne0 = distance;
		consigne1 = distance;
		consigne2 = distance;
		consigne3 = distance;
	}
	else if(distance < 0){ //Si distance négative tourner dans le sens anti-horaire
		motorTurnCCW(0);
		motorTurnCCW(1);
		motorTurnCCW(2);
		motorTurnCCW(3);
		consigne0 = distance;
		consigne1 = distance;
		consigne2 = distance;
		consigne3 = distance;
	}
}


/*
 * LED
 */
 
void initLED(void){
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
}

void openLED(void){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x40);
}

void closeLED(void){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x00);
}
