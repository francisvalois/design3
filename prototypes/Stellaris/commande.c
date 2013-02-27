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
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile unsigned long periodPWM; //Période du PWM
extern volatile unsigned long index;
extern volatile float dt;

//Variables globales
volatile long pos0, pos1, pos2, pos3;
volatile long previous_pos0, previous_pos1, previous_pos2, previous_pos3;
volatile long previous_speed0, previous_speed1, previous_speed2, previous_speed3;
volatile long measured_speed0, measured_speed1, measured_speed2, measured_speed3;
volatile long previous_error0, previous_error1, previous_error2, previous_error3;
volatile float I0, I1, I2, I3;
volatile long consigne0, consigne1, consigne2, consigne3; 
volatile float output0, output1, output2, output3;
volatile float output0_old, output1_old, output2_old, output3_old;
volatile float Kd0, Ki0, Kp0, Kd1, Ki1, Kp1, Kd2, Ki2, Kp2, Kd3, Ki3, Kp3;
volatile float Kd0_s, Ki0_s, Kp0_s, Kd1_s, Ki1_s, Kp1_s, Kd2_s, Ki2_s, Kp2_s, Kd3_s, Ki3_s, Kp3_s;
volatile float Tf0, Tf1, Tf2, Tf3;
volatile long dist_cible0, dist_cible1, dist_cible2, dist_cible3;
volatile float slow_brake_pente, slow_start_pente;
volatile unsigned long slow_brake_index, slow_start_index;
long tolerancePID;
long tolerancePos;
tBoolean slow_brake, slow_start;
tBoolean est_demi_consigne;
//Fichier qui contient des fonctions pour traiter les différents types de commandes à effectuer

long ouput0_table[10], ouput1_table[10], ouput2_table[10], ouput3_table[10];


//Pour tests
long pos0_table[300], pos1_table[300], pos2_table[300], pos3_table[300];
long speed0_table[300], speed1_table[300], speed2_table[300], speed3_table[300];
long output0_table[300], output1_table[300], output2_table[300], output3_table[300];
long fraction0_table[300], fraction1_table[300], fraction2_table[300], fraction3_table[300];

//Declaration de fonctions
//commande.c
void ajusterConsigne(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void motorBrake(volatile long mnumber);
void motorHardBrake(volatile long mnumber);
void ajustementVitesse(void);

/*
 * COMMANDE
 */

// Fonction qui gere les commandes
// \param command : commande à traiter
// \return booléen qui dit si la commande a été traitée avec succès.
tBoolean CommandHandler(){
	return true;
}

/*
 * PID/Asservissement
 */
 
 //
 void resetVariables(void){
 	I0=0;
 	I1=0;
 	I2=0;
 	I3=0;
 	consigne0=0;
 	consigne1=0;
 	consigne2=0;
 	consigne3=0;
 	output0=0;
 	output1=0;
 	output2=0;
 	output3=0;
 	output0_old=0;
 	output1_old=0;
 	output2_old=0;
 	output3_old=0;
 	dist_cible0=0;
 	dist_cible1=0;
 	dist_cible2=0;
 	dist_cible3=0;
 	slow_brake_pente=0;
 	slow_start_pente=0;
 	slow_brake=false;
 	slow_start=false;
 	previous_error0=0;
 	previous_error1=0;
 	previous_error2=0;
 	previous_error3=0;
 	pos0=0;
 	pos1=0;
 	pos2=0;
 	pos3=0;
 	previous_pos0=0;
 	previous_pos1=0;
 	previous_pos2=0;
 	previous_pos3=0;
 	previous_speed0=0;
 	previous_speed1=0;
  	previous_speed2=0;
   	previous_speed3=0;
   	measured_speed0=0;
    measured_speed1=0;
   	measured_speed2=0;
   	measured_speed3=0;
   	est_demi_consigne = false;
 }

//Filtre PID, ref: http://www.telecom-robotics.org/node/326

long PIDHandler0(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp0*error + Ki0*(*I) + Kd0*D; //*Tf/(1+Tf/dt);
  *previous_error = error;
  return output;
}
long PIDHandler1(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp1*error + Ki1*(*I) + Kd1*D;
  *previous_error = error;
  return output;
}
long PIDHandler2(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp2*error + Ki2*(*I) + Kd2*D;
  *previous_error = error;
  return output;
}
long PIDHandler3(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp3*error + Ki3*(*I) + Kd3*D;
  *previous_error = error;
  return output;
}
long SlowPIDHandler0(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp0_s*error + Ki0_s*(*I) + Kd0_s*D; //*Tf/(1+Tf/dt);
  *previous_error = error;
  return output;
}
long SlowPIDHandler1(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp1_s*error + Ki1_s*(*I) + Kd1_s*D;
  *previous_error = error;
  return output;
}
long SlowPIDHandler2(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp2_s*error + Ki2_s*(*I) + Kd2_s*D;
  *previous_error = error;
  return output;
}
long SlowPIDHandler3(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp3_s*error + Ki3_s*(*I) + Kd3_s*D;
  *previous_error = error;
  return output;
}

// Asservissement en vitesse des moteurs pour qu'il atteignent leur propre consigne
void asservirMoteurs(void){
	pos0 = position_m3; //pas inversé
	pos1 = pos1 + QEIVelocityGet(QEI1_BASE)*QEIDirectionGet(QEI1_BASE); //pas inversé
	pos2 = position_m2;//pas inversé
	pos3 =  pos3 - QEIVelocityGet(QEI0_BASE)*QEIDirectionGet(QEI0_BASE); //inversé
	
	ajustementVitesse();
	
	//Motor 0
	measured_speed0 =  (pos0 - previous_pos0)/dt;
	if(measured_speed0 < 0){
		measured_speed0 = -measured_speed0;
	}
	previous_pos0 = pos0;
	if(consigne0 > 3200){
		output0 = PIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	else{
		output0 = SlowPIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	//Motor 1
	measured_speed1 = QEIVelocityGet(QEI1_BASE)/dt;//(pos1 - previous_pos1)*10;
	if(measured_speed1 < 0){
		measured_speed1 = -measured_speed1;
	}
	previous_pos1 = pos1;
	if(consigne1 > 3200){
		output1 = PIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	else{
		output1 = SlowPIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	//Motor 2
	measured_speed2 = (pos2 - previous_pos2)/dt;
	if(measured_speed2 < 0){
		measured_speed2 = -measured_speed2;
	}
	previous_pos2 = pos2;
	if(consigne2 > 3200){
		output2 = PIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	else{
		output2 = SlowPIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	//Motor 3
	measured_speed3 = QEIVelocityGet(QEI0_BASE)/dt;//(pos3 - previous_pos3)*10;
	if(measured_speed3 < 0){
		measured_speed3 = -measured_speed3;
	}
	previous_pos3 = pos3;
	if(consigne3 > 3200){
		output3 = PIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}
	else{
		output3 = SlowPIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}
	
	/*output0 = output0_old +(dt/Tf0)*(output0-output0_old);
	output1 = output1_old +(dt/Tf1)*(output1-output1_old);
	output2 = output2_old +(dt/Tf2)*(output2-output2_old);
	output3 = output3_old +(dt/Tf3)*(output3-output3_old);
	output0_old = output0;
	output1_old = output1;
	output2_old = output2;
	output3_old = output3;*/
	
	//output0_table[index%10] = output0;
	//output1_table[index%10] = output1;
	//output2_table[index%10] = output2;
	//output3_table[index%10] = output3;

	
	//Traduction 6400e de tour fraction appliqué au pulse width
	float fraction0;
	float fraction1;
	float fraction2;
	float fraction3;
	//Une équation linéaire est utilisée x*0.5/7700 = % du duty cycle
	fraction0 = ((output0*0.5)/7700);
	if(fraction0 > 0.99){
		fraction0 = 0.99;
	}
	else if(fraction0 < 0){
		fraction0 = 0;
	}
	fraction1 = ((output1*0.5)/7700);
	if(fraction1 > 0.99){
		fraction1 = 0.99;
	}else if(fraction1 < 0){
		fraction1 = 0;
	}
	fraction2 = ((output2*0.5)/7700);
	if(fraction2 > 0.99){
		fraction2 = 0.99;
	}else if(fraction2 < 0){
		fraction2 = 0;
	}
	fraction3 = ((output3*0.5)/7700);
	if(fraction3 > 0.99){
		fraction3 = 0.99;
	}else if(fraction3 < 0){
		fraction3 = 0;
	}
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, (periodPWM*fraction0));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, (periodPWM*fraction1));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, (periodPWM*fraction2));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, (periodPWM*fraction3));
	
	pos0_table[index]=pos0;
	pos1_table[index]=pos1;
	pos2_table[index]=pos2;
	pos3_table[index]=pos3;
	speed0_table[index]=measured_speed0;
	speed1_table[index]=measured_speed1;
	speed2_table[index]=measured_speed2;
	speed3_table[index]=measured_speed3;
	output0_table[index]=output0;
	output1_table[index]=output1;
	output2_table[index]=output2;
	output3_table[index]=output3;
	fraction0_table[index]=fraction0;
	fraction1_table[index]=fraction1;
	fraction2_table[index]=fraction2;
	fraction3_table[index]=fraction3;
	
}


//Fonction qui ajuste la vitesse selon la position.
void ajustementVitesse(void){
	long abs_pos3, abs_pos1, abs_dist_cible3, abs_dist_cible1;
	if(pos3 < 0){
		abs_pos3 = - pos3;
	}
	else{
		abs_pos3 = pos3;
	}
	if(dist_cible3 < 0){
		abs_dist_cible3 = - dist_cible3;
	}
	else{
		abs_dist_cible3 = dist_cible3;
	}
	if(pos1 < 0){
		abs_pos1 = - pos1;
	}
	else{
		abs_pos1 = pos1;
	}
	if(dist_cible1 < 0){
		abs_dist_cible1 = - dist_cible1;
	}
	else{
		abs_dist_cible1 = dist_cible1;
	}
	/*if(!est_demi_consigne && (((abs_dist_cible3 != 0) && ((abs_dist_cible3 - abs_pos3) < 3000)) || ((abs_dist_cible1 != 0) && ((abs_dist_cible1 - abs_pos1) < 3000)))){
		float demi_consigne0=consigne0*0.5;
		float demi_consigne1=consigne1*0.5;
		float demi_consigne2=consigne2*0.5;
		float demi_consigne3=consigne3*0.5;
		consigne0=(long)(demi_consigne0);
		consigne1=(long)(demi_consigne1);
		consigne2=(long)(demi_consigne2);
		consigne3=(long)(demi_consigne3);
		est_demi_consigne = true;
	}*/
	if((abs_dist_cible3 != 0 && abs_pos3 > (abs_dist_cible3-tolerancePos))
			|| (abs_dist_cible1 !=0 && abs_pos1 > (abs_dist_cible1-tolerancePos))){
		motorHardBrake(0);
		motorHardBrake(1);
		motorHardBrake(2);
		motorHardBrake(3);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 0);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);
		//resetVariables();
	}
}


/*
 * MOTEUR
 */

//Initilisation des ports et pins pour transmettre des commandes aux moteurs
void initMotorCommand(void){
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xF0);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_5 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
	GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4 | GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
	
}

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorTurnCCW(volatile long mnumber){
	if(mnumber == 0){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x10);
	}
	else if(mnumber == 1){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0x40);
	}
	else if(mnumber == 2){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x80); //Inversé
	}
	else if(mnumber == 3){
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x20); //Inversé
	}
}

// Commandes a transmettre aux moteurs
// \param mnumber : numero du moteur
void motorTurnCW(volatile long mnumber){
	if(mnumber == 0){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x20);
	}
	else if(mnumber == 1){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0x80);
	}
	else if(mnumber == 2){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x20); //Inversé
	}
	else if(mnumber == 3){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x10); //Inversé
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
void motorBrake(volatile long mnumber){
	if(mnumber == 0){ //M1
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
		consigne0=0;
		I0=0;
		previous_error0=0;
		dist_cible0=0;
		//position_m3=0;
	}
	else if(mnumber == 1){ //M2
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0x00);
		consigne1=0;
		I1=0;
		previous_error1=0;
		dist_cible1=0;
		//QEIPositionSet(QEI1_BASE, 0);
	}
	else if(mnumber == 2){ //M3
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
		consigne2=0;
		I2=0;
		previous_error2=0;
		dist_cible2=0;
		//position_m2=0;
	}
	else if(mnumber == 3){//M4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
		consigne3=0;
		I3=0;
		previous_error3=0;
		dist_cible3=0;
		//QEIPositionSet(QEI0_BASE, 0);
	}
}

// Freinage sec du moteur
// \param mnumber : numero du moteur
void motorHardBrake(volatile long mnumber){
	if(mnumber == 0){ //M1
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
		consigne0=0;
		I0=0;
		previous_error0=0;
		dist_cible0=0;
		position_m3=0;
	}
	else if(mnumber == 1){ //M2
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0xC0);
		consigne1=0;
		I1=0;
		previous_error1=0;
		dist_cible1=0;
		QEIPositionSet(QEI1_BASE, 0);
	}
	else if(mnumber == 2){ //M3
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
		consigne2=0;
		I2=0;
		previous_error2=0;
		dist_cible2=0;
		position_m2=0;
	}
	else if(mnumber == 3){//M4
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
		consigne3=0;
		I3=0;
		previous_error3=0;
		dist_cible3=0;
		QEIPositionSet(QEI0_BASE, 0);
	}
}

// Freinage en pente du moteur
// \param pente : pente
void motorSlowBrake(unsigned long pente){
	slow_brake_pente = pente;
	slow_brake_index = pente;
	slow_brake = true;
}

// Depart en pente du moteur
// \param pente : position
void motorSlowStart(unsigned long pente){
	slow_start_pente = pente;
	slow_brake_index = pente;
	slow_start = true;
}

void moveLateral(long distance, long vitesse){
	if(distance > 0){ //A droite
		dist_cible1 = distance;
		dist_cible2 = -distance; //La rotation est inversé par rapport à l'autre roue et donc la distance est opposée
		consigne1=vitesse;
		consigne2=vitesse;
		motorTurnCW(1);
		motorTurnCCW(2);
	}
	if(distance < 0){ //A gauche
		dist_cible1 = distance;
		dist_cible2 = -distance;
		consigne1=vitesse;
		consigne2=vitesse;
		motorTurnCCW(1);
		motorTurnCW(2);
	}
}
void moveFront(long distance, long vitesse){
	if(distance > 0){ //Avance
		dist_cible0 = distance;
		dist_cible3 = -distance;
		consigne0=vitesse;
		consigne3=vitesse;
		motorTurnCW(0);
		motorTurnCCW(3);
	}
	else if(distance < 0){ //Recule
		dist_cible0 = distance;
		dist_cible3 = -distance;
		consigne0=vitesse;
		consigne3=vitesse;
		motorTurnCCW(0);
		motorTurnCW(3);
	}
}
void turn(long distance, long vitesse){
	if(distance > 0){//Si distance positive tourner dans le sens horaire
		motorTurnCW(0);
		motorTurnCW(1);
		motorTurnCW(2);
		motorTurnCW(3);
		consigne0=vitesse;
		consigne1=vitesse;
		consigne2=vitesse;
		consigne3=vitesse;
		dist_cible0 = distance;
		dist_cible1 = distance;
		dist_cible2 = distance;
		dist_cible3 = distance;
	}
	else if(distance < 0){ //Si distance négative tourner dans le sens anti-horaire
		motorTurnCCW(0);
		motorTurnCCW(1);
		motorTurnCCW(2);
		motorTurnCCW(3);
		consigne0=vitesse;
		consigne1=vitesse;
		consigne2=vitesse;
		consigne3=vitesse;
		dist_cible0 = distance;
		dist_cible1 = distance;
		dist_cible2 = distance;
		dist_cible3 = distance;
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
