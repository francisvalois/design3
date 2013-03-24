#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
//Variables globales externes
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile unsigned long periodPWM; //Période du PWM
extern volatile unsigned long index; //temps en ms/index du timer


//Variables globales
volatile long pos0, pos1, pos2, pos3; //Position des moteurs
volatile long previous_pos0, previous_pos1, previous_pos2, previous_pos3;
volatile long previous_speed0, previous_speed1, previous_speed2, previous_speed3;
volatile long measured_speed0, measured_speed1, measured_speed2, measured_speed3;
volatile long previous_error0, previous_error1, previous_error2, previous_error3;
volatile float I0, I1, I2, I3; //Integrale de l'asservissement
volatile long consigne0, consigne1, consigne2, consigne3; //Consigne de vitesse
volatile long dist_cible0, dist_cible1, dist_cible2, dist_cible3; //Consigne de position
volatile long output0, output1, output2, output3; //Commande
volatile long offset, offset2;
//Variable de PID
volatile float Kd0, Ki0, Kp0, Kd1, Ki1, Kp1, Kd2, Ki2, Kp2, Kd3, Ki3, Kp3;
volatile float Kd0_m, Ki0_m, Kp0_m, Kd1_m, Ki1_m, Kp1_m, Kd2_m, Ki2_m, Kp2_m, Kd3_m, Ki3_m, Kp3_m;
volatile float Kd0_s, Ki0_s, Kp0_s, Kd1_s, Ki1_s, Kp1_s, Kd2_s, Ki2_s, Kp2_s, Kd3_s, Ki3_s, Kp3_s;
volatile float Kd0_d, Ki0_d, Kp0_d, Kd1_d, Ki1_d, Kp1_d, Kd2_d, Ki2_d, Kp2_d, Kd3_d, Ki3_d, Kp3_d;
volatile float Tf0, Tf1, Tf2, Tf3; //Cste du filtre de commande de sortie
volatile float dt; //periode d'asservissement
volatile long time = 0;
tBoolean est_demi_consigne;
tBoolean est_en_mouvement;
tBoolean a_atteint_consigne;

//Pour tests
long pos0_table[300], pos1_table[300], pos2_table[300], pos3_table[300];
long speed0_table[300], speed1_table[300], speed2_table[300], speed3_table[300];
long output0_table[300], output1_table[300], output2_table[300], output3_table[300];
long fraction0_table[300], fraction1_table[300], fraction2_table[300], fraction3_table[300];

//Declaration de fonctions
//moteur.c
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void ajustementVitesse(void);
//qei
void resetQEIs(void);

/*
 * PID/Asservissement moteur
 */
 
 /*
  * Fonction qui reset les variables utilisé par l'asservissement
  */
 void resetVariables(void){
 	I0=0;
 	I1=0;
 	I2=0;
 	I3=0;
 	consigne0=0;
 	consigne1=0;
 	consigne2=0;
 	consigne3=0;
 	time=0;
 	output0=0;
 	output1=0;
 	output2=0;
 	output3=0;
 	dist_cible0=0;
 	dist_cible1=0;
 	dist_cible2=0;
 	dist_cible3=0;
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
   	est_en_mouvement = false;
   	a_atteint_consigne = true;
   	offset=0;
   	offset2=-640;
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
long SPIDHandler0(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp0_s*error + Ki0_s*(*I) + Kd0_s*D; //*Tf/(1+Tf/dt);
  *previous_error = error;
  return output;
}
long SPIDHandler1(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp1_s*error + Ki1_s*(*I) + Kd1_s*D;
  *previous_error = error;
  return output;
}
long SPIDHandler2(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp2_s*error + Ki2_s*(*I) + Kd2_s*D;
  *previous_error = error;
  return output;
}
long SPIDHandler3(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp3_s*error + Ki3_s*(*I) + Kd3_s*D;
  *previous_error = error;
  return output;
}
long MPIDHandler0(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp0_m*error + Ki0_m*(*I) + Kd0_m*D; //*Tf/(1+Tf/dt);
  *previous_error = error;
  return output;
}
long MPIDHandler1(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp1_m*error + Ki1_m*(*I) + Kd1_m*D;
  *previous_error = error;
  return output;
}
long MPIDHandler2(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp2_m*error + Ki2_m*(*I) + Kd2_m*D;
  *previous_error = error;
  return output;
}
long MPIDHandler3(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp3_m*error + Ki3_m*(*I) + Kd3_m*D;
  *previous_error = error;
  return output;
}
long DPIDHandler0(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp0_d*error + Ki0_d*(*I) + Kd0_d*D; //*Tf/(1+Tf/dt);
  *previous_error = error;
  return output;
}
long DPIDHandler1(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp1_d*error + Ki1_d*(*I) + Kd1_d*D;
  *previous_error = error;
  return output;
}
long DPIDHandler2(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp2_d*error + Ki2_d*(*I) + Kd2_d*D;
  *previous_error = error;
  return output;
}
long DPIDHandler3(volatile long *consigne, volatile long *measured_value, volatile float *I, volatile long *previous_error, float dt)
{
  long error = *consigne - *measured_value;
  *I = *I + error*dt;
  float D = (error - *previous_error)/dt;
  long output = Kp3_d*error + Ki3_d*(*I) + Kd3_d*D;
  *previous_error = error;
  return output;
}

// Asservissement en vitesse des moteurs pour qu'il atteignent leur propre consigne
void asservirMoteurs(void){
	pos0 = pos0 - QEIVelocityGet(QEI0_BASE)*QEIDirectionGet(QEI0_BASE); //inversé
	pos1 = pos1 + QEIVelocityGet(QEI1_BASE)*QEIDirectionGet(QEI1_BASE);
	pos2 = position_m2;
	pos3 = position_m3;
	
	ajustementVitesse();
	
	//Motor 0
	measured_speed0 =  (pos0 - previous_pos0)/dt;
	if(measured_speed0 < 0){
		measured_speed0 = -measured_speed0;
	}
	previous_pos0 = pos0;
	if(consigne0 > 4800){
		output0 = PIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	else if(consigne0 > 2400){
		output0 = MPIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	else if(consigne0 > 1200){
		output0 = SPIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	else{
		output0 = DPIDHandler0(&consigne0, &measured_speed0, &I0, &previous_error0, dt);
	}
	//Motor 1
	measured_speed1 = QEIVelocityGet(QEI1_BASE)/dt;//(pos1 - previous_pos1)*10;
	if(measured_speed1 < 0){
		measured_speed1 = -measured_speed1;
	}
	previous_pos1 = pos1;
	if(consigne1 > 4800){
		output1 = PIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	else if(consigne1 > 2400){
		output1 = MPIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	else if(consigne1 > 1200){
		output1 = SPIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	else{
		output1 = DPIDHandler1(&consigne1, &measured_speed1, &I1, &previous_error1, dt);
	}
	//Motor 2
	measured_speed2 = (pos2 - previous_pos2)/dt;
	if(measured_speed2 < 0){
		measured_speed2 = -measured_speed2;
	}
	previous_pos2 = pos2;
	if(consigne2 > 4800){
		output2 = PIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	else if(consigne2 > 2400){
		output2 = MPIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	else if(consigne2 > 1200){
		output2 = SPIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	else{
		output2 = DPIDHandler2(&consigne2, &measured_speed2, &I2, &previous_error2, dt);
	}
	//Motor 3
	measured_speed3 = QEIVelocityGet(QEI0_BASE)/dt;//(pos3 - previous_pos3)*10;
	if(measured_speed3 < 0){
		measured_speed3 = -measured_speed3;
	}
	previous_pos3 = pos3;
	if(consigne3 > 4800){
		output3 = PIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}
	else if(consigne3 > 2400){
		output3 = MPIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}
	else if(consigne3 > 1200){
		output3 = SPIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}
	else{
		output3 = DPIDHandler3(&consigne3, &measured_speed3, &I3, &previous_error3, dt);
	}

	
	//Traduction 6400e de tour fraction appliqué au pulse width
	float fraction0;
	float fraction1;
	float fraction2;
	float fraction3;
	//Une équation linéaire est utilisée x*0.5/7700 = % du duty cycle
	fraction0 = (((output0+offset)*0.5)/7700);
	if(fraction0 > 0.99){
		fraction0 = 0.99;
	}
	else if(fraction0 < 0){
		fraction0 = 0;
	}
	fraction1 = (((output1+offset)*0.5)/7700);
	if(fraction1 > 0.99){
		fraction1 = 0.99;
	}else if(fraction1 < 0){
		fraction1 = 0;
	}
	fraction2 = (((output2+offset2)*0.5)/7700);
	if(fraction2 > 0.99){
		fraction2 = 0.99;
	}else if(fraction2 < 0){
		fraction2 = 0;
	}
	fraction3 = (((output3+offset)*0.5)/7700);
	if(fraction3 > 0.99){
		fraction3 = 0.99;
	}else if(fraction3 < 0){
		fraction3 = 0;
	}
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, (periodPWM*fraction0));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, (periodPWM*fraction1));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, (periodPWM*fraction2));
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, (periodPWM*fraction3));
	
	//Si le robot est immobile
	if(a_atteint_consigne && measured_speed0==0 && measured_speed1==0 && measured_speed2==0 && measured_speed3==0){
		resetVariables();
		resetQEIs();
		ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);//periodPWM / 4);
		ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);
		ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 0);
		ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);
		est_en_mouvement = false;
	}
	else{
		est_en_mouvement = true;
	}
	pos0_table[index%300]=pos0;
	pos1_table[index%300]=pos1;
	pos2_table[index%300]=pos2;
	pos3_table[index%300]=pos3;
	speed0_table[index%300]=measured_speed0;
	speed1_table[index%300]=measured_speed1;
	speed2_table[index%300]=measured_speed2;
	speed3_table[index%300]=measured_speed3;
	output0_table[index%300]=output0;
	output1_table[index%300]=output1;
	output2_table[index%300]=output2;
	output3_table[index%300]=output3;
	fraction0_table[index%300]=fraction0;
	fraction1_table[index%300]=fraction1;
	fraction2_table[index%300]=fraction2;
	fraction3_table[index%300]=fraction3;
	
}


//Fonction qui ajuste la vitesse selon la position.
void ajustementVitesse(void){
	long abs_pos0, abs_pos1, abs_dist_cible0, abs_dist_cible1;
	if(pos0 < 0){
		abs_pos0 = - pos0;
	}
	else{
		abs_pos0 = pos0;
	}
	if(dist_cible0 < 0){
		abs_dist_cible0 = - dist_cible0;
	}
	else{
		abs_dist_cible0 = dist_cible0;
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
	
	if(abs_pos0 > abs_dist_cible0 -4000 && abs_dist_cible0 != 0 ){
		if(time >= 0){//0ms depuis que abs_pos0 > abs_dist_cible0 -4000
			if(consigne0 >4000 && consigne3 >4000){
				consigne0 = consigne0*0.875;
				consigne3 = consigne3*0.875;
			}
			else if(consigne0 >1400 && consigne0 <4000 && consigne3 <4000&&
			consigne3 >1400 && abs_pos0 < abs_dist_cible0 -1500){
				consigne0 = consigne0;
				consigne3 = consigne3;
			}
			else if(consigne0 >1400 && consigne0 <4000 && consigne3 <4000&&
			consigne3 >1400 && abs_pos0 > abs_dist_cible0 -1500){
				consigne0 = consigne0*0.875;
				consigne3 = consigne3*0.875;
			}
			else{
				consigne0 = 1200;
				consigne3 = 1200;
			}
		}
		time++;
	
	
		if((abs_dist_cible0 != 0 && abs_pos0 > (abs_dist_cible0)) || (abs_dist_cible1 !=0 && abs_pos1 > (abs_dist_cible1))){ 
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xF0);
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
				a_atteint_consigne = true;
		}
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
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
	GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4 | GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
	resetVariables();
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
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x80); //inversé
	}
	else if(mnumber == 3){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x20);
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
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x20); //inversé
	}
	else if(mnumber == 3){
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x10);
	}
}

void moveLateral(long distance, long vitesse){
	a_atteint_consigne = false;
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
	a_atteint_consigne = false;
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
	a_atteint_consigne = false;
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


