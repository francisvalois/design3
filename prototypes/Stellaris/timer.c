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
//#include "ecran.h"
//#include "qei.h"
//#include "commande.h"

#define BUFFER_LEN          256

typedef struct {
    volatile short        buffer[BUFFER_LEN];   // buffer
    volatile long         read;  // prochain élément à lire
    volatile long         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales externes
//qei.c
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile long speed, position;
//commande.c
extern tBoolean is_waiting_for_y;
extern unsigned long captured_index;
extern volatile tBoolean is_drawing;
extern volatile short number_to_draw;
extern volatile short segment_to_draw;
//main.c
extern volatile unsigned long speed_table[200];
extern volatile unsigned long pos_table[200];
extern volatile CircularBuffer send_buffer;
extern volatile float dt;
//pwm
extern volatile unsigned long periodPWM, pulsewidth; //Période du PWM
//motor.c
extern tBoolean a_atteint_consigne;
extern tBoolean est_en_mouvement;

//Variables globales
volatile unsigned long index;
volatile long posqei1;
volatile long speedqei1;

void resetVariables(void);
void resetQEI(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void motorBrake(volatile long mnumber);
void motorHardBrake(volatile long mnumber);
void asservirMoteurs(void);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
void turn(long distance, long vitesse);

void draw(volatile short number);

//Fonction qui gère les interruption du timer et appele les fonctions d'asservissement
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //ANALYSE
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    
    
	/*speed = QEIVelocityGet(QEI0_BASE);
	position = QEIPositionGet(QEI0_BASE);
	speedqei1 = QEIVelocityGet(QEI1_BASE);
	posqei1 = QEIPositionGet(QEI1_BASE);
	speed_table[index]=speed;
	pos_table[index]=position;*/
	if(index==0){
		draw(20);
	}
	if(is_drawing && a_atteint_consigne && !est_en_mouvement){
		switch(number_to_draw){
			case 10:
				switch(segment_to_draw){ //segment a dessiner
					case 1: //1er segment
						moveFront(3613, 803);
						moveLateral(4817, 1070);
						segment_to_draw++; //Prochain mouvement faire prochain segment
						break;
					case 2:
						moveFront(-12042, 1338);
						segment_to_draw++;
						break;
					case 3:
						is_drawing = false; //Fin du dessin, reset les variables
						number_to_draw = 0;
						segment_to_draw = 0;
						break;
				}
				break;
			case 1:
				switch(segment_to_draw){
					case 1:
						moveFront(1806, 803);
						moveLateral(2408, 1070);
						segment_to_draw++;
						break;
					case 2:
						moveFront(-6021, 1338);
						segment_to_draw++;
						break;
					case 3:
						is_drawing = false;
						number_to_draw = 0;
						segment_to_draw = 0;
						break;
				}
				break;
			case 20:
				switch(segment_to_draw){
					case 1:
						moveFront(1806, 800);
						segment_to_draw++;
						break;
					case 2:
						moveLateral(9032, 800);
						segment_to_draw++;
						break;
					case 3:
						moveFront(-4787, 800);
						segment_to_draw++;
						break;
					case 4:
						moveFront(-7225, 800);
						moveLateral(-8972, 1000);
						segment_to_draw++;
						break;
				 	case 5:
						moveLateral(9032, 1150);
						segment_to_draw++;	
						break;
					case 6:
						is_drawing = false;
						number_to_draw = 0;
						segment_to_draw = 0;
						break;
				}
				break;
			case 2:
				switch(segment_to_draw){
					case 1:
						moveFront(1806, 800);
						segment_to_draw++;
						break;
					case 2:
						moveLateral(9032, 800);
						segment_to_draw++;
						break;
					case 3:
						moveFront(-4787, 800);
						segment_to_draw++;
						break;
					case 4:
						moveFront(-7225, 800);
						moveLateral(-8972, 1000);
						segment_to_draw++;
						break;
				 	case 5:
						moveLateral(9032, 1150);
						segment_to_draw++;	
						break;
					case 6:
						is_drawing = false;
						number_to_draw = 0;
						segment_to_draw = 0;
						break;
				}
				break;
		}
	}
	//Asservissement Moteurs
	asservirMoteurs();
	
	//Vérifer si timeout sur commande de déplacement
	/*if(is_waiting_for_y && index - 150 > captured_index){
		is_waiting_for_y=false;
	}*/
	
	index++;
	
	//FIN ANALYSE
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
}

//Fonctionne qui configure et initie le timerA0
void initTimer(void){

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()*dt); //Au 1/10 de sec


    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}
