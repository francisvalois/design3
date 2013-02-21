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
    long        buffer[BUFFER_LEN];   // buffer
    int         read;  // prochain élément à lire
    int         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales externes
//qei.c
extern volatile unsigned long periodPWM; //Période du PWM
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile long speed, position;
//commande.c
extern volatile long slow_brake_pente, slow_start_pente;
extern tBoolean slow_brake, slow_start;
//main.c
extern volatile unsigned long speed_table[200];
extern volatile unsigned long pos_table[200];
extern volatile CircularBuffer send_buffer;

//Variables globales
volatile unsigned long index;

void asservirMoteurs(void);
void moveLateral(long distance);

//Fonction qui gère les interruption du timer et appele les fonctions d'asservissement
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	speed = QEIVelocityGet(QEI0_BASE);
	position = QEIPositionGet(QEI0_BASE);
	speed_table[index]=speed;
	pos_table[index]=position;
	/*if(index==10){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/2);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, periodPWM/2);
	}
	if(index==40){
		motorBrake(0);
		motorBrake(1);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0 |PWM_OUT_1, 0);
	}
	if(index==80){
		motorTurnCCW(0);
		motorTurnCCW(1);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/4);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, periodPWM/4);
	}*/
	if(index==10){
		moveLateral(2*6400);
	}
	if(index==300){
		long i=0;
		while(i<300){
			if(!(UART0_FR_R & UART_FR_TXFF)){
				pos_table[i] = 0xABCDEF12;
				pos_table[i] = pos_table[i] >> 8;
				UART0_DR_R = pos_table[i] >> 24;
				UART0_DR_R = pos_table[i] >> 16;
				UART0_DR_R = pos_table[i] >> 8;
				UART0_DR_R = pos_table[i];
				i++;
			}

		}
		i=0;
		while(i<300){
			if(!(UART0_FR_R & UART_FR_TXFF)){
				UART0_DR_R = speed_table[i];
				i++; 	
			}
		}
	}
	index++;
	//send_buffer.buffer[send_buffer.write%256] = position;
	//send_buffer.write++;
	
	//Asservissement Moteurs
	asservirMoteurs();
	//Changement graduelle de la vitesse de moteurs
	if(slow_brake){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, PWMPulseWidthGet(PWM_BASE, PWM_OUT_0) - periodPWM/slow_brake_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, PWMPulseWidthGet(PWM_BASE, PWM_OUT_1) - periodPWM/slow_brake_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, PWMPulseWidthGet(PWM_BASE, PWM_OUT_2) - periodPWM/slow_brake_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, PWMPulseWidthGet(PWM_BASE, PWM_OUT_3) - periodPWM/slow_brake_pente);
	}
	if(slow_start){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, PWMPulseWidthGet(PWM_BASE, PWM_OUT_0) + periodPWM/slow_start_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, PWMPulseWidthGet(PWM_BASE, PWM_OUT_1) + periodPWM/slow_start_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, PWMPulseWidthGet(PWM_BASE, PWM_OUT_2) + periodPWM/slow_start_pente);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, PWMPulseWidthGet(PWM_BASE, PWM_OUT_3) + periodPWM/slow_start_pente);
	}
}

//Fonctionne qui configure et initie le timerA0
void initTimer(void){

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/10); //Au 1/10 de sec


    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}
