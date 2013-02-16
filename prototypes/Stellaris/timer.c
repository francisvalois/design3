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
#include "qei.h"

extern volatile unsigned long periodPWM; //Période du PWM
extern volatile unsigned long position_m2, position_m3; //
extern volatile unsigned long speed, position;


volatile unsigned long speed_table[200];
volatile unsigned long pos_table[200];
volatile unsigned long index;

//Fonction qui gère les interruption du timer
// TODO à modifier, ceci était seulement pour les tests
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	speed = QEIVelocityGet(QEI0_BASE);
	position = QEIPositionGet(QEI0_BASE);
	speed_table[index]=position_m2;
	pos_table[index]=position_m3;
	if(index==10){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/2);
	}
	if(index==50){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);
	}
	if(index==100){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/4);
	}
	index++;
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
