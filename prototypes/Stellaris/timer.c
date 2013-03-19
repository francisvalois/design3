#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"

//Variables globales externes
//commande.c
extern tBoolean is_waiting_for_y;
extern volatile float dt;
extern volatile long position_m2, position_m3;

//Variables globales
volatile unsigned long index;
void resetVariables(void);

void asservirMoteurs(void);
void motorTurnCCW(volatile long mnumber);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
//commande.c
void draw(volatile long number);

volatile long test0;
volatile long test1;
volatile long test2;
volatile long test3;

//Fonction qui gère les interruption du timer et appele les fonctions d'asservissement
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //ANALYSE
	//GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    
    /*test0 = -QEIVelocityGet(QEI0_BASE)*QEIDirectionGet(QEI0_BASE); //inversé
	test1 = QEIVelocityGet(QEI1_BASE)*QEIDirectionGet(QEI1_BASE);
	test2 = position_m2;
	test3 = position_m3;*/
    
    
    /*if(index==0){
    	//moveFront(6400, 800);
    	moveLateral(6400, 800);
    	//draw(80);
    }*/

	//Asservissement Moteurs
	asservirMoteurs();
	
	//Vérifer si timeout sur commande de déplacement
	/*if(is_waiting_for_y && index - 150 > captured_index){
		is_waiting_for_y=false;
	}*/
	
	index++;
	
	//FIN ANALYSE
	//GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
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
