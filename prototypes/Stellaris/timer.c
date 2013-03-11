#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

//Variables globales externes
//commande.c
extern tBoolean is_waiting_for_y;
extern volatile float dt;

//Variables globales
volatile unsigned long index;

void asservirMoteurs(void);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
//commande.c
void draw(volatile short number);

//Fonction qui gère les interruption du timer et appele les fonctions d'asservissement
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //ANALYSE
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    
	if(index==0){
		draw(8);
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
