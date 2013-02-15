#include "timer.h"




void timerInit()
{
	 //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Enable du timer0 qu'on utilise pour les interupts
	SYSCTL_RCGC1_R  |= SYSCTL_RCGC1_TIMER0;  // enable du timer 0
    NVIC_EN0_R |= NVIC_EN0_INT19;

     // Configuration du Timer0 avec les interrupts
 	TIMER0_CFG_R |= TIMER_CFG_32_BIT_TIMER;  // timer en mode 32 bit
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // Periodic Timer mode
    TIMER0_TAILR_R = 16000; // valeur de registre du timer (1 us)
    TIMER0_IMR_R = TIMER_IMR_TATOIM;// Active l'interruption en mode time-out
    TIMER0_CTL_R = TIMER_CTL_TAEN;  // Part le timer
}

void BoucleDAttente(short Nombre_microseconde_Dattente)
{
	int i = 0;
	int N = Nombre_microseconde_Dattente;
	for(i = 0; i<N;i++)
	{
		SysCtlDelay(US);
	}
}

