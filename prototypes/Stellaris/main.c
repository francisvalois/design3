// Pierre-Luc Buhler 910 098 468
// & Simon Grenier 910 102 197
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
//*****************************************************************************
//
// Main de Kinocto
//
//*****************************************************************************

//Variables globales
volatile unsigned long position; // Position à faire afficher
volatile unsigned long speed; // Vitesse à faire afficher

//Variables globales extern
extern volatile unsigned long position_m2, position_m3; //
extern volatile unsigned long status, state, state_m2, state_m3;
extern volatile unsigned long speed_table[200];
extern volatile unsigned long pos_table[200];


//Fonction externe provenant de lcd.c: TODO supprimer et utiliser plutôt celles de ecran.c/.h
void init_lcd(void);
void write_position(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);
void afficher_pos(volatile unsigned long secondes);
void afficher_speed(volatile unsigned long secondes);

//Déclaration des fonctions externes
//pwm.c
void initPWM(void);
//timer.c
void initTimer(void);
//commande.c
void initMotorCommand(void);
tBoolean handleCommand(volatile unsigned char* command);
void initMotorCommand(void);
void motorTurnCCW(unsigned short mnumber);
void motorTurnCW(unsigned short mnumber);
void motorBrake(unsigned short mnumber);
void initLED(void);
void openLED(void);
void closeLED(void);
//antenne.c
void initAntenne(void);
void activateAntenneInt(void);
void deactivateAntenneInt(void);
void AntenneHandler(void);


int main(void)
{
    volatile unsigned long ulLoop;
    
    // Port GPIO J, D et E pour le LCD et les pins d'analyses du timing
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    
    //Enable les GPIO pour le LCD et l'analyse des timings des interrupts	
	ROM_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);

	//Activer les interruptions
	IntMasterEnable();
   
    //Initialisation des périphériques
    initQEI();
    //init_lcd();
    initPWM();
    initTimer();
 
    //Initialisation des variables globales
    ulLoop = 0;
    position=0;
    speed=0;
    state=0;
    state_m2=0;
    state_m3=0;
    position_m2=0;
    position_m3=0;

    while(1)
	{
		EncoderHandler(); // Traitement des encodeurs en quadrature
	}
}
