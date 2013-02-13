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
//#include "ecran.h"
//*****************************************************************************
//
// Main pour la tranmission de donnée de l'ordinateur au UART
//
//*****************************************************************************
volatile unsigned long position; // Position à faire afficher
volatile unsigned long speed; // Position à faire afficher

void init_lcd(void);
void write_position(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);
void afficher_pos(volatile unsigned long secondes);
void afficher_speed(volatile unsigned long secondes);

int main(void)
{
    volatile unsigned long ulLoop;
    
    // Port GPIO J et D pour le LCD et les pins d'analyses du timing
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    
    //Enable les GPIO pour le LCD et l'analyse des timings des interrupts	
	ROM_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	
	//QEI
    GPIOPinConfigure(GPIO_PC4_PHA0);
    GPIOPinConfigure(GPIO_PC6_PHB0);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);  //Ph A
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);  //Ph B
    QEIDisable(QEI0_BASE);
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET|
	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 6399);
	QEIEnable(QEI0_BASE);
	QEIPositionSet(QEI0_BASE,0);
	ROM_QEIVelocityDisable(QEI0_BASE);
	ROM_QEIVelocityConfigure (QEI0_BASE, QEI_VELDIV_1, 10000);
	ROM_QEIVelocityEnable(QEI0_BASE);
   
    init_lcd();

    // Loop forever waiting for interrupt
    while(1)
	{
		//Analyse timing pour la boucle du main, Pin 3
		char etatOutputs;
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		etatOutputs = ROM_GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~etatOutputs);
		
		// Met à jour le temps affiché sur le LCD
		position = QEIPositionGet(QEI0_BASE);
		speed = ROM_QEIVelocityGet(QEI0_BASE);
		//position = 1320;
		afficher_pos(position);
		afficher_speed(speed);
		
		
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		etatOutputs = ROM_GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~etatOutputs);
	}
}
