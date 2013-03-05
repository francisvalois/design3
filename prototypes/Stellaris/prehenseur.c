#include "inc/lm3s9b92.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
/*
 * Préhenseur
 */

void initPrehenseur(void){
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);
}

void descendrePrehenseur(void){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);
}

void monterPrehenseur(void){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);
}