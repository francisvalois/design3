#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
/*
 * Préhenseur
 */

void initPrehenseur(void){
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
	ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
}

void descendrePrehenseur(void){
	ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x20);
}

void monterPrehenseur(void){
	ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);
}
