#include "inc/lm3s9b92.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
/*
 * LED
 */
 
void initLED(void){
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
}

void openLED(void){
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x10);
}

void closeLED(void){
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00);
}
