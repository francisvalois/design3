#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"

#define BUFFER_LEN          256

typedef struct {
    volatile long        buffer[BUFFER_LEN];   // buffer
    long         read;  // prochain élément à lire
    long         write;   // prochain endroit où écrire
} CircularBuffer;

/*
 * Sonar
 */

volatile unsigned long time_sonar;
volatile unsigned long previous_time_sonar;
volatile CircularBuffer sonarTimeDelta;
volatile unsigned long systick_time;
volatile tBoolean waiting_for_falling_edge;
volatile long patate;

void initSonar(void){
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_6);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
	ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);
	ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_BOTH_EDGES);
	ROM_GPIOPinIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
	//ROM_IntEnable(INT_GPIOA);
	
	
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
	ROM_SysTickEnable();
	systick_time = ROM_SysTickPeriodGet();
	
	
	time_sonar = 0;
	previous_time_sonar = 0;
	sonarTimeDelta.read = 0;
	sonarTimeDelta.write = 0;
	patate = 0;
	waiting_for_falling_edge = false;

}

void enableSonar(void){
	ROM_IntEnable(INT_GPIOA);
}

void disableSonar(void){
	ROM_IntDisable(INT_GPIOA);
}

void sonarIntHandler(void){
	time_sonar = ROM_SysTickValueGet();
	ROM_GPIOPinIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
	if(ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == GPIO_PIN_5 || !waiting_for_falling_edge){
		previous_time_sonar = time_sonar;
		waiting_for_falling_edge=true;
	}
	else if(ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0x00 || waiting_for_falling_edge){
		if(previous_time_sonar > time_sonar){
			sonarTimeDelta.buffer[sonarTimeDelta.write++%BUFFER_LEN] = previous_time_sonar - time_sonar;
		}
		else{
			sonarTimeDelta.buffer[sonarTimeDelta.write++%BUFFER_LEN] = previous_time_sonar + ROM_SysTickPeriodGet() - time_sonar;
		}
		waiting_for_falling_edge=false;
	}
	patate++;
}
