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
// Main pour la tranmission de donnée de l'ordinateur au UART
//
//*****************************************************************************
volatile unsigned long position; // Position à faire afficher
volatile unsigned long speed; // Position à faire afficher
extern volatile unsigned long count2, count3;
extern volatile unsigned long status, state, state2, state3;
volatile unsigned long width;
volatile unsigned long speed_table[200];
volatile unsigned long index;


void init_lcd(void);
void write_position(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);
void afficher_pos(volatile unsigned long secondes);
void afficher_speed(volatile unsigned long secondes);
void PWMInit(void);
void MotorTurnCCW(unsigned short);
void MotorTurnCW(unsigned short);
void MotorBrake(unsigned short);
void InitTimer(void);

void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	speed = ROM_QEIVelocityGet(QEI0_BASE);
	speed_table[index]=speed;
	index++;
}


void InitTimer(void){

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/10);


    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void)
{
    volatile unsigned long ulLoop;
    
    // Port GPIO J et D pour le LCD et les pins d'analyses du timing
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    
    //Enable les GPIO pour le LCD et l'analyse des timings des interrupts	
	ROM_GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, 0xFF, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	
	ROM_IntMasterEnable();
   
    initQEI();
    init_lcd();
    PWMInit();
    InitTimer();
    ulLoop = 0;
    width = 1;
    position=0;
    state=0;
    state2=0;
    state3=0;
    count2=0;
    count3=0;

    // Loop forever waiting for interrupt
    while(1)
	{
		EncoderHandler();
		//Analyse timing pour la boucle du main, Pin 3
		char etatOutputs;
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		etatOutputs = ROM_GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~etatOutputs);
		
		// Met à jour le temps affiché sur le LCD
		 
		/*for(index=0; 2000>index; index++){
			position = 6400 - QEIPositionGet(QEI0_BASE);
			speed = ROM_QEIVelocityGet(QEI0_BASE);
			speed_table[index]=speed;
			for(ulLoop=0; 100000>ulLoop; ulLoop++){}
		}*/
		//afficher_pos(position);
		//afficher_speed(speed);
		//ulLoop++;
		//if(ulLoop > 2000){
		//	ulLoop = 0;
		//	width++;
		//	ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, (ROM_SysCtlClockGet()/440/100)*(width%100));
		//}
		
		ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		etatOutputs = ROM_GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_3);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~etatOutputs);
	}
}
