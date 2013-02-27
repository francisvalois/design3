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

volatile unsigned long state, state_m2, state_m3; //États des encodeurs (m2 = moteur2, m3=moteur3)
volatile unsigned long previous_status, previous_state_m2, previous_state_m3; //Pour le traitement des encodeurs
volatile long position_m0, position_m1, position_m2, position_m3; //Position des moteurs m2, m3
//volatile unsigned long speed_table[300];
//volatile unsigned long pos_table[300];

extern volatile float dt;

// IMPORTANT: Moteur 0 et 1 utilisent les QEI, tandis que les moteurs 2 et 3
//			  utilisent le decodeur logiciels plus bas.

void initQEI(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //PhA sur C4 et PhB sur C6
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	
	//init QEI0 du micro pour moteur 0
    GPIOPinConfigure(GPIO_PC4_PHA0);
    GPIOPinConfigure(GPIO_PC6_PHB0);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);  //Ph A
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);  //Ph B
    QEIDisable(QEI0_BASE);
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET|
	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1000000); //64 counts par révolution de moteur avec un ratio 100:1 
	QEIEnable(QEI0_BASE);
	QEIPositionSet(QEI0_BASE,0);
	QEIVelocityDisable(QEI0_BASE);
	QEIVelocityConfigure (QEI0_BASE, QEI_VELDIV_1, ROM_SysCtlClockGet()*dt);
	QEIVelocityEnable(QEI0_BASE);
	
	//init QEI1 du micro pour moteur 1
    GPIOPinConfigure(GPIO_PE3_PHA1);
    GPIOPinConfigure(GPIO_PE2_PHB1);
	GPIOPinTypeQEI(GPIO_PORTE_BASE, GPIO_PIN_3);  //Ph A
    GPIOPinTypeQEI(GPIO_PORTE_BASE, GPIO_PIN_2);  //Ph B
    QEIDisable(QEI1_BASE);
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET|
	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1000000); //64 counts par révolution de moteur avec un ratio 100:1 
	QEIEnable(QEI1_BASE);
	QEIPositionSet(QEI1_BASE,0);
	QEIVelocityDisable(QEI1_BASE);
	QEIVelocityConfigure (QEI1_BASE, QEI_VELDIV_1, ROM_SysCtlClockGet()*dt);
	QEIVelocityEnable(QEI1_BASE);
	
	//init decodeur fait a la mitaine pour moteur 2 et 3 (pins J4 et J5 pour moteur 2, J6 et J7 pour moteur 3)
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7);
	IntEnable(INT_GPIOE);
	
	//En cas que les QEI matérielles marchent pas
	/*GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7);
	IntEnable(INT_GPIOE);*/
	
	position_m0 = 0;
	position_m1 = 0;
	position_m2 = 0;
	position_m3 = 0;
}

//Gère les interruptions des encodeurs en quadrature des moteurs 2 et 3
void EncoderIntHandler(void){
	GPIOPinIntClear(GPIO_PORTE_BASE, 0xC3);
	state = GPIOPinRead(GPIO_PORTE_BASE, 0xC3);
}

//Decodeur logiciel pour traitement des encodeurs en quadratures
void EncoderHandler(){
	//encodeur moteur 2
	state_m2 = state & 0x03; //Pour le moteur 2, on observe seulement les pins E0 et E1
	if(state_m2 != previous_state_m2){
		switch(state_m2)
		{
			case 0x00:
				if(previous_state_m2==0x01)position_m2++;
				else position_m2--;
				break;
			case 0x01:
				if(previous_state_m2==0x03)position_m2++;
				else position_m2--;
				break;
			case 0x02:
				if(previous_state_m2==0x00)position_m2++;
				else position_m2--;
				break;
			case 0x03:
				if(previous_state_m2==0x02)position_m2++;
				else position_m2--;
				break;
		}
		previous_state_m2 = state_m2;
	}
	
	//encodeur moteur 3
	state_m3 = state & 0xC0; //Pour le moteur 3, on observe seulement les pins E6 et E7
	if(state_m3 != previous_state_m3){
		switch(state_m3)
		{
			case 0x00:
				if(previous_state_m3==0x40)position_m3++;
				else position_m3--;
				break;
			case 0x40:
				if(previous_state_m3==0xC0)position_m3++;
				else position_m3--;
				break;
			case 0x80:
				if(previous_state_m3==0x00)position_m3++;
				else position_m3--;
				break;
			case 0xC0:
				if(previous_state_m3==0x80)position_m3++;
				else position_m3--;
				break;
		}
		previous_state_m3 = state_m3;
	}
}

