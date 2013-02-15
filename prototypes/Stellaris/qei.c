#include "qei.h"
// 2 = moteur2, 3 = moteur3
volatile unsigned long status, state, state2, state3;
volatile unsigned long previous_status, previous_state2, previous_state3;
volatile unsigned long count2, count3;

// IMPORTANT: Moteur 0 et 1 utilisent les QEI, tandis que les moteurs 2 et 3
//			  utilisent les decodeurs codes plus bas.

void initQEI(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //PhA sur C4 et PhB sur C6
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	
	//init QEI du micro pour moteur 0
    GPIOPinConfigure(GPIO_PC4_PHA0);
    GPIOPinConfigure(GPIO_PC6_PHB0);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);  //Ph A
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);  //Ph B
    QEIDisable(QEI0_BASE);
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET|
	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 6399); //64 counts par révolution de moteur avec un ratio 100:1 
	QEIEnable(QEI0_BASE);
	QEIPositionSet(QEI0_BASE,0);
	QEIVelocityDisable(QEI0_BASE);
	QEIVelocityConfigure (QEI0_BASE, QEI_VELDIV_1, 1000000);
	QEIVelocityEnable(QEI0_BASE);
	
	//init decodeur fait a la mitaine pour moteur 2 et 3
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	IntEnable(INT_GPIOJ);
	
	count2 = 0;
	count3 = 0;
}
void EncoderIntHandler(void){
	//status = GPIOPinIntStatus(GPIO_PORTJ_BASE, false);
	//GPIOPinIntClear(GPIO_PORTJ_BASE, status);
	//status = ~(previous_status | status); //Effectue un toggle sur
	GPIOPinIntClear(GPIO_PORTJ_BASE, 0xF0);
	state = GPIOPinRead(GPIO_PORTJ_BASE, 0xF0);
}

void EncoderHandler(){
	//encodeur moteur 2
	state2 = state & 0x30;
	if(state2 != previous_state2){
		switch(state2) //Regarde les pins PJ4 et PJ5 pour l'encodeur du moteur 2
		{
			case 0x00:
				if(previous_state2==0x10)count2++;
				else count2--;
				break;
			case 0x10:
				if(previous_state2==0x30)count2++;
				else count2--;
				break;
			case 0x20:
				if(previous_state2==0x00)count2++;
				else count2--;
				break;
			case 0x30:
				if(previous_state2==0x20)count2++;
				else count2--;
				break;
		}
		previous_state2 = state2;
	}
	
	//encodeur moteur 3
	state3 = state & 0xC0;
	if(state3 != previous_state3){
		switch(state3) //Regarde les pins PJ6 et PJ7 pour l'encodeur du moteur 3
		{
			case 0x00:
				if(previous_state3==0x40)count3++;
				else count3--;
				break;
			case 0x40:
				if(previous_state3==0xC0)count3++;
				else count3--;
				break;
			case 0x80:
				if(previous_state3==0x00)count3++;
				else count3--;
				break;
			case 0xC0:
				if(previous_state3==0x80)count3++;
				else count3--;
				break;
		}
		previous_state3 = state3;
	}
}
/*
void MotorTurnCCW(unsigned short){
}

void MotorTurnCW(unsigned short){
}

void MotorBrake(unsigned short){
}*/
