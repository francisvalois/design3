#include "qei.h"
volatile unsigned long state, state_m2, state_m3; //États des encodeurs (m2 = moteur2, m3=moteur3)
volatile unsigned long previous_status, previous_state_m2, previous_state_m3; //Pour le traitement des encodeurs
volatile unsigned long position_m2, position_m3; //Position des moteurs m2, m3

// IMPORTANT: Moteur 0 et 1 utilisent les QEI, tandis que les moteurs 2 et 3
//			  utilisent le decodeur logiciels plus bas.

void initQEI(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //PhA sur C4 et PhB sur C6
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	
	//init QEI0 du micro pour moteur 0
    GPIOPinConfigure(GPIO_PC4_PHA0);
    GPIOPinConfigure(GPIO_PC6_PHB0);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);  //Ph A
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);  //Ph B
    QEIDisable(QEI0_BASE);
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET|
	QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 6400); //64 counts par révolution de moteur avec un ratio 100:1 
	QEIEnable(QEI0_BASE);
	QEIPositionSet(QEI0_BASE,0);
	QEIVelocityDisable(QEI0_BASE);
	QEIVelocityConfigure (QEI0_BASE, QEI_VELDIV_1, ROM_SysCtlClockGet()/10);
	QEIVelocityEnable(QEI0_BASE);
	
	//init decodeur fait a la mitaine pour moteur 2 et 3 (pins J4 et J5 pour moteur 2, J6 et J7 pour moteur 3)
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	IntEnable(INT_GPIOJ);
	
	position_m2 = 0;
	position_m3 = 0;
}

//Gère les interruptions des encodeurs en quadrature des moteurs 2 et 3
void EncoderIntHandler(void){
	GPIOPinIntClear(GPIO_PORTJ_BASE, 0xF0);
	state = GPIOPinRead(GPIO_PORTJ_BASE, 0xF0);
}

//Decodeur logiciel pour traitement des encodeurs en quadratures
void EncoderHandler(){
	//encodeur moteur 2
	state_m2 = state & 0x30; //Pour le moteur 2, on observe seulement les pins J4 et J5
	if(state_m2 != previous_state_m2){
		switch(state_m2)
		{
			case 0x00:
				if(previous_state_m2==0x10)position_m2++;
				else position_m2--;
				break;
			case 0x10:
				if(previous_state_m2==0x30)position_m2++;
				else position_m2--;
				break;
			case 0x20:
				if(previous_state_m2==0x00)position_m2++;
				else position_m2--;
				break;
			case 0x30:
				if(previous_state_m2==0x20)position_m2++;
				else position_m2--;
				break;
		}
		previous_state_m2 = state_m2;
	}
	
	//encodeur moteur 3
	state_m3 = state & 0xC0; //Pour le moteur 3, on observe seulement les pins J6 et J7
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

