#include "gestionAntenne.h"
#define TRUE 1
#define FALSE 0
/*
 * Pin PA5 utilisé 
 */
 //15666;
volatile int demi_periode = 25000;
volatile unsigned char tableau_bits[80];
volatile unsigned int nb_bits_recu = 0;
volatile unsigned char tableau_bits_t1[40];
volatile unsigned char tableau_bits_t2[40];
volatile unsigned int compteur = 0;
volatile unsigned int periode = 0;
volatile char mot;
volatile long test;


volatile char int_timer_en_cours = FALSE;

/*void traiter_mot_recu(char mot_recu)
{
	lcd_clear();
	lcd_goto(0x00);
	switch((mot_recu & 0x2))
	{
		case 0:
		{
			lcd_sendString("F: petit");
			break;
		}
		case 0x2:
		{
			lcd_sendString("F: grand");
			break;
		}
		default:
		{
			break;
		}
	}
	switch(((mot_recu & 0xC) >> 2))
	{
		case 0:
		{
			lcd_sendString(" O: NORD");
			break;
		}
		case 0x1:
		{
			lcd_sendString(" O: EST");
			break;
		}
		case 0x2:
		{
			lcd_sendString(" O: SUD");
			break;
		}
		case 0x3:
		{
			lcd_sendString(" O: OUEST");
			break;
		}
		default:
		{
			break;
		}
	}
	lcd_goto(0x40);
	switch(((mot_recu & 0x70) >> 4))
	{
		case 0:
		{
			lcd_sendString("Sudocube: 0");
			break;
		}
		case 0x1:
		{
			lcd_sendString("Sudocube: 1");
			break;
		}
		case 0x2:
		{
			lcd_sendString("Sudocube: 2");
			break;
		}
		case 0x3:
		{
			lcd_sendString("Sudocube: 3");
			break;
		}
		case 0x4:
		{
			lcd_sendString("Sudocube: 4");
			break;
		}
		case 0x5:
		{
			lcd_sendString("Sudocube: 5");
			break;
		}
		case 0x6:
		{
			lcd_sendString("Sudocube: 6");
			break;
		}
		case 0x7:
		{
			lcd_sendString("Sudocube: 7");
			break;
		}
		default:
		{
			break;
		}
	}
}*/

void antenne_Initialiser()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
	//GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	//ROM_IntEnable(INT_GPIOB);
	//GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
	//NVIC_EN0_R |= NVIC_EN0_INT1; //Active les interruptions pour le port A
	//GPIO_PORTB_IBE_R = GPIO_PIN_2; //Interrupt Both edges pin 5
	//GPIO_PORTA_IM_R |= GPIO_PIN_5;
	IntPrioritySet(INT_GPIOB, 0x04);//Set priorité plus haute que uart et pi
	//desactiverInterruptionsManchester();
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFFFFFF); //Au 1/10 de sec
    //ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	//SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER1; //Enable du timer 0
	//TIMER1_CFG_R |= TIMER_CFG_32_BIT_TIMER;
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; //Périodique
	TIMER1_TAILR_R = 0xFFFFFFFF;
	TIMER1_CTL_R = TIMER_CTL_TAEN;
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;//Debug
	//GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_7);//Debug
	mot = 0x00;
	test = 0;
}

void activerInterruptionsManchester()
{
	GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_IntEnable(INT_GPIOB);
	ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	//GPIO_PORTB_IM_R |= GPIO_PIN_2;
}

void desactiverInterruptionsManchester()
{
	ROM_TimerDisable(TIMER1_BASE, TIMER_A);
	ROM_IntDisable(INT_GPIOB);
	GPIOPinIntDisable(GPIO_PORTB_BASE, GPIO_PIN_2);
	//GPIO_PORTB_IM_R &= ~GPIO_PIN_2;
}

void IntAntenne(){
	volatile static unsigned int temp = 0;
	volatile static unsigned int temp_avant = 0;
	volatile static unsigned int data[64];
	volatile static unsigned int nb_bits = 0;
	volatile char mot_recu = 0;
	activerInterruptionsManchester();
	volatile unsigned int i = 0;
	for(i = 0; i < 150; i++){}
	if(periode == 0)
	{
		if(compteur == 0 && (GPIO_PORTB_DATA_R & 0x04) == 0x04)
		{
			TIMER1_TAV_R = 0;
			compteur++;
		}
		else
		{
			temp = TIMER1_TAV_R;
			if(temp < 7600 && temp > 4000 && temp < temp_avant + 500 && temp_avant - 500 < temp)
			{
				periode = temp + (10400 - temp)/10;
			}
			else
			{
				compteur = 0;
				temp_avant = temp;
			}
		}
	}
	else
	{
		if((GPIO_PORTB_DATA_R & 0x04) == 0x04)
		{// 0
			TIMER1_TAV_R = 0;
			//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_7, 0xFF);
			while(TIMER1_TAV_R < periode){}
			//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_7, 0x00);
			if((GPIO_PORTB_DATA_R & 0x04) == 0x04)
			{// 0
				data[nb_bits] = 1;
				nb_bits++;
			}
			else
			{// 1
				data[nb_bits] = 0;
				nb_bits++;
			}
		}
		else
		{// 1
			TIMER1_TAV_R = 0;
			//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_7, 0xFF);
			while(TIMER1_TAV_R < periode){}
			//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_7, 0x00);
			if((GPIO_PORTB_DATA_R & 0x04) == 0x04)
			{// 0
				data[nb_bits] = 1;
				nb_bits++;
			}
			else
			{// 1
				data[nb_bits] = 0;
				nb_bits++;
			}
		}
		if(nb_bits >= 64)
		{
			volatile int nb_de_un = 0;
			volatile int i = 0;
			for(i = 0; i < 56; i++)
			{
				if(data[i] == 1)
				{
					nb_de_un++;
				}
				else
				{
					nb_de_un = 0;
				}
				if(nb_de_un >= 8)
				{
					if(data[i + 1] == 0)
					{
						volatile int j = 0;
						for(j = 0; j < 7; j++)
						{
							if(data[i + 2 + j] == 1)
							{
								mot_recu = mot_recu << 1;
								mot_recu++;
							}
							else
							{
								mot_recu = mot_recu << 1;
							}
						}
						mot = mot_recu;
						/*traiter_mot_recu(mot_recu);
						UARTCharPut(UART0_BASE, mot_recu);//Envoi du mot décodé vers le PC
						UARTCharPut(UART0_BASE, mot_recu);//Envoi du mot décodé vers le PC
						UARTCharPut(UART0_BASE, mot_recu);//Envoi du mot décodé vers le PC
						UARTCharPut(UART0_BASE, mot_recu);//Envoi du mot décodé vers le PC
						desactiverInterruptionsManchester();
						interrupt_StartTimer();//Redémarre l'asservissement*/
						i = 64;
						test = 1;
					}
				}
			}
			nb_bits = 0;
			periode = 0;
			compteur = 0;
		}
	}
	GPIO_PORTB_ICR_R = 0xFF;//Clear l'interrupt!
}
