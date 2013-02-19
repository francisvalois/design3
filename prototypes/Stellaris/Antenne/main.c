#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include <string.h>


//#include "ecran.h"
#include "uart.h"
#include "timer.h"
//#include "commandes.h"
//#include "main.h"




void reverse(char s[])
{
	int c, i ,j;
	for(i=0, j=strlen(s)-1; i<j; i++,j--)
	{
	c = s[i];
	s[i] = s[j];
	s[j] = c;
	}
}


void itoa(int n, char s[5])
{
	char srev[5];
	int lenght = 0;
	while(n > 0)
	{
		int a = n % 10;
		srev[lenght++] = a | '0';
		n /= 10;
	}
	while(lenght < 5)
	{
		srev[lenght++] = '0';
	}
	
	lenght--;
	int rev = 0;
	while(lenght >= 0)
	{
		s[rev++] = srev[lenght--];
	}
	
	s[rev] = '\0';
	
}

volatile unsigned int timerValue=0;
volatile unsigned int dataRecu = 0;
volatile unsigned int dataLues = 0;


char buffer[20];

void timerInterrupt()
{
	IntMasterDisable(); //disable les interuptions
	GPIO_PORTD_DATA_R |=GPIO_PIN_5;
	timerValue++;
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT; //reset le flag d'interruption
	TIMER0_TAILR_R = 16000; //on remet la valeur du timer pour qu'il reshoot à 1 us.
	BoucleDAttente(1000);
	GPIO_PORTD_DATA_R &= ~(GPIO_PIN_5);
	IntMasterEnable();
}

void uartInterrupt()
{
    //IntMasterDisable();
    unsigned long ulStatus;
    GPIO_PORTD_DATA_R |= GPIO_PIN_6;
	ulStatus = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ulStatus);
	while(UARTCharsAvail(UART0_BASE))
	{
		//UARTCharPutNonBlocking(UART0_BASE,
                                   //UARTCharGetNonBlocking(UART0_BASE));
		buffer[dataRecu++ % sizeof(buffer)] = UARTCharGetNonBlocking(UART0_BASE);
	} 
    GPIO_PORTD_DATA_R &= ~(GPIO_PIN_6);
}

int
main(void)
{

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //PortD qui est utilisé pour le contrôle de l'écran
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //PortE qui est utilisé pour l'affichage de l'écran.
    volatile unsigned long ulLoop;
    ulLoop = SYSCTL_RCGC2_R;
    
    /*  Port E : Affichage Ecran */
    GPIO_PORTE_DIR_R |= 0xFF; //direction, output
    GPIO_PORTE_DEN_R |= 0xFF; //Enable
    //GPIO_PORTE_PDR_R |= 0xFF; //resistance interne de pulldown apparemment c'est pas pertinant pour l'écran 
    
    /*Port D : Control Ecran et LED */
    GPIO_PORTD_DIR_R |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); //Direction les pins 0 1 2, 3, 5 ,6 et 7 sont des sortie. 4 est une entrée
    GPIO_PORTD_DEN_R |= 0xFF; //Enable 


    //GPIO_PORTD_DATA_R |= GPIO_PIN_0;
    uartInit();
	//ecranInit();
    //ecranClear();
    //ecranSetPosCursor(0x00); //on met le curseur au debut de la permière ligne!
    //ecranWriteLine("VC DF",5);
    //ecranSetPosCursor(0x40); // on met le curseur au debut de la deuxième ligne!
    timerInit();

    //enable d'initéruptions.
    //char position = 0x00;
	IntMasterEnable();
	int compte = 10;
    while(1)
    {
    	if(timerValue % 1000 == 0 && compte >= 0)
    	{
    		compte--;
    		char string1[5];
    		char string2[5];
		    itoa(timerValue, string);
		    itoa(compte, string2);
    	}
    	else
    	{
    		string = string1 + "$" + string2 + "\n";
    		int i;
    		for(i = 0; i < len(string); i++)
    		{
    			uartEcrireOctet(string[i]);
    		}
    	}

		if(dataRecu-dataLues==8) //truc qui indique que le buffer est plein.
		{
			char opcode, arg, chksum;
			opcode = buffer[dataLues++ % sizeof(buffer) ];
			arg = buffer[dataLues++ % sizeof(buffer)];
			int i;
			chksum = 0;
			//for(i=0; i<2; i++)
			//{
			//	chksum += buffer[dataRecu+i];
			//}
			//chksum = 0xff - chksum+1;
			/*
			 * Insert v�rification de checksum ici
			 */
			//if(chksum == 0) //todo 
			//{
			//position = commandeDecode(position,opcode,arg);
			//}
		}
		//else
		//{
		//dataLues=dataRecu;
		//}
		//dataLues = (dataLues+2)%20;
    }
}

