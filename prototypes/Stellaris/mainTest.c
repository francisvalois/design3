//***********************************************************
//
//       Main de test pour dÈcodage d'antenne
//       2013-03-10
//        
//
//***********************************************************
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "antenne.h"
#include <string.h>

#define MAX_TIMER_EVENTS 100  // taille de l'Èchantillion 
#define TIMEOUT_VAL 5000 // periode du signal PWM pour test
#define NB_ESSAIS_MAX 10 // nombre d'essais maximal pour lire le signal

static unsigned short g_usTimerBuf[MAX_TIMER_EVENTS];
static unsigned long g_ulTimer0BIntCount = 0; // compte d'erreur d'interrupt
static unsigned long g_uluDMAErrCount = 0; // compte d'erreur de DMA
static volatile unsigned int g_bDoneFlag = 0; // flag de complÈtion de transfert DMA


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    unsigned long ulStatus;

    //
    // Check for uDMA error bit
    //
    ulStatus = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ulStatus)
    {
        ROM_uDMAErrorStatusClear();
        g_uluDMAErrCount++;
    }
}

//*****************************************************************************
//
// ISR pour le Timer1B. Est appel√© seulement √† la fin du tranfert DMA. 
// Les autres interrupts d√©clenchent des transferts DMA. 
//
//*****************************************************************************
void
Timer0BIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_CAPB_EVENT);

    //
    // Read the uDMA channel status to verify it is done
    //
    ulStatus = uDMAChannelModeGet(UDMA_CHANNEL_TMR1B);
    if(ulStatus == UDMA_MODE_STOP)
    {
        //
        // Disable the capture timer and set the done flag
        //
        ROM_TimerDisable(TIMER1_BASE, TIMER_B);
        g_bDoneFlag = 1;
    }

    //
    // Increment a counter to indicate the number of times this handler
    // was invoked
    //
    g_ulTimer0BIntCount++;
}

//*****************************************************************************
//
// Initialisation du signal PWM utilis√© pour test. 
// On doit connecter les broches PD0 et PB2 pour effectuer le test.
//
//*****************************************************************************
static void
SetupSignalSource(void)
{
    //
    // Enable the GPIO port used for PWM0 output.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Enable the PWM peripheral and configure the GPIO pin
    // used for PWM0.  GPIO pin D0 is used for PWM0 output.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    GPIOPinConfigure(GPIO_PD0_PWM0);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);

    //
    // Configure the PWM0 output to generate a 50% square wave with a
    // period of 5000 processor cycles.
    //
    PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, TIMEOUT_VAL);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, TIMEOUT_VAL / 2);
    PWMOutputState(PWM_BASE, PWM_OUT_0_BIT, 1);
    PWMGenEnable(PWM_BASE, PWM_GEN_0);
}

//*****************************************************************************
//
// Fonction d'envoi de cha√Ænes de caract√®res par UART
//
//*****************************************************************************

void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPut/*NonBlocking*/(UART0_BASE, *pucBuffer++);
    }
}

//*****************************************************************************
//
// Fonctions de transformation d'entier en cha√Ænes de caract√®res
//
//*****************************************************************************
void reverseChaine(char s[])
{
	int c, i ,j;
	for(i=0, j=strlen(s)-1; i<j; i++,j--)
	{
	c = s[i];
	s[i] = s[j];
	s[j] = c;
	}
}
void itoa(unsigned short n, unsigned char s[10])
{
	char srev[10];
	int lenght = 0;
	while(n > 0)
	{
		int a = n % 10;
		srev[lenght++] = a | '0';
		n /= 10;
	}
	while(lenght < 10)
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

//*****************************************************************************
//
//                             Main de test
//
//*****************************************************************************

void getAntenne(void)
{
    int okDecode = 1;
    //int okRead = 1;
    int nbEssais = 0;
    //unsigned char bitsDecode[7];
	//
    // Set the clocking to run directly from the crystal.
    //

    SetupSignalSource(); 
    initTimer1B();
    initDMA(g_usTimerBuf);
    TimerIntEnable(TIMER1_BASE, TIMER_CAPB_EVENT);
    IntEnable(INT_TIMER1B);

    ROM_IntMasterEnable();

    //
    //   On fait 10 essais de dÈcodage au maximum et on envoie le rÈsultat sur le UART
    //
    const unsigned char nbColonnes = '1';
    UARTSend(&nbColonnes,1);

    while(okDecode == 1 && nbEssais < NB_ESSAIS_MAX)
    {
    	startTimer1B();
    	unsigned long ulIdx;
    	unsigned short usTimerElapsed;
    	while(!g_bDoneFlag)
    	{
    	}
    	if(g_uluDMAErrCount != 0)
    	{
        	continue;
    	}
    	if(g_ulTimer0BIntCount != 1)
    	{
        	continue;
    	}
    	for(ulIdx = 1; ulIdx < MAX_TIMER_EVENTS; ulIdx++)
    	{
    		if(g_usTimerBuf[ulIdx] == g_usTimerBuf[ulIdx - 1])
        	{
        		continue;
        	}
        	usTimerElapsed = g_usTimerBuf[ulIdx - 1] - g_usTimerBuf[ulIdx];
        	unsigned char data[10];
        	itoa(usTimerElapsed,data);
        	UARTSend(data,10);
    	}
    	nbEssais++;
        //unsigned short *signalBuffer;
        //unsigned int *signalBufferSize;
        //startTimer1B();
        //while(!g_bDoneFlag)
    	//{
    	//}
        //if(g_uluDMAErrCount != 0)
    	//{
        //	continue;
    	//}
    	//if(g_ulTimer0BIntCount != 1)
    	//{
        //	continue;
    	//}

        //readSignal(&g_usTimerBuf, signalBuffer, signalBufferSize);
        g_bDoneFlag = 0;
    	g_ulTimer0BIntCount = 0;
        //okDecode = decodeSignal(signalBuffer, *signalBufferSize, bitsDecode);
    }
    //if(okDecode == 0)
    //{
    //    unsigned char code = '0';
    //    UARTSend(&code,1);
    //}
    //else
    //{
    //    unsigned char code = '1';
    //    UARTSend(&code,1);
    //}
    //UARTSend(bitsDecode,7);
    unsigned char fin = '\n';
    UARTSend(&fin, 1);
    while(1);
    IntDisable(INT_TIMER1B);
}
