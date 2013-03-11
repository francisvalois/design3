//***********************************************************
//
//       Main de test pour décodage d'antenne
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

//***********************************************************
//
//    Defines et variables globales
//
//***********************************************************
#define TIMEOUT_VAL 5000 // periode du signal PWM pour test
#define MAX_TIMER_EVENTS 100  // taille de l'échantillion 
#define NB_ESSAIS_MAX 10 // nombre d'essais maximal pour lire le signal

static unsigned short g_usTimerBuf[MAX_TIMER_EVENTS]; // buffer qui contient les timings
static unsigned long g_ulTimer0BIntCount = 0; // compte d'erreur d'interrupt
static unsigned long g_uluDMAErrCount = 0; // compte d'erreur de DMA
volatile unsigned int g_bDoneFlag = 0; // flag de complétion de transfert DMA

//***********************************************************
//
// Table de contrôle du contrôleur DMA
//
//***********************************************************
#if defined(ewarm)
#pragma data_alignment=1024
unsigned char ucControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ucControlTable, 1024)
unsigned char ucControlTable[1024];
#else
unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));
#endif

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
// ISR pour le Timer1B. Est appelé seulement à la fin du tranfert DMA. 
// Les autres interrupts déclenchent des transferts DMA. 
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
// Initialisation du signal PWM utilisé pour test. 
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
// Fonction d'envoi de chaînes de caractères par UART
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
// Fonctions de transformation d'entier en chaînes de caractères
//
//*****************************************************************************
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

int main()
{
    int okDecode = 1;
    int okRead = 1;
    int nbEssais = 0;
    unsigned char bitsDecode[7];
	//
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
    //
    // Initialisation du UART
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_EVEN));   

    SetupSignalSource(); 
    timerInit();
    DMAinit();

    ROM_IntMasterEnable();

    //
    //   On fait 10 essais de décodage au maximum et on envoie le résultat sur le UART
    //

    while(okDecode == 1 && nbEssais < NB_ESSAIS_MAX)
    {
        unsigned short *signalBuffer;
        unsigned int *signalBufferSize;
        int nbEssaisLecture = 0;
        startTimer1B();
        while(okRead == 1 && nbEssaisLecture < NB_ESSAIS_MAX)
        {
            okRead = readSignal(signalBuffer, signalBufferSize);
            nbEssaisLecture++;
        }
        okDecode = decodeSignal(signalBuffer, *signalBufferSize, bitsDecode);
    }
    if(okDecode == 0)
    {
        unsigned char code = '0';
        UARTSend(&code,1);
    }
    else
    {
        unsigned char code = '1';
        UARTSend(&code,1);
    }
    UARTSend(bitsDecode,7);
    unsigned char fin = '\n';
    UARTSend(&fin, 1);
    while(1);
}