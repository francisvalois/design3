//***********************************************************
//
//       antenne.c
//       Fonctions de décodage d'antenne
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

//*****************************************************************************
//
// Initialisation du TIMER1B 
//
//*****************************************************************************
static void initTimer1B()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//
    // Configure the Timer1 CCP3 function to use PB2
    //
    GPIOPinConfigure(GPIO_PB2_CCP3);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);
    //
    // Set up Timer1B for edge-timer mode, both edges
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(TIMER1_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    TimerLoadSet(TIMER1_BASE, TIMER_B, 0xffff);
}

//*****************************************************************************
//
// Initialisation du uDMA
//
//*****************************************************************************
static void initDMA()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	//
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    ROM_IntEnable(INT_UDMAERR);
    //
    // Enable the uDMA controller.
    //
    ROM_uDMAEnable();
    //
    // Point at the control table to use for channel control structures.
    //
    ROM_uDMAControlBaseSet(ucControlTable);
    //
    // Put the attributes in a known state for the uDMA Timer1B channel.  These
    // should already be disabled by default.
    //
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_TMR1B,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
    //
    // Configure DMA channel for Timer1B to transfer 16-bit values, 1 at a
    // time.  The source is fixed and the destination increments by 16-bits
    // (2 bytes) at a time.
    //
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_TMR1B | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_16 | UDMA_ARB_1);

    //
    // Set up the transfer parameters for the Timer1B primary control
    // structure.  The mode is set to basic, the transfer source is the
    // Timer1B register, and the destination is a memory buffer.  The
    // transfer size is set to a fixed number of capture events.
    //
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_TMR1B | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(TIMER1_BASE + TIMER_O_TBR),
                               g_usTimerBuf, MAX_TIMER_EVENTS);
}

//*****************************************************************************
//
// Démarrage du timer et du DMA
//
//*****************************************************************************
static void startTimer1B()
{
    TimerIntEnable(TIMER1_BASE, TIMER_CAPB_EVENT);
    TimerEnable(TIMER1_BASE, TIMER_B);
    IntEnable(INT_TIMER1B);

    //
    // Now enable the DMA channel for Timer1B.  It should now start performing
    // transfers whenever there is a rising edge detected on the CCP3 pin.
    //
    ROM_uDMAChannelEnable(UDMA_CHANNEL_TMR1B);
}

//*****************************************************************************
//
// Lecture du signal d'antenne 
// Retourne 0 si lecture correcte, 1 si erreur
//
//*****************************************************************************
static int readSignal(unsigned short *signalBuffer, unsigned int *signalBufferSize)
{
	unsigned long ulIdx;
    unsigned short usTimerElapsed;
	//
	//   Attend que le transfert DMA soit terminé
	//
	while(!g_bDoneFlag)
    {
    }
    // 
    //   Vérifie s'il y a eu des erreurs de transfert DMA
    //
    if(g_uluDMAErrCount != 0)
    {
        return 1;
    }
    //
    //   Vérifie s'il y a eu des erreurs du côté du timer
    //
    if(g_ulTimer0BIntCount != 1)
    {
        return 1;
    }


}

//*****************************************************************************
//
// Décodage du signal d'antenne 
// Retourne 0 si lecture correcte, 1 si erreur
//
//*****************************************************************************
static int decodeSignal(unsigned char signal[7])
{

}
