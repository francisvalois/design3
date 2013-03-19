#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

void initUART(void)
{
    //La version lâche.
	//
    // Initialize the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //NVIC_EN0_R |= NVIC_EN0_INT5;
    //IntMasterEnable();


    
    GPIOPinConfigure(GPIO_PA0_U0RX); //Pin 0 du port A utilisé pour le RX du UART 
    GPIOPinConfigure(GPIO_PA1_U0TX); //Pin 1 du port A utilisé pour le TX du UART
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //pin type du UART
    ROM_UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_EVEN));
    //IntEnable(INT_UART0);
    ROM_UARTEnable(UART0_BASE);
    UART0_LCRH_R |= UART_LCRH_EPS; // even parity select
    UART0_LCRH_R |= UART_LCRH_PEN; // parity enable
    UART0_LCRH_R |= UART_LCRH_WLEN_8; // word lenght 8 bits
    
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}
