#include "uart.h"

void initUART(void)
{
    //La version lâche.
	//
    // Initialize the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //NVIC_EN0_R |= NVIC_EN0_INT5;
    //IntMasterEnable();


    
    GPIOPinConfigure(GPIO_PA0_U0RX); //Pin 0 du port A utilisé pour le RX du UART 
    GPIOPinConfigure(GPIO_PA1_U0TX); //Pin 1 du port A utilisé pour le TX du UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //pin type du UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_EVEN));
    //IntEnable(INT_UART0);
    UARTEnable(UART0_BASE);
    UART0_LCRH_R |= UART_LCRH_EPS; // even parity select
    UART0_LCRH_R |= UART_LCRH_PEN; // parity enable
    UART0_LCRH_R |= UART_LCRH_WLEN_8; // word lenght 8 bits
    
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}
