#include "uart.h"

void uartInit()
{
    //La version lÃ¢che.
	//
    // Initialize the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    NVIC_EN0_R |= NVIC_EN0_INT5;
    IntMasterEnable();


    
    GPIOPinConfigure(GPIO_PA0_U0RX); //Pin 0 du port A utilisé pour le RX du UART 
    GPIOPinConfigure(GPIO_PA1_U0TX); //Pin 1 du port A utilisé pour le TX du UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //pin type du UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    //IntEnable(INT_UART0);
    UARTEnable(UART0_BASE);
    UART0_LCRH_R |= UART_LCRH_EPS; // even parity select
    UART0_LCRH_R |= UART_LCRH_PEN; // parity enable
    UART0_LCRH_R |= UART_LCRH_WLEN_8; // word lenght 8 bits
    
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

//la version des vrai!
/*void uartInit()
{
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;

    NVIC_EN0_R |= NVIC_EN0_INT5;

    GPIO_PORTA_AFSEL_R |= (GPIO_PIN_0 | GPIO_PIN_1);
    GPIO_PORTA_DR2R_R  |= (GPIO_PIN_0 | GPIO_PIN_1);
    GPIO_PORTA_PCTL_R  |= (GPIO_PIN_0 | GPIO_PIN_1);

    int IBRD = 16000000 / (16 * 192000); //5.208333
    int FBRD = (int)(0.20833 * 64 + 0.5); //14
    UART0_CTL_R  = 0X00;
    UART0_IBRD_R |= FBRD;
    UART0_FBRD_R |= IBRD;
    UART0_LCRH_R |= UART_LCRH_WLEN_8;
    UART0_IM_R   |= UART_IM_RXIM; 
   //REGISTRE DE MASQUE D'INTERRUPT UART0_MIS_R  |= 
    UART0_CTL_R  |= UART_CTL_UARTEN;

}*/

char uartLireOctet()
{
	while(UART0_FR_R & UART_FR_RXFE)
	{
		//attend un charactère
	}
	
	return UART0_DR_R;
}

void uartEcrireOctet(char octet)
{
	while(UART0_FR_R & UART_FR_TXFF)
	{
		//attend que la transmit FIFO se libère
	}
	
	UART0_DR_R = octet;
}
