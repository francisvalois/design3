//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2009-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 6288 of the EK-LM3S9B92 Firmware Package.
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include <string.h>

#include "inc/lm3s9b92.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the FTDI virtual serial port on the evaluation board) will be
//! configured in 115,200 baud, 8-n-1 mode.  All characters received on the
//! UART are transmitted back to the UART.
//
//*****************************************************************************

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

char inBuffer[24];
volatile unsigned long dataIn = 0;
volatile unsigned long dataRead = 0;

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        inBuffer[(dataIn++) % sizeof(inBuffer)] = UARTCharGet/*NonBlocking*/(UART0_BASE);
    }
}

//*****************************************************************************
//
// Send a string to the UART.
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
// Sends a 32-bit Int / long on the UART
//
//*****************************************************************************
void sendInt(long x)
{
	unsigned char bytes[4];
	
	bytes[0] = (x >> 24) & 0xFF;
	bytes[1] = (x >> 16) & 0xFF;
	bytes[2] = (x >> 8) & 0xFF;
	bytes[3] = x & 0xFF;
	
	UARTSend(bytes, 4);
	
}

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


void itoa(int n, unsigned char s[10])
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
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 19,200, 8-E-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_EVEN));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX/* | UART_INT_RT*/);

    //
    // Prompt for text to be entered.
    //

    //
    // Loop forever echoing data through the UART.
    //
    unsigned long testArray[] = {110,10,3,4,5,5,4,3,2,1};
    
    int compte = 0;

    while(1)
    {
    	if(compte == 3)
    	{
    		dataIn = 0;
    		dataRead = 0;
    		compte = 0;
    	}

    	if(dataIn - dataRead == 8 || dataRead - dataIn == 16)
    	{
    		dataRead = (dataRead + 8)%24;
    		int i;
    		const unsigned char nbColonnes = '2';
    		UARTSend(&nbColonnes,1);
    		for(i = 0;i < sizeof(testArray)/4; i++)
    		{
    			unsigned char data[10];
    			itoa(testArray[i],data);
    			UARTSend(data,10);

    		}
    		const unsigned char fin = '\n';
    		UARTSend(&fin,1);
    		compte++;
    	}	
    
    }
}

