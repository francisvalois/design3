#ifndef ECRAN_H_
#define ECRAN_H_

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "inc/lm3s9b92.h"




//definition de trucs qu vont etre pratiques

//#define STCTRL (*((volatile unsigned long *)0xE000E010))
//#define STRELOAD (*((volatile unisgned long *)0xE000E014))
//#define attend(t) SysCtlDelay((SysCtlClock()/3)/(1000/t))


#define ECRAN_DATA GPIO_PORTE_DATA_R
#define ECRAN_CTRL GPIO_PORTA_DATA_R
//volatile unsigned long ulLoop;
	
//bit de data du LCD
#define ECRAN_D0 0x01 //PE0
#define ECRAN_D1 0x02 //PE1
#define ECRAN_D2 0x04 //PE2
#define ECRAN_D3 0x08 //PE3 
#define ECRAN_D4 0x10 //PE4
#define ECRAN_D5 0x20 //PE5
#define ECRAN_D6 0x40 //PE6
#define ECRAN_D7 0x80 //PE7

//bit de control du LCD
#define ECRAN_RS 0x01 //PA0  reset
#define ECRAN_RW 0x02 //PA1  read/write
#define ECRAN_EN 0x04 //PA2  enable
#define ECRAN_BF 0x08 //PA3  busy flag, connecter à la pin D7 de l'écran



//volatile unsigned long ulLoop;
void ecranClear(void);
void ecranInit(void);
void ecranWriteChar(char caractere);
void ecranWriteLine(char * line, unsigned short size);
void ecranSetPosCursor(short pos);
void ecranAttend(void);
void ecranControl(unsigned long input);



#endif /*ECRAN_H_*/
