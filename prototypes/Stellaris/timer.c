#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
//#include "ecran.h"
//#include "qei.h"
//#include "commande.h"

#define BUFFER_LEN          256

typedef struct {
    volatile short        buffer[BUFFER_LEN];   // buffer
    volatile long         read;  // prochain élément à lire
    volatile long         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales externes
//qei.c
extern volatile long position_m2, position_m3; //Position du moteur 2 et 3
extern volatile long speed, position;
//commande.c
extern tBoolean is_waiting_for_y;
extern unsigned long captured_index;
extern volatile tBoolean is_drawing;
extern volatile short number_drawn;
//main.c
extern volatile unsigned long speed_table[200];
extern volatile unsigned long pos_table[200];
extern volatile CircularBuffer send_buffer;
extern volatile float dt;
//pwm
extern volatile unsigned long periodPWM, pulsewidth; //Période du PWM

//Variables globales
volatile unsigned long index;
volatile long posqei1;
volatile long speedqei1;

void resetVariables(void);
void resetQEI(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void motorBrake(volatile long mnumber);
void motorHardBrake(volatile long mnumber);
void asservirMoteurs(void);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
void turn(long distance, long vitesse);

void draw(volatile short number);

//Fonction qui gère les interruption du timer et appele les fonctions d'asservissement
void TimerInt(void){

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
    //ANALYSE
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    
    
	/*speed = QEIVelocityGet(QEI0_BASE);
	position = QEIPositionGet(QEI0_BASE);
	speedqei1 = QEIVelocityGet(QEI1_BASE);
	posqei1 = QEIPositionGet(QEI1_BASE);
	speed_table[index]=speed;
	pos_table[index]=position;*/
	if(index==0){
		draw(3);
	}
	if(is_drawing && number_drawn == 10){
		if(index%250==0){
			moveFront(3613, 803);
			moveLateral(4817, 1070);
		}
		else if(index%250==100){
			moveFront(-12042, 1338);
		}
		else if(index%250==249){
			is_drawing = false;
			number_drawn = 0;
		}
	}
	else if(is_drawing && number_drawn == 1){
		if(index%250==0){
			moveFront(1806, 803);
			moveLateral(2408, 1070);
		}
		else if(index%250==100){
			moveFront(-6021, 1338);
		}
		else if(index%250==249){
			is_drawing = false;
			number_drawn = 0;
		}
	}
	else if(is_drawing && number_drawn == 20){
		if(index%600==0){
			moveFront(1806, 800);
		}
		else if(index%600==100){
			moveLateral(9032, 800);
		}
		else if(index%600==250){
			moveFront(-4787, 800);
		}
		else if(index%600==350){
			moveFront(-7225, 800);
			moveLateral(-8972, 1000);
		}
		else if(index%600==500){
			moveLateral(9032, 1150);	
		}
		else if(index%600==599){
			is_drawing = false;
			number_drawn = 0;
		}
	}
	else if(is_drawing && number_drawn == 3){
		if(index==0){
			moveFront(1806, 800);
		}
		else if(index==100){
			moveLateral(9032, 800);
		}
		else if(index==250){
			moveFront(-4787, 800);
		}
		else if(index==350){
			moveFront(-7225, 800);
			moveLateral(-8972, 1000);
		}
		else if(index==500){
			moveLateral(9032, 1150);	
		}
	}
	/*else if(index%250==100){
			moveFront(-12042, 1338);
	}*/
	/*else if(index%400==300){
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x20);
		//resetVariables();
		moveLateral(-6533, 800);
		//moveFront(6533, 800); //21.1cm
		//moveFront(6533, 800); //21.1cm

		//turn(15709,1600); //1 tours
	}*/
	/*if(index==50){
		resetVariables();
		//motorBrake(0);
		//motorTurnCW(0x01);
		motorHardBrake(0);
		motorHardBrake(1);
		motorHardBrake(2);
		motorHardBrake(3);
		turn(15709,1600);
	}if(index==200){
		resetVariables();
		motorHardBrake(0);
		motorHardBrake(1);
		motorHardBrake(2);
		motorHardBrake(3);
		moveFront(-3200, 800);
	}*/
	/*if(index==100){
		//motorBrake(1);
		//motorTurnCW(0x02);
		//GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0x00);
		//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x20);
	}
	if(index==150){
		motorBrake(2);
		motorTurnCW(0x03);
		//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
		//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x10);
	}
	if(index==200){
		motorBrake(0x03);
		//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x0);
	}*/
		
	/*if(index==10){
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/2);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, periodPWM/2);
	}
	if(index==40){
		motorBrake(0);
		motorBrake(1);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0 |PWM_OUT_1, 0);
	}
	if(index==80){
		motorTurnCCW(0);
		motorTurnCCW(1);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, periodPWM/4);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, periodPWM/4);
	}*/
	/*if(index==10){
		moveLateral(2*6400);
	}*/
	/*if(index%10==0){
		volatile long temp_speed;
		temp_speed = QEIPositionGet(QEI0_BASE);
		//long i=0;	
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN] = temp_speed >> 24;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN] = temp_speed >> 16;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN] = temp_speed >> 8;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN] = temp_speed;
	}*/
	index++;
	//send_buffer.buffer[send_buffer.write%256] = position;
	//send_buffer.write++;
	
	//Asservissement Moteurs
	asservirMoteurs();
	
	//Vérifer si timeout sur commande de déplacement
	/*if(is_waiting_for_y && index - 150 > captured_index){
		is_waiting_for_y=false;
	}*/
	
	//FIN ANALYSE
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
}

//Fonctionne qui configure et initie le timerA0
void initTimer(void){

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()*dt); //Au 1/10 de sec


    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}
