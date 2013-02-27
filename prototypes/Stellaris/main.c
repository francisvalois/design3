// Pierre-Luc Buhler 910 098 468
// & Simon Grenier 910 102 197
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
//#include "uart.h"
//#include "commande.h"
//#include "timer.h"
//*****************************************************************************
//
// Main de Kinocto
//
//*****************************************************************************
#define BUFFER_LEN          256

//Type def
typedef struct {
    long        buffer[BUFFER_LEN];   // buffer
    int         read;  // prochain �l�ment � lire
    int         write;   // prochain endroit o� �crire
} CircularBuffer;

//Variables globales externes
extern volatile unsigned long state, state_m2, state_m3; //�tats des encodeurs (m2 = moteur2, m3=moteur3)
extern volatile unsigned long previous_status, previous_state_m2, previous_state_m3; //Pour le traitement des encodeurs
extern volatile long position_m2, position_m3; //Position des moteurs m2, m3
extern volatile long pos0, pos1, pos2, pos3;
extern volatile long previous_error0, previous_error1, previous_error2, previous_error3;
extern volatile float I0, I1, I2, I3;
extern volatile long consigne0, consigne1, consigne2, consigne3; 
extern volatile float Kd0, Ki0, Kp0, Kd1, Ki1, Kp1, Kd2, Ki2, Kp2, Kd3, Ki3, Kp3;
extern volatile float Kd0_s, Ki0_s, Kp0_s, Kd1_s, Ki1_s, Kp1_s, Kd2_s, Ki2_s, Kp2_s, Kd3_s, Ki3_s, Kp3_s;
extern volatile float Tf0, Tf1, Tf2, Tf3;
extern volatile float slow_brake_pente, slow_start_pente;
extern long tolerancePos;
extern tBoolean slow_brake, slow_start;
extern volatile long posqei1;
extern volatile long speedqei1;

//Variables globales
volatile CircularBuffer receive_buffer;
volatile CircularBuffer send_buffer;
volatile long position; // Position � faire afficher
volatile long speed; // Vitesse � faire afficher
volatile unsigned long speed_table[300];
volatile unsigned long pos_table[300];
volatile float dt;


//Fonction externe provenant de lcd.c: TODO supprimer et utiliser plut�t celles de ecran.c/.h
void init_lcd(void);
void write_position(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);
void afficher_pos(volatile unsigned long secondes);
void afficher_speed(volatile unsigned long secondes);

//D�claration des fonctions externes
//pwm.c
void initPWM(void);
//timer.c
void initTimer(void);
//antenne.c
void initAntenne(void);
void activateAntenneInt(void);
void deactivateAntenneInt(void);
void AntenneHandler(void);
//qei.c
void initQEI(void);
void EncoderIntHandler(void);
void EncoderHandler(void);
//commande.c
tBoolean CommandHandler(void);
void initMotorCommand(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void motorBrake(volatile long mnumber);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
void initLED(void);
//uart.c
void initUART(void);
void UARTHandler(void);


int main(void)
{
	//Set clock so it runs directly on the crystal
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    volatile unsigned long ulLoop;
    
    // Initialisation des ports
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    
    //Enable les GPIO pour les timings des interrupts et autres	
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

	//Mettre les interrupts du port E en haute prorit�s: �vite collisions avec QEI logiciel
	IntPrioritySet(INT_GPIOE,0); 
	//Activer les interruptions
	IntMasterEnable();

 
    //Initialisation des variables globales
    ulLoop = 0;
    position=0;
    speed=0;
    state=0;
    state_m2=0;
    state_m3=0;
    position_m2=0;
    position_m3=0;
    receive_buffer.read=0;
    receive_buffer.write=0;
    send_buffer.read=0;
    send_buffer.write=0;
    previous_error0=0;
    previous_error1=0;
    previous_error2=0;
    previous_error3=0;
    I0=0;
    I1=0;
    I2=0;
    I3=0;
    consigne0=0;
    consigne1=0;
    consigne2=0;
    consigne3=0;
    //Pour les d�placements rapides
    Kd0 = 0.1;
    Ki0 = 7;//5.64;
    Kp0 = 1.75;
    Tf0 = 0.1;
    Kd1 = 0.1;
    Ki1 = 7;//5.64;
    Kp1 = 1.75;
    Tf1 = 0.1;
    Kd2 = 0.1;
    Ki2 = 7;//5.64;
    Kp2 = 1.75;
    Tf2 = 0.1;
    Kd3 = 0.1;
    Ki3 = 7;//5.64;
    Kp3 = 1.75;
    Tf3 = 0.1;
    //Pour les mouvements lents
    Kd0_s = 0.05;
    Ki0_s = 10;//5.64;
    Kp0_s = 1.6;
    Kd1_s = 0.05;
    Ki1_s = 10;//5.64;
    Kp1_s = 1.5;
    Kd2_s = 0.05;
    Ki2_s = 10;//5.64;
    Kp2_s = 1.5;
    Kd3_s = 0.05;
    Ki3_s = 10;//5.64;
    Kp3_s = 1.6;
    dt = 0.1;
    slow_brake_pente = 1;
    slow_start_pente = 1;
    slow_brake = false;
    slow_start = false;
    tolerancePos = 0;
    posqei1 = 0;
    speedqei1=0;
    
    
    


	initMotorCommand();
	initPWM();
    //initUART();
	initLED();   
    initQEI();
    //init_lcd();
    initTimer();

    while(1)
	{
		EncoderHandler(); // Traitement des encodeurs en quadrature pour les moteurs 2 et 3
		//UARTHandler(); //G�re les traitements du UART
		//CommandHandler(); //Traitement des commandes
	}
}
