// Pierre-Luc Buhler 910 098 468
// & Simon Grenier 910 102 197
#include "inc/lm3s9b92.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "ecran.h"
//*****************************************************************************
//
// Main de Kinocto
//
//*****************************************************************************
#define BUFFER_LEN          256

//Type def
typedef struct {
    volatile long        buffer[BUFFER_LEN];   // buffer
    long         read;  // prochain élément à lire
    long         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales externes
extern unsigned long state, state_m2, state_m3; //États des encodeurs (m2 = moteur2, m3=moteur3)
extern unsigned long previous_status, previous_state_m2, previous_state_m3; //Pour le traitement des encodeurs
extern long position_m2, position_m3; //Position des moteurs m2, m3
extern long pos0, pos1, pos2, pos3;
extern long previous_error0, previous_error1, previous_error2, previous_error3;
extern float I0, I1, I2, I3;
extern long consigne0, consigne1, consigne2, consigne3; 
extern float Kd0, Ki0, Kp0, Kd1, Ki1, Kp1, Kd2, Ki2, Kp2, Kd3, Ki3, Kp3;
extern float Kd0_m, Ki0_m, Kp0_m, Kd1_m, Ki1_m, Kp1_m, Kd2_m, Ki2_m, Kp2_m, Kd3_m, Ki3_m, Kp3_m;
extern float Kd0_s, Ki0_s, Kp0_s, Kd1_s, Ki1_s, Kp1_s, Kd2_s, Ki2_s, Kp2_s, Kd3_s, Ki3_s, Kp3_s;
extern float Kd0_d, Ki0_d, Kp0_d, Kd1_d, Ki1_d, Kp1_d, Kd2_d, Ki2_d, Kp2_d, Kd3_d, Ki3_d, Kp3_d;
extern float dt;
extern volatile CircularBuffer buffer_commande;
extern tBoolean a_atteint_consigne;
extern tBoolean est_en_mouvement;
extern long number_to_draw;
extern long segment_to_draw;
extern tBoolean is_drawing;
extern volatile long offset, offset2;

//Variables globales
volatile CircularBuffer receive_buffer;
volatile CircularBuffer send_buffer;
volatile tBoolean is_waiting_for_action;





//Fonction externe provenant de lcd.c: TODO supprimer et utiliser plutôt celles de ecran.c/.h
void init_lcd(void);
void afficher_param(char sudocube, char orient, char T);
void write_position(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);

//Déclaration des fonctions externes
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
//command.c
void initCommande(void);
void traiterNouvelleCommande(long commande_a_traiter[8]);
tBoolean CommandHandler(void);
//moteur.c
void initMotorCommand(void);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
//uart.c
void initUART(void);
//prehenseur.c
void initPrehenseur(void);
void descendrePrehenseur(void);
void monterPrehenseur(void);
//led.c
void initLED(void);
//sonar.c
void initSonar(void);

int main(void)
{
	//Set clock so it runs directly on the crystal
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    volatile unsigned long ulLoop;
    
    // Initialisation des ports
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    
    //Enable les GPIO pour les timings des interrupts et autres	
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);

	//Mettre les interrupts du port E en haute prorités: évite collisions avec QEI logiciel
	ROM_IntPrioritySet(INT_GPIOE,0); 
	//Activer les interruptions
	ROM_IntMasterEnable();

 
    //Initialisation des variables globales
    ulLoop = 0;
    receive_buffer.read=0;
    receive_buffer.write=0;
    send_buffer.read=0;
    send_buffer.write=0;
    buffer_commande.read = 0;
	buffer_commande.write = 0;
    //QEI
    state=0;
    state_m2=0;
    state_m3=0;
    position_m2=0;
    position_m3=0;
    //Pour les déplacements rapides @6400
    Kd0 = 0.1;
    Ki0 = 7;
    Kp0 = 1.75;
    Kd1 = 0.1;
    Ki1 = 7;
    Kp1 = 1.75;
    Kd2 = 0.1;
    Ki2 = 7;
    Kp2 = 1.75;
    Kd3 = 0.1;
    Ki3 = 7;
    Kp3 = 1.75;
	//Pour les déplacements moyens @3200
    Kd0_m = 0.05;
    Ki0_m = 10;
    Kp0_m = 1.6;
    Kd1_m = 0.05;
    Ki1_m = 10;
    Kp1_m = 1.5;
    Kd2_m = 0.05;
    Ki2_m = 10;
    Kp2_m = 1.5;
    Kd3_m = 0.05;
    Ki3_m = 10;
    Kp3_m = 1.6;
    //Pour les mouvements lents @1600
    Kd0_s = 0.05;
    Ki0_s = 12;
    Kp0_s = 1.2;
    Kd1_s = 0.05;
    Ki1_s = 12;
    Kp1_s = 1.2;
    Kd2_s = 0.05;
    Ki2_s = 12;
    Kp2_s = 1.2;
    Kd3_s = 0.05;
    Ki3_s = 12;
    Kp3_s = 1.2;
	//Pour les mouvements de dessin @800
	Kd0_d = 0.2;
    Ki0_d = 10;
    Kp0_d = 1.2;
    Kd1_d = 0.2;
    Ki1_d = 10;
    Kp1_d = 1.2;
    Kd2_d = 0.2;
    Ki2_d = 10;
    Kp2_d = 1.2;
    Kd3_d = 0.2;
    Ki3_d = 10;
    Kp3_d = 1.2;
    dt = 0.1;
    
    is_waiting_for_action = false;
    
    
    init_lcd();
    afficher_param('2', 'S', 'G');
	//ecranInit();
	initCommande();
	initMotorCommand();
	initLED();
	initPrehenseur();
	initSonar();
	initPWM();
    initUART();   
    initQEI();
    initTimer();
    


    while(1)
	{
		EncoderHandler(); // Traitement des encodeurs en quadrature pour les moteurs 2 et 3

		//si un charactère dans la Receive FIFO
		if(!(UART0_FR_R & UART_FR_RXFE)) //&& (send_buffer.read > send_buffer.write-256))
		{
			receive_buffer.buffer[receive_buffer.write%BUFFER_LEN] = UART0_DR_R;
			if(!(receive_buffer.buffer[receive_buffer.write%BUFFER_LEN] > 90 || 32 > receive_buffer.buffer[receive_buffer.write%BUFFER_LEN])){
				receive_buffer.write++;
			}
		}
		
	
		if(!(UART0_FR_R & UART_FR_TXFF) && (send_buffer.read < send_buffer.write))
		{
			UART0_DR_R = send_buffer.buffer[send_buffer.read%BUFFER_LEN];
			send_buffer.read++;
		}
		if(receive_buffer.write - receive_buffer.read > 7){
			long i;
			long commande[8];
			for(i=0; i < 8; i++){
				commande[i] = receive_buffer.buffer[receive_buffer.read%BUFFER_LEN];
				receive_buffer.read++;
			}
			traiterNouvelleCommande(commande); //Traitement des commandes
		}
		if(buffer_commande.write - buffer_commande.read >= 8 && !is_drawing && a_atteint_consigne && !est_en_mouvement){
			CommandHandler();
		}
		
		if(is_waiting_for_action && a_atteint_consigne){
			send_buffer.buffer[send_buffer.write++%BUFFER_LEN] = 'E';
			is_waiting_for_action = false;
		}
		
		//Vérifier si doit dessiner
		if(is_drawing && a_atteint_consigne && !est_en_mouvement){
			switch(number_to_draw){
				case 10: // 1 Large
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(3613, 803);
							moveLateral(4817, 1070);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveFront(-12042, 1338);
							segment_to_draw++;
							break;
						case 3:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 1:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(1806, 803);
							moveLateral(2408, 1070);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-6021, 1338);
							segment_to_draw++;
							break;
						case 3:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 20:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(1806, 800);
							segment_to_draw++;
							break;
						case 2:
							moveLateral(9032, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-4787, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-7225, 800);
							moveLateral(-8972, 1000);
							segment_to_draw++;
							break;
					 	case 5:
							moveLateral(9032, 1150);
							segment_to_draw++;	
							break;
						case 6:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 2:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(903, 800);
							segment_to_draw++;
							break;
						case 2:
							moveLateral(4516, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-2408, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-3613, 800);
							moveLateral(-4516, 1000);
							segment_to_draw++;
							break;
					 	case 5:
							moveLateral(4516, 800);
							segment_to_draw++;	
							break;
						case 6:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 30:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveLateral(10838, 800);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-5419, 800);
							moveLateral(-10486, 1548);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-1658, 700);
							moveLateral(9945, 4200);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-1740, 800);
							segment_to_draw++;
							break;
					 	case 5:
					 		moveFront(-2500, 700);
							moveLateral(-10000, 2800);
							segment_to_draw++;	
							break;
						case 6:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 3:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveLateral(5419, 800);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-2710, 800);
							moveLateral(-5243, 1548);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-779, 700);
							moveLateral(4673, 4200);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-879, 800);
							segment_to_draw++;
							break;
					 	case 5:
					 		moveFront(-1175, 700);
							moveLateral(-4700, 2800);
							segment_to_draw++;	
							break;
						case 6:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 40:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(13247, 1656);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-7528, 1565);
							moveLateral(-7528, 1565);
							segment_to_draw++;
							break;
						case 3:
							moveLateral(8970, 1642);
							segment_to_draw++;
							break;
						case 4:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 4:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveFront(6632, 828);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-3764, 783);
							moveLateral(-3764, 783);
							segment_to_draw++;
							break;
						case 3:
							moveLateral(4516, 821);
							segment_to_draw++;
							break;
						case 4:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 50:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveLateral(-10638, 1667);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-3612, 803);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-4677, 518);
							moveLateral(10838, 1200);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-2942, 778);
							moveLateral(-10248, 2710);
							segment_to_draw++;
							break;
						case 5:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;	
				case 5:
					switch(segment_to_draw){
						case 1:
							offset=640;
							offset2=0;
							descendrePrehenseur();
							moveLateral(-5319, 1667);
							segment_to_draw++;
							break;
						case 2:
							moveFront(-1806, 803);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-2339, 518);
							moveLateral(5419, 1200);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-1411, 778);
							moveLateral(-4915, 2710);
							segment_to_draw++;
							break;
						case 5:
							monterPrehenseur();
							is_drawing = false;
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 60:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveLateral(-5821, 1000);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveFront(-2408,803);
							moveLateral(-4817, 1606);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-8330, 800);
							segment_to_draw++;
							break;
						case 4:
							moveLateral(11078, 800);
							segment_to_draw++;
							break;
						case 5:
							moveFront(4817, 800);
							segment_to_draw++;
							break;
						case 6:
							moveLateral(-11078, 800);
							segment_to_draw++;
							break;
						case 7:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 6:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveLateral(-2771, 800);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveFront(-1204,803);
							moveLateral(-2409, 1606);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-4165, 800);
							segment_to_draw++;
							break;
						case 4:
							moveLateral(5539, 800);
							segment_to_draw++;
							break;
						case 5:
							moveFront(2409, 800);
							segment_to_draw++;
							break;
						case 6:
							moveLateral(-5489, 800);
							segment_to_draw++;
							break;
						case 7:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;		
				case 70:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveFront(1204, 800);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveLateral(13247, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-10206, 800);
							moveLateral(-10206, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-2408, 800);
							segment_to_draw++;
							break;
						case 5:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 7:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveFront(602, 800);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveLateral(6623, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-5118, 800);
							moveLateral(-5118, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-1204, 800);
							segment_to_draw++;
							break;
						case 5:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;		
				case 80:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveFront(1806, 800);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveLateral(9934, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-1806, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-7225, 803);
							moveLateral(-9574, 1070);
							segment_to_draw++;
							break;
						case 5:
							moveFront(-1806, 800);
							segment_to_draw++;
							break;
						case 6:
							moveLateral(9934, 800);
							segment_to_draw++;
							break;
						case 7:
							moveFront(1806, 800);
							segment_to_draw++;
							break;
						case 8:
							moveFront(7225, 803);
							moveLateral(-9574,1070);
							segment_to_draw++;
							break;
						case 9:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
							break;
					}
					break;
				case 8:
					switch(segment_to_draw){ //segment a dessiner
						case 1: //1er segment
							offset=0;
							offset2=-640;
							descendrePrehenseur();
							moveFront(903, 800);
							segment_to_draw++; //Prochain mouvement faire prochain segment
							break;
						case 2:
							moveLateral(4967, 800);
							segment_to_draw++;
							break;
						case 3:
							moveFront(-903, 800);
							segment_to_draw++;
							break;
						case 4:
							moveFront(-3613, 803);
							moveLateral(-4787, 1070);
							segment_to_draw++;
							break;
						case 5:
							moveFront(-903, 800);
							segment_to_draw++;
							break;
						case 6:
							moveLateral(4967, 800);
							segment_to_draw++;
							break;
						case 7:
							moveFront(903, 800);
							segment_to_draw++;
							break;
						case 8:
							moveFront(3613, 803);
							moveLateral(-4787,1070);
							segment_to_draw++;
							break;
						case 9:
							monterPrehenseur();
							is_drawing = false; //Fin du dessin, reset les variables
							number_to_draw = 0;
							segment_to_draw = 0;
							send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
 							break;
					}
					break;
				default:
					break;
			}
		}
	}
}
