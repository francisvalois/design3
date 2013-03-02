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

#define BUFFER_LEN          256

typedef struct {
    volatile short        buffer[BUFFER_LEN];   // buffer
    volatile long         read;  // prochain élément à lire
    volatile long         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales extern
//main.c
extern volatile CircularBuffer receive_buffer;
//commande.c
extern tBoolean est_en_mouvement;
extern tBoolean a_atteint_consigne;
//timer.c
extern volatile unsigned long index;

volatile short commande[8];
volatile unsigned long captured_index;
volatile long deplacement_x;
volatile long deplacement_y;
tBoolean is_waiting_for_y;



//Declaration de fonctions
//commande.c
void resetVariables(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void motorBrake(volatile long mnumber);
void motorHardBrake(volatile long mnumber);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
void turn(long distance, long vitesse);

/*
 * COMMANDE
 */
 
void initCommande(void){
	is_waiting_for_y = false;
	captured_index = 0;
	deplacement_x = 0;
	deplacement_y = 0;
}
 
 
// Fonction qui gere les commandes
// \param command : commande à traiter
// \return booléen qui dit si la commande a été traitée avec succès.
tBoolean CommandHandler(){
	long i;
	for(i=0; i < 8; i++){
		commande[i] = receive_buffer.buffer[receive_buffer.read%BUFFER_LEN];
		receive_buffer.read++;
	}

	//Si reçoit seconde commande contient valeurs en y
	if(is_waiting_for_y && (commande[0] == ' ' || commande[0] == '-')){
		is_waiting_for_y = false;
	 	deplacement_y = (commande[1]-'0')*1000000;
	 	deplacement_y += (commande[2]-'0')*100000;
	 	deplacement_y += (commande[3]-'0')*10000;
	 	deplacement_y += (commande[4]-'0')*1000;
	 	deplacement_y += (commande[5]-'0')*100;
	 	deplacement_y += (commande[6]-'0')*10;
	 	deplacement_y += (commande[7]-'0');
	 	long consigne;
	 	long deplacement_x_abs;
	 	tBoolean x_is_negative = false;
	 	//Valeur absolue de x pour les comparaisons
	 	if(deplacement_x < 0){
	 		deplacement_x_abs = -deplacement_x;
	 		x_is_negative = true;
	 	}
	 	else{
	 		deplacement_x_abs = deplacement_x;
	 	}
	 	//Ajustement de la consigne de vitesse selon le déplacement
	 	if(deplacement_y > 4363 && deplacement_x == 0){
	 		consigne = 6400;
	 		deplacement_y -= 1891;
	 	}
	 	else if(deplacement_x_abs > 4363 && deplacement_y == 0){
	 		consigne = 6400;
	 		deplacement_x_abs -= 1891;
	 	}
	 	else if(deplacement_y > 1745 && deplacement_x == 0){
	 		consigne = 6400;
	 		deplacement_y -= 524;
	 	}
	 	else if(deplacement_x_abs > 1745 && deplacement_y == 0){
	 		consigne = 3200;
	 		deplacement_x_abs -= 524;
	 	}
	 	else{
	 		consigne = 1600;
	 	}
	 	
	 	//Remettre x négatif si était négatif
	 	if(x_is_negative){
	 		deplacement_x = -deplacement_x_abs;
	 	}
	 	else{
	 		deplacement_x = deplacement_x_abs;
	 	}
	 	
	 	if(commande[0] == '-'){
	 		deplacement_y = -deplacement_y;
	 	}
	 	moveFront(deplacement_y, consigne);
	 	moveLateral(deplacement_x, consigne);
	 	initCommande();
	 	return true;
	}
	else{
		is_waiting_for_y = false;
		initCommande();
	}		
	if(commande[0] == ' ' || commande[0] == '-'){
	 	deplacement_x = (commande[1]-'0')*1000000;
	 	deplacement_x += (commande[2]-'0')*100000;
	 	deplacement_x += (commande[3]-'0')*10000;
	 	deplacement_x += (commande[4]-'0')*1000;
	 	deplacement_x += (commande[5]-'0')*100;
	 	deplacement_x += (commande[6]-'0')*10;
	 	deplacement_x += (commande[7]-'0');
	 	if(commande[0] == '-'){
	 		deplacement_x = -deplacement_x;
	 	}
		is_waiting_for_y = true;
		captured_index = index;
		return true;
	}
	else if(commande[0] == 'T'){
		long degree;
		degree = (commande[2]-'0')*100;
		degree += (commande[3]-'0')*10;
		degree += (commande[4]-'0');
		degree = degree*16000/360-200;
		if(commande[1] == 'N'){
			degree = -degree;
		}
		turn(degree, 1600);
		initCommande();
		return true;
		
	}
	else if(commande[0] == 'b'){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
		a_atteint_consigne = true; //Indiquer comme si la consigne a été atteinte
		initCommande();
		return true;
		
	}
	else if(commande[0] == 'B'){
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xF0);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
		a_atteint_consigne = true; //Indiquer comme si la consigne a été atteinte
		initCommande();
		return true;
	}
	else if(commande[0] == 'R'){
		SysCtlReset();
	}
	//else if()
	return true;
}
