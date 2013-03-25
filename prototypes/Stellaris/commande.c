#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#define BUFFER_LEN          256

typedef struct {
    volatile long        buffer[BUFFER_LEN];   // buffer
    volatile long         read;  // prochain élément à lire
    volatile long         write;   // prochain endroit où écrire
} CircularBuffer;

//Variables globales extern
//main.c
extern volatile CircularBuffer receive_buffer;
extern volatile tBoolean is_waiting_for_action;
extern volatile CircularBuffer send_buffer;
//commande.c
extern tBoolean est_en_mouvement;
extern tBoolean a_atteint_consigne;
//timer.c
extern volatile unsigned long index;
//moteur.c
extern volatile long offset, offset2;

//Variables globales
volatile long commande[8];
volatile unsigned long captured_index;
volatile long deplacement_x;
volatile long deplacement_y;
tBoolean is_waiting_for_y;
volatile CircularBuffer buffer_commande;
extern volatile CircularBuffer sonarTimeDelta;

extern tBoolean is_drawing;
long number_to_draw;
long segment_to_draw;



//Declaration de fonctions
//commande.c
void draw(volatile long number);
//motor.c
void resetVariables(void);
void motorTurnCCW(volatile long mnumber);
void motorTurnCW(volatile long mnumber);
void moveLateral(long distance, long vitesse);
void moveFront(long distance, long vitesse);
void turn(long distance, long vitesse);
//led.c
void closeLED(void);
void openLED(void);
//prehenseur.c
void descendrePrehenseur(void);
void monterPrehenseur(void);
//sonar.c
void enableSonar(void);
void disableSonar(void);

/*
 * COMMANDE
 */
 
void initCommande(void){
	is_waiting_for_y = false;
	captured_index = 0;
	deplacement_x = 0;
	deplacement_y = 0;
	is_drawing = false;
	number_to_draw = 0;
	segment_to_draw = 0;
}
 

// Fonction qui traite les nouvelles commandes.
void traiterNouvelleCommande(long commande_a_traiter[8]){
	//Checker si nouvelle commande est un Reset ou un brake, prend alors le dessus sur les autres commandes.
	if(commande_a_traiter[0] == 'R'){ //Reset du micro
		SysCtlReset();
	}
	else if(commande_a_traiter[0] == 'b'){ //Brake de base
		ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
		ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0x00);
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x00);
		a_atteint_consigne = true; //Indiquer comme si la consigne a été atteinte
		initCommande();
		buffer_commande.read = 0;
		buffer_commande.write = 0;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		
	}
	else if(commande_a_traiter[0] == 'B'){ //Brake moteur
		ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xF0);
		ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7, 0xA0);
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0x30);
		a_atteint_consigne = true; //Indiquer comme si la consigne a été atteinte
		initCommande();
		buffer_commande.read = 0;
		buffer_commande.write = 0;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
	}
	else{	//Sinon on la met dans le buffer des commandes en attentes
		long i;
		for(i=0; i < 8; i++){
			buffer_commande.buffer[buffer_commande.write%BUFFER_LEN] = commande_a_traiter[i];
			buffer_commande.write++;
		}
	}
}
	
// Fonction qui gere les commandes
// \return booléen qui dit si la commande a été traitée avec succès.
tBoolean CommandHandler(void){
	long i;
	for(i=0; i < 8; i++){
		commande[i] = buffer_commande.buffer[buffer_commande.read%BUFFER_LEN];
		buffer_commande.read++;
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
	 	/*if(deplacement_y > 4363 && deplacement_x == 0){
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 6400;
	 		deplacement_y -= 1591;
	 	}
	 	else if(deplacement_x_abs > 4363 && deplacement_y == 0){
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 6400;
	 		deplacement_x_abs -= 1591;
	 	}*/
	 	if(deplacement_y > 0 && deplacement_y < 1745 && deplacement_x == 0){
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 1600;
	 		//deplacement_y -= 504;
	 	}
	 	else if(deplacement_y > 1745 && deplacement_y < 6000 && deplacement_x == 0){
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 3200;
	 		//deplacement_y -= 504;
	 	}
	 	else if(deplacement_x_abs > 6000 && deplacement_y == 0){
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 6400;
	 		//deplacement_x_abs -= 504;
	 	}
	 	else{
	 		offset = 0; //Offset sur commande des moteurs
	 		offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
	 		consigne = 1600;
	 		//deplacement_x_abs *= 0.84;
	 		//deplacement_y *= 0.84;
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
	 	is_waiting_for_action = true;
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
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'T'){
		long degree;
		degree = (commande[2]-'0')*100;
		degree += (commande[3]-'0')*10;
		degree += (commande[4]-'0');
		degree = degree*17250/360;
		if(commande[1] == 'P'){
			degree = -degree;
		}
		else if(commande[1] != 'N'){
			initCommande();
			is_waiting_for_action = true;
			return false;
		}
		offset = 0; //Offset sur commande des moteurs
	 	offset2 = 0; //Offset sur commande moteur 2 (moteur plus brusque)
		turn(degree, 1600);
		initCommande();
		is_waiting_for_action = true;
		return true;	
	}
	else if(commande[0] == 'D'){ //dessiner numero
		if(commande[1] == 'G'){ //dessin grand
			draw((commande[2]-'0')*10);
		}
		else{
			draw(commande[2]-'0');
		}
	}
	else if(commande[0] == 'P'){ //Descendre préhenseur
		descendrePrehenseur();
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'M'){ //Monter préhenseur
		monterPrehenseur();
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'A'){ //Demander donnée antenne
		return true;
	}
	else if(commande[0] == 'O'){ //Ouvrir LED
		openLED();
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'C'){ //Fermer (Close) LED
		closeLED();
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'L'){ // afficher sur LCD
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	else if(commande[0] == 'S'){ // envoyer donn�es sonar
		enableSonar();
		long average = 0;
		ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0xFF);
		while(sonarTimeDelta.write == sonarTimeDelta.read){
		}
		average += sonarTimeDelta.buffer[sonarTimeDelta.read++%BUFFER_LEN];
		ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00);
		disableSonar();
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= (average & 0xFF000000) >> 24;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= (average & 0x00FF0000) >> 16;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= (average & 0x0000FF00) >> 8;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= (average & 0x000000FF) >> 0;
		send_buffer.buffer[send_buffer.write++%BUFFER_LEN]= 'E';
		return true;
	}
	initCommande();
	return false;
}


void draw(volatile long number){
	if(!is_drawing){
		number_to_draw = number;
		segment_to_draw = 1; //Commencer par le segment 1
		is_drawing = true;
	}
	
}
