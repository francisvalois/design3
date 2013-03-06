// Simon Grenier 910 102 970
//

#include "inc/lm3s9b92.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

//Definition de pins
//Port A
#define LCD_RS  GPIO_PIN_0
#define LCD_RS_Display  GPIO_PIN_0
#define LCD_RS_Instruction  0
#define LCD_RW  GPIO_PIN_1
#define LCD_RW_Read  GPIO_PIN_1
#define LCD_RW_Write  0
#define LCD_E   GPIO_PIN_2
//Port B = data

//Prototypes de fonctions
void init_lcd(void);
void write_names(void);
void wait(void);
void write_char(char mychar);
void clear(void);
void write(char mychar);
void afficher_temps(volatile unsigned long secondes);

// Fonction qui gère le temps de traitement du LCD
void wait(void) {
	// Change pin type
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	//Read busy flag until it's 0
	volatile unsigned long busyflag;
	do {
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Read);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
		busyflag = ROM_GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_7);
		ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	} while(busyflag == 128);
	// Restore pin type
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
}

// Fonction qui affiche le temps dans la ligne du haut
// Le temps a 5 digits (0 a 99999)
void afficher_pos(volatile unsigned long secondes) {
	// Lecture de l'adresse actuelle du curseur
	wait();
	unsigned char adresse_curseur;
	// Change pin type
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	// Read adress
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Read);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	adresse_curseur = ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x7F); // 7 pins d'adresse
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	// Restore pin type
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	
	// Changement de l'adresse du curseur pour aller a la ligne du haut
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x80 | 0x0B);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	
	// Ecriture du temps
	write_char('0' + (secondes/10000)%10);
	write_char('0' + (secondes/1000)%10);
	write_char('0' + (secondes/100)%10);
	write_char('0' + (secondes/10)%10);
	write_char('0' + (secondes/1)%10);
	
	// Ecriture de l'adresse du curseur pour retourner a la ligne du bas
	// Set DDRAM address to second line
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x80 | adresse_curseur);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
}

void afficher_speed(volatile unsigned long secondes) {
	// Lecture de l'adresse actuelle du curseur
	wait();
	unsigned char adresse_curseur;
	// Change pin type
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	// Read adress
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Read);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	adresse_curseur = ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x7F); // 7 pins d'adresse
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	// Restore pin type
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	
	// Changement de l'adresse du curseur pour aller a la ligne du haut
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x80 | 0xC0);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	
	// Ecriture du temps
	write_char('0' + (secondes/10000)%10);
	write_char('0' + (secondes/1000)%10);
	write_char('0' + (secondes/100)%10);
	write_char('0' + (secondes/10)%10);
	write_char('0' + (secondes/1)%10);
	
	// Ecriture de l'adresse du curseur pour retourner a la ligne du bas
	// Set DDRAM address to second line
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x80 | adresse_curseur);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
}

//Fonction qui écrit les noms des équipiers dans la mémoire du LCD pour l'affichage sur la 1ère ligne.
void write_position(void) {
	// Write names
	write_char('P');
	write_char('o');
	write_char('s');
	write_char('i');
	write_char('t');
	write_char('i');
	write_char('o');
	write_char('n');
	write_char(' ');
	write_char(':');
	write_char(' ');
	// Set DDRAM address to second line
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0xC0);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
}

//Fonction qui écrit le charactère "mychar" en mémoire du LCD
void write_char(char mychar) {
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Display | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, mychar);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
}

volatile unsigned long nbrchar; //Variable global qui contient le nombre de charactère affiché
                                // sur la ligne du bas.

// Fonction qui gère les écritures des charactères "mychar" dans la mémoire du LCD pour l'affichage
void write(char mychar) {
	//Si moins de 16 charactères affichés (ligne du bas pas encore pleine), continuer d'écrire.
	if (nbrchar < 16) {
		write_char(mychar);
		nbrchar++;
	}
}

//Fonction qui effectue un "Clear" sur le LCD et qui met les nom des équipiers sur la ligne du haut.
void clear(void) {
	// Clear LCD
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x01);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	// Write names
	write_position();
	// Reset counter
	nbrchar = 0;
}
	
// Fonction qui initialise le LCD
void init_lcd(void) {
	// Enable ports on board
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	// Configure pins
	ROM_GPIOPadConfigSet(GPIO_PORTE_BASE, 0xFF, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
	// Enable data as output
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0xFF); // Toutes les pins
	// Default values of pins to zero
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW | LCD_E, 0x00);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x00);
	// Set LCD to 2line mode
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x38);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	// Activate cursor blinking
	wait();
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_RS | LCD_RW, LCD_RS_Instruction | LCD_RW_Write);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, LCD_E);
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, 0xFF, 0x0F);
	ROM_GPIOPinWrite(GPIO_PORTJ_BASE, LCD_E, 0);
	// Clear LCD
	clear();
}
