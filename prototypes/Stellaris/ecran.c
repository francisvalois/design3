#include "ecran.h"


/*
 * Cette fonction permet de s'assurer que l'écran est pas occuper avant d'écrire quelque chose dessus
 */
void ecranAttend(void)
{
    ECRAN_CTRL &= ~(ECRAN_RS); // RS = 0
    ECRAN_CTRL |= ECRAN_RW; // RW = 1
    volatile unsigned long busyflag = ECRAN_DATA & ECRAN_D7;
    while (busyflag != 0) //on regarde le busyflag pour s'assurer que l'écran est pas occupé
    { 
            busyflag = ECRAN_DATA & ECRAN_D7;
    }
     return;
}

/*
 * Cette fonction exécute la routine d'initialisation de l'écran dans le mode qu'on veux
 */
void ecranInit(void)
{
    ecranControl(ECRAN_D5 | ECRAN_D4 | ECRAN_D3 | ECRAN_D2);
    ecranControl(ECRAN_D3 | ECRAN_D2 | ECRAN_D1);
}

/*
 * Cette fonction permet de faire des commandes tel que l'initalisation, changement de positions et 
 * trucs du genre, tout ce qui est pas l'écriture de caractères sur l'écran
 */
void ecranControl(unsigned long input)
{
ecranAttend();
 volatile unsigned long ulLoop;
ECRAN_DATA = input; //on met notre entrée sur le port de données
ECRAN_CTRL &=~(ECRAN_RS | ECRAN_RW); //reset et read/write à zéro pour pas qu'il ré-écrive par erreur
ECRAN_CTRL ^= ECRAN_EN;
ulLoop = SYSCTL_RCGC2_R;
        for (ulLoop = 0; ulLoop < 200; ulLoop++) 
        {
             // on met un delai pour que le LCD puisse voir le enable
        }
ECRAN_CTRL ^=ECRAN_EN;
ulLoop = SYSCTL_RCGC2_R; 
        for (ulLoop = 0; ulLoop < 200; ulLoop++) 
        {
        // on attend un peu pour que l'instruction s'execute avant de changer la pin 7
        }
 
ECRAN_DATA &= ~(ECRAN_D7); //on reset la pin 7 parce elle output aussi le busy flag et ont pas veux être pogner dans ecranAttend()
}



/*
 * Cette fonction efface le contenue de l'écran
 */
void ecranClear(void)
{
    ecranControl(ECRAN_D0);
    volatile unsigned long ulLoop;
    
    for (ulLoop = 0; ulLoop < 2000; ulLoop++) 
    {
        //loop pour être sur que ton est bien clearer
    }
}

/*
 * Cette fonction change la position du curseur d'écriture de l'écran
 */
void ecranSetPosCursor(short pos)
{
    ecranControl(GPIO_PIN_7 | pos);
}

/*
 * Cette fonction permet d'écrire un caractère sur l'écran
 */
void ecranWriteChar(char caractere)
{
    volatile unsigned long ulLoop;
    ecranAttend();
    ECRAN_DATA = caractere;
    ECRAN_CTRL &= ~(ECRAN_RW); //RW = 0
    ECRAN_CTRL |=  ECRAN_RS; // rs = 1
    ECRAN_CTRL ^=  ECRAN_EN; //En = 1
    ulLoop = SYSCTL_RCGC2_R;
    for (ulLoop = 0; ulLoop < 200; ulLoop++) 
    {
        //petit délai, juste pour être sûr
    }
    ECRAN_CTRL ^= ECRAN_EN; //En = 0
}


/*
 * Cette fonction permet d'écrire une chaine de caractères sur l'écran
 */
void ecranWriteLine(char * line, unsigned short size) //todo
{
    unsigned short i=0;
    for(i=0; i <size;i++)
    {
         ecranWriteChar(line[i]);
    }
}
