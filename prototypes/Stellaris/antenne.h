//***********************************************************
//
//       antenne.h
//       Fonctions de dÃ©codage d'antenne
//       2013-03-10
//        
//
//***********************************************************
#ifndef ANTENNE_H_
#define ANTENNE_H_

#define FACTEUR_0 2.7
#define FACTEUR_1 1.75
#define MAX_TIMER_EVENTS 100  // taille de l'échantillion 

//#define TIMEOUT_VAL 5000 // periode du signal PWM pour test
//#define MAX_TIMER_EVENTS 100  // taille de l'échantillion 
//#define NB_ESSAIS_MAX 10 // nombre d'essais maximal pour lire le signal

//static unsigned short g_usTimerBuf[MAX_TIMER_EVENTS]; // buffer qui contient les timings
//static unsigned long g_ulTimer0BIntCount = 0; // compte d'erreur d'interrupt
//static unsigned long g_uluDMAErrCount = 0; // compte d'erreur de DMA
//static volatile unsigned int g_bDoneFlag = 0; // flag de complétion de transfert DMA


void initTimer1B();
void initDMA(unsigned short *ptrBuffer);
void startTimer1B();
void reverse(unsigned short *s[2]);
int bitsCompare(unsigned char s1[7], unsigned char s2[7]);
void readSignal(unsigned short *ptrBuffer, unsigned short *signalBuffer, unsigned int *signalBufferSize);
int decodeSignal(unsigned short *signalBuffer, unsigned int signalBufferSize, unsigned char signal[7]);

#endif /*ECRAN_H_*/
