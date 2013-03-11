//***********************************************************
//
//       antenne.h
//       Fonctions de décodage d'antenne
//       2013-03-10
//        
//
//***********************************************************
#ifndef ANTENNE_H_
#define ANTENNE_H_

#define FACTEUR_0 2.7
#define FACTEUR_1 1.75

static void initTimer1B();
static void initDMA();
static void startTimer1B();
void reverse(unsigned short s[][2]);
int readSignal(unsigned short *signalBuffer, unsigned int *signalBufferSize);
int decodeSignal(unsigned short *signalBuffer, unsigned int signalBufferSize, unsigned char signal[7]);

#endif /*ECRAN_H_*/