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

static void initTimer1B();
static void initDMA();
static void startTimer1B();
static int readSignal(unsigned short *signalBuffer, unsigned int *signalBufferSize);
static int decodeSignal(unsigned char signal[7]);

#endif /*ECRAN_H_*/