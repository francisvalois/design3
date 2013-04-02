#ifndef GESTIONANTENNE_H_
#define GESTIONANTENNE_H_

#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"

//#include "gestionUART.h"
//#include "gestionInterrupt.h"
//#include "gestionLCD.h"

void antenne_Initialiser(void);
void IntAntenne(void);
void activerInterruptionsManchester(void);
void desactiverInterruptionsManchester(void);
//void traiter_mot_recu(char mot_recu);

#endif /*GESTIONANTENNE_H_*/
