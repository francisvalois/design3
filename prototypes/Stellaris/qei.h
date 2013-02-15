#ifndef QEI_H_
#define QEI_H_

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

void initQEI(void);
void EncoderIntHandler(void);
void EncoderHandler(void);
#endif /*QEI_H_*/
