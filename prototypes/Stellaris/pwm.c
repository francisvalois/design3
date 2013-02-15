#include "inc/lm3s9b92.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

volatile unsigned long ulPeriod;

void PWMInit(void){	
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinConfigure(GPIO_PD0_PWM0);
    GPIOPinConfigure(GPIO_PD1_PWM1);
    GPIOPinConfigure(GPIO_PD2_PWM2);
    GPIOPinConfigure(GPIO_PD3_PWM3);
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Calcul de la periode du PWM selon la vitess de microcontroleur
    ulPeriod = ROM_SysCtlClockGet() / 440;

    // Configuraiton des PWM à 440Hz
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, ulPeriod);

    // PWM0 duty cycle a 25% et PWM1 duty cycle a 75%.
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulPeriod / 4);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, (ulPeriod * 3) / 4);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, ulPeriod / 2);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, ulPeriod / 5);

    // Mettre les PWM en sorties
    ROM_PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN_1);
}
