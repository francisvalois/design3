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

volatile unsigned long periodPWM; //Période du PWM
volatile unsigned long pulsewidth;

//Fonction qui initie les PWM (pin D0, D1, D2 et D3)
void initPWM(void){	
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinConfigure(GPIO_PD0_PWM0);
    GPIOPinConfigure(GPIO_PD1_PWM1);
    GPIOPinConfigure(GPIO_PD2_PWM2);
    GPIOPinConfigure(GPIO_PD3_PWM3);
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Calcul de la periode du PWM selon la vitess du microcontroleur
    periodPWM = ROM_SysCtlClockGet() / 440;
    pulsewidth = periodPWM/2;
    

    // Configuraiton les PWM à 440Hz
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, periodPWM);
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, periodPWM);

    // Modifier le duty cycle des PWM
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, 0);//periodPWM / 4);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, 0);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, 0);
    ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);

    // Mettre les PWM en sorties
    ROM_PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN_1);
}
