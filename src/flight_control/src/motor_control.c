#include "pwm.h"

void pwm_init(void)
{
    SetOutputEPWM1(SINGLE_OUT, PWM_MODE_1);
    OpenEPWM1(0xFF, ECCP_1_SEL_TMR12);
    SetDCEPWM1(0x6FF);
}
