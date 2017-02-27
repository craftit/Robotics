
#include <stdint.h>
#include "stm32_tim.h"
#include "hal.h"

#if STM32_HAS_TIM1

#endif

void InitPWM(void)
{
  rccEnableTIM1(false);
  rccResetTIM1();

}

void ShutdownPWM(void)
{
  rccDisableTIM1(false);

}
