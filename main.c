#include "stm32f37x.h"                  // Keil::Device:Startup
#include "init.h"

uint8_t is_config = 0;

int main(void)
{
  /* Checking jumper */
  if (!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
    is_config = 1;

  if (!is_config && !(RCC->BDCR & RCC_BDCR_RTCEN))
    /* Normal mode can't be used without RTC. Enter infinite cycle.
        If it's the first running, you should configure this controller over the USB first. */
    while (1);

  Initialization(is_config);

  while (1);
}
