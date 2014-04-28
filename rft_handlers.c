#include "rft_handlers.h"
#include "init.h"

// Number of samples
#define TIME_PERIODS_COUNT 3
// Multiplication constant (2048 / 2047.95)
#define TIMER_SHIFTING_VALUE 1.000024414658561
// Additional number of cycles for startup
#define WARM_UP_TIME 10

#define DEFAULT_MASTER_ADDRESS 0x01

#define PACKET_TYPE_INIT  0x00
#define PACKET_TYPE_REQ   0x01
#define PACKET_TYPE_ANS   0x02
#define PACKET_TYPE_OUT   0x03

uint8_t packet[MAX_PACKET_LOAD + 2];
uint8_t packet_length, address;

uint8_t period_number, is_sampling = 0;
uint16_t time_periods[TIME_PERIODS_COUNT];

void Set_Address(uint8_t value)
{
  address = value;
}

void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line15) == SET)
  {
    EXTI_ClearITPendingBit(EXTI_Line15);
    SPI_RFT_IRO_IRQHandler();
  }
}

void TX_Complete(void)
{
	if (GetPeripheralParams().b.is_power_save && !is_sampling)
	{
		/* Enter sleep mode */
		SPI_RFT_Enter_Sleep_Mode();

		// Starting WUT
		RTC_WakeUpCmd(ENABLE);
		// Shut down
		PWR_EnterSTANDBYMode();
	}
  SPI_RFT_Start_Polling();
}

void RX_Complete(void)
{
  uint8_t i;
  uint16_t* temp;
	uint32_t sum;

  if ((packet[0] == address) || (packet[0] == 0xFF))
  {
		if (is_sampling)
		{
			TIM_Cmd(TIM6,DISABLE);
			time_periods[period_number++] = TIM_GetCounter(TIM6);

			if (period_number >= TIME_PERIODS_COUNT)
			{
				/* Sampling complete! */
				is_sampling = 0;

				sum = 0;
				for (i = 0; i < TIME_PERIODS_COUNT; i++)
					sum += time_periods[i];
				sum = sum / TIME_PERIODS_COUNT;

				sum *= TIMER_SHIFTING_VALUE;

				// Configure WUT
				RTC_ITConfig(RTC_IT_WUT,ENABLE);
				RTC_WakeUpCmd(DISABLE);
				RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
				RTC_SetWakeUpCounter(sum - WARM_UP_TIME);
			}
			else
			{
				TIM_SetCounter(TIM6,0);
				TIM_Cmd(TIM6,ENABLE);
			}
		}
    switch (packet[1])
    {
      case PACKET_TYPE_INIT:  // device initialization
      {
				if (GetPeripheralParams().b.is_power_save)
				{
					// Start to count time between packets
					for (i = 0; i < TIME_PERIODS_COUNT; i++)
						time_periods[i] = 0;
					period_number = 0;
					is_sampling = 1;
					TIM_Cmd(TIM6,ENABLE);
				}
				break;
			}
      case PACKET_TYPE_REQ:  // data request
      {
        temp = (uint16_t*)packet;
        packet_length = 0;
        if (GetPeripheralParams().b.adc_enabled)
        {
          for (i = 0; i < 8; i++)
            *temp++ = ReadADC(i);
          packet_length += 16;
        }
        if (GetPeripheralParams().b.input_enabled)
        {
          *(uint32_t*)temp = GetLogicInputs();
          packet_length += 4;
        }
        SPI_RFT_Write_Packet(DEFAULT_MASTER_ADDRESS,PACKET_TYPE_ANS,packet,packet_length);
        break;
      }
      case PACKET_TYPE_OUT:  // output data
      {
        temp = (uint16_t*)packet;
        if (GetPeripheralParams().b.dac_enabled)
        {
          WriteDACs(++temp);
          temp += 8;
        }
        if (GetPeripheralParams().b.output_enabled)
          SetLogicOutputs(*(uint32_t*)temp);
        break;
      }
    }
  }
}

uint8_t* RX_Begin(uint8_t length)
{
  packet_length = length;
  if ((length < 3) || (length > sizeof(packet)))
    return NULL;
  else
    return packet;
}
