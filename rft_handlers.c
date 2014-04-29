#include "rft_handlers.h"
#include "init.h"

// Number of samples
#define TIME_PERIODS_COUNT 3
// Multiplication constant (2048 / 2047.95)
#define TIMER_SHIFTING_VALUE (float)1.000024414658561
// Additional number of cycles for startup
#define WARM_UP_TIME 10

#define DEFAULT_MASTER_ADDRESS	0x01
#define BROADCAST_ADDRESS				0xFF

#define PACKET_TYPE_INIT  0x00
#define PACKET_TYPE_ANS   0x01
#define PACKET_TYPE_OUT   0x02
#define PACKET_TYPE_OK    0xFF

// Fields
#define PCKT_TO_OFST		0x00
#define PCKT_FROM_OFST	0x01
#define PCKT_CMD_OFST		0x02

uint8_t packet[MAX_PACKET_LOAD + PROTO_BYTES_CNT];
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

void MakePacket(uint8_t to, uint8_t from, uint8_t cmd)
{
	packet[PCKT_TO_OFST] = to;
	packet[PCKT_FROM_OFST] = from;
	packet[PCKT_CMD_OFST] = cmd;
}

void RX_Complete(void)
{
  uint8_t i, sender;
  uint16_t* temp;
  uint32_t dsum;
	float sum;

  if ((packet[PCKT_TO_OFST] == address) || (packet[PCKT_TO_OFST] == BROADCAST_ADDRESS))
  {
    sender = packet[PCKT_FROM_OFST];
		if (is_sampling)
		{
			TIM_Cmd(TIM6,DISABLE);
			time_periods[period_number++] = TIM_GetCounter(TIM6);

			if (period_number >= TIME_PERIODS_COUNT)
			{
				/* Sampling complete! */
				is_sampling = 0;

				dsum = time_periods[0];
				for (i = 1; i < TIME_PERIODS_COUNT; i++)
          if (dsum > time_periods[i])
            dsum = time_periods[i];
				sum = (float)dsum / (float)TIME_PERIODS_COUNT;

				sum *= TIMER_SHIFTING_VALUE;
        dsum = sum - WARM_UP_TIME;

        if (dsum > 0xFFFF)
          dsum = 0xFFFF;

				// Configure WUT
				RTC_ITConfig(RTC_IT_WUT,ENABLE);
				RTC_WakeUpCmd(DISABLE);
				RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
				RTC_SetWakeUpCounter(dsum);
			}
			else
			{
				TIM_SetCounter(TIM6,0);
				TIM_Cmd(TIM6,ENABLE);
			}
		}
    switch (packet[PCKT_CMD_OFST])
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
					MakePacket(sender,address,PACKET_TYPE_OK);
          SPI_RFT_Write_Packet(packet,PROTO_BYTES_CNT);
				}
				break;
			}
      case PACKET_TYPE_OUT:  // output data
      {
        /* setting outputs */
        temp = (uint16_t*)(packet + PROTO_BYTES_CNT);
        if (GetPeripheralParams().b.dac_enabled)
        {
          WriteDACs(++temp);
          temp += 8;
        }
        if (GetPeripheralParams().b.output_enabled)
          SetLogicOutputs(*(uint32_t*)temp);

        /* getting inputs */
        temp = (uint16_t*)(packet + PROTO_BYTES_CNT);
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
				MakePacket(sender,address,PACKET_TYPE_ANS);
        SPI_RFT_Write_Packet(packet,packet_length + PROTO_BYTES_CNT);
        break;
      }
    }
  }
}

uint8_t* RX_Begin(data_len_t length)
{
  packet_length = length;
  if ((length < 3) || (length > sizeof(packet)))
    return NULL;
  else
    return packet;
}
