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
#define PACKET_TYPE_REQ   0x02
#define PACKET_TYPE_TRSMT	0x03
#define PACKET_TYPE_ECHO	0xFD
#define PACKET_TYPE_OK    0xFF

#define DATA_LENGTH			(sizeof(data_len_t))

// Fields
#define PCKT_TO_OFST		0x00
#define PCKT_FROM_OFST	0x01
#define PCKT_CMD_OFST		0x02
#define PCKT_DATA_OFST	0x03

#define TRMS_TO_OFST		0x00
#define TRMS_CMD_OFST		0x01
#define TRMS_DATA_OFST	0x02

// Work packet
uint8_t packet[MAX_PACKET_LOAD + PROTO_BYTES_CNT];
// In-going transmission
uint8_t in_trans[MAX_PACKET_LOAD + PROTO_BYTES_CNT];
// Out-going transmission
uint8_t out_trans[MAX_PACKET_LOAD + PROTO_BYTES_CNT];
data_len_t packet_length;
uint8_t address;

uint8_t period_number, is_sampling = 0;
uint8_t current_transmission_from, is_transmit = 0;
uint8_t transmt_count, transmt_index;
uint16_t time_periods[TIME_PERIODS_COUNT];
uint8_t *curr_transmt_pos, *curr_pack_pos;

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

void StartTimer6(void)
{
  uint8_t i;

  for (i = 0; i < TIME_PERIODS_COUNT; i++)
    time_periods[i] = 0;
  period_number = 0;
  is_sampling = 1;
  TIM_Cmd(TIM6,ENABLE);
}

void GetOutputs(uint16_t *p, data_len_t *len)
{
  uint8_t i;

  len = 0;
  if (GetPeripheralParams().b.adc_enabled)
  {
    for (i = 0; i < 8; i++)
      *p++ = ReadADC(i);
    len += 16;
  }
  if (GetPeripheralParams().b.input_enabled)
  {
    *(uint32_t*)p = GetLogicInputs();
    len += 4;
  }
}

void SetOutputs(uint16_t *p)
{
  if (GetPeripheralParams().b.dac_enabled)
  {
    WriteDACs(p);
    p += 16;
  }
  if (GetPeripheralParams().b.output_enabled)
    SetLogicOutputs(*(uint32_t*)p);
}

void HandlePacket(uint8_t *h_packet)
{
  data_len_t *temp;

  switch (h_packet[TRMS_CMD_OFST])
  {
    case PACKET_TYPE_INIT:
    {
      if (GetPeripheralParams().b.is_power_save)
      {
        // Start to count time between packets
        StartTimer6();
        *(data_len_t*)curr_pack_pos = 0;
        curr_pack_pos += DATA_LENGTH;
        *(curr_pack_pos + TRMS_TO_OFST) = address;
        *(curr_pack_pos + TRMS_CMD_OFST) = PACKET_TYPE_OK;
        curr_pack_pos += TRMS_DATA_OFST;
      }
      break;
    }
    case PACKET_TYPE_REQ:
    {
      /* setting outputs */
      SetOutputs((uint16_t*)(h_packet + TRMS_DATA_OFST));

      /* getting outputs */
      temp = (data_len_t*)curr_pack_pos;
      curr_pack_pos += DATA_LENGTH;
      *(curr_pack_pos + TRMS_TO_OFST) = address;
      *(curr_pack_pos + TRMS_CMD_OFST) = PACKET_TYPE_ANS;
      curr_pack_pos += TRMS_DATA_OFST;
      GetOutputs((uint16_t*)curr_pack_pos,temp);

      curr_pack_pos += *temp;
      break;
    }
  }
}


void TransmitNextFromBuffer(void)
{
	uint32_t curr_length, i;

	// Check with ourselves
	if (*(curr_transmt_pos + DATA_LENGTH + TRMS_TO_OFST) == address)
  {
    // Recipient is we!
    curr_transmt_pos += DATA_LENGTH;
    HandlePacket(curr_transmt_pos);
    return;
  }

	// Length
	curr_length = *(data_len_t*)curr_transmt_pos;
	curr_transmt_pos += DATA_LENGTH;

	// Protocol fields
	MakePacket(*(curr_transmt_pos + TRMS_TO_OFST),address,
						 *(curr_transmt_pos + TRMS_CMD_OFST));

	// Data
	curr_transmt_pos += TRMS_DATA_OFST;
	for (i = 0; i < curr_length; i++)
		// Copy first packet into output buffer
		packet[PCKT_DATA_OFST + i] = *curr_transmt_pos++;

	SPI_RFT_Write_Packet(packet,curr_length + PROTO_BYTES_CNT);
}

void RX_Complete(void)
{
  uint8_t sender;
  uint32_t dsum, i;
	uint16_t curr_length;
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
		if (is_transmit)
		{
			// Transmission in progress
			// All received packets should be packed to output packet
			curr_length = packet_length - PROTO_BYTES_CNT;
			*(data_len_t*)curr_pack_pos = curr_length;
			curr_pack_pos += DATA_LENGTH;
			*(curr_pack_pos + TRMS_TO_OFST) = sender;
			*(curr_pack_pos + TRMS_CMD_OFST) = packet[PCKT_CMD_OFST];
			curr_pack_pos += TRMS_DATA_OFST;
			for (i = 0; i < curr_length; i++)
			{
				*curr_pack_pos++ = packet[PCKT_DATA_OFST + i];
			}
			transmt_index++;
			if (transmt_index < transmt_count)
			{
				// Still have something to transmit
				TransmitNextFromBuffer();
			}
			else
			{
				// Nothing to transmit; send packet to main sender
				out_trans[PCKT_TO_OFST] = current_transmission_from;
				out_trans[PCKT_FROM_OFST] = address;
				out_trans[PCKT_CMD_OFST] = PACKET_TYPE_TRSMT;
				out_trans[PCKT_DATA_OFST] = transmt_count;

				curr_length = curr_pack_pos - out_trans;
				SPI_RFT_Write_Packet(out_trans,curr_length);

				is_transmit = 0;
			}
			return;
		}
    switch (packet[PCKT_CMD_OFST])
    {
      case PACKET_TYPE_INIT:  // device initialization
      {
				if (GetPeripheralParams().b.is_power_save)
				{
					// Start to count time between packets
					StartTimer6();
					MakePacket(sender,address,PACKET_TYPE_OK);
          SPI_RFT_Write_Packet(packet,PROTO_BYTES_CNT);
				}
				break;
			}
      case PACKET_TYPE_REQ:  // output data
      {
        SetOutputs((uint16_t*)(packet + PCKT_DATA_OFST));
        GetOutputs((uint16_t*)(packet + PCKT_DATA_OFST),&packet_length);

				MakePacket(sender,address,PACKET_TYPE_ANS);
        SPI_RFT_Write_Packet(packet,packet_length + PROTO_BYTES_CNT);
        break;
      }
			case PACKET_TYPE_TRSMT:		// Incapsulated packets
			{
				if (!is_transmit)		// First request
				{
					current_transmission_from = sender;
					is_transmit = 1;
					for (i = 0; i < packet_length; i++)
					{
						// Copy received packet to buffer
						in_trans[i] = packet[i];
					}
					// Start first transmission
					transmt_count = in_trans[PCKT_DATA_OFST];
					transmt_index = 0;
					curr_pack_pos = &out_trans[PCKT_DATA_OFST + 1];
					curr_transmt_pos = &in_trans[PCKT_DATA_OFST + 1];

					TransmitNextFromBuffer();
				}
				break;
			}
    }
  }
}

uint8_t* RX_Begin(data_len_t length)
{
  packet_length = length;
  if ((length < PROTO_BYTES_CNT) || (length > sizeof(packet)))
    return NULL;
  else
    return packet;
}
