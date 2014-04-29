// Event handlers for wireless module

#ifndef __RFT_HANDLERS_H__
#define __RFT_HANDLERS_H__

#include <STM32F37x.h>
#include "mrf49xa.h"

void Set_Address(uint8_t value);

void TX_Complete(void);
void RX_Complete(void);
uint8_t* RX_Begin(data_len_t length);

#endif
