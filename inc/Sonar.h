#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"


#define I2C_BUBBER_LEN 2

extern xQueueHandle I2C_rx_msg_queue;
extern volatile xSemaphoreHandle I2C2_Mutex;

void I2C_init (void);
void I2C_Write (uint8_t Address, uint8_t * data, uint16_t len);
void I2C_Read (uint8_t Address, uint16_t len);

#endif /* SONAR_H_INCLUDED */
