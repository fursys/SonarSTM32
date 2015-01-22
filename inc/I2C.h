#ifndef SONAR_H_INCLUDED
#define SONAR_H_INCLUDED

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"


#define I2C1_QUEUE_LEN 5
#define I2C2_QUEUE_LEN 5

#define I2C1_MAX_DATA_SIZE 6
#define I2C2_MAX_DATA_SIZE 4

typedef struct
{
    uint8_t I2C_addr;
    uint8_t  * I2C_data;
    uint16_t I2C_len;
    uint16_t I2C_pointer;
    uint8_t Transmit;
} I2C_data_frame;



extern xQueueHandle I2C1_rx_msg_queue;
extern xQueueHandle I2C2_rx_msg_queue;

extern volatile xSemaphoreHandle I2C1_Mutex;
extern volatile xSemaphoreHandle I2C2_Mutex;

void I2C_init (I2C_TypeDef* I2Cx);
void I2C_Write (I2C_TypeDef* I2Cx, uint8_t Address, uint8_t * data, uint16_t len);
void I2C_Read (I2C_TypeDef* I2Cx, uint8_t Address, uint16_t len);

#endif /* SONAR_H_INCLUDED */
