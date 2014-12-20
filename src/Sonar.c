
#include<stdlib.h>
#include "stm32f10x.h"
#include "Sonar.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "gpiodef.h"

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint8_t)0x01)
#define OAR1_ADD0_Reset         ((uint8_t)0xFE)



uint8_t I2C_addr;
uint8_t  * I2C_data;
uint16_t I2C_len;
uint16_t I2C_pointer = 0;
uint8_t Transmit;

static portBASE_TYPE xHigherPriorityTaskWoken;
xQueueHandle I2C_rx_msg_queue;
volatile xSemaphoreHandle I2C2_Mutex;

void I2C2_EV_IRQHandler(void)
{
    uint16_t sr_1_reg = I2C2->SR1;
    uint16_t sr_2_reg;
    if (sr_1_reg & I2C_SR1_SB) //I2C START event
    {
        I2C2->DR = I2C_addr; //Send Address
        //GPIOB->BSRR = GPIO_Pin_4; //-> HIGHT debug
    }
    else if (sr_1_reg & I2C_SR1_ADDR) //Address sent (Master)
    {
        //GPIOA->BSRR = GPIO_Pin_9; //-> HIGHT debug
        //If transmitter, then send byte
        sr_2_reg = I2C2->SR2;
        if (Transmit)
        {
            I2C2->DR = I2C_data[I2C_pointer++];
        }
        else //If receiver
        {


            if (I2C_len == 1) //Case of a single byte to be received
            {
                /*
                    – In the ADDR event, clear the ACK bit.
                    – Clear ADDR
                    – Program the STOP/START bit.
                    – Read the data after the RxNE flag is set.
                */
                I2C2->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
                I2C2->CR1 |= I2C_CR1_STOP;
                xSemaphoreGive ( I2C2_Mutex );
            }
            else if (I2C_len == 2)
            {
                I2C2->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
            }

        }
    }
    else if (sr_1_reg & I2C_SR1_BTF) //Data byte transfer finished
    {
        //GPIOA->BSRR = GPIO_Pin_10; //-> HIGHT debug
        //If transmitter, then send byte
        sr_2_reg = I2C2->SR2;
        if (Transmit)
        {
            I2C2->CR1 |= I2C_CR1_STOP;
            I2C_pointer = 0;
            xSemaphoreGive ( I2C2_Mutex );
        }
        else //If receiver
        {
            I2C_data[I2C_pointer++] = I2C2->DR;
            I2C_data[I2C_pointer++] = I2C2->DR;
            if (I2C_pointer == I2C_len )
            {
                I2C2->CR1 &= ~I2C_CR1_ACK; //Acknowledge disable
                I2C2->CR1 |= I2C_CR1_STOP;
                I2C_pointer = 0;
                xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGive ( I2C2_Mutex );
                xQueueSendToBackFromISR(I2C_rx_msg_queue, &I2C_data, &xHigherPriorityTaskWoken);
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
                vPortFree (I2C_data);
            }
        }

    }
    else if (sr_1_reg & I2C_SR1_TXE) //Transmit buffer empty
    {
        if (I2C_pointer<I2C_len)
        {
            I2C2->DR = I2C_data[I2C_pointer++];
        }
    }
    else if (sr_1_reg & I2C_SR1_RXNE) //Receive buffer not empty
    {
        //GPIOA->BSRR = GPIO_Pin_11; //-> HIGHT debug
        I2C_data[I2C_pointer++] = I2C2->DR;
        if (I2C_pointer + 1 == I2C_len )
        {
            I2C2->CR1 &= ~I2C_CR1_ACK; //Acknowledge disable
            I2C2->CR1 |= I2C_CR1_STOP;
        }
        else if (I2C_pointer == I2C_len )
        {
            I2C_pointer = 0;

            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGive ( I2C2_Mutex );
            xQueueSendToBackFromISR(I2C_rx_msg_queue, &I2C_data, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
            vPortFree (I2C_data);
        }

    }
    //GPIOB->BRR = GPIO_Pin_4; // -> LOW debug
    //GPIOA->BRR = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; // -> LOW debug
}


void I2C_init (void)
{
    /* Enable AFIO and GPIOB clock */
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR    |= RCC_APB1ENR_I2C2EN;
    //DBGMCU->CR |= DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT;
    /* Configure I2C2 SDA (PB11)*/
    //GPIOB->CRH      &= ~GPIO_CRH_CNF;
    GPIOB->CRH      |= GPIO_CRH_MODE11;//11: Output mode, max speed 50 MHz.
    GPIOB->CRH      |= GPIO_CRH_CNF11; //11: Alternate function output Open-drain
    /* Configure I2C2 SCL (PB10)*/
    GPIOB->CRH      |= GPIO_CRH_MODE10; //11: Output mode, max speed 50 MHz.
    GPIOB->CRH      |= GPIO_CRH_CNF10; //11: Alternate function output Open-drain


    /* Reset I2C2 */
    RCC->APB1RSTR   |= RCC_APB1RSTR_I2C2RST;
    RCC->APB1RSTR    &= ~RCC_APB1RSTR_I2C2RST;
    I2C2->CR1 |= 0x8000;
    I2C2->CR1 &= ~0x8000;
    /* Configure I2C2 */
    I2C2->CR2       |= (I2C_CR2_FREQ_2 | I2C_CR2_FREQ_5); //Peripheral clock frequency 36 MHz
    I2C2->CCR       = 0x801E; //fast mode, 400 KHz 2/3 duty cycle
    I2C2->TRISE     |=0x0E; //Rise time 300ns
    I2C2->OAR1      =0x4000; // 7-bit addressing
    I2C2->CR1       |= I2C_CR1_PE; //Peripheral enable
    I2C2->CR2       |= I2C_CR2_ITBUFEN; //1:TxE = 1 or RxNE = 1 generates Event Interrupt
    I2C2->CR2       |= I2C_CR2_ITEVTEN; //Event interrupt enable

    NVIC_EnableIRQ (I2C2_EV_IRQn);
    //I2C2->CR1 |= I2C_CR1_STOP;
    I2C_rx_msg_queue = xQueueCreate (5,4);
    I2C2_Mutex = xSemaphoreCreateMutex();
}
void I2C_Write (uint8_t Address, uint8_t * data, uint16_t len)
{

    I2C_addr = Address & OAR1_ADD0_Reset;
    I2C_data = data;
    I2C_len = len;
    I2C_pointer = 0;
    Transmit = 1;

    I2C2->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    //I2C2->CR1 &= ~I2C_CR1_STOP;
    I2C2->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation
}
void I2C_Read (uint8_t Address, uint16_t len)
{
    I2C_addr = Address | OAR1_ADD0_Set;
    I2C_data = pvPortMalloc (len);
    I2C_len = len;
    I2C_pointer = 0;
    Transmit = 0;
    if (len == 2)
    {
       I2C2->CR1 |=  I2C_CR1_POS;
    }
    I2C2->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    I2C2->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation
}
