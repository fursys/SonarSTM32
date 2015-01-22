
#include<stdlib.h>
#include "stm32f10x.h"
#include "I2C.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "gpiodef.h"

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint8_t)0x01)
#define OAR1_ADD0_Reset         ((uint8_t)0xFE)

static portBASE_TYPE xHigherPriorityTaskWoken;

I2C_data_frame I2C1_DataFrame;
I2C_data_frame I2C2_DataFrame;

xQueueHandle I2C1_rx_msg_queue;
xQueueHandle I2C2_rx_msg_queue;

volatile xSemaphoreHandle I2C1_Mutex;
volatile xSemaphoreHandle I2C2_Mutex;
//------------------------------------------------------------------------------------------------
void I2Cx_EV_IRQHandler (I2C_TypeDef* I2Cx, I2C_data_frame* DFx, xSemaphoreHandle I2Cx_Mutex, xQueueHandle I2Cx_rx_msg_queue)
{
    uint16_t sr_1_reg = I2Cx->SR1;
    uint16_t sr_2_reg;
    if (sr_1_reg & I2C_SR1_SB) //I2C START event
    {
        I2Cx->DR = DFx->I2C_addr; //Send Address
        //GPIOB->BSRR = GPIO_Pin_4; //-> HIGHT debug
    }
    else if (sr_1_reg & I2C_SR1_ADDR) //Address sent (Master)
    {
        //GPIOA->BSRR = GPIO_Pin_9; //-> HIGHT debug
        //If transmitter, then send byte
        sr_2_reg = I2Cx->SR2;
        if (DFx->Transmit)
        {
            I2Cx->DR = DFx->I2C_data[DFx->I2C_pointer++];
        }
        else //If receiver
        {


            if (DFx->I2C_len == 1) //Case of a single byte to be received
            {
                /*
                    – In the ADDR event, clear the ACK bit.
                    – Clear ADDR
                    – Program the STOP/START bit.
                    – Read the data after the RxNE flag is set.
                */
                I2Cx->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
                I2Cx->CR1 |= I2C_CR1_STOP;
                xSemaphoreGive ( I2Cx_Mutex ); //Give point 1
            }
            else if (DFx->I2C_len == 2)
            {
                I2Cx->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
            }

        }
    }
    else if (sr_1_reg & I2C_SR1_BTF) //Data byte transfer finished
    {
        //GPIOA->BSRR = GPIO_Pin_10; //-> HIGHT debug
        //If transmitter, then send byte
        sr_2_reg = I2Cx->SR2;
        if (DFx->Transmit)
        {
            I2Cx->CR1 |= I2C_CR1_STOP;
            DFx->I2C_pointer = 0;
            xSemaphoreGive ( I2Cx_Mutex );
        }
        else //If receiver
        {
            DFx->I2C_data[DFx->I2C_pointer++] = I2Cx->DR;
            DFx->I2C_data[DFx->I2C_pointer++] = I2Cx->DR;
            if (DFx->I2C_pointer == DFx->I2C_len )
            {
                I2Cx->CR1 &= ~I2C_CR1_ACK; //Acknowledge disable
                I2Cx->CR1 |= I2C_CR1_STOP;
                DFx->I2C_pointer = 0;
                xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGive ( I2Cx_Mutex ); //Give point 2
                xQueueSendToBackFromISR(I2Cx_rx_msg_queue, &(DFx->I2C_data), &xHigherPriorityTaskWoken);
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
                vPortFree (DFx->I2C_data);
            }
        }

    }
    else if (sr_1_reg & I2C_SR1_TXE) //Transmit buffer empty
    {
        if (DFx->I2C_pointer<DFx->I2C_len)
        {
            I2Cx->DR = DFx->I2C_data[DFx->I2C_pointer++];
        }
    }
    else if (sr_1_reg & I2C_SR1_RXNE) //Receive buffer not empty
    {
        //GPIOA->BSRR = GPIO_Pin_11; //-> HIGHT debug
        DFx->I2C_data[DFx->I2C_pointer++] = I2Cx->DR;
        if (DFx->I2C_pointer + 1 == DFx->I2C_len )
        {
            I2Cx->CR1 &= ~I2C_CR1_ACK; //Acknowledge disable
            I2Cx->CR1 |= I2C_CR1_STOP;
        }
        else if (DFx->I2C_pointer == DFx->I2C_len )
        {
            DFx->I2C_pointer = 0;

            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGive ( I2Cx_Mutex ); //Give point 3
            xQueueSendToBackFromISR(I2Cx_rx_msg_queue, &(DFx->I2C_data), &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
            vPortFree (DFx->I2C_data);
        }

    }
    //GPIOB->BRR = GPIO_Pin_4; // -> LOW debug
    //GPIOA->BRR = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; // -> LOW debug
}
//------------------------------------------------------------------------------------------------
void I2C1_EV_IRQHandler(void)
{
    I2Cx_EV_IRQHandler (I2C1, &I2C1_DataFrame, I2C1_Mutex, I2C1_rx_msg_queue);
}
//------------------------------------------------------------------------------------------------
void I2C2_EV_IRQHandler(void)
{
    I2Cx_EV_IRQHandler (I2C2, &I2C2_DataFrame, I2C2_Mutex, I2C2_rx_msg_queue);
}
//------------------------------------------------------------------------------------------------
void I2C_init (I2C_TypeDef* I2Cx)
{
    /* Enable AFIO and GPIOB clock */
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;

    if (I2Cx == I2C1)
    {
        RCC->APB1ENR    |= RCC_APB1ENR_I2C1EN; //Enable I2C2
        /* Configure I2C1 SDA (PB7)*/
        GPIOB->CRL      |= GPIO_CRL_MODE7;//11: Output mode, max speed 50 MHz.
        GPIOB->CRL      |= GPIO_CRL_CNF7; //11: Alternate function output Open-drain
        /* Configure I2C1 SCL (PB6)*/
        GPIOB->CRL      |= GPIO_CRL_MODE6; //11: Output mode, max speed 50 MHz.
        GPIOB->CRL      |= GPIO_CRL_CNF6; //11: Alternate function output Open-drain

        /* Reset I2C2 */
        RCC->APB1RSTR   |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR    &= ~RCC_APB1RSTR_I2C1RST;


        NVIC_EnableIRQ (I2C1_EV_IRQn);
        I2C1_rx_msg_queue = xQueueCreate (I2C1_QUEUE_LEN, I2C1_MAX_DATA_SIZE);
        I2C1_Mutex = xSemaphoreCreateMutex();
    }
    else
    {
        RCC->APB1ENR    |= RCC_APB1ENR_I2C2EN; //Enable I2C2
        /* Configure I2C2 SDA (PB11)*/
        GPIOB->CRH      |= GPIO_CRH_MODE11;//11: Output mode, max speed 50 MHz.
        GPIOB->CRH      |= GPIO_CRH_CNF11; //11: Alternate function output Open-drain
        /* Configure I2C2 SCL (PB10)*/
        GPIOB->CRH      |= GPIO_CRH_MODE10; //11: Output mode, max speed 50 MHz.
        GPIOB->CRH      |= GPIO_CRH_CNF10; //11: Alternate function output Open-drain

        /* Reset I2C2 */
        RCC->APB1RSTR   |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR    &= ~RCC_APB1RSTR_I2C2RST;


        NVIC_EnableIRQ (I2C2_EV_IRQn);
        I2C2_rx_msg_queue = xQueueCreate (I2C2_QUEUE_LEN, I2C2_MAX_DATA_SIZE);
        I2C2_Mutex = xSemaphoreCreateMutex();
    }


    I2Cx->CR1 |= 0x8000;
    I2Cx->CR1 &= ~0x8000;
    /* Configure I2C2 */
    I2Cx->CR2       |= (I2C_CR2_FREQ_2 | I2C_CR2_FREQ_5); //Peripheral clock frequency 36 MHz
    I2Cx->CCR       = 0x801E; //fast mode, 400 KHz 2/3 duty cycle
    I2Cx->TRISE     |=0x0E; //Rise time 300ns
    I2Cx->OAR1      =0x4000; // 7-bit addressing
    I2Cx->CR1       |= I2C_CR1_PE; //Peripheral enable
    I2Cx->CR2       |= I2C_CR2_ITBUFEN; //1:TxE = 1 or RxNE = 1 generates Event Interrupt
    I2Cx->CR2       |= I2C_CR2_ITEVTEN; //Event interrupt enable
}
//------------------------------------------------------------------------------------------------
void I2C_Write (I2C_TypeDef* I2Cx, uint8_t Address, uint8_t * data, uint16_t len)
{
    I2C_data_frame * DFx;
    //TODO: Data length (len) must be <= I2C1_MAX_DATA_SIZE or I2C2_MAX_DATA_SIZE, check is required
    if (I2Cx == I2C1) DFx = &I2C1_DataFrame;
    else DFx = &I2C2_DataFrame;

    DFx->I2C_addr = Address & OAR1_ADD0_Reset;
    DFx->I2C_data = data;
    DFx->I2C_len = len;
    DFx->I2C_pointer = 0;
    DFx->Transmit = 1;

    I2Cx->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    //I2C2->CR1 &= ~I2C_CR1_STOP;
    I2Cx->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation
}
//------------------------------------------------------------------------------------------------
void I2C_Read (I2C_TypeDef* I2Cx, uint8_t Address, uint16_t len)
{
    I2C_data_frame * DFx;
    //TODO: Data length (len) must be <= I2C1_MAX_DATA_SIZE or I2C2_MAX_DATA_SIZE, check is required
    if (I2Cx == I2C1) DFx = &I2C1_DataFrame;
    else DFx = &I2C2_DataFrame;

    DFx->I2C_addr = Address | OAR1_ADD0_Set;
    DFx->I2C_data = pvPortMalloc (len);
    DFx->I2C_len = len;
    DFx->I2C_pointer = 0;
    DFx->Transmit = 0;

    if (len == 2)
    {
       I2Cx->CR1 |=  I2C_CR1_POS;
    }
    I2Cx->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    I2Cx->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation
}
