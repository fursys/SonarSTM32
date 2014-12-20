
#include<stdlib.h>
#include "stm32f10x.h"
#include "Sonar.h"

uint8_t I2C_addr;
uint8_t * I2C_data;
uint16_t I2C_len;
uint16_t I2C_pointer = 0;

static portBASE_TYPE xHigherPriorityTaskWoken;
xQueueHandle I2C_rx_msg_queue;

void I2C2_EV_IRQHandler(void)
{
    uint16_t sr_1_reg = I2C2->SR1;
    if (sr_1_reg & I2C_SR1_SB) //I2C START event
    {
        I2C2->DR = I2C_addr; //Send Address
    }
    else if (sr_1_reg & I2C_SR1_ADDR) //Address sent (Master)
    {
        //If transmitter, then send byte
        if (I2C2->SR2 & I2C_SR2_TRA) I2C2->DR = I2C_data[I2C_pointer++];
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
            }
            else if (I2C_len == 2)
            {
                I2C2->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
            }

        }
    }
    else if (sr_1_reg & I2C_SR1_BTF) //Data byte transfer finished
    {
        //If transmitter, then send byte
        if (I2C2->SR2 & I2C_SR2_TRA)
        {
         I2C2->CR1 |= I2C_CR1_STOP;
         I2C_pointer = 0;
        }
        else //If receiver
        {
            if (I2C_len == 2)
            {
                I2C2->CR1 |= I2C_CR1_STOP;
                I2C_data[I2C_pointer++] = I2C2->DR;
                I2C_data[I2C_pointer++] = I2C2->DR;
                I2C_pointer = 0;
                xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendToBackFromISR(I2C_rx_msg_queue, &I2C_data, &xHigherPriorityTaskWoken);
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
                vPortFree (I2C_data);
            }
            else if (I2C_pointer==I2C_len-2)
            {
                I2C2->CR1 &= ~I2C_CR1_ACK; //Acknowledge disable
                I2C_data[I2C_pointer++] = I2C2->DR;
                I2C2->CR1 |= I2C_CR1_STOP;
                I2C_data[I2C_pointer++] = I2C2->DR;
            }
        }

    }
    else if (sr_1_reg & I2C_SR1_TXE) //Transmit buffer empty
    {
        if (I2C_pointer<I2C_len) I2C2->DR = I2C_data[I2C_pointer++];
    }
    else if (sr_1_reg & I2C_SR1_RXNE) //Receive buffer not empty
    {
        if (I2C_len != 2)
        {

            if (I2C_pointer==I2C_len-1)
            {
                I2C_data[I2C_pointer++] = I2C2->DR;
                I2C2->CR1       &= ~I2C_CR1_ACK; //Acknowledge disable
                I2C2->CR1 |= I2C_CR1_STOP;
            }
            else if (I2C_pointer==I2C_len-2)
            {

            }

            else I2C_data[I2C_pointer++] = I2C2->DR;
        }
        else if (I2C_len !=1 )
        {
            I2C_data[I2C_pointer++] = I2C2->DR;
            I2C_pointer = 0;
            xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendToBackFromISR(I2C_rx_msg_queue, &I2C_data, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);
            vPortFree (I2C_data);
        }


}


void I2C_init (void)
{
    /* Enable AFIO and GPIOA clock */
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR    |= RCC_APB1ENR_USART2EN;

}
    /* Enable AFIO and GPIOB clock */
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR    |= RCC_APB1ENR_I2C2EN;

    /* Configure I2C2 SDA (PB11)*/
    GPIOB->CRH      &= ~GPIO_CRH_CNF;
    GPIOB->CRH      &= ~GPIO_CRH_MODE11;//00: Input mode (reset state)
    GPIOB->CRH      |= GPIO_CRH_CNF10_0; // 01: Floating input (reset state)
    /* Configure I2C2 SCL (PB10)*/
    GPIOB->CRH      |= GPIO_CRH_CNF10; //11: Alternate function output Open-drain
    GPIOB->CRH      |= GPIO_CRH_MODE10; //11: Output mode, max speed 50 MHz.

    /* Configure I2C2 */
    I2C2->CR2       |= (I2C_CR2_FREQ_2 | I2C_CR2_FREQ_5); //Peripheral clock frequency 36 MHz
    I2C2->CCR       = 0x801E; //fast mode, 400 KHz 2/3 duty cycle
    I2C2->TRISE     |=0x0E; //Rise time 300ns
    I2C2->OAR1      |=0x4000; // 7-bit addressing
    I2C2->CR1       |= I2C_CR1_PE; //Peripheral enable
    //I2C2->CR2       |= I2C_CR2_ITBUFEN; //1:TxE = 1 or RxNE = 1 generates Event Interrupt
    I2C2->CR2       |= I2C_CR2_ITEVTEN; //Event interrupt enable

    NVIC_EnableIRQ (I2C2_EV_IRQn);
}
void I2C_Write (uint8_t Address, uint8_t * data, uint16_t len)
{

    I2C_addr = Address;
    I2C_data = data;
    I2C_len = len;
    I2C_pointer = 0;

    I2C2->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    I2C2->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation

}
void I2C_Read (uint8_t Address, uint16_t len)
{
    I2C_addr = Address;
    I2C_data = pvPortMalloc (len);
    I2C_len = len;
    I2C_pointer = 0;

    if (len == 2)
    {
       I2C2->CR1 |=  I2C_CR1_POS;
    }
    I2C2->CR1       |= I2C_CR1_ACK; //Acknowledge enable
    I2C2->CR1       |= I2C_CR1_START; //This bit is set and cleared by software and cleared by hardware when start is sent or PE=0. 1: Repeated start generation
}
