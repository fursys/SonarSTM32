#include<stdlib.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "XBEE.h"
#include <string.h>


xSemaphoreHandle XB_send_Semaphore;
//uint8_t xb_rx_buffer [110];
uint8_t xb_tx_buffer [110];

uint16_t dropped_frames = 0;
uint32_t sent_frames = 0;

uint8_t ReceivingState = 0;
uint8_t ByteReceived = 0;

uint8_t Received_RSSI = 0;
uint8_t tx_frame_id = 1;
//xSemaphoreHandle xb_rx_semaphore;

xQueueHandle xb_received_frames;


xQueueHandle xb_rx_msg_queue;
xQueueHandle xb_tx_msg_queue;
xb_frame rx_frame;
xb_frame tx_frame;

xb_message cur_tx_message;

/*
struct tx_pack_16
{
	uint8_t delimiter = 0x7E;
	uint16_t lenght;
	uint8_t api_identifier = 0x01;
	uint8_t frame_id;
	uint16_t dest_address;
	uint8_t options = 0;
	uint8_t * rf_data;

};

*/
uint8_t CalcCheckSumm(uint8_t * packet, uint8_t p_lenght)
{
	uint16_t res = 0;
	for (uint8_t i = 3; i < p_lenght-1; i++)
	{
		res = (uint16_t) (0xFF & (res + packet [i]));
	}
	return (uint8_t) (0xFF - res);
}
//============================================================================================
uint8_t IsCheckSummCorrect (uint8_t * packet, uint8_t p_lenght)
{
	uint16_t res = 0;
	for (uint8_t i = 0; i < p_lenght ; i++)
	{
		res = (uint16_t)(0xFF & (res + packet [i]));
	}
	if (res == 0xFF) return 1;
	else return 0;
}
//============================================================================================
//********************************************************************************
//Function: ����� ������ � ������ "������-DMA-USART1"                           //
//Argument: ���������� ������ � ������                                          //
//********************************************************************************
void XB_StartDMA_RX(unsigned int LengthBufer, uint8_t * ar)
{
  DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR6_EN); //��������� ������ ������
  DMA1_Channel6->CMAR = (uint32_t)ar;
  DMA1_Channel6->CNDTR =  LengthBufer;      //��������� ���������� ������ ��� ������
  DMA1->IFCR          |=  DMA_IFCR_CTCIF6;  //�������� ���� ��������� ������
  DMA1_Channel6->CCR  |=  DMA_CCR6_EN;      //��������� ������ ������
}
//============================================================================================
void XB_StartDMA_TX (unsigned int LengthBufer, uint8_t * ar)
{
  DMA1_Channel7->CCR &= (uint16_t)(~DMA_CCR7_EN); //��������� ������ ������
  DMA1_Channel7->CMAR = (uint32_t)ar;
  DMA1_Channel7->CNDTR =  LengthBufer;      //��������� ���������� ������ ��� ������
  DMA1->IFCR          |=  DMA_IFCR_CTCIF7;  //�������� ���� ��������� ������
  DMA1_Channel7->CCR  |=  DMA_CCR7_EN;      //��������� ������ ������

  //GPIO_SetBits( GPIOA, GPIO_Pin_9 );
}
//============================================================================================
void XB_RCC_Configuration(void)
{

    /* Enable AFIO and GPIOA clock */
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR    |= RCC_APB1ENR_USART2EN;
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

//============================================================================================
void XB_GPIO_Configuration(void)
{
    /* Configure USART Tx as alternate function push-pull */
    GPIOA->CRL      &= ~GPIO_CRL_CNF2;
    GPIOA->CRL      |= GPIO_CRL_MODE2_1;//Output mode, max speed 2 MHz.
    GPIOA->CRL      |= GPIO_CRL_CNF2_1;
    /* Configure USART Rx as input floating */
    GPIOA->CRL      &= ~GPIO_CRL_CNF3;
    GPIOA->CRL      &= ~GPIO_CRL_MODE3; //00: Input mode (reset state)
    GPIOA->CRL      |= GPIO_CRL_CNF3_0; //01: Floating input (reset state)
}

//============================================================================================
void DMA1_Channel7_IRQHandler (void)//TX DMA IRQ handler
{
  //���� ����� ��������
  if(DMA1->ISR & DMA_ISR_TCIF7)
  {
	//���-�� ������
	//vPortFree (tx_frame.frame_array);
	//tx_frame.frame_array = NULL;
	//xSemaphoreGive( XB_send_Semaphore);
    DMA1->IFCR = DMA_IFCR_CTCIF7; //������� ��� ����������
	//GPIO_ResetBits( GPIOB, GPIO_Pin_15 );

	//GPIO_ResetBits( GPIOA, GPIO_Pin_9 ); //debug bit
  }

  //���� �������� �������� ������
  if(DMA1->ISR & DMA_ISR_HTIF7)
  {
	DMA1->IFCR |= DMA_IFCR_CHTIF7; //������� ��� ����������
  }      //���-�� ������

  //���� ��������� ������ ��� ������
  if(DMA1->ISR & DMA_ISR_TEIF7)
  {
  	vPortFree (tx_frame.frame_array);
	tx_frame.frame_array = NULL;
	DMA1->IFCR |= DMA_IFCR_CTEIF7; //������� ��� ����������
  }      //���-�� ������

  DMA1->IFCR |= DMA_IFCR_CGIF7; //������� ��� ����������� ����������
}

//============================================================================================
void DMA1_Channel6_IRQHandler (void) //RX DMA IRQ handler
{
  //���� ����� ��������
  if(DMA1->ISR & DMA_ISR_TCIF6)
  {
	//���-�� ������


	USART2->CR3         &=  ~USART_CR3_DMAR; //��������� �������� USART ����� DMA
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //�������� ���������� �� ����� USART
	USART2->CR1 |=USART_CR1_RXNEIE;//�������� ���������� �� ����� USART
    DMA1->IFCR = DMA_IFCR_CTCIF6; //������� ��� ����������

	//������ ��������� � ������� ����������� ������
	static portBASE_TYPE xHigherPriorityTaskWoken;
	//GPIO_ResetBits( GPIOA, GPIO_Pin_11 );//debug bit
	//GPIO_SetBits( GPIOA, GPIO_Pin_10 ); //debug bit
	xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(xb_received_frames, &rx_frame, &xHigherPriorityTaskWoken);
	/* ��� ������������ ������-����������. ��� ���� ��������� ������-����������� ���� ���������� ������������� � ������ ������ ������������� ������. ������� ����������� �������� ������������� - ��� �� �������� ����, ��� ����� ���������� ����������� ���������� ���������� ������� ������ ����������.*/
	/* ������, ����������� ������������ ���������. �� ������ ���������� ��� ������� ����� ���� ������! */
	 portEND_SWITCHING_ISR(xHigherPriorityTaskWoken == pdTRUE);

  }

  //���� �������� �������� ������
  if(DMA1->ISR & DMA_ISR_HTIF6)
  {
	DMA1->IFCR |= DMA_IFCR_CHTIF6; //������� ��� ����������
  }      //���-�� ������

  //���� ��������� ������ ��� ������
  if(DMA1->ISR & DMA_ISR_TEIF6)
  {
  	vPortFree (rx_frame.frame_array); //������� ������
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //�������� ���������� USART
	USART2->CR1 &= ~USART_CR1_RXNEIE;
	DMA1->IFCR |= DMA_IFCR_CTEIF6; //������� ��� ����������
  }      //���-�� ������

  DMA1->IFCR |= DMA_IFCR_CGIF6; //������� ��� ����������� ����������
}
//============================================================================================
void USART2_IRQHandler(void)
{
  //if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  if(USART2->SR & USART_SR_RXNE)
  {
    /* Read one byte from the receive data register */
    ByteReceived = USART2->DR;
	switch (ReceivingState)
	{
		case 0: //Find packet header
			if (ByteReceived == 0X7E)
			{
				ReceivingState++;
				//GPIO_SetBits( GPIOA, GPIO_Pin_11 );//debug bit
			}
			break;
		case 1: //Receive lenght MSB
			rx_frame.lenght = ByteReceived <<8;
			ReceivingState++;
			break;
		case 2://Receive lenght LSB

			ReceivingState = 0;
			rx_frame.lenght |= ByteReceived;
			rx_frame.lenght++;//����� ��������� ���� ���� CRC

			USART2->CR3         |=  USART_CR3_DMAR;					//��������� �������� USART ����� DMA
			//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
			USART2->CR1 &= ~USART_CR1_RXNEIE;
			rx_frame.frame_array = pvPortMalloc (120);
			XB_StartDMA_RX (rx_frame.lenght, rx_frame.frame_array);	//Start DMA transfer
			break;
		default:
			break;
	}

      /* Disable the USART Receive interrupt */
      //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  }

  //if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  if (USART2->SR & USART_SR_TXE)
  {
      /* Disable the USART Transmit interrupt */
      //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      USART2->SR &= ~USART_SR_TXE;

  }
}

//********************************************************************************
//Function: ������������� DMA ��� ������ � USART (�������� ������)            //
//********************************************************************************
void XB_USART_TX_DMA_Init(void)
{
 //�������� ������������ DMA1
 if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != RCC_AHBENR_DMA1EN)
 RCC->AHBENR |= RCC_AHBENR_DMA1EN;
 //������ ����� ��������� � ��������� � ���������� ������ ��� ������
 DMA1_Channel7->CPAR  =  (uint32_t)&USART2->DR;   //����� �������� ���������
 DMA1_Channel7->CMAR  =  0;//(uint32_t)xb_tx_buffer;   	//����� ������ � ������
 DMA1_Channel7->CNDTR =  6;                      //���������� ������ ��� ������
 //----------------- ����������� � ��������� ������������  ----------------
 //��������� �������� ����� ���������� � ���� ������� (������� ��� �����������)
 DMA1_Channel7->CCR   =  0;									//����������� �������� ������������
 DMA1_Channel7->CCR  &= (uint16_t) (~DMA_CCR7_CIRC);		//��������� ����������� �����
 DMA1_Channel7->CCR  |=  DMA_CCR7_DIR;            			//�����������: ������ �� ������
 //��������� ������ � ������������ �����������
 //DMA1_Channel7->CCR  &= (uint16_t)(~DMA_CCR7_PSIZE); 		//����������� ������ 8 ���
 DMA1_Channel7->CCR   &= ~DMA_CCR7_PSIZE;          //����������� ������ 8 ���
 DMA1_Channel7->CCR   &= (uint16_t)(~DMA_CCR7_PINC);		//�������������� ��������� ���������
 //��������� ������ � �������
 DMA1_Channel7->CCR   &= (uint16_t)(~DMA_CCR7_MSIZE);		//����������� ������ 8 ���
 DMA1_Channel7->CCR  |=  DMA_CCR7_MINC;						//������������ ��������� ���������

 //��������� ���������� �� ���������� ������:
 DMA1_Channel7->CCR |= DMA_CCR7_TCIE;						//����� 7
 NVIC_EnableIRQ (DMA1_Channel7_IRQn);						//��������� ���������� �� DMA
 NVIC_SetPriority (DMA1_Channel7_IRQn, 15);
  USART2->CR3         |=  USART_CR3_DMAT;					//��������� �������� USART ����� DMA
}
//********************************************************************************
//Function: ������������� DMA ��� ������ � USART (�������� ������)            //
//********************************************************************************
void XB_USART_RX_DMA_Init(void)
{
 //�������� ������������ DMA1
 if ((RCC->AHBENR & RCC_AHBENR_DMA1EN) != RCC_AHBENR_DMA1EN)
 RCC->AHBENR |= RCC_AHBENR_DMA1EN;
 //������ ����� ��������� � ��������� � ���������� ������ ��� ������
 DMA1_Channel6->CPAR  =  (uint32_t)&USART2->DR;       //����� �������� ���������
 DMA1_Channel6->CMAR  =  0;   				//����� ������ � ������ ����� ���������� ��� ������ ��������
 DMA1_Channel6->CNDTR =  6;                         //���������� ������ ��� ������
 //----------------- ����������� � ��������� ������������  ----------------
 //��������� �������� ����� ���������� � ���� ������� (������� ��� �����������)
 DMA1_Channel6->CCR   =  0;									//����������� �������� ������������
 DMA1_Channel6->CCR  &= (uint16_t) (~DMA_CCR6_CIRC);		//��������� ����������� �����
 DMA1_Channel6->CCR  &= (uint16_t) (~DMA_CCR6_DIR);         //�����������: ������ � ������
 //��������� ������ � ������������ �����������
 DMA1_Channel6->CCR   &= (uint16_t)(~DMA_CCR6_PSIZE);		//����������� ������ 8 ���
 DMA1_Channel6->CCR   &= (uint16_t)(~DMA_CCR6_PINC);		//�������������� ��������� ���������
 //��������� ������ � �������
 DMA1_Channel6->CCR   &= (uint16_t)(~DMA_CCR6_MSIZE);		//����������� ������ 8 ���
 DMA1_Channel6->CCR  |=  DMA_CCR6_MINC;						//������������ ��������� ���������

 //��������� ���������� �� ���������� ������:
 DMA1_Channel6->CCR |= DMA_CCR6_TCIE;						//����� 7
 NVIC_EnableIRQ (DMA1_Channel6_IRQn);						//��������� ���������� �� DMA
 NVIC_SetPriority (DMA1_Channel6_IRQn, 15);


}
//============================================================================================
void XB_RxPackParser( void *pvParameters )
{
	xb_frame current_frame;
	xb_message current_msg;
	portBASE_TYPE q_res;
	while (1)
	{
		//xSemaphoreTake( xb_rx_semaphore, portMAX_DELAY);
		xQueueReceive (xb_received_frames, &current_frame,portMAX_DELAY); //Get frame from Queue



		//��������� checksumm
		if (IsCheckSummCorrect (current_frame.frame_array,current_frame.lenght))
		{

			//���������� �����
			uint8_t data_pointer = 0;
			switch (current_frame.frame_array[data_pointer++])
			{
				case 0x81: //RX packet 16-bit address
					//GPIO_SetBits( GPIOB, GPIO_Pin_14 );
					current_msg.lenght = current_frame.lenght - 6;
					current_msg.msg = pvPortMalloc (current_msg.lenght);
					current_msg.type = current_frame.frame_array[0];
					current_msg.address = current_frame.frame_array[1] << 8;
					current_msg.address |= current_frame.frame_array[2];
					current_msg.rssi = current_frame.frame_array[3];
					current_msg.options = current_frame.frame_array[4];
					memcpy (current_msg.msg, current_frame.frame_array + 5,current_msg.lenght);
					current_msg.msg[current_msg.lenght++] = 0; //������� ��������� ������
					q_res = xQueueSendToBack (xb_rx_msg_queue, &current_msg, 4);//�������� �������� ����� � ������� ������-����������� �������
					if (q_res != pdPASS)
					{
						vPortFree (current_msg.msg);
						//TODO: ���� �� �������� � ������������ �������!!!
						//dropped_frames++;
					}
					break;
				case 0x89: //Transmit status

					//GPIO_SetBits( GPIOB, GPIO_Pin_11 ); //debug bit
					if ((tx_frame_id == current_frame.frame_array[data_pointer++]) && (current_frame.frame_array[data_pointer] == 0))
					{
                        sent_frames++;
						xSemaphoreGive( XB_send_Semaphore);
					}
					//GPIO_ResetBits( GPIOB, GPIO_Pin_11 ); //debug bit
					//GPIO_ResetBits( GPIOA, GPIO_Pin_10 ); //debug bit
					break;
				case 0x80: //RX packet 64-bit address
					break;
				case 0x97: //Remote AT command result
					break;
				case 0x88: //AT command response
					break;
				case 0x8A://Modem status
					break;
				default:
					break;
			}
			//��������� ������������� � ���������?


		}

		vPortFree (current_frame.frame_array);
		current_frame.frame_array = NULL;


	}

}
//============================================================================================
void XB_Sender( void *pvParameters )
{



//uint8_t tx_packet_data_lenght = 0;
uint8_t tx_packet_pointer = 0;

while(1)
{
	xQueueReceive (xb_tx_msg_queue, &cur_tx_message,portMAX_DELAY); //Get message from Queue
	//if (cur_tx_message.retries) GPIO_SetBits( GPIOA, GPIO_Pin_8 ); //debug bit
	//for (int i = 0; i < cur_tx_message.lenght; i += 100) //�������� �� 100 ����
	//{
		switch (cur_tx_message.type)
		{
			case 0x01://TX request 16-bit address

				//���������� ������ �������� ������
				//if (cur_tx_message.lenght - i >= 100) tx_packet_data_lenght = 100;
				//else tx_packet_data_lenght = cur_tx_message.lenght - i;
				//tx_frame.lenght = tx_packet_data_lenght + 9; //����� ������ ������ + ��������� ����������
				tx_frame.lenght = cur_tx_message.lenght + 9;
				//tx_frame.frame_array = pvPortMalloc (tx_frame.lenght); //��������� 4 ����� �� ��������� ���� ����� ������ � ����������� �����
				//tx_frame.frame_array = pvPortMalloc (120);
				tx_frame.frame_array = cur_tx_message.msg;

				tx_packet_pointer = 0;

				tx_frame.frame_array[tx_packet_pointer++] = 0x7E;
				tx_frame.frame_array[tx_packet_pointer++] = 0;
				tx_frame.frame_array[tx_packet_pointer++] = tx_frame.lenght - 4; //������ ����� ������
				tx_frame.frame_array[tx_packet_pointer++] = cur_tx_message.type; //API command ID
				if ((cur_tx_message.retries) > 0)
				{
					tx_frame.frame_array[tx_packet_pointer++] = tx_frame_id++ ? tx_frame_id : tx_frame_id++;
				}
				else //���� ���������� ������� �������� <= 0, ������ ����� ������ = 0, ������������� �������� �� �����
				{
					tx_frame.frame_array[tx_packet_pointer++] = 0;
				}
				tx_frame.frame_array[tx_packet_pointer++] = (0xFF & (cur_tx_message.address >> 8));	//Address MSB
				tx_frame.frame_array[tx_packet_pointer++] = (0xFF & (cur_tx_message.address));		//Address LSB
				tx_frame.frame_array[tx_packet_pointer++] = 0x00;                         		//Options 0x00 = Other;0x01 = Desable ASK; 0x04 = Send packet with Broadcast Pan ID

				//����� ������ ��� ������ � ��������
				//memcpy (&tx_frame.frame_array[tx_packet_pointer], &cur_tx_message.msg, cur_tx_message.lenght);
				tx_packet_pointer += cur_tx_message.lenght;
				tx_frame.frame_array[tx_packet_pointer] = CalcCheckSumm(tx_frame.frame_array,tx_frame.lenght);


				XB_StartDMA_TX (tx_frame.lenght, tx_frame.frame_array); //������� ��������� ���������

				if ( xSemaphoreTake( XB_send_Semaphore,30 / portTICK_RATE_MS) != pdTRUE )
				{
                    //GPIOA->BSRR = GPIO_Pin_9; //-> HIGHT debug

					//������������� �� �������� ������ �� ��������
					if ((cur_tx_message.retries--) > 0)
					{
						// � ���������� ������� �� ����� 0
						//������� ��� ���

						if (xQueueSendToFront(xb_tx_msg_queue, &cur_tx_message, 0) != pdPASS)
						{
						    //GPIOA->BSRR = GPIO_Pin_10; //-> HIGHT debug
							//���� ����� � ������� ��� ���, ������� ���������
							vPortFree (cur_tx_message.msg);
							dropped_frames++;
							//GPIOA->BRR = GPIO_Pin_10; // -> LOW debug
						}
					}
					else
					{
						//���� ������� ������ ���, ������� ���������
						vPortFree (cur_tx_message.msg);
						dropped_frames++;

					}
					//GPIOA->BRR = GPIO_Pin_9; // -> LOW debug
				}
				else
				{
					//GPIO_ResetBits( GPIOA, GPIO_Pin_8 ); //debug bit
					vPortFree (cur_tx_message.msg);
				}
				break;
			case 0x00: //TX request 64-bit address
				break;
			case 0x08: //AT Command
				break;
			case 0x09: //AT Command - Queue Parameter Value
				break;
			case 0x17: //Remote AT Command Request
				break;
			default:
				break;
		}

	//}
	//vPortFree(cur_tx_message.msg);
	//cur_tx_message.msg = NULL;
	//xSemaphoreTake( XB_send_Semaphore,200 / portTICK_RATE_MS);
	//xSemaphoreTake( XB_send_Semaphore, portMAX_DELAY);//���� ���� ����������� �����, ������ ����� ��������� � ��������� ����������.
														//������� �������� � ����������� ���������� ������ DMA �� ���������� �������� ������

}
}
//============================================================================================
void XB_init (void)
{
	XB_RCC_Configuration();
	XB_GPIO_Configuration ();

	USART2->BRR = 625; //57600


	//USART2->CR1 |= USART_CR1_TCIE; //1: A USART interrupt is generated whenever TC=1 in the USART_SR register
	USART2->CR1 |= USART_CR1_TE; //1: Transmitter is enabled
	USART2->CR1 |= USART_CR1_RE; //1: Receiver is enabled and begins searching for a start bit
	USART2->CR1 |= USART_CR1_RXNEIE; //1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register




	XB_USART_TX_DMA_Init ();
	XB_USART_RX_DMA_Init ();


	NVIC_EnableIRQ (USART2_IRQn);
	NVIC_SetPriority (USART2_IRQn,15);


    USART2->CR1 |=USART_CR1_UE; //1: USART enabled

	xb_received_frames = xQueueCreate (RX_QUEUE_LEN,sizeof(xb_frame));
	xb_tx_msg_queue = xQueueCreate (TX_QUEUE_LEN,sizeof(xb_message));
	xb_rx_msg_queue = xQueueCreate (RX_QUEUE_LEN,sizeof(xb_message));
	vSemaphoreCreateBinary(XB_send_Semaphore);
	xSemaphoreTake( XB_send_Semaphore, portTICK_RATE_MS);


	xTaskCreate( XB_RxPackParser,"XB_RxPackParser", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
	xTaskCreate( XB_Sender,"XB_Sender", configMINIMAL_STACK_SIZE, NULL, 1, NULL );


}
//============================================================================================
void XB_send_data (char *ar, int len, uint16_t addr, uint8_t retries)
{
	xb_message msg;
	portBASE_TYPE q_res; //���������� ��� �������� ���������� ���������� � �������
	//a = uxQueueMessagesWaiting(xb_tx_msg_queue);
	msg.msg = pvPortMalloc (len+10);
	//msg.msg = pvPortMalloc (120);
    if (msg.msg != NULL)
	{
		//��������� ������ ����� �� �������, ����� ����� �������� ��������� ���������� � ������ �������
		memcpy (msg.msg+8,ar,len);
		msg.type = 0x01;
		msg.lenght = len; //����� ������ ��� ����� ������!
		msg.address = addr;
		msg.retries = retries;

		q_res = xQueueSendToBack(xb_tx_msg_queue, &msg, 4);
		if (q_res != pdPASS)
		{
			vPortFree (msg.msg);
			dropped_frames++;
		}
	}
	else dropped_frames++;

}



