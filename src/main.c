/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/

#include "stm32f10x.h"

#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "XBEE.h"
#include "I2C.h"
#include "gpiodef.h"


// standart PWM 50 Hz
#define PWM_ARR 0xFFFF
#define PWM_PSC 0x16 //0x0B
#define MAX_THROTTLE_VAL 0x15AE//0x0CCD
#define MIN_PWM_VAL 0x0672//0x0CCD
#define SERVO_STARTUP_POS 0x0AD7
#define SONAR_ADDR 0xE1
#define SONAR_STARTUP_DELAY 5000
#define SERVO_STEP 100

#define I2C_MUTEX_WAIT_TIME 10


#define SENSORS_INTERVAL 50
#define XBEE_ADDR 2
#define SERVO_MOVE_ON_STARTUP 0

#define ACCEL_ADDR 	0x30
#define MAG_ADDR 	0x3C
#define GYRO_ADDR	0xD0
#define PressAddr   0xB8

volatile xSemaphoreHandle XBMutex;
volatile xSemaphoreHandle SonarMutex;

char strResult1 [27];
uint8_t Buf_Tx[2];


uint16_t ServoValue = 0;
uint8_t ServoDir = 1;
uint16_t Range = 0;



ParametersStruct Params;

void mag_init ( void)
{

	xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x00;
	Buf_Tx[1] = 0x18;
	I2C_Write (I2C1, MAG_ADDR,Buf_Tx,2);

    xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x02;
	Buf_Tx[1] = 0x00;
	I2C_Write (I2C1, MAG_ADDR,Buf_Tx,2);

	xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x01;
	Buf_Tx[1] = 0x20; //Gain X/Y 1055, Z 950
	I2C_Write (I2C1, MAG_ADDR,Buf_Tx,2);

	xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x03;
	Buf_Tx[1] = 0x00; //Gain X/Y 1055, Z 950
	I2C_Write (I2C1, MAG_ADDR,Buf_Tx,1);

}
//------------------------------------------------------------------------------------------------
void accel_init ( void)
{

	xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x20;
	Buf_Tx[1] = 0x2F;
    I2C_Write (I2C1, ACCEL_ADDR,Buf_Tx,2);

    xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x23;
	Buf_Tx[1] = 1<<5 | 1<<4 | 1<<7; //set sensivity 8g BDU- on
	I2C_Write (I2C1, ACCEL_ADDR,Buf_Tx,2);
}
//------------------------------------------------------------------------------------------------
void gyro_init ( void)
{

	xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x20;
	Buf_Tx[1] = 0x1F; //Power on
    I2C_Write (I2C1, GYRO_ADDR,Buf_Tx,2);

    xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x23;
	Buf_Tx[1] = 1<<5 | 1<<7; //FS = 2000 dps. BDU = on
    I2C_Write (I2C1, GYRO_ADDR,Buf_Tx,2);

    xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x24;
	Buf_Tx[1] = 1<<6; //FIFO enable.
    I2C_Write (I2C1, GYRO_ADDR,Buf_Tx,2);

    xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
	Buf_Tx[0] = 0x2E;
	Buf_Tx[1] = 0; //FIFO bypass mode.
    I2C_Write (I2C1, GYRO_ADDR,Buf_Tx,2);
}
//------------------------------------------------------------------------------------------------
void reverse(char s[])
{
	int i, j;
	char c;
	for (i = 0, j = strlen(s)-1; i<j; i++, j--)
	{
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}
//------------------------------------------------------------------------------------------------
void itoa(int n, char s[], int *len)
{
	int i, sign;
	if ((sign = n) < 0) //записываем знак
		n = -n; // делаем n положительным числом
	i = 0;
	do
	{ //генерируем цифры в обратном пор€дке
		s[i++] = n % 10 + '0'; //берем следующую цифру
	} while ((n /= 10) > 0); // удал€ем
	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';
	reverse(s);
	*len = i;
}
//------------------------------------------------------------------------------------------------
void I2C_error (uint8_t step)
{
    char strResult [2];
    strResult [0] = 255;
    strResult [1] = step;
    xSemaphoreTake( XBMutex, portMAX_DELAY );
    XB_send_data (strResult, 20, 2, 1);
    xSemaphoreGive( XBMutex );
    vTaskDelay( portMAX_DELAY );


}
//------------------------------------------------------------------------------------------------
void SensorsRead( void *pvParameters )
{


    uint8_t * out;
    uint8_t Buf_Tx1 [1];

    accel_init();
    gyro_init ();
    mag_init ();
    //uint8_t * sframes;

  	//size_t FreeHeap;
  	//uint8_t * freeHeapPtr;

    while( 1 )
    {
        //GPIOA->BSRR = GPIO_Pin_9; // -> HIGHT debug
        //Accel read
        Buf_Tx1[0] = 0xA8; //Set accel first data register address
        if (xSemaphoreTake( I2C1_Mutex, I2C_MUTEX_WAIT_TIME ) == pdFALSE ) I2C_error (1);

        I2C_Write (I2C1, ACCEL_ADDR,Buf_Tx1,1);

        if (xSemaphoreTake( I2C1_Mutex, I2C_MUTEX_WAIT_TIME ) == pdFALSE ) I2C_error (2);
        I2C_Read(I2C1, ACCEL_ADDR, 6);
        if (xQueueReceive (I2C1_rx_msg_queue, &out, portMAX_DELAY) == pdFALSE) I2C_error (3);


        strResult1 [1] = out[0];
        strResult1 [2] = out[1];
        strResult1 [3] = out[2];
        strResult1 [4] = out[3];
        strResult1 [5] = out[4];
        strResult1 [6] = out[5];
        //end Accel read


        //Gyro read
        Buf_Tx1[0] = 0xA8; //Set gyro first data register address
        if (xSemaphoreTake( I2C1_Mutex, I2C_MUTEX_WAIT_TIME ) == pdFALSE ) I2C_error (4);
        I2C_Write (I2C1, GYRO_ADDR,Buf_Tx1,1);

        if (xSemaphoreTake( I2C1_Mutex, I2C_MUTEX_WAIT_TIME ) == pdFALSE ) I2C_error (5);
        I2C_Read(I2C1, GYRO_ADDR, 6);
        if (xQueueReceive (I2C1_rx_msg_queue, &out, portMAX_DELAY) == pdFALSE) I2C_error (6);



        strResult1 [7] = out[0];
        strResult1 [8] = out[1];
        strResult1 [9] = out[2];
        strResult1 [10] = out[3];
        strResult1 [11] = out[4];
        strResult1 [12] = out[5];
        //end Gyro read

        //MAg read
        //xSemaphoreTake( I2C1_Mutex, portMAX_DELAY );
        //Buf_Tx1[0] = 0x03;//Set mag first data register address
        //I2C_Write (I2C1, MAG_ADDR,Buf_Tx1,1);
        if (xSemaphoreTake( I2C1_Mutex, I2C_MUTEX_WAIT_TIME ) == pdFALSE ) I2C_error (7);
        I2C_Read(I2C1, MAG_ADDR, 7);
        if (xQueueReceive (I2C1_rx_msg_queue, &out, portMAX_DELAY) == pdFALSE) I2C_error (8);

        strResult1 [13] = out[0];
        strResult1 [14] = out[1];
        strResult1 [15] = out[2];
        strResult1 [16] = out[3];
        strResult1 [17] = out[4];
        strResult1 [18] = out[5];

/*
        sframes = &sent_frames;
        strResult1 [19] = sframes[0];
        strResult1 [20] = sframes[1];
        strResult1 [21] = sframes[2];
        strResult1 [22] = sframes[3];


        //end Mag read




        FreeHeap = xPortGetFreeHeapSize();
        freeHeapPtr = &FreeHeap;

        strResult1 [23] = freeHeapPtr[0];
        strResult1 [24] = freeHeapPtr[1];
        strResult1 [25] = freeHeapPtr[2];
        strResult1 [26] = freeHeapPtr[3];
*/
        strResult1 [0] = 3;

        //GPIOA->BRR = GPIO_Pin_9; // -> LOW debug
        //Send data
        //GPIOA->BSRR = GPIO_Pin_10; // -> HIGHT debug
        if (xSemaphoreTake( XBMutex, portMAX_DELAY ) == pdFALSE) I2C_error (9);
        //GPIOA->BSRR = GPIO_Pin_11; //-> HIGHT debug
        XB_send_data (strResult1, 19, XBEE_ADDR, 2);
        //GPIOA->BRR = GPIO_Pin_11; // -> LOW debug
        xSemaphoreGive( XBMutex );

        //GPIOA->BRR = GPIO_Pin_10; // -> LOW debug

        //vTaskDelay( 14 / portTICK_RATE_MS );
        vTaskDelay( Params.sensors_interval / portTICK_RATE_MS );

    }
}
//------------------------------------------------------------------------------------------------
void Blink2( void *pvParameters )
{
  	volatile int State=0;


	while( 1 ){
            switch( State ){
                case 0:
                    GPIOB->BRR = GPIO_Pin_15; // -> LOW
                    State = 1;
                    break;

                case 1:
                    GPIOB->BSRR = GPIO_Pin_15; //-> HIGHT
                    State = 0;
                    break;
                default:
                    State = 0;
                    break;
            }

		vTaskDelay( Params.sensors_interval / portTICK_RATE_MS );
	}
}
//---------------------------------------------------------------------------
//Command parcer task
void CommandParcer ( void *pvParameters )
{
    xb_message current_msg;
    uint8_t  buf [(sizeof (Params) + 1)];
    //uint8_t msg_type = 0;
	while(1)
	{
		xQueueReceive (xb_rx_msg_queue, &current_msg,portMAX_DELAY); //Get message from Queue
		//msg_type = current_msg.msg[0];
		switch (current_msg.msg[0])
		{
            case 1: //empty

                break;
            case 2: //empty

                break;
            case 6: //set parameters
                memcpy (&Params, &current_msg.msg[1], current_msg.lenght - 3);
                break;
            case 7://read parameters

                //buf = pvPortMalloc (sizeof (Params) + 1);
                buf [0] = 7;
                memcpy (&buf[1], &Params, sizeof (Params));
                xSemaphoreTake( XBMutex, portMAX_DELAY );
                XB_send_data (buf, sizeof (Params) + 1, XBEE_ADDR, 1);
                xSemaphoreGive( XBMutex );
                //vPortFree (buf);
                break;
            default:
                break;
		}

		vPortFree (current_msg.msg);
		current_msg.msg = NULL;

	}
}

//------------------------------------------------------------------------------------------------
/*
void xbTest( void *pvParameters )
{
    uint8_t ar [100];
    uint8_t * sframes;

  	size_t FreeHeap;
  	uint8_t * freeHeapPtr;

    uint32_t frame_no = 0;
    uint8_t * frame_no_ptr;
    while( 1 )
    {

        ar[0] = 4;

        sframes = &sent_frames;
        ar [1] = sframes[0];
        ar [2] = sframes[1];
        ar [3] = sframes[2];
        ar [4] = sframes[3];



        FreeHeap = xPortGetFreeHeapSize();
        freeHeapPtr = &FreeHeap;

        ar [5] = freeHeapPtr[0];
        ar [6] = freeHeapPtr[1];
        ar [7] = freeHeapPtr[2];
        ar [8] = freeHeapPtr[3];

        frame_no++;
        frame_no_ptr=&frame_no;
        ar [9] = frame_no_ptr[0];
        ar [10] = frame_no_ptr[1];
        ar [11] = frame_no_ptr[2];
        ar [12] = frame_no_ptr[3];


        xSemaphoreTake( XBMutex, portMAX_DELAY );
        XB_send_data (ar, 30, 2, 1);
        xSemaphoreGive( XBMutex );

        vTaskDelay( Params.sensors_interval / portTICK_RATE_MS );
    }
}*/
//------------------------------------------------------------------------------------------------
void Sonar( void *pvParameters )
{
    uint8_t  tx_data [2];
    uint8_t * rx_data;
    char strResult [10];

    vTaskDelay( SONAR_STARTUP_DELAY / portTICK_RATE_MS );
	while( 1 )
    {
        Range = 0;
        tx_data [0] = 0x00;
        tx_data [1] = 0x51;
        //xSemaphoreTake( SonarMutex, portMAX_DELAY );

        xSemaphoreTake( I2C2_Mutex, portMAX_DELAY );
        I2C_Write (I2C2, SONAR_ADDR,tx_data,2);
        vTaskDelay( 70 / portTICK_RATE_MS );

        //Move sonar
        if (Params.servo_move)
        {
            if (ServoDir)
            {
                TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
                ServoValue+=Params.servo_step;
            }
            else
            {
                TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
                ServoValue-=Params.servo_step;
            }

            if (ServoValue>= MAX_THROTTLE_VAL) ServoDir = 0;
            else if (ServoValue <= 0) ServoDir = 1;
            vTaskDelay( 10 / portTICK_RATE_MS );
        }
        //end sonar move

        //Read sonar sequence
        tx_data [0] = 0x02;
        xSemaphoreTake( I2C2_Mutex, portMAX_DELAY );
        I2C_Write (I2C2, SONAR_ADDR,tx_data,1);
        xSemaphoreTake( I2C2_Mutex, portMAX_DELAY );
        I2C_Read(I2C2, SONAR_ADDR, 2);
        xQueueReceive (I2C2_rx_msg_queue, &rx_data, portMAX_DELAY);

        strResult [0] = 2;
        strResult [1] = rx_data[0];
        strResult [2] = rx_data[1];
        strResult [3] = ServoValue & 0xFF;
        strResult [4] = (ServoValue>>8) & 0xFF;

        xSemaphoreTake( XBMutex, portMAX_DELAY );
        XB_send_data (strResult, 5, XBEE_ADDR, 1);
        xSemaphoreGive( XBMutex );

        vTaskDelay( 100 / portTICK_RATE_MS );

	}
}
//------------------------------------------------------------------------------------------------
void ServoInit(void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_TIM1EN;	//TIM2 RCC enable
    RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
	//AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP;
    GPIOA->CRH      &= ~GPIO_CRH_CNF8;
	GPIOA->CRH      |= (GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1); //пин ставим на выход, альтернативна€ функци€


	TIM1->CR1       = 0x00000000;       //сброс настроек таймера (можно не делать)
	TIM1->ARR       = PWM_ARR;           // макс. значение до которого считаем
	TIM1->CR1       |= TIM_CR1_ARPE;    //Ѕуферизаци€ регистра ARR (макс. значение до которого считаем)
	TIM1->PSC       = PWM_PSC;           // пределитель
	TIM1->CCR1      = MIN_PWM_VAL + SERVO_STARTUP_POS;	//Counter value

	//TIM2->CCMR2 =  = 0x00000000;

    //”станавливаем режим Ў»ћ
    TIM1->CCMR1     |= TIM_CCMR1_OC1PE
                    | TIM_CCMR1_OC1M_1
                    | TIM_CCMR1_OC1M_2; //(TIM_CCMR1_OC1PE - загрузка значений только по обновлению
                                        //TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 - режим Ў»ћ 1 <110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
                                        //else inactive. In downcounting, channel 1 is inactive (OC1REF=С0Т) as long as
                                        //TIMx_CNT>TIMx_CCR1 else active (OC1REF=Т1Т).>  )


	//TIM1->DIER 		|= TIM_DIER_CC3IE; //Match interrupt enable
	TIM1->CCER      |= TIM_CCER_CC1E; //разрешаем выход канала 1
    TIM1->BDTR      |= TIM_BDTR_MOE;    //включаем выходы (б...! еще раз! Ќахрена?)
    TIM1->EGR       |= TIM_EGR_UG;      //сброс счетчика в 0
    TIM1->CR1       |= TIM_CR1_CEN;      //разрешаем работу таймера
}
//------------------------------------------------------------------------------------------------
int main(void)
{

    RCC->APB2ENR	|= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;                    //включаю тактирование порта (если не включали ранее)
    RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
    //Blink port
	//GPIOB->CRH		&= ~ (0xFFFF);
	GPIOA->CRH      |= (GPIO_CRH_MODE9_1);//Output mode, max speed 2 MHz.
    GPIOA->CRH      &= ~GPIO_CRH_CNF9;

    GPIOA->CRH      |= (GPIO_CRH_MODE10_1);//Output mode, max speed 2 MHz.
    GPIOA->CRH      &= ~GPIO_CRH_CNF10;

    GPIOA->CRH      |= (GPIO_CRH_MODE11_1);//Output mode, max speed 2 MHz.
    GPIOA->CRH      &= ~GPIO_CRH_CNF11;

    AFIO->MAPR      |= AFIO_MAPR_SWJ_CFG_1;
    GPIOB->CRL      |= (GPIO_CRL_MODE4_1);//Output mode, max speed 2 MHz.
    GPIOB->CRL      &= ~GPIO_CRL_CNF4;
    //GPIOB->CRL      |= GPIO_CRL_CNF4_1;
    GPIOB->BRR = GPIO_Pin_4; // -> LOW



    GPIOB->CRH      |= (GPIO_CRH_MODE15_1);//Output mode, max speed 2 MHz.
    GPIOB->CRH      &= ~GPIO_CRH_CNF15;

    Params.sensors_interval = SENSORS_INTERVAL;
    Params.servo_step = SERVO_STEP;
    Params.servo_move = SERVO_MOVE_ON_STARTUP;
    //Params.servo_move = 1;

    ServoInit();

    XBMutex = xSemaphoreCreateMutex();
    SonarMutex = xSemaphoreCreateMutex();
    XB_init ();
    I2C_init(I2C2);
    I2C_init(I2C1);
    //TIM1->CCR1 = MIN_PWM_VAL;
    //TIM1->CCR1 = MIN_PWM_VAL+MAX_THROTTLE_VAL;

	xTaskCreate( Blink2,"Blink2", 64, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate(SensorsRead,"SensorsRead", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate(Sonar,"Sonar", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    //xTaskCreate(xbTest,"xbTest", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate(CommandParcer,"CommandParcer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();
    return 0;
}


