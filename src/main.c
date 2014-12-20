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


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "XBEE.h"
#include "Sonar.h"
#include "gpiodef.h"


// standart PWM 50 Hz
#define PWM_ARR 0xFFFF
#define PWM_PSC 0x16 //0x0B
#define MAX_THROTTLE_VAL 0x15AE//0x0CCD
#define MIN_PWM_VAL 0x0672//0x0CCD
#define SONAR_ADDR 0xE1
#define SONAR_STARTUP_DELAY 5000
#define SERVO_STEP 100


void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );





volatile xSemaphoreHandle XBMutex;
volatile xSemaphoreHandle SonarMutex;

uint16_t ServoValue = 0;
uint8_t ServoDir = 1;
uint16_t Range = 0;

/*
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    int a = 10,b = 12;
    if (&pcTaskName == "12")
    {
        a +=b;
    }
}
*/
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
void ServoMove( void *pvParameters )
{
    //char strResult [120];
	while( 1 )
    {
        //xSemaphoreTake( SonarMutex, portMAX_DELAY );

        if (ServoDir)
        {
            TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
            ServoValue+=100;
        }
        else
        {
            TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
            ServoValue-=100;
        }

        if (ServoValue>= MAX_THROTTLE_VAL) ServoDir = 0;
        else if (ServoValue <= 0) ServoDir = 1;

/*
        strResult [0] = 1;
        strResult [1] = ServoValue & 0xFF;
        strResult [2] = (ServoValue>>8) & 0xFF;


        xSemaphoreTake( XBMutex, portMAX_DELAY );
        XB_send_data (strResult, 3, 2, 0);
        xSemaphoreGive( XBMutex );
*/

        vTaskDelay( 30 / portTICK_RATE_MS );
        //xSemaphoreGive( SonarMutex );

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
		vTaskDelay( 100 / portTICK_RATE_MS );
	}
}
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
        I2C_Write (SONAR_ADDR,tx_data,2);
        vTaskDelay( 70 / portTICK_RATE_MS );

        //Move sonar
        if (ServoDir)
        {
            TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
            ServoValue+=SERVO_STEP;
        }
        else
        {
            TIM1->CCR1 = MIN_PWM_VAL + ServoValue;
            ServoValue-=SERVO_STEP;
        }

        if (ServoValue>= MAX_THROTTLE_VAL) ServoDir = 0;
        else if (ServoValue <= 0) ServoDir = 1;
        vTaskDelay( 10 / portTICK_RATE_MS );
        //end sonar move

        //Read sonar sequence
        tx_data [0] = 0x02;
        xSemaphoreTake( I2C2_Mutex, portMAX_DELAY );
        I2C_Write (SONAR_ADDR,tx_data,1);
        xSemaphoreTake( I2C2_Mutex, portMAX_DELAY );
        I2C_Read(SONAR_ADDR,2);
        xQueueReceive (I2C_rx_msg_queue, &rx_data, portMAX_DELAY);

        strResult [0] = 2;
        strResult [1] = rx_data[0];
        strResult [2] = rx_data[1];
        strResult [3] = ServoValue & 0xFF;
        strResult [4] = (ServoValue>>8) & 0xFF;

        xSemaphoreTake( XBMutex, portMAX_DELAY );
        XB_send_data (strResult, 5, 2, 0);
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
	TIM1->CCR1      = MIN_PWM_VAL;	//Counter value

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

    ServoInit();

    XBMutex = xSemaphoreCreateMutex();
    SonarMutex = xSemaphoreCreateMutex();
    XB_init ();
    I2C_init();
    //TIM1->CCR1 = MIN_PWM_VAL;
    //TIM1->CCR1 = MIN_PWM_VAL+MAX_THROTTLE_VAL;

	xTaskCreate( Blink2,"Blink2", 64, NULL, tskIDLE_PRIORITY, NULL );
    //xTaskCreate(ServoMove,"ServoMove", 64, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate(Sonar,"Sonar", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );



	/* Start the scheduler. */
	vTaskStartScheduler();
    return 0;
}


