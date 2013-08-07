
/* Includes ------------------------------------------------------------------*/

//#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"

#include "stm32f4_discovery.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_ON 0xF0
#define LED_OFF 0x0F

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char readData[32];
unsigned char writeData[32];
int rxCounter, txCounter;

#ifdef SPI_IT
typedef struct stqspi
{
	uint8_t led;
}QLED;
#endif

/* Private function prototypes -----------------------------------------------*/
uint8_t RecvData();

/* Private functions ---------------------------------------------------------*/


void init_SPI(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

#ifdef MASTER 


	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// enable SPI2 peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* configure pins used by SPI1
	* PE7 = NSS(CS)
	* PA5 = SCK
	* PA5 = MISO
	* PA7 = MOSI
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);


	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 |GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	

	GPIO_SetBits(GPIOE,GPIO_Pin_3);	//Let the LIS302DL disable
	GPIO_SetBits(GPIOE,GPIO_Pin_7); //Pull High

	/* configure SPI1 in Mode 0
	* CPOL = 0 --> clock is low when idle
	* CPHA = 0 --> data is sampled at the first edge
	*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master; // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft|SPI_NSSInternalSoft_Set ; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE); // enable SPI1	

#else //Slave mode

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// enable SPI2 peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* configure pins used by SPI2
	* PA15 = NSS(CS)
	* PA5 = SCK
	* PA5 = MISO
	* PA7 = MOSI
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7 |  GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI1);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	

	GPIO_SetBits(GPIOE,GPIO_Pin_3);		//Let the LIS302DL disable

	/* configure SPI1 in Mode 0
	* CPOL = 0 --> clock is low when idle
	* CPHA = 0 --> data is sampled at the first edge
	*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave; // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);
	//SPI_Cmd(SPI1, ENABLE); // enable SPI1	

	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

#endif	

}


/*
	User Button PA0
*/

void init_LED()
{
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  //STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);  

			STM_EVAL_LEDOff(LED4);
    		STM_EVAL_LEDOff(LED3);
    		STM_EVAL_LEDOff(LED5);
    		STM_EVAL_LEDOff(LED6);

}

//Init Push Button

void init_PB()
{

	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Here the GPIOA module is initialized.
	* We want to use PA0 as an input because
	* the USER button on the board is connected
	* between this pin and VCC.
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;	// we want to configure PA0
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // we want it to be an input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; // this enables the pulldown resistor --> we want to detect a high level
	GPIO_Init(GPIOA, &GPIO_InitStruct);	// this passes the configuration to the Init function which takes care of the low level stuff
}




#ifdef MASTER

// FOR MASTER Uesed 
uint8_t RemoteLED_OnOff(uint8_t action)
{
	uint32_t TimeOutLed=10000;

	
	GPIO_SetBits(GPIOE,GPIO_Pin_7);  //Chip Select

	if(action == 1)	//Led On
		SPI_I2S_SendData(SPI1,LED_ON);
	else 			//Led Off
		SPI_I2S_SendData(SPI1,LED_OFF);
	
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);	

	while(SPI_I2S_GetFlagStatus(SPI1,SPI_FLAG_TXE) == RESET);
	

	return 1;
}


void pb_task(void *pvParameters)
{
	uint8_t bNewLedOnOff=0 ,bOldLedOnOff=0,bLedOnOff=0;

	while(1)
	{
		
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)== Bit_SET)  //User Button Pressed
		{
			bNewLedOnOff = 1;
			if(bNewLedOnOff ^ bOldLedOnOff)
			{

				if(bLedOnOff)
				{
					RemoteLED_OnOff(0);
					bLedOnOff = 0;
					STM_EVAL_LEDOff(LED3);
				}
				else
				{
					RemoteLED_OnOff(1);
					bLedOnOff = 1;
					STM_EVAL_LEDOn(LED3);
				}
				bOldLedOnOff = bNewLedOnOff;
			}			
		}
		else 
			bOldLedOnOff = 0;

	}

}


#else //slave

void spi_recv_msg_task(void *pvParameters)
{
	uint8_t rcv_tmp = 0;
	uint32_t TimeOutLed = 10000;

	while(1)
	{
#ifndef SPI_IT

		while(SPI_I2S_GetFlagStatus(SPI1,SPI_FLAG_RXNE)== RESET);
	
		rcv_tmp = (uint8_t)SPI_I2S_ReceiveData(SPI1);
		
		if(rcv_tmp == LED_ON)
		{
			STM_EVAL_LEDOff(LED4);
    		STM_EVAL_LEDOff(LED3);
    		STM_EVAL_LEDOff(LED5);
    		STM_EVAL_LEDOff(LED6);

		}
		else if(rcv_tmp == LED_OFF)
		{
   			STM_EVAL_LEDOn(LED4);
    		STM_EVAL_LEDOn(LED3);
    		STM_EVAL_LEDOn(LED5);
    		STM_EVAL_LEDOn(LED6);
		}
#endif //NO define SPI Interrupt
	}
}
#endif


#ifdef SPI_IT
void SPI1_IRQHandler(void)
{
	u8 temp1;
  
	
    
#if 0
  	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET)
  	{     
        if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET)
        {
        	SPI_I2S_SendData(SPI1, temp1);
        }

	} 
#endif

#if 1
	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET)
	{
		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);

		STM_EVAL_LEDOn(LED5);
		temp1 = SPI_I2S_ReceiveData(SPI1); 
  		readData[rxCounter++] = temp1;

		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
  	}

	 
#endif

}

#endif

int main(void)
{
	uint8_t  pressed=0,new_button_state,last_button_state;

#ifdef SPI_IT
	xQueueHandle qhSPI;
#endif

  	init_SPI();

	init_LED();
	
	init_PB();

 

#ifdef MASTER
	STM_EVAL_LEDOff(LED4);
    STM_EVAL_LEDOff(LED3);
    STM_EVAL_LEDOn(LED5);
    STM_EVAL_LEDOff(LED6);
	xTaskCreate(pb_task,
	            (signed portCHAR *) "Push Button Task",
	            512 /* stack size */, NULL, tskIDLE_PRIORITY + 2, NULL);
#else

#ifdef SPI_IT

#endif

	xTaskCreate(spi_recv_msg_task,
	            (signed portCHAR *) "SPI Recv Task",
	            512 /* stack size */, NULL, tskIDLE_PRIORITY + 2, NULL);
#endif

	/* Start running the tasks. */
	vTaskStartScheduler();
	
	STM_EVAL_LEDOff(LED5);

	return 0;
}



void vApplicationTickHook()
{
}


