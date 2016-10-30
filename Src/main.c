/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/////////////////////////////////////ds1302///////////////////////////////////////////////////////////////
#define ds1302_sec_add			0x80
#define ds1302_min_add			0x82
#define ds1302_hr_add			0x84
#define ds1302_date_add			0x86
#define ds1302_month_add		0x88
#define ds1302_day_add			0x8a
#define ds1302_year_add			0x8c
#define ds1302_control_add		0x8e
#define ds1302_charger_add		0x90
#define ds1302_clkburst_add		0xbe

#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)

#ifndef UBYTE
#define UBYTE unsigned char
#endif

#define DS1302_GPIO	GPIOB			//порт на котором висит DS1302
#define DS1302_SCLK	GPIO_PIN_0		//пины для общения
#define DS1302_SDA	GPIO_PIN_2
#define DS1302_RST	GPIO_PIN_1
///////////////////////////////////end_ds1302////////////////////////////////////////////////
//---------------------------LCD-----------------------------------------------
#define d4_set  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, ENABLE)
#define d5_set  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, ENABLE)
#define d6_set  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ENABLE)
#define d7_set  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, ENABLE)

#define d4_reset  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, DISABLE)
#define d5_reset  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, DISABLE)
#define d6_reset  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, DISABLE)
#define d7_reset  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, DISABLE)

#define e1  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, ENABLE)
#define e0  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, DISABLE)

#define rs1  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, ENABLE)
#define rs0  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DISABLE)

//--------------R/W-на землю---------------------------------------------------
//-----------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
////////////////////////////////ds1302//////////////////////////////////////////////
void DS1302_Init(void);
static void DS1302_WriteByte(UBYTE addr, UBYTE d);
static UBYTE DS1302_ReadByte(UBYTE addr);
void DS1302_WriteTime(UBYTE *buf);
void DS1302_ReadTime(UBYTE *buf);
///////////////////////////////end_ds1302////////////////////////////////////////////
//-----------------------------LCD---------------------------------------------
void LCD_ini(void);
void delay(void);
void LCD_WriteData(uint8_t dt);
void LCD_Command(uint8_t dt);
void LCD_Data(uint8_t dt);
void LCD_SendChar(char ch);
void LCD_Clear(void);
void LCD_String(char* st);
void LCD_SetPos(uint8_t x, uint8_t y);

//-----------------------------------------------------------------------------
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char str[20];
	unsigned char timedat[8] = {0, 2016, 9, 29, 12, 17, 1, 4}; // устанавлемое время по умолчанию пусто, год, месяц, дата, час, минута, секунда, день.
	unsigned char timebuf[16] = {0, };
	unsigned char databuf[16] = {0, };
	//char str[]="Stm32O_O";

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  	  	DS1302_Init();
  	  	LCD_ini();
  	  	//sprintf(str,"Stm32-Clock");
  	    //LCD_String(str);
  	  	//DS1302_WriteTime(timedat);  //запись времени

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //void LCD_SetPos(1,1)
	   //sprintf(str,"123456789987654321");
	   //LCD_String(str);
//	  LCD_String(timebuf);
//	  LCD_SetPos(1,2);
//	  LCD_String(timebuf);
	  DS1302_ReadTime(timedat);
	  //sprintf((char*)timebuf, "%02d-%02d %02d.%02d.%02d", timedat[2], timedat[3], timedat[4], timedat[5], timedat[6]); //на нормальный дисплей
	  sprintf((char*)timebuf, "%02d-%02d %02d", timedat[2], timedat[3], timedat[4]);// на дисплей 1601-А
	  LCD_SetPos(0,0);
	  LCD_String(timebuf);
	  sprintf((char*)timebuf, ":%02d:%02d", timedat[5], timedat[6]);// на дисплей 1601-А
	  LCD_SetPos(0,1);
	  LCD_String(timebuf);
	  //HAL_Delay(200);
	  //LCD_Clear();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//--------------------------------------LCD-------------------------------------
void LCD_ini(void){
	HAL_Delay(40);
	rs0;
	uint8_t i=0;
	for(i=0;i<3;i++)
	 {
		LCD_WriteData(3);
		e1;
		delay();
		e0;
		HAL_Delay(1);
	 }
	LCD_Command(0x28);//режим 4 бита, 2 линии(для большого это 4)
	HAL_Delay(1);
	LCD_Command(0x28);//еще раз для надежности
	HAL_Delay(1);
	LCD_Command(0x0C);//включить дисплей(D=1),и все курсоры (0x0F),отключить курсор(0x0C)
	HAL_Delay(1);
	LCD_Command(0x01);//убрать мусор
	HAL_Delay(2);
	LCD_Command(0x06);//пишем влево
	HAL_Delay(1);
	LCD_Command(0x02);//курсор в нулевое положение
	HAL_Delay(2);

}
//------------------------------------------------------------------------------
void LCD_WriteData(uint8_t dt){
	if(((dt>>3) & 0x01)==1){d7_set;}else{d7_reset;}
	if(((dt>>2) & 0x01)==1){d6_set;}else{d6_reset;}
	if(((dt>>1) & 0x01)==1){d5_set;}else{d5_reset;}
	if((dt & 0x01)==1){d4_set;}else{d4_reset;}

}
//-----------------------------------------------------------------------------
void delay(void){
	uint16_t i;
	for(i=0;i<1000;i++){};
}
//-----------------------------------------------------------------------------
void LCD_Command(uint8_t dt){
	rs0;
	LCD_WriteData(dt>>4);
	e1;
	delay();
	e0;
	LCD_WriteData(dt);
	e1;
	delay();
	e0;
}
//------------------------------------------------------------------------------
void LCD_Data(uint8_t dt){
	rs1;
	LCD_WriteData(dt>>4);
	e1;
	delay();
	e0;
	LCD_WriteData(dt);
	e1;
	delay();
	e0;
}
//------------------------------------------------------------------------------
void LCD_Clear(void){

	LCD_Command(0x01);//убрать мусор
	HAL_Delay(2);
}
//------------------------------------------------------------------------------
void LCD_SendChar(char ch){

	LCD_Data((uint8_t)ch);
	delay();
}
//------------------------------------------------------------------------------
void LCD_String(char* st){

	uint8_t i=0;
	while(st[i]!=0)
	{
		LCD_Data(st[i]);
		HAL_Delay(1);
		i++;

	}
}
//------------------------------------------------------------------------------
void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
	case 0:
		LCD_Command(x|0x80);
		HAL_Delay(1);
	break;
	case 1:
		LCD_Command((0x40 + x)|0x80);
		HAL_Delay(1);
	break;
	case 2:
		LCD_Command((0x14 + x)|0x80);
		HAL_Delay(1);
	break;
	case 3:
		LCD_Command((0x54 + x)|0x80);
		HAL_Delay(1);
	break;
	}
}
//////////////////////////////////////////////////////////////////////
//		((char)((i/100)%10)) 	12(3)		вывод цифер из числа
//		((char)((i/10)%10)) 	1(2)3
//		((char)(i%10)) 			(1)23
//////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------
/////////////////////////////////////////////////////--DS1302--//////////////////////////////////////////////////////////////////

//#define ds1302_sec_add			0x80
//#define ds1302_min_add			0x82
//#define ds1302_hr_add			0x84
//#define ds1302_date_add			0x86
//#define ds1302_month_add		0x88
//#define ds1302_day_add			0x8a
//#define ds1302_year_add			0x8c
//#define ds1302_control_add		0x8e
//#define ds1302_charger_add		0x90
//#define ds1302_clkburst_add		0xbe
//
//#define HEX2BCD(v)	((v) % 10 + (v) / 10 * 16)
//#define BCD2HEX(v)	((v) % 16 + (v) / 16 * 10)
//
//#ifndef UBYTE
//#define UBYTE unsigned char
//#endif
//
//#define DS1302_GPIO	GPIOB			//порт на котором висит DS1302
//#define DS1302_SCLK	GPIO_PIN_0		//пины для общения
//#define DS1302_SDA	GPIO_PIN_2
//#define DS1302_RST	GPIO_PIN_1

static void DS1302_WriteByte(UBYTE addr, UBYTE d)
{
	UBYTE i;


	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, ENABLE);

	addr = addr & 0xFE;
	for (i = 0; i < 8; i ++)
	{

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (addr & 1) ? ENABLE : DISABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, ENABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, DISABLE);
		addr >>= 1;
	}

	for (i = 0; i < 8; i ++)
	{

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (d & 1) ? ENABLE : DISABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, ENABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, DISABLE);
		d >>= 1;
	}


	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, DISABLE);
}

static UBYTE DS1302_ReadByte(UBYTE addr)
{
	UBYTE i;
	UBYTE temp = 0;


	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, ENABLE);
	addr = addr | 0x01;

	for (i = 0; i < 8; i ++)
	{


		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SDA, (addr & 1) ? ENABLE : DISABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, ENABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, DISABLE);
		addr >>= 1;

	}


	for (i = 0; i < 8; i ++)
	{

		temp >>= 1;

		if(HAL_GPIO_ReadPin(DS1302_GPIO, DS1302_SDA))
			temp |= 0x80;

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, ENABLE);

		HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, DISABLE);
	}



	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, DISABLE);


	return temp;
}

void DS1302_WriteTime(UBYTE *buf)
{
	DS1302_WriteByte(ds1302_control_add,0x00);			
	DS1302_WriteByte(ds1302_sec_add,0x80);				
	//Ds1302_Write_Byte(ds1302_charger_add,0xa9);			
	DS1302_WriteByte(ds1302_year_add,HEX2BCD(buf[1]));		
	DS1302_WriteByte(ds1302_month_add,HEX2BCD(buf[2]));	
	DS1302_WriteByte(ds1302_date_add,HEX2BCD(buf[3]));		
	DS1302_WriteByte(ds1302_hr_add,HEX2BCD(buf[4]));		
	DS1302_WriteByte(ds1302_min_add,HEX2BCD(buf[5]));		
	DS1302_WriteByte(ds1302_sec_add,HEX2BCD(buf[6]));		
	DS1302_WriteByte(ds1302_day_add,HEX2BCD(buf[7]));		
	DS1302_WriteByte(ds1302_control_add,0x80);			
}

void DS1302_ReadTime(UBYTE *buf)
{
   	UBYTE tmp;

	tmp = DS1302_ReadByte(ds1302_year_add);
	buf[1] = BCD2HEX(tmp);	 

	tmp = DS1302_ReadByte(ds1302_month_add);
	buf[2] = BCD2HEX(tmp);	

	tmp = DS1302_ReadByte(ds1302_date_add);
	buf[3] = BCD2HEX(tmp);		

	tmp = DS1302_ReadByte(ds1302_hr_add);
	buf[4] = BCD2HEX(tmp);		

	tmp = DS1302_ReadByte(ds1302_min_add);
	buf[5] = BCD2HEX(tmp);		

	tmp = DS1302_ReadByte((ds1302_sec_add))&0x7F;
	buf[6] = BCD2HEX(tmp);	

	tmp = DS1302_ReadByte(ds1302_day_add);
	buf[7] = BCD2HEX(tmp);		
}

void DS1302_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	 __HAL_RCC_GPIOB_CLK_ENABLE();

	 GPIO_InitStruct.Pin = DS1302_SCLK | DS1302_SDA | DS1302_RST;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(DS1302_GPIO, &GPIO_InitStruct);


	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_RST, DISABLE);
		
	HAL_GPIO_WritePin(DS1302_GPIO, DS1302_SCLK, DISABLE);
    DS1302_WriteByte(ds1302_sec_add,0x00);
}

////////////////////////////////////////////////////////end--DS1302--///////////////////////////////////////////////////////////////
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
