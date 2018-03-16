/**
  ******************************************************************************
  * @file    LCD_Paint/Src/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-February-2017
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "color.h"
#include "save.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdlib.h>
//#include "stm32f4xx_hal_adc.h"
/** @addtogroup STM32F4xx_HAL_Applications
  * @{
  */

/** @addtogroup LCD_Paint
  * @{
  */

extern LTDC_HandleTypeDef  hltdc_eval;

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
  uint16_t bfType;  /* specifies the file type */
  uint32_t bfSize;  /* specifies the size in bytes of the bitmap file */
  uint16_t bfReserved1;  /* reserved : must be 0 */
  uint16_t bfReserved2;  /* reserved : must be 0 */
  uint32_t bOffBits;  /* species the offset in bytes from the bitmapfileheader to the bitmap bits */
  uint16_t Padding;   /* padding to multiple of 32 bits */
} BitMapFileHeader_Typedef;

typedef struct
{
  uint32_t biSize;  /* specifies the number of bytes required by the struct */
  uint32_t biWidth;  /* specifies width in pixels */
  uint32_t biHeight;  /* species height in pixels */
  uint16_t biPlanes; /* specifies the number of color planes, must be 1 */
  uint16_t biBitCount; /* specifies the number of bit per pixel */
  uint32_t biCompression; /* specifies the type of compression */
  uint32_t biSizeImage;  /* size of image in bytes */
  uint32_t biXPelsPerMeter;  /* number of pixels per meter in x axis */
  uint32_t biYPelsPerMeter;  /* number of pixels per meter in y axis */
  uint32_t biClrUsed;  /* number of colors used by the bitmap */
  uint32_t biClrImportant;  /* number of colors that are important */
} BitMapFileInfoHeader_Typedef;

/* Private define ------------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOK
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOG
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define ADC_Pin GPIO_PIN_1
#define ADC_GPIO_Port GPIOB
enum Select{
	STORAGE_MODE = 100,
	BREWING_MODE
};
enum Color{
	NONE = 0,
	RED ,
	GREEN,
	BLUE
};
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;   /* File system object for SD card logical drive */
FIL MyFile;      /* File object */
char SDPath[4];  /* SD card logical drive path */
const char DEVICE_id[4] = "6045";
char buf_long[14];
char buf_short[6];
char receive[20];
char makesure[8];
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
double adc_value = 0;
int va_light[5];
int va_temp[5];
int va_weight[5];
int va = 0;
int va_w = 0;
int va_t = 0;
int value_light[7];
int value_temp = 0;
int value_weight = 0;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
int MODE = 0;
static int change_mode = 0;
int notification_count = 0;
/* BMP file information to save the drawing pad to file BMP in RGB888 format */
//static BitMapFileHeader_Typedef     bmpFileHeader;
//static BitMapFileInfoHeader_Typedef bmpFileInfoHeader;

//static uint8_t *  p_bmp_converted_pixel_data = (uint8_t *)CONVERTED_FRAME_BUFFER;

//static uint32_t Radius = 10;
TS_StateTypeDef  TS_State = {0};

/* Private function prototypes -----------------------------------------------*/
static void ready_detect(void);
static void register_press(void);
static void storage_brewing(void);
static void Enjoy_show(void);
static void storage_mode(void);
static void brewing_mode(void);
//static void GetPosition(void);
static void SystemClock_Config(void);
void SystemClock_Config_uart(void);
static void Error_Handler(void);
static void Save(void);
static void LTDC_Operation(uint32_t Enable_LTDC);

void MX_SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
void _Error_Handler(char * , int );
void	va_light_get(void);
void	va_temp_get(void);
void 	va_weight_get(void);
void Poll_for_RGB(int,int,int);
void loading_mode(void);
void success_mode(void);
void insert_mode(void);
void get_id_mode(void);
void detecting_mode(void);
void transmit_receive(char*,int,int,char);
void SD_init(void);
void LCD_init(void);
void receive_str(char*,int);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{


//  p_bmp_converted_pixel_data = (uint8_t *)CONVERTED_FRAME_BUFFER;

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */

  uint32_t ts_status  = TS_OK;

  HAL_Init();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
  MX_ADC3_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);

	LCD_init();
	
	
	
  /*##-3- Touch screen initialization ########################################*/
  BSP_TS_ResetTouchData(&TS_State);

  /* If calibration is not yet done, proceed with calibration */
  if (TouchScreen_IsCalibrationDone() == 0)
  {
    ts_status = Touchscreen_Calibration();
    if(ts_status == TS_OK)
    {
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 65, (uint8_t *)"Touchscreen calibration success.", CENTER_MODE);
    }
  } /* of if (TouchScreen_IsCalibrationDone() == 0) */



  /*##-4- Link the SD Card disk I/O driver ###################################*/

  /* Clear the LCD and display waiting message */
  BSP_LCD_Clear(LCD_COLOR_BLACK );
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW );
  BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE );
  BSP_LCD_SetFont(&Font24);
	
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-240, BSP_LCD_GetYSize()/2-120, (uint8_t *)0x08080000);
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-50, BSP_LCD_GetYSize()/2-180, (uint8_t *)0x08030000);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 27, (uint8_t*)"Please WAIT for few seconds", CENTER_MODE);
  //BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 - 8, (uint8_t*)"Now initializing......", CENTER_MODE);
	
	SD_init();
  /*##-5- START ######################################################*/
	/*SystemClock_Config_uart();
	while (1)
		{	
		while (1)
		{
			int err = HAL_BUSY;
			while(err != HAL_OK)
				err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
			HAL_Delay(1000);
			if(err == HAL_OK)
				HAL_UART_Transmit(&huart6,(uint8_t*)buf_short,6,10000);
			break;
		}
	}
	SystemClock_Config();*/
  HAL_Delay(1000);
	register_press();
	ready_detect();
	HAL_Delay(1000);
	storage_brewing();
	while(1)
	{
		va_weight_get();
		if(MODE == 0 && va_weight[0]>1){
			Enjoy_show();
			MODE = 0;	
		}
		else if(MODE==0 && va_weight[0]<1)
			storage_brewing();
		else if( MODE ==  STORAGE_MODE)
			storage_mode();
		else if( MODE ==  BREWING_MODE)
			brewing_mode();
	}
}

/**
  * @brief  Configures and gets Touch screen position.
  * @param  None
  * @retval None
  */

/**
  * @brief  Draws the menu.
  * @param  None
  * @retval None
  */


/**
  * @brief  Saves the picture in microSD.
  * @param  None
  * @retval None
  */
static void Save(void)
{
	static uint32_t counter = 0;
  uint8_t str[30];

  /* Check if the SD card is plugged in the slot */
  if(BSP_SD_IsDetected() != SD_PRESENT)
  {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"No SD card detected !!", RIGHT_MODE);
    Error_Handler();
  }
  else
  {
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"Saving Txt file to SD card", RIGHT_MODE);

    /* Format the string */
    sprintf((char *)str, "fdata_%lu.txt", counter);
		//sprintf((char *)str, "image_%lu.txt", counter);
    /* Disable the LTDC to avoid charging the bandwidth for nothing while the BMP file is */
    /* written to SD card */
    //LTDC_Operation(0);
    if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
      /* 'image.bmp' file Open for write Error */
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t*)"   File Creation Error !!", RIGHT_MODE);
      Error_Handler();
    }
		else
    {
			char fbuf[2009];
			sprintf(fbuf, "%d,%d,%d,%d,%d,%d,%d,%d,%d",value_light[0],value_light[1],value_light[2],value_light[3],value_light[4],value_light[5],value_light[6],value_temp,value_weight);
			uint32_t writtenByte = 0;
			f_write(&MyFile, fbuf,sizeof(uint16_t)*1000,(void *)&writtenByte);
			f_close(&MyFile);
			counter++;
			HAL_Delay(1000);
		}
  }
}

/**
  * @brief  Disable/Enable LTDC activity as in DSI Video mode the LTDC is all the time
  *         in active window pumping data with its DMA, it creates a huge bandwidth consumption
  *         that can penalize other IPs, here the save to SD functionality.
  * @param  Enable_LTDC : 0 to disable LTDC, 1 to re-enable LTDC
  * @retval None
  */
static void LTDC_Operation(uint32_t Enable_LTDC)
{
  /* Desactivate the DSI wrapper */
  DSI->WCR &= ~(DSI_WCR_DSIEN);

  if(Enable_LTDC == 1)
  {
    __HAL_LTDC_ENABLE(&hltdc_eval); /* Switch back On bit LTDCEN */
  }
  else if (Enable_LTDC == 0)
  {
    __HAL_LTDC_DISABLE(&hltdc_eval); /* Switch Off bit LTDCEN */
  }

  /* Reactivate the DSI Wrapper */
 DSI->WCR |= DSI_WCR_DSIEN;
}

/**
  * @brief  Prepares the picture to be saved in microSD.
  * @param  None
  * @retval None
  */

/**
  * @brief  Updates the selected color
  * @param  None
  * @retval None
  */

/**
  * @brief  Updates the selected size
  * @param  size: Size to be updated
  * @retval None
  */


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}
static void _Error_Handler(char* a,int b)
{
  /* user define */
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4

  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if defined(USE_STM32469I_DISCO_REVA)
  RCC_OscInitStruct.PLL.PLLM = 25;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif /* USE_STM32469I_DISCO_REVA */
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
void SystemClock_Config_uart(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

/* GPIO init function */
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	
  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void ready_detect(void)
{
  /* Clear the LCD */
	
  BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK  );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(50, 100, (uint8_t*)"Your Device is Ready", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	BSP_LCD_DrawLine(50,150,BSP_LCD_GetXSize()-50,150);
	BSP_LCD_DisplayStringAt(50, 180, (uint8_t*)"You can insert your wine now!", LEFT_MODE);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
	int i;
	for(i = 0;i<5;i++)
		BSP_LCD_DrawBitmap(i*150+75, BSP_LCD_GetYSize()/2+5, (uint8_t *)0x08060000);
	while(1)
	{
		va_weight_get();
		if(va_weight[0]<1){
				loading_mode();
				HAL_Delay(1000);
				LTDC_Operation(0);
				char str[6];
				sprintf(str, "i,%s",DEVICE_id );
				SystemClock_Config_uart();
				while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,6,10000);
					while(err != HAL_OK)
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					if(err == HAL_OK){
						//sprintf((char *)str, "y,%s",DEVICE_id );
						//if(strcmp(buf_short,str))
							//while(1){;}
						{
							SystemClock_Config();
							LTDC_Operation(1);
							HAL_Delay(1000);
							insert_mode();
							break;
						}
					}
				}
			break;
		}
	}
}
static void register_press(void){
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK  );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(50, 100, (uint8_t*)"Your Device is Ready", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	BSP_LCD_DrawLine(50,150,BSP_LCD_GetXSize()-50,150);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t*)"Get Device ID", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(BSP_LCD_GetXSize()/2-200,BSP_LCD_GetYSize()/2-50,BSP_LCD_GetXSize()/2,100);
	BSP_LCD_DrawRect(BSP_LCD_GetXSize()/2-195,BSP_LCD_GetYSize()/2-45,BSP_LCD_GetXSize()/2-10,90);
	while(1)
	{
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected){
			int x = -1,y = -1;
			x = TouchScreen_Get_Calibrated_X(TS_State.touchX[0]);
			y = TouchScreen_Get_Calibrated_Y(TS_State.touchY[0]);
			if((x > BSP_LCD_GetXSize()/2-195) & (y > (BSP_LCD_GetYSize()/2-45)) & (x < (BSP_LCD_GetXSize()/2-195)+(BSP_LCD_GetXSize()/2-10)) & (y < (BSP_LCD_GetYSize()/2-45)+90)){
				get_id_mode();
				HAL_Delay(1000);
				LTDC_Operation(0);
				char str[6];
				sprintf(str, "r,%s",DEVICE_id );
				SystemClock_Config_uart();
				while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,6,10000);
					while(err != HAL_OK)
					{
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					}
					if(err == HAL_OK){
						/*sprintf((char *)str, "y,%s",DEVICE_id );
						if(strcmp(buf_short,str))
							while(1){;}*/
						{
							SystemClock_Config();
							LTDC_Operation(1);
							success_mode();
							break;
						}
					}
					else{while(1){;}
					}
				}
				break;
			}
		}
	}
}
static void storage_brewing(void){
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize());
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillRect(BSP_LCD_GetXSize()/2, 0, BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize());
	BSP_LCD_DisplayStringAt(150, BSP_LCD_GetYSize()/2, (uint8_t*)"Storage", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0,0, (uint8_t*)"Please choose a mode:", LEFT_MODE);
	
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2+150, BSP_LCD_GetYSize()/2, (uint8_t*)"Brewing", LEFT_MODE);
	
	while(1)
	{
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected){
			int x = -1,y = -1;
		x = TouchScreen_Get_Calibrated_X(TS_State.touchX[0]);
    y = TouchScreen_Get_Calibrated_Y(TS_State.touchY[0]);
		if((x > 0) & (y > (0)) & (x < BSP_LCD_GetXSize()/2) & (y < BSP_LCD_GetYSize())){
			MODE = STORAGE_MODE;
			//****
			char str[8];
			sprintf(str,"m,%s,%d",DEVICE_id,1);
			BSP_LCD_Clear(LCD_COLOR_BLACK );
			HAL_Delay(1000);
			LTDC_Operation(0);
			HAL_Delay(1000);
			SystemClock_Config_uart();
			while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,8,10000);
					while(err != HAL_OK)
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					if(err == HAL_OK){
						//sprintf((char *)str, "y,%s",DEVICE_id );
						//if(strcmp(buf_short,str))
							//while(1){;}
						{
							SystemClock_Config();
							LTDC_Operation(1);
							HAL_Delay(1000);
							break;
						}
					}
				}
			//****
			break;
		}
		if((x > BSP_LCD_GetXSize()/2) & (y > (0)) & (x < BSP_LCD_GetXSize()) & (y < BSP_LCD_GetYSize())){
			MODE = BREWING_MODE;
			//****
			char str[8];
			sprintf(str,"m,%s,%d",DEVICE_id,2);
			
			BSP_LCD_Clear(LCD_COLOR_BLACK );
			HAL_Delay(1000);
			LTDC_Operation(0);
			HAL_Delay(1000);
			SystemClock_Config_uart();
			while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,8,10000);
					while(err != HAL_OK)
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					if(err == HAL_OK){
						//sprintf((char *)str, "y,%s",DEVICE_id );
						//if(strcmp(buf_short,str))
							//while(1){;}
						{
							SystemClock_Config();
							LTDC_Operation(1);
							HAL_Delay(1000);
							break;
						}
					}
				}
			//****
			break;
		}
		}
	}
}
static void Enjoy_show(void){
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0,BSP_LCD_GetYSize()/2,(uint8_t*)"Enjoy the Wine", CENTER_MODE);
}
static void storage_mode(void){
	char str[50];
	char category[30];
	char origin[30];
	char year[30];
	char temperature[30];
	char light[30];
	float score;
	while(1)
	{
		va_weight_get();
		if(va_weight[0]>1){
			BSP_LCD_Clear(LCD_COLOR_BLACK);
			char str[6];
			sprintf(str,"e,%s",DEVICE_id);
			HAL_Delay(1000);
			LTDC_Operation(0);
			HAL_Delay(1000);
			SystemClock_Config_uart();
			while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,6,10000);
					while(err != HAL_OK)
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					if(err == HAL_OK){
						//sprintf((char *)str, "y,%s",DEVICE_id );
						//if(strcmp(buf_short,str))
							//while(1){;}
						{
							SystemClock_Config();
							LTDC_Operation(1);
							HAL_Delay(1000);
							break;
						}
					}
				}
			Enjoy_show();
			MODE = 0;
			return;
		}
		
		//send data to serveronfig();*/
		//get the data from the server



		//string above should be replaced by the data received
		//start show the suggestion
		LCD_init();
		BSP_LCD_Clear(LCD_COLOR_BLACK );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
		BSP_LCD_DrawLine(150,100,BSP_LCD_GetXSize()-150,BSP_LCD_GetYSize()-75);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+50, (uint8_t*)category, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+70, (uint8_t*)origin, LEFT_MODE);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+90, (uint8_t*)year, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 70, (uint8_t*)"Storage Condition:", LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 90, (uint8_t*)temperature, LEFT_MODE);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2,110, (uint8_t*)light, LEFT_MODE);
		sprintf((char *)str, "Score:%.1f",score );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2+130,BSP_LCD_GetYSize()/2+50, (uint8_t*)str, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		int i = 0;
		for (i = 0;i<10;i++)
			BSP_LCD_DrawVLine(BSP_LCD_GetXSize()/2+295+i,BSP_LCD_GetYSize()/2-30,10*score);
		//here should be replaced by the animation ,sensing right now
		//The clock setting
		char send[14];
		char co;
		HAL_Delay(1000);
		LTDC_Operation(0);
		HAL_Delay(1000);
		SystemClock_Config_uart();
		
		
		Poll_for_RGB(NONE,NONE,NONE);//WHITE
		HAL_Delay(1000);
		va_light_get();
		value_light[0] = va;
		co = 'w';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,NONE,NONE);//RED
		HAL_Delay(1000);
		va_light_get();
		value_light[1] = va;
		co = 'r';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,GREEN,NONE);//GREEN
		HAL_Delay(1000);
		va_light_get();
		value_light[2] = va;
		co = 'g';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,NONE,BLUE);//BLUE
		HAL_Delay(1000);
		va_light_get();
		value_light[3] = va;
		co = 'b';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,GREEN,NONE);//YELLOW
		HAL_Delay(1000);
		va_light_get();
		value_light[4] = va;
		co = 'y';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,NONE,BLUE);//PURPLE
		HAL_Delay(1000);
		va_light_get();
		value_light[5] = va;
		co = 'p';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,GREEN,BLUE);//VIOLATE
		HAL_Delay(1000);
		va_light_get();
		value_light[6] = va;
		co = 'v';
		transmit_receive(send,14,va,co);
		
		va_temp_get();
		value_temp = va_t;
		HAL_Delay(1000);
		co = 't';
		transmit_receive(send,14,va_t,co);
		
		va_weight_get();
		value_temp = va_w;
		HAL_Delay(1000);
		co = 'm';
		transmit_receive(send,14,va_w,co);
		
		notification_count = 1;
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(category,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(origin,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(year,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(temperature,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		score = atof(receive);
		strcpy(light,"In the shadow");
		
		HAL_Delay(1000);
		SystemClock_Config();
		HAL_Delay(1000);
		BSP_LCD_SetFont(&Font24);
		LTDC_Operation(1);
		HAL_Delay(1000);
		LCD_init();
		BSP_LCD_Clear(LCD_COLOR_BLACK );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
		BSP_LCD_DrawLine(150,100,BSP_LCD_GetXSize()-150,BSP_LCD_GetYSize()-75);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+50, (uint8_t*)category, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+70, (uint8_t*)origin, LEFT_MODE);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+90, (uint8_t*)year, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 70, (uint8_t*)"Storage Condition:", LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 90, (uint8_t*)temperature, LEFT_MODE);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2,110, (uint8_t*)light, LEFT_MODE);
		sprintf((char *)str, "Score:%.1f",score );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2+130,BSP_LCD_GetYSize()/2+50, (uint8_t*)str, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		i = 0;
		for (i = 0;i<10;i++)
			BSP_LCD_DrawVLine(BSP_LCD_GetXSize()/2+295+i,BSP_LCD_GetYSize()/2-30,10*score);		

		HAL_Delay(10000);
	}

		//add the save and leave icon, change to 60 sec
}
static void brewing_mode(void){
	char str[50];
	char category[30];
	char origin[30];
	char year[30];
	char temperature[30];
	char light[30];
	float score;
	while(1)
	{
		va_weight_get();
		if(va_weight[0]>1){
			BSP_LCD_Clear(LCD_COLOR_BLACK);
			char str[6];
			sprintf(str,"e,%s",DEVICE_id);
			HAL_Delay(1000);
			LTDC_Operation(0);
			HAL_Delay(1000);
			SystemClock_Config_uart();
			while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,6,10000);
					while(err != HAL_OK)
						err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					if(err == HAL_OK){
						//sprintf((char *)str, "y,%s",DEVICE_id );
						//if(strcmp(buf_short,str))
							//while(1){;}
						{
							SystemClock_Config();
							LTDC_Operation(1);
							HAL_Delay(1000);
							break;
						}
					}
				}
			Enjoy_show();
			MODE = 0;
			return;
		}
		
		//send data to server
		//get the data from the server
		//string above should be replaced by the data received
		//start show the suggestion
		LCD_init();
		BSP_LCD_Clear(LCD_COLOR_BLACK );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
		BSP_LCD_DrawLine(150,100,BSP_LCD_GetXSize()-150,BSP_LCD_GetYSize()-75);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+50, (uint8_t*)category, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+70, (uint8_t*)origin, LEFT_MODE);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+90, (uint8_t*)year, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 70, (uint8_t*)"Storage Condition:", LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 90, (uint8_t*)temperature, LEFT_MODE);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2,110, (uint8_t*)light, LEFT_MODE);
		sprintf((char *)str, "Score:%.1f",score );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2+130,BSP_LCD_GetYSize()/2+50, (uint8_t*)str, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		int i = 0;
		for (i = 0;i<10;i++)
			BSP_LCD_DrawVLine(BSP_LCD_GetXSize()/2+295+i,BSP_LCD_GetYSize()/2-30,10*score);
			//add the save and leave icon, change to 60 sec
		
		
		
		char send[14];
		char co;
		//here should be replaced by the animation ,sensing right now
		//clock setting
		HAL_Delay(1000);
		LTDC_Operation(0);
		HAL_Delay(1000);
		SystemClock_Config_uart();
		
		Poll_for_RGB(NONE,NONE,NONE);//WHITE
		HAL_Delay(1000);
		va_light_get();
		value_light[0] = va;
		co = 'w';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,NONE,NONE);//RED
		HAL_Delay(1000);
		va_light_get();
		value_light[1] = va;
		co = 'r';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,GREEN,NONE);//GREEN
		HAL_Delay(1000);
		va_light_get();
		value_light[2] = va;
		co = 'g';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,NONE,BLUE);//BLUE
		HAL_Delay(1000);
		va_light_get();
		value_light[3] = va;
		co = 'b';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,GREEN,NONE);//YELLOW
		HAL_Delay(1000);
		va_light_get();
		value_light[4] = va;
		co = 'y';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(RED,NONE,BLUE);//PURPLE
		HAL_Delay(1000);
		va_light_get();
		value_light[5] = va;
		co = 'p';
		transmit_receive(send,14,va,co);
		
		Poll_for_RGB(NONE,GREEN,BLUE);//VIOLATE
		HAL_Delay(1000);
		va_light_get();
		value_light[6] = va;
		co = 'v';
		transmit_receive(send,14,va,co);
		
		va_temp_get();
		value_temp = va_t;
		HAL_Delay(1000);
		co = 't';
		transmit_receive(send,14,va_t,co);
		
		va_weight_get();
		value_temp = va_w;
		HAL_Delay(1000);
		co = 'm';
		transmit_receive(send,14,va_w,co);
		
		notification_count = 1;
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(category,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(origin,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(year,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		strcpy(temperature,receive);
		HAL_Delay(1000);
		receive_str(makesure,notification_count);
		notification_count++;
		score = atof(receive);
		strcpy(light,"In the shadow");
		
		SystemClock_Config();
		HAL_Delay(1000);
		BSP_LCD_SetFont(&Font24);
		LTDC_Operation(1);
		HAL_Delay(1000);
	
		LCD_init();
		BSP_LCD_Clear(LCD_COLOR_BLACK );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
		BSP_LCD_DrawLine(150,100,BSP_LCD_GetXSize()-150,BSP_LCD_GetYSize()-75);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+50, (uint8_t*)category, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+70, (uint8_t*)origin, LEFT_MODE);
		BSP_LCD_DisplayStringAt(40, BSP_LCD_GetYSize()/2+90, (uint8_t*)year, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 70, (uint8_t*)"Storage Condition:", LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2, 90, (uint8_t*)temperature, LEFT_MODE);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2,110, (uint8_t*)light, LEFT_MODE);
		sprintf((char *)str, "Score:%.1f",score );
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAt(BSP_LCD_GetXSize()/2+130,BSP_LCD_GetYSize()/2+50, (uint8_t*)str, LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		i = 0;
		for (i = 0;i<10;i++)
			BSP_LCD_DrawVLine(BSP_LCD_GetXSize()/2+295+i,BSP_LCD_GetYSize()/2-30,10*score);
		
		HAL_Delay(10000);
	}

}
void va_light_get(void){
		//uint8_t str[30];
		int i;
		va = 0;
		HAL_ADC_Start_IT(&hadc1);
		while(HAL_ADC_PollForConversion(&hadc1,1000)!=HAL_OK);
		adc_value = HAL_ADC_GetValue(&hadc1);
		adc_value = (adc_value/4096)*3.3;
		for(i=0;i<5;i++)
				va_light[i]=0;
		va_light[0] = ((int)adc_value%10)*10000;
		va_light[1] = ((int)(adc_value*10)%10)*1000;
		va_light[2] = ((int)(adc_value*100)%10)*100;
		va_light[3] = ((int)(adc_value*1000)%10)*10;
		va_light[4] = (int)(adc_value*10000)%10;
		for(i=0;i<5;i++)
			va += va_light[i];
			//va = 32991 - va;
		//sprintf((char *)str, "adc value:%d   ",va );
		//BSP_LCD_DisplayStringAt(150, 100, (uint8_t *)str, LEFT_MODE);
/* error handler init function */
}
void va_temp_get(void)
{
		//uint8_t str[30];
		int i=0;
		HAL_ADC_Start_IT(&hadc2);
		while(HAL_ADC_PollForConversion(&hadc2,1000)!=HAL_OK);
		adc_value = HAL_ADC_GetValue(&hadc2);
		adc_value = (adc_value/4096)*3.3;
		va_t = 0;
		va_temp[0] = ((int)adc_value%10)*10000;
		va_temp[1] = ((int)(adc_value*10)%10)*1000;
		va_temp[2] = ((int)(adc_value*100)%10)*100;
		va_temp[3] = ((int)(adc_value*1000)%10)*10;
		va_temp[4] = (int)(adc_value*10000)%10;
		for(i=0;i<5;i++)
				va_t += va_temp[i];
		//sprintf((char *)str, "adc value:%d.%d%d%d%d",va_temp[0],va_temp[1],va_temp[2],va_temp[3],va_temp[4] );
		//BSP_LCD_DisplayStringAt(150, 200, (uint8_t *)str, LEFT_MODE);
}
void va_weight_get(void){
		//uint8_t str[30];
		int i=0;
		HAL_ADC_Start_IT(&hadc3);
		while(HAL_ADC_PollForConversion(&hadc3,1000)!=HAL_OK);
		adc_value = HAL_ADC_GetValue(&hadc3);
		adc_value = (adc_value/4096)*3.3;
		va_w = 0;
		va_weight[0] = ((int)adc_value%10)*10000;
		va_weight[1] = ((int)(adc_value*10)%10)*1000;
		va_weight[2] = ((int)(adc_value*100)%10)*100;
		va_weight[3] = ((int)(adc_value*1000)%10)*10;
		va_weight[4] = (int)(adc_value*10000)%10;
			for(i=0;i<5;i++)
				va_w += va_weight[i];
		//sprintf((char *)str, "adc value:%d.%d%d%d%d",va_weight[0],va_weight[1],va_weight[2],va_weight[3],va_weight[4] );
		//BSP_LCD_DisplayStringAt(150, 300, (uint8_t *)str, LEFT_MODE);
		
/* error handler init function */
}
void Poll_for_RGB(int R,int G,int B)//D11 D12 D13
{
	if(R!=0)
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	if(G!=0)
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	if(B!=0)
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
}
void loading_mode(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-50, BSP_LCD_GetYSize()/2-50, (uint8_t *)0x08010000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2+80, (uint8_t *)"Now loading", CENTER_MODE);
}
void detecting_mode(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-50, BSP_LCD_GetYSize()/2-50, (uint8_t *)0x08010000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2+80, (uint8_t *)"Now detecting", CENTER_MODE);
}
void success_mode(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-50, BSP_LCD_GetYSize()/2-50, (uint8_t *)0x08010000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2+80, (uint8_t *)"Success", CENTER_MODE);
}
void insert_mode(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawBitmap(BSP_LCD_GetXSize()/2-50, BSP_LCD_GetYSize()/2-50, (uint8_t *)0x08010000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2+80, (uint8_t *)"The wine ", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2+100, (uint8_t *)"been inserted!", CENTER_MODE);
}
void get_id_mode(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK );
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK  );
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(50, 100, (uint8_t*)"Your Device is Ready", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	BSP_LCD_DrawLine(50,150,BSP_LCD_GetXSize()-50,150);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t *)0x08050000);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, (uint8_t*)"Youe Device ID is #6045", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
}
#endif
void transmit_receive(char* str,int len,int data,char s)
{
				char str_short[6];
				if(data<10)
					sprintf(str, "u,%s,%c,0000%d",DEVICE_id,s,data);
				else if(data<100)
					sprintf(str, "u,%s,%c,000%d",DEVICE_id,s,data);
				else if(data<1000)
					sprintf(str, "u,%s,%c,00%d",DEVICE_id,s,data);
				else if(data<10000)
					sprintf(str, "u,%s,%c,0%d",DEVICE_id,s,data);
				else
					sprintf(str, "u,%s,%c,%d",DEVICE_id,s,data);
				
				while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,len,10000);
					while(err != HAL_OK){
							err = HAL_UART_Receive(&huart6,(uint8_t*)buf_short,6,10000);
					}
					if(err == HAL_OK){
						//sprintf(str_short, "y,%s",DEVICE_id );
						/*if(strcmp(buf_short,str_short))
							while(1){;}*/
						//else{
							break;
						//}
					}
					//else{while(1){;}
					
				}
}
void receive_str(char* str,int noti)
{
				sprintf(str,"n,%s,%d",DEVICE_id,noti);
				while (1)
				{
					int err = HAL_BUSY;
					HAL_UART_Transmit(&huart6,(uint8_t*)str,8,10000);
					while(err != HAL_OK){
							err = HAL_UART_Receive(&huart6,(uint8_t*)receive,20,200000);
					}
					if(err == HAL_OK){
						break;
					}
				
				}
}
void SD_init(void)
{
	  if(FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
  {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 11, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
  }

/*##-4- Register the file system object to the FatFs module ################*/
  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
    {
    /* FatFs Initialization Error */
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 11, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
   }
  /* Create a FAT file system (format) on the logical drive */
  if(f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK)
  {
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 11, (uint8_t*)"FAT FS Error !!", CENTER_MODE);
    Error_Handler();
  }
}
void LCD_init(void)
{
	uint8_t  sdram_status = SDRAM_OK;
  uint8_t  lcd_status = LCD_OK;
	/*##-1- Initialize the SDRAM */
  sdram_status = BSP_SDRAM_Init();
  if(sdram_status != SDRAM_OK)
  {
    Error_Handler();
  }

  /*##-2- LCD Initialization #################################################*/
  /* Initialize the LCD DSI */
  lcd_status = BSP_LCD_Init() ;
  if(lcd_status != LCD_OK)
  {
    Error_Handler();
  }

  lcd_status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE);
  if(lcd_status != LCD_OK)
  {
    Error_Handler();
  }
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);

  /* Clear the LCD Background layer */
  BSP_LCD_Clear(LCD_COLOR_ORANGE);
}

/**
  * @}
  */
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
