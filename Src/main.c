/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 16/05/2015 22:24:02
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "util.h"
#include "main.h"

#define UART_TIMEOUT 5000
#define UART_SPEED 19200
#define NUMBER_OF_ADC 8

#define SAMPLES_PER_ADC 8
#define BUFFER_SIZE (NUMBER_OF_ADC*SAMPLES_PER_ADC)

#define HID_REPORT_SIZE 10		//total size of the keyboard report
#define HID_REPORT_KBD_OFFSET	3 		//which byte the keyboard keypresses starts at	
#define ASCII_OFFSET (4 - 'a')	//convert ASCII char into HID report by adding this offset: ie 'c' would be = 'c' + ASCII_OFFSET

#define VCC 5
#define R2 1000
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
UART_HandleTypeDef huart4;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
char aTxBuffer[50];
uint16_t ADC1_DMA_buffer[BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_TIM14_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int rounded;

//behaves like (unsigned int) ((float) Rp_MAX / (float) Rp) except only works with unsigned ints.
unsigned int divide_and_round(unsigned int num, unsigned int denom)
{
	rounded = (((num << 8) / denom) & 0xFF);
	if(rounded > 128)
		rounded = 255 - rounded;
	
	unsigned int scaled = (num << 1) / denom;					//remove this divide?
	return (scaled >> 1) + (scaled & 0x1);
}

unsigned int Rc = 2694 + -100; //1045; //acutal 1016
unsigned int Vcc = 4095 * 511 / 330;		//Vcc is 5v - actually 5.11V!, but ADC only goes to 3.3v
unsigned int Rp_MAX = 80000;	//highest resistor

unsigned int get_sw_bitstring_from_raw_slow(unsigned int Vout)
{
	//calculate resistance of switches + resistors in parallel
	unsigned int Rp = (Rc * Vcc) / Vout - Rc;					//REMOVE THIS DIVIDE?
	
	//convert resistance to bitstring representing switches (needs explanation)
	unsigned int sw_bitstring = divide_and_round(Rp_MAX, Rp);
	
	//debug printing
	//snprintf(aTxBuffer, sizeof(aTxBuffer), "er: %d val: %3d Rp: %10d v: %d\n", rounded, sw_bitstring, Rp, Vout);	
	//_puts(aTxBuffer);
		
	return sw_bitstring;
}

//maybe use floating point for all these calculations even though slower since this is only done once?
float Vout_raw_from_sw_bitstring(float sw_bitstring)
{
	float Rp = Rp_MAX / sw_bitstring;
	return (Vcc * Rc) / (Rp + Rc); 				//Vcc raw
}

unsigned int Vout_expected[32];
unsigned int Vout_cutoffs[31];
void pre_generate_table()
{
	int i;
	for(i = 0; i < 32; i++)
	{
		Vout_expected[i] = (unsigned int) Vout_raw_from_sw_bitstring(i);
		snprintf(aTxBuffer, sizeof(aTxBuffer), "i: %d, v:%d\n", i, Vout_expected[i]);	
		_puts(aTxBuffer);
	}
	
	for(i = 0; i < 31; i++)
	{
		Vout_cutoffs[i] = (Vout_expected[i] + Vout_expected[i+1])/2;
	}

}

void get_sw_bitstring_from_Vout(unsigned int i)
{
	
}

uint8_t report[HID_REPORT_SIZE] = {0};

int i = 0;
void interrupt_1ms()
{
	//send HID report every 1ms 
	Send_Report(report, HID_REPORT_SIZE);
	
	//clear the report after it's sent
	int i;
	for(i = 0; i < HID_REPORT_SIZE; i++)
	{
		//clear the report
		report[i] = 0;
	}
	
	i++; 

	if(i > 1000)
		light_on();
	else
		light_off();
	
	if(i > 2000)
	{
		i = 0;
		//_puts("running!");
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART4_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */
  
  //calibrate the ADC
  HAL_ADCEx_Calibration_Start(&hadc);
  
  //NOTE: DMA configured to load a half word (16 bits) at a time from the ADC
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1_DMA_buffer, BUFFER_SIZE);

  //pre-calculate Vout values:
  pre_generate_table();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  //just before entering loop, start interrrupts
  //each bit represents a key is currently pressed/not pressed
  unsigned char keymap[] = {'a' + ASCII_OFFSET, 
							'b' + ASCII_OFFSET,
							'c' + ASCII_OFFSET,
							'd' + ASCII_OFFSET,
							'e' + ASCII_OFFSET,}; 
  
  
  unsigned int PC_keypress_bitstring[2] = {0};	//which keys the PC thinks are pressed
  HAL_TIM_Base_Start_IT(&htim14);
  
  while (1)
  {
	uint8_t report[HID_REPORT_SIZE] = {0};
	unsigned int DEV_keypress_bitstring[2] = {0};		//which keys the Device knows are pressed
	
	int adc_i = 0;
	//for(adc_i = 0; adc_i < NUMBER_OF_ADC; adc_i++)
	{
		//Calculate average voltage (should really do median
		unsigned int Vout = 0;
		int index;
		for(index = adc_i; index < BUFFER_SIZE; index += NUMBER_OF_ADC)
		{
			Vout += ADC1_DMA_buffer[index];
		}			
		Vout /= SAMPLES_PER_ADC;									//can remove this divide if samples_per_adc is a power of 2
		
		unsigned char sw_bitstring = get_sw_bitstring_from_raw_slow(Vout);
		
		//write into keypress array. for simplicity, do in blocks of 8 bits (even though only 5 bits are actually used) 
		((unsigned char *) DEV_keypress_bitstring)[adc_i] |= sw_bitstring;
		
	}
	
	//now all data has been collected from ADCs
	//find the difference between what the pc thinks they keyboard's state is and what the actual keyboard state is
	//a '1' in the 'diff' array indicates there is a difference
	/*unsigned int diff[2];
	for(i = 0; i < 2; i++)
	{
		diff[i] = DEV_keypress_bitstring[i] ^ PC_keypress_bitstring[i]; 	
	}*/
	
	//snprintf(aTxBuffer, sizeof(aTxBuffer), "dev: %d pc: %d diff:%d\n", DEV_keypress_bitstring[0], PC_keypress_bitstring[0], diff[0]);	
	//_puts(aTxBuffer); 
	
	//find the first 6 differences. after acknowledging a difference, update the 
	int i;
	for(i = 0; i < 2; i++)
	{
		int keysFound;
		for(keysFound = 0; keysFound < 6; keysFound++)
		{
			if(DEV_keypress_bitstring[i] == 0)	//no keys pressed 
				break;
				
			unsigned int single_one = (unsigned int) (DEV_keypress_bitstring[i] & -DEV_keypress_bitstring[i]);
			unsigned int bit_index = find_first_set_single_one(single_one);	
			
			report[keysFound + HID_REPORT_KBD_OFFSET] = keymap[bit_index]; //add to the report
			
			DEV_keypress_bitstring[i] = single_one ^ DEV_keypress_bitstring[i];	//remove the bit which was just found
		}
		
		/*int diff_count;
		for(diff_count = 0; diff_count < 6; diff_count++)
		{
			if(diff[i] == 0)	//no difference between Device and PC's keystate
				break;
			
			unsigned int single_one = (unsigned int) (diff[i] & -diff[i]);	//only leaves the rightmost bit
			unsigned int bit_index = find_first_set_single_one(single_one);						
			
			//translate the bit index into a character 
			report[diff_count + HID_REPORT_KBD_OFFSET] = keymap[bit_index];
			
			snprintf(aTxBuffer, sizeof(aTxBuffer), "CHANGE: %d\n", bit_index);	
			_puts(aTxBuffer);
			
			//update the PC_keypress_bitstring by xoring it with the single_one value (flip the bit at that bit position):
			//also remove it from the difference bitstring (strange have to do it twice..)
			PC_keypress_bitstring[i] = single_one ^ PC_keypress_bitstring[i];
			diff[i] = single_one ^ diff[i];
		}*/
	}

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  
    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* USART4 init function */
void MX_USART4_UART_Init(void)
{

  huart4.Instance = USART4;
  huart4.Init.BaudRate = UART_SPEED;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart4);

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{
	// Reference manual specifies prescaler is actuallly f CK_PSC / (PSC[15:0] + 1). 
	// This is so that a prescaler of 0 is actually a scaling of 1., 1 is 2 etc. so 
	// that all the values can be used
	// see STM32 example code given by ST!
	unsigned int tickFrequency = 60000;
	unsigned int interruptFrequency = 1000;
	unsigned int prescaler = SystemCoreClock / tickFrequency - 1;	
	unsigned int period = tickFrequency / interruptFrequency - 1;	//starts counting from 0, so subtract one
	
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = prescaler;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = period;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
