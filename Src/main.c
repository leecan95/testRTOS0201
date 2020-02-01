/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include"string.h"
#include "stm32f4xx_hal.h"
#include "tft_spi.h"
#include "myimage.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId Task1Handle;
osThreadId Task2Handle;

/* USER CODE BEGIN PV */
extern uint16_t BACK_COLOR, POINT_COLOR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartTask1(void const * argument);
void StartTask02(void const * argument);
void IAQform(void);
void FrameUART(void);
void Convertvalue(void);
void CheckUART(void);

/* USER CODE BEGIN PFP */
uint16_t count=0;
char buffer5[20];
int tmpIntPM1,tmpIntte1,tmpInthu1,tmpIntPM2,tmpIntte2,tmpInthu2;
//uint8_t rxBuffer[4];
uint8_t frame[60];
uint8_t tempe1[5] = "";
uint8_t tempe2[5] = "";
uint8_t humi1[5] = "";
uint8_t humi2[5] = "";
uint8_t CO2[5] = "";
uint8_t VOC[5] = "";
uint8_t PM251[5] = "";
uint8_t PM252[5] = "";
char  Rx_data[2], Rx_Buffer[40], Transfer_cplt;
char buffer[20],buffer1[20],buffer2[20],buffer3[20],buffer4[20];
int tmpInt1,tmpInt2;
uint16_t i = 0;
int16_t err;
uint16_t tvoc_ppb = 1, co2_ppm = 2;
uint32_t iaq_baseline;
int32_t temperature, humidity;
float temp, hum;
float PM25;
int16_t ret;
//Động cơ
int speed ;
int power;
int Vanst = 0;
// mode
int nightmode ;
int filmode;
int ctrmode;
bool huong;
int Rx_indx = 0;
uint8_t rxBuffer[4];
uint16_t val[5];
int test = 12345;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
   tft_init();
   tft_puts_image(vht1);
   HAL_Delay(3000);
   tft_fill(0,0,320,240,BLACK);
   HAL_UART_Receive_IT(&huart1, Rx_data, 1);
   // set PWM cho tốc độ động cơ quạt là trung bình
      speed = 1;
      power = 0;
      ctrmode = 0;
      filmode = 0;
      nightmode = 0;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1 */
  osThreadDef(Task1, StartTask1, osPriorityNormal, 0, 128);
  Task1Handle = osThreadCreate(osThread(Task1), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, StartTask02, osPriorityIdle, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_LED_Pin|LCD_DC_RS_Pin|LCD_RESET_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC4 PC5 LCD_LED_Pin 
                           LCD_DC_RS_Pin LCD_RESET_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|LCD_LED_Pin 
                          |LCD_DC_RS_Pin|LCD_RESET_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if (huart->Instance == USART1)  //Xét UART nhận dữ liệu
        {
        if (Rx_indx==0 && Transfer_cplt==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_buffer trước khi nhận dữ liệu mới

        if (Rx_data[0]!='\n') //Nếu nhận dữ liệu là khác dấu xuống dòng
            {
            Rx_Buffer[Rx_indx++]=Rx_data[0];    //thêm dữ liệu vào Rx_Buffer
            }
        else            //nếu là dấu xuống dòng \n thì hoàn thành việc đọc 1 khung truyền
            {
            Rx_indx=0;
            Transfer_cplt=1;//Cờ báo hiệu đã chuyển dữ liệu xong và tiến hành đọc dữ liệu
            }

        HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //Kích hoạt ngắt UART mỗi data nhận được
        }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask1 */



















void StartTask1(void const * argument)
{
    

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	count++;
	sprintf(buffer5,"%d",count);
	CheckUART();
	POINT_COLOR=CYAN;
	tft_fill(0,0,320,240,BLACK);
	tft_puts26x48(58,3,(int8_t*)" Viettel",TFT_STRING_MODE_NO_BACKGROUND);
	tft_puts26x48(58,5,(int8_t*)" Viettel",TFT_STRING_MODE_NO_BACKGROUND);
	CheckUART();
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {

	tft_fill(0,0,320,240,BLACK);
	POINT_COLOR=CYAN;
	IAQform();
	tft_puts26x48(160,90,(int8_t*)buffer5,TFT_STRING_MODE_BACKGROUND);
	//tft_puts18x32(160,3,(int8_t*)"PRODUCTS",TFT_STRING_MODE_NO_BACKGROUND);
	if (count >= 50 && count < 60){
		Convertvalue();
		FrameUART();
		HAL_UART_Transmit(&huart1, (char *) &frame, sizeof(frame), HAL_MAX_DELAY);
	}
    osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

void IAQform(void){

	  tft_draw_circle(150,119,87);
	  tft_draw_circle(150,119,88);
	  tft_draw_circle(150,119,89);
	  tft_draw_circle(150,119,90);
	  tft_draw_circle(150,119,91);
	  tft_draw_circle(150,119,92);


	  tft_puts26x48(100,80,(int8_t*)"IAQ",TFT_STRING_MODE_NO_BACKGROUND);
	  tft_puts26x48(100,81,(int8_t*)"IAQ",TFT_STRING_MODE_NO_BACKGROUND);

	  POINT_COLOR=GBLUE;
	  tft_puts8x16(17,3,(int8_t*)"VIETTEL HIGH TECHNOLOGY",TFT_STRING_MODE_NO_BACKGROUND);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void Convertvalue(void){

	PM25 = 22.33;
	tmpIntPM1 = (int)PM25;
	float tmpFPM = PM25 - tmpIntPM1;
	tmpIntPM2 = trunc(tmpFPM * 100);
	temperature = 9977;
	temp = (float)(temperature/1000.0f);
	tmpIntte1 = (int)temp;
	float tmpT = temp - tmpIntte1;
	tmpIntte2 = trunc(tmpT * 100);
	humidity = 9876;
	hum = (float)(humidity/1000.0f);
	tmpInthu1 = (int) hum;
	float tmpH = hum - tmpInthu1;
	tmpInthu2 = trunc(tmpH * 100);
}

/* Tạo Khung truyền với các giá trị lấy từ cảm biến và động cơ */
void FrameUART(void){
		itoa(tmpIntte1,(char*)tempe1,10);
		itoa(tmpIntte2,(char*)tempe2,10);
		itoa(tmpInthu1,(char*)humi1,10);
		itoa(tmpInthu2,(char*)humi2,10);
		itoa(co2_ppm,(char*)CO2,10);
		itoa(tvoc_ppb,(char*)VOC,10);
		itoa(tmpIntPM1,(char*)PM251,10);
		itoa(tmpIntPM2,(char*)PM252,10);

	   strcpy((char *) frame, (char *) "TE");
	   strcat((char *) frame,(char *) tempe1);
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "A");
	   strcat((char *) frame,(char *) tempe2);
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "HU");
	   strcat((char *) frame, (char *) humi1 );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "Q");
	   strcat((char *) frame, (char *) humi2 );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "CO");
	   strcat((char *) frame, (char *) CO2 );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "VO");
	   strcat((char *) frame, (char *) VOC );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "PM");
	   strcat((char *) frame, (char *) PM251 );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "S");
	   strcat((char *) frame, (char *) PM252 );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "PO");
	   if (power == 1)
	   strcat((char *) frame, (char *) "1" );
	   else if (power == 0)
	   strcat((char *) frame, (char *) "0" );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "SP");
	   if(speed == 0)
		   strcat((char *) frame, (char *) "0" );
	   else if(speed == 1)
		   strcat((char *) frame, (char *) "1" );
	   else if(speed == 2)
		   strcat((char *) frame, (char *) "2" );
	   else if(speed == 3)
		   strcat((char *) frame, (char *) "3" );
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "NIGH");
	   if (nightmode ==0){
		   strcat((char *) frame, (char *) "0" );
	   }
	   else if (nightmode == 1){
		   strcat((char *) frame, (char *) "1" );
	   }
	   strcat((char *) frame, (char *) ":");
	   strcat((char *) frame, (char *) "FILT");
	   if (filmode ==0){
			strcat((char *) frame, (char *) "0" );
		   }
	   else if (filmode == 1){
			strcat((char *) frame, (char *) "1" );
		   }
	   strcat((char *) frame, (char *) ":");
		   strcat((char *) frame, (char *) "CTRL");
		   if (ctrmode ==0){
				strcat((char *) frame, (char *) "0" );
			   }
		   else if (ctrmode == 1){
				strcat((char *) frame, (char *) "1" );
			   }
	   strcat((char *) frame, (char *) "\n");
}

void CheckUART(void){
	if(Transfer_cplt){

	  for (int i =0; i<=strlen(Rx_Buffer); i++){
		  if(Rx_Buffer[i] =='h' && Rx_Buffer[i+1]=='i'){ // Nhận nút điều khiển động cơ hight từ App

			  speed = 3;
			 // fan_1_run(hightspeed);
			 // fan_2_run(hightspeed);
			 // num = SPEED_HIGHT;
			  __HAL_TIM_SET_COUNTER(&htim2, 0);
			//  FLAG=1;
			 // FLAG1=1;
			 // FLAG2=1;
			//  FLAG3=1;
			//  FLAG4=1;
			  // Set tốc độ động cơ PWM ở đây
		  }
		  else if(Rx_Buffer[i] =='l' && Rx_Buffer[i+1]=='o'&& Rx_Buffer[i+2]=='w'){ // Nhận nút điều khiển động cơ low từ app


			  speed = 1;
			 // fan_1_run(lowspeed);
			 // fan_2_run(lowspeed);
			 // num = SPEED_LOW;
			  __HAL_TIM_SET_COUNTER(&htim2, 0);
			 // FLAG=1;
			 // FLAG1=1;
			 // FLAG2=1;
			 // FLAG3=1;
			 // FLAG4=1;
			 // Set tốc độ động cơ PWM ở đây
		 }
		  else if(Rx_Buffer[i] =='m' && Rx_Buffer[i+1]=='e'&& Rx_Buffer[i+2]=='d'){ // Nhận nút điều khiển động cơ med từ app

			  speed = 2;
			 // fan_1_run(medspeed);
			 // fan_2_run(medspeed);
			  //num = SPEED_MEDIUM;
			  __HAL_TIM_SET_COUNTER(&htim2, 0);
			 // FLAG=1;
			  //FLAG1=1;
			 // FLAG2=1;
			 // FLAG3=1;
			 // FLAG4=1;
			 // Set tốc độ động cơ PWM ở đây
		 }
	  //}
	 //for (int i =0; i<=strlen(Rx_Buffer); i++){
		 if(Rx_Buffer[i] =='P' && Rx_Buffer[i+1]=='O'&& Rx_Buffer[i+2]=='N'){

			 power = 1;
			 speed = 2;
			// fan_1_run(medspeed);
			// fan_2_run(medspeed);
			// num = POWER_ON;
			 __HAL_TIM_SET_COUNTER(&htim2, 0);
			 // FLAG=1;
			 // FLAG1=1;
			 // FLAG2=1;
			 // FLAG3=1;
			 // FLAG4=1;
			 // Set tốc độ động cơ PWM ở đây
			 // Set van khí ở đây
		 }
		 else if(Rx_Buffer[i] =='P' && Rx_Buffer[i+1]=='O'&& Rx_Buffer[i+2]=='F'){


			 power = 0;
			 speed = 0;
			// fan_1_run(100);
			// fan_2_run(100);
			// num = POWER_OFF;
			 __HAL_TIM_SET_COUNTER(&htim2, 0);
			 // FLAG=1;
			 // FLAG1=1;
			 // FLAG2=1;
			 // FLAG3=1;
			//  FLAG4=1;
			 //Tắt động cơ và đóng van khí ở đây
		 }
	// }
	// for (int i =0; i<=strlen(Rx_Buffer); i++){
	 		 if(Rx_Buffer[i] =='n' && Rx_Buffer[i+1]=='i'&& Rx_Buffer[i+2]=='g'){

	 			 nightmode = 1;
	 			// fan_1_run(lowspeed);
	 			// fan_2_run(lowspeed);
	 			// num = NIGHT_ON;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			//  FLAG=1;
				//  FLAG1=1;
				//  FLAG2=1;
				//  FLAG3=1;
				//  FLAG4=1;
	 		 }
	 		 else if(Rx_Buffer[i] =='d' && Rx_Buffer[i+1]=='a'&& Rx_Buffer[i+2]=='y'){

	 			nightmode = 0;
	 			//fan_1_run(medspeed);
	 			//fan_2_run(medspeed);
	 			//num = NIGHT_OFF;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			 // FLAG=1;
				  //FLAG1=1;
				 // FLAG2=1;
				//  FLAG3=1;
				 // FLAG4=1;
	 		 }
	 	//}
	 	//for (int i =0; i<=strlen(Rx_Buffer); i++){
	 		if(Rx_Buffer[i] =='f' && Rx_Buffer[i+1]=='r'&& Rx_Buffer[i+2]=='e'){

	 			filmode = 1;
	 			//num = FRESH_AIR;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			//  FLAG=1;
				//  FLAG1=1;
				//  FLAG2=1;
				//  FLAG3=1;
				//  FLAG4=1;
				//  Vanst = 1;

	 			//đk van mở van
	 		 }
	 		else if(Rx_Buffer[i] =='i' && Rx_Buffer[i+1]=='n'&& Rx_Buffer[i+2]=='d'){

	 			filmode = 0;
	 			//num = INDOOR;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			 // FLAG=1;
				 // FLAG1=1;
				//  FLAG2=1;
				//  FLAG3=1;
				//  FLAG4=1;
				//  Vanst = 0;
	 			//đk van đóng
	 		}
	 	//}
	 	//for (int i =0; i<=strlen(Rx_Buffer); i++){ // chế độ auto
	 		if(Rx_Buffer[i] =='m' && Rx_Buffer[i+1]=='a'&& Rx_Buffer[i+2]=='n'){

	 			ctrmode = 1;
	 			//num = MANUAL;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			//  FLAG=1;
				//  FLAG1=1;
				//  FLAG2=1;
				//  FLAG3=1;
				//  FLAG4=1;
	 		}
	 		else if(Rx_Buffer[i] =='a' && Rx_Buffer[i+1]=='u'&& Rx_Buffer[i+2]=='t'){ // chế độ manual

	 			ctrmode = 0;
	 			//num = AUTO;
	 			 __HAL_TIM_SET_COUNTER(&htim2, 0);
	 			//  FLAG=1;
				//  FLAG1=1;
				//  FLAG2=1;
				//  FLAG3=1;
				//  FLAG4=1;
	 		}
	 	}
	  for (int i =0; i<=strlen(Rx_Buffer); i++){
		  if(Rx_Buffer[i] =='s' && Rx_Buffer[i+1]=='t'&& Rx_Buffer[i+2]=='a'){
			 //Đọc giá trị từ cảm biến
			 // ret = sps30_read_measurement(&m);
			 // if (ret < 0) {
				// PM25 = PM25;
							  //error
			 // } else {
			//	 PM25 = m.mc_2p5;
			//  }
			//  err = svm_measure_iaq_blocking_read(&tvoc_ppb, &co2_ppm, &temperature, &humidity);

			 // if (err == STATUS_OK) {
					//lay thanh cong
			//  } else {
				//	co2_ppm = co2_ppm;
				//	temperature = temperature;
				//	humidity = humidity;
				//	tvoc_ppb = tvoc_ppb;
					//khong the doc tu cam bien
			  //}

			  Convertvalue();
			  FrameUART();
			  HAL_UART_Transmit(&huart1, (char *) &frame, sizeof(frame), HAL_MAX_DELAY);
		   }
	  }

	  Transfer_cplt = 0;
	  HAL_Delay(400);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
