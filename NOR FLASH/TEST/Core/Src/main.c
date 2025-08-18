/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define OLED_ADDRESS 0x3C
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Ledstatus;
//----------------------------------------------------------------------
// Send printf to uart1
int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == 1 || fd == 2) {
    hstatus = HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return -1;
  }
  return -1;
}
//----------------------------------------------------------------------
GPIO_TypeDef* ports[] = {LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port, LED4_GPIO_Port,
                           LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port, LED8_GPIO_Port};
uint16_t pins[] = {LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin,
                     LED5_Pin, LED6_Pin, LED7_Pin, LED8_Pin};
//----------------------------------------------------------------------
uint8_t Ledstatus;
void prinLedStatus()
{

  for (int i = 0; i < 8; i++)
  {
	if (Ledstatus & (1 << i))
	{
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
	}else
	{
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
	}
  }

}
//-----------------------------------------------------------------------------
void HandleButtons(uint16_t GPIO_Pin)
{
    if (Ledstatus == 0)
    {
        Ledstatus = 1;
    }
    if (GPIO_Pin == BTN1_Pin)
    {
        Ledstatus <<= 1;
        if (Ledstatus == 0) Ledstatus = 1;
    }
    else if (GPIO_Pin == BTN2_Pin)
    {
        Ledstatus >>= 1;
        if (Ledstatus == 0) Ledstatus = 0x80;
    }
}


//--------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t PGIO_Pin)
{
	HandleButtons(PGIO_Pin);
}
//--------------------------------------------------------------------------

void handlSlideSwitch()
{
	GPIO_PinState btnState = HAL_GPIO_ReadPin(SlideSwitch_GPIO_Port, SlideSwitch_Pin);
	if (btnState == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LEDR_GPIO_Port,LEDR_Pin, GPIO_PIN_RESET);
	}
}

//--------------------------------------------------------------------------
void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, r); // Красный
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, g); // Зелёный
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, b); // Синий
}
//---------------------------------------------------------------------------

void Buzzer_SetDuty(uint16_t duty) {
    if (duty > htim2.Init.Period) duty = htim2.Init.Period;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}


//--------------------------------------------------------------------------
void handlSlideToBuzzer()
{
	GPIO_PinState btnState = HAL_GPIO_ReadPin(SlideSwitch_GPIO_Port, SlideSwitch_Pin);
	if (btnState == GPIO_PIN_SET)
	{
		 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		 Buzzer_SetDuty( (htim2.Init.Period + 1) / 2 );
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}
}

//-------------------------------------------------------------------------
int32_t encoder_value = 0;
uint8_t active_leds = 0;
const uint8_t led_sequence[8] = {1, 3, 7, 15, 31, 63, 127, 255};
uint16_t adc_value = 0;
//-------------------------------------------------------------------------

void UpdateEncoder()
{
	static uint16_t last_cnt = 0;
	    uint16_t curr_cnt = TIM3->CNT;
	    int16_t diff = curr_cnt - last_cnt;

	    // Обработка переполнения с ARR = 30
	    if(diff > 15) {
	        diff -= 30;
	    }
	    else if(diff < -15) {
	        diff += 30;
	    }

	    encoder_value += diff;
	    last_cnt = curr_cnt;

	    // Изменяем индекс только при значительном изменении положения
	    static int8_t accumulated_diff = 0;
	    accumulated_diff += diff;

	    if(accumulated_diff >= 4) { // Порог срабатывания +
	        if(active_leds < 7) active_leds++;
	        accumulated_diff = 0;
	    }
	    else if(accumulated_diff <= -4) { // Порог срабатывания -
	        if(active_leds > 0) active_leds--;
	        accumulated_diff = 0;
	    }

}


//-------------------------------------------------------------------------

int HandlAdctoLed(float value)
{
	int index = (int)(value / 0.5f); // шаг 0.5 В
	if(index > 7) index = 7;
	return led_sequence[index];
}


//-------------------------------------------------------------------------
/* Variables for AHT10 */
#define AHT10_ADDR         (0x38 << 1)
#define AHT10_INIT_CMD     0xE1
#define AHT10_MEASURE_CMD  0xAC
#define AHT10_DATA0        0x33
#define AHT10_DATA1        0x00

uint8_t AHT10_RX_Data[6];
uint32_t AHT10_ADC_Raw;
float AHT10_Temperature;
float AHT10_Humidity;


void AHT10_Init(void)
{
    uint8_t init_cmd[3] = {AHT10_INIT_CMD, 0x08, 0x00}; // 0x08 — калибровка
    HAL_I2C_Master_Transmit(&hi2c2, AHT10_ADDR, init_cmd, 3, 100);
    HAL_Delay(500);
}


void AHT10_ReadData(void)
{
    uint8_t measure_cmd[3] = {AHT10_MEASURE_CMD, AHT10_DATA0, AHT10_DATA1};


    HAL_I2C_Master_Transmit(&hi2c2, AHT10_ADDR, measure_cmd, 3, 100);
    HAL_Delay(100);


    HAL_I2C_Master_Receive(&hi2c2, AHT10_ADDR, AHT10_RX_Data, 6, 100);


    uint32_t hum_raw = ((uint32_t)(AHT10_RX_Data[1]) << 12) |
                       ((uint32_t)(AHT10_RX_Data[2]) << 4)  |
                       ((AHT10_RX_Data[3] & 0xF0) >> 4);

    AHT10_Humidity = ((float)hum_raw / 1048576.0f) * 100.0f;


    AHT10_ADC_Raw = ((uint32_t)(AHT10_RX_Data[3] & 0x0F) << 16) |
                    ((uint32_t)(AHT10_RX_Data[4]) << 8) |
                    (AHT10_RX_Data[5]);

    AHT10_Temperature = ((float)AHT10_ADC_Raw / 1048576.0f) * 200.0f - 50.0f;
}


//-------------------------------------------------------------------
uint16_t read_val = 0;
#define CMD_READ   0b10
#define CMD_WRITE  0b01
#define CMD_ERASE  0b11
#define CMD_EWEN   0b00
#define CMD_EWDS   0b00
#define CMD_ERAL   0b00
#define CMD_WRAL   0b00
#define EWEN_ADDR  0b110000
#define EWDS_ADDR  0b000000
#define ERAL_ADDR  0b100000
#define WRAL_ADDR  0b010000

// Timing constants (from datasheet)
#define T_CS_MIN_US    1    // Minimum CS low time (250ns, using 1us for safety)
#define T_WP_MAX_MS    5    // Maximum write cycle time

static SPI_HandleTypeDef *eeprom_spi = &hspi2;

// Chip Select control
static inline void EEPROM_CS_Enable(void) {
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
    HAL_Delay(T_CS_MIN_US);  // Meet tCS requirement
}

static inline void EEPROM_CS_Disable(void) {
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
}

// Check Ready/Busy status (via DO pin)
static bool EEPROM_CheckReady(void) {
    uint8_t status;
    EEPROM_CS_Enable();
    HAL_SPI_Receive(eeprom_spi, &status, 1, HAL_MAX_DELAY);
    EEPROM_CS_Disable();
    return (status & 0x01);  // Ready when DO = 1
}

// Wait for write completion with timeout
static bool EEPROM_WaitReady(void) {
    uint32_t start = HAL_GetTick();
    while(!EEPROM_CheckReady()) {
        if(HAL_GetTick() - start > T_WP_MAX_MS) {
            return false;  // Timeout
        }
    }
    return true;
}


// Enable Write Operations
void EEPROM_EWEN(void) {
    uint16_t cmd = (1 << 8) | (CMD_EWEN << 6) | (EWEN_ADDR << 0);
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    EEPROM_CS_Enable();
    HAL_SPI_Transmit(eeprom_spi, buf, 2, HAL_MAX_DELAY);
    EEPROM_CS_Disable();
}

// Disable Write Operations
void EEPROM_EWDS(void) {
    uint16_t cmd = (1 << 8) | (CMD_EWDS << 6) | (EWDS_ADDR << 0);
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    EEPROM_CS_Enable();
    HAL_SPI_Transmit(eeprom_spi, buf, 2, HAL_MAX_DELAY);
    EEPROM_CS_Disable();
}

uint32_t received_data = 0x0AAABFF;

uint16_t extract_16bit_value(uint32_t raw_data) {
    // Shift right by 7 bits to remove trailing 1s, then mask 16 bits
    return (raw_data >> 7) & 0xFFFF;
}

// Read 16-bit word (with proper dummy bit handling)
uint16_t EEPROM_ReadWord(uint8_t address) {
	uint16_t cmd = (1 << 8) | (CMD_READ << 6) | (address & 0x3F);
	uint8_t tx[2] = {cmd >> 8, cmd & 0xFF};
	uint8_t rx[3] = {0};

	EEPROM_CS_Enable();
	HAL_SPI_Transmit(eeprom_spi, tx, 2, HAL_MAX_DELAY);
	HAL_SPI_Receive(eeprom_spi, rx, 3, HAL_MAX_DELAY);
	EEPROM_CS_Disable();

	// Combine all 24 bits received
	uint32_t raw_data = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];

	// Extract correct 16 bits (bits 7-22)
	return (raw_data >> 7) & 0xFFFF;
}

// Write 16-bit word with full protocol
bool EEPROM_WriteWord(uint8_t address, uint16_t data) {
    // 1. Enable writes
    EEPROM_EWEN();

    // 2. Prepare write command + data
    uint16_t cmd = (1 << 8) | (CMD_WRITE << 6) | (address & 0x3F);
    uint8_t tx[4] = {cmd >> 8, cmd & 0xFF, data >> 8, data & 0xFF};

    // 3. Start write operation
    EEPROM_CS_Enable();
    HAL_SPI_Transmit(eeprom_spi, tx, 4, HAL_MAX_DELAY);

    // 4. Check status immediately (before CS goes high)
    uint8_t status;
    HAL_SPI_Receive(eeprom_spi, &status, 1, HAL_MAX_DELAY);
    EEPROM_CS_Disable();

    // 5. If we got status, wait for completion
    if(status & 0x01) {
        return true;  // Already done
    }

    // 6. Otherwise wait with timeout
    if(!EEPROM_WaitReady()) {
        return false;  // Write failed
    }

    // 7. Disable writes (recommended)
    EEPROM_EWDS();
    return true;
}

//---------------------------------------------------------------------

uint8_t read_val_nor[] = {0xAA, 0xBA, 0xCA, 0xDA};
uint8_t write_val_nor[4] = {0};


void FM25F01C_CS_Low(void) {
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET );
}

void FM25F01C_CS_High(void) {
    HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET );
}


void FM25F01C_WriteEnable(void) {
    uint8_t cmd = 0x06; // WREN instruction
    FM25F01C_CS_Low();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    FM25F01C_CS_High();
}

void FM25F01C_WriteDisable(void) {
    uint8_t cmd = 0x04; // WRDI instruction
    FM25F01C_CS_Low();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    FM25F01C_CS_High();
}

uint8_t FM25F01C_ReadStatusRegister(void) {
    uint8_t cmd = 0x05; // RDSR instruction
    uint8_t status = 0;
    FM25F01C_CS_Low();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &status, 1, HAL_MAX_DELAY);
    FM25F01C_CS_High();
    return status;
}

void FM25F01C_WaitForWriteComplete(void) {
    while (FM25F01C_ReadStatusRegister() & 0x01); // Проверка бита WIP (Write In Progress)
}

void FM25F01C_PageProgram(uint32_t address, uint8_t *data, uint16_t length) {
    uint8_t cmd[4];
    cmd[0] = 0x02; // Page Program instruction
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FM25F01C_WriteEnable();
    FM25F01C_CS_Low();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, data, length, HAL_MAX_DELAY);
    FM25F01C_CS_High();
    FM25F01C_WaitForWriteComplete();
}

void FM25F01C_ReadData(uint32_t address, uint8_t *buffer, uint16_t length) {
    uint8_t cmd[4];
    cmd[0] = 0x03; // Read Data instruction
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    FM25F01C_CS_Low();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, buffer, length, HAL_MAX_DELAY);
    FM25F01C_CS_High();
}





//---------------------------------------------------------------------

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_ADC_Start(&hadc1);

  SSD1306_Init();

  SSD1306_GotoXY(0,0);
  SSD1306_Puts ("HELLO WORLD", &Font_7x10, 1);
  SSD1306_UpdateScreen();


  AHT10_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint16_t write_data = 0x5555;

  // Запись слова по адресу
  EEPROM_WriteWord(0x10, write_data);
  HAL_Delay(10);  // Подождём завершения операции

  // Чтение обратно
   read_val = EEPROM_ReadWord(0x010);


  FM25F01C_PageProgram(0x000000, write_val_nor, sizeof(write_val_nor));
  FM25F01C_ReadData(0x000000, read_val_nor, sizeof(read_val_nor));

  Ledstatus = 0;
  while (1)
  {
	  UpdateEncoder();

	  // Управляем светодиодами в зависимости от положения энкодера
	  //Ledstatus = active_leds;
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adc_value = HAL_ADC_GetValue(&hadc1);


	  float voltage = (adc_value * 3.3f) / 4095.0f;

	  Ledstatus = HandlAdctoLed(voltage);
	  prinLedStatus();
	  handlSlideSwitch();
	  handlSlideToBuzzer();


	  char buf[20];
	  sprintf(buf, "VOLTAGE %.2f", voltage);

	  SSD1306_GotoXY(0, 10);
	  SSD1306_Puts(buf, &Font_7x10, 1);
	  SSD1306_UpdateScreen();


	  AHT10_ReadData();

	  char buf2[20];
	  sprintf(buf2, "Temperature %.2f", AHT10_Temperature);

	  SSD1306_GotoXY(0, 20);
	  SSD1306_Puts(buf2, &Font_7x10, 1);
	  SSD1306_UpdateScreen();

	  char buf3[20];
	  sprintf(buf3, "Humidity %.2f", AHT10_Humidity);

	  SSD1306_GotoXY(0, 30);
	  SSD1306_Puts(buf3, &Font_7x10, 1);
	  SSD1306_UpdateScreen();


	  char buf4[20];
	  if(read_val == write_data)
	  {
		  sprintf(buf4, "EEPROM OK");
	  }
	  else
	  {
		  sprintf(buf4, "EEPROM NOK");
	  }

	  SSD1306_GotoXY(0, 40);
	  SSD1306_Puts(buf4, &Font_7x10, 1);
	  SSD1306_UpdateScreen();


	  char buf5[20];
	  if(read_val_nor[0] == write_val_nor[0] &&
	     read_val_nor[1] == write_val_nor[1] &&
		 read_val_nor[2] == write_val_nor[2] &&
		 read_val_nor[3] == write_val_nor[3] )
	  {
		  sprintf(buf5, "NOR FLASH OK");
	  }
	  else
	  {
		  sprintf(buf5, "NOR FLASH NOK");
	  }

	  SSD1306_GotoXY(0, 50);
	  SSD1306_Puts(buf5, &Font_7x10, 1);
	  SSD1306_UpdateScreen();


	/*  RGB_SetColor(255, 0, 0);   // Красный
	  HAL_Delay(500);
	  RGB_SetColor(0, 255, 0);   // Зелёный
	  HAL_Delay(500);
	  RGB_SetColor(0, 0, 255);   // Синий
	  HAL_Delay(500);
	  RGB_SetColor(255, 255, 255); // Белый
	  HAL_Delay(500);
	  RGB_SetColor(128, 0, 255);
	  HAL_Delay(500);*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 30;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEDR_Pin|CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SlideSwitch_Pin */
  GPIO_InitStruct.Pin = SlideSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SlideSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED5_Pin LED7_Pin LED8_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED5_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED6_Pin */
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDR_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS2_Pin */
  GPIO_InitStruct.Pin = CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
