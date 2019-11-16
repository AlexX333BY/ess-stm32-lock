/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define LED_SIGNAL_TIMEOUT 2000
#define DISPLAY_REACT_TIME 1
#define PASSWORD_LENGTH 4

/* Private typedef -----------------------------------------------------------*/
enum InputState
{
  IDLE,
  FIRST_PASSWORD_INPUT,
  NEW_PUBLIC_PASSWORD_INPUT
};

struct LedsState
{
  bool red, yellow, green;
};

enum SymbolType
{
  NONE,
  NUMBER,
  STAR,
  SHARP
};

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
const uint8_t MASTER_PASSWORD[] = { 1, 3, 3, 7 };
uint8_t publicPassword[] = { 0, 0, 0, 0 };
enum InputState currentState = IDLE;
uint8_t enteredSymbolsCount;
uint8_t enteredSymbols[4];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  
enum SymbolType GetPressedSymbol(uint8_t*);
int8_t GetPressedRow(void);
int8_t GetPressedColumn(void);

void SetLedsState(const struct LedsState);
void SetLedsStateFor(const struct LedsState, const uint32_t, const struct LedsState);
uint16_t GetPinsForNumber(const uint8_t);

void IdleHandler(void);
void FirstPasswordInputHandler(void);
void NewPublicPasswordInputHandler(void);
void Reset(void);

bool ArePasswordsEqual(const uint8_t[], const uint8_t[], const uint8_t length);

/* Private user code ---------------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (currentState)
  {
    case IDLE:
      IdleHandler();
      break;
    case FIRST_PASSWORD_INPUT:
      FirstPasswordInputHandler();
      break;
    case NEW_PUBLIC_PASSWORD_INPUT:
      NewPublicPasswordInputHandler();
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  const uint16_t allSegments = DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_E_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
  const uint16_t displayControls[PASSWORD_LENGTH] = { DISPLAY_1_Pin, DISPLAY_2_Pin, DISPLAY_3_Pin, DISPLAY_4_Pin };
  for (uint8_t curSym = 0; curSym < enteredSymbolsCount; ++curSym)
  {
    HAL_GPIO_WritePin(GPIOA, displayControls[curSym], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, allSegments, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GetPinsForNumber(enteredSymbols[curSym]), GPIO_PIN_RESET);
    HAL_Delay(DISPLAY_REACT_TIME);
    HAL_GPIO_WritePin(GPIOA, displayControls[curSym], GPIO_PIN_RESET);
  }
  for (uint8_t curSym = enteredSymbolsCount; curSym < PASSWORD_LENGTH; ++curSym)
  {
    HAL_GPIO_WritePin(GPIOA, displayControls[curSym], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, allSegments, GPIO_PIN_SET);
    HAL_Delay(DISPLAY_REACT_TIME);
    HAL_GPIO_WritePin(GPIOA, displayControls[curSym], GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(GPIOA, allSegments, GPIO_PIN_RESET);
}

enum SymbolType GetPressedSymbol(uint8_t* number)
{
  const int8_t row = GetPressedRow(), column = GetPressedColumn();
  enum SymbolType type = NONE;
  if ((row != -1) && (column != -1))
  {
    if ((row == 3) && (column == 0))
    {
      type = STAR;
    }
    else if ((row == 3) && (column == 2))
    {
      type = SHARP;
    }
    else
    {
      type = NUMBER;
      if (number != NULL)
      {
        if (row == 3)
        {
          *number = 0;
        }
        else
        {
          *number = 3 * row + column + 1;
        }
      }
    }
  }
  return type;
}

int8_t GetPressedRow(void)
{
  int8_t row = -1;
  if (HAL_GPIO_ReadPin(KEYPAD_A_GPIO_Port, KEYPAD_A_Pin) == GPIO_PIN_SET)
  {
    row = 0;
  }
  else if (HAL_GPIO_ReadPin(KEYPAD_B_GPIO_Port, KEYPAD_B_Pin) == GPIO_PIN_SET)
  {
    row = 1;
  }
  else if (HAL_GPIO_ReadPin(KEYPAD_C_GPIO_Port, KEYPAD_C_Pin) == GPIO_PIN_SET)
  {
    row = 2;
  }
  else if (HAL_GPIO_ReadPin(KEYPAD_D_GPIO_Port, KEYPAD_D_Pin) == GPIO_PIN_SET)
  {
    row = 3;
  }
  return row;
}

int8_t GetPressedColumn(void)
{
  int8_t column = -1;
  if (HAL_GPIO_ReadPin(KEYPAD_1_GPIO_Port, KEYPAD_1_Pin) == GPIO_PIN_SET)
  {
    column = 0;
  }
  else if (HAL_GPIO_ReadPin(KEYPAD_2_GPIO_Port, KEYPAD_2_Pin) == GPIO_PIN_SET)
  {
    column = 1;
  }
  else if (HAL_GPIO_ReadPin(KEYPAD_3_GPIO_Port, KEYPAD_3_Pin) == GPIO_PIN_SET)
  {
    column = 2;
  }
  return column;
}

void SetLedsState(const struct LedsState state)
{
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, state.red ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, state.yellow ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, state.green ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SetLedsStateFor(const struct LedsState state, const uint32_t delay, const struct LedsState fallbackState)
{
  SetLedsState(state);
  HAL_Delay(delay);
  SetLedsState(fallbackState);
}

uint16_t GetPinsForNumber(const uint8_t numberToDisplay)
{
  switch (numberToDisplay)
  {
    case 0:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_E_Pin | DISPLAY_F_Pin;
    case 1:
      return DISPLAY_B_Pin | DISPLAY_C_Pin;
    case 2:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_D_Pin | DISPLAY_E_Pin | DISPLAY_G_Pin;
    case 3:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_G_Pin;
    case 4:
      return DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
    case 5:
      return DISPLAY_A_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
    case 6:
      return DISPLAY_A_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_E_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
    case 7:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin;
    case 8:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_E_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
    case 9:
      return DISPLAY_A_Pin | DISPLAY_B_Pin | DISPLAY_C_Pin | DISPLAY_D_Pin | DISPLAY_F_Pin | DISPLAY_G_Pin;
    default:
      return 0;
  }
}

void IdleHandler()
{
  if (GetPressedSymbol(NULL) == STAR)
  {
    enteredSymbolsCount = 0;
    HAL_TIM_Base_Start_IT(&htim1);
    currentState = FIRST_PASSWORD_INPUT;
  }
}

void FirstPasswordInputHandler(void)
{
  uint8_t pressedNumber;
  const enum SymbolType symType = GetPressedSymbol(&pressedNumber);
  if (symType == NUMBER)
  {
    enteredSymbols[enteredSymbolsCount++] = pressedNumber;
    if (enteredSymbolsCount == PASSWORD_LENGTH)
    {
      if (ArePasswordsEqual(enteredSymbols, MASTER_PASSWORD, PASSWORD_LENGTH))
      {
        Reset();
        const struct LedsState state = { .red = true, .yellow = true, .green = true };
        SetLedsState(state);
        currentState = NEW_PUBLIC_PASSWORD_INPUT;
        HAL_TIM_Base_Start_IT(&htim1);
        while (GetPressedSymbol(NULL) == NUMBER)
        {
          HAL_Delay(1);
        }
      }
      else
      {
        const bool isPasswordCorrect = ArePasswordsEqual(enteredSymbols, publicPassword, PASSWORD_LENGTH);
        const struct LedsState state = { .red = !isPasswordCorrect, .yellow = false, .green = isPasswordCorrect },
          fallbackState = { .red = false, .yellow = false, .green = false };
        SetLedsStateFor(state, LED_SIGNAL_TIMEOUT, fallbackState);
        Reset();
      }
    }
    else
    {
      while (GetPressedSymbol(NULL) == NUMBER)
      {
        HAL_Delay(1);
      }
    }
  }
  else if (symType == SHARP)
  {
    const struct LedsState state = { .red = false, .yellow = true, .green = false },
      fallbackState = { .red = false, .yellow = false, .green = false };
    SetLedsStateFor(state, LED_SIGNAL_TIMEOUT, fallbackState);
    Reset();
  }
}

void NewPublicPasswordInputHandler(void)
{
  uint8_t pressedNumber;
  const enum SymbolType symType = GetPressedSymbol(&pressedNumber);
  if (symType == NUMBER)
  {
    enteredSymbols[enteredSymbolsCount++] = pressedNumber;
    if (enteredSymbolsCount == PASSWORD_LENGTH)
    {
      for (uint8_t i = 0; i < PASSWORD_LENGTH; ++i)
      {
        publicPassword[i] = enteredSymbols[i];
      }
      const struct LedsState state = { .red = false, .yellow = false, .green = true },
        fallbackState = { .red = false, .yellow = false, .green = false };
      SetLedsStateFor(state, LED_SIGNAL_TIMEOUT, fallbackState);
      Reset();
    }
    else
    {
      while (GetPressedSymbol(NULL) == NUMBER);
    }
  }
  else if ((symType == STAR) || (symType == SHARP))
  {
    const struct LedsState state = { .red = true, .yellow = false, .green = false },
      fallbackState = { .red = false, .yellow = false, .green = false };
    SetLedsStateFor(state, LED_SIGNAL_TIMEOUT, fallbackState);
    Reset();
  }
}

void Reset(void)
{
  HAL_TIM_Base_Stop_IT(&htim1);
  enteredSymbolsCount = 0;
  const struct LedsState state = { .red = false, .yellow = false, .green = false };
  SetLedsState(state);
  currentState = IDLE;
}

bool ArePasswordsEqual(const uint8_t first[], const uint8_t second[], const uint8_t length)
{
  bool result = true;
  for (uint8_t i = 0; result && (i < length); ++i)
  {
    result = result && (first[i] == second[i]);
  }
  return result;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();

  /* Infinite loop */
  const uint16_t Scan_Pins[] = { KEYPAD_A_Pin, KEYPAD_B_Pin, KEYPAD_C_Pin, KEYPAD_D_Pin };
  const uint8_t Scan_Pins_Count = sizeof(Scan_Pins) / sizeof(Scan_Pins[0]);
  for (uint8_t Cur_Pin = 0;; Cur_Pin = (Cur_Pin + 1) % Scan_Pins_Count)
  {
    HAL_GPIO_WritePin(GPIOB, Scan_Pins[Cur_Pin], GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, Scan_Pins[Cur_Pin], GPIO_PIN_RESET);
    HAL_Delay(1);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10 * DISPLAY_REACT_TIME;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISPLAY_G_Pin|DISPLAY_F_Pin|DISPLAY_E_Pin|DISPLAY_D_Pin 
                          |DISPLAY_C_Pin|DISPLAY_B_Pin|DISPLAY_A_Pin|DISPLAY_1_Pin 
                          |DISPLAY_2_Pin|DISPLAY_3_Pin|DISPLAY_4_Pin|LED_RED_Pin 
                          |LED_YELLOW_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD_A_Pin|KEYPAD_B_Pin|KEYPAD_C_Pin|KEYPAD_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPLAY_G_Pin DISPLAY_F_Pin DISPLAY_E_Pin DISPLAY_D_Pin 
                           DISPLAY_C_Pin DISPLAY_B_Pin DISPLAY_A_Pin DISPLAY_1_Pin 
                           DISPLAY_2_Pin DISPLAY_3_Pin DISPLAY_4_Pin LED_RED_Pin 
                           LED_YELLOW_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = DISPLAY_G_Pin|DISPLAY_F_Pin|DISPLAY_E_Pin|DISPLAY_D_Pin 
                          |DISPLAY_C_Pin|DISPLAY_B_Pin|DISPLAY_A_Pin|DISPLAY_1_Pin 
                          |DISPLAY_2_Pin|DISPLAY_3_Pin|DISPLAY_4_Pin|LED_RED_Pin 
                          |LED_YELLOW_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_A_Pin KEYPAD_B_Pin KEYPAD_C_Pin KEYPAD_D_Pin */
  GPIO_InitStruct.Pin = KEYPAD_A_Pin|KEYPAD_B_Pin|KEYPAD_C_Pin|KEYPAD_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_1_Pin KEYPAD_2_Pin KEYPAD_3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_1_Pin|KEYPAD_2_Pin|KEYPAD_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
