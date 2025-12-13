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
#include "I2C_lcd.h"
#include <string.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum{
	ING,
	WIN,
	OVER
}Game_status;
Game_status game_status=ING;

typedef enum{
	LEVEL1=1,
	LEVEL2,
	LEVEL3
}Level;
Level level=LEVEL1;

typedef enum{
	UP,
	DOWN,
	RIGHT,
	LEFT,
	NONE
}Direction;

typedef struct{
	int row;
	int col;
	int image_num;
	int past_position[2][16];
	uint8_t food;
}Character;

typedef struct{
	int row;
	int col;
	int image_num;
	uint8_t clock_before;
}Enemy;

uint32_t dir[2];

uint8_t clk_pulse;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
Direction Dir_Joystick() // 조이스틱 방향 출력 함수
{
	// 중간값에 가까워질수록 민감도 높아짐.
	if(dir[0] > 3150) return LEFT;
	else if(dir[0] < 600) return RIGHT;
	else if(dir[1] > 3150) return UP;
	else if(dir[1] < 600) return DOWN;
	else return NONE;
}

void LoseSound(void);
void WinSound(void);
void LevelupInit(Character *character, Enemy *enemy);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Move_Pacman(Character *character, Direction direc)
{
	switch(direc)
	{
	case RIGHT :
		character->col++; // RIGHT 입력 시, 1칸 이동
		if(character->col > 15) character->col = 15; // 칸 넘어가지 말고 그 자리에서 정지
		character->image_num &= ~(0x2); // 오른쪽 방향 이미지 : 0, 1
		character->image_num ^= 1;
//		LCD_Display_Charactor(character);
		break;

	case LEFT :
		character->col--;
		if(character->col < 0) character->col = 0;
		character->image_num |= 0x2; // 왼쪽 방향 이미지 : 2, 3
		character->image_num ^= 1;
//		LCD_Display_Charactor(character);
		break;

	case UP :
		character->row--;
		if(character->row < 0) character->row = 0;
//		LCD_Display_Charactor(character);
		break;

	case DOWN :
		character->row++;
		if(character->row > 1) character->row = 1;
//		LCD_Display_Charactor(character);
		break;

	default :
//		LCD_Display_Charactor(character);
		break;
	}
	character->past_position[character->row][character->col] = 1;
	// 캐릭터 시작은 0,0
	// 모든 위치에 한 번씩 다 도달하여 이차원배열이 1로 채워지면,
	// 먹이를 다 먹었다는 뜻.
}


void LCD_Display_Charactor(Character *character)
{
	uint8_t count = 0; // 먹이의 갯수를 세는 변수 count
//	lcd_clear(); // 클리어하지 말고 OverWrite하면 점멸 현상 대부분 완화됨.
	lcd_put_cur(character->row, character->col); // 캐릭터의 현재 위치로 커서 이동
	lcd_send_data(character->image_num); // 캐릭터 이미지 데이터를 LCD에 출력

	// 캐릭터 먹이 생성
	for(int i = 0 ; i <= 1 ; i++)
	{
		for(int j = 0 ; j < 16 ; j++)
		{
			if (character->past_position[i][j] != 1) // pacman이 지나가지 않은 곳에 먹이 생성
			{
				lcd_put_cur(character->row, character->col); // 캐릭터의 현재 위치로 커서 이동
				lcd_send_data(character->image_num);
				lcd_put_cur(i, j); // 지나간 위치 빼고 모든 위치에 먹이 배치
				lcd_send_data(0xa5); // 먹이 모양 : 0xa5 / LCD 데이터 시트 참조
				count++;
			}
			else if(character->past_position[i][j] == 1) // 먹이를 먹은 자리만 clear(OverWrite하기 위해)
			{
				lcd_put_cur(character->row, character->col); // 캐릭터의 현재 위치로 커서 이동
				lcd_send_data(character->image_num);
				lcd_put_cur(i, j);
				lcd_send_data(0x20); // 빈 칸 데이터 : 0010 0000(=0x20)
			}
		}
	}
	// 생성된 먹이의 갯수와 캐릭터가 먹은 먹이의 수를 비교하여 소리 조절
	if(count < character->food)
	{
		// PWM 사용하여 소리 출력
		TIM3->CCR1 = TIM3->ARR / 2;
		character->food = count;
	}
}


void Move_Enemy(Enemy *enemy, Character character, uint8_t clk_pulse) // enemy는 포인터 변수, charactor는 구조체이므로 enemy->col, charaoctr->col
{
	uint8_t move = rand()%2; // 0 또는 1 중의 무작위값 선택

	if(clk_pulse == 1 && enemy->clock_before == 0)
	{
		if(move == 0)
		{
			if(enemy->row != character.row) // Enemy와 Charactoc의 행이 다른 경우
			{
				enemy->row = character.row; // Enemy는 Charactor쪽으로 이동한다.
				lcd_put_cur(enemy->row, enemy->col);
				lcd_send_data(enemy->image_num);
			}
		}
		else if(move == 1)
		{
			if(enemy->col > character.col) // enemy가 charactor보다 오른쪽에 있다면,
			{
				enemy->col--; // 왼쪽으로 이동시켜라.
				lcd_put_cur(enemy->row, enemy->col);
				lcd_send_data(enemy->image_num);
				if(enemy->col < 0) // 화면 밖으로 나가지 않도록 설정
					enemy->col = 0;
			}
			else if(enemy->col < character.col) // enemy가 charactor보다 왼쪽에 있다면,
			{
				enemy->col++; // 오른쪽으로 이동시켜라
				lcd_put_cur(enemy->row, enemy->col);
				lcd_send_data(enemy->image_num);
				if(enemy->col > 15) // 화면 밖으로 나가지 않도록 설정
					enemy->col = 15;
			}
		}
	}
	enemy->clock_before = clk_pulse;
}


void LCD_Display_Enemy(Enemy enemy)
{
	lcd_put_cur(enemy.row, enemy.col);
	lcd_send_data(enemy.image_num);
}


Game_status GameStatus(Character *character, Enemy *enemy)
{
	uint8_t cnt = 0;
	for(int i = 0 ; i <= 1 ; i++)
	{
		for(int j = 0 ; j < 16 ; j++)
		{
			if(character->past_position[i][j] == 1)
				cnt++;
		}
	}
	if(character->row == enemy->row && character->col == enemy->col)
	{
		LoseSound();
		return OVER;
	}
	else if(cnt == 32 && level == 1)
	{
		WinSound();
		lcd_clear();
		lcd_put_cur(0, 1);
		lcd_send_string("Level 2");
		HAL_Delay(500);
		lcd_put_cur(1, 9);
		lcd_send_string("Start!");
		HAL_Delay(800);
		LevelupInit(character, enemy);
		TIM2->PSC = 8750; // 문어 속도 Lv1보다 빠르게
//		TIM2->PSC = 5000; // 문어 속도 Lv1보다 빠르게
		return game_status;
	}
	else if(cnt == 32 && level == 2)
	{
		WinSound();
		lcd_clear();
		lcd_put_cur(0, 1);
		lcd_send_string("Level 3");
		HAL_Delay(500);
		lcd_put_cur(1, 9);
		lcd_send_string("Start!");
		HAL_Delay(800);
		LevelupInit(character, enemy);
		TIM2->PSC = 6500; // 문어 속도 Lv2보다 빠르게
//		TIM2->PSC = 3500; // 문어 속도 Lv2보다 빠르게
		return game_status;
	}
	else if(cnt == 32 && level == 3)
	{
		WinSound();
		return WIN;
	}
	else return game_status;
}


void LevelupInit(Character *character, Enemy *enemy)
{
	level++; // enum type의 장점, 문자에 각 번호가 할당되어 연산식 사용이 편하다.
	character->row = 0;
	character->col = 0;
	character->food = 31;
	for(int i = 0 ; i <= 1 ; i++)
	{
		for(int j = 0 ; j < 16 ; j++)
		{
			character->past_position[i][j] = 0;
		}
	}
	enemy->row = 1;
	enemy->col = 8;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	clk_pulse ^= 0x01;
}

void StartSound()
{
	TIM3->ARR = 156;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->ARR = 111;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(100);

	// Setting for food eating Sound
	TIM3->ARR = 1060;
	TIM3->CCR1 = 0;
}


void LoseSound()
{
	TIM3->ARR = 290;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(80);

	TIM3->ARR = 391;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(80);

	TIM3->ARR = 290;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(80);

	TIM3->ARR = 391;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(80);
}


void WinSound()
{
	TIM3->ARR = 593;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(100);

	TIM3->CCR1 = 0;
	HAL_Delay(10);

	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(100);

	TIM3->CCR1 = 0;
	HAL_Delay(10);

	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(100);

	TIM3->CCR1 = 0;
	HAL_Delay(10);

	TIM3->ARR = 767;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(300);

	TIM3->ARR = 593;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(300);

	TIM3->ARR = 508;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(300);

	TIM3->ARR = 1029;
	TIM3->CCR1 = TIM3->ARR / 2;
	HAL_Delay(300);
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, dir, 2);

  // LCD 초기화 함수 호출
    lcd_init();

    // Pacman 이미지 데이터
    char pac1[] = {0x07,  0x0F,  0x1C,  0x18,  0x18,  0x1C,  0x0F,  0x07}; // 우 벌
    char pac2[] = {0x00,  0x0F,  0x1F,  0x18,  0x1C,  0x1F,  0x0F,  0x00}; // 우 닫
    char pac3[] = {0x1C,  0x1E,  0x07,  0x03,  0x03,  0x07,  0x1E,  0x1C}; // 좌 벌
    char pac4[] = {0x00,  0x1E,  0x1F,  0x03,  0x07,  0x1F,  0x1E,  0x00}; // 좌 닫
    char enemy[] = {0x0E,  0x1F,  0x15,  0x1F,  0x0E,  0x15,  0x15,  0x15}; // 문어

    // Pacman 이미지를 LCD의 DDRAM에 저장
      lcd_send_cmd(0x40); // LCD 화면의 DDRAM 주소를 설정하여 화면의 원하는 위치에 출력, DDRAM Address 2열 1번의 주소가 0x40
      for(int i = 0 ; i < 8 ; i++)
    	  lcd_send_data(pac1[i]);

      lcd_send_cmd(0x40+8); // 8bit씩이니까 2번은 0x40 + 8
      for(int i = 0 ; i < 8 ; i++)
    	  lcd_send_data(pac2[i]);

      lcd_send_cmd(0x40+16);
      for(int i = 0 ; i < 8 ; i++)
    	  lcd_send_data(pac3[i]);

      lcd_send_cmd(0x40+24);
      for(int i = 0 ; i < 8 ; i++)
    	  lcd_send_data(pac4[i]);

      lcd_send_cmd(0x40+32);
      for(int i = 0 ; i < 8 ; i++)
    	  lcd_send_data(enemy[i]);
      // 위 코드로 인해 pac1, pac2, pac3, pac4, enemy의 데이터가 LCD 화면에 순서대로 출력된다.
      // lcd_send_cmd()를 사용하여 DDRAM 주소를 설정
      // lcd_send_data()를 사용하여 데이터 출력하면 LCD 화면에 그래픽이 표시된다.


      // 시작 화면
      lcd_put_cur(0, 0);
      lcd_send_string("Press the Button");

      lcd_put_cur(1, 4);
      lcd_send_string("to Start");


      // 시작 버튼 누를 때까지 대기
      while(HAL_GPIO_ReadPin(Joystick_Button_GPIO_Port, Joystick_Button_Pin))
    	  HAL_Delay(100); // 눌리면, 100ms 뒤 시작

      HAL_TIM_Base_Start_IT(&htim2); // TIM2를 인터럽트로 사용
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Sound를 위한 PWM

      StartSound();

      lcd_clear();
      lcd_put_cur(0, 1);
      lcd_send_string("LEVEL 1");
      HAL_Delay(500);

      lcd_put_cur(1, 9);
      lcd_send_string("Start!");
      HAL_Delay(800);


      // Pacman Init
      Character pacman;
      memset(&pacman, 0, sizeof(pacman)); // pacman 구조체를 0으로 초기화한다(모든 멤버 변수를 0으로 설정)
      pacman.food = 31;

      // Enemy Init
      Enemy octopus;
      memset(&octopus, 0, sizeof(octopus)); // octopus 구조체를 0으로 초기화한다(모든 멤버 변수를 0으로 설정)
      octopus.image_num = 4;
      octopus.row = 1; // 처음 시작 위치
      octopus.col = 8;


      lcd_clear();
      lcd_put_cur(pacman.row, pacman.col);
      lcd_send_data(pacman.image_num);
    //  TIM2->PSC = 8000; // 문어 속도를 좀 더 빠르게 설정

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(game_status == ING)
	{
		Move_Pacman(&pacman, Dir_Joystick());
		Move_Enemy(&octopus, pacman, clk_pulse);
  //		LCD_Display_Charactor(&pacman);
  //		LCD_Display_Enemy(octopus);

		game_status = GameStatus(&pacman, &octopus);

  //		HAL_Delay(100);
		HAL_Delay(50);
		LCD_Display_Charactor(&pacman);
		LCD_Display_Enemy(octopus);
		TIM3->CCR1 = 0;
	}
	else if(game_status == WIN)
	{
		lcd_put_cur(0, 4);
		lcd_send_string("YOU WIN");
		lcd_put_cur(1, 0);
		lcd_send_string("Congratulations!");
	}
	else if(game_status == OVER)
	{
		lcd_put_cur(0, 4);
		lcd_send_string("YOU LOSE");
	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 230-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Joystick_Button_Pin */
  GPIO_InitStruct.Pin = Joystick_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Joystick_Button_GPIO_Port, &GPIO_InitStruct);

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
