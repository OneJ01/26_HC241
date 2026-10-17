/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ELEV_MAX_PULSE      950

#define HOME_BACKOFF_STEPS  300
#define HOME_SEARCH_STEPS   1500

#define HOME_EXTRA_BACKOFF_STEPS  300

#define HOME_PHASE_IDLE           0
#define HOME_PHASE_BACKOFF        1
#define HOME_PHASE_EXTRA_BACKOFF  2
#define HOME_PHASE_SEARCH         3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t fire_flag = 0;

volatile int32_t m0_pos = 0;
volatile uint8_t m0_homed = 0;
volatile uint8_t m0_homing = 0;
volatile uint8_t m0_home_phase = HOME_PHASE_IDLE;

volatile uint8_t home_done_flag = 0;
volatile uint8_t home_fail_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

typedef struct {
  volatile uint32_t target_steps;
  volatile uint32_t done_steps;
  volatile uint8_t  running;
  volatile int8_t   dir_sign;   // +1: POS 증가(M0-), -1: POS 감소(M0+)
} StepperState;

static StepperState s0 = {0}, s1 = {0};

static void motor0_start(uint32_t steps, uint8_t dir);

/* -----------------------------------------------------------------------
 * UART 송신 헬퍼
 * ACK/NAK/ERR 응답을 ROS2 로 전송.
 * HAL_UART_Transmit 블로킹(timeout 10ms) — 응답 문자열은 짧으므로 충분.
 * ----------------------------------------------------------------------- */
static void uart_send(const char *msg)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)msg,
                      (uint16_t)strlen(msg),
                      10);
}

static uint8_t home45_read(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) ? 1 : 0;
}

static uint8_t inp_read(void)
{
    /*
     * CS-D508 ALM output = In Position
     * ALM- = GND
     * ALM+ = PC8(ALM_Pin)
     * GPIO internal pull-up 사용
     *
     * 예상 논리:
     *   In Position 활성   → ALM 출력 ON  → GPIO LOW  → INP=1
     *   In Position 비활성 → ALM 출력 OFF → GPIO HIGH → INP=0
     */
    return (HAL_GPIO_ReadPin(ALM_GPIO_Port, ALM_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

static void home_start_backoff(void)
{
    /*
     * M0- : 랙 길이 증가, 홈에서 벗어나는 방향
     */
    m0_homing = 1;
    m0_home_phase = HOME_PHASE_BACKOFF;
    motor0_start(HOME_BACKOFF_STEPS, 0);  // dir=0 → M0-
}

static void home_start_search(void)
{
    /*
     * M0+ : 랙 길이 감소, 홈으로 접근하는 방향
     */
    m0_homing = 1;
    m0_home_phase = HOME_PHASE_SEARCH;
    motor0_start(HOME_SEARCH_STEPS, 1);   // dir=1 → M0+
}

static void home_start_extra_backoff(void)
{
    /*
     * HOME45=0이 된 뒤에도 M0- 방향으로 조금 더 이동해서
     * 센서 감지 구간을 확실히 벗어난다.
     */
    m0_homing = 1;
    m0_home_phase = HOME_PHASE_EXTRA_BACKOFF;
    motor0_start(HOME_EXTRA_BACKOFF_STEPS, 0);  // dir=0 → M0-
}

static void status_send(void)
{
    char buf[220];

    snprintf(buf, sizeof(buf),
             "POS=%ld HOMED=%u HOMING=%u PHASE=%u M0RUN=%u HOME45=%u INP=%u TARGET=%lu DONE=%lu MAX=%ld\r\n",
             (long)m0_pos,
             (unsigned int)m0_homed,
             (unsigned int)m0_homing,
             (unsigned int)m0_home_phase,
             (unsigned int)s0.running,
             (unsigned int)home45_read(),
             (unsigned int)inp_read(),
             (unsigned long)s0.target_steps,
             (unsigned long)s0.done_steps,
             (long)ELEV_MAX_PULSE);

    uart_send(buf);
}

static void motor0_start(uint32_t steps, uint8_t dir)
{
  s0.target_steps = steps;
  s0.done_steps   = 0;
  s0.running      = 1;
  s0.dir_sign = dir ? -1 : 1;

  /* DIR (dir=1 정방향, 0 역방향) */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* ENA 활성화 (LOW = enable) */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
}

static void motor1_start(uint32_t steps, uint8_t dir)
{
  s1.target_steps = steps;
  s1.done_steps   = 0;
  s1.running      = 1;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
}

static void motor0_stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

  /*
   * 일반 M0 이동일 때만 POS 갱신.
   * HOME0 동작 중에는 센서 감지 후 POS=0으로 직접 설정하므로
   * 여기서 done_steps를 POS에 반영하지 않는다.
   */
  if (s0.running && !m0_homing) {
    if (s0.dir_sign > 0) {
      m0_pos += (int32_t)s0.done_steps;
    } else {
      m0_pos -= (int32_t)s0.done_steps;
    }
  }

  s0.running = 0;
}

static void motor1_stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  s1.running = 0;
}

static uint8_t rx_ch;
static char rx_line[32];
static uint8_t rx_pos = 0;

static void uart_rx_line_reset(void)
{
  rx_pos = 0;
  memset(rx_line, 0, sizeof(rx_line));
}

/* -----------------------------------------------------------------------
 * handle_line : 수신된 한 줄 명령 처리 + ACK/NAK 응답
 *
 * 응답 형식 (ROS2 turret_control_node 와 약속):
 *   ACK M0\n   — M0 명령 정상 처리 시작
 *   ACK M1\n   — M1 명령 정상 처리 시작
 *   ACK STOP\n — STOP 정상 처리
 *   ACK FIRE\n — FIRE 정상 수신 (솔레노이드 구동은 main loop에서)
 *   NAK BUSY\n — 해당 모터가 이미 동작 중
 *   NAK FORMAT\n — 명령 형식 오류 (파싱 불가)
 * ----------------------------------------------------------------------- */
static void handle_line(const char *line)
{
  /* SYNC / FLUSH */
  if (strcmp(line, "SYNC") == 0 || strcmp(line, "FLUSH") == 0) {
	uart_rx_line_reset();
	uart_send("ACK SYNC\n");
	return;
  }

  /* STATUS */
  if (strcmp(line, "STATUS") == 0) {
	status_send();
	return;
  }

  /* ── HOME0 ───────────────────────────────────────────────────────── */
  if (strcmp(line, "HOME0") == 0) {
    if (s0.running) {
      uart_send("NAK BUSY\n");
      return;
    }

    /*
     * HOME0 시작:
     * - 현재 HOME45=1이면 먼저 M0- 방향으로 살짝 빠져서 HOME45=0을 만든다.
     * - 현재 HOME45=0이면 바로 M0+ 방향으로 접근한다.
     */
    m0_homed = 0;
    m0_homing = 1;
    m0_home_phase = HOME_PHASE_IDLE;
    home_done_flag = 0;
    home_fail_flag = 0;

    if (home45_read()) {
      home_start_backoff();
    } else {
      home_start_search();
    }

    uart_send("ACK HOME0\n");
    return;
  }

  /* ── STOP ─────────────────────────────────────────────────────────── */
  if (strcmp(line, "STOP") == 0) {
    motor0_stop();
    motor1_stop();

    if (m0_homing) {
      m0_homing = 0;
      m0_home_phase = HOME_PHASE_IDLE;
      m0_homed = 0;
    }

    uart_send("ACK STOP\n");
    return;
  }

  /* ── FIRE ─────────────────────────────────────────────────────────── */
  if (strcmp(line, "FIRE") == 0) {
    fire_flag = 1;
    uart_send("ACK FIRE\n");
    return;
  }

  /* ── M0 / M1 ──────────────────────────────────────────────────────── */
  if (line[0] == 'M' && (line[1] == '0' || line[1] == '1')) {

    /* 부호 문자 확인 — '+' 또는 '-' 없으면 형식 오류 */
    if (line[2] != '+' && line[2] != '-') {
      uart_send("NAK FORMAT\n");
      return;
    }

    int idx  = line[1] - '0';
    int val  = atoi(line + 2);   /* "+1000" / "-250" 파싱 */

    uint8_t  dir   = (val >= 0) ? 1 : 0;
    uint32_t steps = (val >= 0) ? (uint32_t)val : (uint32_t)(-val);

    if (idx == 0) {
      if (s0.running) {
        uart_send("NAK BUSY\n");
        return;
      }

      /* 소프트웨어 리밋 체크 */
      if (steps > 0) {
        int32_t next_pos;

        if (dir) {
          /* M0+ : 랙 길이 감소, 홈 방향, POS 감소 */
          next_pos = m0_pos - (int32_t)steps;
        } else {
          /* M0- : 랙 길이 증가, 홈에서 멀어지는 방향, POS 증가 */
          next_pos = m0_pos + (int32_t)steps;
        }

        if (next_pos < 0) {
          uart_send("NAK LIMIT_MIN\n");
          return;
        }

        if (next_pos > ELEV_MAX_PULSE) {
          uart_send("NAK LIMIT_MAX\n");
          return;
        }
      }

      /* steps=0 이면 이동 불필요 — ACK만 반환 */
      if (steps > 0) motor0_start(steps, dir);
      uart_send("ACK M0\n");

    } else {  /* idx == 1 */
      if (s1.running) {
        uart_send("NAK BUSY\n");
        return;
      }
      if (steps > 0) motor1_start(steps, dir);
      uart_send("ACK M1\n");
    }
    return;
  }

  /* ── 알 수 없는 명령 ─────────────────────────────────────────────── */
  uart_send("NAK FORMAT\n");
}

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);

  /* ENA 초기 비활성화 (HIGH = disable) */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_SET);
  /* DIR 초기값 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);

  uart_rx_line_reset();
  HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
  uart_send("BOOT OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (fire_flag) {
      fire_flag = 0;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if (home_done_flag) {
      home_done_flag = 0;
      uart_send("DONE HOME0\n");
    }

    if (home_fail_flag) {
      home_fail_flag = 0;
      uart_send("ERR HOME0\n");
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : HOME45_Pin */
  GPIO_InitStruct.Pin = HOME45_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HOME45_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ALM_Pin */
  GPIO_InitStruct.Pin = ALM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ALM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;

  if (s0.running) {

    if (m0_homing) {

      /*
       * HOME0 1단계: BACKOFF
       * HOME45=1 상태에서 M0- 방향으로 살짝 빠져나감.
       * HOME45가 0이 되는 순간 멈추고 SEARCH 단계로 전환.
       */
        if (m0_home_phase == HOME_PHASE_BACKOFF) {

          if (!home45_read()) {
            /*
             * HOME45가 0이 된 순간 바로 SEARCH로 가지 않고,
             * 센서 감지 구간을 확실히 벗어나기 위해 추가 backoff 수행.
             */
            motor0_stop();
            home_start_extra_backoff();
          } else {
            s0.done_steps++;

            if (s0.done_steps >= s0.target_steps) {
              motor0_stop();

              m0_homing = 0;
              m0_home_phase = HOME_PHASE_IDLE;
              m0_homed = 0;

              home_fail_flag = 1;
            }
          }
        }
        /*
         * HOME0 1.5단계: EXTRA_BACKOFF
         * HOME45=0이 된 뒤에도 M0- 방향으로 정해진 pulse만큼 더 이동.
         * 이후 M0+ 방향 SEARCH 시작.
         */
        else if (m0_home_phase == HOME_PHASE_EXTRA_BACKOFF) {

          s0.done_steps++;

          if (s0.done_steps >= s0.target_steps) {
            motor0_stop();
            home_start_search();
          }
        }
      /*
       * HOME0 2단계: SEARCH
       * M0+ 방향으로 다시 홈에 접근.
       * HOME45가 1이 되는 첫 순간 정지하고 POS=0 처리.
       */
      else if (m0_home_phase == HOME_PHASE_SEARCH) {

        if (home45_read()) {
          motor0_stop();

          m0_pos = 0;
          m0_homed = 1;
          m0_homing = 0;
          m0_home_phase = HOME_PHASE_IDLE;

          home_done_flag = 1;
        } else {
          s0.done_steps++;

          if (s0.done_steps >= s0.target_steps) {
            motor0_stop();

            m0_homing = 0;
            m0_home_phase = HOME_PHASE_IDLE;
            m0_homed = 0;

            home_fail_flag = 1;
          }
        }
      }

    } else {
      /*
       * 일반 M0 이동
       */
      s0.done_steps++;

      if (s0.done_steps >= s0.target_steps) {
        motor0_stop();
      }
    }
  }

  if (s1.running) {
    s1.done_steps++;
    if (s1.done_steps >= s1.target_steps) {
      motor1_stop();
    }
  }

  /* 둘 다 멈췄으면 타이머 인터럽트도 끄기 (불필요한 인터럽트 방지) */
  if (!s0.running && !s1.running) {
    HAL_TIM_Base_Stop_IT(&htim2);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;

  char c = (char)rx_ch;

  if (c == '\r' || c == '\n') {
    rx_line[rx_pos] = '\0';

    if (rx_pos > 0) {
      handle_line(rx_line);
    }

    rx_pos = 0;
  } else {
    if (rx_pos < sizeof(rx_line) - 1) {
      rx_line[rx_pos++] = c;
    } else {
      rx_pos = 0;   /* 버퍼 오버플로 → 라인 리셋 */
    }
  }

  HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
