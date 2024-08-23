/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
// CAN通信
// https://blog.csdn.net/prolop87/article/details/122671441

// 串口模拟
// https://blog.csdn.net/Hard_Yao/article/details/128516418
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "leaf_ota.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TCA9535_ADDRESS         (0x20)         // TCA9535的I2C地址
#define TCA9535_INPUT_PORT0_REG (0x00) // 输入端口0的寄存器地址
#define TCA9535_CONFIG_REG      (0x06)

#define CAN_RxExtId 0x17532F75

#define false 0
#define true 1

//=============================================================================
// 数据帧类型
//=============================================================================
#define CMD_OPEN_DOOR 0xA0
#define CMD_CLOSE_DOOR 0xA1
#define CMD_ALARM 0xA2
#define CMD_GET_DOOR_STATE 0xA3

#define CMD_TURN_ON 0xB1
#define CMD_TURN_OFF 0xB2

#define CMD_OTA_VERSION 0xC0
#define CMD_OTA_REQUEST 0xC1
#define CMD_OTA_DATA 0xC2
#define CMD_OTA_END 0xC3
#define CMD_OTA_RESET 0xC4

static uint16_t version = 0x0100; // 1.0
static uint16_t ota_file_size = 0;
static uint8_t ota_file_crc = 0;
static uint16_t firmwareSize = 0;
static uint16_t firmwareBatch = 0;
uint8_t firmwareBuffer[1024+8];

static int bOTA = 0;

int bPowerOff = 0;
int bPowerOff_count = 12;


int bTurnOff = 0;
int bTurnOff_count = 60;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

typedef struct
{
  uint8_t dir;
  uint8_t pwm;
  GPIO_TypeDef *GPIOx1;
  uint16_t GPIO_Pin1;
  GPIO_TypeDef *GPIOx2;
  uint16_t GPIO_Pin2;
} Motor_DataDef;

Motor_DataDef motor_pwm[8] = {
    {0, 0, MOTOR_IN1_0_GPIO_Port, MOTOR_IN1_0_Pin, MOTOR_IN2_0_GPIO_Port, MOTOR_IN2_0_Pin},
    {0, 0, MOTOR_IN1_1_GPIO_Port, MOTOR_IN1_1_Pin, MOTOR_IN2_1_GPIO_Port, MOTOR_IN2_1_Pin},
    {0, 0, MOTOR_IN1_2_GPIO_Port, MOTOR_IN1_2_Pin, MOTOR_IN2_2_GPIO_Port, MOTOR_IN2_2_Pin},
    {0, 0, MOTOR_IN1_3_GPIO_Port, MOTOR_IN1_3_Pin, MOTOR_IN2_3_GPIO_Port, MOTOR_IN2_3_Pin},
    {0, 0, MOTOR_IN1_4_GPIO_Port, MOTOR_IN1_4_Pin, MOTOR_IN2_4_GPIO_Port, MOTOR_IN2_4_Pin},
    {0, 0, MOTOR_IN1_5_GPIO_Port, MOTOR_IN1_5_Pin, MOTOR_IN2_5_GPIO_Port, MOTOR_IN2_5_Pin},
    {0, 0, MOTOR_IN1_6_GPIO_Port, MOTOR_IN1_6_Pin, MOTOR_IN2_6_GPIO_Port, MOTOR_IN2_6_Pin},
    {0, 0, MOTOR_IN1_7_GPIO_Port, MOTOR_IN1_7_Pin, MOTOR_IN2_7_GPIO_Port, MOTOR_IN2_7_Pin}};

uint16_t tm3_count = 0;
uint16_t tm4_count = 0;

uint16_t lastInuput = 0xFFFF; // 用于存储TCA9535的输入状态

// 当前传感器的状态
uint16_t cur_input = 0;
uint8_t bSendTCA = 0;

// CAN 通讯
uint8_t Rx_Flag;
uint8_t Tx_Flag;
uint8_t RxBuf[8];
uint8_t TxBuf[8];
uint8_t key;

CAN_TxHeaderTypeDef TxHeader; // 发送
CAN_RxHeaderTypeDef RxHeader; // 接收

uint8_t RxData[8]; // 数据接收数组，can的数据帧只有8帧

uint32_t CAN_TxExtId = 0x1800D0D8; // 发送ID

// KEY
volatile uint32_t keyPressTime = 0; // 记录按键按下时间
volatile enum { IDLE,
                LONG_PRESS_START,
                TURN_ON,
                TURN_OFF } keyState = IDLE,
                           runState = TURN_OFF; // 状态机

// ADC_Values
uint16_t ADC_Values[5];

// send adc data
uint8_t bSendAdc = 0;

/* 定义接收缓冲区 */
#define RXBUFFERSIZE 50
// uint8_t buffer1[RXBUFFERSIZE] = {0}; // USART1存储接收数据
// uint8_t UART1_Rx_flg = 0;            // USART1接收完成标志
// uint8_t sofar1 = 0;                  // USART1接受数据计数
uint8_t UART1_temp[1] = {0}; // USART1接收数据缓存

uint8_t buffer3[RXBUFFERSIZE] = {0}; // USART3存储接收数据
uint8_t UART3_Rx_flg = 0;            // USART3接收完成标志
uint8_t sofar3 = 0;                  // USART3接受数据计数
uint8_t UART3_temp[1] = {0};         // USART3接收数据缓存

// 1s timer
uint8_t bTimer1S = 0;

// 4s timer
uint8_t bTimer4S = 0;

uint8_t bAlarm = 0;
uint8_t alarm_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t GetChipID(void);

uint16_t ReadTCA9535Inputs(void);
void timer1s(void);

void CAN_Filter_Init(void); // 过滤器初始化
uint8_t CAN_Send_Msg(uint8_t *msg, uint8_t len);

void Key_Scan(void);

// ADC Read
uint16_t ADC_Read(uint32_t Channel);

void Send_TCA(void);
void Send_ADC(void);

int send_cmd(uint8_t *cmd, uint8_t len);

void Report_State(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================== RS485 =========================================
/*****************************************************************************
函数名称 : CheckSum
功能描述 : 计算Checksum
输入参数 : uBuff:开始指针
          uBuffLen:长度

返回参数 : 所有的数值相加,取低8位,取反加1
*****************************************************************************/
unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
{
  unsigned char i, uSum = 0;
  for (i = 0; i < uBuffLen; i++)
  {
    uSum = uSum + uBuff[i];
  }
  uSum = (~uSum) + 1;
  return uSum;
}


unsigned char CheckSumFlash(uint32_t start_addr, uint16_t file_size)
{
  unsigned char uSum = 0;
  uint16_t batch = 0;
  uint16_t i = 0;

  for (batch = 0; batch < file_size / 256; batch++)
  {
    // Read 1024 bytes from flash
    STMFLASH_Read((start_addr + batch * 256), (uint32_t *) &firmwareBuffer[0], 64);
    for (i = 0; i < 256; i++)
    {
      uSum = uSum + firmwareBuffer[i];
    }
  }

  // read the remaining bytes
  STMFLASH_Read((start_addr + batch * 256), (uint32_t *) &firmwareBuffer[0], file_size % 256);

  for (i = 0; i < file_size % 256; i++)
  {
    uSum = uSum + firmwareBuffer[i];
  }

  uSum = (~uSum) + 1;

  return uSum;
}

void open_door(uint8_t door)
{
  if (cur_input |= (1 << door * 2))
  {
    printf("Door %d is already opened !\r\n", door);
    return;
  }
  else
  {
    printf("Open Door %d !\r\n", door);
    motor_pwm[door].dir = 1;
    motor_pwm[door].pwm = 50;
  }
}

void close_door(uint8_t door)
{
  printf("Close Door %d !\r\n", door);
  if (cur_input &= (1 << door * 2))
  {
    printf("Door %d is already closed !\r\n", door);
    return;
  }
  else
  {
    motor_pwm[door].dir = 0;
    motor_pwm[door].pwm = 50;
  }
}

unsigned char get_door_state(void)
{
  unsigned char door_state = 0;
  for (int i = 0; i < 8; i++)
  {
    if (cur_input & (1 << i * 2))
    {
      printf("Door %d is opened !\r\n", i);
      door_state |= (1 << i);
    }
  }

  return door_state;
}

// ============================== CAN =========================================
/*****************************************************************************
函数名称 : can_process
功能描述 : 收数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 在MCU串口接收函数中调用该函数,并将接收到的数据作为参数传入
*****************************************************************************/

void dump_data(void)
{
  uint32_t * temp;

  temp = (uint32_t *) &firmwareBuffer[0];

  // dump the flash data from APP_NEW_FW_START_ADR
  for (int bank=0; bank < 1; bank++)
  {
    printf("Read Flash Data %x !\r\n", APP_NEW_FW_START_ADR + bank * 1024);

    STMFLASH_Read((APP_NEW_FW_START_ADR + bank * 1024), temp, 256);
    for (int i = 0; i < 256; i++)
    {
      printf("%08x ", temp[i]);
    }
    printf("\r\n");
  }

  for (int bank = 18; bank < 19; bank++)
  {
    printf("Read Flash Data %x !\r\n", APP_NEW_FW_START_ADR + bank * 1024);

    STMFLASH_Read((APP_NEW_FW_START_ADR + bank * 1024), temp, 256);
    for (int i = 0; i < 256; i++)
    {
      printf("%08x ", temp[i]);
    }
    printf("\r\n");
  }

}


uint8_t convert_vol(uint16_t adc, float scale)
{
  float voltage = adc / 4096.0 * 3.4 * 16;
  // printf("ADC = %d, Voltage = %f \r\n", adc, voltage);
  return (uint8_t) ((voltage - 25.0) / scale);
}


uint8_t convert_adc(uint16_t adc, float scale)
{
  float voltage = adc / 4096.0 * 3.4 * 1.36;
  float current = (voltage - 2.5) / scale ; // 20A 量程

  // printf("ADC = %d, Voltage = %f Current = %f\r\n", adc, voltage, current);

  return (uint8_t) (current / 0.2 + 128);
}


void can_process()
{
  // printf("Receive CAN Succeed !\r\n");
  unsigned char data = RxBuf[1];
  // uint16_t input;

  switch (RxBuf[0])
  {
  case CMD_OPEN_DOOR: // OPEN DOOR
    printf("Open Door !\r\n");

    // 从data最低位开始，如果这位的数值是1，就打开对应的门
    for (int i = 0; i < 8; i++)
    {
      if ((data >> i) & 0x01)
      {
        printf("Open Door %d !\r\n", i);
        open_door(i);
      }
    }

    // Send ACK
    TxBuf[0] = CMD_OPEN_DOOR;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

    // 清空接收、发送数组，保留Rxbuf内容
    memset(TxBuf, 0, sizeof(TxBuf));
    break;

  case CMD_CLOSE_DOOR:
    printf("Close Door !\r\n");
    // 从data最低位开始，如果这位的数值是1，就打开对应的门
    for (int i = 0; i < 8; i++)
    {
      if ((data >> i) & 0x01)
      {
        printf("Close Door %d !\r\n", i);
        close_door(i);
      }
    }

    // Send ACK
    TxBuf[0] = CMD_CLOSE_DOOR;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

    break;

  case CMD_ALARM:
    printf("Alarm !\r\n");

    if (RxBuf[1] == 0x0) {
      printf("Alarm Off !\r\n");
      bAlarm = 0;
      HAL_GPIO_WritePin(PWR_5V_EN_GPIO_Port, PWR_5V_EN_Pin, GPIO_PIN_SET);
    } else {
      bAlarm = 1;
      alarm_count = RxBuf[1];
      HAL_GPIO_WritePin(PWR_5V_EN_GPIO_Port, PWR_5V_EN_Pin, GPIO_PIN_RESET);
      printf("Alarm On !\r\n");
    }

    TxBuf[0] = CMD_ALARM;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

    break;

  case CMD_GET_DOOR_STATE:
    printf("Get Door State !\r\n");

    // ADC: 2176 2499 2460 2520 2510

    ADC_Values[0] = ADC_Read(ADC_CHANNEL_10); // ADC SYSTEM VOLTAGE
    ADC_Values[1] = ADC_Read(ADC_CHANNEL_11); // ADC SYSTEM CURRENT
    ADC_Values[2] = ADC_Read(ADC_CHANNEL_12); // ADC 12V CURRENT
    ADC_Values[3] = ADC_Read(ADC_CHANNEL_13); // ADC VM CURRENT
    ADC_Values[4] = ADC_Read(ADC_CHANNEL_8);  // ADC VBAT CHARGE CURRENT

    // 门状态
    TxBuf[0] = CMD_GET_DOOR_STATE;
    TxBuf[1] = get_door_state();
    // 电量  ADC_Values[0]
    TxBuf[2] = convert_vol(ADC_Values[0], 0.05);
    // 电流     ADC_Values[1]
    TxBuf[3] = convert_adc(ADC_Values[1], 0.067);
    // 12V电流   ADC_Values[2]
    TxBuf[4] = convert_adc(ADC_Values[2], 0.1);
    // VM电流    ADC_Values[3]
    TxBuf[5] = convert_adc(ADC_Values[3], 0.1);
    // 这个从配置中获取
    TxBuf[6] = dev.charge_flag & 0xFF;
    // 故障码
    TxBuf[7] = 0x00;

    Tx_Flag = CAN_Send_Msg(TxBuf, 8);
    break;

  case CMD_TURN_ON:
    printf("Turn On !\r\n");
    // enable PWR_12V_EN_Pin
    HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_SET);
    // Send ACK
    TxBuf[0] = CMD_TURN_ON;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

    break;

  case CMD_TURN_OFF:
    printf("Turn Off !\r\n");
    bPowerOff = 1;
    bPowerOff_count = 12;
    // Send ACK
    TxBuf[0] = CMD_TURN_OFF;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

    break;

  // OTA proces
  case CMD_OTA_VERSION:
    printf("OTA Version : %x !\r\n", version);
    TxBuf[0] = CMD_OTA_VERSION;
    TxBuf[1] = (version & 0xFF00 ) >> 8;
    TxBuf[2] = version & 0xFF;
    Tx_Flag = CAN_Send_Msg(TxBuf, 3);
    break;

  case CMD_OTA_REQUEST:

    ota_file_size = (RxBuf[1] << 8) | RxBuf[2];
    ota_file_crc = RxBuf[3];

    printf("OTA Request %d, %d !\r\n", ota_file_size, ota_file_crc);

    TxBuf[0] = CMD_OTA_REQUEST;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

	  Erase_page(APP_NEW_FW_START_ADR);
    printf("Flash Erase Finished !\r\n");

    firmwareBatch = 0;
    firmwareSize = 0;
    bOTA = 1;
    break;

  case CMD_OTA_DATA:
    for (int i = 1; i < RxHeader.DLC; i++)
    {
      firmwareBuffer[firmwareSize++] = RxData[i];
    }

    if (firmwareSize >= 256)
    {
      //Write firmwareBuffer to flash
      STMFLASH_Write((APP_NEW_FW_START_ADR + firmwareBatch * 256), (uint32_t *) &firmwareBuffer[0], 64);
      printf("OTA Data Receive batch %d !\r\n", firmwareBatch++);

      for (int i = 256; i < firmwareSize; i++)
      {
        firmwareBuffer[i-256] = firmwareBuffer[i];
      }
      firmwareSize -= 256;

      TxBuf[0] = CMD_OTA_DATA;
      TxBuf[1] = 0x10;
      Tx_Flag = CAN_Send_Msg(TxBuf, 2);
    }

    if (firmwareSize + 256 * firmwareBatch >= ota_file_size)
    {
      //Write firmwareBuffer to flash
      STMFLASH_Write((APP_NEW_FW_START_ADR + firmwareBatch * 256), (uint32_t *) &firmwareBuffer[0], firmwareSize/4);
      printf("Success:  OTA Data Receive Finished! Remain = %d \r\n", firmwareSize);
      TxBuf[0] = CMD_OTA_DATA;
      TxBuf[1] = 0x11;
      Tx_Flag = CAN_Send_Msg(TxBuf, 2);

      bOTA = 0;
    }

    break;

  case CMD_OTA_END:
    printf("OTA End, remain firmwareSize = %d, total_size = %d, crc = %d !\r\n", firmwareSize, ota_file_size, ota_file_crc);

    // check the CRC of the firmware
    if (ota_file_crc == CheckSumFlash(APP_NEW_FW_START_ADR, ota_file_size))
    {
      printf("OTA CRC Check Succeed !\r\n");
      // reset the system
      TxBuf[0] = CMD_OTA_END;
      TxBuf[1] = 0x10;
      Tx_Flag = CAN_Send_Msg(TxBuf, 2);
    }
    else
    {
      printf("OTA CRC Check Failed !\r\n");
      TxBuf[0] = CMD_OTA_END;
      TxBuf[1] = 0x61;
      Tx_Flag = CAN_Send_Msg(TxBuf, 2);
    }

    bOTA = 0;

    break;

  case CMD_OTA_RESET:
    printf("OTA Reset !\r\n");

    dev.backup_flag = Startup_Update;
    version = (RxBuf[1] << 8) | RxBuf[2];
    dev.version = version;

    write_boot_config();
    printf("BOOT FLAG Writed !\r\n");

    TxBuf[0] = CMD_OTA_RESET;
    TxBuf[1] = 0x10;
    Tx_Flag = CAN_Send_Msg(TxBuf, 2);

	  // restart the system
    HAL_NVIC_SystemReset();

	  break;


  default:
    break;
  }

  // 清空接收、发送数组，保留Rxbuf内容
  memset(RxData, 0, sizeof(RxData));
  memset(TxBuf, 0, sizeof(TxBuf));
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__set_PRIMASK(0);   //  打开全局中断
  SCB->VTOR = 0x8004000U;
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Init();

  // do some test here

  // end of test

  printf("Program Started !\r\n");

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // timer1s
    if (bTimer1S)
    {
      timer1s();
      bTimer1S = 0;
    }

    // timer4s
    if (bTimer4S)
    {
      // check the setting of the pwm signal
      printf("Motor PWM Setting !\r\n");
      bTimer4S = 0;
    }

    // CAN Receive
    if (Rx_Flag) // 如果接收标志位被置1，则开展逻辑判断
    {
      can_process();
      Rx_Flag = 0; // 标志位置0，等待下一次中断
    }

    // ADC
    if (bSendAdc && bOTA == 0)
    {
      Report_State();
      bSendAdc = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  // Start the timer
  // HAL_TIM_Base_Start_IT(&htim3);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  // Start the timer
  // HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */
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
  HAL_UART_Receive_IT(&huart1, UART1_temp, 1);
  /* USER CODE END USART1_Init 2 */
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* 启动中断接收 */
  HAL_UART_Receive_IT(&huart3, UART3_temp, 1);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STB_Pin | PWR_12V_EN_Pin | PWR_5V_EN_Pin | MOTOR_IN2_2_Pin | MOTOR_IN1_2_Pin | MOTOR_IN2_3_Pin | MOTOR_IN1_3_Pin | MOTOR_IN2_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_0_Pin | MOTOR_IN2_0_Pin | MOTOR_IN1_1_Pin | MOTOR_IN2_1_Pin | WDI_Pin | LM5176_EN_Pin | MOTOR_IN2_4_Pin | MOTOR_IN1_4_Pin | MOTOR_IN2_5_Pin | MOTOR_IN1_5_Pin | RS485_EN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin | LM5176_VIN_EN_Pin | RS485_EN_0_Pin | MOTOR_IN2_7_Pin | MOTOR_IN1_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_IN1_6_GPIO_Port, MOTOR_IN1_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STB_Pin PWR_12V_EN_Pin PWR_5V_EN_Pin MOTOR_IN2_2_Pin
                           MOTOR_IN1_2_Pin MOTOR_IN2_3_Pin MOTOR_IN1_3_Pin MOTOR_IN2_6_Pin */
  GPIO_InitStruct.Pin = STB_Pin | PWR_12V_EN_Pin | PWR_5V_EN_Pin | MOTOR_IN2_2_Pin | MOTOR_IN1_2_Pin | MOTOR_IN2_3_Pin | MOTOR_IN1_3_Pin | MOTOR_IN2_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN1_0_Pin MOTOR_IN2_0_Pin MOTOR_IN1_1_Pin MOTOR_IN2_1_Pin
                           WDI_Pin LM5176_EN_Pin MOTOR_IN2_4_Pin MOTOR_IN1_4_Pin
                           MOTOR_IN2_5_Pin MOTOR_IN1_5_Pin RS485_EN_1_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_0_Pin | MOTOR_IN2_0_Pin | MOTOR_IN1_1_Pin | MOTOR_IN2_1_Pin | WDI_Pin | LM5176_EN_Pin | MOTOR_IN2_4_Pin | MOTOR_IN1_4_Pin | MOTOR_IN2_5_Pin | MOTOR_IN1_5_Pin | RS485_EN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEPER_Pin */
  GPIO_InitStruct.Pin = BEEPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BEEPER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LM5176_VIN_EN_Pin RS485_EN_0_Pin MOTOR_IN2_7_Pin
                           MOTOR_IN1_7_Pin */
  GPIO_InitStruct.Pin = LED_Pin | LM5176_VIN_EN_Pin | RS485_EN_0_Pin | MOTOR_IN2_7_Pin | MOTOR_IN1_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_IN1_6_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_IN1_6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // SET BEEPER_Pin to high
  HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
  // enable PWR_12V_EN_Pin
  HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_SET);
  // PWR_5V_EN_Pin
  HAL_GPIO_WritePin(PWR_5V_EN_GPIO_Port, PWR_5V_EN_Pin, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/*
  1KHz,0.01ms interval for TIM4
  1KHz,1ms interval for TIM3
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM3)
  {
    tm3_count++;
    if (tm3_count >= 1000)
      tm3_count = 0;

    if (tm3_count == 0)
    {
      bTimer1S = 1;
    }
  }

  if (htim->Instance == TIM4)
  {
    tm4_count++;

    // 50us 一个tick
    if (tm4_count >= 100)
      tm4_count = 0;

    // if (tm4_count == 0)
    // {
    //   HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_0_Pin, GPIO_PIN_SET);
    // }
    // else
    // {
    //   if (tm4_count == 25)
    //   {
    //     HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_0_Pin, GPIO_PIN_RESET);
    //   }
    // }

    uint16_t pin;
    GPIO_TypeDef *port;

    if (tm4_count == 0)
    {
      /* code */
      for (int i = 0; i < 8; i++)
      {
        Motor_DataDef md = motor_pwm[i];
        if (md.pwm > 0)
        {
          pin = md.dir == 0 ? md.GPIO_Pin1 : md.GPIO_Pin2;
          port = md.dir == 0 ? md.GPIOx1 : md.GPIOx2;
          HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
          // port->ODR |=  pin;
        }
      }
      // HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_0_Pin, GPIO_PIN_SET);
    }
    else
    {
      /* code */
      for (int i = 0; i < 8; i++)
      {
        Motor_DataDef md = motor_pwm[i];
        if (tm4_count == md.pwm)
        {
          pin = md.dir == 0 ? md.GPIO_Pin1 : md.GPIO_Pin2;
          port = md.dir == 0 ? md.GPIOx1 : md.GPIOx2;
          HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
          // port->ODR &= ~pin;
        }
      }

      // single example
      //  if (tm4_count == 25)
      //  {
      //    HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_0_Pin, GPIO_PIN_RESET);
      //  }
    }
  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) /* 检查是否是USART1的回调 */
  {
    // rs485_receive_input(UART1_temp[0]);
    // HAL_UART_Receive_IT(&huart1, UART1_temp, 1);
  }

  if (huart->Instance == USART3) /* 检查是否是USART3的回调 */
  {
    // if (sofar3 < RXBUFFERSIZE - 1)
    //   buffer3[sofar3++] = UART3_temp[0];
    // if (0x0a == UART3_temp[0])
    //   UART3_Rx_flg = 1;
    // else
    //   HAL_UART_Receive_IT(&huart3, UART3_temp, 1);
  }
}

// 初始化TCA9535，配置端口0 和端口1全为输入
void InitTCA9535(void)
{
  uint16_t input = ReadTCA9535Inputs();
  ReadTCA9535Inputs();
  // print input value
  printf("TCA9535 input: 0x%x\n", input);
  if (HAL_I2C_Mem_Write(&hi2c2, TCA9535_ADDRESS << 1, TCA9535_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0xFF, 0xFF}, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

// 读取TCA9535的输入状态
uint16_t ReadTCA9535Inputs(void)
{
  uint8_t data[2];         // 用于存储读取到的两个字节数据
  uint16_t inputState = 0; // 用于存储16位的输入状态

  uint8_t tca9335_config_reg = TCA9535_INPUT_PORT0_REG;

  // 锁定总线，开始传输

  if (HAL_I2C_Master_Transmit(&hi2c2, TCA9535_ADDRESS << 1, &tca9335_config_reg, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    // 发送寄存器地址错误处理
    Error_Handler();
  }

  // 接收两个输入寄存器的数据
  if (HAL_I2C_Master_Receive(&hi2c2, TCA9535_ADDRESS << 1, data, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    // 数据接收错误处理
    Error_Handler();
  }

  if (HAL_I2C_Mem_Read(&hi2c2, TCA9535_ADDRESS << 1, TCA9535_INPUT_PORT0_REG, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    // 数据接收错误处理
    Error_Handler();
  }

  // 合并两个字节为16位数据
  inputState = (data[1] << 8) | data[0];
  // printf("TCA9535 input: 0x%x\n", inputState);

  return inputState;
}

static int time1s_count = 0;

void timer1s(void)
{
  uint16_t input = ReadTCA9535Inputs();

  if (input != lastInuput)
  {
    // find the difference bits between lastInuput and input
    uint16_t diff = lastInuput ^ input;

    // 串口发送对应的bit位的状态
    for (int i = 0; i < 16; i++)
    {
      if (diff & (1 << i))
      {
        // printf("TCA9535 input bit %d: %d\n", i, (input & (1 << i)) >> i);
        if ((input & (1 << i)) >> i)
        {
          // change the motor pwm speed to 0
          motor_pwm[i % 2].pwm = 0;
        }
      }
    }

    cur_input = input;
    bSendTCA = 1;
  }

  time1s_count++;

  if (time1s_count == 5)
  {
    bSendAdc = 1;
    time1s_count = 0;
  }

  // 切断12V 电压
  if (bPowerOff == 1)
  {
    bPowerOff_count--;
    if (bPowerOff_count == 0)
    {
      HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_RESET);
      bPowerOff = 0;
    }
  }

  // Alarm
  if (bAlarm == 1)
  {
    alarm_count--;
    if (alarm_count == 0)
    {
      HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET);
      bAlarm = 0;
    }
  }

  // bTurnOff
  // if (bTurnOff)
  // {
  //   if (bTurnOff_count == 0)
  //   {
  //     printf(" Send Turn off cmd !\r\n");
  //     bTurnOff = 0;
  //     TxBuf[0] = CMD_TURN_OFF;
  //     TxBuf[1] = 0xFF;
  //     Tx_Flag = CAN_Send_Msg(TxBuf, 2);
  //   }
  //   else
  //   {
  //     bTurnOff_count--;
  //   }
  // }

}

int send_cmd(uint8_t *cmd, uint8_t len)
{
  HAL_GPIO_WritePin(RS485_EN_0_GPIO_Port, RS485_EN_0_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart1, cmd, len, 0xffff);
  HAL_GPIO_WritePin(RS485_EN_0_GPIO_Port, RS485_EN_0_Pin, GPIO_PIN_RESET);
  return 0;
}

// 重定义printf和getchar，便于调试
int fputc(int ch, FILE *f)
{
  HAL_GPIO_WritePin(RS485_EN_0_GPIO_Port, RS485_EN_0_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  HAL_GPIO_WritePin(RS485_EN_0_GPIO_Port, RS485_EN_0_Pin, GPIO_PIN_RESET);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

/*CAN接收中断函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanNum)
{
  uint32_t i;

  Rx_Flag = 1; // 接收标志位
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  for (i = 0; i < RxHeader.DLC; i++)
    RxBuf[i] = RxData[i]; // 用RxBuf转存RxData的数据
}

/*CAN发送数据，入口参数为要发送的数组指针，数据长度，返回0代表发送数据无异常，返回1代表传输异常*/
uint8_t CAN_Send_Msg(uint8_t *msg, uint8_t len)
{
  uint8_t i = 0;
  uint32_t TxMailbox;
  uint8_t message[8];

  TxHeader.ExtId = CAN_TxExtId; // 扩展标识符(29位)
  TxHeader.IDE = CAN_ID_EXT;    // 使用扩展帧
  TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
  TxHeader.DLC = len;

  for (i = 0; i < len; i++)
  {
    message[i] = msg[i];
  }

  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, message, &TxMailbox) != HAL_OK) // 发送
  {
    printf("CAN Send failed!\r\n");
    return 1;
  }
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3)
  {
  }

  return 0;
}

/*CAN过滤器初始化*/
void CAN_Filter_Init(void)
{
  uint32_t CPU_ID[3]; // 定义一个数组存储CPU的ID
  CPU_ID[0] = HAL_GetUIDw0();
  CPU_ID[1] = HAL_GetUIDw1();
  CPU_ID[2] = HAL_GetUIDw2();

  CAN_TxExtId = (CPU_ID[0] + CPU_ID[0] + CPU_ID[0]) & 0x1FFFFFFF; // 获取设备ID

  // Read boot config
  read_boot_config();

  if (dev.version != 0 && dev.version != 0xffff)
  {
    version = dev.version;
  } else {
    dev.version = 0x0100;
    dev.backup_flag = 0xFFFFFFF;   //Start Normal
    dev.charge_flag = 0;
    dev.serial_no = 0x12345678;
    dev.can_server_id = 0x17532F75;
    write_boot_config();
  }

  printf("Device info: CAN_TxExtId: %x, version: %x, can_server_id: %x, serial_no: %x, charge_flag: %x, backup_flag: %x\n", CAN_TxExtId,  dev.version, dev.can_server_id, dev.serial_no, dev.charge_flag, dev.backup_flag);

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;                      /* 过滤器组0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* 屏蔽位模式 */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 32位。*/

  sFilterConfig.FilterIdHigh = (((uint32_t)CAN_RxExtId << 3) & 0xFFFF0000) >> 16;                  /* 要过滤的ID高位 */
  sFilterConfig.FilterIdLow = (((uint32_t)CAN_RxExtId << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; /* 要过滤的ID低位 */
  sFilterConfig.FilterMaskIdHigh = 0xFFFF;                                                         /* 过滤器高16位每位必须匹配 */
  sFilterConfig.FilterMaskIdLow = 0xFFFF;                                                          /* 过滤器低16位每位必须匹配 */
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                               /* 过滤器被关联到FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;                                                         /* 使能过滤器 */
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  TxHeader.ExtId = CAN_TxExtId; // 扩展标识符(29位)
  TxHeader.IDE = CAN_ID_EXT;    // 使用标准帧
  TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}

void Key_Scan(void)
{
  static uint32_t lastKeyState = GPIO_PIN_RESET;
  uint32_t currentKeyState = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

  // 去抖动处理，可以通过延时或者状态机方式实现，这里简化处理
  if (currentKeyState != lastKeyState)
  {
    HAL_Delay(20); // 简单的去抖动延时
    currentKeyState = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
  }

  if (currentKeyState == GPIO_PIN_RESET && lastKeyState == GPIO_PIN_SET)
  {
    keyState = LONG_PRESS_START; // 按键从释放变为按下，开始计时
    // HAL_TIM_Base_Start_IT(&htim3); // 启动定时器中断
  }
  else if (currentKeyState == GPIO_PIN_SET && lastKeyState == GPIO_PIN_RESET)
  {
    keyState = IDLE; // 按键释放，重置状态
    // HAL_TIM_Base_Stop_IT(&htim3); // 停止定时器中断
    keyPressTime = 0; // 重置计时
  }
  lastKeyState = currentKeyState;
}

uint16_t ADC_Read(uint32_t Channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = Channel; /* 通道 */
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; /* 采样时间 */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

void Send_TCA(void)
{
  // CAN发送2个字节的数据
  // send the charge status to CAN
  uint16_t input = cur_input;

  TxBuf[0] = 0xAB;
  TxBuf[1] = input & 0xFF;
  TxBuf[2] = input >> 8;

  Tx_Flag = CAN_Send_Msg(TxBuf, 3); // 发送数据，根据返回值判定发送是否异常
  // if (Tx_Flag)
  //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
  // else
  //   printf("Send TCA completed !\r\n");

  // 清空接收、发送数组，保留Rxbuf内容
  memset(TxBuf, 0, sizeof(TxBuf));
  // HAL_Delay(50);

  lastInuput = cur_input;
  // 串口发送
  printf("TCA9535 input: %x\n", input);
  bSendTCA = 0;
}

void Report_State(void)
{
  ADC_Values[0] = ADC_Read(ADC_CHANNEL_10); // ADC SYSTEM VOLTAGE
  ADC_Values[1] = ADC_Read(ADC_CHANNEL_11); // ADC SYSTEM CURRENT
  ADC_Values[2] = ADC_Read(ADC_CHANNEL_12); // ADC 12V CURRENT
  ADC_Values[3] = ADC_Read(ADC_CHANNEL_13); // ADC VM CURRENT
  ADC_Values[4] = ADC_Read(ADC_CHANNEL_8);  // ADC VBAT CHARGE CURRENT

  // 门状态
  TxBuf[0] = CMD_GET_DOOR_STATE;
  TxBuf[1] = get_door_state();
  // 电量  ADC_Values[0]
  TxBuf[2] = convert_vol(ADC_Values[0], 0.05);
  // 电流     ADC_Values[1]
  TxBuf[3] = convert_adc(ADC_Values[1], 0.067);
  // 12V电流   ADC_Values[2]
  TxBuf[4] = convert_adc(ADC_Values[2], 0.1);
  // VM电流    ADC_Values[3]
  TxBuf[5] = convert_adc(ADC_Values[3], 0.1);
  // 这个从配置中获取
  TxBuf[6] = dev.charge_flag & 0xFF;
  // 故障码
  TxBuf[7] = 0x00;

  Tx_Flag = CAN_Send_Msg(TxBuf, 8);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    // printf("KEY_Pin\n");
  }

  if (GPIO_Pin == INT_Pin)
  {
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
