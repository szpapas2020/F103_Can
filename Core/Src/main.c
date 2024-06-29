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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TCA9535_ADDRESS 0x20         // TCA9535的I2C地址
#define TCA9535_INPUT_PORT0_REG 0x00 // 输入端口0的寄存器地址
#define TCA9535_CONFIG_REG 0x06

#define CAN_RxExtId 0x1800D8D0
// #define CAN_TxExtId 0x1800D0D8

#define false 0
#define true 1

//=============================================================================
// 帧的字节顺序
//=============================================================================
#define HEAD 0
#define LENGTH 1
#define ADDR 2
#define CMD 3
#define DATA_START 4

//=============================================================================
// 数据帧类型
//=============================================================================
#define CMD_OPEN_DOOR 0xA0
#define CMD_CLOSE_DOOR 0xA1
#define CMD_ALARM 0xA2
#define CMD_GET_DOOR_STATE 0xA3

#define CMD_TURN_ON 0xB1
#define CMD_TURN_OFF 0xB2

//=============================================================================
#define VERSION 0x00         // 协议版本号
#define FIRM_UPDATA_SIZE 256 // 升级包大小
//=============================================================================

#define UART_QUEUE_LMT 1024    // 数据接收队列大小,如MCU的RAM不够,可缩小
#define UART_RECV_BUF_LMT 1024 // 固件升级缓冲区,需大缓存,必须大于260
#define UART_SEND_BUF_LMT 512  // 根据用户DP数据大小量定,必须大于32

typedef enum
{
  MCU_UART_REV_STATE_FOUND_NULL,
  MCU_UART_REV_STATE_FOUND_HEAD, // 1byte
  MCU_UART_REV_STATE_FOUND_LEN,  // 1byte
  MCU_UART_REV_STATE_FOUND_ADD,  // 1byte
  MCU_UART_REV_STATE_FOUND_CMD,  // 1byte
  MCU_UART_REV_STATE_FOUND_DATA, // nbyte
  MCU_UART_REV_STATE_FOUND_CRC,
  MCU_UART_REV_STATE_UNKOWN,
} mcu_uart_rev_state_type_t;

unsigned char volatile queue_buf[UART_QUEUE_LMT]; // 队列缓存  1024
unsigned char uart_rx_buf[UART_RECV_BUF_LMT];     // 接收缓存  1024
unsigned char uart_tx_buf[UART_SEND_BUF_LMT];     // 发送缓存  512

unsigned char *queue_in;
unsigned char *queue_out;

static volatile mcu_uart_rev_state_type_t current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_NULL;
static uint8_t uart_rx_buf_temp[1] = {0};
static uint16_t uart_data_len = 0;
static volatile uint16_t UART_RX_Count = 0;

int bPowerOff = 0;
int bPowerOff_count = 12;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================== RS485 =========================================

/*****************************************************************************
函数名称 : get_queue_total_data
功能描述 : 读取队列内数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char get_queue_total_data(void)
{
  if (queue_in != queue_out)
    return 1;
  else
    return 0;
}

/*****************************************************************************
函数名称 : Queue_Read_Byte
功能描述 : 读取队列1字节数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char Queue_Read_Byte(void)
{
  unsigned char value;

  if (queue_out != queue_in)
  {
    // 有数据
    if (queue_out >= (unsigned char *)(queue_buf + sizeof(queue_buf)))
    {
      // 数据已经到末尾
      queue_out = (unsigned char *)(queue_buf);
    }

    value = *queue_out++;
  }

  return value;
}

/*****************************************************************************
函数名称 : hex_to_bcd
功能描述 : hex转bcd
输入参数 : Value_H:高字节/Value_L:低字节
返回参数 : bcd_value:转换完成后数据
*****************************************************************************/
unsigned char hex_to_bcd(unsigned char Value_H, unsigned char Value_L)
{
  unsigned char bcd_value;

  if ((Value_H >= '0') && (Value_H <= '9'))
    Value_H -= '0';
  else if ((Value_H >= 'A') && (Value_H <= 'F'))
    Value_H = Value_H - 'A' + 10;
  else if ((Value_H >= 'a') && (Value_H <= 'f'))
    Value_H = Value_H - 'a' + 10;

  bcd_value = Value_H & 0x0f;

  bcd_value <<= 4;
  if ((Value_L >= '0') && (Value_L <= '9'))
    Value_L -= '0';
  else if ((Value_L >= 'A') && (Value_L <= 'F'))
    Value_L = Value_L - 'a' + 10;
  else if ((Value_L >= 'a') && (Value_L <= 'f'))
    Value_L = Value_L - 'a' + 10;

  bcd_value |= Value_L & 0x0f;

  return bcd_value;
}

/*****************************************************************************
函数名称 : hexstr2byte
功能描述 : hex字符串转换为byte
输入参数 : 字符串
返回参数 : 转换后的数据放在bufout中，长度为原来的一半
*****************************************************************************/
int hexstr2byte(const char *buf, int len, char *bufout)
{
  int ret = -1;
  int i = 0;
  uint8_t low;
  uint8_t high;

  if (NULL == buf || len <= 0 || NULL == bufout)
  {
    return ret;
  }

  ret = 0;
  for (i = 0; i < len; i = i + 2)
  {
    if (((buf[i]) >= '0') && (buf[i] <= '9'))
    {
      high = (uint8_t)(buf[i] - '0');
    }
    else if ((buf[i] >= 'A') && (buf[i] <= 'F'))
    {
      high = (uint8_t)(buf[i] - 'A') + 10;
    }
    else if ((buf[i] >= 'a') && (buf[i] <= 'f'))
    {
      high = (uint8_t)(buf[i] - 'a') + 10;
    }
    else
    {
      ret = -1;
      break;
    }

    if (((buf[i + 1]) >= '0') && (buf[i + 1] <= '9'))
    {
      low = (uint8_t)(buf[i + 1] - '0');
    }
    else if ((buf[i + 1] >= 'A') && (buf[i + 1] <= 'F'))
    {
      low = (uint8_t)(buf[i + 1] - 'A') + 10;
    }
    else if ((buf[i + 1] >= 'a') && (buf[i + 1] <= 'f'))
    {
      low = (uint8_t)(buf[i + 1] - 'a') + 10;
    }
    else
    {
      ret = -1;
      break;
    }

    bufout[i / 2] = (char)((high << 4) | (low & 0x0F));
  }
  return ret;
}

// make a byte to 2 ascii hex
int byte2hexstr(uint8_t *bufin, int len, char *bufout)
{
  int i = 0;
  uint8_t tmp_l = 0x0;
  uint8_t tmp_h = 0;
  if ((NULL == bufin) || (len <= 0) || (NULL == bufout))
  {
    return -1;
  }
  for (i = 0; i < len; i++)
  {
    tmp_h = (bufin[i] >> 4) & 0X0F;
    tmp_l = bufin[i] & 0x0F;
    bufout[2 * i] = (tmp_h > 9) ? (tmp_h - 10 + 'a') : (tmp_h + '0');
    bufout[2 * i + 1] = (tmp_l > 9) ? (tmp_l - 10 + 'a') : (tmp_l + '0');
  }
  bufout[2 * len] = '\0';

  return 0;
}

/*****************************************************************************
函数名称 : my_strlen
功能描述 : 求字符串长度
输入参数 : src:源地址
返回参数 : len:数据长度
*****************************************************************************/
unsigned long my_strlen(unsigned char *str)
{
  unsigned long len = 0;
  if (str == NULL)
  {
    return 0;
  }

  for (len = 0; *str++ != '\0';)
  {
    len++;
  }

  return len;
}
/*****************************************************************************
函数名称 : my_memset
功能描述 : 把src所指内存区域的前count个字节设置成字符c
输入参数 : src:源地址
           ch:设置字符
           count:设置数据长度
返回参数 : src:数据处理完后的源地址
*****************************************************************************/
void *my_memset(void *src, unsigned char ch, unsigned short count)
{
  unsigned char *tmp = (unsigned char *)src;

  if (src == NULL)
  {
    return NULL;
  }

  while (count--)
  {
    *tmp++ = ch;
  }

  return src;
}
/*****************************************************************************
函数名称 : mymemcpy
功能描述 : 内存拷贝
输入参数 : dest:目标地址
           src:源地址
           count:数据拷贝数量
返回参数 : src:数据处理完后的源地址
*****************************************************************************/
void *my_memcpy(void *dest, const void *src, unsigned short count)
{
  unsigned char *pdest = (unsigned char *)dest;
  const unsigned char *psrc = (const unsigned char *)src;
  unsigned short i;

  if (dest == NULL || src == NULL)
  {
    return NULL;
  }

  if ((pdest <= psrc) || (pdest > psrc + count))
  {
    for (i = 0; i < count; i++)
    {
      pdest[i] = psrc[i];
    }
  }
  else
  {
    for (i = count; i > 0; i--)
    {
      pdest[i - 1] = psrc[i - 1];
    }
  }

  return dest;
}
/*****************************************************************************
函数名称 : memcmp
功能描述 : 内存比较
输入参数 : buffer1:内存1
           buffer2:内存2
            count:比较长度
返回参数 : 大小比较值，0:buffer1=buffer2; -1:buffer1<buffer2; 1:buffer1>buffer2
*****************************************************************************/
int my_memcmp(const void *buffer1, const void *buffer2, int count)
{
  if (!count)
    return (0);
  while (--count && *(char *)buffer1 == *(char *)buffer2)
  {
    buffer1 = (char *)buffer1 + 1;
    buffer2 = (char *)buffer2 + 1;
  }
  return (*((unsigned char *)buffer1) - *((unsigned char *)buffer2));
}

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

/*
 *@brief Function for receive uart data.
 *@param
 *
 *@note
 *
 * 数据解析过程
 *
 *
 * */
static int mcu_common_uart_data_unpack(uint8_t data)
{
  // printf("%02x ", data);

  int ret = false;
  uart_rx_buf_temp[0] = data;

  if (uart_rx_buf_temp[0] == 0xA0 && current_uart_rev_state_type == MCU_UART_REV_STATE_FOUND_NULL)
  {
    my_memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
    my_memcpy(uart_rx_buf, uart_rx_buf_temp, 1);
    my_memset(uart_rx_buf_temp, 0, 1);
    UART_RX_Count = 1;
    current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_HEAD;
    uart_data_len = 0;
    return ret;
  }

  switch (current_uart_rev_state_type)
  {
  case MCU_UART_REV_STATE_FOUND_NULL:
    break;
  case MCU_UART_REV_STATE_FOUND_HEAD:
    uart_rx_buf[UART_RX_Count++] = data;
    current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_LEN;
    uart_data_len = data;
    break;
  case MCU_UART_REV_STATE_FOUND_LEN:
    uart_rx_buf[UART_RX_Count++] = data;
    uart_data_len--;
    if (uart_data_len == 0)
    {
      current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_DATA;
    }
    break;
  case MCU_UART_REV_STATE_FOUND_DATA:
    uart_rx_buf[UART_RX_Count++] = data; // CRC
    current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_CRC;
    // check crc
    if (uart_rx_buf[UART_RX_Count - 1] == CheckSum(uart_rx_buf, UART_RX_Count - 1))
    {
      printf("CRC Check OK !\r\n");
      ret = true;
    }
    else
    {
      my_memset(uart_rx_buf_temp, 0, 1);
      my_memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
      UART_RX_Count = 0;
      current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_NULL;
      uart_data_len = 0;
    }
    break;
  default:
    my_memset(uart_rx_buf_temp, 0, 1);
    my_memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
    UART_RX_Count = 0;
    current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_NULL;
    uart_data_len = 0;
    break;
  };
  return ret;
}

void open_door(uint8_t door)
{
  if( cur_input |= (1 << door*2) ) {
    printf("Door %d is already opened !\r\n", door);
    return;
  } else {
    printf("Open Door %d !\r\n", door);
    motor_pwm[door].dir = 1;
    motor_pwm[door].pwm = 50;
  }

}


void close_door(uint8_t door)
{
  printf("Close Door %d !\r\n", door);
  if (cur_input &= (1 << door*2) ) {
    printf("Door %d is already closed !\r\n", door);
    return;
  } else {
    motor_pwm[door].dir = 0;
    motor_pwm[door].pwm = 50;
  }
}

unsigned char get_door_state(void)
{
  unsigned char door_state = 0;
  for (int i = 0; i < 8; i++)
  {
    if (cur_input & (1 << i*2))
    {
      printf("Door %d is opened !\r\n", i);
      door_state |= (1 << i);
    }
  }

  return door_state;
}

/*****************************************************************************
函数名称 : data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void data_handle(unsigned short offset)
{

  // unsigned char addr = uart_rx_buf[offset + ADDR];
  unsigned char cmd = uart_rx_buf[offset + CMD];
  unsigned char data = uart_rx_buf[offset + DATA_START];

  unsigned char cmd_ack[6] = {0xA0, 0x04, 0x00, 0xA0, 0x10, 0x00};
  unsigned char cmd_status[12] = {0xA0, 0x0A, 0x00, 0xA3, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00};


  switch (cmd)
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
    cmd_ack[CMD] = CMD_OPEN_DOOR;
    cmd_ack[5] = CheckSum(cmd_ack, 5);
    send_cmd(cmd_ack, sizeof(cmd_ack));

    break;

  case CMD_CLOSE_DOOR: //  CLOSE DOOR
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
    cmd_ack[CMD] = CMD_CLOSE_DOOR;
    cmd_ack[5] = CheckSum(cmd_ack, 5);
    send_cmd(cmd_ack, sizeof(cmd_ack));
    break;

  case CMD_ALARM: // ALARM
    printf("Alarm !\r\n");

    cmd_ack[CMD] = CMD_ALARM;
    cmd_ack[5] = CheckSum(cmd_ack, 5);
    send_cmd(cmd_ack, sizeof(cmd_ack));
    break;

  case CMD_GET_DOOR_STATE: //  GET STATE
    printf("Get Door State !\r\n");

  ADC_Values[0] = ADC_Read(ADC_CHANNEL_10); // ADC SYSTEM VOLTAGE
  ADC_Values[1] = ADC_Read(ADC_CHANNEL_11); // ADC SYSTEM CURRENT
  ADC_Values[2] = ADC_Read(ADC_CHANNEL_12); // ADC 12V CURRENT
  ADC_Values[3] = ADC_Read(ADC_CHANNEL_13); // ADC VM CURRENT
  ADC_Values[4] = ADC_Read(ADC_CHANNEL_8);  // ADC VBAT CHARGE CURRENT

    // 门状态
    cmd_status[4] = get_door_state();
    // 电量  ADC_Values[0]
    cmd_status[5] = 0x64;   // 100%
    // 电流  ADC_Values[1]
    cmd_status[6] = 0x22;   // 100%
    // 12V电流  ADC_Values[2]
    cmd_status[7] = 0x22;   // 100%
    // VM电流  ADC_Values[3]
    cmd_status[8] = 0x22;   // 100%
    // 充电电流  ADC_Values[4]
    cmd_status[9] = 0x22;   // 100%
    // 故障码
    cmd_status[10] = 0x00;
    // 校验和
    cmd_status[11] = CheckSum(cmd_status, 11);
    send_cmd(cmd_status, sizeof(cmd_status));
    break;

  case CMD_TURN_ON: //  TURN ON
    printf("Turn On !\r\n");
    // enable PWR_12V_EN_Pin
    HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_SET);
    // Send ACK
    cmd_ack[CMD] = CMD_TURN_ON;
    cmd_ack[5] = CheckSum(cmd_ack, 5);
    send_cmd(cmd_ack, sizeof(cmd_ack));

    break;

  case CMD_TURN_OFF: //  TURN OFF
    printf("Turn Off !\r\n");
    bPowerOff = 1;
    bPowerOff_count = 12;
    // Send ACK
    cmd_ack[CMD] = CMD_TURN_OFF;
    cmd_ack[5] = CheckSum(cmd_ack, 5);
    send_cmd(cmd_ack, sizeof(cmd_ack));
    break;

  default:
    printf("Unknown CMD !\r\n");
    break;
  }
}

/*****************************************************************************
函数名称  : UART_service
功能描述  : bt串口处理服务
输入参数 : 无
返回参数 : 无
使用说明 : 在MCU主函数while循环中调用该函数
           新建一个任务，处理这个buffer中的数据。
           TCP 接收进程，将收到的数据放在队列中。

*****************************************************************************/
void rs485_service(void)
{
  if (get_queue_total_data() > 0)
  {
    unsigned char uc = Queue_Read_Byte();

    if (mcu_common_uart_data_unpack(uc))
    {
      data_handle(0);
      // rx_value_len = UART_rx_buf[LENGTH_HIGH] * 0x100 + UART_rx_buf[LENGTH_LOW] + PROTOCOL_HEAD;

      my_memset(uart_rx_buf_temp, 0, 1);
      my_memset(uart_rx_buf, 0, sizeof(uart_rx_buf));

      UART_RX_Count = 0;
      current_uart_rev_state_type = MCU_UART_REV_STATE_FOUND_NULL;
      uart_data_len = 0;
    }
  }
}

/*****************************************************************************
函数名称 : rs485_protocol_init
功能描述 : 协议串口初始化函数
输入参数 : 无
返回参数 : 无
使用说明 : 必须在MCU初始化代码中调用该函数
*****************************************************************************/
void rs485_protocol_init(void)
{
  queue_in = (unsigned char *)queue_buf;
  queue_out = (unsigned char *)queue_buf;
}

/*****************************************************************************
函数名称 : rs485_receive_input
功能描述 : 收数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 在MCU串口接收函数中调用该函数,并将接收到的数据作为参数传入
*****************************************************************************/
void rs485_receive_input(unsigned char value)
{
  // printf("%02x ", value);

  if ((queue_in > queue_out) && ((queue_in - queue_out) >= sizeof(queue_buf)))
  {
    // 数据队列满
    printf("RS485 queue buf is FULL \r\n");
  }
  else if ((queue_in < queue_out) && ((queue_out - queue_in) == 0))
  {
    // 数据队列满
    printf("RS485 queue buf is FULL 2 \r\n");
  }
  else
  {
    // 队列不满
    if (queue_in >= (unsigned char *)(queue_buf + sizeof(queue_buf)))
    {
      queue_in = (unsigned char *)(queue_buf);
    }

    *queue_in++ = value;
  }
}

// ==============================END OF RS485 =========================================

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Init();
  printf("Program Started !\r\n");

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  rs485_protocol_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t input;
    uint8_t mt_id, pwm, dir;

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
      printf("Receive CAN Succeed !\r\n");
      switch (RxBuf[0])
      {
      case 0xAA: // query the status of the charge status

        break;
      case 0xAB: // query the position of the motor
        input = ReadTCA9535Inputs();
        TxBuf[0] = 0xAB;
        TxBuf[1] = input & 0xFF;
        TxBuf[2] = input >> 8;

        Tx_Flag = CAN_Send_Msg(TxBuf, 3); // 发送数据，根据返回值判定发送是否异常
        // if (Tx_Flag)
        //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
        // else
        //   printf("Send completed !\r\n");

        // 清空接收、发送数组，保留Rxbuf内容
        memset(TxBuf, 0, sizeof(TxBuf));

        break;
      case 0xAC: // query the current and voltage of the motor
        // send the adc data to CAN
        TxBuf[0] = 0xAC;
        TxBuf[1] = ADC_Values[0] & 0xFF;
        TxBuf[2] = ADC_Values[0] >> 8;
        TxBuf[3] = ADC_Values[1] & 0xFF;
        TxBuf[4] = ADC_Values[1] >> 8;
        Tx_Flag = CAN_Send_Msg(TxBuf, 5); // 发送数据，根据返回值判定发送是否异常
        // if (Tx_Flag)
        //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
        // else
        //   printf("Send completed !\r\n");

        // 清空接收、发送数组，保留Rxbuf内容
        memset(TxBuf, 0, sizeof(TxBuf));

        break;
      case 0xAD: // query the current and voltage of the system
        TxBuf[0] = 0xAD;
        TxBuf[1] = ADC_Values[2] & 0xFF;
        TxBuf[2] = ADC_Values[2] >> 8;
        TxBuf[3] = ADC_Values[3] & 0xFF;
        TxBuf[4] = ADC_Values[3] >> 8;
        Tx_Flag = CAN_Send_Msg(TxBuf, 5); // 发送数据，根据返回值判定发送是否异常
        // if (Tx_Flag)
        //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
        // else
        //   printf("Send completed !\r\n");
        // 清空接收、发送数组，保留Rxbuf内容
        memset(TxBuf, 0, sizeof(TxBuf));

        break;

      case 0xBB: // set the motor pwm BB 00 dir pwm
        mt_id = RxBuf[1];
        dir = RxBuf[2];
        pwm = RxBuf[3];

        if (mt_id > 7 || dir > 1 || pwm > 100)
        {
          printf("Invalid data !\r\n");
          break;
        }
        else
        {
          motor_pwm[mt_id].dir = dir;
          motor_pwm[mt_id].pwm = pwm;
        }
        break;
      default:
        break;
      }

      // 清空接收、发送数组，保留Rxbuf内容
      memset(RxData, 0, sizeof(RxData));
      memset(TxBuf, 0, sizeof(TxBuf));

      Rx_Flag = 0; // 标志位置0，等待下一次中断
    }

    // TCA9535
    if (bSendTCA)
    {
      Send_TCA();
      bSendTCA = 0;
    }

    // ADC
    if (bSendAdc)
    {
      Send_ADC();
      bSendAdc = 0;
    }

    // RS485_1
    rs485_service();

    // RS485_3
    if (UART3_Rx_flg)
    {
      printf("RS485_3 Request!");

      memset(buffer3, 0, 50);
      sofar3 = 0;
      UART3_Rx_flg = 0;
      /* 重新启动中断接收 */
      HAL_UART_Receive_IT(&huart3, UART3_temp, 1);
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
    // Key_Scan();        // 持续检测按键状态
    // keyPressTime += 1; // 增加计数值
    // switch (keyState)
    // {
    // case LONG_PRESS_START:
    //   if (keyPressTime >= 3000) // 3秒
    //   {
    //     if (runState == TURN_ON) // 如果是开机状态，则执行关机操作
    //     {
    //       HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin | PWR_5V_EN_Pin, GPIO_PIN_RESET); // 关闭PC4和PC5
    //       runState = TURN_OFF;
    //       keyState = IDLE;
    //     }
    //     else // 首次长按，开机操作
    //     {
    //       HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin | PWR_5V_EN_Pin, GPIO_PIN_SET); // 开启PC4和PC5
    //       runState = TURN_ON;
    //       keyState = IDLE;
    //     }
    //     keyPressTime = 0; // 重置计时
    //   }
    //   break;
    // default:
    //   break;
    // }

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

  // if (htim->Instance == TIM4)
  // {
  //   // check the setting of the pwm signal
  //   tm4_count++;
  //   uint16_t pin;
  //   GPIO_TypeDef *port;
  //   if (tm4_count >= 100)
  //     tm4_count = 0;
  //   if (tm4_count == 0)
  //   {
  //     /* code */
  //     for (int i = 0; i < 8; i++)
  //     {
  //       Motor_DataDef md = motor_pwm[i];
  //       if (md.pwm > 0)
  //       {
  //         pin = md.dir == 0 ? md.GPIO_Pin1 : md.GPIO_Pin2;
  //         port = md.dir == 0 ? md.GPIOx1 : md.GPIOx2;
  //         HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  //         // port->ODR |=  pin;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     for (int i = 0; i < 8; i++)
  //     {
  //       Motor_DataDef md = motor_pwm[i];
  //       if (tm4_count == md.pwm)
  //       {
  //         pin = md.dir == 0 ? md.GPIO_Pin1 : md.GPIO_Pin2;
  //         port = md.dir == 0 ? md.GPIOx1 : md.GPIOx2;
  //         HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  //         // port->ODR &= ~pin;
  //       }
  //     }
  //   }
  // }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) /* 检查是否是USART1的回调 */
  {
    rs485_receive_input(UART1_temp[0]);
    HAL_UART_Receive_IT(&huart1, UART1_temp, 1);
  }

  if (huart->Instance == USART3) /* 检查是否是USART3的回调 */
  {
    if (sofar3 < RXBUFFERSIZE - 1)
      buffer3[sofar3++] = UART3_temp[0];
    if (0x0a == UART3_temp[0])
      UART3_Rx_flg = 1;
    else
      HAL_UART_Receive_IT(&huart3, UART3_temp, 1);
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

  // 读取ADC的值
  ADC_Values[0] = ADC_Read(ADC_CHANNEL_10); // ADC SYSTEM VOLTAGE
  ADC_Values[1] = ADC_Read(ADC_CHANNEL_11); // ADC SYSTEM CURRENT
  ADC_Values[2] = ADC_Read(ADC_CHANNEL_12); // ADC 12V CURRENT
  ADC_Values[3] = ADC_Read(ADC_CHANNEL_13); // ADC VM CURRENT
  ADC_Values[4] = ADC_Read(ADC_CHANNEL_8);  // ADC VBAT CHARGE CURRENT

  // bSendAdc = 1;

  // 切断12V 电压
  if (bPowerOff == 1) {
    bPowerOff_count --;
    if (bPowerOff_count == 0) {
      HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_RESET);
      bPowerOff = 0;
    }
  }

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
  // printf("CPU_ID: %x %x %x\n", CPU_ID[0], CPU_ID[1], CPU_ID[2]);

  CAN_TxExtId = (CPU_ID[0] + CPU_ID[0] + CPU_ID[0]) & 0x1FFFFFFF; // 获取设备ID
  printf("CAN_TxExtId: %x\n", CAN_TxExtId);

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

void Send_ADC(void)
{
  // send the adc data to CAN
  TxBuf[0] = 0xAC;
  TxBuf[1] = ADC_Values[0] & 0xFF;
  TxBuf[2] = ADC_Values[0] >> 8;
  TxBuf[3] = ADC_Values[1] & 0xFF;
  TxBuf[4] = ADC_Values[1] >> 8;
  TxBuf[5] = ADC_Values[2] & 0xFF;
  TxBuf[6] = ADC_Values[2] >> 8;
  Tx_Flag = CAN_Send_Msg(TxBuf, 7); // 发送数据，根据返回值判定发送是否异常
                                    // if (Tx_Flag)
                                    //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
                                    // else
  printf("Send ADC(0-2) completed !\r\n");

  // 清空接收、发送数组，保留Rxbuf内容
  memset(TxBuf, 0, sizeof(TxBuf));

  TxBuf[0] = 0xAD;
  TxBuf[1] = ADC_Values[3] & 0xFF;
  TxBuf[2] = ADC_Values[3] >> 8;
  TxBuf[3] = ADC_Values[4] & 0xFF;
  TxBuf[4] = ADC_Values[4] >> 8;
  Tx_Flag = CAN_Send_Msg(TxBuf, 5); // 发送数据，根据返回值判定发送是否异常
                                    // if (Tx_Flag)
                                    //   printf("Send failed ,please check your data !\r\n"); // 返回1代表数据发送异常
                                    // else
  printf("Send ADC(3-4) completed !\r\n");
  // 清空接收、发送数组，保留Rxbuf内容
  memset(TxBuf, 0, sizeof(TxBuf));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    // printf("KEY_Pin\n");
  }

  if (GPIO_Pin == INT_Pin)
  {
    // printf("INT_Pin\n");
    // HAL_GPIO_TogglePin(GPIOB, LED_Pin);
    // HAL_GPIO_TogglePin(BEEPER_GPIO_Port, BEEPER_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_0_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_0_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_1_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_1_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_2_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_2_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_3_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_3_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_4_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_4_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_5_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_5_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_6_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_6_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN1_7_Pin);
    // HAL_GPIO_TogglePin(GPIOA, MOTOR_IN2_7_Pin);
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
