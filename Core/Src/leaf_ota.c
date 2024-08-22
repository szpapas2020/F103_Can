#include "leaf_ota.h"
#include "main.h"
#include "stdio.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "stm32f1xx_hal_flash.h"


typedef const uint16_t uc16;
typedef const uint8_t  uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;


#define STM32_FLASH_BASE 0x08000000     //STM32 FLASH����ʼ��ַ
#define FLASH_WAITETIME  50000          //FLASH�ȴ���ʱʱ��

typedef struct
{
  uint32_t serial_no;
  uint32_t can_server_id;
  uint16_t version;
  uint16_t backup_flag;
} dev_info;

dev_info dev;


uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(vu32 *)faddr;
}

void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead)
{
    uint32_t i;
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //��ȡ4���ֽ�.
        ReadAddr += 4;                            //ƫ��4���ֽ�.
    }
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
{
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    //uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
        return; //�Ƿ���ַ

    HAL_FLASH_Unlock();                   //����
    //addrx = WriteAddr;                    //д�����ʼ��ַ
    endaddr = WriteAddr + NumToWrite * 4; //д��Ľ�����ַ

    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //�ȴ��ϴβ������
    if (FlashStatus == HAL_OK)
    {
        while (WriteAddr < endaddr) //д����
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) //д������
            {
                break; //д���쳣
            }
            WriteAddr += 4;
            pBuffer++;
        }
    }
    HAL_FLASH_Lock(); //����
}

// ��ȡ�û�����
static void read_boot_config(void)
{
  STMFLASH_Read(FLASH_USER_START_ADDR, (uint32_t *)&dev, sizeof(dev));
}

static void write_boot_config(void)
{
    Erase_Config();
    STMFLASH_Write(FLASH_USER_START_ADDR, (uint32_t *) &dev.serial_no, sizeof(dev) / 4);
}

// Erase flash��24K
static void Erase_page(uint32_t addr)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = addr;
  EraseInitStruct.NbPages = 12;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  HAL_FLASH_Lock();
}

// Erase Config, 2K
static void Erase_Config(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
  EraseInitStruct.NbPages = 1;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  HAL_FLASH_Lock();
}


// �����������������ִ�У�����ֻ����־
static unsigned int Check_Start_Mode(void)
{
  unsigned int mode = 0;

  // ��Flash �е�����
  read_boot_config();

  printf("backup flag = %d\r\n", dev.backup_flag);

  if (dev.backup_flag == 0x0)
  {
    return Startup_Normol;
  }

  // ����
  if (dev.backup_flag == 1)
  {
    mode = Startup_Update;
    return mode;
  }

  return Startup_Normol;
}

static uint32_t temp[256];

static void CopyApp(uint32_t src_addr, uint32_t dest_addr, unsigned int byte_size)
{
  /* Erase */
  printf("> Start erase flash......\r\n");
  Erase_page(dest_addr);          // 24K
  printf("> Erase 24K finished......\r\n");

  printf("> Start copy......\r\n");

  for (int i = 0; i < byte_size / 1024; i++)
  {
    printf(".");

    STMFLASH_Read((src_addr + i * 1024), temp, 256);
    STMFLASH_Write((dest_addr + i * 1024), temp, 256);
  }

  printf("\r\n > Copy down......\r\n");

  dev.backup_flag = 0;
  write_boot_config();
}
