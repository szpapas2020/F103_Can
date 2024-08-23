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


#define STM32_FLASH_BASE 0x08000000     //STM32 FLASH的起始地址
#define FLASH_WAITETIME  50000          //FLASH等待超时时间

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
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取4个字节.
        ReadAddr += 4;                            //偏移4个字节.
    }
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite)
{
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    //uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
        return; //非法地址

    HAL_FLASH_Unlock();                   //解锁
    //addrx = WriteAddr;                    //写入的起始地址
    endaddr = WriteAddr + NumToWrite * 4; //写入的结束地址

    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
    if (FlashStatus == HAL_OK)
    {
        while (WriteAddr < endaddr) //写数据
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) //写入数据
            {
                break; //写入异常
            }
            WriteAddr += 4;
            pBuffer++;
        }
    }
    HAL_FLASH_Lock(); //上锁
}

// 读取用户配置
void read_boot_config(void)
{
  STMFLASH_Read(FLASH_USER_START_ADDR, (uint32_t *)&dev, sizeof(dev));
}

void write_boot_config(void)
{
    Erase_Config();
    STMFLASH_Write(FLASH_USER_START_ADDR, (uint32_t *) &dev.serial_no, sizeof(dev) / 4);
}

// Erase flash，24K
void Erase_page(uint32_t addr)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = addr;
  EraseInitStruct.NbPages = 24;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  HAL_FLASH_Lock();
}
// Erase Config, 2K
void Erase_Config(void)
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
