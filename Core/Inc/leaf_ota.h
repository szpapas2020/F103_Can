/*
 * @Description:
 * @Version: 2.0
 * @Company: BT
 * @Date: 2021-06-30 17:02:16
 * @LastEditors: liu jun
 * @LastEditTime: 2021-07-09 10:52:10
 * @FilePath: \F429.bootloader.6290\Core\Inc\leaf_ota.h
 */
#ifndef __LEAF_OTA_H_
#define __LEAF_OTA_H_

#include "main.h"

typedef struct
{
  uint32_t serial_no;
  uint32_t can_server_id;
  uint16_t version;
  uint16_t charge_flag;
  uint32_t backup_flag;
} dev_info;

extern dev_info dev;

/*=====用户配置(根据自己的分区进行配置)=====*/
#define BootLoader_Size 0x2000U  ///< BootLoader的大小 8K
#define Application_Size 0x6000U ///< 应用程序的大小 24K

#define Application_Addr 0x08004000U ///< 应用程序1的首地址    16K 开始
/*==========================================*/

/* 启动的步骤 */
#define Startup_Normol 0xFFFFFFFF ///< 正常启动
#define Startup_Update 0xAAAAAAAA ///< 升级再启动

// 保存用户数据的地址, 第2个扇区的位置,
#define FLASH_USER_START_ADDR ((uint32_t)0x800F800U) // 62K的位置

#define APP_NEW_FW_START_ADR (0x0800A000) // 40K 位置开始备份，到64K结束
#define APP_NEW_FW_END_ADR (0x08010000 - 1)
#define APP_NEW_FW_MAX_SIZE (APP_NEW_FW_END_ADR - APP_NEW_FW_START_ADR) // 256K

void Start_BootLoader(void);
void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead);
void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumToWrite);
void read_boot_config(void);
void write_boot_config(void);
void Erase_page(uint32_t addr);
void Erase_Config(void);
uint32_t Check_Start_Mode(void);
void CopyApp(uint32_t src_addr, uint32_t dest_addr, unsigned int byte_size);

#endif
