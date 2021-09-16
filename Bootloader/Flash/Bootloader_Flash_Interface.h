/**
  ******************************************************************************
  * File Name          : Bootloader_Flash_Interface.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#define BOOT_FLASH_DEBUG(x)
#define BOOT_FLASH_SET_ERROR(x)    boot_flash_if.error = x;\
								   boot_flash_if.status = BOOT_FLASH_STATUS_ERROR;
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  Bootloader flash status 
  */
typedef enum {
	BOOT_FLASH_STATUS_READY = 0x00U,
	BOOT_FLASH_STATUS_BUSY = 0x01U,
	BOOT_FLASH_STATUS_ERROR = 0x02U,
} BOOT_FlashStatusTypeDef;
/** 
  * @brief  Bootloader flash error 
  */
typedef enum {
	BOOT_FLASH_ERROR_OK = 0,
	BOOT_FLASH_ERROR_ERASE = 1,
	BOOT_FLASH_ERROR_WRITE = 2,
	BOOT_FLASH_ERROR_CHECK = 3,
	BOOT_FLASH_ERROR_OPTIONS_BYTES = 4,
	BOOT_FLASH_ERROR_PROTECTION = 5,
	BOOT_FLASH_ERROR_SECTOR = 6,
	BOOT_FLASH_ERROR_ENABLE_WRP = 7,
	BOOT_FLASH_ERROR_DISABLE_WRP = 8,
	BOOT_FLASH_ERROR_RDP_LEVEL_CONFIG = 9,
} BOOT_FlashErrorTypeDef;
/** 
  * @brief  Bootloader flash  structure
  */
typedef struct __BOOT_HandleFlashInterfaceTypeDef
{
	BOOT_FlashStatusTypeDef status;
	BOOT_FlashErrorTypeDef error;
} BOOT_HandleFlashInterfaceTypeDef;

extern BOOT_HandleFlashInterfaceTypeDef boot_flash_if;
/* Exported constants --------------------------------------------------------*/
/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12 ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13 ((uint32_t)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14 ((uint32_t)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15 ((uint32_t)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16 ((uint32_t)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17 ((uint32_t)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18 ((uint32_t)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19 ((uint32_t)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20 ((uint32_t)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21 ((uint32_t)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22 ((uint32_t)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23 ((uint32_t)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */
#define FLASH_SECTOR_NO 0xFFU
/* Exported Macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
uint32_t Erase_Sectors(uint32_t start_address, uint32_t end_address);
uint32_t Write_Block_Flash(uint32_t size, uint32_t address_flash, uint32_t *address_data);
void Jump_To_Address_Flash(uint32_t addr);
uint32_t Get_Sector_Size(uint32_t Sector);
uint32_t Get_Sector(uint32_t address);
HAL_StatusTypeDef FLASH_OB_EnableWRP(uint32_t WRPSector, uint32_t Banks);
HAL_StatusTypeDef FLASH_OB_DisableWRP(uint32_t WRPSector, uint32_t Banks);
HAL_StatusTypeDef FLASH_OB_RDP_LevelConfig(uint8_t Level);
uint8_t FLASH_OB_GetRDP(void);
bool FLASH_OB_IsWRP(uint32_t WRPSector);
#endif /*__FLASH_H*/
	   /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
