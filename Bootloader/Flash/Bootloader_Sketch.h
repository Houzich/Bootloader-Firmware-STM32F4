/**
  ******************************************************************************
  * File Name          : Bootloader_Sketch.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOTLOADER_SKETCH_H
#define __BOOTLOADER_SKETCH_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Bootloader_Serial_Interface.h"
#include "Bootloader_Flash_Interface.h"

#define BOOT_DEBUG(x)

#define FLASH_BOOTLOADER_SECTOR 		(uint16_t)(FLASH_SECTOR_0)
#define FLASH_BOOTLOADER_SECTOR_ADDR 	ADDR_FLASH_SECTOR_0
#define FLASH_APPLICATION_SECTOR 		(uint16_t)(FLASH_SECTOR_1)
#define FLASH_APPLICATION_SECTOR_ADDR 	ADDR_FLASH_SECTOR_1
#define FLASH_APPLICATION_END_ADDR 		MAIN_MEMORY_END

#define MAIN_MEMORY_BASE        0x08000000U
#define MAIN_MEMORY_END         0x080FFFFFU
#define SYSTEM_MEMORY_BASE      0x1FFF0000U
#define SYSTEM_MEMORY_END       0x1FFF77FFU
#define OTP_AREA_BASE           FLASH_OTP_BASE
#define OTP_AREA_END            FLASH_OTP_END
#define OPTIONS_BYTES_BASE 		0x1FFFC000U
#define OPTIONS_BYTES_END 		0x1FFFC00FU

#define IS_MAIN_MEMORY_ADDRESS(ADDRESS) (((ADDRESS) >= MAIN_MEMORY_BASE) && ((ADDRESS) <= MAIN_MEMORY_END))
#define IS_SYSTEM_MEMORY_ADDRESS(ADDRESS) (((ADDRESS) >= SYSTEM_MEMORY_BASE) && ((ADDRESS) <= SYSTEM_MEMORY_END))
#define IS_OTP_AREA_ADDRESS(ADDRESS) (((ADDRESS) >= OTP_AREA_BASE) && ((ADDRESS) <= OTP_AREA_END))
#define IS_OPTIONS_BYTES_ADDRESS(ADDRESS) (((ADDRESS) >= OPTIONS_BYTES_BASE) && ((ADDRESS) <= OPTIONS_BYTES_END))
#define IS_OPTCR_ADDRESS(ADDRESS) ((ADDRESS) >= OPTCR_BYTE0_ADDRESS&&(ADDRESS) <= OPTCR_BYTE3_ADDRESS)


#define IS_MEMORY_ADDRESS(ADDRESS) (IS_MAIN_MEMORY_ADDRESS(ADDRESS)||IS_SYSTEM_MEMORY_ADDRESS(ADDRESS)||\
									IS_OTP_AREA_ADDRESS(ADDRESS)||IS_OPTIONS_BYTES_ADDRESS(ADDRESS)||IS_OPTCR_ADDRESS(ADDRESS))
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  Bootloader main error 
  */
typedef enum {
	BOOT_STATUS_START = 0x00U,
	BOOT_STATUS_WAIT_ID_SESSION = 0x01U,
	BOOT_STATUS_BUSY_SESSION = 0x02U,
	BOOT_STATUS_ERROR = 0x03U,
	BOOT_STATUS_FATAL_ERROR = 0x04U,
	BOOT_STATUS_JUMP_TO_APPLICATION = 0x05U,
} BOOT_StatusTypeDef;
/** 
  * @brief  Bootloader main status 
  */
typedef enum {
	BOOT_ERROR_OK = 0x00U,
	BOOT_ERROR_SERIAL = 0x01U,
	BOOT_ERROR_FLASH = 0x02U,
} BOOT_ErrorTypeDef;
/** 
  * @brief  Bootloader step session 
  */
#pragma pack(push,1)
typedef const struct __BOOT_CommandStepTypeDef
{
	uint8_t message[4];
	bool (*link_fn)(void);		/* Link status callback function prototype */
	bool (*if_true_fn)(void);	/* Link if true callback function prototype */
	bool (*if_false_fn)(void); 	/* Link if false callback function prototype */
	int next_true_step;
	int next_false_step;
} BOOT_CommandStepTypeDef;
#pragma pack(pop)
/** 
  * @brief  Bootloader session 
  */
#pragma pack(push,1)
typedef const struct __BOOT_CommandTypeDef
{
	struct __BOOT_CommandStepTypeDef step[15];
	uint16_t finish;
	uint16_t id;
} BOOT_CommandTypeDef;
#pragma pack(pop)
/** 
  * @brief  Bootloader main structure
  */
typedef struct __BOOT_HandleTypeDef
{
	BOOT_StatusTypeDef status;
	BOOT_ErrorTypeDef error;
	BOOT_HandleSeriaInterfaceTypeDef *boot_serial_if;
	BOOT_HandleFlashInterfaceTypeDef *boot_flash_if;
	BOOT_CommandStepTypeDef *step_curr;
	BOOT_CommandTypeDef *session_curr;
} BOOT_HandleTypeDef;

extern BOOT_HandleTypeDef boot;

/* Exported constants --------------------------------------------------------*/
#define ID_COMMAND_GET ID_HTONS(0x00FF)
#define ID_COMMAND_GET_VERSION ID_HTONS(0x00FE)
#define ID_COMMAND_GET_ID ID_HTONS(0x02FD)
#define ID_COMMAND_READ_MEMORY ID_HTONS(0x11EE)
#define ID_COMMAND_GO ID_HTONS(0x21DE)
#define ID_COMMAND_WRITE_MEMORY ID_HTONS(0x31CE)
#define ID_COMMAND_ERASE_MEMORY ID_HTONS(0x44BB)
#define ID_COMMAND_EXTENDED_ERASE_MEMORY ID_HTONS(0x44BB)
#define ID_COMMAND_WRITE_PROTECT ID_HTONS(0x639C)
#define ID_COMMAND_WRITE_UNPROTECT ID_HTONS(0x738C)
#define ID_COMMAND_READOUT_PROTECT ID_HTONS(0x827D)
#define ID_COMMAND_READOUT_UNPROTECT ID_HTONS(0x926D)

#define END_STEP (uint32_t)(-1)
#define NEXT_STEP (uint32_t)NULL
/* Exported Macros -----------------------------------------------------------*/
#define ID_HTONS(x) ((((x)&0x00ffUL) << 8) | (((x)&0xff00UL) >> 8))
/* Exported functions --------------------------------------------------------*/
void Bootloader_Init(UART_HandleTypeDef *huart);
void Bootloader_Loop(void);
#endif /*__BOOTLOADER_SKETCH_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
