/**
  ******************************************************************************
  * File Name          : Bootloader_Serial_Interface.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define BOOT_SERIAL_DEBUG(x)
#define BOOT_SERIAL_SET_ERROR(x)    boot_serial_if.error = x;\
									boot_serial_if.status = BOOT_SERIAL_STATUS_ERROR;
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  Bootloader commands codes 
  */
typedef enum {
	GET = (uint8_t)0x00,							 //Gets the version and the allowed commands supported by the current version of the bootloader
	GET_VERSION = (uint8_t)0x01,			 //Gets the bootloader version and the Read Protection status of the Flash memory
	GET_ID = (uint8_t)0x02,						 //Gets the chip ID
	READ_MEMORY = (uint8_t)0x11,			 //Reads up to 256 bytes of memory starting from an address specified by the application
	GO = (uint8_t)0x21,								 //Jumps to user application code located in the internal Flash memory or in SRAM
	WRITE_MEMORY = (uint8_t)0x31,			 //Writes up to 256 bytes to the RAM or Flash memory starting from an address specified by the application
	ERASE = (uint8_t)0x43,						 //Erases from one to all the Flash memory pages
	EXTENDED_ERASE = (uint8_t)0x44,		 //Erases from one to all the Flash memory pages using two byte addressing mode (available only for v3.0 usart bootloader versions and above).
	WRITE_PROTECT = (uint8_t)0x63,		 //Enables the write protection for some sectors
	WRITE_UNPROTECT = (uint8_t)0x73,	 //Disables the write protection for all Flash memory sectors
	READOUT_PROTECT = (uint8_t)0x82,	 //Enables the read protection
	READOUT_UNPROTECT = (uint8_t)0x92, //Disables the read protection
	XOR = (uint8_t)0x00,							 //
	ACK = (uint8_t)0x79,							 //ACK answer
	NACK = (uint8_t)0x1F,							 //NACK answer
	START = (uint8_t)0x7F
} BOOT_ByteCommandsTypeDef;

/** 
  * @brief  Bootloader serial status 
  */
typedef enum {
	BOOT_SERIAL_STATUS_READY = 0x00U,
	BOOT_SERIAL_STATUS_BUSY = 0x01U,
	BOOT_SERIAL_STATUS_ERROR = 0x02U,
	BOOT_SERIAL_STATUS_JUMP_TO_APPLICATION = 0x03U,
} BOOT_SerialStatusTypeDef;

/** 
  * @brief  Bootloader serial error   
  */
typedef enum {
	BOOT_SERIAL_ERROR_OK = 1U,
	BOOT_SERIAL_ERROR_TX = 2U,
	BOOT_SERIAL_ERROR_RX = 3U,
	BOOT_SERIAL_ERROR_TX_TIMEOUT = 4U,
	BOOT_SERIAL_ERROR_RX_TIMEOUT = 5U,
	BOOT_SERIAL_ERROR_CHECKSUMM = 6U,
	BOOT_SERIAL_ERROR_MEMORY_ADDRESS = 7U,
	BOOT_SERIAL_ERROR_SIZE_DATA = 8U,
	BOOT_SERIAL_ERROR_DATA = 9U,
	BOOT_SERIAL_ERROR_ERASE_BOOT_PAGE = 10U,
	BOOT_SERIAL_ERROR_WRITE_BOOT_PAGE = 11U,
	BOOT_SERIAL_ERROR_DISABLE_WDP_BOOT_SECTOR = 12U,
	BOOT_SERIAL_ERROR_ENABLE_WDP_BOOT_SECTOR = 13U,
	BOOT_SERIAL_ERROR_WRITE_IN_PROTECT_SECTOR = 14U,
	BOOT_SERIAL_ERROR_ERASE_IN_PROTECT_SECTOR = 15U,
	BOOT_SERIAL_ERROR_ENABLE_WDP = 16U,
	BOOT_SERIAL_ERROR_DISABLE_WDP = 17U,
} BOOT_SerialErrorTypeDef;
/** 
  * @brief  Bootloader serial warning
  */
typedef enum {
	BOOT_SERIAL_WARNING_OK = 0x00U,
	BOOT_SERIAL_WARNING_CHECK_WRITE_ADDR = 0x01U,
	BOOT_SERIAL_WARNING_TIMEOUT_RECEIVE_START_ADDR = 0x02U,
} BOOT_SerialWarningTypeDef;
/**
  * @brief  Bootloader buffer structure
  */
#define BOOT_STACK_MAX_SIZE 16
#define BOOT_DATA_SIZE 1024
typedef uint32_t stack_t;
typedef struct __BOOT_DataTypeDef
{
	stack_t stack[BOOT_STACK_MAX_SIZE];
	uint32_t stack_cnt;
	uint8_t data[BOOT_DATA_SIZE];
	uint32_t store_addr;
} BOOT_DataTypeDef;

extern BOOT_DataTypeDef buff;

/** 
  * @brief  Bootloader serial structure
  */
typedef struct __BOOT_HandleSeriaInterfaceTypeDef
{
	BOOT_SerialStatusTypeDef status;
	BOOT_SerialErrorTypeDef error;
	BOOT_SerialWarningTypeDef warning;
	UART_HandleTypeDef *huart;
	BOOT_DataTypeDef *buff;
	uint32_t tx_timeout;
	uint32_t rx_timeout;
} BOOT_HandleSeriaInterfaceTypeDef;

extern BOOT_HandleSeriaInterfaceTypeDef boot_serial_if;
/* Exported constants --------------------------------------------------------*/
/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

bool Send_ACK(void);
bool Send_NACK(void);
bool RDP_Is_Active(void);
bool Receive_Start_Addr_and_Check_Checksum(void);
bool Receive_Number_of_Bytes_and_Check_Checksum(void);
bool Send_Data(void);
bool Send_ID(void);
bool Jump_To_Application(void);
bool Check_Is_Memory_Addr(void);
bool Write_Flash_Memory(void);
bool Check_Option_Byte_Address(void);
bool Write_Option_Byte_Area(void);
bool Receive_Number_of_Pages_Erased(void);
bool Global_Erase(void);
bool Receive_Page_Codes_and_Checksum(void);
bool Erase_Corresponding_Pages(void);
bool Generate_Syatem_Reset(void);
bool Receive_Number_Sectors_Codes_and_Checksum(void);
bool Enable_WDP_Sectors(void);
bool Disable_WDP(void);
bool Disable_RDP(void);
bool Enable_RDP(void);
bool Clear_RAM_Memory(void);
bool Receive_Data(void);
bool Check_Number_of_Pages_Erased(void);
void BOOT_Clear_Stack_Count(void);
uint32_t BOOT_Get_Temp_Address(void);
bool BOOT_USART_Send(uint8_t *data, uint16_t size);
bool BOOT_USART_Receive(uint8_t *data, uint16_t size);
#endif /*__BOOTLOADER_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
