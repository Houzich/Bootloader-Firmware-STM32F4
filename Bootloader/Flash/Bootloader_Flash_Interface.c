/**
  ******************************************************************************
  * File Name          : Bootloader_Flash_Interface.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "Bootloader_Sketch.h"
#include "Bootloader_Serial_Interface.h"
#include "Bootloader_Flash_Interface.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BOOT_HandleFlashInterfaceTypeDef boot_flash_if;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/*###############################################################*/
/*###############################################################* Get_Sector -->*/
/*###############################################################*/
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t Get_Sector(uint32_t address)
{
  uint32_t sector = 0;

  if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;
  }
  else
  {
    sector = FLASH_SECTOR_NO;
  }
  return sector;
}
/*###############################################################*/
/*###############################################################* Get_Sector_Size -->*/
/*###############################################################*/
/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
uint32_t Get_Sector_Size(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;
  if ((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) ||
      (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if  (Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}
/*###############################################################*/
/*###############################################################* Read_Word_Flash -->*/
/*###############################################################*/
uint32_t Read_Word_Flash(uint32_t address)
{
  return (*(__IO uint32_t *)address);
}
/*###############################################################*/
/*###############################################################* Write_Block_Flash -->*/
/*###############################################################*/
uint32_t Write_Block_Flash(uint32_t size, uint32_t address_flash, uint32_t *address_data)
{
  __IO uint32_t Addr_flash;
  __IO uint32_t *Addr_data;

  if (!IS_FLASH_SECTOR(Get_Sector(address_flash)))
  {
	BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_SECTOR);
    BOOT_FLASH_DEBUG("ERROR FLASH SECTOR\r\n");
    return BOOT_FLASH_ERROR_SECTOR;
  }

  /* Unlock the Flash to enable the flash control register access *************/
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
	BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_WRITE);
    return BOOT_FLASH_ERROR_WRITE;
  }

  /* FLASH Block program */
  Addr_flash = address_flash;
  Addr_data = address_data;
  while (Addr_flash < address_flash + size)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Addr_flash, *Addr_data) == HAL_OK)
    {
      Addr_flash += 4;
      Addr_data += 1;
    }
    else
    {
      HAL_FLASH_Lock();
      BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_WRITE);
      return BOOT_FLASH_ERROR_WRITE;
    } /* Error occurred while writing data in Flash memory.*/
  }

  /* Check the correctness of written data */
  Addr_flash = address_flash;
  Addr_data = address_data;
  while (Addr_flash < address_flash + size)
  {
    if ((*(__IO uint32_t *)Addr_flash) != *Addr_data)
    {
      HAL_FLASH_Lock();
      BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_CHECK);
      return BOOT_FLASH_ERROR_CHECK;
    }
    Addr_flash += 4;
    Addr_data += 1;
  }

  HAL_FLASH_Lock();
  return BOOT_FLASH_ERROR_OK;
}
/*###############################################################*/
/*###############################################################* Erase_Sectors -->*/
/*###############################################################*/
uint32_t Erase_Sectors(uint32_t first_sector, uint32_t nb_of_sectors)
{
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t FirstSector = 0, NbOfSectors = 0;
  uint32_t SECTORError = 0;

  if (HAL_FLASH_Unlock() != HAL_OK)
    goto error;
  /* Get the 1st sector to erase */
  FirstSector = first_sector;
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = nb_of_sectors;
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) == HAL_OK)
  {
    HAL_FLASH_Lock();
    return BOOT_FLASH_ERROR_OK;
  }

error:
  HAL_FLASH_Lock();
  BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_ERASE);
  return BOOT_FLASH_ERROR_ERASE;
}

/*###############################################################*/
/*###############################################################* Jump_To_Addres_Flash -->*/
/*###############################################################*/
void Jump_To_Address_Flash(uint32_t addr)
{
  void (*SysMemJump)(void);

  HAL_RCC_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  __disable_irq();

  __HAL_SYSCFG_REMAPMEMORY_FLASH();

  SysMemJump = (void (*)(void))(*((uint32_t *)(addr + 4)));

  __set_MSP(*(uint32_t *)addr);

  SysMemJump();
}
/*###############################################################*/
/*###############################################################* FLASH_OB_EnableWRP -->*/
/*###############################################################*/
#define FLASH_TIMEOUT_VALUE 50000U /* 50 s */
HAL_StatusTypeDef FLASH_OB_EnableWRP(uint32_t WRPSector, uint32_t Banks)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_OB_WRP_SECTOR(WRPSector));
  assert_param(IS_FLASH_BANK(Banks));
  /* Wait for last operation to be completed */


#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) ||\
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) ||\
    defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) ||\
    defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
  HAL_FLASHEx_OB_DeSelectPCROP();
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F401xC || STM32F401xE || STM32F410xx ||\
          STM32F411xE || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx ||
          STM32F413xx || STM32F423xx */
  if (HAL_FLASH_OB_Unlock() != HAL_OK) goto error;
  if (FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) != HAL_OK) goto error;

    *(__IO uint16_t *)OPTCR_BYTE2_ADDRESS &= (~WRPSector);

  if (HAL_FLASH_OB_Launch() != HAL_OK) goto error;
  if (HAL_FLASH_OB_Lock() != HAL_OK) goto error;
  return status;
  error:
  HAL_FLASH_OB_Lock();
    BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_ENABLE_WRP);
    return BOOT_FLASH_ERROR_ENABLE_WRP;
}
/*###############################################################*/
/*###############################################################* FLASH_OB_DisableWRP -->*/
/*###############################################################*/
HAL_StatusTypeDef FLASH_OB_DisableWRP(uint32_t WRPSector, uint32_t Banks)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_OB_WRP_SECTOR(WRPSector));
  assert_param(IS_FLASH_BANK(Banks));

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) ||\
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) ||\
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) ||\
    defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) ||\
    defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
  HAL_FLASHEx_OB_DeSelectPCROP();
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F401xC || STM32F401xE || STM32F410xx ||\
          STM32F411xE || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx || STM32F412Rx || STM32F412Cx ||
          STM32F413xx || STM32F423xx */
  if (HAL_FLASH_OB_Unlock() != HAL_OK) goto error;
  if (FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) != HAL_OK) goto error;

    *(__IO uint16_t *)OPTCR_BYTE2_ADDRESS |= (uint16_t)(WRPSector);

  if (HAL_FLASH_OB_Launch() != HAL_OK) goto error;
  if (HAL_FLASH_OB_Lock() != HAL_OK) goto error;
  return status;
  error:
  HAL_FLASH_OB_Lock();
    BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_DISABLE_WRP);
    return BOOT_FLASH_ERROR_DISABLE_WRP;
}

/*###############################################################*/
/*###############################################################* FLASH_OB_DisableWRP -->*/
/*###############################################################*/
static uint16_t FLASH_OB_GetWRP(void)
{
  /* Return the FLASH write protection Register value */
  return (*(__IO uint16_t *)(OPTCR_BYTE2_ADDRESS));
}
/*###############################################################*/
/*###############################################################* FLASH_OB_IsWRP -->*/
/*###############################################################*/
bool FLASH_OB_IsWRP(uint32_t WRPSector)
{
	uint16_t optcr_byte2 = FLASH_OB_GetWRP();
	if(optcr_byte2&(1<<WRPSector)) return false;
	return true;
}
/*###############################################################*/
/*###############################################################* FLASH_OB_RDP_LevelConfig -->*/
/*###############################################################*/
/**
  * @brief  Set the read protection level.
  * @param  Level specifies the read protection level.
  *          This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  *   
  * @note WARNING: When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  *    
  * @retval HAL Status
  */
HAL_StatusTypeDef FLASH_OB_RDP_LevelConfig(uint8_t Level)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_OB_RDP_LEVEL(Level));
  if (HAL_FLASH_OB_Unlock() != HAL_OK) goto error;
  /* Wait for last operation to be completed */
  if (FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE) != HAL_OK) goto error;

  if (status == HAL_OK)
  {
    *(__IO uint8_t *)OPTCR_BYTE1_ADDRESS = Level;
  }

  if (HAL_FLASH_OB_Launch() != HAL_OK) goto error;
  if (HAL_FLASH_OB_Lock() != HAL_OK) goto error;
  return status;
  error:
  HAL_FLASH_OB_Lock();
    BOOT_FLASH_SET_ERROR(BOOT_FLASH_ERROR_RDP_LEVEL_CONFIG);
    return BOOT_FLASH_ERROR_RDP_LEVEL_CONFIG;
}
/*###############################################################*/
/*###############################################################* FLASH_OB_GetRDP -->*/
/*###############################################################*/
/**
  * @brief  Returns the FLASH Read Protection level.
  * @retval FLASH ReadOut Protection Status:
  *         This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  */
uint8_t FLASH_OB_GetRDP(void)
{
  uint8_t readstatus = OB_RDP_LEVEL_0;

  if (*(__IO uint8_t *)(OPTCR_BYTE1_ADDRESS) == (uint8_t)OB_RDP_LEVEL_2)
    readstatus = OB_RDP_LEVEL_2;
  else if (*(__IO uint8_t *)(OPTCR_BYTE1_ADDRESS) == (uint8_t)OB_RDP_LEVEL_1)
    readstatus = OB_RDP_LEVEL_1;
  else
    readstatus = OB_RDP_LEVEL_0;

  return readstatus;
}
