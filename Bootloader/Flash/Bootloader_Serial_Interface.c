/**
  ******************************************************************************
  * File Name          : Bootloader_Serial_Interface.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include "Bootloader_Sketch.h"
#include "Bootloader_Serial_Interface.h"
#include "Bootloader_Flash_Interface.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HTONS(x) ((((x)&0x00ffUL) << 8) | (((x)&0xff00UL) >> 8))
#define HTONL(x) ((((x) & 0x000000ffUL) << 24) | \
                 (((x) & 0x0000ff00UL) <<  8) | \
                 (((x) & 0x00ff0000UL) >>  8) | \
                 (((x) & 0xff000000UL) >> 24))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BOOT_DataTypeDef buff;
BOOT_HandleSeriaInterfaceTypeDef boot_serial_if;
/* Private function prototypes -----------------------------------------------*/
static void push(BOOT_DataTypeDef *stack, const stack_t value);
static stack_t pop(BOOT_DataTypeDef *stack);
/* Exported functions --------------------------------------------------------*/
/*###############################################################*/
/*###############################################################* Transmit_Enable -->*/
/*###############################################################*/
void Transmit_Enable(void)
{
	//HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(RS485_DE_Port, RS485_DE_Pin, GPIO_PIN_SET);
}
/*###############################################################*/
/*###############################################################* Receive_Enable -->*/
/*###############################################################*/
void Receive_Enable(void)
{
	//HAL_GPIO_WritePin(RS485_DE_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}
/*###############################################################*/
/*###############################################################* BOOT_USART_Send -->*/
/*###############################################################*/
bool BOOT_USART_Send(uint8_t *data, uint16_t size)
{
	Transmit_Enable();
	uint32_t timeout = boot_serial_if.rx_timeout;
	HAL_StatusTypeDef err = HAL_UART_Transmit(boot_serial_if.huart, data, size, timeout);
	if (err == HAL_OK)
	{
		return true;
	}
	else if (err == HAL_TIMEOUT)
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_TX_TIMEOUT);
		__HAL_UART_CLEAR_OREFLAG(boot_serial_if.huart);
		return false;
	}
	else
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_TX);
		BOOT_SERIAL_DEBUG("ERROR USART\r\n")
		return false;
	}
}
/*###############################################################*/
/*###############################################################* BOOT_USART_Receive -->*/
/*###############################################################*/
bool BOOT_USART_Receive(uint8_t *data, uint16_t size)
{
	Receive_Enable();
	uint32_t timeout = boot_serial_if.rx_timeout;
	HAL_StatusTypeDef err = HAL_UART_Receive(boot_serial_if.huart, data, size, timeout);
	if (err == HAL_OK)
	{
		return true;
	}
	else if (err == HAL_TIMEOUT)
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_RX_TIMEOUT);
		__HAL_UART_CLEAR_OREFLAG(boot_serial_if.huart);
		return false;
	}
	else
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_RX);
		BOOT_SERIAL_DEBUG("ERROR USART\r\n")
		return false;
	}
}

/*###############################################################*/
/*###############################################################* Send_ACK -->*/
/*###############################################################*/
bool Send_ACK(void)
{
	uint32_t chr = ACK;
	return BOOT_USART_Send((uint8_t *)&chr, 1);
}
/*###############################################################*/
/*###############################################################* Send_NACK -->*/
/*###############################################################*/
bool Send_NACK(void)
{
	uint32_t chr = NACK;
	return BOOT_USART_Send((uint8_t *)&chr, 1);
}

/*###############################################################*/
/*###############################################################* Send_ID -->*/
/*###############################################################*/
bool Send_ID(void)
{
	uint32_t ID;
	ID = HAL_GetDEVID();
	ID = HTONS(ID);
	return BOOT_USART_Send((uint8_t *)&ID, 2);
}
/*###############################################################*/
/*###############################################################* RDP_Is_Active -->*/
/*###############################################################*/
bool RDP_Is_Active(void)
{
	if(OB_RDP_LEVEL_0 == FLASH_OB_GetRDP()) return true;
	else return false;
}
/*###############################################################*/
/*###############################################################* Receive_Start_Addr_and_Check_Checksum -->*/
/*###############################################################*/
bool Receive_Start_Addr_and_Check_Checksum(void)
{
	uint8_t chr[5 + 1];  //+ 1 - because in HAL_UART_Receive *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
	if(!BOOT_USART_Receive((uint8_t *)&chr, 5)) {
		boot_serial_if.warning = BOOT_SERIAL_WARNING_TIMEOUT_RECEIVE_START_ADDR;
		BOOT_SERIAL_DEBUG("WARNING TIMEOUT RECEIVE START ADDR\r\n")
		return false;
	}
	if((chr[0]^chr[1]^chr[2]^chr[3]) == chr[4]){
	uint32_t addr;
	memcpy(&addr,&chr,4);
	buff.store_addr = HTONL(addr);
	if(!IS_MEMORY_ADDRESS(buff.store_addr)){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_MEMORY_ADDRESS);
		BOOT_SERIAL_DEBUG("ERROR MEMORY ADDRESS\r\n")
		return false;
	}
	return true;
	}
	else {
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_CHECKSUMM);
		BOOT_SERIAL_DEBUG("ERROR CHECKSUMM\r\n")
		return false;
	}
}
/*###############################################################*/
/*###############################################################* Receive_Number_of_Bytes_and_Check_Checksum -->*/
/*###############################################################*/
bool Receive_Number_of_Bytes_and_Check_Checksum(void)
{
	uint8_t chr[2 + 1];//+ 1 - because in HAL_UART_Receive *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
	if(!BOOT_USART_Receive((uint8_t *)&chr, 2))return false;
	uint8_t num_bytes = chr[0];
	if((chr[0]^chr[1]) != 0xFF){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_CHECKSUMM);
		BOOT_SERIAL_DEBUG("ERROR CHECKSUMM\r\n")
		return false;
	}

	if((IS_MAIN_MEMORY_ADDRESS(buff.store_addr))&&((buff.store_addr + num_bytes) > MAIN_MEMORY_END)){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_SIZE_DATA);
		BOOT_SERIAL_DEBUG("ERROR SIZE DATA\r\n")
		return false;
	}

	push(&buff, num_bytes);
	return true;

}
/*###############################################################*/
/*###############################################################* Send_Data -->*/
/*###############################################################*/
bool Send_Data(void)
{
	int num_bytes = pop(&buff);
	pop(&buff);

	//Note: do not use memory addresses!!!
	//return BOOT_USART_Send((uint8_t *)buff.store_addr, num_bytes+1);

	//because function HAL_UART_Transmit has a bug:
    //tmp = (uint16_t*) pData; //!!!! - not (uint8_t*) pData
    //huart->Instance->DR = (*tmp & (uint16_t)0x01FF);
	//when the data address is equal to the last byte in the flash (0x080FFFFF) or last last byte in Option bytes area
	//an attempt is made to read the address 0x08100000.This causes an Hard fault.
	//Therefore, write the data to the buffer before sending
	memcpy(buff.data,(uint8_t *)buff.store_addr,num_bytes+1);
	return BOOT_USART_Send(buff.data, num_bytes+1);

}
/*###############################################################*/
/*###############################################################* Receive_Data -->*/
/*###############################################################*/
bool Receive_Data(void)
{
	uint16_t num;
	uint16_t ch_summ;
	if(!BOOT_USART_Receive((uint8_t *)&num, 1)) return false;
	if(!BOOT_USART_Receive((uint8_t *)&buff.data, num + 1)) return false;
	if(!BOOT_USART_Receive((uint8_t *)&ch_summ, 1)) return false;
	push(&buff, num);
	uint8_t xor_data = num;
	for(int i=0; i < (num + 1); i++){
	xor_data^=buff.data[i];
	}
	if(ch_summ!=xor_data)
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_CHECKSUMM);
		BOOT_SERIAL_DEBUG("ERROR CHECKSUMM\r\n")
		return false;
	}
	return true;
}
/*###############################################################*/
/*###############################################################* Jump_To_Application -->*/
/*###############################################################*/
bool Jump_To_Application(void)
{
	boot_serial_if.status = BOOT_SERIAL_STATUS_JUMP_TO_APPLICATION;
	return true;
}
//Write Memory Command
/*###############################################################*/
/*###############################################################* Check_Is_Flash_Addr -->*/
/*###############################################################*/
bool Check_Is_Memory_Addr(void)
{

	if(IS_MEMORY_ADDRESS(buff.store_addr))
	{
		if(IS_MAIN_MEMORY_ADDRESS(buff.store_addr))
		{
			uint16_t sector = Get_Sector(buff.store_addr);

			if(FLASH_OB_IsWRP(sector))
			{
				BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_WRITE_IN_PROTECT_SECTOR);
				BOOT_SERIAL_DEBUG("ERROR TRYING TO WRITE PROTECTED SECTOR\r\n")
				return false;
			}
			if(sector == FLASH_BOOTLOADER_SECTOR)
			{
				BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_WRITE_BOOT_PAGE);
				/*Note:
				No error is returned when performing write operations on write-protected sectors.No error is
				returned when the start address is invalid.*/
				BOOT_SERIAL_DEBUG("ERROR TRYING TO WRITE THE BOOTLOADER PAGE\r\n")
				return false;
			}
		}
		return true;
	}
	else
	{
		boot_serial_if.warning = BOOT_SERIAL_WARNING_CHECK_WRITE_ADDR;
		BOOT_SERIAL_DEBUG("WARNING CHECK WRITE ADDR\r\n")
		return false;
	}

}
/*###############################################################*/
/*###############################################################* Write_Flash_Memory -->*/
/*###############################################################*/
bool Write_Flash_Memory(void)
{
	uint32_t size = pop(&buff);
	uint16_t sector = Get_Sector(buff.store_addr);

	if(FLASH_OB_IsWRP(sector))
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_WRITE_IN_PROTECT_SECTOR);
		BOOT_SERIAL_DEBUG("ERROR TRYING TO WRITE PROTECTED SECTOR\r\n")
		return false;
	}
	if(sector == FLASH_BOOTLOADER_SECTOR)
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_WRITE_BOOT_PAGE);
		/*Note: 
		No error is returned when performing write operations on write-protected sectors.No error is
		returned when the start address is invalid.*/
		BOOT_SERIAL_DEBUG("ERROR TRYING TO WRITE THE BOOTLOADER PAGE\r\n")
		return false;
	}else if(!IS_FLASH_SECTOR(sector)){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_DATA);
		BOOT_SERIAL_DEBUG("ERROR NUM SECTOR FOR WRITE\r\n")
		return false;
	}

	if(Write_Block_Flash(size + 1, buff.store_addr, (uint32_t *)buff.data)!=BOOT_FLASH_ERROR_OK)
		return false;
	return true;
}
/*###############################################################*/
/*###############################################################* Check_Option_Byte_Address -->*/
/*###############################################################*/
bool Check_Option_Byte_Address(void)
{
	if(((buff.store_addr) >= OPTIONS_BYTES_BASE) && ((buff.store_addr) <= OPTIONS_BYTES_END)) return true;
	return false;
}
/*###############################################################*/
/*###############################################################* Write_Option_Byte_Area -->*/
/*###############################################################*/
bool Write_Option_Byte_Area(void)
{
	uint32_t size = pop(&buff);
	if((size + 1)>16)return false;
	for(int i=0; i < (size + 1); i++)
	*(__IO uint8_t *)(OPTIONS_BYTES_BASE + i) = buff.data[i];
	return true;
}
//Erase Memory Command
/*###############################################################*/
/*###############################################################* Receive_Number_of_Pages_Erased -->*/
/*###############################################################*/
bool Receive_Number_of_Pages_Erased(void)
{
	uint32_t num; //uint32_t - because in HAL_UART_Receive *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
	if(!BOOT_USART_Receive((uint8_t *)&num, 2))return false;
	push(&buff, HTONS(num));
	return true;
}
/*###############################################################*/
/*###############################################################* Check_Number_of_Pages_Erased -->*/
/*###############################################################*/
bool Check_Number_of_Pages_Erased(void)
{
	uint16_t num = pop(&buff);
	push(&buff, num);

	if((num&0xFFF0) == 0xFFF0)return true;
	return false;
}
/*###############################################################*/
/*###############################################################* Global_Erase -->*/
/*###############################################################*/
bool Global_Erase(void)
{
	if(BOOT_FLASH_ERROR_OK != Erase_Sectors(FLASH_SECTOR_1, 11))
	{
		return false;
	}
	return true;
}
/*###############################################################*/
/*###############################################################* Receive_Page_Codes_and_Checksum -->*/
/*###############################################################*/
bool Receive_Page_Codes_and_Checksum(void)
{
	uint16_t num = pop(&buff);
	uint16_t ch_summ; //uint16_t - because in HAL_UART_Receive *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
	if(!BOOT_USART_Receive((uint8_t *)&buff.data, (num + 1)*2))return false;
	if(!BOOT_USART_Receive((uint8_t *)&ch_summ, 1))return false;
	push(&buff, num);
	uint8_t xor_data = num;
	for(int i=0; i <(num+1)*2; i++){
	xor_data^=buff.data[i];
	}
	if(ch_summ!=xor_data)
	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_CHECKSUMM);
		BOOT_SERIAL_DEBUG("ERROR CHECKSUMM\r\n")
		return false;
	}
	return true;
}
/*###############################################################*/
/*###############################################################* Erase_Corresponding_Pages -->*/
/*###############################################################*/
bool Erase_Corresponding_Pages(void)
{
	uint32_t num = pop(&buff);
	uint16_t *p = (uint16_t *)&buff.data;
	for(int i = 0; i < (num+1); i++){
		uint16_t sector = HTONS(*p);
		if(sector == FLASH_BOOTLOADER_SECTOR)
		{
			BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_ERASE_BOOT_PAGE);
			/*Note: No error is returned when performing erase operations on write protected sectors.*/
			BOOT_SERIAL_DEBUG("ERROR TRYING TO ERASE THE BOOTLOADER PAGE\r\n")
			return false;
		}else if(!IS_FLASH_SECTOR(sector)){
			BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_DATA);
			BOOT_SERIAL_DEBUG("ERROR NUM SECTOR FOR ERASE\r\n")
			return false;
		}
		else if(FLASH_OB_IsWRP(sector))
		{
			BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_ERASE_IN_PROTECT_SECTOR);
			BOOT_SERIAL_DEBUG("ERROR TRYING TO ERASE PROTECTED SECTOR\r\n")
			return false;
		}
		if(BOOT_FLASH_ERROR_OK != Erase_Sectors(sector, 1)) return false;
		p++;
	}
	return true;
}

//Write Protect/Unprotect Command
/*###############################################################*/
/*###############################################################* Generate_Syatem_Reset -->*/
/*###############################################################*/
bool Generate_Syatem_Reset(void)
{
//	NVIC_SystemReset();
	return true;
}

/*###############################################################*/
/*###############################################################* Enable_WDP_Sectors -->*/
/*###############################################################*/
bool Enable_WDP_Sectors(void)
{
	uint32_t num = pop(&buff);
	uint16_t wdp_sectors = 0;
	for(int i=0; i < (num + 1); i++){
		if(buff.data[i]!=FLASH_BOOTLOADER_SECTOR)
		{
			wdp_sectors |= (1<<buff.data[i]);
		}
		else
		{
			BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_ENABLE_WDP_BOOT_SECTOR);
			BOOT_SERIAL_DEBUG("ERROR TRYING TO ENABLE WDP THE BOOTLOADER PAGE\r\n")
			return false;
		}
	}
	if (wdp_sectors==0)	{
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_ENABLE_WDP);
		BOOT_SERIAL_DEBUG("ERROR ENABLE WDP\r\n")
		return false;
	}

	if (HAL_OK != FLASH_OB_EnableWRP(wdp_sectors, FLASH_BANK_1)){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_ENABLE_WDP);
		BOOT_SERIAL_DEBUG("ERROR ENABLE WDP\r\n")
		return false;
	}
	return true;
}
/*###############################################################*/
/*###############################################################* Disable_WDP_Sectors -->*/
/*###############################################################*/
bool Disable_WDP(void)
{
	if(HAL_OK != FLASH_OB_DisableWRP(0x0FFE, FLASH_BANK_1)){
		BOOT_SERIAL_SET_ERROR(BOOT_SERIAL_ERROR_DISABLE_WDP);
		BOOT_SERIAL_DEBUG("ERROR DISABLE WDP\r\n")
		return false;
	}
	return true;
}
//Readout Protect/Unprotect Command
/*###############################################################*/
/*###############################################################* Disable_RDP -->*/
/*###############################################################*/
bool Disable_RDP(void)
{
	if(HAL_OK != FLASH_OB_RDP_LevelConfig(OB_RDP_LEVEL_0)) return false;
	return true;
}
/*###############################################################*/
/*###############################################################* Enable_RDP -->*/
/*###############################################################*/
bool Enable_RDP(void)
{
	if(HAL_OK != FLASH_OB_RDP_LevelConfig(OB_RDP_LEVEL_1)) return false;
	return true;
}
/*###############################################################*/
/*###############################################################* Clear_RAM_Memory -->*/
/*###############################################################*/
bool Clear_RAM_Memory(void)
{
	//when will the python script
	return true;
}
/*###############################################################*/
/*###############################################################* push -->*/
/*###############################################################*/
static void push(BOOT_DataTypeDef *stack, const stack_t value) {
    if (stack->stack_cnt >= BOOT_STACK_MAX_SIZE) {
        return;
    }
    stack->stack[stack->stack_cnt] = value;
    stack->stack_cnt++;
}

/*###############################################################*/
/*###############################################################* pop -->*/
/*###############################################################*/
static stack_t pop(BOOT_DataTypeDef *stack) {
    if (stack->stack_cnt == 0) {
        return 0;
    }
    stack->stack_cnt--;
    return stack->stack[stack->stack_cnt];
}
/*###############################################################*/
/*###############################################################* BOOT_Clear_Stack_Count -->*/
/*###############################################################*/
void BOOT_Clear_Stack_Count(void)
{
	buff.stack_cnt = 0;
}
/*###############################################################*/
/*###############################################################* BOOT_Get_Temp_Address -->*/
/*###############################################################*/
uint32_t BOOT_Get_Temp_Address(void)
{
	return buff.store_addr;
}
