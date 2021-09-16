/**
  ******************************************************************************
  * File Name          : Bootloader_Sketch.c
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "Bootloader_Sketch.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ___________DECISION_BLOCK(x, fn, true_fn, true_step, false_fn, false_step)   \
		.step[x].link_fn = fn,   \
		.step[x].if_true_fn = true_fn,   \
		.step[x].next_true_step = true_step,   \
		.step[x].if_false_fn = false_fn,   \
		.step[x].next_false_step = false_step,


#define __DECISION_ASC_NACK_BLOCK(x,fn)   \
		.step[x].link_fn = fn,   \
		.step[x].if_true_fn = Send_ACK,   \
		.step[x].if_false_fn = Send_NACK,   \
		.step[x].next_true_step = NEXT_STEP,   \
		.step[x].next_false_step = END_STEP,

#define ____________PROCESS_BLOCK(x,fn)   \
		.step[x].link_fn = fn,   \
		.step[x].if_true_fn = NULL,   \
		.step[x].if_false_fn = NULL,   \
		.step[x].next_true_step = NEXT_STEP,   \
		.step[x].next_false_step = NEXT_STEP,
		   \
#define __________SEND_BYTE_BLOCK(x,byte)   \
		.step[x].message[0] = byte,   \
		.step[x].link_fn = NULL,   \
		.step[x].if_true_fn = NULL,   \
		.step[x].if_false_fn = NULL,   \
		.step[x].next_true_step = NEXT_STEP,   \
		.step[x].next_false_step = NEXT_STEP,   \

#define BOOT_SESSION_SIZE (sizeof(BOOT_Session) / sizeof(BOOT_CommandTypeDef *))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BOOT_HandleTypeDef boot;
static const BOOT_CommandTypeDef GetCommandScript =
	{
		.id = ID_COMMAND_GET,
		__________SEND_BYTE_BLOCK(0,ACK)				//Byte = ACK,		   //
		__________SEND_BYTE_BLOCK(1,11)					//N = 11 = the number of bytes to follow – 1 except current and ACKs.
		__________SEND_BYTE_BLOCK(2,0x30)				//Bootloader version (0 < Version < 255), example: 0x10 = Version 1.0
		__________SEND_BYTE_BLOCK(3,GET)				//–Get command
		__________SEND_BYTE_BLOCK(4,GET_VERSION)		//–Get Version and Read Protection Status
		__________SEND_BYTE_BLOCK(5,GET_ID)				//–Get ID
		__________SEND_BYTE_BLOCK(6,READ_MEMORY)		//–Read Memory command
		__________SEND_BYTE_BLOCK(7,GO)					//–Go command
		__________SEND_BYTE_BLOCK(8,WRITE_MEMORY)		//–Write Memory command
		__________SEND_BYTE_BLOCK(9,EXTENDED_ERASE)		// or 0x44–Erase command or Extended Erase command (these commands are exlusive)
		__________SEND_BYTE_BLOCK(10,WRITE_PROTECT)		//–Write Protect command
		__________SEND_BYTE_BLOCK(11,WRITE_UNPROTECT)   //–Write Unprotect command
		__________SEND_BYTE_BLOCK(12,READOUT_PROTECT)   //–Readout Protect command
		__________SEND_BYTE_BLOCK(13,READOUT_UNPROTECT) //–Readout Unprotect command
		__________SEND_BYTE_BLOCK(14,ACK)			//ACK
		.finish = 15,};

static const BOOT_CommandTypeDef GetVersionCommandScript =
	{
		.id = ID_COMMAND_GET_VERSION,
		__________SEND_BYTE_BLOCK(0,ACK)  //ACK
		__________SEND_BYTE_BLOCK(1,0x30) //Bootloader version (0 < Version <= 255), example: 0x10 = Version 1.0
		__________SEND_BYTE_BLOCK(2,0x00) //Option byte 1: 0x00 to keep the compatibility with generic bootloader protocol
		__________SEND_BYTE_BLOCK(3,0x00) //Option byte 2: 0x00 to keep the compatibility with generic bootloader protocol
		__________SEND_BYTE_BLOCK(4,ACK)  //ACK
		.finish = 5,};

static const BOOT_CommandTypeDef GetIDCommandScript =
	{
		.id = ID_COMMAND_GET_ID,
		__________SEND_BYTE_BLOCK(0,ACK)  //ACK
		__________SEND_BYTE_BLOCK(1,1)    //N = the number of bytes – 1 (N = 1 for STM32), except for current byte and ACKs.
		__DECISION_ASC_NACK_BLOCK(2,Send_ID)
		.finish = 3,};

static const BOOT_CommandTypeDef ReadoutUnprotectCommandScript =
{
		.id = ID_COMMAND_READOUT_UNPROTECT,
		__________SEND_BYTE_BLOCK(0, ACK)
		____________PROCESS_BLOCK(1, Disable_RDP)
		__________SEND_BYTE_BLOCK(2, ACK)
		____________PROCESS_BLOCK(3, Clear_RAM_Memory)
		____________PROCESS_BLOCK(4, Generate_Syatem_Reset)
		.finish = 5,
};

static const BOOT_CommandTypeDef ReadoutProtectCommandScript =
	{
		.id = ID_COMMAND_READOUT_PROTECT,
		__DECISION_ASC_NACK_BLOCK(0, RDP_Is_Active)
		____________PROCESS_BLOCK(1, Enable_RDP)
		__________SEND_BYTE_BLOCK(2, ACK)
		____________PROCESS_BLOCK(4, Generate_Syatem_Reset)
		.finish = 5,
};

static const BOOT_CommandTypeDef WriteUnprotectCommandScript =
	{
		.id = ID_COMMAND_WRITE_UNPROTECT,
		__DECISION_ASC_NACK_BLOCK(0, RDP_Is_Active)
		___________DECISION_BLOCK(1, Disable_WDP, NULL, NEXT_STEP, Send_NACK, END_STEP)
		__________SEND_BYTE_BLOCK(2, ACK)
		____________PROCESS_BLOCK(3, Generate_Syatem_Reset)
		.finish = 4,
};

static const BOOT_CommandTypeDef WriteProtectCommandScript =
	{
		.id = ID_COMMAND_WRITE_PROTECT,
		__DECISION_ASC_NACK_BLOCK(0, RDP_Is_Active)
		___________DECISION_BLOCK(1, Receive_Data, Enable_WDP_Sectors, NEXT_STEP, Send_NACK, END_STEP)
		__________SEND_BYTE_BLOCK(2, ACK)
		____________PROCESS_BLOCK(3, Generate_Syatem_Reset)
		.finish = 4,
};

static const BOOT_CommandTypeDef EraseMemoryCommandScript =
	{
		.id = ID_COMMAND_ERASE_MEMORY,
		__DECISION_ASC_NACK_BLOCK(0, RDP_Is_Active)
		____________PROCESS_BLOCK(1, Receive_Number_of_Pages_Erased)
		___________DECISION_BLOCK(2, Check_Number_of_Pages_Erased, Global_Erase, 5, NULL, NEXT_STEP)
		___________DECISION_BLOCK(3, Receive_Page_Codes_and_Checksum, NULL, NEXT_STEP, Send_NACK, END_STEP)
		___________DECISION_BLOCK(4, Erase_Corresponding_Pages, NULL, NEXT_STEP, Send_NACK, END_STEP)
		__________SEND_BYTE_BLOCK(5, ACK)
		.finish = 6,
};

static const BOOT_CommandTypeDef WriteMemoryCommandScript =
	{
		.id = ID_COMMAND_WRITE_MEMORY,
		__DECISION_ASC_NACK_BLOCK(0, RDP_Is_Active)
		__DECISION_ASC_NACK_BLOCK(1, Receive_Start_Addr_and_Check_Checksum)
		____________PROCESS_BLOCK(2, Receive_Data)	//Receive the number of bytes to be written (1 byte), the data (N + 1 bytes) and checksum
		___________DECISION_BLOCK(3, Check_Is_Memory_Addr, Write_Flash_Memory, 5, NULL, NEXT_STEP)
		___________DECISION_BLOCK(4, Check_Option_Byte_Address, Write_Option_Byte_Area, NEXT_STEP, NULL, END_STEP)
		__________SEND_BYTE_BLOCK(5, ACK)
		.finish = 6,
};
static const BOOT_CommandTypeDef ReadMemoryCommandScript =
	{
		.id = ID_COMMAND_READ_MEMORY,
		__DECISION_ASC_NACK_BLOCK(0,RDP_Is_Active)
		__DECISION_ASC_NACK_BLOCK(1,Receive_Start_Addr_and_Check_Checksum)
		__DECISION_ASC_NACK_BLOCK(2,Receive_Number_of_Bytes_and_Check_Checksum)	//Receive the number of bytes to be transmitted – 1 (N bytes) and for its complemented byte
		____________PROCESS_BLOCK(3,Send_Data)						//Send Gata to the host
		.finish = 4,
};
static const BOOT_CommandTypeDef GoCommandScript =
	{
		.id = ID_COMMAND_GO,
		__DECISION_ASC_NACK_BLOCK(0,RDP_Is_Active)
		__DECISION_ASC_NACK_BLOCK(1,Receive_Start_Addr_and_Check_Checksum)
		____________PROCESS_BLOCK(2,Jump_To_Application)
		.finish = 3,
};

const BOOT_CommandTypeDef *BOOT_Session[] =
	{
		&GetCommandScript,
		&GetVersionCommandScript,
		&GetIDCommandScript,
		&ReadoutUnprotectCommandScript,
		&ReadoutProtectCommandScript,
		&WriteUnprotectCommandScript,
		&WriteProtectCommandScript,
		&EraseMemoryCommandScript,
		&WriteMemoryCommandScript,
		&GoCommandScript,
		&ReadMemoryCommandScript,
};
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*###############################################################*/
/*###############################################################* Bootloader_Init -->*/
/*###############################################################*/
void Bootloader_Init(UART_HandleTypeDef *huart)
{
	boot.status = BOOT_STATUS_START;
	boot.error = BOOT_ERROR_OK;
	boot.boot_serial_if = &boot_serial_if;
	boot.boot_flash_if = &boot_flash_if;
	boot.session_curr = NULL;
	boot.step_curr = NULL;

	boot.boot_serial_if->buff = &buff;
	boot.boot_serial_if->huart = huart;
	boot.boot_serial_if->tx_timeout = 500;
	boot.boot_serial_if->rx_timeout = 500;
	boot.boot_serial_if->status = BOOT_SERIAL_STATUS_READY;
	boot.boot_serial_if->error = BOOT_SERIAL_ERROR_OK;
	//boot.boot_serial_if->warning = BOOT_SERIAL_WARNING_OK;

	boot.boot_flash_if->status = BOOT_FLASH_STATUS_READY;
	boot.boot_flash_if->error = BOOT_FLASH_ERROR_OK;
}

/*###############################################################*/
/*###############################################################* Check_Error_Bootloader -->*/
/*###############################################################*/
static bool Check_Bootloader_Status(void)
{
	/*------ Check Status ------*/
		if ((boot.boot_serial_if->status == BOOT_SERIAL_STATUS_ERROR)||(boot.boot_flash_if->status == BOOT_FLASH_STATUS_ERROR))
		{
			/*------ Check Error ------*/
			if (boot.boot_serial_if->error != BOOT_SERIAL_ERROR_OK)
			{
				boot.error = BOOT_ERROR_SERIAL; //Set bootloader serial error
				//Fatal error?
				switch(boot.boot_serial_if->error)
				{
					case BOOT_SERIAL_ERROR_TX: 						boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_SERIAL_ERROR_RX: 						boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_SERIAL_ERROR_TX_TIMEOUT: 				boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_RX_TIMEOUT: 				boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_CHECKSUMM: 				boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_DATA: 					boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_ERASE_BOOT_PAGE: 		boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_WRITE_BOOT_PAGE: 		boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_DISABLE_WDP_BOOT_SECTOR: boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_ENABLE_WDP_BOOT_SECTOR: 	boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_WRITE_IN_PROTECT_SECTOR: boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_ERASE_IN_PROTECT_SECTOR: boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_ENABLE_WDP: 				boot.status = BOOT_STATUS_ERROR; break;
					case BOOT_SERIAL_ERROR_DISABLE_WDP: 			boot.status = BOOT_STATUS_ERROR; break;
					default: 										boot.status = BOOT_STATUS_FATAL_ERROR; break;
				}
			}
			if (boot.boot_flash_if->error != BOOT_FLASH_ERROR_OK)
			{
				boot.error = BOOT_ERROR_FLASH; //Set bootloader flash error
				//Fatal error?
				switch(boot.boot_flash_if->error)
				{
					case BOOT_FLASH_ERROR_ERASE: 			boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_WRITE: 			boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_CHECK: 			boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_OPTIONS_BYTES: 	boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_PROTECTION: 		boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_SECTOR: 			boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_ENABLE_WRP: 		boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_DISABLE_WRP: 		boot.status = BOOT_STATUS_FATAL_ERROR; break;
					case BOOT_FLASH_ERROR_RDP_LEVEL_CONFIG: boot.status = BOOT_STATUS_FATAL_ERROR; break;
					default: 								boot.status = BOOT_STATUS_FATAL_ERROR; break;
				}
			}
		}
		else if (boot.boot_serial_if->status == BOOT_SERIAL_STATUS_JUMP_TO_APPLICATION)
		{
			boot.status = BOOT_STATUS_JUMP_TO_APPLICATION;
		}

		if(boot.status == BOOT_STATUS_FATAL_ERROR) return true;
		if(boot.status == BOOT_STATUS_JUMP_TO_APPLICATION) return true;
	return false;
}
/*###############################################################*/
/*###############################################################* Bootloader_Command_Step_By_Step -->*/
/*###############################################################*/
void Bootloader_Command_Step_By_Step(BOOT_HandleTypeDef *boot)
{
	volatile uint16_t cnt_step = 0;
	bool ret = false;
	int next;
	BOOT_CommandTypeDef *session = boot->session_curr;
	BOOT_Clear_Stack_Count();
	while (cnt_step < session->finish)
	{
		if(Check_Bootloader_Status())return;

		BOOT_CommandStepTypeDef *step = &session->step[cnt_step];
		boot->step_curr = step;
		bool (*fn)(void) = step->link_fn;
		bool (*fn_true)(void) = step->if_true_fn;
		bool (*fn_false)(void) = step->if_false_fn;
		if(fn == NULL){
		BOOT_USART_Send((uint8_t *)&step->message[0], 1);
		cnt_step++;
		}else{
			ret=fn();
			if(ret){
				if(fn_true != NULL) fn_true();
				next = step->next_true_step;
				if(next==END_STEP) cnt_step = session->finish;
				else if(next == NEXT_STEP) cnt_step++;
				else cnt_step = next;
			} else {
				if(fn_false != NULL) fn_false();
				next = step->next_false_step;
				if(next == END_STEP) cnt_step = session->finish;
				else if(next == NEXT_STEP) cnt_step++;
				else cnt_step = next;
			}
		}
	}

}
/*###############################################################*/
/*###############################################################* Bootloader_Listen_Command -->*/
/*###############################################################*/
uint32_t Bootloader_Listen_Command(void)
{
	volatile uint32_t cmd_id = 0; //uint32_t - because in HAL_UART_Receive *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
	boot.status = BOOT_STATUS_WAIT_ID_SESSION;
	BOOT_USART_Receive((uint8_t *)&cmd_id, 2);
	if(boot.boot_serial_if->error == BOOT_SERIAL_ERROR_RX_TIMEOUT)
	{
		if(((uint16_t)cmd_id==0x007F)||((uint16_t)cmd_id==0x7F00)){
			Send_ACK();
		}else if(cmd_id!=0){
			Send_NACK();
		}

		boot.boot_serial_if->error = BOOT_SERIAL_ERROR_OK;
		boot.boot_serial_if->status = BOOT_SERIAL_STATUS_READY;
	}

	for (int idx = 0; idx < BOOT_SESSION_SIZE; idx++)
	{
		if (BOOT_Session[idx]->id == (uint16_t)cmd_id)
		{
			boot.status = BOOT_STATUS_BUSY_SESSION;
			boot.session_curr = (BOOT_CommandTypeDef *)BOOT_Session[idx];
			Bootloader_Command_Step_By_Step(&boot);
		}
		if(Check_Bootloader_Status())return 1;
	}
	return 0;
}
/*###############################################################*/
/*###############################################################* Bootloader_Loop -->*/
/*###############################################################*/
void Bootloader_Loop(void)
{
	//Send ASK, start bootloader mode
	uint8_t chr = ACK;
	BOOT_USART_Send((uint8_t *)&chr, 1);

	while (1)
	{
		Bootloader_Listen_Command();
		if (boot.status == BOOT_STATUS_ERROR)
		{
			Bootloader_Init(boot.boot_serial_if->huart);
		}
		if (boot.status == BOOT_STATUS_FATAL_ERROR)
		{
			BOOT_DEBUG("BOOTLOADER FATAL ERROR\r\n");
			while(1){}
		}
		else if (boot.status == BOOT_STATUS_JUMP_TO_APPLICATION)
		{
			Jump_To_Address_Flash(BOOT_Get_Temp_Address());
		}
	}
}
