#ifndef COMMON_H
#define COMMON_H

#include "ZComDef.h"

#define GENERICAPP_ENDPOINT            10

#define GENERICAPP_PROFID          0x0F04
#define GENERICAPP_DEVICEID        0x0001
#define GENERICAPP_DEVICE_VERSION       0
#define GENERICAPP_FLAGS                0
#define GENERICAPP_MAX_CLUSTERS         1
#define GENERICAPP_CLUSTERID            1

#define USR_EVENT_UART					0x01

#define UART_PORT						0
#define UART_BUF_SIZE					128
#define UART_READ_NUM 					8

#define UART_CMD_SND					0x01		//send
#define	UART_CMD_TSA					0x02		//tell short addr
#define UART_CMD_TEA					0x03		//tell ext addr
#define UART_CMD_TPSA					0x04		//tell parent short addr
#define UART_CMD_TPEA					0x05		//tell parent ext addr

typedef struct 
{
	uint16  short_addr;
	uint8  *ext_addr;
	uint16  parent_short_addr;
	uint8  *parent_ext_addr;
	
}nwk_addr_t;

extern void GenericApp_Init( byte task_id );
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );
#endif










