#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "Common.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
#include "OnBoard.h"
#endif

/* HAL */
//#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "OSAL_Nv.h"

#define MODBUS_TIMEOUT          3
#define MODBUS_NUM              16

const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
    GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
    GENERICAPP_ENDPOINT,              //  int Endpoint;
    GENERICAPP_PROFID,                //  uint16 AppProfId[2];
    GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
    GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
    GENERICAPP_FLAGS,                 //  int   AppFlags:4;
    GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
    (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
    GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
    (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t GenericApp_epDesc;
byte GenericApp_TaskID;
byte GenericApp_TransID;
unsigned char uartbuf[UART_BUF_SIZE];
uint16 uartbuf_len = 0;
nwk_addr_t g_nwt_addr;

uint16 g_short_addr[MODBUS_NUM];
unsigned char g_modbus_timeout[MODBUS_NUM];

void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void GenericApp_SendTheMessage(void);
static void rxCB(uint8 port, uint8 event);
void GenericApp_SendTheUart(unsigned char *buf, int len);
void uart_cmd(unsigned char *buf, int len);

void GenericApp_Init( byte task_id )
{
    halUARTCfg_t uartconfig;

    GenericApp_TaskID = task_id;//osal分配的任务ID随着用户添加任务的增多而改变
    GenericApp_TransID = 0;//消息发送ID（多消息时有顺序之分）

    //定义本设备用来通信的APS层端点描述符
    GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;//应用程序的端口号
    GenericApp_epDesc.task_id = &GenericApp_TaskID;  //描述符的任务ID
    GenericApp_epDesc.simpleDesc                     //简单描述符
        = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
    GenericApp_epDesc.latencyReq = noLatencyReqs;    //延时策略
    afRegister( &GenericApp_epDesc );                //向AF层登记描述符

	osal_memset(g_short_addr, 0xff, MODBUS_NUM*2);
    osal_memset(g_modbus_timeout, MODBUS_TIMEOUT, MODBUS_NUM);

    uartconfig.configured   = TRUE;
    uartconfig.baudRate     = HAL_UART_BR_9600;
    uartconfig.flowControl  = FALSE;
    uartconfig.callBackFunc = rxCB;
    HalUARTOpen(UART_PORT, &uartconfig);

}

UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
    afIncomingMSGPacket_t *MSGpkt;

    if ( events & SYS_EVENT_MSG )
    {
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
        while ( MSGpkt )
        {
            switch ( MSGpkt->hdr.event )
            {
                case AF_INCOMING_MSG_CMD:
                    GenericApp_MessageMSGCB(MSGpkt);
                    break;
                default:
                    break;
            }
            osal_msg_deallocate( (uint8 *)MSGpkt );
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
        }
        return (events ^ SYS_EVENT_MSG);
    }
    if(events & USR_EVENT_UART)
    {
        if(uartbuf_len == 8)
        {
            if(g_short_addr[uartbuf[0]] == 0xff)
            {
                GenericApp_SendTheUart(uartbuf, uartbuf_len);
            }
            
			
        }
        uartbuf_len = 0;
        osal_stop_timerEx(GenericApp_TaskID, USR_EVENT_UART);
        return (events ^ USR_EVENT_UART);
    }
    return 0;
}

void GenericApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
    unsigned char modbus_addr;
    uint16 src_addr;
    switch ( pkt->clusterId )
    {
        case GENERICAPP_CLUSTERID:

            if(pkt->cmd.DataLength > 8)
            {
                src_addr =   pkt->cmd.Data[0];
                src_addr += (pkt->cmd.Data[1]<<8);
                modbus_addr = pkt->cmd.Data[4];
                g_short_addr[modbus_addr-1] = src_addr;

                g_modbus_timeout[modbus_addr-1] = 0;

            }

            HalUARTWrite(0, pkt->cmd.Data+4, pkt->cmd.DataLength-4);
            break;
    }
}


void GenericApp_SendTheUart(unsigned char *buf, int len)
{
    unsigned char snd_buf[UART_BUF_SIZE + 4];
    unsigned char *p;
    afAddrType_t devDstAddr;
    unsigned char modbus_addr;
    modbus_addr = buf[0]-1;

    devDstAddr.addrMode=(afAddrMode_t)Addr16Bit;
    devDstAddr.endPoint=GENERICAPP_ENDPOINT;
    devDstAddr.addr.shortAddr=g_short_addr[modbus_addr];

    g_nwt_addr.short_addr = NLME_GetShortAddr();
    p = (unsigned char *)&(g_nwt_addr.short_addr);
    snd_buf[0] = *p;
    snd_buf[1] = *(p+1);

    p = (unsigned char *)&(g_short_addr[modbus_addr]);
    snd_buf[2] = *p;
    snd_buf[3] = *(p+1);

    osal_memcpy(snd_buf+4, buf, len);

    AF_DataRequest(&devDstAddr,
                   &GenericApp_epDesc,
                   GENERICAPP_CLUSTERID,
                   len+4,
                   snd_buf,
                   &GenericApp_TransID,
                   AF_DISCV_ROUTE,
                   AF_DEFAULT_RADIUS);


    if(g_modbus_timeout[modbus_addr] < MODBUS_TIMEOUT)
    {
        g_modbus_timeout[modbus_addr]++;
    }
    else
    {
        g_short_addr[modbus_addr] = 0xffff;
    }

}

static void rxCB(uint8 port, uint8 event)
{
    uint16 numread = 0;
    numread = HalUARTRead(port, uartbuf+uartbuf_len, UART_READ_NUM);
    if(numread > 0)
    {
        uartbuf_len += numread;
        osal_start_timerEx(GenericApp_TaskID, USR_EVENT_UART, 10);

    }

}

/*
void uart_cmd(unsigned char *buf, int len)
{
    uint8 cmd = 0;
    if(len < 0 )
        return ;

    cmd = buf[0];

    switch(cmd)
    {
        case UART_CMD_SND:
            GenericApp_SendTheUart(uartbuf+1, uartbuf_len-1);
            break;
        case UART_CMD_TSA:
            g_nwt_addr.short_addr = NLME_GetShortAddr();
            HalUARTWrite(UART_PORT, (unsigned char*)&g_nwt_addr.short_addr, sizeof(uint16));
            break;
        case UART_CMD_TEA:
            g_nwt_addr.ext_addr = NLME_GetExtAddr();
            HalUARTWrite(UART_PORT, g_nwt_addr.ext_addr, 64);
            break;
        case UART_CMD_TPSA:
            g_nwt_addr.parent_short_addr = NLME_GetShortAddr();
            HalUARTWrite(UART_PORT, (unsigned char*)&g_nwt_addr.parent_short_addr, sizeof(uint16));
            break;
        case UART_CMD_TPEA:
            NLME_GetCoordExtAddr(g_nwt_addr.parent_ext_addr);
            HalUARTWrite(UART_PORT, g_nwt_addr.parent_ext_addr, 64);
            break;
    }

    osal_memset(uartbuf, 0, uartbuf_len);
    uartbuf_len = 0;

}
*/
