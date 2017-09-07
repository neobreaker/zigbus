#include "ZComDef.h"
#include "hal_drivers.h"  //Ӳ������ͷ�ļ�
#include "OSAL.h"         //����ϵͳͷ�ļ�
#include "OSAL_Tasks.h"   //����ϵͳ����ͷ�ļ�

#if defined ( MT_TASK )   //����Ӧ��ͷ�ļ�
  #include "MT.h"
  #include "MT_TASK.h"
#endif

#include "nwk.h"          //�����ͷ�ļ�
#include "APS.h"          //Ӧ��֧�ֲ�ͷ�ļ�
#include "ZDApp.h"        //�豸����ͷ�ļ�

#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  #include "ZDNwkMgr.h"
#endif

#if defined ( ZIGBEE_FRAGMENTATION )
  #include "aps_frag.h"
#endif
#include "Common.h"

// ����ע��
const pTaskEventHandlerFn tasksArr[] = {
  macEventLoop,           //MAC����ѭ��
  nwk_event_loop,         //�����������
  Hal_ProcessEvent,       //Ӳ���㺯��
#if defined( MT_TASK )
  MT_ProcessEvent,        //����֧�ֲ㶨��
#endif
  APS_event_loop,         //Ӧ��֧�ֲ������¼�����
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_ProcessEvent,
#endif
  ZDApp_event_loop,       //�豸����㺯��
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_event_loop,
#endif
  GenericApp_ProcessEvent //�Լ��������������
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

void osalInitTasks( void )//���������ID�ķ��䣬�Լ���������ĳ�ʼ��
{
  uint8 taskID = 0;
  //�����ڴ�ռ�
  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  macTaskInit( taskID++ );//MAC�������ID��
  nwk_init( taskID++ );   //����ID����
  Hal_Init( taskID++ );   //Ӳ��ID����
#if defined( MT_TASK )
  MT_TaskInit( taskID++ );
#endif
  APS_Init( taskID++ );
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_Init( taskID++ );
#endif
  ZDApp_Init( taskID++ );
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_Init( taskID++ );
#endif
  GenericApp_Init( taskID );//�Լ������ʼ������
}

/*********************************************************************
*********************************************************************/
