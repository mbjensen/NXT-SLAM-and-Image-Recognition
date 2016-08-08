/*
 *  kernel_cfg.c
 *  Mon Nov 30 08:46:28 2015
 *  SG Version 2.00
 *  sg.exe medina.oil -os=ECC2 -IC:/cygwin/nxtOSEK/toppers_osek/sg/impl_oil -template=C:/cygwin/nxtOSEK/toppers_osek/sg/lego_nxt.sgt
 */
#include "osek_kernel.h"
#include "kernel_id.h"
#include "alarm.h"
#include "interrupt.h"
#include "resource.h"
#include "task.h"

#define __STK_UNIT VP
#define __TCOUNT_STK_UNIT(sz) (((sz) + sizeof(__STK_UNIT) - 1) / sizeof(__STK_UNIT))

#define TNUM_ALARM     0
#define TNUM_COUNTER   0
#define TNUM_ISR2      0
#define TNUM_RESOURCE  0
#define TNUM_TASK      3
#define TNUM_EXTTASK   3

const UINT8 tnum_alarm    = TNUM_ALARM;
const UINT8 tnum_counter  = TNUM_COUNTER;
const UINT8 tnum_isr2     = TNUM_ISR2;
const UINT8 tnum_resource = TNUM_RESOURCE;
const UINT8 tnum_task     = TNUM_TASK;
const UINT8 tnum_exttask  = TNUM_EXTTASK;

 /****** Object OS ******/

 /****** Object TASK ******/

const TaskType TaskSlam = 0;
const TaskType TaskSafety = 1;
const TaskType TaskImage = 2;

extern void TASKNAME( TaskSlam )( void );
extern void TASKNAME( TaskSafety )( void );
extern void TASKNAME( TaskImage )( void );

static __STK_UNIT _stack_TaskSlam[__TCOUNT_STK_UNIT(512)];
static __STK_UNIT _stack_TaskSafety[__TCOUNT_STK_UNIT(512)];
static __STK_UNIT _stack_TaskImage[__TCOUNT_STK_UNIT(512)];

const Priority tinib_inipri[TNUM_TASK] = { TPRI_MINTASK + 2, TPRI_MINTASK + 2, TPRI_MINTASK + 2, };
const Priority tinib_exepri[TNUM_TASK] = { TPRI_MAXTASK, TPRI_MAXTASK, TPRI_MAXTASK, };
const UINT8 tinib_maxact[TNUM_TASK] = { (1) - 1, (1) - 1, (1) - 1, };
const AppModeType tinib_autoact[TNUM_TASK] = { 0x00000001, 0x00000001, 0x00000001, };
const FP tinib_task[TNUM_TASK] = { TASKNAME( TaskSlam ), TASKNAME( TaskSafety ), TASKNAME( TaskImage ), };
const __STK_UNIT tinib_stk[TNUM_TASK] = { (__STK_UNIT)_stack_TaskSlam, (__STK_UNIT)_stack_TaskSafety, (__STK_UNIT)_stack_TaskImage, };
const UINT16 tinib_stksz[TNUM_TASK] = { 512, 512, 512, };

TaskType tcb_next[TNUM_TASK];
UINT8 tcb_tstat[TNUM_TASK];
Priority tcb_curpri[TNUM_TASK];
UINT8 tcb_actcnt[TNUM_TASK];
EventMaskType tcb_curevt[TNUM_EXTTASK];
EventMaskType tcb_waievt[TNUM_EXTTASK];
ResourceType tcb_lastres[TNUM_TASK];
DEFINE_CTXB(TNUM_TASK);

 /****** Object COUNTER ******/


const TickType cntinib_maxval[TNUM_COUNTER+1] = { 0};
const TickType cntinib_maxval2[TNUM_COUNTER+1] = { 0};
const TickType cntinib_tickbase[TNUM_COUNTER+1] = { 0};
const TickType cntinib_mincyc[TNUM_COUNTER+1] = { 0};

AlarmType cntcb_almque[TNUM_COUNTER+1];
TickType cntcb_curval[TNUM_COUNTER+1];

 /****** Object ALARM ******/


const CounterType alminib_cntid[TNUM_ALARM+1] = { 0};
const FP alminib_cback[TNUM_ALARM+1] = { (FP)NULL};
const AppModeType alminib_autosta[TNUM_ALARM+1] = { 0};
const TickType alminib_almval[TNUM_ALARM+1] = { 0};
const TickType alminib_cycle[TNUM_ALARM+1] = { 0};

AlarmType almcb_next[TNUM_ALARM+1];
AlarmType almcb_prev[TNUM_ALARM+1];
TickType almcb_almval[TNUM_ALARM+1];
TickType almcb_cycle[TNUM_ALARM+1];

 /****** Object RESOURCE ******/


const Priority resinib_ceilpri[TNUM_RESOURCE+1] = { 0};

Priority rescb_prevpri[TNUM_RESOURCE+1];
ResourceType rescb_prevres[TNUM_RESOURCE+1];

 /****** Object EVENT ******/

const EventMaskType EventSleepI2C = (1UL << 0);
const EventMaskType EventSleep = (1UL << 1);

 /****** Object ISR ******/


#define IPL_MAXISR2 0
const IPL ipl_maxisr2 = IPL_MAXISR2;


const Priority isrinib_intpri[TNUM_ISR2+1] = { 0};
ResourceType isrcb_lastres[TNUM_ISR2+1];

 /****** Object APPMODE ******/

void object_initialize( void )
{
	task_initialize();
}




