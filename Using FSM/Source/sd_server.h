#ifndef SD_SERVER_H
#define SD_SERVER_H
#include <integer.h>
#include "sd_io.h"

// request types
typedef enum {REQ_NONE, REQ_INIT, REQ_READ, REQ_WRITE} SDS_REQ_T;
// status and results
typedef enum {STAT_IDLE, STAT_BUSY} SDS_STATUS_T;
	
typedef struct { // SD Server Transaction Data
	SDS_REQ_T Request;
	SD_DEV * Device;
	uint8_t * Data;
	uint32_t Sector;
	SDS_STATUS_T Status;
	SDRESULTS ErrorCode;
} SDS_TD_T ;

// States for SD Server FSM
typedef enum {S_IDLE, S_INIT, S_READ, S_WRITE, S_ERROR} SDS_STATE_T; 
typedef struct { // SD Server Transaction Data
	SDS_STATUS_T Status_fsm;
	SDRESULTS ErrorCode_fsm;
	int Start_fsm;
	int set_fsm;
} FSM ;
extern FSM Read;
extern FSM Write;
extern FSM Init;

extern SDS_TD_T g_trans; // Transaction request object

void Task_SD_Server(void);

/*
To request service...
1. Requesting task R waits until Status == STAT_IDLE and Request == REQ_NONE
2. R sets up transaction information Device,Data,Sector. 
3. R requests requests transaction by setting Request to REQ_INIT, REQ_READ, or REQ_WRITE.
4. (Let other tasks run. When server accepts and copies request, it sets Request=REQ_NONE and Status to STAT_BUSY 
5. R determines transaction is done by polling for g_trans.Status==STAT_IDLE and g_trans.Request==REQ_NONE

*/


#endif
