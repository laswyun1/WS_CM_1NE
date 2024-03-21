/**
 * @file wholeBodyCtrlTask.c
 * @date Created on: Sep 21, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */
#include "AS_whole_body_ctrl.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

TaskObj_t wholeBodyCtrlTask;
extern osSemaphoreId_t sdio_sync_semaphore;			//sdio transmit sync semaphore
extern osMutexId_t mutex_sd_buffering;				//buffering mutex

GravCompObj_t gravComp_RH;
GravCompObj_t gravComp_LH;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

uint32_t wholeBodyCtrlLoopCnt;

/* For Data Save */
//////////////////////////////////////////////////////////////////////////////////////////////
uint8_t sd_buf_1[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 1
uint8_t sd_buf_2[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 2

uint_fast16_t sd_buf_1_pos = 0;						//sd buf 1 position(index)
uint_fast16_t sd_buf_2_pos = 0;						//sd buf 2 position(index)

uint8_t *sd_buf_cur = sd_buf_1;						//current buffer pointer : start with sd buf 1
uint8_t *sd_buf_next = sd_buf_2;					//next buffer pointer

// uint8_t sdSendData[SEND_DATA_SIZE] ;                 //data buffer
/* For H10 Battery Usage Data Logging */
uint8_t sdSendData[SEND_DATA_SIZE];   //data buffer

int convert_res = 0;
uint32_t dataSaveCnt = 0;

static uint32_t lastDataSaveTime = 0;
static uint32_t oneMinuteCnt = 0;
uint8_t dataSaveFinished = 0;
//////////////////////////////////////////////////////////////////////////////////////////////


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static uint8_t oneTimeMDSet = 1;

// Tuning //
static uint8_t MD_modeIdx_H10L = 0;
static int16_t MD_TauMax_H10L = 0;
static uint8_t MD_modeIdx_H10R = 0;
static int16_t MD_TauMax_H10R = 0;
static uint8_t MD_modeIdx_K10L = 0;
static int16_t MD_TauMax_K10L = 0;
static uint8_t MD_modeIdx_K10R = 0;
static int16_t MD_TauMax_K10R = 0;

// For Right
static float auxInput_RH_BF = 0.2;
static float auxInput_RK_BF = 0.2;

static float auxInput_RH_FB = -0.2;
static float auxInput_RK_FB = -0.2;

static uint32_t swingTimeStack_R_B9 = 0;
static uint32_t swingTimeStack_R_B10 = 0;

// For Left
static float auxInput_LH_BF = 0.2;
static float auxInput_LK_BF = 0.2;

static float auxInput_LH_FB = -0.2;
static float auxInput_LK_FB = -0.2;

static uint32_t swingTimeStack_L_B9 = 0;
static uint32_t swingTimeStack_L_B10 = 0;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static void SaveDataMassStorage(void);

/* For Data Save */
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len);			// Data buffering Transmission functions

static void fillArrayWithVariable(int var, char byteArray[], int arraySize);				// data generator
static void intToStr(int num, char str[], int maxSize);										// integer to string translator

/* Data Processing 2024.01.30 By HD (Not used sprintf Ver, to be deleted soon) */
static void ReverseStr(char* str, int length);
static void IntToStringRec(int value, char* output, int* index);
static void IntToString(int value, char* output);
static int FindStringLength(char* str, int index);
static void FloatToString(float value, char* output, int afterpoint);
static void FillBufferData(DataPoint data[], int numDataPoints, char buffer[], int bufferSize);

/* ----------------------- ROUTINE ------------------------ */

/* --------------------- SDO CALLBACK --------------------- */

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(wholeBodyCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitWholeBodyCtrl(void)
{
	/* Init Task */
	InitTask(&wholeBodyCtrlTask);
	DOPC_AssignTaskID(&wholeBodyCtrlTask, TASK_IDX_WHOLE_BODY_CTRL);

	/* Init Device */

    /* State Definition */
    TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_OFF,      StateOff_Ent,	    StateOff_Run,     NULL,					false);
    TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_STANDBY,  NULL,			    StateStandby_Run, NULL,					true);
    TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run,  StateEnable_Ext,  	false);
    TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_ERROR,    NULL,			    StateError_Run,   NULL,					false);

	/* Routine Definition */

    /* DOD Definition */
	// DOD
    DOP_CreateDOD(TASK_IDX_WHOLE_BODY_CTRL);

    // SDO
    DOP_COMMON_SDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL)

    // PDO
    DOP_COMMON_PDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL, wholeBodyCtrlTask);

    Download_Test_RobotSetting(RS_File_AS_Test);

    InitFDCANDevMngr();
    InitGravityCompParams(&gravComp_RH);
    InitGravityCompParams(&gravComp_LH);
}

void RunWholeBodyCtrl(void)
{
	/* Run Device */
    RunTask(&wholeBodyCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void)
{

}

static void StateOff_Run(void)
{
	StateTransition(&wholeBodyCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
	if (wholeTaskFlag == 1) {
		StateTransition(&wholeBodyCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
}

static void StateEnable_Ent(void)
{
//    EntRoutines(&wholeBodyCtrlTask.routine);
    wholeBodyCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{
    if (wholeBodyCtrlLoopCnt == WHOLE_BODY_LOOP_START_CNT) {
#ifdef SUIT_H10
        SetPDO_H10(AS_MD_IDX_1);
        SetPDO_H10(AS_MD_IDX_2);
#endif
#ifdef SUIT_K10
        SetPDO_K10(AS_MD_IDX_1);
        SetPDO_K10(AS_MD_IDX_2);
#endif
	} else if (wholeBodyCtrlLoopCnt > WHOLE_BODY_LOOP_START_CNT && oneTimeMDSet == 1) {
		/* Set Routine, TASK_STATE_ENABLE */
	    AllDevSetRoutines();
	    AllDevEnableStates();

		systemStateFlag = SYSTEM_ASSIST;

		oneTimeMDSet = 0;
	} else if (wholeBodyCtrlLoopCnt > WHOLE_BODY_LOOP_START_CNT) {
#ifdef SUIT_H10
        // Gravity Compensation //
        GravityCompensator(&gravComp_RH, gaitObj_RH.AngleData.degUnbiased[0]);
        GravityCompensator(&gravComp_LH, gaitObj_LH.AngleData.degUnbiased[0]);
        SendAuxInput(AS_MD_IDX_1, gravComp_RH.FTC.resCurrent, 1);
        SendAuxInput(AS_MD_IDX_2, gravComp_LH.FTC.resCurrent, 1);
        
        // For Right H10 - RH //
        if (ISIFlags_RH.I7_Flag == 1) {
            MD_modeIdx_H10R = 4;
            MD_TauMax_H10R = -670;
            Send_F_Vector(AS_MD_IDX_1, MD_modeIdx_H10R, MD_TauMax_H10R, 0);
        } else if (ISIFlags_RH.I8_Flag == 1) {
            MD_modeIdx_H10R = 4;
            MD_TauMax_H10R = 670;
            Send_F_Vector(AS_MD_IDX_1, MD_modeIdx_H10R, MD_TauMax_H10R, 0);
        }

        // if (ISIFlags_RH.I9_Flag == 1) {
        //     if (ISIFlags_RH.I1_Flag == 1) {
        //         swingTimeStack_R_B10 = 0;
        //         swingTimeStack_R_B9++;
        //         if (swingTimeStack_R_B9 >= 100) {
        //             SendAuxInput(AS_MD_IDX_1, auxInput_RH_BF, 1);
        //         }
        //     }
        // }
        // else if (ISIFlags_RH.I10_Flag == 1) {
        //     swingTimeStack_R_B9 = 0;
        //     swingTimeStack_R_B10++;
        //     if (ISIFlags_RH.I1_Flag == 1) {
        //         if (swingTimeStack_R_B10 >= 100) {
        //             SendAuxInput(AS_MD_IDX_1, auxInput_RH_FB, 1);
        //         }
        //     }
        // }

        //-----------------------------------------------------------------------------------------------------//
        // For Left H10 - LH //
        if (ISIFlags_LH.I7_Flag == 1) {
            MD_modeIdx_H10L = 4;
            MD_TauMax_H10L = 670;
            Send_F_Vector(AS_MD_IDX_2, MD_modeIdx_H10L, MD_TauMax_H10L, 0);
        } else if (ISIFlags_LH.I8_Flag == 1) {
            MD_modeIdx_H10L = 4;
            MD_TauMax_H10L = -670;
            Send_F_Vector(AS_MD_IDX_2, MD_modeIdx_H10L, MD_TauMax_H10L, 0);
        }

        // if (ISIFlags_LH.I9_Flag == 1) {
        //     if (ISIFlags_LH.I1_Flag == 1) {
        //         swingTimeStack_L_B10 = 0;
        //         swingTimeStack_L_B9++;
        //         if (swingTimeStack_L_B9 >= 100) {
        //             SendAuxInput(AS_MD_IDX_2, auxInput_LH_BF, 1);
        //         }
        //     }
        // }
        // else if (ISIFlags_LH.I10_Flag == 1) {
        //     swingTimeStack_L_B9 = 0;
        //     swingTimeStack_L_B10++;
        //     if (ISIFlags_LH.I1_Flag == 1) {
        //         if (swingTimeStack_L_B10 >= 100) {
        //             SendAuxInput(AS_MD_IDX_2, auxInput_LH_FB, 1);
        //         }
        //     }
        // }
#endif /* SUIT_H10 */

#ifdef SUIT_K10
        //-----------------------------------------------------------------------------------------------------//
        // For Right K10 - RK //
        if (ISIFlags_RK.I8_Flag == 1) {
            MD_modeIdx_K10R = 1;					// 1, -280, 0, 1.0
            MD_TauMax_K10R = -510;
            Send_F_Vector(AS_MD_IDX_1, MD_modeIdx_K10R, MD_TauMax_K10R, 0);
        }
        else if (ISIFlags_RK.I16_Flag == 1) {
            MD_modeIdx_K10R = 3;					// 3, 320, 0, 1.0
            MD_TauMax_K10R = 670;

            Send_F_Vector(AS_MD_IDX_1, MD_modeIdx_K10R, MD_TauMax_K10R, 0);
        }

        //-----------------------------------------------------------------------------------------------------//
        // For Left K10 - LK //
        if (ISIFlags_LK.I8_Flag == 1) {
            MD_modeIdx_K10L = 1;
            MD_TauMax_K10L = -510;
            Send_F_Vector(AS_MD_IDX_2, MD_modeIdx_K10L, MD_TauMax_K10L, 0);
        }
        else if (ISIFlags_LK.I16_Flag == 1) {
            MD_modeIdx_K10L = 3;
            MD_TauMax_K10L = 670;
            Send_F_Vector(AS_MD_IDX_2, MD_modeIdx_K10L, MD_TauMax_K10L, 0);
        }
#endif /* SUIT_K10 */

        /* Data Storage */
        // SaveDataMassStorage();
    }

    // RunRoutines(&wholeBodyCtrlTask.routine);
    wholeBodyCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
//    ExtRoutines(&wholeBodyCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/* ----------------------- FUNCTION ----------------------- */

/* Data Storage Test Functions */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void SaveDataMassStorage(void)
{
    if((wholeBodyCtrlLoopCnt - WHOLE_BODY_LOOP_START_CNT + 1) < (MIN_TO_MILLISEC * DATA_SAVE_TIME)) { //@1ms * MIN_TO_MILLISEC * SEND_DATA_SIZE(Byte) * DATA_SAVE_TIME(Min) : 21MByte Transfer within 5 Min.
        /* Data Conversion To String & Save Buffer */
        memset(sdSendData, 0, sizeof(sdSendData));
        dataSaveCnt = wholeBodyCtrlLoopCnt - WHOLE_BODY_LOOP_START_CNT + 1;

        snprintf((char*)sdSendData, sizeof(sdSendData) - 2, "%"PRIu32"; %.2f; %.2f; %.2f; %.2f; %.2f; %.2f;\r\n",
            dataSaveCnt, medianVolt, batData.batCurr,
            userCtrlObj[AS_MD_IDX_1].data.totalCurrentInput, userCtrlObj[AS_MD_IDX_1].data.CurrentOutput,
            userCtrlObj[AS_MD_IDX_2].data.totalCurrentInput, userCtrlObj[AS_MD_IDX_2].data.CurrentOutput);

        /* Buffered Transmit to SD */
        _SD_DBuffered_Transmit(sdSendData, sizeof(sdSendData));
    }
}

/* Key Function : SDIO Transmit with double buffer & swap(Switching) buffer algorithm */
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len) 
{
    osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);					// Mutex lock

    uint_fast16_t *cur_buf_pos = (sd_buf_cur == sd_buf_1) ? &sd_buf_1_pos : &sd_buf_2_pos;
    uint_fast16_t cur_buf_space = SWAP_SD_BUF_SIZE - *cur_buf_pos;

    if (cur_buf_space >= data_len) {
        memcpy(sd_buf_cur + *cur_buf_pos, data, data_len);
        *cur_buf_pos += data_len;
    } else {
        memcpy(sd_buf_cur + *cur_buf_pos, data, cur_buf_space);
        uint_fast16_t remaining_data_len = data_len - cur_buf_space;

        if (sd_buf_cur == sd_buf_1) {
            sd_buf_1_pos += cur_buf_space;
            sd_buf_cur = sd_buf_2;
            cur_buf_pos = &sd_buf_2_pos;
            sd_buf_2_pos = 0;
        } else {
            sd_buf_2_pos += cur_buf_space;
            sd_buf_cur = sd_buf_1;
            cur_buf_pos = &sd_buf_1_pos;
            sd_buf_1_pos = 0;
        }

        memcpy(sd_buf_cur, data + cur_buf_space, remaining_data_len);
        *cur_buf_pos = remaining_data_len;

        osSemaphoreRelease(sdio_sync_semaphore);							// release semaphore : start writing buffered data to SD card
    }

    osMutexRelease(mutex_sd_buffering);										// Mutex unlock
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Data Store Test by TJ */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void intToStr(int num, char str[], int maxSize) 
{
    int i = 0;
    int isNegative = 0;

    // 음수 체크
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 숫자를 역순으로 추출하여 문자열에 저장
    do {
        str[i++] = num % 10 + '0';
        num /= 10;
    } while (num > 0);

    // 음수일 경우 '-'를 추가
    if (isNegative) {
        str[i++] = '-';
    }

    // 문자열 뒤집기
    int len = i;
    for (int j = 0; j < len / 2; j++) {
        char temp = str[j];
        str[j] = str[len - j - 1];
        str[len - j - 1] = temp;
    }

    // 널 종료 문자('\0') 추가
    str[i] = '\0';

    // 문자열 길이가 maxSize보다 작을 경우 나머지를 널 바이트로 채움
    for (int j = len; j < maxSize; j++) {
        str[j] = '\0';
    }
}
    
static void fillArrayWithVariable(int var, char byteArray[], int arraySize)
{
    // 배열을 널 바이트로 초기화합니다.
    memset(byteArray, 0, arraySize);

    // 변수를 문자열로 변환합니다.
    char var_str[21];  // 최대 20자리 문자열 + 널 종료 문자('\0') 고려
    intToStr(var, var_str, sizeof(var_str));

    // 문자열을 배열에 복사합니다.
    int i;
    for (i = 0; i < arraySize - 2; i++) { // 2 바이트를 더 남겨두고 "\r\n"을 추가합니다.
        byteArray[i] = var_str[i];
    }

    // 끝에 "\r\n" 추가합니다.
    byteArray[i++] = '\r';
    byteArray[i++] = '\n';
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Data Processing 2024.01.30 */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void ReverseStr(char* str, int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

static void IntToStringRec(int value, char* output, int* index)
{
    if (value == 0) return;

    int digit = value % 10;
    if (digit < 0) digit = -digit;

    output[(*index)++] = digit + '0';

    IntToStringRec(value / 10, output, index);
}

static void IntToString(int value, char* output)
{
    int index = 0;
    if (value < 0) {
        output[index++] = '-'; // 음수 부호 추가
        value = -value; // 값의 절대값을 사용합니다.
    }

    if (value == 0) {
        output[index++] = '0';
    } else {
        IntToStringRec(value, output, &index);
        if (value < 0) {
            ReverseStr(output + 1, index - 1);
        } else {
            ReverseStr(output, index);
        }
    }
    output[index] = '\0';
}

static int FindStringLength(char* str, int index)
{
    if (str[index] == '\0') {
        return index;
    }
    return FindStringLength(str, index + 1);
}

static void FloatToString(float value, char* output, int afterpoint)
{
    // Extract integer part and convert
    int ipart = (int)value;
    IntToString(ipart, output);

    // 문자열 길이 찾기
    int i = 0;
    while (output[i] != '\0') i++;

    // Handle fractional part
    if (afterpoint > 0) {
        output[i++] = '.';
        float fpart = value - (float)ipart;
        if (value < 0) fpart = -fpart; // 음수 처리

        // 소수점 이하 값을 문자열로 변환
        for (int j = 0; j < afterpoint; j++) {
            fpart *= 10.0;
            int digit = (int)fpart;
            output[i++] = '0' + digit;
            fpart -= digit;
        }
        output[i] = '\0';
    }
}

static void FillBufferData(DataPoint data[], int numDataPoints, char buffer[], int bufferSize)
{
    // 배열을 널 바이트로 초기화합니다.
    memset(buffer, 0, bufferSize);

    int i = 0;

    for (int j = 0; j < numDataPoints; j++) {
        char tempStr[16]; // 임시 문자열 버퍼(각 데이터마다 1:1대응), float의 최대 표현 자리수(16자리수)
        memset(tempStr, 0, sizeof(tempStr)); // tempStr을 0으로 초기화

        osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);    // Mutex lock

        // 각 데이터 타입에 따른 처리
        switch (data[j].DataTp) {
            case DATATYPE_UINT32:
                IntToString((int)data[j].uint32Data, tempStr);
                break;
            case DATATYPE_INT:
                IntToString(data[j].intData, tempStr);
                break;
            case DATATYPE_INT16:
                IntToString((int)data[j].int16Data, tempStr);
                break;
            case DATATYPE_FLOAT:
                FloatToString(data[j].floatData, tempStr, 2); // 소수점 이하 2자리
                break;
            default:
                tempStr[0] = '\0'; // 알 수 없는 데이터 타입 처리
        }

        // 버퍼에 문자열과 구분자 추가
        int len = strlen(tempStr);
        if (i + len + 2 < bufferSize - 2) { // +2는 구분자 공간, -2는 CRLF 공간
            strncpy(&buffer[i], tempStr, len);
            i += len;

            // 모든 데이터에 구분자 추가
            buffer[i++] = ';';
            buffer[i++] = ' ';
        }

        osMutexRelease(mutex_sd_buffering);    // Mutex unlock
    }

    osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);    // Mutex lock

    // 버퍼의 나머지 부분을 null로 채우기
    while (i < bufferSize - 2) {
        buffer[i++] = '\0';
    }

    // 버퍼의 끝에 "\r\n" 추가
    buffer[i++] = '\r';
    buffer[i++] = '\n';

    osMutexRelease(mutex_sd_buffering);    // Mutex unlock
}

/* ----------------------- ROUTINE ------------------------ */

/* --------------------- SDO CALLBACK --------------------- */

#endif /* SUIT_MINICM_ENABLED */
