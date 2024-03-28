// ############################################################
// ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
// ��Ȩ���У�����ؾ�
// EtherCAT��վѧϰ��
// Author��͢���������
// �Ա�����: https://shop461235811.taobao.com/
// �ҵĲ��ͣ�https://blog.csdn.net/zhandouhu/article/category/9455918
// ############################################################
/**
\addtogroup EL9800Appl EL9800 application
@{
*/

/**
\file el9800appl.c
\author EthercatSSC@beckhoff.com
\brief Implementation

\version 5.11

<br>Changes to version V5.10:<br>
V5.11 ECAT11: create application interface function pointer, add eeprom emulation interface functions<br>
V5.11 EL9800 1: reset outputs on fallback from OP state<br>
<br>Changes to version V5.01:<br>
V5.10 ECAT6: Add "USE_DEFAULT_MAIN" to enable or disable the main function<br>
<br>Changes to version V5.0:<br>
V5.01 EL9800 2: Add TxPdo Parameter object 0x1802<br>
<br>Changes to version V4.30:<br>
V4.50 ECAT2: Create generic application interface functions. Documentation in Application Note ET9300.<br>
V4.50 COE2: Handle invalid PDO assign values.<br>
V4.30 : create file
*/

/*-----------------------------------------------------------------------------------------
------
------    Includes
------
-----------------------------------------------------------------------------------------*/
#include "ecat_def.h"

#if EL9800_APPLICATION

/* ECATCHANGE_START(V5.11) ECAT11*/
#include "applInterface.h"
/* ECATCHANGE_END(V5.11) ECAT11*/

// #include "el9800hw.h"

#define _EVALBOARD_
#include "el9800appl.h"
#undef _EVALBOARD_

#if MCI_HW
#include "mcihw.h"
#endif
#if EL9800_HW
#include "el9800hw.h"
#endif
/*--------------------------------------------------------------------------------------
------
------    local types and defines
------
--------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    local variables and constants
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*/
///////////////////////////////////////////////////////////////////////////////////////

void APPL_AckErrorInd(UINT16 stateTrans)
{
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
           all general settings were checked to start the mailbox handler. This function
           informs the application about the state transition, the application can refuse
           the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.

*/
///////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/
///////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
             all general settings were checked to start the input handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case the application need to be complete
            the transition by calling ECAT_StateChange.
*/
///////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/
///////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
           The return code NOERROR_INWORK can be used, if the application cannot confirm
           the state transition immediately, in that case the application need to be complete
           the transition by calling ECAT_StateChange.
*/
///////////////////////////////////////////////////////////////////////////////////////

UINT16 APPL_StartOutputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*/
///////////////////////////////////////////////////////////////////////////////////////

#include "bsp_led.h"

extern CanTxMsg Can1_TxMessage; // 发送缓冲区
extern CanRxMsg Can1_RxMessage; // 接收缓冲区

extern CanTxMsg Can2_TxMessage; // 发送缓冲区
extern CanRxMsg Can2_RxMessage; // 接收缓冲区

UINT16 APPL_StopOutputHandler(void)
{
    /*ECATCHANGE_START(V5.11) EL9800 1*/
    sDOOutputs.bLED1 = 0;
    sDOOutputs.bLED2 = 0;
    sDOOutputs.bLED3 = 0;
    sDOOutputs.bLED4 = 0;
    sDOOutputs.bLED5 = 0;
    sDOOutputs.bLED7 = 0;
    sDOOutputs.bLED6 = 0;
    sDOOutputs.bLED8 = 0;

    Can1_TxMessage.StdId = 0;
    Can1_TxMessage.ExtId = 0;
    Can1_TxMessage.IDE = 0;
    Can1_TxMessage.RTR = 0;
    Can1_TxMessage.DLC = 0;

    Can1_TxMessage.Data[0] = 0;
    Can1_TxMessage.Data[1] = 0;
    Can1_TxMessage.Data[2] = 0;
    Can1_TxMessage.Data[3] = 0;
    Can1_TxMessage.Data[4] = 0;
    Can1_TxMessage.Data[5] = 0;
    Can1_TxMessage.Data[6] = 0;
    Can1_TxMessage.Data[7] = 0;

    Can2_TxMessage.StdId = 0;
    Can2_TxMessage.ExtId = 0;
    Can2_TxMessage.IDE = 0;
    Can2_TxMessage.RTR = 0;
    Can2_TxMessage.DLC = 0;

    Can2_TxMessage.Data[0] = 0;
    Can2_TxMessage.Data[1] = 0;
    Can2_TxMessage.Data[2] = 0;
    Can2_TxMessage.Data[3] = 0;
    Can2_TxMessage.Data[4] = 0;
    Can2_TxMessage.Data[5] = 0;
    Can2_TxMessage.Data[6] = 0;
    Can2_TxMessage.Data[7] = 0;

    /*ECATCHANGE_END(V5.11) EL9800 1*/
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*/
///////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize)
{
#if COE_SUPPORTED
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM *pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

    /*Scan object 0x1C12 RXPDO assign*/
    for (PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if (pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for (PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                //				result = *(UINT8 *)pPDO->pVarPtr;
                //				result=(OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3);
                pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt + 1), pPDO) >> 3)); // goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16)((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if (result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for (PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if (pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for (PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt + 1), pPDO) >> 3)); // goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16)((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
#else
    *pInputSize = 6;
    *pOutputSize = 2;
    return ALSTATUSCODE_NOERROR;
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*/
///////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4 && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
#pragma interrupt_level 1
#endif
void APPL_InputMapping(UINT16 *pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the TxPDO Assign object to get the assigned TxPDOs */
    for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)
    {
        switch (sTxPDOassign.aEntries[j])
        {
        /* TxPDO 1 */
        case 0x1A00:
            *pTmpData++ = SWAPWORD(((UINT16 *)&sDIInputs)[1]);
            break;
        /* TxPDO 3 */
        case 0x1A02:
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[1]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[2]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[3]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[4]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[5]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[6]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[7]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[8]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[9]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[10]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[11]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[12]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[13]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[14]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[15]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[16]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[17]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[18]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[19]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[20]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[21]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[22]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[23]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[24]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[25]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[26]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[27]);
            *pTmpData++ = SWAPWORD(((UINT16 *)&sAIInputs)[28]);
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*/
///////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4 && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
#pragma interrupt_level 1
#endif
void APPL_OutputMapping(UINT16 *pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the RxPDO Assign object to get the assigned RxPDOs */
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch (sRxPDOassign.aEntries[j])
        {
        /* RxPDO 2 */
        case 0x1601:
            ((UINT16 *)&sDOOutputs)[1] = SWAPWORD(*pTmpData);
            ((UINT16 *)&sDOOutputs)[2] = (SWAPWORD(*pTmpData) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 1)) & 0xFF) << 8; //
            ((UINT16 *)&sDOOutputs)[3] = (SWAPWORD(*(pTmpData + 1)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 2)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[4] = (SWAPWORD(*(pTmpData + 2)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 3)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[5] = (SWAPWORD(*(pTmpData + 3)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 4)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[6] = (SWAPWORD(*(pTmpData + 4)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 5)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[7] = (SWAPWORD(*(pTmpData + 5)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 6)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[8] = (SWAPWORD(*(pTmpData + 6)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 7)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[9] = (SWAPWORD(*(pTmpData + 7)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 8)) & 0xFF) << 8; //
            ((UINT16 *)&sDOOutputs)[10] = (SWAPWORD(*(pTmpData + 8)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 9)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[11] = (SWAPWORD(*(pTmpData + 9)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 10)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[12] = (SWAPWORD(*(pTmpData + 10)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 11)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[13] = (SWAPWORD(*(pTmpData + 11)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 12)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[14] = (SWAPWORD(*(pTmpData + 12)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 13)) & 0xFF) << 8;
            ((UINT16 *)&sDOOutputs)[15] = (SWAPWORD(*(pTmpData + 13)) >> 8 & 0xFF) | (SWAPWORD(*(pTmpData + 14)) & 0xFF) << 8;
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief    This function will called from the synchronisation ISR
            or from the mainloop if no synchronisation is supported
*/
///////////////////////////////////////////////////////////////////////////////////////

void APPL_Application(void)
{
    //if (sDOOutputs.bLED1 != 0)
    //{
    //    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    //}
    //else
    //{
    //    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    //}

    //if (sDOOutputs.bLED2 != 0)
    //{
    //    GPIO_SetBits(GPIOB, GPIO_Pin_14);
    //}
    //else
    //{
    //    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    //}
//
    //if (sDOOutputs.bLED3 != 0)
    //{
    //    GPIO_SetBits(GPIOB, GPIO_Pin_13);
    //}
    //else
    //{
    //    GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    //}

    Can1_TxMessage.StdId = sDOOutputs.can1_h0;
    Can1_TxMessage.ExtId = sDOOutputs.can1_h1;
    Can1_TxMessage.IDE = sDOOutputs.can1_h2;
    Can1_TxMessage.RTR = sDOOutputs.can1_h3;
    Can1_TxMessage.DLC = sDOOutputs.can1_h4;

    Can1_TxMessage.Data[0] = sDOOutputs.can1_d0;
    Can1_TxMessage.Data[1] = sDOOutputs.can1_d1;
    Can1_TxMessage.Data[2] = sDOOutputs.can1_d2;
    Can1_TxMessage.Data[3] = sDOOutputs.can1_d3;
    Can1_TxMessage.Data[4] = sDOOutputs.can1_d4;
    Can1_TxMessage.Data[5] = sDOOutputs.can1_d5;
    Can1_TxMessage.Data[6] = sDOOutputs.can1_d6;
    Can1_TxMessage.Data[7] = sDOOutputs.can1_d7;

    CAN_Transmit(CAN1, &Can1_TxMessage);

    Can2_TxMessage.StdId = sDOOutputs.can2_h0;
    Can2_TxMessage.ExtId = sDOOutputs.can2_h1;
    Can2_TxMessage.IDE = sDOOutputs.can2_h2;
    Can2_TxMessage.RTR = sDOOutputs.can2_h3;
    Can2_TxMessage.DLC = sDOOutputs.can2_h4;

    Can2_TxMessage.Data[0] = sDOOutputs.can2_d0;
    Can2_TxMessage.Data[1] = sDOOutputs.can2_d1;
    Can2_TxMessage.Data[2] = sDOOutputs.can2_d2;
    Can2_TxMessage.Data[3] = sDOOutputs.can2_d3;
    Can2_TxMessage.Data[4] = sDOOutputs.can2_d4;
    Can2_TxMessage.Data[5] = sDOOutputs.can2_d5;
    Can2_TxMessage.Data[6] = sDOOutputs.can2_d6;
    Can2_TxMessage.Data[7] = sDOOutputs.can2_d7;

    CAN_Transmit(CAN2, &Can2_TxMessage);

    sDIInputs.bSwitch1 = 0;
    sDIInputs.bSwitch2 = 0;
    sDIInputs.bSwitch3 = 0;
    sDIInputs.bSwitch4 = 0;
    sDIInputs.bSwitch5 = 0;
    sDIInputs.bSwitch6 = 0;
    sDIInputs.bSwitch7 = 0;
    sDIInputs.bSwitch8 = 0;

    /* start the conversion of the A/D converter */
    //		while(!(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==SET));

    sAIInputs.i16Analoginput = 0;

    sAIInputs.can1_h0 = Can1_RxMessage.StdId;
    sAIInputs.can1_h1 = Can1_RxMessage.ExtId;
    sAIInputs.can1_h2 = Can1_RxMessage.IDE;
    sAIInputs.can1_h3 = Can1_RxMessage.RTR;
    sAIInputs.can1_h4 = Can1_RxMessage.DLC;

    sAIInputs.can1_d0 = Can1_RxMessage.Data[0];
    sAIInputs.can1_d1 = Can1_RxMessage.Data[1];
    sAIInputs.can1_d2 = Can1_RxMessage.Data[2];
    sAIInputs.can1_d3 = Can1_RxMessage.Data[3];
    sAIInputs.can1_d4 = Can1_RxMessage.Data[4];
    sAIInputs.can1_d5 = Can1_RxMessage.Data[5];
    sAIInputs.can1_d6 = Can1_RxMessage.Data[6];
    sAIInputs.can1_d7 = Can1_RxMessage.Data[7];

    sAIInputs.can2_h0 = Can2_RxMessage.StdId;
    sAIInputs.can2_h1 = Can2_RxMessage.ExtId;
    sAIInputs.can2_h2 = Can2_RxMessage.IDE;
    sAIInputs.can2_h3 = Can2_RxMessage.RTR;
    sAIInputs.can2_h4 = Can2_RxMessage.DLC;

    sAIInputs.can2_d0 = Can2_RxMessage.Data[0];
    sAIInputs.can2_d1 = Can2_RxMessage.Data[1];
    sAIInputs.can2_d2 = Can2_RxMessage.Data[2];
    sAIInputs.can2_d3 = Can2_RxMessage.Data[3];
    sAIInputs.can2_d4 = Can2_RxMessage.Data[4];
    sAIInputs.can2_d5 = Can2_RxMessage.Data[5];
    sAIInputs.can2_d6 = Can2_RxMessage.Data[6];
    sAIInputs.can2_d7 = Can2_RxMessage.Data[7];

    /* we toggle the TxPDO Toggle after updating the data of the corresponding TxPDO */
    sAIInputs.bTxPDOToggle ^= 1;

    /* we simulate a problem of the analog input, if the Switch4 is on in this example,
       in this case the TxPDO State has to set to indicate the problem to the master */
    if (sDIInputs.bSwitch4)
        sAIInputs.bTxPDOState = 1;
    else
        sAIInputs.bTxPDOState = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param     index               index of the requested object.
 \param     subindex            subindex of the requested object.
 \param     objSize             size of the requested object data, calculated with OBJ_GetObjectLength
 \param     pData               Pointer to the buffer where the data can be copied to
 \param     bCompleteAccess     Indicates if a complete read of all subindices of the
                                object shall be done or not

 \return    ABORTIDX_XXX

 \brief     Handles SDO read requests to TxPDO Parameter
*/
///////////////////////////////////////////////////////////////////////////////////////
UINT8 ReadObject0x1802(UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM *pData, UINT8 bCompleteAccess)
{

    if (bCompleteAccess)
        return ABORTIDX_UNSUPPORTED_ACCESS;

    if (subindex == 0)
    {
        *pData = TxPDO1802Subindex0;
    }
    else if (subindex == 6)
    {
        /*clear destination buffer (no excluded TxPDO set)*/
        if (dataSize > 0)
            MBXMEMSET(pData, 0x00, dataSize);
    }
    else if (subindex == 7)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8 *)pData;

        // Reset Buffer
        *pu8Data = 0;

        *pu8Data = sAIInputs.bTxPDOState;
    }
    else if (subindex == 9)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8 *)pData;

        // Reset Buffer
        *pu8Data = 0;

        *pu8Data = sAIInputs.bTxPDOToggle;
    }
    else
        return ABORTIDX_SUBINDEX_NOT_EXISTING;

    return 0;
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*/
///////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GetDeviceID()
{
    UINT16 Value = (UINT16)(SWITCH_8 << 7) | (SWITCH_7 << 6) | (SWITCH_6 << 5) | (SWITCH_5 << 4) | (SWITCH_4 << 3) | (SWITCH_3 << 2) | (SWITCH_2 << 1) | (SWITCH_1);
    return Value;
}
#endif

#if USE_DEFAULT_MAIN
/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*/
///////////////////////////////////////////////////////////////////////////////////////
uint32_t times = 0x1000000;
#if _STM32_IO8
int main(void)
#else
void main(void)
#endif
{
    /* initialize the Hardware and the EtherCAT Slave Controller */
    HW_Init();

    MainInit();

    bRunApplication = TRUE;
    do
    {
        MainLoop();

    } while (bRunApplication == TRUE);

    HW_Release();
#if _STM32_IO8
    return 0;
#endif
}
#endif // #if USE_DEFAULT_MAIN
#endif // #if EL9800_APPLICATION

/** @} */
