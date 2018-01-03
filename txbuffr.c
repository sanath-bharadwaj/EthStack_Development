#include "Eth.h"
#include "rba_Eth.h"
#include "Eth_Cfg_SchM.h"
#include "Eth_TxBuffer.h"
#include "Eth_Prv.h"


#ifdef ETH_CONFIGURED


/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

/*
 ***************************************************************************************************
 * Type Definitions
 ***************************************************************************************************
 */

/*
 ***************************************************************************************************
 * Variables
 ***************************************************************************************************
 */
#define ETH_START_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"
    VAR(Eth_TxBuffer_TransmitBufferQueueType_tst, ETH_VAR) Eth_TxBuffer_TransmitBufferQueue_ast[ETH_MAX_CTRLS_SUPPORTED];
#define ETH_STOP_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"

/*
***************************************************************************************************
* Prototype for Static functions: Start
***************************************************************************************************
*/

#define ETH_START_SEC_CODE
#include "Eth_MemMap.h"

/*
 ***************************************************************************************************
 * Prototype for Static functions: End
 ***************************************************************************************************
 */

/**
 ***************************************************************************************************
 * \moduledescription
 * Either increments QueueIndex or sets it to 0 for a queue wrap around
 * Is used for updating the TxBuffer index or DescQueue index.
 *
 * Parameter In:
 * \param LastIndex_u8   Last index in queue
 *
 * Parameter InOut:
 * \param QueueIndex_pu8 Queue index to be advanced
 *
 * \return               None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxBuffer_AdvanceQueueIndex(P2VAR(uint8, AUTOMATIC, AUTOMATIC) QueueIndex_pu8,
                                                    VAR(uint8, AUTOMATIC) LastIndex_u8)
{
    SchM_Enter_Eth(CONTROLLER);
    if ( (*QueueIndex_pu8 >= LastIndex_u8) ) /* last buffer in queue? */
    { /* yes */
        /* wrap round to start of queue */
        *QueueIndex_pu8 = 0U;
    }
    else
    { /* no */
        /* advance to next buffer in queue */
        *QueueIndex_pu8 += 1U;
    }
    SchM_Exit_Eth(CONTROLLER);
}

/*
 *  Transmit Queue Buffer Management
 */

/**
 ***************************************************************************************************
 * \moduledescription
 * Does the transmit buffer queue initialization
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 *
 * \return           None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxBuffer_TransmitBufferQueueInit(VAR(uint8, AUTOMATIC) CtrlIdx_u8)
{
    Eth_TxBuffer_TransmitBufferQueueRefType_t TransmitBufferQueue_pst;  /* local pointer to management structure for TX buffer */
    uint8                                     BufferIndex_u8;

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];

    /* set transmit buffer queue pointers to values from PBCfg */
    TransmitBufferQueue_pst->First_pu8            = &Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxBuffers_pu8[0];
    TransmitBufferQueue_pst->StateTable_pen       = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufferStateTable_pen;
    TransmitBufferQueue_pst->CurrentIndex_u8      = 0U;
    TransmitBufferQueue_pst->LastIndex_u8         = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufTotal_u8 - 1U;
    TransmitBufferQueue_pst->TxBufferToDescLinkTable_pu8 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufferToDescLinkTable_pu8;

    /* loop through all buffer elements of the TX buffer*/
    for ( BufferIndex_u8 = 0; BufferIndex_u8 <= TransmitBufferQueue_pst->LastIndex_u8; BufferIndex_u8++ )
    {
        /* initialize flags and counters */
        TransmitBufferQueue_pst->StateTable_pen[BufferIndex_u8]       = ETH_BUFFER_FREE;
    }
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Does the transmit buffer queue reset
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 *
 * \return           None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxBuffer_TransmitBufferQueueReset(VAR(uint8, AUTOMATIC) CtrlIdx_u8)
{
    Eth_TxBuffer_TransmitBufferQueueRefType_t TransmitBufferQueue_pst; /* local pointer to management structure for TX buffer */
    uint8                                     BufferIndex_u8;

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];

    /* reset transmit buffer queue index */
    TransmitBufferQueue_pst->CurrentIndex_u8 = 0U;

    /* loop through all buffer elements of the TX buffer*/
    for ( BufferIndex_u8 = 0; BufferIndex_u8 <= TransmitBufferQueue_pst->LastIndex_u8; BufferIndex_u8++ )
    {
        /* initialize flags and counters */
        TransmitBufferQueue_pst->StateTable_pen[BufferIndex_u8]       = ETH_BUFFER_FREE;
    }
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Allocates buffer(s) in the transmit buffer pool
 * If not the requested length can be provided, the available buffer is provided and still BUFREQ_OK
 * is returned.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8      Index of the controller within the context of the Ethernet Driver
 *
 * Parameter InOut:
 * \param LenBytePtr_pu16 In: desired length in bytes, out: granted length in bytes
 *
 * Parameter Out:
 * \param BufIdxPtr_pu8   Index to the granted buffer resource. To be used for subsequent requests
 * \param BufPtr_ppo      Pointer to the granted buffer
 *
 * \return                BufReq_ReturnType {BUFREQ_OK: success;
 *                                           BUFREQ_E_BUSY: all buffers in use}
 *
 ***************************************************************************************************
 */
FUNC(BufReq_ReturnType, ETH_CODE) Eth_TxBuffer_AllocateBuffer( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                               P2VAR(uint8, AUTOMATIC, AUTOMATIC) BufIdxPtr_pu8,
                                                               P2VAR(Eth_DataRefType_t, AUTOMATIC, AUTOMATIC) BufPtr_ppo,
                                                               P2VAR(uint16, AUTOMATIC, AUTOMATIC) LenBytePtr_pu16)
{
    Eth_TxBuffer_TransmitBufferQueueRefType_t TransmitBufferQueue_pst;   /* local pointer to management structure for TX buffer */
    uint16                                    RegularTxBufLenByte_u16;   /* local variable for regular or standard length of a TX buffer element; Hint: The last element has always a size of 1536 for wrap-around reasons */
    uint8                                     CurrentIndex_u8;           /* local Idx of the current TX buffer element*/
    BufReq_ReturnType                         Result_o;                 /* local variable holding the value of buffer read status of the indexed buffer*/
    Eth_TxBuffer_TransmitBufferStateType_ten  StateTableVal;            /* local variable holding the state of the indexed buffer */

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];
    RegularTxBufLenByte_u16 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxBufLenByte_u16;

    /*Atomic Section: Guarantees correct functionality when Eth_TxBuffer_AllocateBuffer is preempted by Eth_TxBuffer_AllocateBuffer*/
    /*1.Read and write back of current buffer index has to be executed in the same atomic section*/
    SchM_Enter_Eth(CONTROLLER);

    /*Get Current Index*/
    CurrentIndex_u8 = TransmitBufferQueue_pst->CurrentIndex_u8;

    /*Check buffer availability*/
    StateTableVal = TransmitBufferQueue_pst->StateTable_pen[CurrentIndex_u8];

    if(ETH_BUFFER_FREE == StateTableVal)
    {
        if(*LenBytePtr_pu16 <= RegularTxBufLenByte_u16)
        {
            /* Return buffer index */
            *BufIdxPtr_pu8 = CurrentIndex_u8;

            /* Advance Queue Index if buffer available and Requested Length is less than the available length */
            if(TransmitBufferQueue_pst->CurrentIndex_u8 >= TransmitBufferQueue_pst->LastIndex_u8)
            {
                TransmitBufferQueue_pst->CurrentIndex_u8 = 0;
            }
            else
            {
                TransmitBufferQueue_pst->CurrentIndex_u8 += 1U;
            }

            /* Lock the buffer */
            TransmitBufferQueue_pst->StateTable_pen[CurrentIndex_u8] = ETH_BUFFER_LOCKED;

            /* Return buffer pointer */
            *BufPtr_ppo = &TransmitBufferQueue_pst->First_pu8[CurrentIndex_u8 * RegularTxBufLenByte_u16];
            Result_o = BUFREQ_OK;
        }
        else
        {
            /* Requested Length is greater than the available length, Return BUFREQ_E_OVFL and granted length */
            *LenBytePtr_pu16 = RegularTxBufLenByte_u16;
            Result_o = BUFREQ_E_OVFL;
        }
    }
    else
    {
        /* No buffer available */
        Result_o = BUFREQ_E_BUSY;
    }

    /* This interrupt lock will take 0.42usec irrespective of the buffersize configured */
    SchM_Exit_Eth(CONTROLLER);

    return(Result_o);
}


/**
 ***************************************************************************************************
 * \moduledescription
 * Determines the address of a Eth driver TX buffer from its BufIdx
 * \par Synchronous, Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8       Index of the controller within the context of the Ethernet Driver
 * \param BufferIndex_u8   Index of a buffer in TX buffer queue
 *
 * \return                 Address of the buffer
 *
 ***************************************************************************************************
 */
FUNC(Eth_DataRefType_t, ETH_CODE) Eth_TxBuffer_DetermineBufferAddress(VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                   VAR(uint8, AUTOMATIC) BufferIndex_u8)
{
    Eth_TxBuffer_TransmitBufferQueueRefType_t TransmitBufferQueue_pst; /* local pointer to management structure for TX buffer */
    Eth_DataRefType_t                         BufferAddress_po;

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];

    BufferAddress_po = &TransmitBufferQueue_pst->First_pu8[BufferIndex_u8 * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxBufLenByte_u16];

    return(BufferAddress_po);
}


#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#endif /* ETH_CONFIGURED */


/*<>
**********************************************************************************************************************
* $History___:

* $
**********************************************************************************************************************
</>*/

