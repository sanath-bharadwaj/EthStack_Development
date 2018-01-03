#ifndef ETH_TXBUFFER_H_
#define ETH_TXBUFFER_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */


/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */

/*
 ***************************************************************************************************
 * Extern declarations
 ***************************************************************************************************
 */
#define ETH_START_SEC_CODE
#include "Eth_MemMap.h"

    extern VAR(Eth_TxBuffer_TransmitBufferQueueType_tst, ETH_VAR) Eth_TxBuffer_TransmitBufferQueue_ast[ETH_MAX_CTRLS_SUPPORTED];

    extern FUNC(void, ETH_CODE) Eth_TxBuffer_AdvanceQueueIndex(P2VAR(uint8, AUTOMATIC, AUTOMATIC) QueueIndex_pu8,
                                                               VAR(uint8, AUTOMATIC) LastIndex_u8);

    extern FUNC(void, ETH_CODE) Eth_TxBuffer_TransmitBufferQueueInit(VAR(uint8, AUTOMATIC) CtrlIdx_u8);

    extern FUNC(void, ETH_CODE) Eth_TxBuffer_TransmitBufferQueueReset(VAR(uint8, AUTOMATIC) CtrlIdx_u8);

    extern FUNC(BufReq_ReturnType, ETH_CODE) Eth_TxBuffer_AllocateBuffer( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                          P2VAR(uint8, AUTOMATIC, AUTOMATIC) BufIdxPtr_pu8,
                                                                          P2VAR(Eth_DataRefType_t, AUTOMATIC, AUTOMATIC) BufPtr_ppo,
                                                                          P2VAR(uint16, AUTOMATIC, AUTOMATIC) LenBytePtr_pu16);

    extern FUNC(Eth_DataRefType_t, ETH_CODE) Eth_TxBuffer_DetermineBufferAddress(VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                              VAR(uint8, AUTOMATIC) BufferIndex_u8);

#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#endif /* ETH_TXBUFFER_H_ */
