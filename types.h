#ifndef ETH_TYPES_H_
#define ETH_TYPES_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */

#include "Eth_GeneralTypes.h"
#include "rba_Eth_Types.h"

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

#define ETH_MAC_ADDRESS_LENGTH_BYTE     (6U)

/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */


typedef enum
{
    ETH_BUFFER_FREE   = 0, /* buffer not in use */
    ETH_BUFFER_LOCKED = 1, /* buffer state between Eth_ProvideTxBuffer() call
                              and subsequent Eth_Transmit() call */
    ETH_BUFFER_LINKED = 2  /* buffer state after Eth_Transmit() call until buffer is freed;
                            * Linked to MAC DMA controller or ready to use by MAC DMA controller */
} Eth_TxBuffer_TransmitBufferStateType_ten;


/* global management structure for TxBuffer */
typedef struct
{
    P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)                             			First_pu8;                  /* pointer to start of buffer table */
    volatile P2VAR(Eth_TxBuffer_TransmitBufferStateType_ten, AUTOMATIC, ETH_CFG) 	StateTable_pen;             /* pointer to buffer state table */
    volatile uint8                                                      			CurrentIndex_u8;
    uint8                                                               			LastIndex_u8;               /* number of buffers minus 1 */
    volatile P2VAR(uint8, AUTOMATIC, ETH_CFG)                                       TxBufferToDescLinkTable_pu8;  /* pointer to TxBuffer to desc link table*/
} Eth_TxBuffer_TransmitBufferQueueType_tst;
/* global management structure for TxBuffer */
typedef P2VAR(Eth_TxBuffer_TransmitBufferQueueType_tst, TYPEDEF, ETH_VAR) Eth_TxBuffer_TransmitBufferQueueRefType_t;

typedef struct
{
#ifdef ETH_DEM_REPORTING_SUPPORT
#if ETH_DEM_REPORTING_SUPPORT != STD_OFF
    Dem_EventIdType                    EthEAccess_u16;                /* event id referenced by ETH_E_ACCESS. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthERxFramesLost_u16;          /* event id referenced by ETH_E_RX_FRAMES_LOST. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthECRC_u16;                   /* event id referenced by ETH_E_CRC. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthEUnderSize_u16;             /* event id referenced by ETH_E_UNDERSIZEFRAME. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthEOverSize_u16;              /* event id referenced by ETH_E_OVERSIZEFRAME. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthEAlignment_u16;             /* event id referenced by ETH_E_ALIGNMENT. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthESingleCollision_u16;       /* event id referenced by ETH_E_SINGLECOLLISION. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthEMultipleCollision_u16;     /* event id referenced by ETH_E_MULTIPLECOLLISION. If zero, no event is configured in DEM. */
    Dem_EventIdType                    EthELateCollision_u16;         /* event id referenced by ETH_E_LATECOLLISION. If zero, no event is configured in DEM. */
#else
    P2FUNC(void,AUTOMATIC,EthEAccessErrorCallBack) (VAR( uint8,         AUTOMATIC )  CtrlIdx,     /* Controller Index */
                                                    VAR( Std_ReturnType,AUTOMATIC )  ErrorStatus); /* Error Status as E_OK and E_NOT_OK */
#endif
#endif
} Eth_DemEventsType;

/* Eth_CtrlConfigType_to: Implementation specific structure of the post build configuration */
typedef struct
{
    P2CONST(uint8, AUTOMATIC, ETH_CFG)                                  EthCtrlPhysAddress_pcu8;            /* points to start of 6 byte array containing the physical address */
    P2VAR(uint32, AUTOMATIC, ETH_DMA_DESC)                              EthCtrlRxDescriptors_pu32;          /* points to start of memory reserved for receive descriptors */
    P2VAR(uint32, AUTOMATIC, ETH_DMA_DESC)                              EthCtrlTxDescriptors_pu32;          /* points to start of memory reserved for transmit descriptors */
    P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)                             EthCtrlRxBuffers_pu8;               /* points to start of memory reserved for receive buffers */
    P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)                             EthCtrlTxBuffers_pu8;               /* points to start of memory reserved for transmit buffers */
	volatile P2VAR(boolean, AUTOMATIC, ETH_CFG)                         EthTxConfFlagsTable_pb;             /* points to start of memory reserved for transmit confirmation flags */
    volatile P2VAR(Eth_TxBuffer_TransmitBufferStateType_ten, AUTOMATIC, ETH_CFG) EthTxBufferStateTable_pen; /* points to start of memory reserved for transmit buffer states */
#ifdef ETH_DEM_REPORTING_SUPPORT
#if ETH_DEM_REPORTING_SUPPORT == STD_ON
    Eth_DemEventsType                                                   EthDemEvents_st;                    /* Structure that stores DemEventId for all the events */
#endif
#endif
    uint16                                                              EthCtrlRxBufLenByte_u16;            /* Limits the maximum receive buffer length (frame length) in bytes, @see ETH_MAX_RXBUFLEN */
    uint16                                                              EthCtrlTxBufLenByte_u16;            /* Limits the maximum transmit buffer length (frame length) in bytes, @see ETH_MAX_TXBUFLEN */
    uint16                                                              EthRxBufTotal_u16;                  /* Configures the number of receive buffers. */
    uint8                                                               EthTxBufTotal_u8;                   /* Configures the number of transmit buffers. */
    uint8                                                               EthCtrlIdx_u8;                      /* Specifies the instance ID of the configured controller. */
    volatile P2VAR(uint8, AUTOMATIC, ETH_CFG)                           EthTxDescToBufferLinkTable_pu8;     /* points to start of memory reserved for TxDesctoBuffer Link table */
    volatile P2VAR(uint8, AUTOMATIC, ETH_CFG)                           EthTxBufferToDescLinkTable_pu8;     /* points to start of memory reserved for TxBuffertoDesc Link table */
} Eth_CtrlConfigType_to;


/* Eth_ConfigType: Implementation specific structure of the post build configuration */
typedef struct
{
    Eth_CtrlConfigType_to EthCtrlConfig_ao[ETH_MAX_CTRLS_SUPPORTED];
    uint8                 EthNumCtrlConfig_u8; /* number of controller configurations within a config set */
#ifdef ETH_DEM_REPORTING_SUPPORT
#if ETH_DEM_REPORTING_SUPPORT != STD_ON
    Eth_DemEventsType     EthDemEvents_st;                    /* Structure that stores DemEventId for all the events */
#endif
#endif
} Eth_ConfigType;

/* management structure for RX descriptor buffer elements */
typedef struct
{
    rba_Eth_RxBufferDescriptorRefType_t First_pst;               /* pointer to start of descriptor table */

    /*
     *  the structure elements below are used to store information about the processing state
     *  of the  function rba_Eth_Receive() over consecutive calls (on per controller basis);
     *  the purpose is to avoid gathering the same information again and again during consecutive calls;
     *  example: the function detects the start of a new frame, but the controller has not yet
     *  received all data of the frame, and a descriptor with end of frame bit set is not present yet;
     *  the function will then stop examinig the DMA descriptors and exit;
     *  on the next call, instad of starting the search for a new frame at the descriptor with
     *  start of frame bit set again, the function will use the global context stored in this structure
     *  to directly resume the search where it stopped in the previous call
     */
    uint32                             FrameStartAddress_u32;   /* start address of the frame in the DMA buffer memory */
    uint16                             FrameStartIndex_u16;     /* index in the DMA descriptor table for the descriptor that has start of frame bit set */

    uint16                              CurrentIndex_u16;
    uint16                              LastIndex_u16;          /* number of descriptors minus 1 */
} Eth_ReceiveBufferDescriptorQueueType_tst;
/* management structure for RX descriptor buffer elements */
typedef P2VAR(Eth_ReceiveBufferDescriptorQueueType_tst, TYPEDEF, ETH_VAR) Eth_ReceiveBufferDescriptorQueueRefType_t;

/* management structure for TX descriptor buffer elements */
typedef struct
{
	rba_Eth_TxBufferDescriptorRefType_t  First_pst;                  /* pointer to start of descriptor table */
	volatile uint32                    TxDescrToBeFreedCnt_u32;      /*Count of Tx descriptors(frames) to be freed*/
	volatile P2VAR(boolean, AUTOMATIC, ETH_CFG) TxConfFlagsTable_pb;        /* poiner to start of confirmation flags table */
    uint8                              ConfirmationLowWaterMark_u8;
    volatile uint8                     CurrentIndex_u8;
    uint8                              LastIndex_u8;                /* number of descriptors minus 1 */
    volatile P2VAR(uint8, AUTOMATIC, ETH_CFG)  TxDescToBufferLinkTable_pu8;  /* pointer to Txdesc to Txbuffer link table*/
} Eth_TransmitBufferDescriptorQueueType_tst;
/* management structure for TX descriptor buffer elements */
typedef P2VAR(Eth_TransmitBufferDescriptorQueueType_tst, TYPEDEF, ETH_VAR) Eth_TransmitBufferDescriptorQueueRefType_t;

/* global management structure per Eth controller */
typedef struct
{
    rba_Eth_RegisterMapRefType_t                  Registers_pst;                /* pointer to register bank in HW (memory mapped)*/
    Eth_ReceiveBufferDescriptorQueueType_tst      ReceiveDescriptorQueue_st;    /* Management structure of RxDescQueue */
    Eth_TransmitBufferDescriptorQueueType_tst     TransmitDescriptorQueue_st;   /* Management structure of TxDescQueue */
} Eth_ControllerType_tst;
/* global management structure per Eth controller */
typedef P2VAR(Eth_ControllerType_tst, TYPEDEF, ETH_VAR) Eth_ControllerRefType_t;

#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
/* Structure which holds Dem status and Re Report information for each event */
typedef struct
{
    boolean                  ReReportRequire_b;                 /* Flag Set when DSM reInit done and Re-Reporting of status required */
    Dem_EventStatusType      DemEventStatus;                    /* Latest Status of Dem Event */
} Eth_DSMReInit_DemEventsHandle_tst;
#endif

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)

/* Structure for maintaining software counters for Reception path */
typedef struct
{
    volatile uint32 Eth_RxPacketsLost_u32;
    volatile uint32 Eth_CrcErrors_u32;
    volatile uint32 Eth_UnderSizeFrames_u32;
    volatile uint32 Eth_OverSizeFrames_u32;
    volatile uint32 Eth_AlignmentError_u32;
    volatile uint32 Eth_SingleCollisionError_u32;
    volatile uint32 Eth_MultipleCollisionError_u32;
    volatile uint32 Eth_LateCollisionError_u32;
}Eth_DemErrorCountHandle_tst;

/*Pointer to  Frame  descriptor */
typedef Eth_DemErrorCountHandle_tst * Eth_DemErrorCountHandleRefType_t;

#endif
#endif

/*
 ***************************************************************************************************
 * Extern declarations
 ***************************************************************************************************
 */

#endif /* ETH_TYPES_H_ */

/*<>
**********************************************************************************************************************
*
**********************************************************************************************************************
</>*/
