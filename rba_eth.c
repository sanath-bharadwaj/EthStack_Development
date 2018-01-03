#include "Eth.h"
#include "rba_Eth.h"

#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
#if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))
/* Async MII Implementation requires GPT */
#include "Gpt.h"

#endif
#endif

#include "Eth_Cfg_SchM.h"


#include "EthIf_Cbk.h"

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
#include "Dem.h"
#endif
#endif

#include "rba_Eth_Cfg_Time.h"
#include "rba_Eth_Cfg_Mcu.h"
#include "EthTrcv.h"
#include "rba_Eth_Prv.h"

#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
#include "rba_Eth_Cfg_SIUL2.h"
#endif
/* rba_Eth is submodule of Eth component so private header of Eth can be included */
#include "Eth_Prv.h"

#ifdef ETH_CONFIGURED

/*
 *  SW version checks
 */

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */
#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
#if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))
/* Final GPT Timer Value used while starting the timer*/
#define RBA_ETH_MII_TIMER_VALUE(CLOCK)     (RBA_ETH_MII_TIMER_VALUE_US * ((CLOCK) / (1000000U)))
#endif
#endif
#if defined (RBA_ETH_GLOBAL_TIME_SUPPORT)
#if (RBA_ETH_GLOBAL_TIME_SUPPORT == STD_ON)
#if (RBA_ETH_PTP_CLK_REF != 0U)
/* Frequency calculated for 1nsec is 1GHz so clock provided to ethernet peripheral for PTP should be 1Ghz for correct 1 ns calculation
 * if PTP clock is not 1 Ghz then calibration is required to calculate correct 1ns
 * calibration value is - 1Ghz/(PTP clock frequency)
 * If PTP clock frequency is 32Mhz
 * Calibration offset will be - 1Ghz/32Mhz = 31 */
#define RBA_ETH_SUB_SECOND_INC_VALUE       ((1000000000U/(RBA_ETH_PTP_CLK_REF)))
#else
#error  “Configure RBA_BOOTCTRL_F_SPB clock with proper clock value or disable the Global Time Support“
#endif
#endif
#endif
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
#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
#if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))

#define RBA_ETH_START_SEC_VAR_CLEARED_UNSPECIFIED
#include "rba_Eth_MemMap.h"
   static  VAR(rba_Eth_ReadWriteMiiManag_tst , ETH_VAR) rba_Eth_ReadWriteMiiManag_st;
#define RBA_ETH_STOP_SEC_VAR_CLEARED_UNSPECIFIED
#include "rba_Eth_MemMap.h"

#endif
#endif



/*
 ***************************************************************************************************
 * Prototype for Static functions: Start
 ***************************************************************************************************
 */

#define RBA_ETH_START_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"
static void rba_Eth_IndicateRxFrame(uint8          CtrlIdx_u8,
                                          uint32         Des3_CtrlStatus_u32,
                                          Eth_DataType   *DataPtr_pu8);
 #if defined (RBA_ETH_EN_RX_INTERRUPT)
    #if (RBA_ETH_EN_RX_INTERRUPT == STD_ON)
          static void rba_Eth_Receive_Interrupt(uint8 CtrlIdx_u8);
    #endif
 #endif
#define RBA_ETH_STOP_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"

#define RBA_ETH_START_SEC_CODE
#include "rba_Eth_MemMap.h"

    static void rba_Eth_ReceiveDescriptorQueueInit(uint8 CtrlIdx_u8, Eth_ControllerRefType_t Controller_pst, rba_Eth_RegisterMapRefType_t Registers_pst);

    static void rba_Eth_TransmitDescriptorQueueInit(uint8 CtrlIdx_u8, Eth_ControllerRefType_t Controller_pst, rba_Eth_RegisterMapRefType_t Registers_pst);

    static void rba_Eth_AssembleBottomAndTopPhysAddress(uint32 *PhysAddrBottom_pu32, uint32 *PhysAddrTop_pu32, const uint8 *PhysAddrPtr_pcu8);

    static void rba_Eth_AdvanceRxTailPointer(const Eth_ReceiveBufferDescriptorQueueRefType_t  ReceiveDescriptorQueue_pst,rba_Eth_RegisterMapRefType_t    Registers_pst);

#if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
    static uint32 rba_Eth_FindFirstUnusedOrMatchingAddr(const rba_Eth_RegisterMapRefType_t  Registers_pst, uint32 PhysAddrTop_u32, uint32 PhysAddrBottom_u32, uint8 searchUnusedFlag_u8);

    static void rba_Eth_DisableAllMacAddrFilters(rba_Eth_RegisterMapRefType_t Registers_pst);
#endif

    static void rba_Eth_UpdateWriteBackDesc( uint8                           CtrlIdx_u8,
                                         rba_Eth_RxBufferDescriptorRefType_t CurrentDesc_pst,
                                         uint32                              DataBuffferAddress_u32,
                                         uint8                               DescType_u8 );

    static void rba_Eth_TxRxDescriptorQueueInit(uint8 CtrlIdx_u8, Eth_ControllerRefType_t Controller_pst, rba_Eth_RegisterMapRefType_t Registers_pst);
    static void rba_Eth_ProgramMTLMACRegs(rba_Eth_RegisterMapRefType_t Registers_pst);


/*
 ***************************************************************************************************
 * Prototype for Static functions: End
 ***************************************************************************************************
 */
/*
 *************************************************************************************************
 considering code is started after HW reset --> Assuming all HW register in its default value.
 ************************************************************************************************
 */

 /**
  ***************************************************************************************************
  * \moduledescription
  * initialize MAC Transcation Layer MTL block
  * Note: the reset default values are ALL 0 -> no need to clean first
  * \par Synchronous, Non Reentrant
  *
  * Parameter In:
  * \param Registers_pst  Pointer to controller register map
  *
  * \return               None
  *
  ***************************************************************************************************
  */

 static void rba_Eth_ProgramMTLRegs(rba_Eth_RegisterMapRefType_t Registers_pst)
 {
    /*Map MTL Queue0 to DMA channel0 and MTL queue1 to DMA channel1 */
	uint32 tmpRegVal_u32 =  ((RBA_ETH_MTL_RXQ_DMA_MAP0_Q0MDMACH << RBA_ETH_MTL_RXQ_DMA_MAP0_Q0MDMACH_POS)|
                             (RBA_ETH_MTL_RXQ_DMA_MAP0_Q0DDMACH << RBA_ETH_MTL_RXQ_DMA_MAP0_Q0DDMACH_POS)|
	                         (RBA_ETH_MTL_RXQ_DMA_MAP0_Q1MDMACH << RBA_ETH_MTL_RXQ_DMA_MAP0_Q1MDMACH_POS)|
	                         (RBA_ETH_MTL_RXQ_DMA_MAP0_Q1DDMACH << RBA_ETH_MTL_RXQ_DMA_MAP0_Q1DDMACH_POS));

    Registers_pst->MTL_RXQ_DMA_MAP0 = tmpRegVal_u32;

    /* Enable Transmit store and forward method, Transmit Queue size 4Kbytes and Enable Transmit queue0  */
    tmpRegVal_u32 = ((RBA_ETH_MTL_TXQ0_OPERATION_MODE_TSF << RBA_ETH_MTL_TXQ0_OPERATION_MODE_TSF_POS)|
                    (RBA_ETH_MTL_TXQ0_OPERATION_MODE_TXQEN << RBA_ETH_MTL_TXQ0_OPERATION_MODE_TXQEN_POS)|
                    (RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS << RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS_POS));

    Registers_pst->MTL_TXQ0_OPERATION_MODE = tmpRegVal_u32;

    /* Enable Receive store and forward method, Receive Queue size 8Kbytes and Enable Receive queue0  */
    tmpRegVal_u32 = ((RBA_ETH_MTL_RXQ0_OPERATION_MODE_RSF << RBA_ETH_MTL_RXQ0_OPERATION_MODE_RSF_POS)|
                    (RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS << RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS_POS));

    Registers_pst->MTL_RXQ0_OPERATION_MODE = tmpRegVal_u32;

 }


 /**
   ***************************************************************************************************
   * \moduledescription
   * MAC related configurations:
   * hash filtering configuration - no hash addr filtering for now - skipped
   * Mac frame filter configuration
   * MAC flow control
   * MAC Configuration
   *
   * \par Synchronous, Non Reentrant
   *
   * Parameter In:
   * \param Registers_pst  Pointer to controller register map
   *
   * \return               None
   *
   ***************************************************************************************************
   */

static void rba_Eth_InitMAC(rba_Eth_RegisterMapRefType_t Registers_pst)
{
    /* hash filtering configuration - no hash addr filtering for now - skipped*/
    /* Mac frame filter configuration */
    /* Enable hash/perfect filtering only for SPC58 controller since MAC_PACKET_FILTER is not applicable for IFX5B
       This is required in case hash filter is turned on in the future*/
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    Registers_pst->MAC_PACKET_FILTER = RBA_ETH_MAC_PACKET_FILTER_HPF  << RBA_ETH_MAC_PACKET_FILTER_HPF_POS;
#endif

    /* MAC flow control */
    /* Disable zero quanta pause for first Tx queue */
    Registers_pst->MAC_Q0_TX_FLOW_CTRL = RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ << RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ_POS;
    /* Disable zero quanta pause for second Tx queue only for SPC58 controller */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    Registers_pst->MAC_Q1_TX_FLOW_CTRL = RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ << RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ_POS;
#endif

    /* MAC Configuration */
	/* Enable Source Address Insertion, Full Checksum Offload type, Auto-stripping of FCS (for both ethernet frames having length < 1536 and >= 1536), Full duplex and Carrier sense before Transmission */
    Registers_pst->MAC_CONFIGURATION =  ((RBA_ETH_MAC_CONFIGURATION_SARC    <<  RBA_ETH_MAC_CONFIGURATION_SARC_POS)|
                                         (RBA_ETH_MAC_CONFIGURATION_IPC     <<  RBA_ETH_MAC_CONFIGURATION_IPC_POS) |
                                         (RBA_ETH_MAC_CONFIGURATION_ACS     <<  RBA_ETH_MAC_CONFIGURATION_ACS_POS) |
                                         (RBA_ETH_MAC_CONFIGURATION_DM      <<  RBA_ETH_MAC_CONFIGURATION_DM_POS)  |
                                         (RBA_ETH_MAC_CONFIGURATION_CST     <<  RBA_ETH_MAC_CONFIGURATION_CST_POS));

    /* Enable Receive Queue0
     * Note : Datasheet tells that enable it is for AV traffic but without this nothing works ; no queue gets enabled
     * Hence enabling everything
     */
    Registers_pst->MAC_RXQ_CTRL0 |= RBA_ETH_MAC_RXQ_CTRL0_RXQ0EN;

}

/**
  ***************************************************************************************************
  * \moduledescription
  * MAC and MTL related configurations: Calls rba_Eth_InitMAC and rba_Eth_ProgramMTLRegs functions
  * to configure MAC and MTLregisters respectively
  *
  * \par Synchronous, Non Reentrant
  *
  * Parameter In:
  * \param Registers_pst  Pointer to controller register map
  * \return               None
  *
  ***************************************************************************************************
  */
static void rba_Eth_ProgramMTLMACRegs(rba_Eth_RegisterMapRefType_t Registers_pst)
{
    rba_Eth_ProgramMTLRegs(Registers_pst);
    /* MAC configuration */
    rba_Eth_InitMAC(Registers_pst);
}
/**
 ***************************************************************************************************
 * \moduledescription
 *  Calls Transmit descriptor queue init and receive descriptor queue init to initilaise TX and RX
 *  descriptors
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8     Index of the controller within the context of the Ethernet Driver
 * \param Controller_pst Pointer to controller data structure
 * \param Registers_pst  Pointer to controller register map
 *
 * \return               None
 *
 ***************************************************************************************************
 */

static void rba_Eth_TxRxDescriptorQueueInit(uint8 CtrlIdx_u8,
                                        Eth_ControllerRefType_t Controller_pst,
                                        rba_Eth_RegisterMapRefType_t Registers_pst)
{
    rba_Eth_TransmitDescriptorQueueInit(CtrlIdx_u8, Controller_pst, Registers_pst);
    rba_Eth_ReceiveDescriptorQueueInit(CtrlIdx_u8, Controller_pst, Registers_pst);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Does the receive buffer initialization and the initialization of the management structure for the
 * RX descriptor buffer queue.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8     Index of the controller within the context of the Ethernet Driver
 * \param Controller_pst Pointer to controller data structure
 * \param Registers_pst  Pointer to controller register map
 *
 * \return               None
 *
 ***************************************************************************************************
 */
static void rba_Eth_ReceiveDescriptorQueueInit(uint8 CtrlIdx_u8,
                                               Eth_ControllerRefType_t Controller_pst,
                                               rba_Eth_RegisterMapRefType_t Registers_pst)
{
    uint32 DescriptorIndex_u32; /* index for current processed descriptor*/
    uint32 rxDescrOptions = 0;
    const uint32 RxBufferSize_u32 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16;

    /* the Rx Memory */
    /* MR12 RULE 11.3 VIOLATION: cast from different ptr type in order to keep the config type more opaque. */
    rba_Eth_RxBufferDescriptorRefType_t  RxDescrArray_pst = (rba_Eth_RxBufferDescriptorRefType_t)Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxDescriptors_pu32;


    /* local pointer to RX buffer for assigning the corresponding addresses to the RxDescQueue */
    uint8* ReceiveBuffer_pu8 = &Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBuffers_pu8[0];

    /* set global receive buffer descriptor queue pointers to values of PBCfg */
    Eth_ReceiveBufferDescriptorQueueRefType_t ReceiveDescriptorQueue_pst = &Controller_pst->ReceiveDescriptorQueue_st;

    ReceiveDescriptorQueue_pst->First_pst              = &RxDescrArray_pst[0];
    ReceiveDescriptorQueue_pst->CurrentIndex_u16       = 0U;
    ReceiveDescriptorQueue_pst->LastIndex_u16          = (RBA_ETH_NO_OF_RXDESC - 1U);
    ReceiveDescriptorQueue_pst->FrameStartIndex_u16    = 0U;
    ReceiveDescriptorQueue_pst->FrameStartAddress_u32  = 0U;

    /* MR12 RULE 11.4,11.6 VIOLATION: cast from pointer to number. This check is necessary and read-only. it is ok. */
    ETH_DET_REPORT_NORETURN(((((uint32) ReceiveDescriptorQueue_pst->First_pst) % 8U) != 0U), ETH_SID_ETH_SETCONTROLLERMODE, ETH_E_INV_PARAM);

    /*Enable interrupt on completion if requested*/
    if (Eth_CtrlEnableRxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
    {
        rxDescrOptions|= RBA_ETH_MAC_DMARXDESC3_IOC;
    }

    /* Set ownership bit to DMA; Buffer2 address is valid */
    rxDescrOptions |= ( RBA_ETH_MAC_DMARXDESC3_OWN | RBA_ETH_MAC_DMARXDESC3_BUF2V );

    /* loop through all descriptors in the RxDescBuffer*/
    for ( DescriptorIndex_u32 = 0; DescriptorIndex_u32 <= ReceiveDescriptorQueue_pst->LastIndex_u16; DescriptorIndex_u32++ )
    {
        /* Data Buffer1 is not used */
        RxDescrArray_pst[DescriptorIndex_u32].Des0_DataBuff1Addr_VlanTag_u32 = 0U;

        /* Reserved */
        RxDescrArray_pst[DescriptorIndex_u32].Des1_Reserved_Status_u32 = 0U;



        /* Data Buffer2 is not used */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        /* When Timestamp is enabled, the odd numbered descriptor will be used as context descriptor and hence no need for databuffer address */
        if(0U == (DescriptorIndex_u32 % RBA_ETH_EVEN_RX_BUFFER))
        {
            /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
            /* For Normal Descriptor databuffer address is used*/
            RxDescrArray_pst[DescriptorIndex_u32].Des2_DataBuff2Addr_Status__u32 = (uint32) &ReceiveBuffer_pu8[(DescriptorIndex_u32 >> 1) * RxBufferSize_u32];
            RxDescrArray_pst[DescriptorIndex_u32].Des3_CtrlStatus_u32 = rxDescrOptions;
        }
        else
        {
            /* Here RDES3 updation is for CTXT based descriptor in which only OWN bit is set and BUF2V bit is made as 0 to make this descriptor as invalid buffer so that
			*in case of multiple reception next descriptor used for data reception will be Normal descriptor */
            RxDescrArray_pst[DescriptorIndex_u32].Des3_CtrlStatus_u32 = RBA_ETH_MAC_DMARXDESC3_OWN;
        }
#else
        /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
        RxDescrArray_pst[DescriptorIndex_u32].Des2_DataBuff2Addr_Status__u32 = (uint32) &ReceiveBuffer_pu8[(DescriptorIndex_u32) * RxBufferSize_u32];
        RxDescrArray_pst[DescriptorIndex_u32].Des3_CtrlStatus_u32 = rxDescrOptions;
#endif

    }

    /* Setup the LINK to FIRST descr in list.*/
    /* critical section because global structures are affected*/
    SchM_Enter_Eth(CONTROLLER);

    /* Inform Eth DMA, where the RX descriptor queue starts
     * The RxDescTail pointer register needs to be updated to point to the last descriptor; giving n-1 desc to DMA
     * Update the Rx Desc Ring Length register (Total number of RxDesc-1 so that RollOver can happen)*/
    /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This comes from a HW Reg actually holding a ptr. */
    Registers_pst->DMA_CH0_RXDESC_LIST_ADDRESS = (uint32) &RxDescrArray_pst[0];
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
    Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER = (uint32) &RxDescrArray_pst[(RBA_ETH_MUL_BY_TWO((Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthRxBufTotal_u16)-1))];
    Registers_pst->DMA_CH0_RXDESC_RING_LENGTH  = (uint32) ((RBA_ETH_MUL_BY_TWO(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthRxBufTotal_u16)) - 1U);
#else
    Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER = (uint32) &RxDescrArray_pst[(((Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthRxBufTotal_u16)-1))];
    Registers_pst->DMA_CH0_RXDESC_RING_LENGTH  = (uint32) (Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthRxBufTotal_u16 - 1U);
#endif
    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Does the transmit buffer descriptor queue initialization
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8     Index of the controller within the context of the Ethernet Driver
 * \param Controller_pst Pointer to controller data structure
 * \param Registers_pst  Pointer to controller register map
 *
 * \return               None
 *
 ***************************************************************************************************
 */
static void rba_Eth_TransmitDescriptorQueueInit(uint8 CtrlIdx_u8,
                                                Eth_ControllerRefType_t Controller_pst,
                                                rba_Eth_RegisterMapRefType_t Registers_pst)
{
    uint32 descrIdx_u32;          /* index for current processed descriptor*/
    uint32 txDescr2_Options_u32 = 0;
    uint32 txDescr3_Options_u32 = 0;

    /* local pointer to a the current processed descriptor in the descriptor buffer */
    /* MR12 RULE 11.3 VIOLATION: cast from different ptr type in order to keep the config type more opaque. */
    rba_Eth_TxBufferDescriptorRefType_t TxDescrArray_pst = (rba_Eth_TxBufferDescriptorRefType_t)Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxDescriptors_pu32;

    /* local pointer to TX buffer for assigning the corresponding addresses to the TxDescQueue */
    uint8* TransmitBuffer_pu8 = &Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxBuffers_pu8[0];

    /* local pointer to management structure for TX descriptor buffer elements */
    Eth_TransmitBufferDescriptorQueueRefType_t TxDescQueue_pst = &Controller_pst->TransmitDescriptorQueue_st;


    /* set global transmit buffer descriptor queue pointers to values of PBCfg */
    TxDescQueue_pst->First_pst                   = &TxDescrArray_pst[0];
    TxDescQueue_pst->TxConfFlagsTable_pb         = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxConfFlagsTable_pb;
    TxDescQueue_pst->CurrentIndex_u8             = 0U;
    TxDescQueue_pst->ConfirmationLowWaterMark_u8 = 0U;
    TxDescQueue_pst->LastIndex_u8                = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufTotal_u8 - 1U;

    TxDescQueue_pst->TxDescToBufferLinkTable_pu8 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxDescToBufferLinkTable_pu8;

    /*Enable interrupt on completion if requested*/
    if (Eth_CtrlEnableTxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
    {
        txDescr2_Options_u32 = RBA_ETH_MAC_DMATXDESC2_IOC;
    }
    /* Enable global timestamp for all descriptor if requested */
#if defined RBA_ETH_GLOBAL_TIME_SUPPORT
    #if (RBA_ETH_GLOBAL_TIME_SUPPORT == STD_ON)
    txDescr2_Options_u32 |= RBA_ETH_MAC_DMATXDESC2_TTSE;
     #endif
#endif

    /* Set bits for first and last segment, source address insertion control and full checksum insertion control, */
    txDescr3_Options_u32 = (RBA_ETH_MAC_DMATXDESC3_FD | RBA_ETH_MAC_DMATXDESC3_LD | RBA_ETH_MAC_DMATXDESC3_SAIC| RBA_ETH_MAC_DMATXDESC3_CIC);

    /* loop through all descriptors in the TxDescBuffer*/
    for ( descrIdx_u32 = 0; descrIdx_u32 <= TxDescQueue_pst->LastIndex_u8; descrIdx_u32++)
    {
        /*addr in the Transmit buffer, used to set transmit buffer address  */
        uint32 thisDescrBuffOffset = descrIdx_u32 * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxBufLenByte_u16;

        /* set transmit buffer address */
        /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
        TxDescrArray_pst[descrIdx_u32].Des0_DataBuff1Addr_TimeStampLow_u32 = (uint32) &TransmitBuffer_pu8[thisDescrBuffOffset];

        /* Data buffer 2 is not used */
        TxDescrArray_pst[descrIdx_u32].Des1_DataBuff2Addr_TimeStampHigh_u32 = 0;

        TxDescrArray_pst[descrIdx_u32].Des2_Ctrl_u32 = txDescr2_Options_u32;

        TxDescrArray_pst[descrIdx_u32].Des3_CtrlStatus_u32 = txDescr3_Options_u32; /* own bit is cleared for Tx buffers */

        /* initialize confirmation flag per default to FALSE */
        TxDescQueue_pst->TxConfFlagsTable_pb[descrIdx_u32] = FALSE;
    }

    /* critical section because global structures are affected*/
    SchM_Enter_Eth(CONTROLLER);

    /* The TxDescTail pointer register needs to be updated to point to the next descriptor
     * when a frame corresonding to the current desc needs to be transmitted
     * Inform Eth DMA, where the TX descriptor queue starts
     * Update the Tail pointer register to start of TxDesc Initially
     * Update the Tx Desc Ring Length register (Total number of TxDesc-1 so that RollOver can happen)*/
    /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
    Registers_pst->DMA_CH0_TXDESC_LIST_ADDRESS = (uint32) &TxDescrArray_pst[0];
    Registers_pst->DMA_CH0_TXDESC_TAIL_POINTER = (uint32) &TxDescrArray_pst[0];
    Registers_pst->DMA_CH0_TXDESC_RING_LENGTH  = (uint32) (Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufTotal_u8 - 1U);

    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

}


/**
 ***************************************************************************************************
 * \moduledescription
 * ETH027: Initializes the Ethernet Driver, extension for IFX
 * \par Service ID 0x01, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 *
 * \return        None
 *
 ***************************************************************************************************
 */
void rba_Eth_Init(uint8 CtrlIdx_u8)
{

#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32                        ClcRegReadVal_vu32; /* local variable to ensure writing of CLC register*/
#endif
    /* MR12 RULE 11.4,11.6 VIOLATION: Conversion from number to pointer. We need to convert the base address somewhere....*/
   Eth_Controllers_ast[CtrlIdx_u8].Registers_pst = (rba_Eth_RegisterMapRefType_t)Eth_CtrlRegisterBaseAddress_acu32_MP[CtrlIdx_u8];

#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON

    /* To unlock the MCU protection */
   Eth_Clc_Mcu_Rb_ResetProtection();
    /*Enable the Eth module*/
   Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->CLC = 0;
    /*Lock MCU protection for CLC*/
   Eth_Clc_Mcu_Rb_SetProtection();

   /* During the phase in which Eth module becomes active, any write access to
    * corresponding module registers (while DISS is still set) will generate a bus error. When
    * enabling a disabled module, application software should read back the CLC register
    * "once", to check that DISS is cleared, before writing to any module register (including the
    * CLC register). */

   /* Handling to check if CLC is really zero is not required, since as per user manual, single read of CLC is enough.
    * The idea is to read back CLC before making any further write accesses to Eth module registers */
   ClcRegReadVal_vu32 = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->CLC;

   ((void)(ClcRegReadVal_vu32));
   /* To unlock the MCU protection */

#ifdef RBA_ETH_RGMII_TX_CLK_DELAY_REQ
   /* Enable TX clock delay as per RGMII V2 if the MAC PHY interface type is configured as RGMII */
   Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->SKEWCTL |= RBA_ETH_RGMII_TX_CLK_DELAY_REQ << RBA_ETH_RGMII_TX_CLK_DELAY_REQ_POS;
#endif
#ifdef RBA_ETH_RGMII_RX_CLK_DELAY_REQ
   /* Enable RX clock delay as per RGMII V2 if the MAC PHY interface type is configured as RGMII */
   Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->SKEWCTL |= RBA_ETH_RGMII_RX_CLK_DELAY_REQ << RBA_ETH_RGMII_RX_CLK_DELAY_REQ_POS;
#endif

   Eth_Gpctl_Mcu_Rb_ResetProtection();

   /* Configure the GPCTL register according to the port configuration */
   Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->GPCTL |= rba_Eth_Port_ConfigSets[CtrlIdx_u8];

   Eth_Gpctl_Mcu_Rb_SetProtection();
#endif
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Initializes the indexed controller
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 *
 * \return           Std_ReturnType {E_OK: success; E_NOT_OK: Controller initialization failed}
 *
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_ControllerInit(uint8 CtrlIdx_u8)
{

       Std_ReturnType                       Result_o = E_NOT_OK;    /* local variable holding the status of controller init*/
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
       boolean                              CheckBitStatus_b;       /* local variable holding the status of kernel reset*/
       uint8                                DelayKRST_u8;           /* local variable holding the counter value for delay after kernel reset */
#endif

#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
 #if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))
       rba_Eth_ReadWriteMiiManagRefType_t   ReadWriteMiiManagRef_pst; /* local pointer to global management structure of read/writemii */
#endif
#endif

       Result_o = E_NOT_OK;

#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
 #if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))
       /* Get address of global */
       ReadWriteMiiManagRef_pst = &rba_Eth_ReadWriteMiiManag_st;

       /*Initialize Read and Write Mii Management Structure */
       ReadWriteMiiManagRef_pst->rba_Eth_ReadWriteMiiGptClkTick_u32 = (uint32)(RBA_ETH_MII_TIMER_VALUE(RBA_ETH_GPT_CLOCK_CONFIG));
       ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en = RBA_ETH_NONE;
       ReadWriteMiiManagRef_pst->rba_Eth_MiiCtrlIdx_u8 = CtrlIdx_u8;
       /*Enable GPT timer notification for getting interrupts after timer reaches target time*/
       Gpt_EnableNotification( RBA_ETH_MII_TIMER_CHANNEL );
#endif
#endif

#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
	/* To select the MAC interface (MII/RMII), register SIUL2_SCR0 has to be programmed*/
    if (Eth_CtrlMacPhyInterfaceType_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_INTERFACE_MII)
    {
        SchM_Enter_Eth(CONTROLLER);
        /* MII Interface is selected */
        RBA_ETH_ETHERNET_MODE_SEL_REG |= (uint32)(RBA_ETH_SIUL2_SCR0_ETHERNET_MODE_MII << RBA_ETH_SIUL2_SCR0_ETHERNET_MODE_POS(CtrlIdx_u8));
        SchM_Exit_Eth(CONTROLLER);
        Result_o = E_OK;
    }
    else if (Eth_CtrlMacPhyInterfaceType_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_INTERFACE_RMII)
    {
        SchM_Enter_Eth(CONTROLLER);
        /* In current design selection of RMII interface is not supported */
        /* RMII Interface is selected */
        /* The below line needs to be enabled when support for SIUL2 is present. In the current project stand SIUL2 is not present */
        RBA_ETH_ETHERNET_MODE_SEL_REG &= (uint32)(~(RBA_ETH_SIUL2_SCR0_ETHERNET_MODE_RMII << RBA_ETH_SIUL2_SCR0_ETHERNET_MODE_POS(CtrlIdx_u8)));
        SchM_Exit_Eth(CONTROLLER);
        Result_o = E_OK;
    }
    else
    {
        /* Ethernet MAC supports only MII, RMII interfaces */
    }
#endif

#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    /* Unlock MCU protection*/
    Eth_Krst_Mcu_Rb_ResetProtection();
    /* Clear the Kernel reset status in KRST0 by writing into KRSTCLR */
    Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->KRSTCLR = RBA_ETH_KRSTCLR_CLR << RBA_ETH_KRSTCLR_CLR_POS;

    /* Request for reset by writing into KRST0 and KRST1 */
    Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->KRST[0] = RBA_ETH_KRST0_RST << RBA_ETH_KRST0_RST_POS;
    Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->KRST[1] = RBA_ETH_KRST1_RST << RBA_ETH_KRST1_RST_POS;
    /* Lock MCU protection*/
    Eth_Krst_Mcu_Rb_SetProtection();

    /*Check if reset is complete*/

    CheckBitStatus_b = Eth_CheckBitFieldStatus(&(Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->KRST[0]),(RBA_ETH_KRST0_RSTSTAT << RBA_ETH_KRST0_RSTSTAT_POS), (RBA_ETH_KRST0_RSTSTAT << RBA_ETH_KRST0_RSTSTAT_POS), ETH_RESET_TIMEOUT, CtrlIdx_u8);

    if(CheckBitStatus_b != FALSE)
    {
        Result_o = E_OK;
        /*  Delay of 70 fspb cycles to be added before checking the status of RXQSTS in MTL_RXQ0_DEBUG.
         * The delay given is more than 70 cycles for worst case scenario */
        for (DelayKRST_u8 = 0;DelayKRST_u8 < RBA_ETH_DELAY_COUNTER_RXQSTS; DelayKRST_u8++)
		{
			;
		}
        SchM_Enter_Eth(CONTROLLER);
        /*Check the receive MTL Debug status */
		if(((Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->MTL_RXQ0_DEBUG) & MTL_RXQ0_DEBUG_RXQSTS_MASK) != 0U)
		{
			Result_o = E_NOT_OK;
		}
		else
		{
			Eth_Gpctl_Mcu_Rb_ResetProtection();
			/*EPR of ETH GPCTL register should be updated before SWR bit of DMA register 0 is handled. The value of this bit is
			* latched on only during module reset triggered by SWR bit */
			if (Eth_CtrlMacPhyInterfaceType_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_INTERFACE_MII)
			{
				Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->GPCTL = RBA_ETH_GPCTL_EPR_MIISELECT << RBA_ETH_GPCTL_EPR_POS;
			}
			else if (Eth_CtrlMacPhyInterfaceType_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_INTERFACE_RGMII)
			{
				Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->GPCTL = RBA_ETH_GPCTL_EPR_RGMIISELECT << RBA_ETH_GPCTL_EPR_POS;
			}
			else
			{
				Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->GPCTL = RBA_ETH_GPCTL_EPR_RMIISELECT << RBA_ETH_GPCTL_EPR_POS;
			}
			Eth_Gpctl_Mcu_Rb_SetProtection();
			Result_o = E_OK;
		}
		SchM_Exit_Eth(CONTROLLER);
    }
    else
    {   /* no -> DET error */
        /* empty else to avoid misra warning */
    }

#endif

    return(Result_o);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Initializes the indexed controller
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 * \param CfgIdx_u8  Index of the used configuration
 *
 * \return           Std_ReturnType {E_OK: success; E_NOT_OK: transceiver could not be initialized}
 *
     From the Ethernet MAC Manual:
    Initializing DMA - The initialization sequence in this section is used for MAC configurations
    Steps:
    1. Provide a software reset. This resets all of the MAC internal registers and logic. (Bus Mode Register - bit 0).
    2. Wait for the completion of the reset process (poll bit 0 of the Bus Mode Register, which
       is only cleared after the reset operation is completed).
    3. Initialise Transmit descriptor  with Buffer address , control status bits
       Update Tx Tail pointer, Tx List address , Tx Ring Length registers
    4. Initialise Receive descriptor with Buffer address , control status bits
       Update Rx Tail pointer , Rx List address , Rx Ring length
    5. Initiliase Tx buffer Management structure
    6. Prgram MTL register
    7. Program MAC registers
    8. Set Ethernet Speed
    9. Set Physical Address

 ***************************************************************************************************
 */
static Std_ReturnType rba_Eth_ControllerReset(uint8 CtrlIdx_u8)
{
    Eth_ControllerRefType_t       Controller_pst;   /* local pointer to management structure of Eth controller */
    rba_Eth_RegisterMapRefType_t  Registers_pst;    /* local pointer to register bank in HW (memory mapped) */
    Std_ReturnType                Result_o;         /* local variable holding the status of controlelr init*/
    boolean                       CheckBitStatus_b; /* local variable holding the status of SW reset and ahb status*/
    uint8                         DelaySWR_u8;

    Controller_pst                = &Eth_Controllers_ast[CtrlIdx_u8];
    Registers_pst                 = Controller_pst->Registers_pst;
    Result_o                      = E_NOT_OK;

    /* critical section because global structures are affected*/
    SchM_Enter_Eth(CONTROLLER);

    /* Soft reset. All other bits are unimportant.*/
    Registers_pst->DMA_MODE |= (RBA_ETH_DMA_MODE_SWR << RBA_ETH_DMA_MODE_SWR_POS);

    /* As per the actual behaviour SWR bit should be set to 1 just after writing into the register, but due to errata - it takes atleast 4 CSR cycle to write 1 in SWR bit
     * Solution 1 - Use API Eth_CheckBitFieldStatus() to check wheter 1 is writtern in SWR bit or not but as it is errata it is not clear that bit is set after 4 CSR cycle
     * or it will take less time so this API may wrongly handle this condition if Bit is set to 1 and agian set to 0 before execution of Eth_CheckBitFieldStatus() API
     * Soultion 2 - Read Back is also not a proper solutin as it is not mentioned that ReabBack will happen only after completion of writing process
     * Solution 3 - Provide delay (considering worst case scenario) after executing instruction of writing 1 in SWR bit so that sufficent cycle will be provided to write 1 in SWR bit*/

    /* CSR clock for Ethernet Peripheral is generated from PBRIDGE clock which is normally SYS_CLK/4, To generate 4 CSR delay atleast 16 wait instructions needs to be executed before reading
     * SWR bit, for worst case make delay of 20 cycles
     */
    for (DelaySWR_u8 = 0;DelaySWR_u8 < DELAY_COUNTER_FOR_SWR;DelaySWR_u8++)
    {
        ;
    }
    /*wait for Soft reset to finish. This calls DEM. (Note: ignoring ret value for now) */
    CheckBitStatus_b = Eth_CheckBitFieldStatus(&(Registers_pst->DMA_MODE), (RBA_ETH_DMA_MODE_SWR << RBA_ETH_DMA_MODE_SWR_POS),0,ETH_RESET_TIMEOUT, CtrlIdx_u8);

    SchM_Exit_Eth(CONTROLLER);

    if(CheckBitStatus_b != FALSE)
    {
            /* proper Rx / Tx descriptor chains */
            rba_Eth_TxRxDescriptorQueueInit(CtrlIdx_u8, Controller_pst, Registers_pst);

            /* Initialize TX Buffer management structure. For RX no structure needed. */
            Eth_TxBuffer_TransmitBufferQueueInit(CtrlIdx_u8);

            /* critical section because global structures are affected*/
            SchM_Enter_Eth(CONTROLLER);
            /* Initialise the MTL block */
            rba_Eth_ProgramMTLMACRegs(Registers_pst);


            SchM_Exit_Eth(CONTROLLER);

            if(Eth_CtrlSpeedMode_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_SPEED_10MBPS)
            {
                /* critical section because global structures are affected*/
                SchM_Enter_Eth(CONTROLLER);

                Registers_pst->MAC_CONFIGURATION |=
                ((RBA_ETH_MAC_CONFIGURATION_FES_10MBPS  << RBA_ETH_MAC_CONFIGURATION_FES_POS)  |
                (RBA_ETH_MAC_CONFIGURATION_PS_MII   << RBA_ETH_MAC_CONFIGURATION_PS_POS));

                SchM_Exit_Eth(CONTROLLER);

            }
            else if(Eth_CtrlSpeedMode_acu8_MP[CtrlIdx_u8] == ETH_NET_CFG_SPEED_100MBPS)
            {
                /* critical section because global structures are affected*/
                SchM_Enter_Eth(CONTROLLER);

                Registers_pst->MAC_CONFIGURATION |=
                ((RBA_ETH_MAC_CONFIGURATION_FES_100MBPS  << RBA_ETH_MAC_CONFIGURATION_FES_POS)  |
                (RBA_ETH_MAC_CONFIGURATION_PS_MII   << RBA_ETH_MAC_CONFIGURATION_PS_POS));

                SchM_Exit_Eth(CONTROLLER);

            }
            else
            {
                /* critical section because global structures are affected*/
                SchM_Enter_Eth(CONTROLLER);

                /*FES to be explicity made 0 for GMII?*/
                Registers_pst->MAC_CONFIGURATION |=
                (RBA_ETH_MAC_CONFIGURATION_PS_GMII   << RBA_ETH_MAC_CONFIGURATION_PS_POS);

                SchM_Exit_Eth(CONTROLLER);

            }

            /* MAC SARC addreses - set the sarc */
            /* write configured local MAC address to the Eth Controller*/
            rba_Eth_SetPhysAddr(CtrlIdx_u8);
            /*Everything is fine if we have reached here*/
            Result_o = E_OK;
    }
    else /* If Software reset was not successful*/
    {
        /*Do nothing*/
    }

    return(Result_o);
}


/**
 ***************************************************************************************************
 * \moduledescription
 * This function also (dis/en)ables the interrupts per controller if supported.
 * Enables / disables the indexed controller
 * \par Synchronous, Non Reentrant
 *
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 * \param CfgMode_e  ETH_MODE_DOWN: disable the controller; ETH_MODE_ACTIVE: enable the controller
 *
 * \return           Std_ReturnType {E_OK: success; E_NOT_OK: transceiver could not be initialized}
 *
 STEPS:
   - enter critical section to avoid nasty interrupts at this sensitive time
   - if the ETH is activated:
         - reset the descriptors and TX buffer management
         - enable RX&TX operation mode at MAC
         - enable DMA interrupts.
         - clear any possible interrupts status (optional)
         - enable MAC rx&tx
   - if ETH is deactivated: pretty much the reverse of the steps above
         - disable MAC
         - wait for current transfers to finish
                (optional, sanity only. it is fast anyway because of MAC stop)
         - disable DMA rx&tx
         - clear any left over IRQ status bits
         - reset the descriptor structures at this point...
         (not needed actually)
    -exit critical section
    Note: AFTER the ETH deactivation, a previously pending IRQ may STILL come!!
    the IRQ will have to check the current Eth mode and do nothing if inactive.
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_SetControllerMode(uint8 CtrlIdx_u8,
                                                Eth_ModeType CtrlMode_en)
{
        Std_ReturnType RetVal_en;
        Eth_ControllerRefType_t      Controller_pst;  /* local pointer to global management structure per Eth controller */
        rba_Eth_RegisterMapRefType_t Registers_pst;   /* local pointer to register map structure of the Ethernet MAC (controller) */
        boolean                      CheckBitStatusMAC_b;/* local variable holding the status MAC operations*/
    #ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
        boolean                      CheckBitStatusDMA_b;/* local variable holding the status of DMA operations*/
    #endif

        RetVal_en = E_NOT_OK;
        Controller_pst = &Eth_Controllers_ast[CtrlIdx_u8];
        Registers_pst  = Controller_pst->Registers_pst;


        if (CtrlMode_en == ETH_MODE_ACTIVE) /* change to ETH_MODE_ACTIVE requested? */
        {

            RetVal_en = rba_Eth_ControllerReset(CtrlIdx_u8);

            if (RetVal_en == E_OK)
            {

                /* start critical section, since in the following lines, register values are set */
                SchM_Enter_Eth(CONTROLLER);

                /* Enable the Tx and Rx DMAs to enter into Running State */
                Registers_pst->DMA_CH0_TX_CONTROL |= (RBA_ETH_DMA_CH_N_TX_CONTROL_ST << RBA_ETH_DMA_CH_N_TX_CONTROL_ST_POS);
                Registers_pst->DMA_CH0_RX_CONTROL |= (RBA_ETH_DMA_CH_N_RX_CONTROL_SR << RBA_ETH_DMA_CH_N_RX_CONTROL_SR_POS);
				/* Update RBSZ field as per the Rx buffer configured length */
                Registers_pst->DMA_CH0_RX_CONTROL |= ((uint32)Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16 << RBA_ETH_DMA_CH_N_RX_CONTROL_RBSZ_POS);

                /* Disable and clear all interrupts */
                Registers_pst->DMA_CH0_INTERRUPT_ENABLE = 0;
#if defined (RBA_ETH_GLOBAL_TIME_SUPPORT)
#if (RBA_ETH_GLOBAL_TIME_SUPPORT == STD_ON)

        Registers_pst->MAC_SUB_SECOND_INCREMENT             =  (RBA_ETH_SUB_SECOND_INC_VALUE << RBA_ETH_MAC_SUB_SEC_SEDCOND_INC_POS) ;/*(RBA_ETH_PTP_CLK_REF << RBA_ETH_SUB_SEC_SEDCOND_INC_POS);*/ /* This value is loaded with the sub nano second increment value */

        Registers_pst->MAC_TIMESTAMP_CONTROL                = ((RBA_ETH_MAC_TIME_STAMP_EN << RBA_ETH_MAC_TIME_STAMP_EN_POS)  |
                                                               (RBA_ETH_MAC_TIME_STAMP_INIT << RBA_ETH_MAC_TIME_STAMP_INIT_POS)|
                                                               (RBA_ETH_MAC_TIME_STAMP_RECEIVE_ENALL << RBA_ETH_MAC_TIME_STAMP_ENALL_POS)|
                                                               (RBA_ETH_MAC_TIME_STAMP_IPENA << RBA_ETH_MAC_TIME_STAMP_IPENA_POS)|
                                                               (RBA_ETH_MAC_TIME_STAMP_VER2 << RBA_ETH_MAC_TIME_STAMP_VER2_POS)|
                                                               (RBA_ETH_MAC_TIME_STAMP_CTRLSR << RBA_ETH_MAC_TIME_STAMP_CTRLSR_POS)|
                                                               (RBA_ETH_MAC_TIME_STAMP_SNAPSEL << RBA_ETH_MAC_TIME_STAMP_SNAPSEL_POS)
                                                              );

#endif
#endif

                /* Enable dma interrupt */
                if (Eth_CtrlEnableRxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
                {
                    /* Clear normal IRQ status sticky bit(s)  by writing 1 (NIS =normal interrupts and AIS = abnormal interrupts)*/
                    Registers_pst->DMA_CH0_STATUS |= RBA_ETH_DMA_CH_N_STATUS_HANDLEDRXIRQBITS;

                    Registers_pst->DMA_CH0_INTERRUPT_ENABLE = RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_HANDLEDRXIRQBITS;

                }
                else
                {
                    /*Rx Interrupt de-activated*/
                }

                /* Enable dma interrupt */
                if (Eth_CtrlEnableTxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
                {
                    /* Clear normal IRQ status sticky bit(s)  by writing 1 (NIS =normal interrupts and AIS = abnormal interrupts)*/
                    Registers_pst->DMA_CH0_STATUS |= RBA_ETH_DMA_CH_N_STATUS_HANDLEDTXIRQBITS;

                    Registers_pst->DMA_CH0_INTERRUPT_ENABLE |= RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_HANDLEDTXIRQBITS;

                }
                else
                {
                    /*Tx Interrupt de-activated*/
                }

                /* Finally GO! Enable MAC Rx and Tx */
                Registers_pst->MAC_CONFIGURATION |= ((RBA_ETH_MAC_CONFIGURATION_RE << RBA_ETH_MAC_CONFIGURATION_RE_POS) |
                                                     (RBA_ETH_MAC_CONFIGURATION_TE << RBA_ETH_MAC_CONFIGURATION_TE_POS) );

                /* Exit the critical section */
                SchM_Exit_Eth(CONTROLLER);

            }
            else
            {
                /*nothing to do.*/
            }
        }
        else if (CtrlMode_en == ETH_MODE_DOWN)
        {

            /* start critical section, since in the following lines, register values are set */
            SchM_Enter_Eth(CONTROLLER);

            /* Disable MAC Rx and Tx: this will cause any transaction in progress to fail soon */
            Registers_pst->MAC_CONFIGURATION &= ~((RBA_ETH_MAC_CONFIGURATION_RE << RBA_ETH_MAC_CONFIGURATION_RE_POS) |
                                                 (RBA_ETH_MAC_CONFIGURATION_TE << RBA_ETH_MAC_CONFIGURATION_TE_POS) );

            /* Exit the critical section */
            SchM_Exit_Eth(CONTROLLER);

            /* Wait for any pending DMA transfers to finish. Just in case, to avoid any problems*/
            /* Here, Transmit Queue (MTL_TXQ0_DEBUG) and Receive Queue(MTL_RXQ0_DEBUG)can also be read
             * Time for ETH_AHBTIMEOUTS needs to be decided */
            CheckBitStatusMAC_b = Eth_CheckBitFieldStatus(&(Registers_pst->MAC_DEBUG),RBA_ETH_MAC_DEBUG_ALL, 0, ETH_AHBTIMEOUTS, CtrlIdx_u8);

            if(FALSE == CheckBitStatusMAC_b)
            {
                RetVal_en = E_NOT_OK;
            }
            else
            {
                /* start critical section, since in the following lines, register values are set */
                SchM_Enter_Eth(CONTROLLER);

                /* Disable DMA Tx,Rx */
                Registers_pst->DMA_CH0_TX_CONTROL &= ~(RBA_ETH_DMA_CH_N_TX_CONTROL_ST << RBA_ETH_DMA_CH_N_TX_CONTROL_ST_POS);
                Registers_pst->DMA_CH0_RX_CONTROL &= ~(RBA_ETH_DMA_CH_N_RX_CONTROL_SR << RBA_ETH_DMA_CH_N_RX_CONTROL_SR_POS);

                #ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
                    CheckBitStatusDMA_b = Eth_CheckBitFieldStatus(&(Registers_pst->DMA_DEBUG_STATUS0),RBA_ETH_DMA_DEBUG_RPS0_TPS0, 0, ETH_AHBTIMEOUTS, CtrlIdx_u8);
					/*Check if the Rx and Tx DMA are in stopped state */
					/*Check the status of Transmit Queue (MTL_TXQ0_DEBUG) and Receive Queue(MTL_RXQ0_DEBUG)*/
					if((FALSE == CheckBitStatusDMA_b) || (Registers_pst->MTL_RXQ0_DEBUG != 0U) || (Registers_pst->MTL_TXQ0_DEBUG != 0U))
					{
						RetVal_en = E_NOT_OK;
					}
					else
				#endif
					{
						/* Disable and clear all interrupts */
						Registers_pst->DMA_CH0_INTERRUPT_ENABLE = 0;

						/* Clear transmit status, interrupts or other bits. Write 1 to clear the sticky reg */
						Registers_pst->DMA_CH0_STATUS |= (RBA_ETH_DMA_CH_N_STATUS_HANDLEDTXIRQBITS | RBA_ETH_DMA_CH_N_STATUS_HANDLEDRXIRQBITS);
						RetVal_en = E_OK;
					}


                /* Exit the critical section */
                SchM_Exit_Eth(CONTROLLER);

            }

        }
        else
        {
            /*invalid mode. Do nothing*/
        }


        return RetVal_en;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Sets the physical source address used by the indexed controller ==
 * Write configured local MAC address to the Eth Controller register
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 *
 * \return           None
 *
 ***************************************************************************************************
 */

void rba_Eth_SetPhysAddr(uint8 CtrlIdx_u8)
{

    rba_Eth_RegisterMapRefType_t    Registers_pst;          /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint32                          PhysAddrBottom_u32;     /* MAC address has to be store in register by 2 words. This is the bottom part */
    uint32                          PhysAddrTop_u32;        /* MAC address has to be store in register by 2 words. This is the top part */

    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* shift the phys address bytes into the bottom and top words */
    rba_Eth_AssembleBottomAndTopPhysAddress(&PhysAddrBottom_u32, &PhysAddrTop_u32, &Eth_CtrlPhysAddress_au8_MP[CtrlIdx_u8][0]);

    /* start critical section, since in the following lines, register values are set */
    SchM_Enter_Eth(CONTROLLER);

    /* copy MAC address into specific address register 1 */
    /** If MAC address is FC:D6:BD:00:00:01
     * then assembling of PHY address assembling should be as below
     * MACADD_LOW = 0x00BDD6FC
     * MACADD_HIGH  = 0xxxxx0100
     */

    /* copy MAC address into specific address register 0 */

    Registers_pst->GMA_MAC_ADDR[0].high_u32 = PhysAddrTop_u32 | (RBA_ETH_MAC_ADDRESS0_HIGH_AE << RBA_ETH_MAC_ADDRESS0_HIGH_AE_POS);
    Registers_pst->GMA_MAC_ADDR[0].low_u32  = PhysAddrBottom_u32;

    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

}

/**
 ***************************************************************************************************
 * \moduledescription
 *
 * Update the physical source address to/from the indexed controller
 * filter. If the Ethernet Controller is not capable to do the filtering then software has to do this.
 *
 * \par Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8       Index of the controller within the context of the Ethernet Driver
 * \param PhysAddrPtr_pcu8 Physical source address (MAC address) in network byte order to be entered
 *                         or removed in RX Filter register.
 * \param Action_en        Add or remove the address from the Ethernet controllers filter
 *
 * \return                 Std_ReturnType {E_OK: success; E_NOT_OK: update of RX PhysAddrFilter failed}
 *
 ***************************************************************************************************
 */

#if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
Std_ReturnType rba_Eth_UpdatePhysAddrFilter(uint8 CtrlIdx_u8,
                                            const uint8 *PhysAddrPtr_pcu8,
                                            Eth_FilterActionType Action_en)
{
    rba_Eth_RegisterMapRefType_t    Registers_pst;          /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint32          addr_idx_u32;           /*  the addr to operate on: the match or an unused slot */
    uint32          PhysAddrBottom_u32;     /* MAC address has to be store in register by 2 words. This is the bottom part */
    uint32          PhysAddrTop_u32;        /* MAC address has to be store in register by 2 words. This is the top part */
    Std_ReturnType  Result_o;

    Result_o = E_NOT_OK;
    PhysAddrBottom_u32 =0;
    PhysAddrTop_u32 = 0;

    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* shift the phys address bytes into the bottom and top words */
    rba_Eth_AssembleBottomAndTopPhysAddress(&PhysAddrBottom_u32, &PhysAddrTop_u32, PhysAddrPtr_pcu8);

    /* Is requested MAC set to 00:00:00:00:00:00?*/
    if((PhysAddrBottom_u32 == 0UL) && (PhysAddrTop_u32 == 0UL))
    {   /* Yes */

        switch(Action_en)
        {
            case ETH_ADD_TO_FILTER:
            {
                SchM_Enter_Eth(CONTROLLER);
                /* All RX Filters shall be disabled (except the unicast Filter at position 0)
                 * and promiscuous mode shall be disabled */
                Registers_pst->MAC_PACKET_FILTER &= ~(1UL<<RBA_ETH_MAC_PACKET_FILTER_PR_POS);
                SchM_Exit_Eth(CONTROLLER);

                rba_Eth_DisableAllMacAddrFilters(Registers_pst);
                Result_o = E_OK;
            }
            break;

            case ETH_REMOVE_FROM_FILTER:
            {
                /* Autosar doesnot specify anything about this action*/
            }
            break;

            default:
            {
                /* The Default case added as CDG coding guidelines requires default case for every switch. The code will never come to this section */
            }
            break;
        }

    }
    /* Is requested MAC set to FF:FF:FF:FF:FF:FF? */
    else if((PhysAddrBottom_u32 == RBA_ETH_BROADCAST_ADDR_LOW) && (PhysAddrTop_u32 == RBA_ETH_BROADCAST_ADDR_HIGH))
    {   /* Yes */

        switch(Action_en)
        {
            case ETH_ADD_TO_FILTER:
            {
                /* Completely open the filter
                 * Promiscuous mode shall be Enabled */
                SchM_Enter_Eth(CONTROLLER);
                Registers_pst->MAC_PACKET_FILTER |= (1UL<<RBA_ETH_MAC_PACKET_FILTER_PR_POS);
                SchM_Exit_Eth(CONTROLLER);

                Result_o = E_OK;
            }
            break;

            case ETH_REMOVE_FROM_FILTER:
            {
                /* Enable all address filters enabled in previous ADD_TO_FILTER callsAll.
                 * and Promiscuous mode shall be disabled */
                SchM_Enter_Eth(CONTROLLER);
                Registers_pst->MAC_PACKET_FILTER &= ~(1UL<<RBA_ETH_MAC_PACKET_FILTER_PR_POS);
                SchM_Exit_Eth(CONTROLLER);

                Result_o = E_OK;
            }
            break;

            default:
            {
                /* The Default case added as CDG coding guidelines requires default case for every switch. The code will never come to this section */
            }
            break;
        }
    }
    else
    {

        switch ( Action_en )
        {
            /* Copy MAC address into specific address register */
            case ADD_TO_FILTER:
            {
                addr_idx_u32 = rba_Eth_FindFirstUnusedOrMatchingAddr(Registers_pst, PhysAddrTop_u32, PhysAddrBottom_u32, RBA_ETH_SEARCHUSEDFLAG);
                if (addr_idx_u32 != 0UL)
                {
                    /* copy MAC address into specific address register 0, Enable AE for filtering */
                    SchM_Enter_Eth(CONTROLLER);
                    Registers_pst->GMA_MAC_ADDR[addr_idx_u32].high_u32 = (PhysAddrTop_u32 | (RBA_ETH_MAC_ADDRESS_HIGH_AE << RBA_ETH_MAC_ADDRESS_HIGH_AE_POS));
                    /* DMA Channel number to which the Rx packet whose DA
                     * matches the MAC Address1 content is routed*/
                    Registers_pst->GMA_MAC_ADDR[addr_idx_u32].high_u32 &= (~(RBA_ETH_MAC_ADDRESS_HIGH_DCS_MASK));
                    Registers_pst->GMA_MAC_ADDR[addr_idx_u32].low_u32 = PhysAddrBottom_u32;
                    SchM_Exit_Eth(CONTROLLER);

                    Result_o = E_OK;
                }
                else
                {
                    /* No free slot available, return E_NOT_OK */
                }
            }
            break;

            /* The case checks if the address passed is in filter if yes removes it from the filter */
            case REMOVE_FROM_FILTER:
            {
                addr_idx_u32 = rba_Eth_FindFirstUnusedOrMatchingAddr(Registers_pst, PhysAddrTop_u32, PhysAddrBottom_u32, 0);
                if (addr_idx_u32 != 0UL)
                {
                    SchM_Enter_Eth(CONTROLLER);
                    Registers_pst->GMA_MAC_ADDR[addr_idx_u32].high_u32 = 0; /* clears the AE Enabled bit */
                    Registers_pst->GMA_MAC_ADDR[addr_idx_u32].low_u32 = 0; /* not absolutely necessary, but neatly set to 0 */
                    SchM_Exit_Eth(CONTROLLER);

                    Result_o = E_OK;
                }
                else
                {
                    /* No matching slot return E_NOT_OK. */
                }

            }
            break;

            default:
            {
                /* The Default case added as CDG coding guidelines requires default case for every switch. The code will never come to this section */
            }
            break;
        }
    }
    return(Result_o);

}
#endif

#if defined RBA_ETH_EN_MII
#if (RBA_ETH_EN_MII == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Configures a transceiver register or triggers a function offered by the receiver
 * \par Asynchronous, Non Reentrant.
 *
 * Parameter In:
 * \param CtrlIdx_u8 Index of the controller within the context of the Ethernet Driver
 * \param TrcvIdx_u8 Index of the transceiver on the MII
 * \param RegIdx_u8  Index of the transceiver register on the MII
 * \param RegVal_u16 Value to be written into the indexed register
 *
 * \return           Std_ReturnType {E_OK: MII register write success;}
 *
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_WriteMii(uint8 CtrlIdx_u8,
                                uint8 TrcvIdx_u8,
                                uint8 RegIdx_u8,
                                uint16 RegVal_u16)
{
        uint32                              tmpreg_u32;    /* local variable holding the value to be written into GMII_ADDRESS register*/
        Std_ReturnType                      Eth_ReturnVal; /* Return value informing the status of WriteMii*/
        rba_Eth_RegisterMapRefType_t        Registers_pst; /* local pointer to register map structure of the Ethernet MAC (controller) */
#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
        rba_Eth_ReadWriteMiiManagRefType_t  ReadWriteMiiManagRef_pst; /* local pointer to global management structure of read/writemii */
        ReadWriteMiiManagRef_pst = &rba_Eth_ReadWriteMiiManag_st;
#else
        boolean                             Check_b;       /* local variable holding the Status of WriteMII*/
        Check_b = FALSE;
#endif
#endif
        Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;
        Eth_ReturnVal = E_OK;

        /*  CSR Clock Range CR[2:0] bits value */
        tmpreg_u32 = (RBA_ETH_MAC_MDIO_ADDRESS_CR << RBA_ETH_MAC_MDIO_ADDRESS_CR_POS);

        /* Prepare the MII register address value */
        /* Set the PHY device address */
        tmpreg_u32 |= (((uint32)TrcvIdx_u8 << RBA_ETH_MAC_MDIO_ADDRESS_PA_POS) & (RBA_ETH_MAC_MDIO_ADDRESS_PA_MASK));

        /* Set the PHY register address */
        tmpreg_u32 |= (((uint32)RegIdx_u8 << RBA_ETH_MAC_MDIO_ADDRESS_RDA_POS) & RBA_ETH_MAC_MDIO_ADDRESS_RDA_MASK);
        tmpreg_u32 |= (RBA_ETH_MAC_MDIO_ADDRESS_GOC_W << RBA_ETH_MAC_MDIO_ADDRESS_GOC_POS); /* Set the write mode */
        tmpreg_u32 |= (RBA_ETH_MAC_MDIO_ADDRESS_GB << RBA_ETH_MAC_MDIO_ADDRESS_GB_POS); /* Set the MII Busy bit */

        /* start critical section in order to prevent other tasks from accessing MII */
        SchM_Enter_Eth(CONTROLLER);

#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
        /* Save Controller index and API called into a global structure variable */
        ReadWriteMiiManagRef_pst->rba_Eth_MiiCtrlIdx_u8 = CtrlIdx_u8;
        ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en = RBA_ETH_WRITEMII;
#endif
#endif

        /* Give the value to the MII data register */
        Registers_pst->MAC_MDIO_DATA = RegVal_u16;

        /* Write the result value into the MII Address register */
        Registers_pst->MAC_MDIO_ADDRESS = tmpreg_u32;
        SchM_Exit_Eth(CONTROLLER);

#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
        /* Asynchronous mode of MII communication */
        /* Start GPT Timer with a target time
         * Interrupt will be triggered after timer target time is reached */
        Gpt_StartTimer( RBA_ETH_MII_TIMER_CHANNEL, ReadWriteMiiManagRef_pst->rba_Eth_ReadWriteMiiGptClkTick_u32 );
#else
        /* Synchronous mode for MII communication */
       /*Ideally Eth_CheckBitFieldStatus should have been inside interrupt locks. But this increases the interrupt lock time*/
       /*Short term solution: Move Eth_CheckBitFieldStatus outside the critical section. */
       /*EthTrcv will call Eth_ReadMii and Eth_WriteMii in non-re-entrant way. Also EthTrcv will call either Eth_ReadMii Or Eth_WriteMii at a time (Read and write functions will not pre-empt each other).*/
       Check_b = Eth_CheckBitFieldStatus(&(Registers_pst->MAC_MDIO_ADDRESS), (RBA_ETH_MAC_MDIO_ADDRESS_GB << RBA_ETH_MAC_MDIO_ADDRESS_GB_POS),0, ETH_MDIOTIMEOUTUS, CtrlIdx_u8);

       if(Check_b != FALSE)
       {
           /* Successful write into the MII register */
       }
       else
       {
           Eth_ReturnVal = E_NOT_OK;
       }
#endif
#endif

        return Eth_ReturnVal;

}
/**
 ***************************************************************************************************
 * \moduledescription
 * Reads a transceiver register
 * \par Asynchronous, Non Reentrant.
 *
 * Parameter In:
 * \param CtrlIdx_u8     Index of the controller within the context of the Ethernet Driver
 * \param TrcvIdx_u8     Index of the transceiver on the MII
 * \param RegIdx_u8      Index of the transceiver register on the MII
 *
 * Parameter Out:
 * \param RegValPtr_pu16 Filled with the register content of the indexed register
 *
 * \return               Std_ReturnType {E_OK: MII register read;}
 *
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_ReadMii(uint8 CtrlIdx_u8,
                               uint8 TrcvIdx_u8,
                               uint8 RegIdx_u8,
                               uint16 *RegValPtr_pu16)
{
       uint32                          tmpreg_u32;    /* local variable holding the value to be written into GMII_ADDRESS register*/
       Std_ReturnType                  Eth_ReturnVal; /* Return value informing the status of ReadMii*/
       rba_Eth_RegisterMapRefType_t    Registers_pst; /* local pointer to register map structure of the Ethernet MAC (controller) */
#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
       rba_Eth_ReadWriteMiiManagRefType_t  ReadWriteMiiManagRef_pst; /* local pointer to global management structure of read/writemii */
       ReadWriteMiiManagRef_pst = &rba_Eth_ReadWriteMiiManag_st;
#else
        boolean                             Check_b;       /* local variable holding the Status of ReadMII*/
        Check_b = FALSE;
#endif
#endif

       Eth_ReturnVal = E_OK;
       Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

       /*  CSR Clock Range CR[2:0] bits value */
       tmpreg_u32 = (RBA_ETH_MAC_MDIO_ADDRESS_CR << RBA_ETH_MAC_MDIO_ADDRESS_CR_POS);

       *RegValPtr_pu16 = 0;
       /* Prepare the MII address register value */
       /* Set the PHY device address */
       tmpreg_u32 |= (((uint32)TrcvIdx_u8 << RBA_ETH_MAC_MDIO_ADDRESS_PA_POS) & RBA_ETH_MAC_MDIO_ADDRESS_PA_MASK);
       /* Set the PHY register address */
       tmpreg_u32 |= (((uint32)RegIdx_u8 << RBA_ETH_MAC_MDIO_ADDRESS_RDA_POS) & RBA_ETH_MAC_MDIO_ADDRESS_RDA_MASK);
       tmpreg_u32 |= (RBA_ETH_MAC_MDIO_ADDRESS_GOC_R << RBA_ETH_MAC_MDIO_ADDRESS_GOC_POS); /* Set the read mode */


       /* start critical section in order to prevent other tasks from accessing MII*/
       SchM_Enter_Eth(CONTROLLER);
#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
       /*Save Controller index and API called into a global structure variable */
       ReadWriteMiiManagRef_pst->rba_Eth_MiiCtrlIdx_u8 = CtrlIdx_u8;
       ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en = RBA_ETH_READMII;
#endif
#endif
       Registers_pst->MAC_MDIO_ADDRESS = tmpreg_u32;

       tmpreg_u32 |= (RBA_ETH_MAC_MDIO_ADDRESS_GB << RBA_ETH_MAC_MDIO_ADDRESS_GB_POS); /* Set the MII Busy bit */
       /* Write the result value into the MII Address register */
       Registers_pst->MAC_MDIO_ADDRESS = tmpreg_u32;

       SchM_Exit_Eth(CONTROLLER);

#if defined (RBA_ETH_ASYNCMII_SUPPORT)
#if (RBA_ETH_ASYNCMII_SUPPORT == STD_ON)
       /* Start GPT Timer with a Target time ticks
        * Interrupt will be triggered after timer target time is reached*/
       Gpt_StartTimer(RBA_ETH_MII_TIMER_CHANNEL,ReadWriteMiiManagRef_pst->rba_Eth_ReadWriteMiiGptClkTick_u32);
#else
       /*Ideally Eth_CheckBitFieldStatus should have been inside interrupt locks. But this increases the interrupt lock time*/
       /*Short term solution: Move Eth_CheckBitFieldStatus outside the critical section. */
       /*EthTrcv will call Eth_ReadMii and Eth_WriteMii in non-re-entrant way. Also EthTrcv will call either Eth_ReadMii Or Eth_WriteMii at a time (Read and write functions will not pre-empt each other).*/

       /* Check for the Busy flag. */
       Check_b = Eth_CheckBitFieldStatus(&(Registers_pst->MAC_MDIO_ADDRESS), (RBA_ETH_MAC_MDIO_ADDRESS_GB << RBA_ETH_MAC_MDIO_ADDRESS_GB_POS),0, ETH_MDIOTIMEOUTUS, CtrlIdx_u8);

       if(Check_b != FALSE)
       {
           /* start critical section in order to prevent other tasks from accessing MII*/
           SchM_Enter_Eth(CONTROLLER);

           /* retrieve transceiver register data value from PHY maintenance register */
           *RegValPtr_pu16 = (uint16)(Registers_pst->MAC_MDIO_DATA & (RBA_ETH_MAC_MDIO_DATA_GD << RBA_ETH_MAC_MDIO_DATA_GD_POS));

           SchM_Exit_Eth(CONTROLLER);

       }
       else
       {
          Eth_ReturnVal = E_NOT_OK;
       }
#endif
#endif

       return Eth_ReturnVal;
}
#endif
#endif

#if defined(ETH_GET_COUNTER_VALUE_API)
 #if (ETH_GET_COUNTER_VALUE_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Reads the list of counter values from indexed controller.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param CounterPtr_pst   contains counter values according to IETF RFC 1757, RFC 1643 and RFC 2233.
 *
 *
 * \return   Std_ReturnType    E_OK: success
 *                             E_NOT_OK:  counter values could not be obtained
 *
 ***************************************************************************************************
 */

Std_ReturnType rba_Eth_GetCounterValues(uint8 CtrlIdx_u8,
                                    Eth_CounterType* CounterPtr_pst)
{
    /* local pointer to register map structure of the Ethernet MAC (controller) */
    rba_Eth_RegisterMapRefType_t    Registers_pst;
    Std_ReturnType                  ReturnVal_u8;                        /* Return value informing the status of
                                                                         GetCounterValue*/
    ReturnVal_u8 = E_OK;
    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* critical section because registers are read*/
    SchM_Enter_Eth(CONTROLLER);

    /*dropped packets due to buffer overrun*/
    CounterPtr_pst->DropPktBufOverrun   = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                /* No register available for DropPktBufOverrun error*/
    CounterPtr_pst->DropPktCrc          = Registers_pst->RX_CRC_ERROR_PACKETS;                  /*dropped packets due to CRC errors*/
    CounterPtr_pst->UndersizePkt        = Registers_pst->RX_UNDERSIZE_PACKETS_GOOD;             /*number of undersize packets less than
                                                                                                   64 octets long without errors*/
    CounterPtr_pst->OversizePkt         = Registers_pst->RX_OVERSIZE_PACKETS_GOOD;               /*number of oversize packets longer than
                                                                                                   1518 octets long without errors*/
    CounterPtr_pst->AlgnmtErr           = Registers_pst->RX_ALIGNMENT_ERROR_PACKETS;            /*number of alignment errors that do not
                                                                                                   pass CRC*/
    CounterPtr_pst->SqeTestErr          = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                 /* No register counter for SQE test error*/

    /*No register available for number of inbound packets discarded due to freeing of buffer*/
    CounterPtr_pst->DiscInbdPkt         = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;

    /*No register available for number of erroneous inbound packets*/
    CounterPtr_pst->ErrInbdPkt          = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;

    /*No register available for total number of outbound packets discarded to free up buffer space*/
    CounterPtr_pst->DiscOtbdPkt         = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;
    CounterPtr_pst->ErrOtbdPkt          = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                    /*No register available for total number of
                                                                                                       erroneous outbound packets*/
    CounterPtr_pst->SnglCollPkt         = Registers_pst->TX_SINGLE_COLLISION_GOOD_PACKETS;          /*Count of transmitted frames inhibited
                                                                                                       by one collision*/
    CounterPtr_pst->MultCollPkt         = Registers_pst->TX_MULTIPLE_COLLISION_GOOD_PACKETS;        /*Count of transmitted frames inhibited
                                                                                                       by more than one collision*/
    CounterPtr_pst->DfrdPkt             = Registers_pst->TX_DEFERRED_PACKETS;                        /*Count of deferred transmissions*/
    CounterPtr_pst->LatCollPkt          = Registers_pst->TX_LATE_COLLISION_PACKETS;                  /*Number of late collisions*/
    CounterPtr_pst->HwDepCtr0           = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                     /* No register available for Hardware
                                                                                                        dependent counter value 0*/
    CounterPtr_pst->HwDepCtr1           = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                     /* No register available for Hardware
                                                                                                        dependent counter value 1*/
    CounterPtr_pst->HwDepCtr2           = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                     /* No register available for Hardware
                                                                                                        dependent counter value 2*/
    CounterPtr_pst->HwDepCtr3           = RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE;                      /* No register available for Hardware
                                                                                                        dependent counter value 3*/
    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

    return ReturnVal_u8;
}

 #endif
#endif

#if defined(ETH_GET_RX_STATS_API)
 #if (ETH_GET_RX_STATS_API == STD_ON)
/**
 ***********************************************************************************************************************************
 * \moduledescription
 * Reads the list of Receive statics counter of the indexed controller.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param RxStats_pst    contains List of values according to IETF RFC 2819 (Remote Network Monitoring Management Information Base)
 *
 * \return   Std_ReturnType    E_OK: success
 *                             E_NOT_OK: Statistics counter could not be obtained
 *
 ***********************************************************************************************************************************
 */
Std_ReturnType rba_Eth_GetRxStats(uint8 CtrlIdx_u8,
                                     Eth_RxStatsType *RxStats_pst)
{
    /* local pointer to register map structure of the Ethernet MAC (controller) */
    rba_Eth_RegisterMapRefType_t    Registers_pst;

    /* Return value informing the status of RxStats*/
    Std_ReturnType                  ReturnVal_u8;

    ReturnVal_u8 = E_OK;
    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* critical section because registers are read*/
    SchM_Enter_Eth(CONTROLLER);

    /*No register available for total number of events in which frames are dropped*/
    RxStats_pst->RxStatsDropEvents              = RBA_ETH_GETRXSTATS_INVALID_VALUE;
    /*if register RX_OCTET_COUNT_GOOD_BAD reeached its maximum limit then RxStatsOctets updated with
     * (0xFFFFFFFFFFFFFFFFUL) value becasue of uint64 data type */
    if(Registers_pst->RX_OCTET_COUNT_GOOD_BAD == RBA_ETH_GETRXSTATS_INVALID_VALUE)
    {
        RxStats_pst->RxStatsOctets = RBA_ETH_GETTXRXSTATS_U64_INVALID_VALUE;
    }
    else
    {
        RxStats_pst->RxStatsOctets = (uint64)Registers_pst->RX_OCTET_COUNT_GOOD_BAD;       /*Total number of good and bad bytes received*/
    }
    RxStats_pst->RxStatsPkts                    = Registers_pst->RX_PACKETS_COUNT_GOOD_BAD;     /*Total number of good and bad frames received*/
    RxStats_pst->RxStatsBroadcastPkts           = Registers_pst->RX_BROADCAST_PACKETS_GOOD;     /*Total number of good broadcast frames received*/
    RxStats_pst->RxStatsMulticastPkts           = Registers_pst->RX_MULTICAST_PACKETS_GOOD;     /*Total number of good multicast frames received*/

    /*Total number of frames received with CRC and alignment erros*/
    RxStats_pst->RxStatsCrcAlignErrors          = Registers_pst->RX_CRC_ERROR_PACKETS;
    RxStats_pst->RxStatsCrcAlignErrors          = (RxStats_pst->RxStatsCrcAlignErrors + Registers_pst->RX_ALIGNMENT_ERROR_PACKETS);
    RxStats_pst->RxStatsUndersizePkts           = Registers_pst->RX_UNDERSIZE_PACKETS_GOOD;     /*Number of good frames received with length less
                                                                                            than 64 bytes*/
    RxStats_pst->RxStatsOversizePkts            = Registers_pst->RX_OVERSIZE_PACKETS_GOOD;      /*Number of good frames received with length more
                                                                                            than 1518 bytes*/
    RxStats_pst->RxStatsFragments               = Registers_pst->RX_RUNT_ERROR_PACKETS;         /*Number of frames received with length less than 64
                                                                                            bytes and with CRC error*/
    RxStats_pst->RxStatsJabbers                 = Registers_pst->RX_JABBER_ERROR_PACKETS;       /*Number of frames received with length more than
                                                                                            1518 bytes and with CRC error*/
    RxStats_pst->RxStatsCollisions              = RBA_ETH_GETRXSTATS_INVALID_VALUE;           /*No register available for total number of
                                                                                            collisions*/

    /*Total number of good and bad frames received with length 64 bytes*/
    RxStats_pst->RxStatsPkts64Octets            = Registers_pst->RX_64OCTETS_PACKETS_GOOD_BAD;

    /*Total number of good and bad frames received with length 64 to 127 bytes*/
    RxStats_pst->RxStatsPkts65to127Octets       = Registers_pst->RX_65TO127OCTETS_PACKETS_GOOD_BAD;

    /*Total number of good and bad frames received with length 128 to 255 bytes*/
    RxStats_pst->RxStatsPkts128to255Octets      = Registers_pst->RX_128TO255OCTETS_PACKETS_GOOD_BAD;

    /*Total number of good and bad frames received with length 256 to 511 bytes*/
    RxStats_pst->RxStatsPkts256to511Octets      = Registers_pst->RX_256TO511OCTETS_PACKETS_GOOD_BAD;

    /*Total number of good and bad frames received with length 512 to 1023 bytes*/
    RxStats_pst->RxStatsPkts512to1023Octets     = Registers_pst->RX_512TO1023OCTETS_PACKETS_GOOD_BAD;

    /*Total number of good and bad frames received with length 1024 to max bytes*/
    RxStats_pst->RxStatsPkts1024to1518Octets    = Registers_pst->RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD;
    /*Total number of Unicast packets delivered to higher layer protocol */
    RxStats_pst->RxUnicastFrames                = Registers_pst->RX_UNICAST_PACKETS_GOOD;

    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

    return ReturnVal_u8;
}
#endif
#endif

#if defined(ETH_GET_TX_STATS_API)
#if (ETH_GET_TX_STATS_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Reads the list of Tx statistcs counter values of the indexed controller.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param TxStats_pst    contains List of values to read statistic values for transmission.
 *
 * \return   Std_ReturnType    E_OK: success
 *                             E_NOT_OK: Statistics counter could not be obtained
 *
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_GetTxStats(uint8 CtrlIdx_u8,
                                     Eth_TxStatsType *TxStats_pst)
{
    /* local pointer to register map structure of the Ethernet MAC (controller) */
    rba_Eth_RegisterMapRefType_t    Registers_pst;

    /* Return value informing the status of GetTxStats*/
    Std_ReturnType                  ReturnVal_u8;

    ReturnVal_u8 = E_OK;
    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* critical section because registers are read*/
    SchM_Enter_Eth(CONTROLLER);
    /* Total number of octets transmitted out of the interface */
    /*if register TX_OCTET_COUNT_GOOD_BAD reeached its maximum limit then TxNumberOfOctets updated with
     * (0xFFFFFFFFFFFFFFFFUL) value becasue of uint64 data type */
    if(Registers_pst->TX_OCTET_COUNT_GOOD_BAD == RBA_ETH_GETTXSTATS_INVALID_VALUE)
    {
        TxStats_pst->TxNumberOfOctets = RBA_ETH_GETTXRXSTATS_U64_INVALID_VALUE;
    }
    else
    {
        TxStats_pst->TxNumberOfOctets = (uint64)Registers_pst->TX_OCTET_COUNT_GOOD_BAD;
    }
    /* Total number of packets requested to transmit to a non-unicast address */
    TxStats_pst->TxNUcastPkts           = Registers_pst->TX_MULTICAST_PACKETS_GOOD_BAD;
    TxStats_pst->TxNUcastPkts           = (TxStats_pst->TxNUcastPkts + Registers_pst->TX_BROADCAST_PACKETS_GOOD_BAD);
    /* Total number of packets requested to transmit to a unicast address */
    TxStats_pst->TxUniCastPkts          = Registers_pst->TX_UNICAST_PACKETS_GOOD_BAD;
    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

    return ReturnVal_u8;
}

#endif
#endif

#if defined(ETH_GET_TX_ERROR_COUNTER_API)
#if (ETH_GET_TX_ERROR_COUNTER_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Returns the list of Transmission Error Counters from the indexed controller.
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param etherStats_pu32   contains List of values to read statistic error counter values for transmission.
 *
 * \return   Std_ReturnType    E_OK: success
 *                             E_NOT_OK: Statistics counter could not be obtained
 *
 ***************************************************************************************************
 */
Std_ReturnType rba_Eth_GetTxErrorCounterValues(uint8 CtrlIdx_u8,
                                    Eth_TxErrorCounterValuesType *TxErrorCounterValues_pst)
{
    /* local pointer to register map structure of the Ethernet MAC (controller) */
    rba_Eth_RegisterMapRefType_t    Registers_pst;

    /* Return value informing the status of TxErrorCounterValues*/
    Std_ReturnType                  ReturnVal_u8;

    ReturnVal_u8 = E_OK;
    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    /* critical section because registers are read*/
    SchM_Enter_Eth(CONTROLLER);
    TxErrorCounterValues_pst->TxDroppedNoErrorPkts = RBA_ETH_GETTXERRORCOUNTERVALUE_INVALID_VALUE;      /* No register available for dropped outbound packets withut error*/
    TxErrorCounterValues_pst->TxDroppedErrorPkts   = RBA_ETH_GETTXERRORCOUNTERVALUE_INVALID_VALUE;      /* No register available for not transmitted frame due to error*/
    TxErrorCounterValues_pst->TxDeferredTrans      = Registers_pst->TX_DEFERRED_PACKETS;                /* Total Number of deffered Packets*/
    TxErrorCounterValues_pst->TxSingleCollision    = Registers_pst->TX_SINGLE_COLLISION_GOOD_PACKETS;   /* Total Number of Single Collision Packets*/
    TxErrorCounterValues_pst->TxMultipleCollision  = Registers_pst->TX_MULTIPLE_COLLISION_GOOD_PACKETS; /* Total Number of Multiple Collision Packets*/
    TxErrorCounterValues_pst->TxLateCollision      = Registers_pst->TX_LATE_COLLISION_PACKETS;          /* Total Number of Late Collision Packets*/
    TxErrorCounterValues_pst->TxExcessiveCollision  = Registers_pst->TX_EXCESSIVE_COLLISION_PACKETS;    /* Total Number of Excessive Collision Packets*/
    /* Exit the critical section */
    SchM_Exit_Eth(CONTROLLER);

    return ReturnVal_u8;
}
#endif
#endif

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Updates counter values available in controller to the provided pointer
 * Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param   CtrlIdx_u8                      Index of the controller
 *
 * Parameter Out:
 * \param   Eth_DemErrorCountHandle_pst     Pointer to the structure where the counter values need to be updated
 *
 * \return  None
 *
 ***************************************************************************************************
 */
void rba_Eth_GetDemErrorCounters(uint8 CtrlIdx_u8, Eth_DemErrorCountHandleRefType_t Eth_DemErrorCountHandle_pst)
{
    rba_Eth_RegisterMapRefType_t                Registers_pst;

    Registers_pst                               = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    SchM_Enter_Eth(CONTROLLER);

    Eth_DemErrorCountHandle_pst->Eth_RxPacketsLost_u32          =  0;
    Eth_DemErrorCountHandle_pst->Eth_CrcErrors_u32              =  Registers_pst->RX_CRC_ERROR_PACKETS;
    Eth_DemErrorCountHandle_pst->Eth_AlignmentError_u32         =  Registers_pst->RX_ALIGNMENT_ERROR_PACKETS;
    Eth_DemErrorCountHandle_pst->Eth_OverSizeFrames_u32         =  Registers_pst->RX_OVERSIZE_PACKETS_GOOD;;
    Eth_DemErrorCountHandle_pst->Eth_UnderSizeFrames_u32        =  Registers_pst->RX_UNDERSIZE_PACKETS_GOOD;
    Eth_DemErrorCountHandle_pst->Eth_SingleCollisionError_u32   =  Registers_pst->TX_SINGLE_COLLISION_GOOD_PACKETS;
    Eth_DemErrorCountHandle_pst->Eth_MultipleCollisionError_u32 =  Registers_pst->TX_MULTIPLE_COLLISION_GOOD_PACKETS;
    Eth_DemErrorCountHandle_pst->Eth_LateCollisionError_u32     =  Registers_pst->TX_LATE_COLLISION_PACKETS;

    SchM_Exit_Eth(CONTROLLER);

}
#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * checks the Ready bit in control and status register of the TX DMA descriptor to find out
 * whether tx was done by the hardware
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8         Index of the controller within the context of the Ethernet Driver
 * \param DescriptorIndex_u8 Index of the transmit DMA descriptor
 *
 * \return                   boolean {TRUE: Frame transmitted; FALSE: Frame in not transmitted}
 *
 ***************************************************************************************************
 */
boolean rba_Eth_TxDone(uint8 CtrlIdx_u8,
                       uint8 DescriptorIndex_u8)
{
    Eth_TransmitBufferDescriptorQueueRefType_t      TransmitDescriptorQueue_pst;    /* local pointer to management structure for TX descriptor buffer elements */
    rba_Eth_TxBufferDescriptorRefType_t             CurrentDesc_pst;                /* local pointer to a the current processed descriptor in the descriptor buffer */
    boolean                                         TxDone_b;                       /* local variable holding the status of tx completion*/

    TxDone_b = FALSE;

    TransmitDescriptorQueue_pst = &(Eth_Controllers_ast[CtrlIdx_u8].TransmitDescriptorQueue_st);
    CurrentDesc_pst = &(TransmitDescriptorQueue_pst->First_pst[DescriptorIndex_u8]);

    /* Is frame transmitted? */
    if((CurrentDesc_pst->Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMATXDESC3_OWN) == 0UL)
    {   /* Yes */
        TxDone_b = TRUE;
    }
    else
    {   /* No */
        TxDone_b = FALSE;
    }

    return(TxDone_b);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * This function will program Tx descriptor to initiate transmission of a frame
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8            Index of the controller within the context of the Ethernet Driver
 * \param DescriptorIndex_u8    Index of the transmit DMA descriptor
 * \param LenByte_u16           Data length in byte
 * \param FramePtr_pu8          address of the buffer to be associated with the Tx descriptor
 *
 *
 * return                None
 *
 ***************************************************************************************************
 */
void rba_Eth_ProgramTxDescriptor(uint8 CtrlIdx_u8,
                                 uint8 DescriptorIndex_u8,
                                 uint16 LenByte_u16,
                                 const uint8 *FramePtr_pu8)
{

    Eth_ControllerRefType_t                         Controller_pst;                 /* local pointer to global management structure per Eth controller */
    rba_Eth_RegisterMapRefType_t                    Registers_pst;                  /* local pointer to register map structure of the Ethernet MAC (controller) */
    Eth_TransmitBufferDescriptorQueueRefType_t      TransmitDescriptorQueue_pst;    /* local pointer to management structure for TX descriptor buffer elements */
    rba_Eth_TxBufferDescriptorRefType_t             CurrentDesc_pst;                /* local pointer to a the current processed descriptor in the descriptor buffer */

    Controller_pst = &Eth_Controllers_ast[CtrlIdx_u8];
    Registers_pst = Controller_pst->Registers_pst;

    TransmitDescriptorQueue_pst = &(Controller_pst->TransmitDescriptorQueue_st);
    CurrentDesc_pst = &(TransmitDescriptorQueue_pst->First_pst[DescriptorIndex_u8]);

    /* MR12 RULE 11.4,11.6 VIOLATION: FramePtr_pu8 contains the address location of Tx buffer typecast done to store in Descriptor register.*/
    /* Copy buffer address into TX DMA descriptor */
    CurrentDesc_pst->Des0_DataBuff1Addr_TimeStampLow_u32 = ((uint32)FramePtr_pu8);

    /* Data buffer 2 is not used */
    CurrentDesc_pst->Des1_DataBuff2Addr_TimeStampHigh_u32 = 0;

    /* Copy buffer length into TX DMA descriptor */
    CurrentDesc_pst->Des2_Ctrl_u32 |= (((uint32)LenByte_u16) & RBA_ETH_MAC_DMATXDESC2_B1L_MASK);

    /*Enable interrupt on completion if requested*/
    if (Eth_CtrlEnableTxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
    {
        CurrentDesc_pst->Des2_Ctrl_u32 |= RBA_ETH_MAC_DMATXDESC2_IOC;
    }

    /* Set the buffer descriptor OWN bit -> DMA shall process */
    CurrentDesc_pst->Des3_CtrlStatus_u32 = (RBA_ETH_MAC_DMATXDESC3_OWN | RBA_ETH_MAC_DMATXDESC3_FD | RBA_ETH_MAC_DMATXDESC3_LD | RBA_ETH_MAC_DMATXDESC3_SAIC| RBA_ETH_MAC_DMATXDESC3_CIC);

    /* Inform GMAC that transmit descriptor ring is updated. */
    if(DescriptorIndex_u8 < (TransmitDescriptorQueue_pst->LastIndex_u8) )
    {
        Registers_pst->DMA_CH0_TXDESC_TAIL_POINTER += RBA_ETH_TXDESC_SIZE;
    }
    else
    {
        /* MR12 RULE 11.4,11.6 VIOLATION: FramePtr_pu8 contains the address location of Tx buffer typecast done to store in Descriptor register.*/
        Registers_pst->DMA_CH0_TXDESC_TAIL_POINTER = (uint32)Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlTxDescriptors_pu32;
    }


    return;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * This function will advance the receive tail pointer register to the next descriptor
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param ReceiveDescriptorQueue_pst    pointer to the Rx global management structure
 * \param Registers_pst                 pointer to the global register structure
 *
 * return                None
 *
 ***************************************************************************************************
 */

/* MR12 RULE 8.13 VIOLATION: The pointer is constant*/
static void rba_Eth_AdvanceRxTailPointer(const Eth_ReceiveBufferDescriptorQueueRefType_t ReceiveDescriptorQueue_pst,
                                                     rba_Eth_RegisterMapRefType_t    Registers_pst)
{

    /* MR12 RULE 11.4,11.6  VIOLATION: cast from pointer to number. Need to compare a current desc address and tail pointer */
    if ( (Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER == (uint32)(&ReceiveDescriptorQueue_pst->First_pst[(ReceiveDescriptorQueue_pst->LastIndex_u16 + 1)])) ) /* last buffer in queue? */
    { /* yes */
        /* wrap round to start of queue */
        /* MR12 RULE 11.4,11.6  VIOLATION: cast from pointer to number. Required for providing base desc address */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER = (uint32)(&ReceiveDescriptorQueue_pst->First_pst[2]);
#else
        Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER = (uint32)(&ReceiveDescriptorQueue_pst->First_pst[1]);
#endif
    }
    else
    { /* no */
        /* advance to next buffer in queue */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)/* shd7kor : condition need to be reverted, as of now manaullay timestamp is enabled */
        Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER += RBA_ETH_RXDESC_ADVANCED_SIZE;
#else
        Registers_pst->DMA_CH0_RXDESC_TAIL_POINTER += RBA_ETH_RXDESC_NORMAL_SIZE;
#endif
    }
}

/**
 ***************************************************************************************************
 * \moduledescription
 * This function will writeback the Rx descriptor to initial state for the transmission of a next frame
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CurrentDesc_pst           pointer to the current Rx descriptor
 * \param DataBuffferAddress_u32    Data buffer address which needs to be updated in the rx descriptor
 * \param DescType_u8               Rx Descriptor type
 *                                  0 - Normal descriptor
 *                                  1 - Context descriptor
 *
 * return                None
 *
 ***************************************************************************************************
 */
static void rba_Eth_UpdateWriteBackDesc( uint8                               CtrlIdx_u8,
                                         rba_Eth_RxBufferDescriptorRefType_t CurrentDesc_pst,
                                         uint32                              DataBuffferAddress_u32,
                                         uint8                               DescType_u8 )
{
    uint32  rxDescrOptions = 0;

    CurrentDesc_pst->Des0_DataBuff1Addr_VlanTag_u32 = 0U;
    CurrentDesc_pst->Des1_Reserved_Status_u32 = 0U;
    CurrentDesc_pst->Des2_DataBuff2Addr_Status__u32 = DataBuffferAddress_u32;

    /*Enable interrupt on completion if requested*/
    if (Eth_CtrlEnableRxInterrupt_acu8_MP[CtrlIdx_u8] == STD_ON)
    {
        rxDescrOptions = RBA_ETH_MAC_DMARXDESC3_IOC;
    }

    if(NORMAL_DESCRIPTOR_TYPE == DescType_u8)
    {
        /* Set the OWN bit and Buffer2 address valid bit */
        rxDescrOptions |= ( RBA_ETH_MAC_DMARXDESC3_OWN | RBA_ETH_MAC_DMARXDESC3_BUF2V );
    }
    else
    {
        /* Set the OWN bit */
        rxDescrOptions |= ( RBA_ETH_MAC_DMARXDESC3_OWN );
    }
    CurrentDesc_pst->Des3_CtrlStatus_u32            = rxDescrOptions;

}

#if defined (RBA_ETH_EN_RX_POLLING)
#if (RBA_ETH_EN_RX_POLLING == STD_ON)

/**
 ***************************************************************************************************
 * \moduledescription
 * Triggers frame reception
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 \param RxStatusPtr     Indicates whether a frame has been received and if so, whether more frame
 *                      are available.
 *
 * \return              None
 *
 ***************************************************************************************************
 */
void rba_Eth_Receive(uint8 CtrlIdx_u8, Eth_RxStatusType* RxStatusPtr)
{
    Eth_ControllerRefType_t                         Controller_pst;             /* local pointer to global management structure per Eth controller */
    rba_Eth_RegisterMapRefType_t                    Registers_pst;              /* local pointer to register map structure of the Ethernet MAC (controller) */
    Eth_ReceiveBufferDescriptorQueueRefType_t       ReceiveDescriptorQueue_pst; /* local pointer to management structure for RX descriptor buffer elements */
    rba_Eth_RxBufferDescriptorRefType_t             CurrentDesc_pst;            /* local pointer to a the current processed descriptor in the descriptor buffer */
    uint32                                          Des3_CtrlStatus_u32;        /* Descriptor Control and status register */
    Eth_DataType                                    *DataPtr_pu8;               /* local pointer to the received ethernet frame */
    uint16                                          CurrentIndex_u16;
    uint8*                                          ReceiveBuffer_pu8;          /* pointer for start address of receive buffer */
    uint32                                          DataBuffferAddress_u32;
    /* Get poointer global management structure */
    Controller_pst = &Eth_Controllers_ast[CtrlIdx_u8];
    /* Get Register base address and Receive queue register */
    Registers_pst = Controller_pst->Registers_pst;
    ReceiveDescriptorQueue_pst = &Controller_pst->ReceiveDescriptorQueue_st;

    SchM_Enter_Eth(CONTROLLER);
    /* Get the current descriptor address*/
    CurrentIndex_u16 = ReceiveDescriptorQueue_pst->CurrentIndex_u16;
    CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
    SchM_Exit_Eth(CONTROLLER);

    *RxStatusPtr = ETH_NOT_RECEIVED;

    ReceiveBuffer_pu8 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBuffers_pu8;
    /* tmp storage of descriptor status, faster than the descr mem */
    Des3_CtrlStatus_u32 = CurrentDesc_pst->Des3_CtrlStatus_u32;

    /* Frame received and buffer own by CPU */
    if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_OWN) == 0UL)
    {
        /* Frame received fits in one buffer with out error? */
        if ( ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_FD) != 0UL) &&  /*start */
             ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_LD) != 0UL) &&  /*end*/
             ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_ES) == 0UL))    /*no err*/
        { /* Yes good frame*/

            /* MR12 RULE 11.4,11.6  VIOLATION: cast from number to pointer. This comes from a HW Reg actually holding a ptr */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
            /* current index will be double of the Normal descriptor due to context based descriptor so DataPtr is updating as per normal descriptor (not considering context based index)*/
            DataPtr_pu8 = (uint8*)(((CurrentIndex_u16 >> 1) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16) + ReceiveBuffer_pu8 );
#else
            DataPtr_pu8 = (uint8*)(((CurrentIndex_u16) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16) + ReceiveBuffer_pu8 );
#endif
            /* Call EthIfRxIndicaton function */
            rba_Eth_IndicateRxFrame(CtrlIdx_u8, Des3_CtrlStatus_u32, DataPtr_pu8);

            /* One Ethernet Frame is received */
            *RxStatusPtr = ETH_RECEIVED;
        }
        else
        {
            while(((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_LD) == 0UL) &&
                  ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_OWN) == 0UL))
            {
                /* Update  Writeback format to read back format for the normal descriptor */
                /* Set Buffer Address */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
                /* current index will be double of the Normal descriptor due to context based descriptor so DataBuffferAddress_u32 is updating as per normal descriptor (not considering context based index)*/
                /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
                DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16 >> 1) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#else
                DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#endif
                rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8,CurrentDesc_pst, DataBuffferAddress_u32, NORMAL_DESCRIPTOR_TYPE);
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
                SchM_Enter_Eth(CONTROLLER);
                Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);
                CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
                SchM_Exit_Eth(CONTROLLER);
                rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8,CurrentDesc_pst, DATA_BUFFER_ADDRESS_FOR_CONTEXT_DESCRIPTOR, CONTEXT_DESCRIPTOR_TYPE);
#endif

                SchM_Enter_Eth(CONTROLLER);
                /* Inform MAC that Receive descriptor ring is updated */
                rba_Eth_AdvanceRxTailPointer(ReceiveDescriptorQueue_pst,Registers_pst);
                /* Advance queue index CurrentIndex_u16 */
                Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);
                CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];

                SchM_Exit_Eth(CONTROLLER);

                /* Read the status of descriptor */
                Des3_CtrlStatus_u32 = CurrentDesc_pst->Des3_CtrlStatus_u32;
            }
            if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_ES) != 0UL)
            {
                /* Frame received with errors */
                /* Report DET error as Frame Lost */
                if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_CE) != 0UL)
                {
                    /* CRC error frame received */
                    /* AR422 : Raise DEM Error for CRC errors if configured */
                }
                /* AR422 : Raise DEM error for FRAMES LOST if dem configured */
            }
            else
            {
                /* Frame does not fit in one buffer */
                /* Report DET error as Invalid Configuration */
                /* AR422 : Raise DEM Error for Oversize frame if dem configured */
                /*MR12 RULE 14.3 VIOLATION: if condition is constantly TRUE as DET has to be reported for multibuffer frame */
                ETH_DET_REPORT_NORETURN(TRUE, ETH_SID_ETH_RECEIVE, ETH_E_INV_CONFIG);
            }

        }
        /* Update  Writeback format to read back format for the normal descriptor */
        /* Set Buffer Address */
        /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16 >> 1) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#else
        DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#endif
        rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8, CurrentDesc_pst, DataBuffferAddress_u32, NORMAL_DESCRIPTOR_TYPE); /* Normal descriptor type 0 */

        /* This is required only if PTP timestamp is enabled. Condition need to be reverted */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        /* Advance the current index to point to the context descriptor*/
        SchM_Enter_Eth(CONTROLLER);
        Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);
        CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
        SchM_Exit_Eth(CONTROLLER);
        /* Update Writeback format to read back format for the context descriptor */
        rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8, CurrentDesc_pst, DATA_BUFFER_ADDRESS_FOR_CONTEXT_DESCRIPTOR, CONTEXT_DESCRIPTOR_TYPE); /* context descriptor type 1 */
#endif

        SchM_Enter_Eth(CONTROLLER);
        /* Inform MAC that Receive descriptor ring is updated */
        rba_Eth_AdvanceRxTailPointer(ReceiveDescriptorQueue_pst,Registers_pst);
        /* Advance queue index CurrentIndex_u16 */
        Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);
        CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
        SchM_Exit_Eth(CONTROLLER);

        /* Read the status of descriptor */
        Des3_CtrlStatus_u32 = CurrentDesc_pst->Des3_CtrlStatus_u32;

        /* Check whether more frames are avilable */
        if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_OWN) == 0UL)
        {
            /* two or more frames found. */
            *RxStatusPtr    = ETH_RECEIVED_MORE_DATA_AVAILABLE;
        }
    }
    return;
}
#endif
#endif

#define RBA_ETH_STOP_SEC_CODE
#include "rba_Eth_MemMap.h"

#define RBA_ETH_START_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"

#if defined (RBA_ETH_EN_RX_INTERRUPT)
#if (RBA_ETH_EN_RX_INTERRUPT == STD_ON)

/**
 ***************************************************************************************************
 * \moduledescription
 * Handle interrupts of the indexed controller
 * Ethernet MAC has no different interrupt line for RX and TX interrupts -> one function
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * \return              None
 *
 ***************************************************************************************************
 */

void rba_Eth_RxIrqHdlr(uint8 CtrlIdx_u8)
{
    rba_Eth_RegisterMapRefType_t    Registers_pst;          /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint32                          IntrEventReg_u32;       /* Interrupt status register */
    uint32                          IntrMaskReg_u32;        /* Interrupt Mask register */

    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    SchM_Enter_Eth(CONTROLLER);
    /*
     *  In case RX/TX interrupt masks are disabled then interrupts should not be handled as the controller
     *  is in mode down or is currently being brought into mode down
     */

    IntrMaskReg_u32 = Registers_pst->DMA_CH0_INTERRUPT_ENABLE;

    /* Take the snapshot interrupt status register*/
    IntrEventReg_u32 = Registers_pst->DMA_CH0_STATUS;

    SchM_Exit_Eth(CONTROLLER);

    /* Is Rx interrupt enabled and pending? */
    if( ((IntrMaskReg_u32 & (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE_POS)) != 0UL) &&
        ((IntrEventReg_u32 & (RBA_ETH_DMA_CH_N_STATUS_RI  << RBA_ETH_DMA_CH_N_STATUS_RI_POS)) != 0UL))
    {   /* Yes */

        /* Clear RX interrupt status */
        /* NIS is a sticky bit and must be cleared each time a corresponding bit,
         * which causes NIS to be set, is cleared */

        SchM_Enter_Eth(CONTROLLER);

        Registers_pst->DMA_CH0_STATUS = (0UL
                                 | (RBA_ETH_DMA_CH_N_STATUS_RI  << RBA_ETH_DMA_CH_N_STATUS_RI_POS)
                                 | (RBA_ETH_DMA_CH_N_STATUS_NIS  << RBA_ETH_DMA_CH_N_STATUS_NIS_POS));

        SchM_Exit_Eth(CONTROLLER);

        /* Process received frames */
        rba_Eth_Receive_Interrupt(CtrlIdx_u8);

    }
    else
    {
        /* There are no pending Rx Interrupts */
    }

}
#endif
#endif


#if defined (RBA_ETH_EN_TX_INTERRUPT)
#if (RBA_ETH_EN_TX_INTERRUPT == STD_ON)

/**
 ***************************************************************************************************
 * \moduledescription
 * Handle interrupts of the indexed controller
 * Ethernet MAC has no different interrupt line for RX and TX interrupts -> one function
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * \return              None
 *
 ***************************************************************************************************
 */

void rba_Eth_TxIrqHdlr(uint8 CtrlIdx_u8)
{
    rba_Eth_RegisterMapRefType_t    Registers_pst;          /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint32                          IntrEventReg_u32;       /* Interrupt status register */
    uint32                          IntrMaskReg_u32;        /* Interrupt Mask register */

    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    SchM_Enter_Eth(CONTROLLER);

    /*
     *  In case RX/TX interrupt masks are disabled then interrupts should not be handled as the controller
     *  is in mode down or is currently being brought into mode down
     */

    IntrMaskReg_u32 = Registers_pst->DMA_CH0_INTERRUPT_ENABLE;

    /* Take the snapshot interrupt status register*/
    IntrEventReg_u32 = Registers_pst->DMA_CH0_STATUS;

    SchM_Exit_Eth(CONTROLLER);

    /* Is TX interrupt enabled and pending? */
    if( ((IntrMaskReg_u32 & (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE_POS)) != 0UL)&&
        ((IntrEventReg_u32 & (RBA_ETH_DMA_CH_N_STATUS_TI  << RBA_ETH_DMA_CH_N_STATUS_TI_POS)) != 0UL))
    {   /* Yes */

        /* Clear TX interrupt status */
        /* NIS is a sticky bit and must be cleared each time a corresponding bit,
         * which causes NIS to be set, is cleared */
        SchM_Enter_Eth(CONTROLLER);

        Registers_pst->DMA_CH0_STATUS = (0UL
                                  | (RBA_ETH_DMA_CH_N_STATUS_TI  << RBA_ETH_DMA_CH_N_STATUS_TI_POS)
                                  | (RBA_ETH_DMA_CH_N_STATUS_NIS  << RBA_ETH_DMA_CH_N_STATUS_NIS_POS));

        SchM_Exit_Eth(CONTROLLER);

        /* Provide TX confirmation */
        Eth_TxConfirmation(CtrlIdx_u8);
    }
    else
    {
        /* There are no pending Tx Interrupts */
    }

}

#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * Handle interrupts of the indexed controller
 * Ethernet MAC has no different interrupt line for RX and TX interrupts -> one function
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * \return              None
 *
 ***************************************************************************************************
 */
void rba_Eth_CtrlErrIrqHdlr(uint8 CtrlIdx_u8)
{
    rba_Eth_RegisterMapRefType_t    Registers_pst;          /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint32                          IntrEventReg_u32;       /* Interrupt status register */
    uint32                          IntrMaskReg_u32;        /* Interrupt Mask register */

    Registers_pst = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;

    SchM_Enter_Eth(CONTROLLER);
    /*
     *  In case RX/TX interrupt masks are disabled then interrupts should not be handled as the controller
     *  is in mode down or is currently being brought into mode down
     */

    IntrMaskReg_u32 = Registers_pst->DMA_CH0_INTERRUPT_ENABLE;

    /* Take the snapshot interrupt status register*/
    IntrEventReg_u32 = Registers_pst->DMA_CH0_STATUS;

    SchM_Exit_Eth(CONTROLLER);

    /*Has Ethernet bus error occurred? */
    if( ((IntrMaskReg_u32 &  (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE_POS)) != 0UL) &&
        ((IntrEventReg_u32 & (RBA_ETH_DMA_CH_N_STATUS_FBE  << RBA_ETH_DMA_CH_N_STATUS_FBE_POS))  != 0UL))
    {    /* Yes */

        SchM_Enter_Eth(CONTROLLER);
        /* Clear interrupt status */
        Registers_pst->DMA_CH0_STATUS = (0UL
                                | (RBA_ETH_DMA_CH_N_STATUS_FBE  << RBA_ETH_DMA_CH_N_STATUS_FBE_POS)
                                | (RBA_ETH_DMA_CH_N_STATUS_AIS  << RBA_ETH_DMA_CH_N_STATUS_AIS_POS));

        SchM_Exit_Eth(CONTROLLER);

        /* Report DEM error if ETH_DEM_REPORTING_SUPPORT is referenced from DEM */
#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
        Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEAccess_u16, DEM_EVENT_STATUS_FAILED);
#else
        Eth_CurrentConfig_pco->EthDemEvents_st.EthEAccessErrorCallBack(CtrlIdx_u8,E_NOT_OK);
#endif
#endif
    }
    else
    {
         /* Ethernet Bus error not occurred */
    }

    return;
}


#if defined (RBA_ETH_EN_RX_INTERRUPT)
#if (RBA_ETH_EN_RX_INTERRUPT == STD_ON)

/**
 ***************************************************************************************************
 * \moduledescription
 * \Triggers frame reception. This is called by the Eth_RxIrq handler. This will process all the
 *  received frames during interrupt and for each frame "EthIf_RxIndication" is given.
 *
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 * \par Non Reentrant.
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:       None
 *
 *
 * \return              None
 *
 ***************************************************************************************************
 */
static void rba_Eth_Receive_Interrupt(uint8 CtrlIdx_u8 )
{
    Eth_ControllerRefType_t                         Controller_pst;             /* local pointer to global management structure per Eth controller */
    rba_Eth_RegisterMapRefType_t                    Registers_pst;              /* local pointer to register map structure of the Ethernet MAC (controller) */
    Eth_ReceiveBufferDescriptorQueueRefType_t       ReceiveDescriptorQueue_pst; /* local pointer to management structure for RX descriptor buffer elements */
    rba_Eth_RxBufferDescriptorRefType_t             CurrentDesc_pst;            /* local pointer to a the current processed descriptor in the descriptor buffer */
    uint32                                          Des3_CtrlStatus_u32;        /* Descriptor Control and status register */
    Eth_DataType                                    *DataPtr_pu8;               /* local pointer to the received ethernet frame */
    uint8*                                          ReceiveBuffer_pu8;          /* pointer for start address of receive buffer */
    uint16                                          ReceiveBufferLength_u16;
    uint32                                          DataBuffferAddress_u32;

    /* Get poointer global management structure */
    Controller_pst = &Eth_Controllers_ast[CtrlIdx_u8];
    /* Get Register base address and Receive queue register */
    Registers_pst = Controller_pst->Registers_pst;
    ReceiveDescriptorQueue_pst = &Controller_pst->ReceiveDescriptorQueue_st;

    SchM_Enter_Eth(CONTROLLER);
    /* Get the current descriptor address*/
    CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
    SchM_Exit_Eth(CONTROLLER);

    ReceiveBuffer_pu8 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBuffers_pu8;
    ReceiveBufferLength_u16 = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16;
    /* tmp storage of descriptor status, faster than the descr mem */
    Des3_CtrlStatus_u32 = CurrentDesc_pst->Des3_CtrlStatus_u32;

    /* Frame received and buffer own by CPU */
    while((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_OWN) == 0UL)
    {
        /* Frame received fits in one buffer with out error? */
        if ( ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_FD) != 0UL) &&  /*start */
             ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_LD) != 0UL) &&  /*end*/
             ((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_ES) == 0UL))    /*no err*/
        { /* Yes good frame*/
	        #if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
                DataPtr_pu8 = (uint8*)(ReceiveBuffer_pu8 + (((ReceiveDescriptorQueue_pst->CurrentIndex_u16) >> 1) * ReceiveBufferLength_u16));
			#else
                DataPtr_pu8 = (uint8*)(ReceiveBuffer_pu8 + ((ReceiveDescriptorQueue_pst->CurrentIndex_u16) * ReceiveBufferLength_u16));
			#endif

            /* Call EthIfRxIndicaton function */
            rba_Eth_IndicateRxFrame(CtrlIdx_u8, Des3_CtrlStatus_u32, DataPtr_pu8);
        }
        else
        {
            if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_ES) != 0UL)
            {
                /* Frame received with errors */
                /* Report DET error as Frame Lost */
                if((Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_CE) != 0UL)
                {
                    /* CRC error frame received */
                    /* AR422 : Raise DEM Error for CRC errors if configured */
                }
                /* AR422 : Raise DEM error for FRAMES LOST if dem configured */
            }
            else
            {
                /* Frame does not fit in one buffer */
                /* Report DET error as Invalid Configuration */
                /* AR422 : Raise DEM Error for Oversize frame if dem configured */
                /*MR12 RULE 14.3 VIOLATION: if condition is constantly TRUE as DET has to be reported for multibuffer frame */
                ETH_DET_REPORT_NORETURN(TRUE, ETH_SID_ETH_RECEIVE, ETH_E_INV_CONFIG);
            }
        }

        /* Update  Writeback format to read back format for the normal descriptor */
        /* Set Buffer Address */
        /* MR12 RULE 11.4,11.6 VIOLATION: cast from number to pointer. This is a HW Reg. */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16 >> 1) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#else
        DataBuffferAddress_u32= (uint32) &ReceiveBuffer_pu8[(ReceiveDescriptorQueue_pst->CurrentIndex_u16) * Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlRxBufLenByte_u16];
#endif
        rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8, CurrentDesc_pst, DataBuffferAddress_u32, NORMAL_DESCRIPTOR_TYPE);

        /* This is required only if PTP timestamp is enabled. Condition need to be reverted */
#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
        /* Advance the current index to point to the context descriptor*/
        SchM_Enter_Eth(CONTROLLER);
        Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);
        CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
        SchM_Exit_Eth(CONTROLLER);
        /* Update Writeback format to read back format for the context descriptor */
        rba_Eth_UpdateWriteBackDesc(CtrlIdx_u8, CurrentDesc_pst, DATA_BUFFER_ADDRESS_FOR_CONTEXT_DESCRIPTOR, CONTEXT_DESCRIPTOR_TYPE);
#endif

        SchM_Enter_Eth(CONTROLLER);
        /* Inform MAC that Receive descriptor ring is updated */
        rba_Eth_AdvanceRxTailPointer(ReceiveDescriptorQueue_pst,Registers_pst);
        /* Advance queue index CurrentIndex_u16 */
        Eth_RxBuffer_AdvanceQueueIndex(&(ReceiveDescriptorQueue_pst->CurrentIndex_u16), ReceiveDescriptorQueue_pst->LastIndex_u16);

        CurrentDesc_pst = &ReceiveDescriptorQueue_pst->First_pst[ReceiveDescriptorQueue_pst->CurrentIndex_u16];
        SchM_Exit_Eth(CONTROLLER);

        /* Read the status of descriptor */
        Des3_CtrlStatus_u32 = CurrentDesc_pst->Des3_CtrlStatus_u32;
    }
}
#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * The function passes the received frame to the Ethernet interface using the callback
 * function EthIf_RxIndication.
 *
 * \par Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 * \par Non Reentrant.
 *
 * Parameter In:
 * \param CtrlIdx_u8            Index of the controller within the context of the Ethernet Driver
 * \param Des0_CtrlStatus_u32   Descriptor Control and status Information
 * \param *DataPtr_pu8          Pointer to the received Ethernet frame
 *
 * Parameter Out:               none
 *
 *
 * \return              None
 *
 ***************************************************************************************************
 */
static void rba_Eth_IndicateRxFrame(uint8 CtrlIdx_u8, uint32 Des3_CtrlStatus_u32, Eth_DataType *DataPtr_pu8)
{
    uint32                                          FrameIndex_u32;             /* index for checking whether the received frame is broadcast */
    uint8*                                          PhysAddrPtr_u32;            /* local pointer to the src mac address of the received frame*/
    Eth_DataType                                    *EthPayloadAddPtr_u32;       /* local pointer to point to the payload of the received Eth Frame */
    uint16                                          FrameLength_u16;            /* local variable holding the frame length for which indication is to be provided*/
    Eth_FrameType                                   FrameType_uo;               /* Ethernet frame type of the received frame */
    boolean                                         IsBroadcast_b;              /* local boolean variable which indicates whether the rx frame is broadcast or uin\multicast*/


    /* Initialising the IsBroadcast_b to 1; This will be updated after checking whether the frame is broadcast */
	IsBroadcast_b = TRUE;

    /* Extract frame length from Descriptor register*/
    FrameLength_u16 = (uint16)(Des3_CtrlStatus_u32 & RBA_ETH_MAC_DMARXDESC3_FL_MASK);
    /* 14 bytes is subtracted from the frame length which contains source addr, destination addr and Type. The length field does not contain the FCS length since it is stripped by MAC for all frames */
    FrameLength_u16 = FrameLength_u16 - ETH_SRC_DST_FTYPE_LEN;

    /* Get the Frame type from the received frame */
    FrameType_uo = (((uint16)(((uint16)DataPtr_pu8[ETH_FRAMETYPE_BYTE1_OFFSET])<< ETH_FRAMETYPE_SHIFT)) |  DataPtr_pu8[ETH_FRAMETYPE_BYTE2_OFFSET]);


    /* Check whether it is a broadcast frame or not */
    for(FrameIndex_u32 = 0; FrameIndex_u32 < ETH_MAC_ADDRESS_LENGTH_BYTE; FrameIndex_u32++)
    {
        if(0xFF != DataPtr_pu8[FrameIndex_u32]) /* Check for address "FF:FF:FF:FF:FF:FF" */
        {
            IsBroadcast_b = FALSE;
            break;
        }
    }

    /* Get the Source Physical address from the received frame */
    /* MR12 RULE 11.4,11.6  VIOLATION: cast from pointer to number. This comes from a HW Reg actually holding a ptr */
    PhysAddrPtr_u32      = DataPtr_pu8 + ETH_SOURCE_MAC_ADDRESS_OFFSET;

    /* Pointer to the payload of the received Ethernet frame */
    EthPayloadAddPtr_u32 = DataPtr_pu8 + ETH_SRC_DST_FTYPE_LEN;

    /* Pass the receive frame to application using EthIf call back function */
    /* MR12 RULE 11.4,11.6 VIOLATION: Rx descriptor start address is passed to EthIf layer to fetch data. Casting is done appropriately*/
    EthIf_RxIndication(CtrlIdx_u8, FrameType_uo, IsBroadcast_b, (uint8*)PhysAddrPtr_u32, EthPayloadAddPtr_u32, FrameLength_u16);
}

#define RBA_ETH_STOP_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"

#define RBA_ETH_START_SEC_CODE
#include "rba_Eth_MemMap.h"

#if defined (RBA_ETH_EN_MII) && defined (RBA_ETH_ASYNCMII_SUPPORT)
#if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT ==STD_ON))
/**
 ***************************************************************************************************
 * \moduledescription
 * Provides Read or Write MII Indication depending on the which was function called.
 * Function is a timer Callback
 * \par Non Reentrant.
 *
 * Parameter In:
 * \param               None
 *
 * \return              None
 *
 ***************************************************************************************************
 */

void rba_Eth_Gpt_Callback( void )
{
    Eth_ControllerRefType_t                         Controller_pst;             /* local pointer to global management structure per Eth controller */
    rba_Eth_ReadWriteMiiManagRefType_t              ReadWriteMiiManagRef_pst;  /* local pointer to global management structure of read/writemii */
    rba_Eth_RegisterMapRefType_t                    Registers_pst;              /* local pointer to register map structure of the Ethernet MAC (controller) */
    uint8                                           CtrlIdx_u8;                 /* local variable for storing controller index */
    uint8                                           TrcvIdx_u8;                 /* local variable to hold the transciever id of the mii operation */
    uint8                                           RegIdx_u8;                  /* local variable to hold the register id of the mii operation */
    uint8                                           MiiBusyBit_u8;              /* local variable to hold the Busy bit value */
    uint16                                          RegVal_u16;                 /* local variable to hold the register value of the mii read operation */
    uint32                                          MiiAddrReg_u32;            /* local variable to hold the Gmii Addr reg register value */
    uint32                                          MiiDataReg_u32;            /* local variable to hold the Gmii Data reg register value */


    /* Get address of read/write mii global management structure */
    ReadWriteMiiManagRef_pst = &rba_Eth_ReadWriteMiiManag_st;
    CtrlIdx_u8 = ReadWriteMiiManagRef_pst->rba_Eth_MiiCtrlIdx_u8;

    Controller_pst = &Eth_Controllers_ast[CtrlIdx_u8];
    Registers_pst = Controller_pst->Registers_pst;

    /* start critical section in order to prevent other tasks from accessing MII*/
    SchM_Enter_Eth(CONTROLLER);

    MiiAddrReg_u32 = Registers_pst->MAC_MDIO_ADDRESS;

    SchM_Exit_Eth(CONTROLLER);

    TrcvIdx_u8 = (uint8)((MiiAddrReg_u32 & RBA_ETH_MAC_MDIO_ADDRESS_PA_MASK) >> RBA_ETH_MAC_MDIO_ADDRESS_PA_POS);

    RegIdx_u8  = (uint8)((MiiAddrReg_u32 & RBA_ETH_MAC_MDIO_ADDRESS_RDA_MASK) >> RBA_ETH_MAC_MDIO_ADDRESS_RDA_POS);
    /* If Busy bit is 0 MII Operation is done */
    MiiBusyBit_u8 = (uint8)((MiiAddrReg_u32 & RBA_ETH_MAC_MDIO_ADDRESS_GB_MASK) >> RBA_ETH_MAC_MDIO_ADDRESS_GB_POS);

    /* Operation type is read from the structure and not from the register, because in case the mdio operation is incomplete
     * GMII Address register will not have the correct contents and hence it'll not be possible to conclude which mdio operation failed */

    switch(ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en)
    {
        case RBA_ETH_WRITEMII:
        {
            if( MiiBusyBit_u8 != RBA_ETH_MAC_MDIO_ADDRESS_GB)
            {
                ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en = RBA_ETH_NONE;
                /* Indicate completion of Mii Write */
                EthTrcv_WriteMiiIndication(CtrlIdx_u8,TrcvIdx_u8,RegIdx_u8);
            }
            else
            {
                ETH_DET_REPORT_NORETURN( TRUE , ETH_SID_ETH_WRITEMII, ETH_E_INV_CONFIG);
            }
        }
        break;

        case RBA_ETH_READMII:
        {
            if( MiiBusyBit_u8 != RBA_ETH_MAC_MDIO_ADDRESS_GB)
            {
                ReadWriteMiiManagRef_pst->rba_Eth_MiiOpType_en = RBA_ETH_NONE;
                /* start critical section in order to prevent other tasks from accessing MII*/
                SchM_Enter_Eth(CONTROLLER);

                MiiDataReg_u32 = Registers_pst->MAC_MDIO_DATA;

                SchM_Exit_Eth(CONTROLLER);

                RegVal_u16 = (uint16)(MiiDataReg_u32 & (RBA_ETH_MAC_MDIO_DATA_GD << RBA_ETH_MAC_MDIO_DATA_GD_POS));
                /* Indicate completion of Mii Read */
                EthTrcv_ReadMiiIndication(CtrlIdx_u8,TrcvIdx_u8,RegIdx_u8,RegVal_u16);
            }
            else
            {
                ETH_DET_REPORT_NORETURN( TRUE , ETH_SID_ETH_READMII, ETH_E_INV_CONFIG);
            }
        }
        break;

        case RBA_ETH_NONE:
        {
            ETH_DET_REPORT_NORETURN( TRUE , ETH_SID_ETH_CONTROLLERINIT, ETH_E_INV_CONFIG);
        }
        break;

        default:
        {
            /* The Default case added as CDG coding guidelines requires default case for every switch. The code will never come to this section */
        }
        break;
    }
}
#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * Assembles the bottom and top address register values from the 6 byte phys address
 * \par Synchronous, Reentrant
 *
 * Parameter In:
 * \param PhysAddrPtr_pcu8    Physical source address (MAC address) in network byte order.
 *
 * Parameter Out:
 * \param PhysAddrBottom_pu32 Bottom 4 bytes of the first 6-byte MAC address assembled into a word.
 * \param PhysAddrTop_pu32    Lower 2 bytes of the first 6-byte MAC address assembled into a word.
 *
 * \return                    None
 *
 ***************************************************************************************************
 */

static void rba_Eth_AssembleBottomAndTopPhysAddress(uint32 *PhysAddrBottom_pu32,
                                                    uint32 *PhysAddrTop_pu32,
                                                    const uint8 *PhysAddrPtr_pcu8)
{
    uint32 ValPhysAddrBottom_u32;
    uint32 ValPhysAddrTop_u32;

    ValPhysAddrBottom_u32 = (((uint32)PhysAddrPtr_pcu8[0])
                            | ((uint32)PhysAddrPtr_pcu8[1] << RBA_ETH_PHYADDR_BYTE2_POS)
                            | ((uint32)PhysAddrPtr_pcu8[2] << RBA_ETH_PHYADDR_BYTE3_POS)
                            | ((uint32)PhysAddrPtr_pcu8[3] << RBA_ETH_PHYADDR_BYTE4_POS));

    ValPhysAddrTop_u32   = (((uint32)PhysAddrPtr_pcu8[4])
                            | ((uint32)PhysAddrPtr_pcu8[5] << RBA_ETH_PHYADDR_BYTE6_POS));

    SchM_Enter_Eth(CONTROLLER);

    *PhysAddrBottom_pu32 = ValPhysAddrBottom_u32;

    *PhysAddrTop_pu32    = ValPhysAddrTop_u32;

    SchM_Exit_Eth(CONTROLLER);
}

#if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * loop through the 1-15 addresses and return the first one that matches the incoming addr.
    IF NONE is matching, return the FIRST unused addr.
    if ALL are used and no addr match then, return 0
 * \par Synchronous, Reentrant
 *
 * Parameter In:
 * \param Registers_pst         Pointer to controller register map
 * \param PhysAddrBottom_pu32   Bottom 4 bytes of the first 6-byte MAC address assembled into a word.
 * \param PhysAddrTop_pu32      Lower 2 bytes of the first 6-byte MAC address assembled into a word.
 *
 * Parameter Out:
 * \param PhysAddrBottom_pu32   Bottom 4 bytes of the first 6-byte MAC address assembled into a word.
 * \param PhysAddrTop_pu32      Lower 2 bytes of the first 6-byte MAC address assembled into a word.
 *
 * \return                    None
 *
 ***************************************************************************************************
 */
/* MR12 RULE 8.13 VIOLATION: The pointer is constant*/
static uint32 rba_Eth_FindFirstUnusedOrMatchingAddr(const rba_Eth_RegisterMapRefType_t  Registers_pst,
                                                            uint32 PhysAddrTop_u32,
                                                            uint32 PhysAddrBottom_u32,
                                                            uint8 searchUnusedFlag_u8)
{
    uint32 index_u32;
    uint32 ret_idx_u32 = 0; /* start pessimistic: no match and all used */

    for (index_u32 = 1; index_u32 < RBA_ETH_MAC_HWMACADDR_GROUP1; index_u32++ )
    {
        /* detect and store the FIRST UNUSED addr */
        if (searchUnusedFlag_u8 == 1)
        {
            if((ret_idx_u32 == 0UL) && /* first? */
              ((Registers_pst->GMA_MAC_ADDR[index_u32].high_u32 & RBA_ETH_MAC_ADDRESS_HIGH_AE_MASK) == 0UL))/* unused */
            {
                ret_idx_u32 = index_u32;
            }
            else
            {
                /* ignore */
            }

        }
        else
        {
            /* even better: if the addr is a match, return immediately */
            if (((Registers_pst->GMA_MAC_ADDR[index_u32].high_u32 & 0xFFFFUL) == PhysAddrTop_u32) &&
                (Registers_pst->GMA_MAC_ADDR[index_u32].low_u32 == PhysAddrBottom_u32))
            {
                ret_idx_u32 = index_u32;
            }
            else
            {
                /* ignore */
            }
        }
        if(ret_idx_u32 != 0UL)
        {
            break; /* break early for performance reasons */
        }
        else
        {
            /* continue */
        }
    }

    return ret_idx_u32;
}

/**
 ***************************************************************************************************
 * \moduledescription
 *  Disable all the MAC addr filters, except the special one (the first)
 * \par Synchronous, Reentrant
 *
 * Parameter In:
 * \param Registers_pst         Pointer to controller register map
 *
* Parameter Out:                None
 *
 * \return                      None
 *
 ***************************************************************************************************
 */

static void rba_Eth_DisableAllMacAddrFilters(rba_Eth_RegisterMapRefType_t Registers_pst)
{
    uint32 index_u32;

    for (index_u32 = 1; index_u32 < RBA_ETH_MAC_HWMACADDR_GROUP1; index_u32 ++)
    {
        Registers_pst->GMA_MAC_ADDR[index_u32].high_u32 = 0;
        Registers_pst->GMA_MAC_ADDR[index_u32].low_u32 = 0;
    }
}
#endif/*(ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)*/

#if (RBA_ETH_GLOBAL_TIME_SUPPORT == STD_ON)

/**
 ***************************************************************************************************
 * \moduledescription
 * rba_Eth_GetCurrentTime:
 * Returns a time value out of the HW registers according to the capability of the HW.
 *
 * \par Synchronous, Non-reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameters Out:
 * \param timeQualPtr       quality of HW time stamp, e.g. based on current drift.
 * \param timeStampPtr      current time stamp
 *
 * \return                  Std_ReturnType {E_OK: success; E_NOT_OK: failed to get the current time}
 *
 ***************************************************************************************************
 */

FUNC(Std_ReturnType, ETH_CODE) rba_Eth_GetCurrentTime( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                   P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                   P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    P2VAR( rba_Eth_RegisterMapType_tst, AUTOMATIC, AUTOMATIC )  Registers_pst; /* local pointer to register map structure of the Ethernet MAC (controller) */
    VAR  ( Std_ReturnType,              AUTOMATIC )             Result_o;

    /* Initilize local variables */
    Registers_pst  = Eth_Controllers_ast[CtrlIdx_u8].Registers_pst;
    Result_o = E_NOT_OK;

    /* Interrupt lock to avoid multiple access while reading the 1588 timer registers */
    SchM_Enter_Eth(CONTROLLER);
     timeStampPtr->nanoseconds = Registers_pst->MAC_SYSTEM_TIME_NANOSECONDS;          /*Reading the 1588 timer nanoseconds register */
     timeStampPtr->seconds     = Registers_pst->MAC_SYSTEM_TIME_SECONDS;              /*Reading the 1588 timer seconds register */
     timeStampPtr->secondsHi   = ((uint16) Registers_pst->MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS);  /*Reading the 1588 timer Higher Word seconds register */
    SchM_Exit_Eth(CONTROLLER);

    /* Checking the 1588 timer register values again for coherancy & if found wrapped the new value is updated */
    if(timeStampPtr->nanoseconds > Registers_pst->MAC_SYSTEM_TIME_NANOSECONDS)
    {
        SchM_Enter_Eth(CONTROLLER);
         timeStampPtr->nanoseconds = Registers_pst->MAC_SYSTEM_TIME_NANOSECONDS;          /*Reading the 1588 timer nanoseconds register */
         timeStampPtr->seconds     = Registers_pst->MAC_SYSTEM_TIME_SECONDS;              /*Reading the 1588 timer seconds register */
         timeStampPtr->secondsHi   =((uint16) Registers_pst->MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS);  /*Reading the 1588 timer Higher Word seconds register */
        SchM_Exit_Eth(CONTROLLER);
    }
    else
    {
        /* do nothing */
    }

    /* Check for time stamping quality */
    /* If the nanoseconds field of timeStampPtr is not within the boundary ( 0 - 9,99,999,999 ns)
     * value pointed via the time stamp quality ptr is set to INVALID */
    if((timeStampPtr->nanoseconds <= RBA_ETH_MAC_TIMESTAMP_NS_UPRLIMIT))
    {
        *timeQualPtr = ETH_VALID;
        Result_o = E_OK;
    }
    else
    {
        *timeQualPtr = ETH_INVALID;
        Result_o = E_NOT_OK;
    }

    return (Result_o);

}


/**
 ***************************************************************************************************
 * \moduledescription
 * rba_Eth_EnableEgressTimeStamp:
 * Activates egress time stamping on a dedicated message object.
 *
 * \par Synchronous, Non-reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param BufIdx_u8         Index of the message buffer, where Application expects egress time stamping
 *
 * \return            None
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) rba_Eth_EnableEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                VAR(uint8, AUTOMATIC) BufIdx_u8 )
{
    /* MR12 RULE 15.6 VIOLATION: Null statement added to make compiler independent */
    PARAM_UNUSED(CtrlIdx_u8);
    /* MR12 RULE 15.6 VIOLATION: Null statement added to make compiler independent */
    PARAM_UNUSED(BufIdx_u8);

    /* The API is invoked for each Buffer Index.
     * In Xilinx, enabling of PTP time stamping is done globally i.e., once enabled it is applicable for every PTP frame.
     * So, there are multiple calls possible as the API is invoked for every Eth_ProvideTxBuffer
     * However, in other controllers, the enabling of time stamping is per descriptor.
     * Hence BufIdx is used to enable timestamping for each buffer.
     * Therefore the API is not supported in Xilinx and kept empty so as to avoid the changes in application specific code for PTP.
     * As of now enabling of timestamping is done in SetControllerMode API. */

    return;
}
/**
 ***************************************************************************************************
 * \moduledescription
 * rba_Eth_GetEgressTimeStamp:
 * Reads back the egress time stamp on a dedicated message object.
 *
 * \par Synchronous, Non-reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param BufIdx_u8         Index of the controller within the context of the Ethernet Driver
 *
 * Parameters Out:
 * \param timeQualPtr       quality of HW time stamp, e.g. based on current drift.
 * \param timeStampPtr      current time stamp
 *
 * \return                  none
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) rba_Eth_GetEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                             VAR(uint8, AUTOMATIC) BufIdx_u8,
                                             P2VAR(Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                             P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    Eth_TransmitBufferDescriptorQueueRefType_t                                      TransmitDescriptorQueue_pst;
    rba_Eth_TxBufferDescriptorRefType_t                                              CurrentDesc_pst;                /* local pointer to register descriptor */
    Eth_TxBuffer_TransmitBufferQueueRefType_t                                       TransmitBufferQueue_pst;      /* local pointer to management structure for TX buffer */
    uint8 Descriptor_Index;


    /* Initialization of local variables*/
    TransmitDescriptorQueue_pst = &(Eth_Controllers_ast[CtrlIdx_u8].TransmitDescriptorQueue_st);


    SchM_Enter_Eth(CONTROLLER);

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];
    /* Get the current descriptor index from buffer descriptor link table*/
    Descriptor_Index = TransmitBufferQueue_pst->TxBufferToDescLinkTable_pu8[BufIdx_u8];

    /* Get the current descriptor address*/
    CurrentDesc_pst = &(TransmitDescriptorQueue_pst->First_pst[Descriptor_Index]);             // Descriptor and the buffer are mapped one to one
    SchM_Exit_Eth(CONTROLLER);


    /* Interrupt lock to avoid multiple access while reading the PTP time stamping registers */
     SchM_Enter_Eth(CONTROLLER);
     timeStampPtr->nanoseconds = CurrentDesc_pst->Des0_DataBuff1Addr_TimeStampLow_u32;                                             /*Reading the PTP Sync nanoseconds register */
     timeStampPtr->seconds     = CurrentDesc_pst->Des1_DataBuff2Addr_TimeStampHigh_u32;                                            /*Reading the PTP Sync seconds register */
     timeStampPtr->secondsHi   = ((uint16)(Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS));    /*Reading the 1588 timer seconds register */
     SchM_Exit_Eth(CONTROLLER);


    /* Check for time stamping quality */
    /* If the nanoseconds field of timeStampPtr is not within the boundary ( 0 - 9,99,999,999 ns)
     * value pointed via the time stamp quality ptr is set to INVALID */
    if(timeStampPtr->nanoseconds <= RBA_ETH_MAC_TIMESTAMP_NS_UPRLIMIT)
    {
        *timeQualPtr = ETH_VALID;
    }
    else
    {
        *timeQualPtr = ETH_INVALID;
    }

}
/**
 ***************************************************************************************************
 * \moduledescription
 * rba_Eth_GetIngressTimeStamp:
 * Reads back the ingress time stamp on a dedicated message object.
 *
 * \par Synchronous, Non-reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param DataPtr           Pointer to the message buffer, where Application expects ingress time stamping
 *
 * Parameters Out:
 * \param timeQualPtr       quality of HW time stamp, e.g. based on current drift.
 * \param timeStampPtr      current time stamp
 *
 * \return                  none
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) rba_Eth_GetIngressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                              P2CONST(Eth_DataType, AUTOMATIC, AUTOMATIC) DataPtr,
                                              P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                              P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    Eth_ReceiveBufferDescriptorQueueRefType_t       ReceiveDescriptorQueue_pst; /* local pointer to management structure for RX descriptor buffer elements */
    rba_Eth_RxBufferDescriptorRefType_t             CurrentDesc_pst;            /* local pointer to a the current processed descriptor in the descriptor buffer */
    uint32                                                                          ReceiveContextDescStatus;
    uint32                                                                           ReceiveNormalDescStatus;
    /* Initialization of local variables*/
    ReceiveDescriptorQueue_pst = &Eth_Controllers_ast[CtrlIdx_u8].ReceiveDescriptorQueue_st;
    timeStampPtr->secondsHi    = 0;
    timeStampPtr->seconds      = 0;
    timeStampPtr->nanoseconds  = 0;
    (void)DataPtr;


    SchM_Enter_Eth(CONTROLLER);
    /* Get the current descriptor address*/
    CurrentDesc_pst = &(ReceiveDescriptorQueue_pst->First_pst[(ReceiveDescriptorQueue_pst->CurrentIndex_u16)]);

    ReceiveNormalDescStatus  = ((CurrentDesc_pst->Des1_Reserved_Status_u32) & RBA_ETH_MAC_DMARXDES1_TSA);

    ReceiveNormalDescStatus |= (((CurrentDesc_pst->Des1_Reserved_Status_u32) & RBA_ETH_MAC_DMARXDES1_TD));
    /* Get the context descriptor address*/
    CurrentDesc_pst++;
    ReceiveContextDescStatus = (((CurrentDesc_pst->Des3_CtrlStatus_u32) & RBA_ETH_MAC_DMARXDESC3_CTXT));

     SchM_Exit_Eth(CONTROLLER);


    /* Interrupt lock to avoid multiple access while reading the PTP time stamping registers */


     SchM_Enter_Eth(CONTROLLER);
    if((ReceiveContextDescStatus == RBA_ETH_MAC_DMARXDESC3_CTXT) && (ReceiveNormalDescStatus  == RBA_ETH_MAC_DMARXDES1_TSA))
    {
        timeStampPtr->nanoseconds = CurrentDesc_pst->Des0_DataBuff1Addr_VlanTag_u32;                                             /*Reading the PTP nanoseconds value from the Rx context descriptor*/
        timeStampPtr->seconds     = CurrentDesc_pst->Des1_Reserved_Status_u32;                                            /*Reading the PTP second value from the Rx context Descriptor */
        timeStampPtr->secondsHi   = ((uint16)(Eth_Controllers_ast[CtrlIdx_u8].Registers_pst->MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS));

    }
    SchM_Exit_Eth(CONTROLLER);/*Reading the 1588 timer seconds register */


    /* Check for time stamping quality */
    /* If the nanoseconds field of timeStampPtr is not within the boundary ( 0 - 9,99,999,999 ns)
     * value pointed via the time stamp quality ptr is set to INVALID */
    if((timeStampPtr->nanoseconds <= RBA_ETH_MAC_TIMESTAMP_NS_UPRLIMIT) && (ReceiveNormalDescStatus  == RBA_ETH_MAC_DMARXDES1_TSA))
    {
        *timeQualPtr = ETH_VALID;
    }
    else
    {
        *timeQualPtr = ETH_INVALID;
    }
}

#endif

#define RBA_ETH_STOP_SEC_CODE
#include "rba_Eth_MemMap.h"

#endif /* ETH_CONFIGURED */


/*
**********************************************************************************************************************
*
**********************************************************************************************************************
*/

