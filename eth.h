#ifndef ETH_H_
#define ETH_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */

#include "Eth_Types.h"

/* According to coding guidelines Eth_Cfg.h can be included here. But AR SWS mentions that it should be included
 * in Eth_GeneralTypes.h though neither Eth_Types nor Eth_GeneralTypes have any dependency on it */
/*#include Eth_Cfg.h.*/

/*Included after Eth_Types.h due to dependency and coding guidelines adherence*/
#include "Eth_PBcfg.h"

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

/* Eth version info */
#define ETH_VENDOR_ID                   (6U)

/* Eth Module id */
#define ETH_MODULE_ID                   (88U)

/* AUTOSAR specification version */
#define ETH_AR_RELEASE_MAJOR_VERSION    (4U)

#define ETH_AR_RELEASE_MINOR_VERSION    (2U)

#define ETH_AR_RELEASE_REVISION_VERSION (2U)

/* Software version information */
#define ETH_SW_MAJOR_VERSION            (4U)

#define ETH_SW_MINOR_VERSION            (0U)

#define ETH_SW_PATCH_VERSION            (0U)


/*TODO: Implement Version Check for the next release*/
#if 0
/*Version check with AR versions*/

#if (!defined(COMTYPE_AR_RELEASE_MAJOR_VERSION) || (COMTYPE_AR_RELEASE_MAJOR_VERSION != ETH_AR_RELEASE_MAJOR_VERSION))
  #error "AUTOSAR major version undefined or mismatched"
#endif
#if (!defined(COMTYPE_AR_RELEASE_MINOR_VERSION) || (COMTYPE_AR_RELEASE_MINOR_VERSION != ETH_AR_RELEASE_MINOR_VERSION))
  #error "AUTOSAR minor version undefined or mismatched"
#endif

#endif

/*Version check with generated files*/

#if (!defined(ETH_GEN_MAJOR_VERSION) || (ETH_GEN_MAJOR_VERSION != ETH_SW_MAJOR_VERSION))
  #error "Gen Code major version undefined or mismatched"
#endif
#if (!defined(ETH_GEN_MINOR_VERSION) || (ETH_GEN_MINOR_VERSION != ETH_SW_MINOR_VERSION))
  #error "Gen Code minor version undefined or mismatched"
#endif

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
    /* ETH027: Initializes the Ethernet Driver */
    extern FUNC(void, ETH_CODE) Eth_Init( P2CONST(Eth_ConfigType, AUTOMATIC, ETH_CFG) CfgPtr_pco );

    /* ETH033: Initializes the indexed controller */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_ControllerInit( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                              VAR(uint8, AUTOMATIC) CfgIdx_u8 );


    /* ETH041: Enables / disables the indexed controller */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_SetControllerMode( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                 VAR(Eth_ModeType, AUTOMATIC) CtrlMode_en );

    /* ETH046: Obtains the state of the indexed controller */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_GetControllerMode( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                 P2VAR(Eth_ModeType, AUTOMATIC, AUTOMATIC) CtrlModePtr_pen );

    /* ETH052: Obtains the physical source address used by the indexed controller */
    extern FUNC(void, ETH_CODE) Eth_GetPhysAddr( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                 P2VAR(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8 );

    /* ETH???: Sets the physical source address used by the indexed controller */
    extern FUNC(void, ETH_CODE) Eth_SetPhysAddr(VAR(uint8, AUTOMATIC)  CtrlIdx_u8,
                                                P2CONST(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8);
    #if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_UpdatePhysAddrFilter(VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                   P2CONST(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pcu8,
                                                                   VAR(Eth_FilterActionType, AUTOMATIC) Action_en);
    #endif

#if defined(ETH_EN_MII)
 #if (ETH_EN_MII == STD_ON)
    /* ETH058: Configures a transceiver register or triggers a function offered by the receiver */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_WriteMii( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                      VAR(uint8, AUTOMATIC) TrcvIdx_u8,
                                                      VAR(uint8, AUTOMATIC) RegIdx_u8,
                                                      VAR(uint16, AUTOMATIC) RegVal_u16 );

    /* ETH064: Reads a transceiver register */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_ReadMii( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                     VAR(uint8, AUTOMATIC) TrcvIdx_u8,
                                                     VAR(uint8, AUTOMATIC) RegIdx_u8,
                                                     P2VAR(uint16, AUTOMATIC, AUTOMATIC) RegValPtr_pu16 );
 #endif
#endif

#if defined(ETH_GET_COUNTER_VALUE_API)
#if (ETH_GET_COUNTER_VALUE_API == STD_ON)
    /* ETH0226 : Reads the list of Drop counter values from indexed controller*/
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_GetCounterValues(uint8 CtrlIdx_u8,
                                                               P2VAR(Eth_CounterType, AUTOMATIC, AUTOMATIC) CounterPtr_pst);
#endif
#endif

#if defined(ETH_GET_RX_STATS_API)
#if (ETH_GET_RX_STATS_API == STD_ON)
    /*ETH0233: Reads the list of Statistics counter values from indexed controller for Rx Frame*/
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_GetRxStats(uint8 CtrlIdx_u8,
                                                         P2VAR(Eth_RxStatsType, AUTOMATIC, AUTOMATIC) RxStats_pst);
#endif
#endif

#if defined(ETH_GET_TX_STATS_API)
#if (ETH_GET_TX_STATS_API == STD_ON)
    /*ETH91005: Reads the list of Statistics counter values from indexed controller for Tx Frame*/
    extern Std_ReturnType Eth_GetTxStats(uint8 CtrlIdx_u8,
                                                      P2VAR(Eth_TxStatsType, AUTOMATIC, AUTOMATIC) TxStats_pst);
#endif
#endif

#if defined(ETH_GET_TX_ERROR_COUNTER_API)
#if (ETH_GET_TX_ERROR_COUNTER_API == STD_ON)
    /*ETH91006: Reads the list of Statistics counter values from indexed controller for Tx Error Frame*/
    extern Std_ReturnType Eth_GetTxErrorCounterValues(uint8 CtrlIdx_u8,
                                                      P2VAR(Eth_TxErrorCounterValuesType, AUTOMATIC, AUTOMATIC) TxErrorCounterValues_pst);
#endif
#endif
    /* ETH077: Provides access to a transmit buffer of the specified controller */
    extern FUNC(BufReq_ReturnType, ETH_CODE) Eth_ProvideTxBuffer( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                  P2VAR(uint8, AUTOMATIC, AUTOMATIC) BufIdxPtr_pu8,
                                                                  P2VAR(P2VAR(Eth_DataType, AUTOMATIC, AUTOMATIC), AUTOMATIC, AUTOMATIC) BufPtr_ppo,
                                                                  P2VAR(uint16, AUTOMATIC, AUTOMATIC) LenBytePtr_pu16 );

    /* ETH087: Triggers transmission of a previously filled transmit buffer */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_Transmit( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                        VAR(uint8, AUTOMATIC) BufIdx_u8,
                                                        VAR(Eth_FrameType, AUTOMATIC) FrameType_o,
                                                        VAR(boolean, AUTOMATIC) TxConfirmation_b,
                                                        VAR(uint16, AUTOMATIC) LenByte_u16,
                                                        P2VAR(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8 );
#if defined(ETH_EN_RX_POLLING)
 #if (ETH_EN_RX_POLLING == STD_ON)
    /* ETH095: Triggers frame reception */
    extern FUNC(void, ETH_CODE) Eth_Receive( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                             P2VAR(Eth_RxStatusType, AUTOMATIC, AUTOMATIC) RxStatusPtr );
 #endif
#endif

    /* ETH100: Triggers frame transmission confirmation */
    extern FUNC(void, ETH_CODE) Eth_TxConfirmation( VAR(uint8, AUTOMATIC) CtrlIdx_u8 );

    /* ETH106: Returns the version information of this module */
#if ( ETH_VERSION_INFO_API == STD_ON)
    extern FUNC(void, ETH_CODE) Eth_GetVersionInfo( P2VAR(Std_VersionInfoType, AUTOMATIC, AUTOMATIC) VersionInfoPtr_pst );
#endif
#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
    extern FUNC(void, ETH_CODE) Eth_DemReportErrorStatus( Dem_EventIdType EventId, Dem_EventStatusType EventStatus);
#endif
#endif
# if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
    /* ETH181: Returns a time value out of the HW registers according to the capability of the HW */
    extern FUNC(Std_ReturnType, ETH_CODE) Eth_GetCurrentTime( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                              P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                              P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );

    /* ETH186: Activates egress time stamping */
    extern FUNC(void, ETH_CODE) Eth_EnableEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                           VAR(uint8, AUTOMATIC) BufIdx_u8 );

    /* ETH190: Reads back the egress time stamp */
    extern FUNC(void, ETH_CODE) Eth_GetEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                        VAR(uint8, AUTOMATIC) BufIdx_u8,
                                                        P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                        P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );

    /* ETH195: Reads back the ingress time stamp */
    extern FUNC(void, ETH_CODE) Eth_GetIngressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                         P2CONST(Eth_DataType, AUTOMATIC, AUTOMATIC) DataPtr,
                                                         P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                         P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );


#endif /* ETH_GLOBAL_TIME_SUPPORT */
#ifdef ETH_ECUC_RB_RTE_IN_USE
#if (ETH_ECUC_RB_RTE_IN_USE != STD_ON)
    /* ETH171: The function checks for controller errors and lost frames. Used for polling state changes. */
    extern FUNC(void, ETH_CODE) Eth_MainFunction( void );
#endif
#endif

#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#define ETH_START_SEC_CODE_FAST
#include "Eth_MemMap.h"

#if defined(ETH_EN_RX_INTERRUPT_0)
 #if (ETH_EN_RX_INTERRUPT_0 == STD_ON)

    /* ETH109: Handles frame reception interrupts of the indexed controller */
    extern FUNC(void, ETH_ISR_CODE) Eth_RxIrqHdlr_0( void );

 #endif
#endif
#if (ETH_MAX_CTRLS_SUPPORTED == 2)
#if defined(ETH_EN_RX_INTERRUPT_1)
 #if (ETH_EN_RX_INTERRUPT_1 == STD_ON)

    extern FUNC(void, ETH_ISR_CODE) Eth_RxIrqHdlr_1( void );

 #endif
#endif
#endif
#if defined(ETH_EN_TX_INTERRUPT_0)
 #if (ETH_EN_TX_INTERRUPT_0 == STD_ON)

    /* ETH114: Handles frame transmission interrupts of the indexed controller */
    extern FUNC(void, ETH_ISR_CODE) Eth_TxIrqHdlr_0( void );

 #endif
#endif
#if (ETH_MAX_CTRLS_SUPPORTED == 2)
#if defined(ETH_EN_TX_INTERRUPT_1)
 #if (ETH_EN_TX_INTERRUPT_1 == STD_ON)

    extern FUNC(void, ETH_ISR_CODE) Eth_TxIrqHdlr_1( void );

 #endif
#endif
#endif

    /*Handles transceiever and error interrupts. Available only in some controllers*/
    extern FUNC(void, ETH_CODE) Eth_CombinedIrqHdlr_0( void );

#if (ETH_MAX_CTRLS_SUPPORTED == 2)

    extern FUNC(void, ETH_CODE) Eth_CombinedIrqHdlr_1( void );
#endif

#define ETH_STOP_SEC_CODE_FAST
#include "Eth_MemMap.h"

#endif /* ETH_H_ */

