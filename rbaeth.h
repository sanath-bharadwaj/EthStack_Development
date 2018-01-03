#ifndef RBA_ETH_H_
#define RBA_ETH_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */

#include "rba_Eth_Types.h"

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

/* This Macro is STD_ON because MIB counters dont reset after reading */
#define RBA_ETH_COUNTER_DIFFERENCE STD_ON
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
#define RBA_ETH_START_SEC_CODE
#include "rba_Eth_MemMap.h"

    extern void rba_Eth_Init(uint8 CtrlIdx_u8);

    extern Std_ReturnType rba_Eth_ControllerInit(uint8 CtrlIdx_u8);

    extern Std_ReturnType rba_Eth_SetControllerMode(uint8 CtrlIdx_u8,
                                                    Eth_ModeType CtrlMode_en);

    extern void rba_Eth_SetPhysAddr(uint8 CtrlIdx_u8);

    #if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
    extern Std_ReturnType rba_Eth_UpdatePhysAddrFilter(uint8 CtrlIdx_u8,
                                                       const uint8 *PhysAddrPtr_pcu8,
                                                       Eth_FilterActionType Action_en);
    #endif

#if defined RBA_ETH_EN_MII
#if (RBA_ETH_EN_MII == STD_ON)
    extern Std_ReturnType rba_Eth_WriteMii(uint8 CtrlIdx_u8,
                                           uint8 TrcvIdx_u8,
                                           uint8 RegIdx_u8,
                                           uint16 RegVal_u16);

    extern Std_ReturnType rba_Eth_ReadMii(uint8 CtrlIdx_u8,
                                          uint8 TrcvIdx_u8,
                                          uint8 RegIdx_u8,
                                          uint16 *RegValPtr_pu16);
#endif
#endif

#if defined(ETH_GET_COUNTER_VALUE_API)
#if (ETH_GET_COUNTER_VALUE_API == STD_ON)
    extern Std_ReturnType rba_Eth_GetCounterValues(uint8 CtrlIdx_u8,
                                                P2VAR(Eth_CounterType, AUTOMATIC, AUTOMATIC) CounterPtr_pst);
#endif
#endif

#if defined(ETH_GET_RX_STATS_API)
#if (ETH_GET_RX_STATS_API == STD_ON)
    extern Std_ReturnType rba_Eth_GetRxStats(uint8 CtrlIdx_u8,
                                              P2VAR(Eth_RxStatsType, AUTOMATIC, AUTOMATIC) RxStats_pst);
#endif
#endif

#if defined(ETH_GET_TX_STATS_API)
#if (ETH_GET_TX_STATS_API == STD_ON)
    extern Std_ReturnType rba_Eth_GetTxStats(uint8 CtrlIdx_u8,
                                                  P2VAR(Eth_TxStatsType, AUTOMATIC, AUTOMATIC) TxStats_pst);
#endif
#endif

#if defined(ETH_GET_TX_ERROR_COUNTER_API)
#if (ETH_GET_TX_ERROR_COUNTER_API == STD_ON)
    extern Std_ReturnType rba_Eth_GetTxErrorCounterValues(uint8 CtrlIdx_u8,
                                                  P2VAR(Eth_TxErrorCounterValuesType, AUTOMATIC, AUTOMATIC) TxErrorCounterValues_pst);
#endif
#endif

    extern boolean rba_Eth_TxDone(uint8 CtrlIdx_u8,
                                  uint8 DescriptorIndex_u8);

    extern void rba_Eth_ProgramTxDescriptor(uint8 CtrlIdx_u8,
                                            uint8 DescriptorIndex_u8,
                                            uint16 LenByte_u16,
                                            const uint8 *FramePtr_pu8);

#if defined (RBA_ETH_EN_RX_POLLING)
#if (RBA_ETH_EN_RX_POLLING == STD_ON)
    extern void rba_Eth_Receive(uint8 CtrlIdx_u8,
                                Eth_RxStatusType* RxStatusPtr);
#endif
#endif

#if defined (ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
    extern void rba_Eth_GetDemErrorCounters(uint8 CtrlIdx_u8, Eth_DemErrorCountHandleRefType_t Eth_DemErrorCountHandle_pst);
#endif
#endif

#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
    /* Returns a time value out of the HW registers according to the capability of the HW */
    extern FUNC(Std_ReturnType, ETH_CODE) rba_Eth_GetCurrentTime( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                                  P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                                  P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );
    /* Activates egress time stamping */
    extern FUNC(void, ETH_CODE) rba_Eth_EnableEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                               VAR(uint8, AUTOMATIC) BufIdx_u8 );

    /* Reads back the egress time stamp */
    extern FUNC(void, ETH_CODE) rba_Eth_GetEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                            VAR(uint8, AUTOMATIC) BufIdx_u8,
                                                            P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                            P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );

    /* Reads back the ingress time stamp */
    extern FUNC(void, ETH_CODE) rba_Eth_GetIngressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                             P2CONST(Eth_DataType, AUTOMATIC, AUTOMATIC) DataPtr,
                                                             P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                             P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr );

#endif /* ETH_GLOBAL_TIME_SUPPORT */


#define RBA_ETH_STOP_SEC_CODE
#include "rba_Eth_MemMap.h"

#define RBA_ETH_START_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"

#if defined (RBA_ETH_EN_RX_INTERRUPT)
#if (RBA_ETH_EN_RX_INTERRUPT == STD_ON)
    extern void rba_Eth_RxIrqHdlr(uint8 CtrlIdx_u8);
#endif
#endif


#if defined (RBA_ETH_EN_TX_INTERRUPT)
#if (RBA_ETH_EN_TX_INTERRUPT == STD_ON)
    extern void rba_Eth_TxIrqHdlr(uint8 CtrlIdx_u8);
#endif
#endif
    extern void rba_Eth_CtrlErrIrqHdlr(uint8 CtrlIdx_u8);


#define RBA_ETH_STOP_SEC_CODE_FAST
#include "rba_Eth_MemMap.h"

#endif /* RBA_ETH_H_ */
