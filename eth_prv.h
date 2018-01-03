#include "Eth_TxBuffer.h"
#include "Eth_RxBuffer.h"

#if (ETH_DEV_ERROR_DETECT == STD_ON)
    #include "Det.h"
#endif // (ETH_DEV_ERROR_DETECT == STD_ON)

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

/* MAC address defines */
#define ETH_TARGET_MAC_ADDRESS_OFFSET   (0U)
#define ETH_SOURCE_MAC_ADDRESS_OFFSET   (6U)

#define ETH_FRAMETYPE_OFFSET_BYTE1                (12U)
#define ETH_FRAMETYPE_OFFSET_BYTE2                (13U)
#define ETH_FRAMETYPE_VLAN_OFFSET_BYTE1           (16U)
#define ETH_FRAMETYPE_VLAN_OFFSET_BYTE2           (17U)

/* Ethernet Driver header length */
#define ETH_SRC_DST_FTYPE_LEN           (14U)

#define ETH_FRAMETYPE_LOWER_MASK        (0x00FFU)
#define ETH_FRAMETYPE_UPPER_MASK        (0xFF00U)
#define ETH_FRAGMENT_OFFSET_MASK        (0x1FFFU)
#define ETH_MORE_FRAGMENT_SHIFT         (13U)
#define ETH_MORE_FRAGMENT_FLAGSET       (1U)
#define ETH_ZERO_FRAGMENT_OFFSET        (0U)

#define ETH_FRAMETYPE_SHIFT             (0x08U)
#define ETH_FRAMETYPE_BYTE1_OFFSET      (12U)
#define ETH_FRAMETYPE_BYTE2_OFFSET      (13U)

/* DET macros */
#if(ETH_DEV_ERROR_DETECT == STD_ON)
    #define ETH_DET_REPORT_NORETURN(Condition, ApiId, ErrorId) \
                /* if condition is true -> error */  \
                if ((Condition) ) { (void)Det_ReportError(ETH_MODULE_ID, ETH_INDEX, (ApiId), (ErrorId)); } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0

    #define ETH_DET_REPORT_RETURN_NOERROR(Condition, ApiId, ErrorId) \
                /* if condition is true -> error */  \
                if ((Condition) ) { (void)Det_ReportError(ETH_MODULE_ID, ETH_INDEX, (ApiId), (ErrorId)); return; } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0

    #define ETH_DET_REPORT_RETURN_ERROR(Condition, ApiId, ErrorId, ReturnCode) \
                /* if condition is true -> error */ \
                if ((Condition) ) { (void)Det_ReportError(ETH_MODULE_ID, ETH_INDEX, (ApiId), (ErrorId)); return(ReturnCode); } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0

#else
    #define ETH_DET_REPORT_NORETURN(Condition, ApiId, ErrorId)  \
				/* if condition is true -> Continue with the program w/o reporting any DET error */  \
				if ((Condition) ) { /* Do nothing */ } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0


    #define ETH_DET_REPORT_RETURN_NOERROR(Condition, ApiId, ErrorId) \
				/* if condition is true -> Error, so exit from program w/o reporting any DET error */  \
				if ((Condition) ) { return; } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0


    #define ETH_DET_REPORT_RETURN_ERROR(Condition, ApiId, ErrorId, ReturnCode) \
				/* if condition is true -> Error, so exit from the program and return returncode w/o reporting any DET error  */ \
				if ((Condition) ) { return(ReturnCode); } \
				/*Below statement has no effect and is added to remove MISRA warning */ \
				(void)0

#endif

/* Eth DET values as defined in SWS_eth_00016 */
#define ETH_E_INV_CTRL_IDX      0x01
#define ETH_E_NOT_INITIALIZED   0x02
#define ETH_E_INV_POINTER       0x03
#define ETH_E_INV_PARAM         0x04
#define ETH_E_INV_CONFIG        0x05
#define ETH_E_INV_MODE          0x06
#define ETH_E_FRAMES_LOST       0x07

#define ETH_SID_ETH_INIT                            0x01
#define ETH_SID_ETH_CONTROLLERINIT                  0x02
#define ETH_SID_ETH_SETCONTROLLERMODE               0x03
#define ETH_SID_ETH_GETCONTROLLERMODE               0x04
#define ETH_SID_ETH_WRITEMII                        0x05
#define ETH_SID_ETH_READMII                         0x06
#define ETH_SID_ETH_GETPHYSADDR                     0x08
#define ETH_SID_ETH_PROVIDETXBUFFER                 0x09
#define ETH_SID_ETH_TRANSMIT                        0x0A
#define ETH_SID_ETH_RECEIVE                         0x0B
#define ETH_SID_ETH_TXCONFIRMATION                  0x0C
#define ETH_SID_ETH_GETVERSIONINFO                  0x0D
#define ETH_SID_ETH_RXIRQHDLR                       0x10
#define ETH_SID_ETH_TXIRQHDLR                       0x11
#define ETH_SID_ETH_UPDATEPHYSADDRFILTER            0x12
#define ETH_SID_ETH_SETPHYSADDR                     0x13
#define ETH_SID_ETH_GETCOUNTERVALUES                0x14
#define ETH_SID_ETH_GETRXSTATS                      0x15
#define ETH_SID_ETH_GETTXSTATS                      0x1C
#define ETH_SID_ETH_GETTXERRORCOUNTERVALUES         0x1D
#define ETH_SID_ETH_GETCURRENTTIME                  0x16
#define ETH_SID_ETH_ENABLEEGRESSTIMESTAMP           0x17
#define ETH_SID_ETH_GETEGRESSTIMESTAMP              0x18
#define ETH_SID_ETH_GETINGRESSTIMESTAMP             0x19
#define ETH_SID_ETH_SETCORRECTIONTIME               0x1a
#define ETH_SID_ETH_SETGLOBALTIME                   0x1b
#define ETH_SID_ETH_MAINFUNCTION                    0x20
/*This is a non autosar api and hence numbered from backwards to aviod numbering clash with AR apis*/
#define ETH_SID_ETH_COMBINEDIRQHDLR         0xFE
#define ETH_SID_ETH_DEMREPORTERROR          0xFD

#define ETH_MAXFRAMELENGTH                  (1504U)
#define ETH_MAXFRAMELENGTH_WITHOUT_VLAN     (1500U)

#define ETH_VLAN_FRAMETYPE                  (0x8100U)
#define ETH_PTP_FRAMETYPE                   (0x88F7U)
#define ETH_IPV4_FRAMETYPE                  (0x0800U)

/*Macros for Fragmentation handling*/

#define ETH_IPV4_MORE_FRAGMENT           0x14
#define ETH_IPV4_FRAGMENT_OFFSET         0x15
#define ETH_VLAN_IPV4_MORE_FRAGMENT      0x18
#define ETH_VLAN_IPV4_FRAGMENT_OFFSET    0x19

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
    #define ETH_START_SEC_VAR_CLEARED_8
    #include "Eth_MemMap.h"
	    extern VAR(uint8, ETH_VAR) Eth_CtrlPhysAddress_au8_MP[ETH_MAX_CTRLS_SUPPORTED][ETH_MAC_ADDRESS_LENGTH_BYTE];
    #define ETH_STOP_SEC_VAR_CLEARED_8
    #include "Eth_MemMap.h"

    #define ETH_START_SEC_VAR_CLEARED_UNSPECIFIED
    #include "Eth_MemMap.h"
        extern P2CONST(Eth_ConfigType, ETH_VAR, ETH_CFG) Eth_CurrentConfig_pco;

        extern P2CONST(Eth_CtrlConfigType_to,ETH_VAR, ETH_CFG) Eth_CurrentCtrlConfig_apco[ETH_MAX_CTRLS_SUPPORTED];

	    extern VAR(Eth_ControllerType_tst , ETH_VAR) Eth_Controllers_ast [ETH_MAX_CTRLS_SUPPORTED];
#ifdef ETH_DSM_RE_INIT_SUPPORT
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
	    extern VAR(Eth_DSMReInit_DemEventsHandle_tst , ETH_VAR) Eth_DSMReInit_DemEventsHandler_ast[ETH_TOTAL_DEM_EVENTS];
#endif
#endif
    #define ETH_STOP_SEC_VAR_CLEARED_UNSPECIFIED
    #include "Eth_MemMap.h"

    #define ETH_START_SEC_CODE
    #include "Eth_MemMap.h"
	    extern FUNC(boolean, ETH_CODE) Eth_CheckBitFieldStatus(const volatile uint32 *Reg_pu32,
													   uint32 MaskVal_u32,
													   uint32 ChkVal_u32,
													   uint32 TimeoutCnt_u32,
													   uint8  CtrlIdx_u8);
    #define ETH_STOP_SEC_CODE
    #include "Eth_MemMap.h"

#endif /* ETH_PRV_H_ */

/*<BASDKey>
**********************************************************************************************************************
* $History___:$
**********************************************************************************************************************
</BASDKey>*/


