#include "Eth.h"
#include "rba_Eth.h"
#include "EthIf.h"
#include "EthIf_Cbk.h"
#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
#include "Dem.h"
#endif
#endif
#include "rba_Eth_Cfg_Time.h"
#include "Eth_Prv.h"
#include "Eth_Cfg_SchM.h"
#if defined(ETH_ECUC_RB_RTE_IN_USE)
#if (ETH_ECUC_RB_RTE_IN_USE == STD_ON)
#include "SchM_Eth.h"
#endif
#endif
#ifdef ETH_CONFIGURED


/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */
/* rba_Eth shall define a macro RBA_ETH_COUNTER_DIFFERENCE as STD_ON or STD_OFF.
 * This shall decide if the counter values neecds to stored and difference needs to be checked to get the change in counters
 * or a new value is returned everytime.
 * STD_OFF - Controller resets the counters evenytime the values are read from the register(Clear on read)
 * STD_ON  - Controller returns counter values as it is. */
#define ETH_COUNTER_DIFFERENCE  RBA_ETH_COUNTER_DIFFERENCE
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
#define ETH_START_SEC_VAR_CLEARED_8
#include "Eth_MemMap.h"
    VAR(uint8, ETH_VAR) Eth_CtrlPhysAddress_au8_MP[ETH_MAX_CTRLS_SUPPORTED][ETH_MAC_ADDRESS_LENGTH_BYTE];
#define ETH_STOP_SEC_VAR_CLEARED_8
#include "Eth_MemMap.h"

#define ETH_START_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"
    static P2CONST(Eth_ConfigType, ETH_VAR, ETH_CFG) Eth_CurrentConfig_pco;

    /* global array which contains pointers to the single controller configurations in const PBCfg */
    P2CONST(Eth_CtrlConfigType_to,ETH_VAR, ETH_CFG) Eth_CurrentCtrlConfig_apco[ETH_MAX_CTRLS_SUPPORTED];

    /* Declaration of Eth_ControllerState_aen_MP (local variable) of type Eth_StateType irrespective of DET ON/OFF to handle the state of the controller  */
    static VAR(Eth_StateType, ETH_VAR) Eth_ControllerState_aen_MP[ETH_MAX_CTRLS_SUPPORTED];

    static VAR(Eth_ModeType, ETH_VAR) Eth_ControllerMode_aen_MP[ETH_MAX_CTRLS_SUPPORTED];

    /* array of global management structures per Eth controllers*/
    VAR(Eth_ControllerType_tst , ETH_VAR) Eth_Controllers_ast [ETH_MAX_CTRLS_SUPPORTED];

#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
    VAR(Eth_DSMReInit_DemEventsHandle_tst , ETH_VAR) Eth_DSMReInit_DemEventsHandler_ast[ETH_TOTAL_DEM_EVENTS];
#endif
#ifdef ETH_DEM_REPORTING_SUPPORT
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
     static VAR(Eth_DemErrorCountHandle_tst , ETH_VAR) Eth_DemErrorCountHandle_ast[ETH_MAX_CTRLS_SUPPORTED];
#endif
#endif
#endif
#endif
#define ETH_STOP_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"

/*
 ***************************************************************************************************
 * Prototype for Static functions: Start
 ***************************************************************************************************
 */
#define ETH_START_SEC_CODE
#include "Eth_MemMap.h"

#ifdef ETH_DEM_REPORTING_SUPPORT
#if ETH_DEM_REPORTING_SUPPORT == STD_ON
        static FUNC(void, ETH_CODE) Eth_DemReportInit(uint8 CtrlIdx_u8);

        static FUNC(void, ETH_CODE) Eth_ExtendedProductionErrorsHandle(uint32 CurrentCounterValue_u32
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                                                        ,uint32 PreviousCounterValue_u32
#endif
#endif
                                                        ,Dem_EventIdType ErrorIdValue_u16);
#endif
#endif
/*
 ***************************************************************************************************
 * Prototype for Static functions: End
 ***************************************************************************************************
 */


/**
 ***************************************************************************************************
 * \moduledescription
 * ETH027: Initializes the Ethernet Driver
 * \par Service ID 0x01, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CfgPtr_pco    Points to the implementation specific structure
 *
 * \return        None
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) Eth_Init( P2CONST(Eth_ConfigType, AUTOMATIC, ETH_CFG) CfgPtr_pco )
{
    uint8 CtrlIdx_u8;
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
    uint8 EventIndex_u8;
#endif
    for (CtrlIdx_u8 = 0U; CtrlIdx_u8 < ETH_MAX_CTRLS_SUPPORTED;  CtrlIdx_u8++)
    {
        /* Initial States */
        Eth_ControllerState_aen_MP[CtrlIdx_u8] = ETH_STATE_UNINIT;
        Eth_ControllerMode_aen_MP[CtrlIdx_u8]  = ETH_MODE_DOWN;
    }
    /* DET checks */
#if ( ETH_CFG_CONFIGURATION_VARIANT != ETH_CFG_VARIANT_PRE_COMPILE )
    /* If variant is not precompile then EcuM shall provide the configset that needs to be initialized with
     * And shall not provide it as a NULL_PTR */
    /* If NULL POINTER is provided then report DET */
    ETH_DET_REPORT_RETURN_NOERROR((CfgPtr_pco == NULL_PTR), ETH_SID_ETH_INIT, ETH_E_INV_POINTER);

    /* Initialize the global variable with the provided configuration */
    Eth_CurrentConfig_pco = CfgPtr_pco;
#else
    /* If variant is precompile then EcuM shall provide a NULL_PTR, The default - Eth_ConfigSet[0] shall be used to
     * initialize */
    /* If NULL POINTER is NOT provided then report DET */
    ETH_DET_REPORT_RETURN_NOERROR((CfgPtr_pco != NULL_PTR), ETH_SID_ETH_INIT, ETH_E_INV_POINTER);

    /* Note: AR404 allows passing a NULL_PTR! */
    Eth_CurrentConfig_pco = &Eth_ConfigSets[0];

    (void)CfgPtr_pco;
#endif

#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
    /* Initialzing DEM ReInit handling Structures */
    for(EventIndex_u8 = 0; EventIndex_u8 < ETH_TOTAL_DEM_EVENTS; EventIndex_u8++)
    {
        /* Critical section as global variables are updated */
        SchM_Enter_Eth(CONTROLLER);

        /* Initialize variables */
        Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].ReReportRequire_b = FALSE;
        Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].DemEventStatus    = DEM_EVENT_STATUS_PASSED;

        SchM_Exit_Eth(CONTROLLER);
    }
#endif

    /* reset controller states and modes */
    for (CtrlIdx_u8 = 0U; CtrlIdx_u8 < ETH_MAX_CTRLS_SUPPORTED;  CtrlIdx_u8++)
    {
        rba_Eth_Init(CtrlIdx_u8);

        /* Changing state of the controller irrespective of DET ON/OFF  */
        Eth_ControllerState_aen_MP[CtrlIdx_u8] = ETH_STATE_INIT;
        Eth_ControllerMode_aen_MP[CtrlIdx_u8]  = ETH_MODE_DOWN;
    }

    return;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH033: Initializes the indexed controller
 * \par Service ID 0x02, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 * \param CfgIdx_u8     Index of the used configuration
 *
 * \return          Std_ReturnType {E_OK: success; E_NOT_OK: Controller initialization failed}
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_ControllerInit( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                   VAR(uint8, AUTOMATIC) CfgIdx_u8 )
{
    uint32         Index_u32;
    Std_ReturnType Result_o;

    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_CONTROLLERINIT, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if at least Eth_Init was called -> ETH_STATE_INIT or ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] == ETH_STATE_UNINIT, ETH_SID_ETH_CONTROLLERINIT, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if provided CfgIdx is within the range of available configurations */
    ETH_DET_REPORT_RETURN_ERROR(CfgIdx_u8 >= ETH_CONFIGTYPE_COUNT, ETH_SID_ETH_CONTROLLERINIT, ETH_E_INV_CONFIG, E_NOT_OK);

    Eth_CurrentCtrlConfig_apco[CtrlIdx_u8] = &Eth_CurrentConfig_pco->EthCtrlConfig_ao[CtrlIdx_u8];
    /* copy pre-configured local MAC address from const PBCfg area to RAM table */
    for (Index_u32 = 0U; Index_u32 < ETH_MAC_ADDRESS_LENGTH_BYTE; Index_u32++)
    {
        Eth_CtrlPhysAddress_au8_MP[CtrlIdx_u8][Index_u32]  = Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthCtrlPhysAddress_pcu8[Index_u32];
    }

    Result_o = rba_Eth_ControllerInit(CtrlIdx_u8);

    /* After controller init controller is in mode down */
    Eth_ControllerMode_aen_MP[CtrlIdx_u8] = ETH_MODE_DOWN;

    /* Changing state of the controller irrespective of DET ON/OFF  */
    if (Result_o == E_OK) /* Is controller initialization successful? */
    { /* yes */
        /* change controller state */
        Eth_ControllerState_aen_MP[CtrlIdx_u8] = ETH_STATE_ACTIVE; /* ETH_STATE_ACTIVE doesn't mean ETH_MODE_ACTIVE which means communicate-able*/
    }
    else
    { /* no */
        /* not used */
    }

    return(Result_o);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH041: Enables / disables the indexed controller
 * \par Service ID 0x03, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 * \param CtrlMode_en   ETH_MODE_DOWN:   disable the controller;
 *                      ETH_MODE_ACTIVE: enable the controller
 *
 * \return              Std_ReturnType {E_OK: success; E_NOT_OK: Setting of controller mode failed }
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_SetControllerMode( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                      VAR(Eth_ModeType, AUTOMATIC) CtrlMode_en )
{
    Std_ReturnType Result_o;

    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_SETCONTROLLERMODE, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_SETCONTROLLERMODE, ETH_E_NOT_INITIALIZED, E_NOT_OK);

    if (CtrlMode_en != Eth_ControllerMode_aen_MP[CtrlIdx_u8]) /* check if current mode does not equal as requested */
    { /* yes, requested mode differs */

        Result_o = rba_Eth_SetControllerMode(CtrlIdx_u8, CtrlMode_en);

        if (Result_o == E_OK)
        {
            /* change controller mode in global variable */
            Eth_ControllerMode_aen_MP[CtrlIdx_u8] = CtrlMode_en;
#ifdef ETH_DEM_REPORTING_SUPPORT
#if ETH_DEM_REPORTING_SUPPORT == STD_ON
            /* Init global structure that stores counter values and report DEM_EVENT_STATUS_PASSED
             * for all the events except ETH_E_ACCESS.
             * For ETH_E_ACCESS, This shall be done by Eth_CheckBitFieldStatus API*/
            Eth_DemReportInit(CtrlIdx_u8);
#endif
#endif
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    { /* no, already in requested mode */
        Result_o = E_OK;
    }

    return(Result_o);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH046: Obtains the state of the indexed controller
 * \par Service ID 0x04, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param CtrlModePtr_e ETH_MODE_DOWN: the controller is disabled; disabledETH_MODE_ACTIVE: the controller is enabled
 *
 * \return              Std_ReturnType {E_OK: success; E_NOT_OK: transceiver could not be initialized}
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_GetControllerMode( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                      P2VAR(Eth_ModeType, AUTOMATIC, AUTOMATIC) CtrlModePtr_pen )
{
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETCONTROLLERMODE, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETCONTROLLERMODE, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(CtrlModePtr_pen == NULL_PTR, ETH_SID_ETH_GETCONTROLLERMODE, ETH_E_INV_POINTER, E_NOT_OK);

    /* return state from RAM, since even if state is gathered from different HW registers, this doesn't guarantee
     * that the Eth controller (MAC) can be used. */
    *CtrlModePtr_pen = Eth_ControllerMode_aen_MP[CtrlIdx_u8];

    return(E_OK);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH052: Obtains the physical source address used by the indexed controller
 * \par Service ID 0x08, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param PhysAddrPtr_pu8   Physical source address (MAC address) in network byte order.
 *
 * \return              None
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) Eth_GetPhysAddr( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                      P2VAR(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8 )
{
    uint32 Index_u32;

    /* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETPHYSADDR, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETPHYSADDR, ETH_E_NOT_INITIALIZED);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(PhysAddrPtr_pu8 == NULL_PTR, ETH_SID_ETH_GETPHYSADDR, ETH_E_INV_POINTER);

    for (Index_u32 = 0U; Index_u32 < ETH_MAC_ADDRESS_LENGTH_BYTE; Index_u32++)
    {
        /* read out the value from RAM */
        PhysAddrPtr_pu8[Index_u32] = Eth_CtrlPhysAddress_au8_MP[CtrlIdx_u8][Index_u32];
    }

    return;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH???: Sets the physical source address used by the indexed controller
 * This function changes the MAC address in the 1st RX MAC Filter register (position 1).
 * This MAC address at position 1 is used as the local unicast MAC address which
 * is allowed to be received and inserted in TX Frames as the local MAC address.
 * \par Service ID 0x0E?, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param PhysAddrPtr_pu8   Physical source address (MAC address) in network byte order.
 *
 * \return              None
 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) Eth_SetPhysAddr(VAR(uint8, AUTOMATIC)  CtrlIdx_u8,
                                     P2CONST(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8)
{
    uint32 Index_u32;

    /* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_SETPHYSADDR, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_SETPHYSADDR, ETH_E_NOT_INITIALIZED);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(PhysAddrPtr_pu8 == NULL_PTR, ETH_SID_ETH_SETPHYSADDR, ETH_E_INV_POINTER);

    for (Index_u32 = 0U; Index_u32 < ETH_MAC_ADDRESS_LENGTH_BYTE; Index_u32++)
    {
        /* store in RAM*/
        Eth_CtrlPhysAddress_au8_MP[CtrlIdx_u8][Index_u32] = PhysAddrPtr_pu8[Index_u32];
    }

    /* set in the HW register */
    rba_Eth_SetPhysAddr(CtrlIdx_u8);
}



/**
 ***************************************************************************************************
 * \moduledescription
 * ETH???:
 * Update the physical source address to/from the indexed controller filter. If the Ethernet
 * Controller is not capable to do the filtering, the software has to do this.
 * In the ZGEM there are 4 HW registers which can be used for the HW filtering. Register 1 is only
 * used for the unicast local MAC address. This register is set to the configured value (see
 * EthCtrlPhyAddress ) during INIT phase. During runtime/UP this register 1 can only be changed by
 * Eth_SetPhysAddr function. Thus only the remaining 3 registers can be set by this API. When adding
 * additional MAC addresses, not only the multicast MAC address will pass the HW RX Filter of the MAC.
 * If all of the remaining 3 registers are set by this function, an additional call will return
 * E_NOT_OK. Set Filters with MAC addresses can be deleted by calling this function with
 * PhysAddrPtr_pcu8==MAC address to be deleted and Action_en set==REMOVE_FROM_FILTER.
 *
 * \par Service ID ?, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param PhysAddrPtr_pcu8  Physical source address (MAC address) in network byte order.
 * \param Action_en         Add or remove the address from the Ethernet controllers filter
 *
 * \return                  Std_ReturnType {E_OK: success; E_NOT_OK: update of RX PhysAddrFilter failed}
 *
 ***************************************************************************************************
 */

#if (ETH_UPDATE_PHYS_ADDR_FILTER == STD_ON)
FUNC(Std_ReturnType, ETH_CODE) Eth_UpdatePhysAddrFilter(VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                        P2CONST(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pcu8,
                                                        VAR(Eth_FilterActionType, AUTOMATIC) Action_en)
{
    Std_ReturnType Result_o;

    /* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_UPDATEPHYSADDRFILTER, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_UPDATEPHYSADDRFILTER, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(PhysAddrPtr_pcu8 == NULL_PTR, ETH_SID_ETH_UPDATEPHYSADDRFILTER, ETH_E_INV_POINTER, E_NOT_OK);

    Result_o = rba_Eth_UpdatePhysAddrFilter(CtrlIdx_u8, PhysAddrPtr_pcu8, Action_en);

    return(Result_o);
}
#endif

#if defined(ETH_EN_MII)
 #if (ETH_EN_MII == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH058: Configures a transceiver register or triggers a function offered by the receiver
 * \par Service ID 0x05, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 * \param TrcvIdx_u8    Index of the transceiver on the MII
 * \param RegIdx_u8     Index of the transceiver register on the MII
 * \param RegVal_u16    Value to be written into the indexed register
 *
 * \return Std_ReturnType       E_OK: MII register written
 *                              E_NOT_OK: MII register write failure
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_WriteMii( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                   VAR(uint8, AUTOMATIC) TrcvIdx_u8,
                                   VAR(uint8, AUTOMATIC) RegIdx_u8,
                                   VAR(uint16, AUTOMATIC) RegVal_u16 )
{
    Std_ReturnType    Result_o;
    /* Initialise the Result_o to E_NOT_OK */
    Result_o = E_NOT_OK;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_WRITEMII, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_WRITEMII, ETH_E_NOT_INITIALIZED,E_NOT_OK);

    Result_o = rba_Eth_WriteMii( CtrlIdx_u8, TrcvIdx_u8, RegIdx_u8, RegVal_u16 );

    return Result_o;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH064: Reads a transceiver register
 * \par Service ID 0x06, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param TrcvIdx_u8        Index of the transceiver on the MII
 * \param RegIdx_u8         Index of the transceiver register on the MII
 *
 * Parameter Out:
 * \param RegValPtr_pu16    Filled with the register content of the indexed register
 *
 * \return Std_ReturnType   E_OK: MII register read.
 *                          E_NOT_OK: MII register read failure.
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_ReadMii( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                  VAR(uint8, AUTOMATIC) TrcvIdx_u8,
                                  VAR(uint8, AUTOMATIC) RegIdx_u8,
                                  P2VAR(uint16, AUTOMATIC, AUTOMATIC) RegValPtr_pu16 )
{
    Std_ReturnType    Result_o;
    Result_o = E_NOT_OK;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_READMII, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_READMII, ETH_E_NOT_INITIALIZED,E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(RegValPtr_pu16 == NULL_PTR, ETH_SID_ETH_READMII, ETH_E_INV_POINTER,E_NOT_OK);

    Result_o = rba_Eth_ReadMii( CtrlIdx_u8, TrcvIdx_u8, RegIdx_u8, RegValPtr_pu16 );

    return Result_o;
}
 #endif
#endif

#if defined(ETH_GET_COUNTER_VALUE_API)
 #if (ETH_GET_COUNTER_VALUE_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH0226: Reads the list of Get counter values from indexed controller
 * \par Service ID 0x14, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter InOut:
 *
 * Parameter Out:
 * \param CounterPtr_pst    counter values according to IETF RFC 1757, RFC 1643 and RFC 2233.
 * \return  Std_ReturnType   E_OK: success
 *                           E_NOT_OK: drop counter could not be obtained
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_GetCounterValues( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                 P2VAR(Eth_CounterType, AUTOMATIC, AUTOMATIC) CounterPtr_pst)
{
    Std_ReturnType    Result_o;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETCOUNTERVALUES, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETCOUNTERVALUES, ETH_E_NOT_INITIALIZED,E_NOT_OK);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE.  */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_GETCOUNTERVALUES, ETH_E_INV_MODE, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(CounterPtr_pst == NULL_PTR, ETH_SID_ETH_GETCOUNTERVALUES, ETH_E_INV_POINTER,E_NOT_OK);

    Result_o = rba_Eth_GetCounterValues( CtrlIdx_u8, CounterPtr_pst);

    return Result_o;
}

 #endif
#endif

#if defined(ETH_GET_RX_STATS_API)
 #if (ETH_GET_RX_STATS_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH0233: Reads the list of Statistics Rx counter values from indexed controller
 * \par Service ID 0x15, Synchronous, Non Reentrant
 * Via this function the different counters (error, statistic etc.) can be read out.
 *
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param RxStats_pst    List of values according to IETF RFC 2819
 *
 * \return  Std_ReturnType    E_OK: success
 *                            E_NOT_OK: Statistics counter could not be obtained
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_GetRxStats( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                  P2VAR(Eth_RxStatsType, AUTOMATIC, AUTOMATIC) RxStats_pst)
{
    Std_ReturnType    Result_o;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETRXSTATS, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETRXSTATS, ETH_E_NOT_INITIALIZED,E_NOT_OK);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE. */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_GETRXSTATS, ETH_E_INV_MODE, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(RxStats_pst == NULL_PTR, ETH_SID_ETH_GETRXSTATS, ETH_E_INV_POINTER,E_NOT_OK);

    Result_o = rba_Eth_GetRxStats( CtrlIdx_u8, RxStats_pst);

    return Result_o;

}
 #endif
#endif

#if defined(ETH_GET_TX_STATS_API)
#if (ETH_GET_TX_STATS_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH91005: Reads the list of Statistics Tx counter values from indexed controller
 * \par Service ID 0x1C, Synchronous, Non Reentrant
 * Via this function the different counters (error, statistic etc.) can be read out.
 *
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param TxStats_pst    List of values according to IETF RFC 1213
 *
 * \return  Std_ReturnType    E_OK: success
 *                            E_NOT_OK: Statistics counter could not be obtained
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_GetTxStats( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                  P2VAR(Eth_TxStatsType, AUTOMATIC, AUTOMATIC) TxStats_pst)
{
    Std_ReturnType    Result_o;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETTXSTATS, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETTXSTATS, ETH_E_NOT_INITIALIZED,E_NOT_OK);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE. */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_GETTXSTATS, ETH_E_INV_MODE, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(TxStats_pst == NULL_PTR, ETH_SID_ETH_GETTXSTATS, ETH_E_INV_POINTER,E_NOT_OK);

    Result_o = rba_Eth_GetTxStats( CtrlIdx_u8, TxStats_pst);

    return Result_o;

}
#endif
#endif

#if defined(ETH_GET_TX_ERROR_COUNTER_API)
#if (ETH_GET_TX_ERROR_COUNTER_API == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH91006: Reads the list of Tx Error counter values from indexed controller
 * \par Service ID 0x1D, Synchronous, Non Reentrant
 * Via this function the different counters (error, statistic etc.) can be read out.
 *
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param TxErrorCounterValues_pst    List of values according to IETF RFC 1213
 *
 * \return  Std_ReturnType    E_OK: success
 *                            E_NOT_OK: Statistics counter could not be obtained
 *
 ***************************************************************************************************
 */
FUNC(Std_ReturnType, ETH_CODE) Eth_GetTxErrorCounterValues( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                  P2VAR(Eth_TxErrorCounterValuesType, AUTOMATIC, AUTOMATIC) TxErrorCounterValues_pst)
{
    Std_ReturnType    Result_o;
    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETTXERRORCOUNTERVALUES, ETH_E_INV_CTRL_IDX,E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETTXERRORCOUNTERVALUES, ETH_E_NOT_INITIALIZED,E_NOT_OK);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE. */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_GETTXERRORCOUNTERVALUES, ETH_E_INV_MODE, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(TxErrorCounterValues_pst == NULL_PTR, ETH_SID_ETH_GETTXERRORCOUNTERVALUES, ETH_E_INV_POINTER,E_NOT_OK);

    Result_o = rba_Eth_GetTxErrorCounterValues( CtrlIdx_u8, TxErrorCounterValues_pst);

    return Result_o;

}
#endif
#endif
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH077: Provides access to a transmit buffer of the specified controller
 * \par Service ID 0x09, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8   	    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter InOut:
 * \param LenBytePtr_pu16   In: desired length in bytes, out: granted length in bytes
 *
 * Parameter Out:
 * \param BufIdxPtr_pu8 	Index to the granted buffer resource. To be used for subsequent requests
 * \param BufPtr_ppo    	Pointer to the granted buffer
 *
 * \return                  BufReq_ReturnType {BUFREQ_OK: success;
 *                                             BUFREQ_E_NOT_OK: development error detected;
 *                                             BUFREQ_E_BUSY: all buffers in use}
 *
 ***************************************************************************************************
 */
FUNC(BufReq_ReturnType, ETH_CODE) Eth_ProvideTxBuffer( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                       P2VAR(uint8, AUTOMATIC, AUTOMATIC) BufIdxPtr_pu8,
                                                       P2VAR(P2VAR(Eth_DataType, AUTOMATIC, AUTOMATIC), AUTOMATIC, AUTOMATIC) BufPtr_ppo,
                                                       P2VAR(uint16, AUTOMATIC, AUTOMATIC) LenBytePtr_pu16 )
{
    BufReq_ReturnType Result_o;

    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_INV_CTRL_IDX, BUFREQ_E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_NOT_INITIALIZED, BUFREQ_E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(BufIdxPtr_pu8 == NULL_PTR, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_INV_POINTER, BUFREQ_E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(BufPtr_ppo == NULL_PTR, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_INV_POINTER, BUFREQ_E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(LenBytePtr_pu16 == NULL_PTR, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_INV_POINTER, BUFREQ_E_NOT_OK);
    /* Check if provided length is not bigger than total buffer size; check for MTU must be done in EthIf;
     * Maximum Payload for Eth driver is 1504 including 4bytes for VLAN; Double VLAN tag is not supported
     * Eth can not handle frames bigger than 1518 bytes(MTU: 1504; Eth driver header: 14bytes), since Jumbo Frames are not supported */
    ETH_DET_REPORT_RETURN_ERROR(*LenBytePtr_pu16 > ETH_MAXFRAMELENGTH, ETH_SID_ETH_PROVIDETXBUFFER, ETH_E_INV_PARAM, BUFREQ_E_NOT_OK);

    /* MAC must be switched on -> SetControllerMode(ACTIVE) */
    if(Eth_ControllerMode_aen_MP[CtrlIdx_u8] == ETH_MODE_ACTIVE)
    {

        /* TODO: When invoked with length equal to zero, then still buffer is allocated by Eth driver.
         * In such case, driver could return BUFREQ_E_NOT_OK and should not allocate the buffer. */

        /* Allocating 14 extra Eth header bytes */
        *LenBytePtr_pu16 = *LenBytePtr_pu16 + ETH_SRC_DST_FTYPE_LEN;

        Result_o = Eth_TxBuffer_AllocateBuffer(CtrlIdx_u8, BufIdxPtr_pu8, BufPtr_ppo, LenBytePtr_pu16);

        /* The content of LenBytePtr_pu16 is Valid only when the buffer return status is either BUFREQ_OK or BUFREQ_E_OVFL  */
        if ((Result_o == BUFREQ_OK) || (Result_o == BUFREQ_E_OVFL))
        {
            /* Excluding the Eth header length from the Total length*/
            *LenBytePtr_pu16 = *LenBytePtr_pu16 - ETH_SRC_DST_FTYPE_LEN;

            /* Adding the Eth header length and making the pointer to point the position of payload in Eth frame */
            *BufPtr_ppo += ETH_SRC_DST_FTYPE_LEN;

        }
        else
        {
            /* no touch the len if err */
        }
    }
    else
    {
        Result_o = BUFREQ_E_NOT_OK;
    }

    return(Result_o);
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH087: Triggers transmission of a previously filled transmit buffer or with LenByte_u16==0
 * releases a previously locked buffer.
 * \par Service ID 0xA, Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8       	Index of the controller within the context of the Ethernet Driver
 * \param BufIdx_u8        	Index of the buffer resource
 * \param FrameType_o     	Ethernet frame type; This value is already set by EthIf and thus unused
 *                          in this function.
 * \param TxConfirmation_b  Activates transmission confirmation
 * \param LenByte_u16       Data length in byte
 * \param PhysAddrPtr_pu8   Physical target address (MAC address) in network byte order
 *
 * \return                  Std_ReturnType {E_OK: success;
 *                                          E_NOT_OK: transmission failed}
 *
 ***************************************************************************************************
 */
/* MR12 RULE 8.13 VIOLATION: Pointer to const is not used as prototype is specified by AUTOSAR */
FUNC(Std_ReturnType, ETH_CODE) Eth_Transmit( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                             VAR(uint8, AUTOMATIC) BufIdx_u8,
                                             VAR(Eth_FrameType, AUTOMATIC) FrameType_o,
                                             VAR(boolean, AUTOMATIC) TxConfirmation_b,
                                             VAR(uint16, AUTOMATIC) LenByte_u16,
                                             P2VAR(uint8, AUTOMATIC, AUTOMATIC) PhysAddrPtr_pu8 )
{
	Eth_ControllerRefType_t                         Controller_pst;               /* local pointer to global management structure per Eth controller */
	Eth_TransmitBufferDescriptorQueueRefType_t      TransmitDescriptorQueue_pst;  /* local pointer to management structure for TX descriptor buffer elements */
	Eth_TxBuffer_TransmitBufferQueueRefType_t       TransmitBufferQueue_pst;      /* local pointer to management structure for TX buffer */
	P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)         FramePtr_pu8;                 /* local pointer to the beginning of the Ethernet frame (destination MAC) which shall be transmitted */
	P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)         TargetMacAddressPtr_pu8;      /* local pointer to the remote/destination MAC address for creating the ETH frame content */
	P2VAR(uint8, AUTOMATIC, ETH_DMA_BUFFER)         SourceMacAddressPtr_pu8;      /* local pointer to the local MAC address for creating the ETH frame content */
	uint32                                          FrameIndex_u32;               /* local variable which is used to access the local frame byte wise */
	uint16                                          ActualLenByte_u16;            /* Length including Ethernet header */
	uint8                                           DescriptorIndex_u8;
    Std_ReturnType                                  Result_o;                     /* in this function NOT_OK not returned, since error check is made in the generic Eth_Transmit()*/
    boolean                                         BufferLocked_b;
    Eth_TxBuffer_TransmitBufferStateType_ten        StateTableVal;                /* local variable holding the state of the indexed buffer */


    /* DET checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_TRANSMIT, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_TRANSMIT, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if BufIdx_u8 is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(BufIdx_u8 >= Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthTxBufTotal_u8, ETH_SID_ETH_TRANSMIT, ETH_E_INV_PARAM, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(PhysAddrPtr_pu8 == NULL_PTR, ETH_SID_ETH_TRANSMIT, ETH_E_INV_POINTER, E_NOT_OK);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_TRANSMIT, ETH_E_INV_MODE, E_NOT_OK);
    /* Check if provided length is not bigger than total buffer size; check for MTU must be done in EthIf;
     * Maximum Payload for Eth driver is 1504 including 4bytes for VLAN; Double VLAN tag is not supported
     * Eth can not handle frames bigger than 1518 bytes(MTU: 1504; Eth driver header: 14bytes), since Jumbo Frames are not supported */
    ETH_DET_REPORT_RETURN_ERROR(LenByte_u16  > ETH_MAXFRAMELENGTH, ETH_SID_ETH_TRANSMIT, ETH_E_INV_PARAM, E_NOT_OK);
    /* check if frametype is not VLAN and the provided length is 1504bytes; if yes report a DET */
    ETH_DET_REPORT_RETURN_ERROR(((FrameType_o != ETH_VLAN_FRAMETYPE) && (LenByte_u16 > ETH_MAXFRAMELENGTH_WITHOUT_VLAN)), ETH_SID_ETH_TRANSMIT, ETH_E_INV_PARAM, E_NOT_OK);

    TransmitBufferQueue_pst = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];

    /*Atomic Section: Read of global buffer status*/
    SchM_Enter_Eth(CONTROLLER);
    StateTableVal = TransmitBufferQueue_pst->StateTable_pen[BufIdx_u8];
    SchM_Exit_Eth(CONTROLLER);

    if ( (ETH_BUFFER_LOCKED == StateTableVal) ) /* buffer locked? */
    { /* yes */
        BufferLocked_b = TRUE;
    }
    else
    { /* no */
        BufferLocked_b = FALSE;
    }

    /* check if buffer is already locked by a previous Eth_ProvideTxBuffer call */
    ETH_DET_REPORT_RETURN_ERROR((BufferLocked_b == FALSE), ETH_SID_ETH_TRANSMIT, ETH_E_INV_PARAM, E_NOT_OK);

    /* Including the Ethernet Header length */
    ActualLenByte_u16 = LenByte_u16 + ETH_SRC_DST_FTYPE_LEN;

    Controller_pst              = &Eth_Controllers_ast [CtrlIdx_u8];
    TransmitDescriptorQueue_pst = &Controller_pst->TransmitDescriptorQueue_st;

    /* Set the return value to E_NOT_OK */
    Result_o = E_NOT_OK;

    if (LenByte_u16 != 0U) /* transmit or only buffer release requested? */
    { /* transmit requested */

        /*Atomic Section: Guarantees correct functionality when Eth_Transmit is preempted by Eth_Transmit*/
                /*1.Read and write back of current descriptor has to be executed in the same atomic section*/
                /*2.Global buffer status write to be executed in an atomic section*/

        SchM_Enter_Eth(CONTROLLER);
        /* Allocate a transmit DMA descriptor */
        DescriptorIndex_u8 = TransmitDescriptorQueue_pst->CurrentIndex_u8;

        if(TransmitDescriptorQueue_pst->CurrentIndex_u8 >= TransmitDescriptorQueue_pst->LastIndex_u8)
        {
            TransmitDescriptorQueue_pst->CurrentIndex_u8 = 0U;
        }
        else
        {
            TransmitDescriptorQueue_pst->CurrentIndex_u8 += 1U;
        }

        /* Update the buffer state to ETH_BUFFER_LINKED */
        TransmitBufferQueue_pst->StateTable_pen[BufIdx_u8] = ETH_BUFFER_LINKED;
        /* Update TxDescriptor to Buffer Link table with the linked Buffer Index*/
        TransmitDescriptorQueue_pst->TxDescToBufferLinkTable_pu8[DescriptorIndex_u8] = BufIdx_u8;
        /* Update Tx Buffer to Desc Link table with the linked Descriptor Index*/
        TransmitBufferQueue_pst->TxBufferToDescLinkTable_pu8[BufIdx_u8] = DescriptorIndex_u8;

        SchM_Exit_Eth(CONTROLLER);

        /* get pointer to TX Buffer Queue element from provided BufIdx_u8 */
        FramePtr_pu8            = Eth_TxBuffer_DetermineBufferAddress(CtrlIdx_u8, BufIdx_u8);

        /* copy target and source MAC address into frame */
        TargetMacAddressPtr_pu8 = &FramePtr_pu8[ETH_TARGET_MAC_ADDRESS_OFFSET];
        SourceMacAddressPtr_pu8 = &FramePtr_pu8[ETH_SOURCE_MAC_ADDRESS_OFFSET];
        for ( FrameIndex_u32 = 0U; FrameIndex_u32 < ETH_MAC_ADDRESS_LENGTH_BYTE; FrameIndex_u32++ )
        {
            TargetMacAddressPtr_pu8[FrameIndex_u32] = PhysAddrPtr_pu8[FrameIndex_u32];
            SourceMacAddressPtr_pu8[FrameIndex_u32] = Eth_CtrlPhysAddress_au8_MP[CtrlIdx_u8][FrameIndex_u32];
        }

        /* Update the FrameType in the Transmit Frame */
        FramePtr_pu8[ETH_FRAMETYPE_BYTE1_OFFSET] = (uint8)((FrameType_o & ETH_FRAMETYPE_UPPER_MASK) >> ETH_FRAMETYPE_SHIFT);
        FramePtr_pu8[ETH_FRAMETYPE_BYTE2_OFFSET] = (uint8) (FrameType_o & ETH_FRAMETYPE_LOWER_MASK);

        SchM_Enter_Eth(CONTROLLER);

        /*Program the TX descriptor*/
        rba_Eth_ProgramTxDescriptor(CtrlIdx_u8, DescriptorIndex_u8, ActualLenByte_u16, FramePtr_pu8);

        /* Count of Tx descriptors(frames) to be freed in Eth_TxConfirmation */
        TransmitDescriptorQueue_pst->TxDescrToBeFreedCnt_u32++;

        /* store tx confirmation flag */
        TransmitDescriptorQueue_pst->TxConfFlagsTable_pb[DescriptorIndex_u8] = TxConfirmation_b;

        SchM_Exit_Eth(CONTROLLER);


        Result_o = E_OK;
    }
    else
    { /* only buffer release requested */
        /* release buffer */
        /*Atomic Section: Read of global buffer status*/
        SchM_Enter_Eth(CONTROLLER);
        TransmitBufferQueue_pst->StateTable_pen[BufIdx_u8] = ETH_BUFFER_FREE;
        SchM_Exit_Eth(CONTROLLER);

        Result_o = E_OK;
    }

    return(Result_o);
}
#if defined(ETH_EN_RX_POLLING)
 #if (ETH_EN_RX_POLLING == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH095: Triggers frame reception.
 *
 * SWS_Eth_00096:
 * The function shall read the next frame from the receive buffers. The function passes the received
 * frame to the Ethernet interface using the callback function EthIf_RxIndication and indicates if
 * there are more frames in the receive buffers.
 * -> This is currently not the case. In one function call of Eth_Receive, all RX frames which are
 *    present at function entry, are processed. Thus EthIf_RxIndication might be called multiple times.
 * \par Service ID 0xB, Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param RxStatusPtr   Indicates whether a frame has been received and if so, whether more frame
 *                      are available.
 *
 * \return              None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_Receive( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                  P2VAR(Eth_RxStatusType, AUTOMATIC, AUTOMATIC) RxStatusPtr )
{

	/* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(RxStatusPtr == NULL_PTR, ETH_SID_ETH_RECEIVE, ETH_E_INV_POINTER);

    *RxStatusPtr = ETH_NOT_RECEIVED;

    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_RECEIVE, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_RECEIVE, ETH_E_NOT_INITIALIZED);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_RECEIVE, ETH_E_INV_MODE);

    rba_Eth_Receive(CtrlIdx_u8, RxStatusPtr);

    return;
}
 #endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH100: Triggers frame transmission confirmation
 * This function checks if frames have been sent and if EthIf_TxConfirmation shall be done and makes
 * it accordingly.
 * \par Service ID 0xC, Synchronous, Non Reentrant for same controller, but reentrant for different controllers
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * \return              None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxConfirmation( VAR(uint8, AUTOMATIC) CtrlIdx_u8 )
{
	Eth_ControllerRefType_t                         Controller_pst;               /* local pointer to global management structure per Eth controller */
	Eth_TransmitBufferDescriptorQueueRefType_t      TransmitDescriptorQueue_pst;  /* local pointer to management structure for TX descriptor buffer elements */
	uint32                                          TxBuffersToBeFreed_u32;       /* local variable holding number of buffers to be freed in this call of TxConfirmation*/
    uint32                                          TxBuffersFreedThisCycle_u32;  /* local variable holding number of buffers already freed in this call of TxConfirmation*/
	uint8                                           BufferIndex_u8;
	Eth_DataRefType_t 								Buffer_po;                    /* Address of the frame for which tx confirmation is to be given*/
	boolean                                         TxDone_b;
	Eth_TxBuffer_TransmitBufferQueueRefType_t       TransmitBufferQueue_pst;      /* local pointer to management structure for TX buffer */

	/* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_TXCONFIRMATION, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_TXCONFIRMATION, ETH_E_NOT_INITIALIZED);
    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerMode_aen_MP[CtrlIdx_u8] != ETH_MODE_ACTIVE, ETH_SID_ETH_TXCONFIRMATION, ETH_E_INV_MODE);

    Controller_pst              = &Eth_Controllers_ast [CtrlIdx_u8];
    TransmitDescriptorQueue_pst = &Controller_pst->TransmitDescriptorQueue_st;
    TransmitBufferQueue_pst     = &Eth_TxBuffer_TransmitBufferQueue_ast[CtrlIdx_u8];
    TxBuffersFreedThisCycle_u32 = 0;

    /*Atomic Section: Guarantees correct functionality if Eth_Transmit(triggered through Eth_RxIndication interrupt) interrupts Eth_TxConfirmation*/
    SchM_Enter_Eth(CONTROLLER);
    TxBuffersToBeFreed_u32 = TransmitDescriptorQueue_pst->TxDescrToBeFreedCnt_u32;
    SchM_Exit_Eth(CONTROLLER);

    /* check all buffers between confirmation low (last time checked) and high water mark (current index in queue) */
    while(TxBuffersToBeFreed_u32 > 0U)
    {
        TxDone_b = rba_Eth_TxDone(CtrlIdx_u8,TransmitDescriptorQueue_pst->ConfirmationLowWaterMark_u8);

        if (TxDone_b) /* used bit set by GMAC hardware? -> already transmitted? */
        { /* yes */

            /* determine transmit buffer index for tx confirmation call back and buffer release from the corresponding TxBufferAddress of the TxDescriptor */
            /* Here, the BufferIndex_u8 will be retrieved from the TxDescriptortoBuffer Link table which gives the TxBuffer Id being used for Transmission linked
            to the Descriptor*/
            BufferIndex_u8 = TransmitDescriptorQueue_pst->TxDescToBufferLinkTable_pu8[(TransmitDescriptorQueue_pst->ConfirmationLowWaterMark_u8)];
            Buffer_po      = Eth_TxBuffer_DetermineBufferAddress(CtrlIdx_u8, BufferIndex_u8);

            TxBuffersToBeFreed_u32--;
            TxBuffersFreedThisCycle_u32++;

            /* frame ( all buffers) was sent by hardware -> call EthIf call back function and clear locked and confirmation flag */

            if (TransmitDescriptorQueue_pst->TxConfFlagsTable_pb[TransmitDescriptorQueue_pst->ConfirmationLowWaterMark_u8] == TRUE) /* transmit confirmation requested? */
            {
                EthIf_TxConfirmation(CtrlIdx_u8, BufferIndex_u8,(Eth_DataRefType_t)Buffer_po,0);

                /* clear TxConfFlag */
                TransmitDescriptorQueue_pst->TxConfFlagsTable_pb[TransmitDescriptorQueue_pst->ConfirmationLowWaterMark_u8] = FALSE;

            }
            else
            {
                /* intentionally left blank to avoid MISRA warning */

            }

            /* clear TxConfFlag */
            /* Buffer to be always unlinked after EthIf_TxConfirmation if TxConfirmation was requested. This ensures correct functionality if Eth_ProvideTxBuffer
             * Preempted TxConfirmation*/
            /* Unlink the buffers  */
            SchM_Enter_Eth(CONTROLLER);
            TransmitBufferQueue_pst->StateTable_pen[BufferIndex_u8] = ETH_BUFFER_FREE;
            SchM_Exit_Eth(CONTROLLER);

            /* No atomic section required as Confirmation watermark is only used in TxConfirmation and TxConfirmation cannot preempt itself */
            Eth_TxBuffer_AdvanceQueueIndex(&TransmitDescriptorQueue_pst->ConfirmationLowWaterMark_u8, TransmitDescriptorQueue_pst->LastIndex_u8);
        }
        else /* Frame in TxBuffer element was not transmitted */
        {
            /* stop searching here as confirmation was requested but buffer was not transmitted yet */
            break;
        }
    }

    /*Atomic Section: Guarantees correct functionality if Eth_Transmit(triggered through Eth_RxIndication interrupt) interrupts Eth_TxConfirmation*/
    SchM_Enter_Eth(CONTROLLER);
    TransmitDescriptorQueue_pst->TxDescrToBeFreedCnt_u32 = TransmitDescriptorQueue_pst->TxDescrToBeFreedCnt_u32 - TxBuffersFreedThisCycle_u32;
    SchM_Exit_Eth(CONTROLLER);

    return;
}

/**
 ***************************************************************************************************
 * \moduledescription
 * ETH106: Returns the version information of this module
 * \par Service ID 0xD, Synchronous, Reentrant
 *
 * Parameter In:
 * \param VersionInfoPtr_pst    Version information of this module
 *
 * \return                      None
 *
 ***************************************************************************************************
 */
#if ( ETH_VERSION_INFO_API == STD_ON)
FUNC(void, ETH_CODE) Eth_GetVersionInfo( P2VAR(Std_VersionInfoType, AUTOMATIC, AUTOMATIC) VersionInfoPtr_pst )
{
    /* DET checks */
    /* AR does not specify a return from the function, but it seems to be reasonable */
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(VersionInfoPtr_pst == NULL_PTR, ETH_SID_ETH_GETVERSIONINFO, ETH_E_INV_POINTER);

    VersionInfoPtr_pst->vendorID         = ETH_VENDOR_ID;
    VersionInfoPtr_pst->moduleID         = ETH_MODULE_ID;
    VersionInfoPtr_pst->sw_major_version = ETH_SW_MAJOR_VERSION;
    VersionInfoPtr_pst->sw_minor_version = ETH_SW_MINOR_VERSION;
    VersionInfoPtr_pst->sw_patch_version = ETH_SW_PATCH_VERSION;

    return;
}
#endif

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * This API would store the EventStatus for the corresponding EventId(If ETH_DSM_RE_INIT_SUPPORT == STD_ON)
 *
 * \par Service ID 0xFD, Synchronous, Non-Reentrant
 *
 * Parameter In:
 * \param EventId       Dem event id for which DEM needs to be reported
 * \param EventStatus   Event status that needs to be logged in DEM
 *
 * \return                      None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_DemReportErrorStatus( Dem_EventIdType EventId, Dem_EventStatusType EventStatus)
{
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
     uint8 EventIndex_u8 = 0;
#endif
    /* Check if Event is configured
     * - If configured, a valid EventID(1 to 65535) will be generated by DEM and via Symbolic reference linked
     *   in our Configset
     * - If not configured, then 0U will be generated instead of Symbolic reference in our ConfigSet */
    if(EventId > 0U)
    {
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
        /* For every event there will be one Index starting from zero(ETH_<DEM event name>) from Eth component,
         * And other one which is the DemEvent Id itself(Taken as symbolic name)*/
        /* This loop iterates through all the Dem events
         * Eth_DemEventIdReInitLink would contain DemEventId for each Eth Dem event Index(ETH_<DEM event name>)
         * If any of them matches
         * The status is stored in DemEventStatus */
        for(EventIndex_u8 = 0; EventIndex_u8 < ETH_TOTAL_DEM_EVENTS; EventIndex_u8++)
        {
            if(EventId == Eth_DemEventIdReInitLink_acu16_MP[EventIndex_u8])
            {
                /* Critical section as global variables are updated */
                SchM_Enter_Eth(CONTROLLER);

                /* Store the status */
                Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].DemEventStatus = EventStatus;

                SchM_Exit_Eth(CONTROLLER);
                break;
            }
            else
            {
                /* Under normal execution, this else will never be entered. */
                /* Do nothing but continue */
            }
        }
#endif
        /* Call Dem_ReportErrorStatus by default */
        Dem_ReportErrorStatus(EventId, EventStatus);
    }
    else
    {
        /* Do Nothing */
    }

}
#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * This function periodically checks for Rereporting of Dem events and reports error status to DEM.
 * \par Service ID 0x20, Synchronous, Non-Reentrant
 *
 * Parameter In:
 * \param                       None
 *
 * \return                      None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_MainFunction( void )
{
    /* Local variable declaration */
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
    uint8                                   CtrlIdx_u8;                     /* Local variable used for indexing controller */
    Eth_DemErrorCountHandleRefType_t        DemCounterHandleCurrent_pst;    /* Local pointer to get the current counters value from rba_Eth */
    Eth_DemErrorCountHandle_tst             DemCounterHandleCurrent_st;     /* Local variable to get the current counters value from rba_Eth */
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
    uint8                                   EventIndex_u8;                  /* Local variable used for indexing the events */
#endif
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
    Eth_DemErrorCountHandleRefType_t        DemCounterHandlePast_pst;       /* Local pointer used to point to gloabl variable which would contain counter values */
#endif
#endif
    /* Initialize the pointer */
    DemCounterHandleCurrent_pst     = &DemCounterHandleCurrent_st;
#endif
    /* MainFunction expects a previous call of Eth_Init to be done before it execution
     * But why Eth_Init?
     * If Controller is in mode down state but DEM calls for ReReporting
     * controller wont be in STATE_ACTIVE because of which re reporting wont be possible */

    /* Check for Init is done for all controllers by null pointer check for Eth_CurrentConfig_pco
     * As it would be initialized in Eth_Init*/
    if(Eth_CurrentConfig_pco != NULL_PTR)
    {
#if (ETH_DSM_RE_INIT_SUPPORT == STD_ON)
        /* If DSM ReInit is enabled
         * Loop through all the event structures if for any event Re-Reporting was requested */
        for(EventIndex_u8 = 0; EventIndex_u8 < ETH_TOTAL_DEM_EVENTS; EventIndex_u8++)
        {
            /* If Re-Reporting was requested
             * Call Dem_ReportErrorStatus with the status that was stored */
            if(TRUE == Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].ReReportRequire_b)
            {
                /* Call Dem report Error */
                Dem_ReportErrorStatus(Eth_DemEventIdReInitLink_acu16_MP[EventIndex_u8], Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].DemEventStatus);
                /* Critical section as global variables are updated */
                SchM_Enter_Eth(CONTROLLER);

                /* Reset the re-reporting flag */
                Eth_DSMReInit_DemEventsHandler_ast[EventIndex_u8].ReReportRequire_b = FALSE;

                SchM_Exit_Eth(CONTROLLER);
            }
            else
            {
                /* Do Nothing */
            }
        }
#endif
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
        /* For each controller, Get the counter values and check for Errors.
         * If they have fail the check then report DEM Error for the configured event ID */
        for(CtrlIdx_u8 = 0; CtrlIdx_u8 < ETH_MAX_CTRLS_SUPPORTED; CtrlIdx_u8++)
        {
            /* Check if the controller Mode is active */
            if(ETH_MODE_ACTIVE == Eth_ControllerMode_aen_MP[CtrlIdx_u8])
            {
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                /* Get previous counter values */
                DemCounterHandlePast_pst = &Eth_DemErrorCountHandle_ast[CtrlIdx_u8];
#endif
#endif
                /* Get the counter values from rba_Eth */
                rba_Eth_GetDemErrorCounters(CtrlIdx_u8,DemCounterHandleCurrent_pst);

                /* Check if Errors are found, IF call Dem error */
                /* Alignment Error Check */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_AlignmentError_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_AlignmentError_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEAlignment_u16);

                /* CRC Error Check */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_CrcErrors_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_CrcErrors_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthECRC_u16);

                /* Frames with LateCollision while transmission */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_LateCollisionError_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_LateCollisionError_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthELateCollision_u16);

                /* Frames with Multiple Collision while transmission */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_MultipleCollisionError_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_MultipleCollisionError_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEMultipleCollision_u16);

                /* Oversize frames check */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_OverSizeFrames_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_OverSizeFrames_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEOverSize_u16);

                /* Check for Packets Lost while reception */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_RxPacketsLost_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_RxPacketsLost_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthERxFramesLost_u16);

                /* Frames with Single Collision while transmission */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_SingleCollisionError_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_SingleCollisionError_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthESingleCollision_u16);

                /* Undersize frames check  */
                Eth_ExtendedProductionErrorsHandle(DemCounterHandleCurrent_pst->Eth_UnderSizeFrames_u32,
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                DemCounterHandlePast_pst->Eth_UnderSizeFrames_u32,
#endif
#endif
                Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEUnderSize_u16);

#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                /* Critical section as Global variables are read */
                SchM_Enter_Eth(CONTROLLER);
                /* Store the current values */
                DemCounterHandlePast_pst->Eth_AlignmentError_u32            = DemCounterHandleCurrent_pst->Eth_AlignmentError_u32;
                DemCounterHandlePast_pst->Eth_CrcErrors_u32                 = DemCounterHandleCurrent_pst->Eth_CrcErrors_u32;
                DemCounterHandlePast_pst->Eth_LateCollisionError_u32        = DemCounterHandleCurrent_pst->Eth_LateCollisionError_u32;
                DemCounterHandlePast_pst->Eth_MultipleCollisionError_u32    = DemCounterHandleCurrent_pst->Eth_MultipleCollisionError_u32;
                DemCounterHandlePast_pst->Eth_OverSizeFrames_u32            = DemCounterHandleCurrent_pst->Eth_OverSizeFrames_u32;
                DemCounterHandlePast_pst->Eth_RxPacketsLost_u32             = DemCounterHandleCurrent_pst->Eth_RxPacketsLost_u32;
                DemCounterHandlePast_pst->Eth_SingleCollisionError_u32      = DemCounterHandleCurrent_pst->Eth_SingleCollisionError_u32;
                DemCounterHandlePast_pst->Eth_UnderSizeFrames_u32           = DemCounterHandleCurrent_pst->Eth_UnderSizeFrames_u32;

                SchM_Exit_Eth(CONTROLLER);
#endif
#endif
            }
        }
#endif
    }

    return;
}

#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#define ETH_START_SEC_CODE_FAST
#include "Eth_MemMap.h"
#if defined(ETH_EN_RX_INTERRUPT_0)
 #if (ETH_EN_RX_INTERRUPT_0 == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH109: Handles frame reception interrupts of the indexed controller
 * \par Service ID 0x10, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_RxIrqHdlr_0( void )
{
    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[0] != ETH_STATE_ACTIVE, ETH_SID_ETH_RXIRQHDLR, ETH_E_NOT_INITIALIZED);

    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerMode_aen_MP[0] != ETH_MODE_ACTIVE, ETH_SID_ETH_RXIRQHDLR, ETH_E_INV_MODE);

    /* Call rba_Eth_RxIrqHdlr function */
    rba_Eth_RxIrqHdlr(0U);

    return;
}
 #endif
#endif

#if (ETH_MAX_CTRLS_SUPPORTED == 2)
#if defined(ETH_EN_RX_INTERRUPT_1)
 #if (ETH_EN_RX_INTERRUPT_1 == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH109: Handles frame reception interrupts of the indexed controller
 * \par Service ID 0x10, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_RxIrqHdlr_1( void )
{
    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[1] != ETH_STATE_ACTIVE, ETH_SID_ETH_RXIRQHDLR, ETH_E_NOT_INITIALIZED);

    /* check if Eth_SetControllerMode wasn't called previously -> ETH_MODE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerMode_aen_MP[1] != ETH_MODE_ACTIVE, ETH_SID_ETH_RECEIVE, ETH_E_INV_MODE);

    /* Call rba_Eth_RxIrqHdlr function */
    rba_Eth_RxIrqHdlr(1U);

    return;
}
 #endif
#endif
#endif

#if defined(ETH_EN_TX_INTERRUPT_0)
 #if (ETH_EN_TX_INTERRUPT_0 == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH114: Handles frame transmission interrupts of the indexed controller
 * \par Service ID 0x11, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxIrqHdlr_0( void )
{
    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[0] != ETH_STATE_ACTIVE, ETH_SID_ETH_TXIRQHDLR, ETH_E_NOT_INITIALIZED);

    /* Call rba_Eth_TxIrqHdlr function */
    rba_Eth_TxIrqHdlr(0U);

    return;
}
 #endif
#endif

#if (ETH_MAX_CTRLS_SUPPORTED == 2)
#if defined(ETH_EN_TX_INTERRUPT_1)
 #if (ETH_EN_TX_INTERRUPT_1 == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * ETH114: Handles frame transmission interrupts of the indexed controller
 * \par Service ID 0x11, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_TxIrqHdlr_1( void )
{
    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[1] != ETH_STATE_ACTIVE, ETH_SID_ETH_TXIRQHDLR, ETH_E_NOT_INITIALIZED);

    /* Call rba_Eth_TxIrqHdlr function */
    rba_Eth_TxIrqHdlr(1U);

    return;
}
 #endif
#endif
#endif

/**
 ***************************************************************************************************
 * \moduledescription
 * Handles transciever and error interrupts of the indexed controller
 * \par none
 * non-autosar, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_CombinedIrqHdlr_0( void )
{

    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[0] != ETH_STATE_ACTIVE, ETH_SID_ETH_COMBINEDIRQHDLR, ETH_E_NOT_INITIALIZED);

    rba_Eth_CtrlErrIrqHdlr(0U);

    return;
}

#if (ETH_MAX_CTRLS_SUPPORTED == 2)
/**
 ***************************************************************************************************
 * \moduledescription
 * Handles transciever and error interrupts of the indexed controller
 * \par none
 * non-autosar, Synchronous, Non Reentrant
 *
 * \return    None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_CombinedIrqHdlr_1( void )
{

    /* DET checks */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[1] != ETH_STATE_ACTIVE, ETH_SID_ETH_COMBINEDIRQHDLR, ETH_E_NOT_INITIALIZED);

    rba_Eth_CtrlErrIrqHdlr(1U);

    return;
}
#endif

#define ETH_STOP_SEC_CODE_FAST
#include "Eth_MemMap.h"

#define ETH_START_SEC_CODE
#include "Eth_MemMap.h"
/**
 ***************************************************************************************************
 * Eth_CheckBitFieldStatus - This functions will check the status of bit field and provides a wait
 *                           time of one frame length.Returns TRUE if successful else FALSE.
 *                           Report to DEM error if it is FALSE.
 *                           It waits as long as the (*reg_pu32 & check_mask_u32) != desired_val_u32
 *
 * \param      Reg_pu32            - Register which holds the corresponding bit
 * \param      MaskVal_u32         - Mask value
 * \param      ChkVal_u32          - Value to be checked against after masking.
 * \param      TimeoutCnt_u32      - How many cycles to wait.
 * \param      CtrlIdx_u8          - Controller Index
 * \param      ApiId_u8            - ApiId for DET reporting
 * \return     boolean
 * \retval     TRUE  if bit equals ChkVal_u32 before the timeout occured.
 *             FALSE if timeout happened and DEM was called
 * \usedresources
 ***************************************************************************************************
*/
FUNC(boolean, ETH_CODE) Eth_CheckBitFieldStatus(const volatile uint32 *Reg_pu32, uint32 MaskVal_u32,
        uint32 ChkVal_u32, uint32 TimeoutCnt_u32, uint8 CtrlIdx_u8)
{
    Eth_TickType timeCpu;
    boolean Status_b = FALSE;

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
    Dem_EventStatusType DemStatus_b = DEM_EVENT_STATUS_FAILED;
#else
    Std_ReturnType ReturnStatus_o = E_NOT_OK;
#endif
#endif

    Eth_Cfg_TimerStartusec(&timeCpu);
    do
    {
        if (((*Reg_pu32) & MaskVal_u32) == ChkVal_u32)
        {
            Status_b = TRUE;
#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
            DemStatus_b = DEM_EVENT_STATUS_PASSED;
#else
            ReturnStatus_o = E_OK;
#endif
#endif
            /* CDG-SMT Coding Guidelines for BSW r1.1 Rule 1-6f: For any iteration statement there shall be at most one break statement used for loop termination. */
            break;
        }
        else
        {
            /* intentionally left blank */
        }
        /* MR12 RULE 13.3 VIOLATION: This warning can be removed by mapping the macro to the OS timer functions */
    } while (!(Eth_Cfg_TimerElapsed(&timeCpu, TimeoutCnt_u32)));

    if (Status_b != TRUE)
    {
        if (((*Reg_pu32) & MaskVal_u32) == ChkVal_u32)
        {
            /* Report production Error */
            Status_b = TRUE;
#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
            DemStatus_b = DEM_EVENT_STATUS_PASSED;
#else
            ReturnStatus_o = E_OK;
#endif
#endif
        }
        else
        {
            /* intentionally left blank */
        }
    }
    else
    {
    }

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)

    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEAccess_u16, DemStatus_b);

#else
    Eth_CurrentConfig_pco->EthDemEvents_st.EthEAccessErrorCallBack(CtrlIdx_u8,ReturnStatus_o);
#endif
#endif
    return Status_b;
}

#if defined(ETH_DEM_REPORTING_SUPPORT)
#if (ETH_DEM_REPORTING_SUPPORT == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Initializes all the structure members of the varibale which stores counters for Dem Handling
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CtrlIdx_u8    Index of the controller within the context of the Ethernet Driver
 *
 * Parameter Out:
 * \param None
 *
 * \return None
 *
 ***************************************************************************************************
 */
static FUNC(void, ETH_CODE) Eth_DemReportInit(uint8 CtrlIdx_u8)
{
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
    Eth_DemErrorCountHandleRefType_t  DemCounterHandle_pst;  /* Local variable used to access Dem Error counter global variable */

    /* Get the address of the global variable */
    DemCounterHandle_pst = &Eth_DemErrorCountHandle_ast[CtrlIdx_u8];

    /* Critical section as Global variables are read */
    SchM_Enter_Eth(CONTROLLER);
    /* Reset the counters to Zero */
    DemCounterHandle_pst->Eth_AlignmentError_u32         = 0;
    DemCounterHandle_pst->Eth_CrcErrors_u32              = 0;
    DemCounterHandle_pst->Eth_LateCollisionError_u32     = 0;
    DemCounterHandle_pst->Eth_MultipleCollisionError_u32 = 0;
    DemCounterHandle_pst->Eth_OverSizeFrames_u32         = 0;
    DemCounterHandle_pst->Eth_RxPacketsLost_u32          = 0;
    DemCounterHandle_pst->Eth_SingleCollisionError_u32   = 0;
    DemCounterHandle_pst->Eth_UnderSizeFrames_u32        = 0;

    SchM_Exit_Eth(CONTROLLER);
#endif
#endif
    /* Report EVENT Status as PASSED for all the Dem events except E_E_ACCESS */
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEAlignment_u16, DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthECRC_u16,       DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEOverSize_u16,  DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEUnderSize_u16, DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthERxFramesLost_u16,      DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthELateCollision_u16,     DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthEMultipleCollision_u16, DEM_EVENT_STATUS_PASSED);
    Eth_DemReportErrorStatus(Eth_CurrentCtrlConfig_apco[CtrlIdx_u8]->EthDemEvents_st.EthESingleCollision_u16,   DEM_EVENT_STATUS_PASSED);

}


/**
 ***************************************************************************************************
 * \moduledescription
 * Checks if a there is any change in counter values and reports to Dem with the event Id  provided
 * \par Synchronous, Non Reentrant
 *
 * Parameter In:
 * \param CurrentCounterValue_u32   Current counter value updated by the hardware
 * \param PreviousCounterValue_u32  Counter value previously stored (Used only if ETH_COUNTER_DIFFERENCE is STD_ON)
 * \param ErrorIdValue_u16          Event ID that would be used to report error status to dem
 *
 * Parameter Out:
 * \param None
 *
 * \return None
 *
 ***************************************************************************************************
 */
static FUNC(void, ETH_CODE) Eth_ExtendedProductionErrorsHandle(uint32 CurrentCounterValue_u32
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
                                                        ,uint32 PreviousCounterValue_u32
#endif
#endif
                                                        ,Dem_EventIdType ErrorIdValue_u16)
{
#ifdef ETH_COUNTER_DIFFERENCE
#if (ETH_COUNTER_DIFFERENCE == STD_ON)
    /* Check if the counter values changed */
    if(CurrentCounterValue_u32 != PreviousCounterValue_u32)
#else
    if(CurrentCounterValue_u32 > 0)
#endif
#endif
    {
        /* Report dem error for the provided event ID */
        Eth_DemReportErrorStatus(ErrorIdValue_u16, DEM_EVENT_STATUS_FAILED);
    }
    else
    {
        /* Dummy Else */
    }
}
#endif
#endif

#if (ETH_GLOBAL_TIME_SUPPORT == STD_ON)
/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_GetCurrentTime:
 * Returns a time value out of the HW registers according to the capability of the HW.
 * Is the HW resolution is lower than the Eth_TimeStampType resolution resp. range,
 * than an the remaining bits will be filled with 0.
 *
 * \par Service ID 0x16, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
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

FUNC(Std_ReturnType, ETH_CODE) Eth_GetCurrentTime( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                   P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                                   P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    VAR( Std_ReturnType, AUTOMATIC ) Result_o;

    /* DET Checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETCURRENTTIME, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETCURRENTTIME, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(timeQualPtr == NULL_PTR, ETH_SID_ETH_GETCURRENTTIME, ETH_E_INV_POINTER, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(timeStampPtr == NULL_PTR, ETH_SID_ETH_GETCURRENTTIME, ETH_E_INV_POINTER, E_NOT_OK);

    Result_o = rba_Eth_GetCurrentTime(CtrlIdx_u8,timeQualPtr,timeStampPtr);

    return (Result_o);

}

/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_EnableEgressTimeStamp:
 * Activates egress time stamping on a dedicated message object.
 * Some HW does store once the egress time stamp marker and some HW needs it always before transmission.
 * There will be no disable functionality, due to the fact, that the message type is always time stamped
 * by network design.
 *
 * \par Service ID 0x17, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param BufIdx_u8            Index of the message buffer, where Application expects egress time stamping

 *
 ***************************************************************************************************
 */

FUNC(void, ETH_CODE) Eth_EnableEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                VAR(uint8, AUTOMATIC) BufIdx_u8 )
{

    /* DET Checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_ENABLEEGRESSTIMESTAMP, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_ENABLEEGRESSTIMESTAMP, ETH_E_NOT_INITIALIZED);


    rba_Eth_EnableEgressTimeStamp(CtrlIdx_u8,BufIdx_u8);

}

/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_GetEgressTimeStamp:
 * Reads back the egress time stamp on a dedicated message object.
 * It must be called within the TxConfirmation() function.
 *
 * \par Service ID 0x18, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
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

FUNC(void, ETH_CODE) Eth_GetEgressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                             VAR(uint8, AUTOMATIC) BufIdx_u8,
                                             P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                             P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    /* DET Checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETEGRESSTIMESTAMP, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETEGRESSTIMESTAMP, ETH_E_NOT_INITIALIZED);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(timeQualPtr == NULL_PTR, ETH_SID_ETH_GETEGRESSTIMESTAMP, ETH_E_INV_POINTER);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(timeStampPtr == NULL_PTR, ETH_SID_ETH_GETEGRESSTIMESTAMP, ETH_E_INV_POINTER);

    rba_Eth_GetEgressTimeStamp(CtrlIdx_u8, BufIdx_u8, timeQualPtr, timeStampPtr);

}

/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_GetIngressTimeStamp:
 * Reads back the ingress time stamp on a dedicated message object.
 * It must be called within the RxIndication() function.
 *
 * \par Service ID 0x19, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
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

FUNC(void, ETH_CODE) Eth_GetIngressTimeStamp( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                              P2CONST(Eth_DataType, AUTOMATIC, AUTOMATIC) DataPtr,
                                              P2VAR( Eth_TimeStampQualType, AUTOMATIC, AUTOMATIC ) timeQualPtr,
                                              P2VAR(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    /* DET Checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_GETINGRESSTIMESTAMP, ETH_E_INV_CTRL_IDX);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_GETINGRESSTIMESTAMP, ETH_E_NOT_INITIALIZED);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(DataPtr == NULL_PTR, ETH_SID_ETH_GETINGRESSTIMESTAMP, ETH_E_INV_POINTER);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(timeQualPtr == NULL_PTR, ETH_SID_ETH_GETINGRESSTIMESTAMP, ETH_E_INV_POINTER);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_NOERROR(timeStampPtr == NULL_PTR, ETH_SID_ETH_GETINGRESSTIMESTAMP, ETH_E_INV_POINTER);

    rba_Eth_GetIngressTimeStamp(CtrlIdx_u8, DataPtr, timeQualPtr, timeStampPtr);

}

/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_SetCorrectionTime:
 * Allows the Time Slave to adjust the local ETH Reference clock in HW.
 *
 * \par Service ID 0x1a, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param timeOffsetPtr     offset between time stamp grandmaster and time stamp by local clock: (OriginTimeStampSync[FUP] minus IngressTimeStampSync) + Pdelay
 * \param rateRatioPtr
 *
 * \return                  none
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_SetCorrectionTime(     VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                P2CONST(Eth_TimeIntDiffType, AUTOMATIC, AUTOMATIC) timeOffsetPtr,
                                                P2CONST(Eth_RateRatioType, AUTOMATIC, AUTOMATIC) rateRatioPtr )
{
    /* DET Checks */
   /* check if CtrlIdx is within the maximum range */
   ETH_DET_REPORT_RETURN_NOERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_SETCORRECTIONTIME, ETH_E_INV_CTRL_IDX);
   /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
   ETH_DET_REPORT_RETURN_NOERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_SETCORRECTIONTIME, ETH_E_NOT_INITIALIZED);
   /* check if provided pointer is a NULL pointer*/
   ETH_DET_REPORT_RETURN_NOERROR(timeOffsetPtr == NULL_PTR, ETH_SID_ETH_SETCORRECTIONTIME, ETH_E_INV_POINTER);
   /* check if provided pointer is a NULL pointer*/
   ETH_DET_REPORT_RETURN_NOERROR(rateRatioPtr == NULL_PTR, ETH_SID_ETH_SETCORRECTIONTIME, ETH_E_INV_POINTER);

   rba_Eth_SetCorrectionTime(CtrlIdx_u8, timeOffsetPtr, rateRatioPtr );
}

/**
 ***************************************************************************************************
 * \moduledescription
 * Eth_SetGlobalTime:
 * Allows the Time Master to adjust the global ETH Reference clock in HW.
 * We can use this method to set a global time base on ETH in general or to synchronize the global
 * ETH time base with another time base, e.g. FlexRay.
 *
 * \par Service ID 0x1b, Synchronous, Non-reentrant for the same CtrlIdx, reentrant for different CtrlIdx
 *
 * Parameter In:
 * \param CtrlIdx_u8        Index of the controller within the context of the Ethernet Driver
 * \param timeStampPtr      new time stamp
 *
 * \return                  Std_ReturnType {E_OK: success; E_NOT_OK: failed to set the new time}
 *
 ***************************************************************************************************
 */

FUNC(Std_ReturnType, ETH_CODE) Eth_SetGlobalTime( VAR(uint8, AUTOMATIC) CtrlIdx_u8,
                                                  P2CONST(Eth_TimeStampType, AUTOMATIC, AUTOMATIC ) timeStampPtr )
{
    VAR( Std_ReturnType, AUTOMATIC ) Result_o;

    /* DET Checks */
    /* check if CtrlIdx is within the maximum range */
    ETH_DET_REPORT_RETURN_ERROR(CtrlIdx_u8 >= ETH_MAX_CTRLS_SUPPORTED, ETH_SID_ETH_SETGLOBALTIME, ETH_E_INV_CTRL_IDX, E_NOT_OK);
    /* check if Eth_ControllerInit wasn't called previously -> ETH_STATE_ACTIVE */
    ETH_DET_REPORT_RETURN_ERROR(Eth_ControllerState_aen_MP[CtrlIdx_u8] != ETH_STATE_ACTIVE, ETH_SID_ETH_SETGLOBALTIME, ETH_E_NOT_INITIALIZED, E_NOT_OK);
    /* check if provided pointer is a NULL pointer*/
    ETH_DET_REPORT_RETURN_ERROR(timeStampPtr == NULL_PTR, ETH_SID_ETH_SETGLOBALTIME, ETH_E_INV_POINTER, E_NOT_OK);

    Result_o = rba_Eth_SetGlobalTime(CtrlIdx_u8, timeStampPtr);

    return (Result_o);
}
#endif
#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#endif /* ETH_CONFIGURED */



/*<>
**********************************************************************************************************************
*
**********************************************************************************************************************
</>*/

