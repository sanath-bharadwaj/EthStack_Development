#include "Eth.h"
#include "Eth_PBcfg.h"
#include "Eth_Prv.h"
 
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
 * Variables
 ***************************************************************************************************
 */
/*
 *  memory for DMA descriptors of all controllers
 *  must reside in non-cached memory!
 *  must align to a 32bit memory boundary!
 */
#define RBA_ETH_START_SEC_VAR_CLEARED_16BYTE
#include "rba_Eth_MemMap.h"
static VAR(uint32, ETH_DMA_DESC) Eth_DmaDescMem_au32_MP[ETH_DMA_DESC_MEM_SIZE];
#define RBA_ETH_STOP_SEC_VAR_CLEARED_16BYTE
#include "rba_Eth_MemMap.h"

/*
 *  memory for DMA buffers of all controllers
 *  must reside in cached memory!
 *  must align to a cache boundary!
 */
#define RBA_ETH_START_SEC_VAR_CLEARED_16BYTE
#include "rba_Eth_MemMap.h"
/* MR12 DIR 1.1 VIOLATION: The number of buffers for DMA are limited to uint16 but each buffer size is in multiples of 64. Max ETH_DMA_BUFFER_MEM_SIZE is equal to max(uint16)* 1536. */
static VAR(uint8, ETH_DMA_BUFFER) Eth_DmaBufferMem_au8_MP[ETH_DMA_BUFFER_MEM_SIZE];
#define RBA_ETH_STOP_SEC_VAR_CLEARED_16BYTE
#include "rba_Eth_MemMap.h"


#define ETH_START_SEC_VAR_CLEARED_BOOLEAN
#include "Eth_MemMap.h"
    /* transmit confirmation flags of all controllers */
    static volatile VAR(boolean, ETH_CFG) Eth_TxConfFlagsTable_ab_MP[7];

#define ETH_STOP_SEC_VAR_CLEARED_BOOLEAN
#include "Eth_MemMap.h"

#define ETH_START_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"
    /* state table with transmit states of all controllers */
    static volatile VAR(Eth_TxBuffer_TransmitBufferStateType_ten, ETH_CFG) Eth_TxBufferStateTable_aen_MP[7];
#define ETH_STOP_SEC_VAR_CLEARED_UNSPECIFIED
#include "Eth_MemMap.h"

#define ETH_START_SEC_VAR_CLEARED_8
#include "Eth_MemMap.h"
       /* Tx Descriptor to  Buffer Link Table of all controllers*/
       static volatile VAR(uint8, ETH_CFG) Eth_TxDescToBufferLinkTable_au8_MP[7];
       /* Tx Buffer to  Descriptor Link Table of all controllers*/
       static volatile VAR(uint8, ETH_CFG) Eth_TxBufferToDescLinkTable_au8_MP[7];

#define ETH_STOP_SEC_VAR_CLEARED_8
#include "Eth_MemMap.h"

#define ETH_START_SEC_CONST_FAST_UNSPECIFIED
#include "Eth_MemMap.h"
    /* config sets */
    CONST(Eth_ConfigType, ETH_CFG) Eth_ConfigSets[ETH_CONFIGTYPE_COUNT] =
    {
        /* config set 0 */
        {
            {
                /* ctrl config 0 */
                {
                    &Eth_PhysAddresses_acu8_MP[0][0], /* phys address (6 byte) of controller */
                    &Eth_DmaDescMem_au32_MP[0], /* receive descriptors */
                    &Eth_DmaDescMem_au32_MP[56], /* transmit descriptors */
                    &Eth_DmaBufferMem_au8_MP[0], /* receive buffers */
                    &Eth_DmaBufferMem_au8_MP[10752], /* transmit buffers */
                    &Eth_TxConfFlagsTable_ab_MP[0], /* TxConfirmation flags */
                    &Eth_TxBufferStateTable_aen_MP[0], /* buffer states */
                    {
                        ETH_E_ACCESS_0, /* Macro mapped to Dem event Id */
                        ETH_E_RXFRAMESLOST_0, /* Macro mapped to Dem event Id */
                        ETH_E_CRC_0, /* Macro mapped to Dem event Id */
                        ETH_E_UNDERSIZE_0, /* Macro mapped to Dem event Id */
                        ETH_E_OVERSIZE_0, /* Macro mapped to Dem event Id */
                        ETH_E_ALIGNMENT_0, /* Macro mapped to Dem event Id */
                        ETH_E_SINGLECOLLISION_0, /* Macro mapped to Dem event Id */
                        ETH_E_MULTIPLECOLLISION_0, /* Macro mapped to Dem event Id */
                        ETH_E_LATECOLLISION_0, /* Macro mapped to Dem event Id */
                    },
                    1536U, /* EthCtrlRxBufLenByte */
                    1536U, /* EthCtrlTxBufLenByte */
                    7U, /* EthRxBufTotal */
                    7U, /* EthTxBufTotal */
                    0U, /* EthCtrlIdx */
                    &Eth_TxDescToBufferLinkTable_au8_MP[0], /* TxDescriptor to Buffer links */
                    &Eth_TxBufferToDescLinkTable_au8_MP[0], /* TxBuffer to Descriptor links */
                },
            },
            1U
        },
    };
#define ETH_STOP_SEC_CONST_FAST_UNSPECIFIED
#include "Eth_MemMap.h"

#define ETH_START_SEC_CONST_8
#include "Eth_MemMap.h"
   
    /* TODO: where will the physical address be configured and stored? */
    CONST(uint8, ETH_CFG) Eth_PhysAddresses_acu8_MP[ETH_MAX_CTRLS_SUPPORTED][ETH_MAC_ADDRESS_LENGTH_BYTE] =
    {
        {
                 0xFCU,
                 0xD6U,
                 0xBDU,
                 0x00U,
                 0x00U,
                 0x01U,                 },
    };
#define ETH_STOP_SEC_CONST_8
#include "Eth_MemMap.h"

/* Specifies the Alti bit value that needs to be configured for Configuration */





#define RBA_ETH_START_SEC_CONST_32
#include "rba_Eth_MemMap.h"
	CONST(uint32, ETH_CONST) rba_Eth_Port_ConfigSets [RBA_ETH_PORTCONFIG_COUNT]=
	{
            (                                  
                    0x00000000UL |   /* MDIO_IN */                                                                        
                    0x00000000UL |   /* MDIO_OUT */                                                	                    
                    0x00000000UL |   /* RXD0 */
                    0x00000000UL |   /* RXD1 */
                    0x00000000UL |   /* RXD2 */
                    0x00000000UL |   /* RXD3 */
                    0x00000004UL |   /* RXCLK */                        
                    0x00000C00UL |   /* RXER */
                    0x00000000UL |   /* TXCLK */
                    0x00000000UL |   /* RXDV */                                                                                                                                 
                    0x00000000UL     /* Default value */
          
            )    
	};
#define RBA_ETH_STOP_SEC_CONST_32
#include "rba_Eth_MemMap.h"

/*
 ***************************************************************************************************
 * Prototype for Static functions: Start
 ***************************************************************************************************
 */

/*
 ***************************************************************************************************
 * Prototype for Static functions: End
 ***************************************************************************************************
 */

