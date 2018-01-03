#ifndef ETH_PBCFG_H_
#define ETH_PBCFG_H_


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


 
/* size of memory for DMA descriptors of all controllers */
#define ETH_DMA_DESC_MEM_SIZE (112) /* words */

/* Reference to the DEM event ETH_E_ACCESS */
#ifndef ETH_E_ACCESS_0
#define ETH_E_ACCESS_0   DemConf_DemEventParameter_ETH_E_ACCESS_0
#endif /* ETH_E_ACCESS_0 */

#ifndef ETH_E_RXFRAMESLOST_0
#define ETH_E_RXFRAMESLOST_0   0U
#endif /* ETH_E_RXFRAMESLOST_0 */

#ifndef ETH_E_CRC_0
#define ETH_E_CRC_0   0U
#endif /* ETH_E_CRC_0 */

#ifndef ETH_E_UNDERSIZE_0
#define ETH_E_UNDERSIZE_0   0U
#endif /* ETH_E_UNDERSIZE_0 */

#ifndef ETH_E_OVERSIZE_0
#define ETH_E_OVERSIZE_0   0U
#endif /* ETH_E_OVERSIZE_0 */

#ifndef ETH_E_ALIGNMENT_0
#define ETH_E_ALIGNMENT_0   0U
#endif /* ETH_E_ALIGNMENT_0 */

#ifndef ETH_E_SINGLECOLLISION_0
#define ETH_E_SINGLECOLLISION_0   0U
#endif /* ETH_E_SINGLECOLLISION_0 */

#ifndef ETH_E_MULTIPLECOLLISION_0
#define ETH_E_MULTIPLECOLLISION_0   0U
#endif /* ETH_E_MULTIPLECOLLISION_0 */

#ifndef ETH_E_LATECOLLISION_0
#define ETH_E_LATECOLLISION_0   0U
#endif /* ETH_E_LATECOLLISION_0 */


/* size of last DMA buffer in each receive buffer queue */
#define ETH_LAST_RX_DMA_BUF_LEN_BYTE 1536U

/* size of memory for DMA buffers of all controllers */
#define ETH_DMA_BUFFER_MEM_SIZE (21504) /* bytes */


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

#define ETH_START_SEC_CONST_FAST_UNSPECIFIED
#include "Eth_MemMap.h"
    extern CONST(Eth_ConfigType, ETH_CFG) Eth_ConfigSets[ETH_CONFIGTYPE_COUNT];
#define ETH_STOP_SEC_CONST_FAST_UNSPECIFIED
#include "Eth_MemMap.h"

#define ETH_START_SEC_CONST_8
#include "Eth_MemMap.h"
    extern CONST(uint8, ETH_CFG) Eth_PhysAddresses_acu8_MP[ETH_MAX_CTRLS_SUPPORTED][ETH_MAC_ADDRESS_LENGTH_BYTE];
#define ETH_STOP_SEC_CONST_8
#include "Eth_MemMap.h"

#define RBA_ETH_START_SEC_CONST_32
#include "rba_Eth_MemMap.h"
    extern CONST(uint32, ETH_CONST) rba_Eth_Port_ConfigSets[RBA_ETH_PORTCONFIG_COUNT];
#define RBA_ETH_STOP_SEC_CONST_32
#include "rba_Eth_MemMap.h"

#define Eth_Config      (Eth_ConfigSets[0])

#endif /* _ETH_PBCFG_H_ */

