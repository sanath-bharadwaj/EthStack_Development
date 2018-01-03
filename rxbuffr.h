#ifndef ETH_RXBUFFER_H_
#define ETH_RXBUFFER_H_

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
    extern FUNC(void, ETH_CODE) Eth_RxBuffer_AdvanceQueueIndex(P2VAR(uint16, AUTOMATIC, AUTOMATIC) QueueIndex_pu16,
                                                               VAR(uint16, AUTOMATIC) LastIndex_u16);
#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#endif /* ETH_RXBUFFER_H_ */

/*<BASDKey>
**********************************************************************************************************************
* $History___:
* 
* AR41.17.0.0; 0     18.03.2015 GJO5KOR
*   CSCRM00774400: Memmap for code and data sections
* 
* AR40.15.0.0; 0     13.10.2014 SRY5KOR
*     CSCRM00646980:Fixed review issues. Removed version check.
*     To be implemented in Eth.h in next release
* 
* AR41_ZYNC7000.3.8.0; 0     18.02.2014 NUU1KOR
*   CSCRM00552036: AR4.1.1 requirement implementation
* 
* AR40_ZYNC7000.3.7.0; 2     24.12.2013 NUU1KOR
*   CSCRM00579837 : 
*   Eth_RxBuffer_AdvanceQueueIndex() function parameters updated with correct 
*   datatype
* 
* AR40_ZYNC7000.3.7.0; 1     18.11.2013 SWC1COB
*   SW minor version changed
* 
* AR40_ZYNC7000.3.7.0; 0     13.11.2013 SWC1COB
*   Rx Advance queue buffer added
* 
* $
**********************************************************************************************************************
</BASDKey>*/
