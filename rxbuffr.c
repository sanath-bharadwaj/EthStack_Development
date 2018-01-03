#include "Eth.h"
#include "Eth_RxBuffer.h"



#ifdef ETH_CONFIGURED

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

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

#define ETH_START_SEC_CODE
#include "Eth_MemMap.h"

/**
 ***************************************************************************************************
 * \moduledescription
 * Either increments QueueIndex or sets it to 0 for a queue wrap around
 * Is used for updating the RxBuffer index or DescQueue index.
 *
 * Parameter In:
 * \param LastIndex_u16   Last index in queue
 *
 * Parameter InOut:
 * \param QueueIndex_pu16 Queue index to be advanced
 *
 * \return               None
 *
 ***************************************************************************************************
 */
FUNC(void, ETH_CODE) Eth_RxBuffer_AdvanceQueueIndex(P2VAR(uint16, AUTOMATIC, AUTOMATIC) QueueIndex_pu16,
                                                    VAR(uint16, AUTOMATIC) LastIndex_u16)
{
    if ( (*QueueIndex_pu16 >= LastIndex_u16) ) /* last buffer in queue? */
    { /* yes */
        /* wrap round to start of queue */
        *QueueIndex_pu16 = 0U;
    }
    else
    { /* no */
        /* advance to next buffer in queue */
        *QueueIndex_pu16 += 1U;
    }
}
#define ETH_STOP_SEC_CODE
#include "Eth_MemMap.h"

#endif /* ETH_CONFIGURED */


/*<BASDKey>
**********************************************************************************************************************
* $History___:
*
* AR41.17.0.0; 0     18.03.2015 GJO5KOR
*   CSCRM00774400: Memmap for code and data sections
*
* AR40.15.0.0; 0     13.10.2014 SRY5KOR
*    CSCRM00646980:Develop COMP: Eth, HW independent component
*     Fixed review issues. Removed version check.
*     To be implemented in Eth.h in next release
*
* AR41_ZYNC7000.3.8.0; 0     18.02.2014 NUU1KOR
*   CSCRM00552036: AR4.1.1 requirement implementation
*
* AR40_ZYNC7000.3.7.0; 2     24.12.2013 NUU1KOR
*   CSCRM00579837:
*
*   Datatypes changed in Eth_RxBuffer_AdvanceQueueIndex()
*
* AR40_ZYNC7000.3.7.0; 1     18.11.2013 SWC1COB
*   SW minor version changed
*
* AR40_ZYNC7000.3.7.0; 0     13.11.2013 SWC1COB
*   Rx Advance queue Buffer added
*
* $
**********************************************************************************************************************
</BASDKey>*/

