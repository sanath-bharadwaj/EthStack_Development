#ifndef RBA_ETH_PRV_H_
#define RBA_ETH_PRV_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */

/*
 *  SW version checks
 */

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

#define RBA_ETH_BROADCAST_ADDR_LOW              (0xFFFFFFFFUL)
#define RBA_ETH_BROADCAST_ADDR_HIGH             (0x0000FFFFUL) //(0xFFFF0000UL)

#define RBA_ETH_SIZE_OF_STATISTIC_COUNTER_ARRAY         (368UL)
#define RBA_ETH_SEARCHUSEDFLAG                          (1)

/* CRC stripping is implemented so this MACRO is not used in application as of now */
#define RBA_ETH_CRC_LEN                                 (4U)

/* Macros for mac address handling */

#define RBA_ETH_PHYADDR_BYTE1_POS 0U
#define RBA_ETH_PHYADDR_BYTE1_MASK 0x000000FFUL

#define RBA_ETH_PHYADDR_BYTE2_POS 8U
#define RBA_ETH_PHYADDR_BYTE2_MASK 0x0000FF00UL

#define RBA_ETH_PHYADDR_BYTE3_POS 16U
#define RBA_ETH_PHYADDR_BYTE3_MASK 0x00FF0000UL

#define RBA_ETH_PHYADDR_BYTE4_POS 24U
#define RBA_ETH_PHYADDR_BYTE4_MASK 0xFF000000UL

#define RBA_ETH_PHYADDR_BYTE5_POS 0U
#define RBA_ETH_PHYADDR_BYTE5_MASK 0x000000FFUL

#define RBA_ETH_PHYADDR_BYTE6_POS 8U
#define RBA_ETH_PHYADDR_BYTE6_MASK 0x0000FF00UL


/***************************************************************************************************
 ************************************ Eth miscellaneous values *************************************
***************************************************************************************************/

#define RBA_ETH_TXDESC_SIZE                  (16U)
#define RBA_ETH_RXDESC_NORMAL_SIZE           (16U)
#define RBA_ETH_RXDESC_ADVANCED_SIZE         (32U)

#define RBA_ETH_EVEN_RX_BUFFER               (2U)
#define RBA_ETH_MUL_BY_TWO(value)            ((value)*2U)


/***************************************************************************************************
 ******************** Eth_GetEtherStats() and Eth_GetDropCount() related macros values *************
***************************************************************************************************/
#define RBA_ETH_GETTXSTATS_INVALID_VALUE              (0xFFFFFFFFUL)
#define RBA_ETH_GETRXSTATS_INVALID_VALUE              (0xFFFFFFFFUL)
#define RBA_ETH_GETCOUNTERVALUE_INVALID_VALUE         (0xFFFFFFFFUL)
#define RBA_ETH_GETTXERRORCOUNTERVALUE_INVALID_VALUE  (0xFFFFFFFFUL)
#define RBA_ETH_GETTXRXSTATS_U64_INVALID_VALUE        (0XFFFFFFFFFFFFFFFFULL)
#define RBA_ETH_GETDROPCNT_VALIDVALUES_COUNT            (9U)



/***************************************************************************************************
 ************************************ Eth DMA registers ********************************************
***************************************************************************************************/

/***************************************************************************************************/
/***** DMA mode register *****/
/* bit positions */
#define RBA_ETH_DMA_MODE_SWR_POS         (0UL)
#define RBA_ETH_DMA_MODE_DA_POS          (1UL)
#define RBA_ETH_DMA_MODE_TAA_POS         (2UL)
#define RBA_ETH_DMA_MODE_TXPR_POS        (11UL)
#define RBA_ETH_DMA_MODE_PR_POS          (12UL)
#define RBA_ETH_DMA_MODE_INTM_POS        (16UL)

/* operation values */
#define RBA_ETH_DMA_MODE_SWR         (1UL)
#define RBA_ETH_DMA_MODE_DA          (0UL)
#define RBA_ETH_DMA_MODE_TAA         (0UL)
#define RBA_ETH_DMA_MODE_TXPR        (0UL)
#define RBA_ETH_DMA_MODE_PR          (0UL)
#define RBA_ETH_DMA_MODE_INTM        (0UL)

/***************************************************************************************************/

/***** DMA System Bus Mode register *****/
/* bit positions */
#define RBA_ETH_DMA_SYSBUSMODE_FBL_POS         (0UL)
#define RBA_ETH_DMA_SYSBUSMODE_AAL_POS         (12UL)
#define RBA_ETH_DMA_SYSBUSMODE_MB_POS          (14UL)
#define RBA_ETH_DMA_SYSBUSMODE_RB_POS          (15UL)

/* operation values */
#define RBA_ETH_DMA_SYSBUSMODE_FBL         (0UL)
#define RBA_ETH_DMA_SYSBUSMODE_AAL         (0UL)
#define RBA_ETH_DMA_SYSBUSMODE_MB          (0UL)
#define RBA_ETH_DMA_SYSBUSMODE_RB          (0UL)

/***************************************************************************************************/

/***** DMA Channel n Control register *****/
/* bit positions */
#define RBA_ETH_DMA_CH_N_CONTROL_DSL_POS         (18UL)
#define RBA_ETH_DMA_CH_N_CONTROL_PBLX8_POS         (16UL)

/* operation values */
#define RBA_ETH_DMA_CH_N_CONTROL_DSL         (0UL)
#define RBA_ETH_DMA_CH_N_CONTROL_PBLX8       (0UL)

/***************************************************************************************************/

/***** DMA Channel n Transmit Control register *****/
/* bit positions */
#define RBA_ETH_DMA_CH_N_TX_CONTROL_ST_POS       (0UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_TCW_POS      (1UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_OSF_POS      (4UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_TXPBLX8_POS  (16UL)
/* operation values */
#define RBA_ETH_DMA_CH_N_TX_CONTROL_ST       (1UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_TCW      (0UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_OSF      (0UL)
#define RBA_ETH_DMA_CH_N_TX_CONTROL_TXPBLX8  (0UL)

/***************************************************************************************************/

/***** DMA Channel n Receive Control register *****/
/* bit positions */
#define RBA_ETH_DMA_CH_N_RX_CONTROL_SR_POS       (0UL)

/* As per the datasheet, the RBSZ bit field starts from bit position 4, but actually it starts from bit position 1 */
#define RBA_ETH_DMA_CH_N_RX_CONTROL_RBSZ_POS     (1UL)

#define RBA_ETH_DMA_CH_N_RX_CONTROL_TXPBLX8_POS  (16UL)
#define RBA_ETH_DMA_CH_N_RX_CONTROL_RPF_POS      (31UL)

/* operation values */
#define RBA_ETH_DMA_CH_N_RX_CONTROL_SR       (1UL)
#define RBA_ETH_DMA_CH_N_RX_CONTROL_RBSZ     (0UL)
#define RBA_ETH_DMA_CH_N_RX_CONTROL_TXPBLX8  (0UL)
#define RBA_ETH_DMA_CH_N_RX_CONTROL_RPF      (0UL)

/***************************************************************************************************/

/***** DMA Channel n Interrupt Enable register *****/
/* bit positions */
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE_POS  (0UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE_POS  (6UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE_POS (12UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_AIE_POS  (14UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE_POS  (15UL)

/* operation values */
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE  (1UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE  (1UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE (1UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_AIE  (1UL)
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE  (1UL)

/* this is the list of currently known and handled interrupts. anything else is weird */
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_HANDLEDTXIRQBITS      ((RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE   << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_TIE_POS) | \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE_POS)| \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_AIE   << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_AIE_POS) | \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE   << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE_POS))

/* this is the list of currently known and handled interrupts. anything else is weird */
#define RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_HANDLEDRXIRQBITS      ((RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_RIE_POS)  | \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE_POS)| \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_FBEE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_AIE_POS) | \
                                                                 (RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE  << RBA_ETH_DMA_CH_N_INTERRUPT_ENABLE_NIE_POS))

/***************************************************************************************/

/***** DMA Channel n Status register *****/
/* bit positions */
#define RBA_ETH_DMA_CH_N_STATUS_TI_POS       (0UL)
#define RBA_ETH_DMA_CH_N_STATUS_RI_POS       (6UL)
#define RBA_ETH_DMA_CH_N_STATUS_FBE_POS     (12UL)
#define RBA_ETH_DMA_CH_N_STATUS_AIS_POS      (14UL)
#define RBA_ETH_DMA_CH_N_STATUS_NIS_POS      (15UL)

/* operation values */
#define RBA_ETH_DMA_CH_N_STATUS_TI           (1UL)
#define RBA_ETH_DMA_CH_N_STATUS_RI           (1UL)
#define RBA_ETH_DMA_CH_N_STATUS_FBE         (1UL)
#define RBA_ETH_DMA_CH_N_STATUS_AIS          (1UL)
#define RBA_ETH_DMA_CH_N_STATUS_NIS          (1UL)


/* this is the list of currently known and handled interrupts. anything else is weird */
#define RBA_ETH_DMA_CH_N_STATUS_HANDLEDTXIRQBITS     ((RBA_ETH_DMA_CH_N_STATUS_TI  << RBA_ETH_DMA_CH_N_STATUS_TI_POS)   | \
                                                      (RBA_ETH_DMA_CH_N_STATUS_FBE << RBA_ETH_DMA_CH_N_STATUS_FBE_POS)| \
                                                      (RBA_ETH_DMA_CH_N_STATUS_AIS << RBA_ETH_DMA_CH_N_STATUS_AIS_POS)  | \
                                                      (RBA_ETH_DMA_CH_N_STATUS_NIS << RBA_ETH_DMA_CH_N_STATUS_NIS_POS))

#define RBA_ETH_DMA_CH_N_STATUS_HANDLEDRXIRQBITS     ((RBA_ETH_DMA_CH_N_STATUS_RI  << RBA_ETH_DMA_CH_N_STATUS_RI_POS)   | \
                                                      (RBA_ETH_DMA_CH_N_STATUS_FBE << RBA_ETH_DMA_CH_N_STATUS_FBE_POS)| \
                                                      (RBA_ETH_DMA_CH_N_STATUS_AIS << RBA_ETH_DMA_CH_N_STATUS_AIS_POS)  | \
                                                      (RBA_ETH_DMA_CH_N_STATUS_NIS << RBA_ETH_DMA_CH_N_STATUS_NIS_POS))

/***************************************************************************************/

/***** DMA Debug Status 0 Register *****/

/*Bit Positions*/

#define RBA_ETH_DMA_DEBUG_STATUS_AXWHSTS_POS       (0U)
#define RBA_ETH_DMA_DEBUG_STATUS_RPS0_POS          (8U)
#define RBA_ETH_DMA_DEBUG_STATUS_TPS0_POS          (12U)
#define RBA_ETH_DMA_DEBUG_STATUS_RPS1_POS          (16U)
#define RBA_ETH_DMA_DEBUG_STATUS_TPS1_POS          (20U)


#define RBA_ETH_DMA_DEBUG_RPS0_TPS0  ((RBA_ETH_DMA_DEBUG_STATUS_RPS0     << RBA_ETH_DMA_DEBUG_STATUS_RPS0_POS)      | \
                                      (RBA_ETH_DMA_DEBUG_STATUS_TPS0     << RBA_ETH_DMA_DEBUG_STATUS_TPS0_POS))


/*Operation values*/
#define RBA_ETH_DMA_DEBUG_STATUS_AXWHSTS       (0x01UL)
#define RBA_ETH_DMA_DEBUG_STATUS_RPS0          (0x0FUL)
#define RBA_ETH_DMA_DEBUG_STATUS_TPS0          (0x0FUL)
#define RBA_ETH_DMA_DEBUG_STATUS_RPS1          (0x0FUL)
#define RBA_ETH_DMA_DEBUG_STATUS_TPS1          (0x0FUL)


#define RBA_ETH_DMA_DEBUG_ALL   ((RBA_ETH_DMA_DEBUG_STATUS_AXWHSTS  << RBA_ETH_DMA_DEBUG_STATUS_AXWHSTS_POS)   | \
                                 (RBA_ETH_DMA_DEBUG_STATUS_RPS0     << RBA_ETH_DMA_DEBUG_STATUS_RPS0_POS)      | \
                                 (RBA_ETH_DMA_DEBUG_STATUS_TPS0     << RBA_ETH_DMA_DEBUG_STATUS_TPS0_POS)      | \
                                 (RBA_ETH_DMA_DEBUG_STATUS_RPS1     << RBA_ETH_DMA_DEBUG_STATUS_RPS1_POS)      | \
                                 (RBA_ETH_DMA_DEBUG_STATUS_TPS1     << RBA_ETH_DMA_DEBUG_STATUS_TPS1_POS))


/***************************************************************************************************
 ************************************ Eth MTL registers ********************************************
***************************************************************************************************/


/***************************************************************************************/

/***** Operation Mode register *****/
/* bit positions */
#define RBA_ETH_MTL_OPERATION_MODE_DTXSTS_POS   (1UL)
#define RBA_ETH_MTL_OPERATION_MODE_RAA_POS      (2UL)
#define RBA_ETH_MTL_OPERATION_MODE_SCHALG_POS   (5UL)
#define RBA_ETH_MTL_OPERATION_MODE_CNTPRST_POS  (8UL)
#define RBA_ETH_MTL_OPERATION_MODE_CNTCLR_POS   (9UL)

/* operation values */
#define RBA_ETH_MTL_OPERATION_MODE_DTXSTS   (0UL)
#define RBA_ETH_MTL_OPERATION_MODE_RAA      (0UL)
#define RBA_ETH_MTL_OPERATION_MODE_SCHALG   (0UL)
#define RBA_ETH_MTL_OPERATION_MODE_CNTPRST  (1UL)
#define RBA_ETH_MTL_OPERATION_MODE_CNTCLR   (1UL)

/***************************************************************************************/

/***** Receive Queue and DMA Channel Mapping 0 Register *****/
/* bit positions */
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q0MDMACH_POS   (0UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q0DDMACH_POS   (4UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q1MDMACH_POS   (8UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q1DDMACH_POS   (12UL)

/* operation values */
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q0MDMACH   (0UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q0DDMACH   (0UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q1MDMACH   (1UL)
#define RBA_ETH_MTL_RXQ_DMA_MAP0_Q1DDMACH   (0UL)

/***************************************************************************************/

/***** Queue 0 Transmit Operation Mode Register *****/
/* bit positions */
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_FTQ_POS     (0UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TSF_POS     (1UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TXQEN_POS   (2UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TTC_POS     (4UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS_POS     (16UL)

/* operation values */
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_FTQ     (1UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TSF     (1UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TXQEN   (2UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TTC     (0UL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS     (RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS_TXFIFO_4K)

#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS_TXFIFO_4K  (0x0FUL)
#define RBA_ETH_MTL_TXQ0_OPERATION_MODE_TQS_TXFIFO_2K  (0x08UL)

/***************************************************************************************/

/***** Queue 0 Receive Operation Mode Register *****/
/* bit positions */
#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RSF_POS     (5UL)
#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS_POS     (20UL)

/* operation values */
#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RSF     (1UL)
#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS     (RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS_RXFIFO_8K)

#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS_RXFIFO_8K   (0x1FUL)
#define RBA_ETH_MTL_RXQ0_OPERATION_MODE_RQS_RXFIFO_4K   (0x0FUL)

#define MTL_RXQ0_DEBUG_RXQSTS_MASK (0x00000030UL)

/***************************************************************************************/


/***************************************************************************************************
 ************************************ Eth MAC registers ********************************************
***************************************************************************************************/


/***************************************************************************************/

/***** MAC Configuration Register *****/
/* bit positions */
#define RBA_ETH_MAC_CONFIGURATION_RE_POS        (0UL)
#define RBA_ETH_MAC_CONFIGURATION_TE_POS        (1UL)
#define RBA_ETH_MAC_CONFIGURATION_PRELEN_POS    (2UL)
#define RBA_ETH_MAC_CONFIGURATION_DCRS_POS      (9UL)
#define RBA_ETH_MAC_CONFIGURATION_ECRSFD_POS    (11UL)
#define RBA_ETH_MAC_CONFIGURATION_LM_POS        (12UL)
#define RBA_ETH_MAC_CONFIGURATION_DM_POS        (13UL)
#define RBA_ETH_MAC_CONFIGURATION_FES_POS       (14UL)
#define RBA_ETH_MAC_CONFIGURATION_PS_POS        (15UL)
#define RBA_ETH_MAC_CONFIGURATION_JE_POS        (16UL)
#define RBA_ETH_MAC_CONFIGURATION_JD_POS        (17UL)
#define RBA_ETH_MAC_CONFIGURATION_WD_POS        (19UL)
#define RBA_ETH_MAC_CONFIGURATION_ACS_POS       (20UL)
#define RBA_ETH_MAC_CONFIGURATION_CST_POS       (21UL)
#define RBA_ETH_MAC_CONFIGURATION_S2KP_POS      (22UL)
#define RBA_ETH_MAC_CONFIGURATION_GPSLCE_POS    (23UL)
#define RBA_ETH_MAC_CONFIGURATION_IPG_POS       (24UL)
#define RBA_ETH_MAC_CONFIGURATION_IPC_POS       (27UL)
#define RBA_ETH_MAC_CONFIGURATION_SARC_POS      (28UL)
#define RBA_ETH_MAC_CONFIGURATION_ARPEN_POS     (31UL)

/* operation values */
#define RBA_ETH_MAC_CONFIGURATION_RE            (1UL)
#define RBA_ETH_MAC_CONFIGURATION_TE            (1UL)
#define RBA_ETH_MAC_CONFIGURATION_PRELEN        (0UL)
#define RBA_ETH_MAC_CONFIGURATION_DCRS          (0UL)
#define RBA_ETH_MAC_CONFIGURATION_ECRSFD        (1UL)
#define RBA_ETH_MAC_CONFIGURATION_LM            (RBA_ETH_MAC_CONFIGURATION_LM_DISABLE)
#define RBA_ETH_MAC_CONFIGURATION_DM            (1UL)
#define RBA_ETH_MAC_CONFIGURATION_FES_10MBPS    (0UL)
#define RBA_ETH_MAC_CONFIGURATION_FES_100MBPS   (1UL)
#define RBA_ETH_MAC_CONFIGURATION_PS_GMII       (0x0UL)
#define RBA_ETH_MAC_CONFIGURATION_PS_MII        (0x1UL)
#define RBA_ETH_MAC_CONFIGURATION_JE            (0UL)
#define RBA_ETH_MAC_CONFIGURATION_JD            (0UL)
#define RBA_ETH_MAC_CONFIGURATION_WD            (0UL)
#define RBA_ETH_MAC_CONFIGURATION_ACS           (1UL)
#define RBA_ETH_MAC_CONFIGURATION_CST           (1UL)
#define RBA_ETH_MAC_CONFIGURATION_S2KP          (0UL)
#define RBA_ETH_MAC_CONFIGURATION_GPSLCE        (0UL)
#define RBA_ETH_MAC_CONFIGURATION_IPG           (0UL)
#define RBA_ETH_MAC_CONFIGURATION_IPC           (1UL)
#define RBA_ETH_MAC_CONFIGURATION_SARC          (3UL)
#define RBA_ETH_MAC_CONFIGURATION_ARPEN         (0UL)

#define RBA_ETH_MAC_CONFIGURATION_LM_ENABLE     (1UL)
#define RBA_ETH_MAC_CONFIGURATION_LM_DISABLE    (0UL)




/***************************************************************************************/

/***** MAC Packet Filter Register *****/
/* bit positions */
#define RBA_ETH_MAC_PACKET_FILTER_PR_POS        (0UL)
#define RBA_ETH_MAC_PACKET_FILTER_DBF_POS       (5UL)
#define RBA_ETH_MAC_PACKET_FILTER_HPF_POS       (10UL)
#define RBA_ETH_MAC_PACKET_FILTER_VTFE_POS      (16UL)
#define RBA_ETH_MAC_PACKET_FILTER_RA_POS        (31UL)

/* operation values */
#define RBA_ETH_MAC_PACKET_FILTER_PR        (0UL)
#define RBA_ETH_MAC_PACKET_FILTER_DBF       (OUL)
#define RBA_ETH_MAC_PACKET_FILTER_HPF       (1UL)
#define RBA_ETH_MAC_PACKET_FILTER_VTFE      (0UL)
#define RBA_ETH_MAC_PACKET_FILTER_RA        (0UL)

/***************************************************************************************/

/***** MAC Q0 Flow Control Register *****/
/* bit positions */
#define RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ_POS    (7UL)

/* operation values */
#define RBA_ETH_MAC_Q0_TX_FLOW_CTRL_DZPQ    (1UL)

/***************************************************************************************/

/***** Receive Queue Control 0 Register *****/
/* bit positions */
#define RBA_ETH_MAC_RXQ_CTRL0_RXQ0EN_POS         (0UL)
#define RBA_ETH_MAC_RXQ_CTRL0_RXQ1EN_POS         (2UL)

/* operation values */
#define RBA_ETH_MAC_RXQ_CTRL0_RXQ0EN             (0x00000001UL)
#define RBA_ETH_MAC_RXQ_CTRL0_RXQ1EN             (0x00000001UL)

/***************************************************************************************/

/***************************************************************************************/

/***** Receive Queue Control 1 Register *****/
/* bit positions */
#define RBA_ETH_MAC_RXQ_CTRL1_AVCPQ_POS         (0UL)
#define RBA_ETH_MAC_RXQ_CTRL1_AVPTPQ_POS        (4UL)
#define RBA_ETH_MAC_RXQ_CTRL1_UPQ_POS           (12UL)
#define RBA_ETH_MAC_RXQ_CTRL1_MCBCQ_POS         (16UL)
#define RBA_ETH_MAC_RXQ_CTRL1_MCBCQEN_POS       (20UL)

/* operation values */
#define RBA_ETH_MAC_RXQ_CTRL1_AVCPQ         (0UL)
#define RBA_ETH_MAC_RXQ_CTRL1_AVPTPQ        (0UL)
#define RBA_ETH_MAC_RXQ_CTRL1_UPQ           (0UL)
#define RBA_ETH_MAC_RXQ_CTRL1_MCBCQ         (0UL)
#define RBA_ETH_MAC_RXQ_CTRL1_MCBCQEN       (1UL)

/***************************************************************************************/

/***** MDIO Address Register *****/
/* bit positions */
#define RBA_ETH_MAC_MDIO_ADDRESS_GB_POS         (0UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_C45E_POS       (1UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_GOC_POS        (2UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_POS         (8UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_RDA_POS        (16UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_PA_POS         (21UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_PSE_POS        (27UL)

/* operation values */
#define RBA_ETH_MAC_MDIO_ADDRESS_GB             (1UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_C45E           (0UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_GOC_W          (1UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_GOC_R          (3UL)
#define RBA_ETH_MAC_MDIO_ADDRESS_PSE            (0UL)

/* bit group masks */
#define RBA_ETH_MAC_MDIO_ADDRESS_GB_MASK        (1UL << RBA_ETH_MAC_MDIO_ADDRESS_GB_POS)
#define RBA_ETH_MAC_MDIO_ADDRESS_GOC_MASK       (0x3UL << RBA_ETH_MAC_MDIO_ADDRESS_GOC_POS)
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_MASK        (0x0FUL << RBA_ETH_MAC_MDIO_ADDRESS_CR_POS)
#define RBA_ETH_MAC_MDIO_ADDRESS_RDA_MASK       (0x1FUL << RBA_ETH_MAC_MDIO_ADDRESS_RDA_POS)
#define RBA_ETH_MAC_MDIO_ADDRESS_PA_MASK        (0x1FUL << RBA_ETH_MAC_MDIO_ADDRESS_PA_POS)


/* MDIO Address register CR field definitions */

#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV42              (0x0UL)      /* !<divides hclk by 42  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV62              (0x1UL)      /* !<divides hclk by 62  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV16              (0x2UL)      /* !<divides hclk by 16  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV26              (0x3UL)      /* !<divides hclk by 26  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV102             (0x4UL)      /* !<divides hclk by 102 */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV124             (0x5UL)      /* !<divides hclk by 124 */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV4               (0x8UL)      /* !<divides hclk by 4   */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV6               (0x9UL)      /* !<divides hclk by 6   */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV8               (0x0AUL)     /* !<divides hclk by 8   */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV10              (0x0BUL)     /* !<divides hclk by 10  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV12              (0x0CUL)     /* !<divides hclk by 12  */
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV14              (0x0DUL)     /* !<divides hclk by 14  */
#if 0
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV16              (0x0EUL)     /* !<divides hclk by 16  open point*/
#endif
#define RBA_ETH_MAC_MDIO_ADDRESS_CR_DIV18              (0x0FUL)     /* !<divides hclk by 18  */

/***************************************************************************************/

/***** MAC Debug Register *****/

/*Bit Positions*/

#define RBA_ETH_MAC_DEBUG_RPESTS_POS            (0U)
#define RBA_ETH_MAC_DEBUG_RFCFCSTS_POS          (1U)
#define RBA_ETH_MAC_DEBUG_TPESTS_POS           (16U)
#define RBA_ETH_MAC_DEBUG_TFCSTS_POS           (17U)

/*Operation values*/
#define RBA_ETH_MAC_DEBUG_RPESTS                (0x1UL)
#define RBA_ETH_MAC_DEBUG_RFCFCSTS              (0x3UL)
#define RBA_ETH_MAC_DEBUG_TPESTS                (0x1UL)
#define RBA_ETH_MAC_DEBUG_TFCSTS                (0x3UL)

#define RBA_ETH_MAC_DEBUG_ALL   ((RBA_ETH_MAC_DEBUG_RPESTS   << RBA_ETH_MAC_DEBUG_RPESTS_POS)     | \
                                 (RBA_ETH_MAC_DEBUG_RFCFCSTS << RBA_ETH_MAC_DEBUG_RFCFCSTS_POS)   | \
                                 (RBA_ETH_MAC_DEBUG_TPESTS   << RBA_ETH_MAC_DEBUG_TPESTS_POS)     | \
                                 (RBA_ETH_MAC_DEBUG_TFCSTS   << RBA_ETH_MAC_DEBUG_TFCSTS_POS))

/***************************************************************************************/

/***** Eth GMAC GMII DATA Register *****/
/* Reset value */
#define RBA_ETH_MAC_MDIO_DATA_RST_VALUE                               (0x00000000UL)

/* Definitions of field "gd" */
#define RBA_ETH_MAC_MDIO_DATA_GD_POS                                  (0UL)
#define RBA_ETH_MAC_MDIO_DATA_GD                                      (0xFFFFUL)


/***************************************************************************************/

/***** Eth MAC Address Register *****/

/* bit positions */
#define RBA_ETH_MAC_ADDRESS0_HIGH_AE_POS                           (31UL)


/* bit group masks */
#define RBA_ETH_MAC_ADDRESS0_HIGH_AE_MASK                          (1UL<<RBA_ETH_MAC_ADDRESS0_HIGH_AE_POS)

/* operation values */
#define RBA_ETH_MAC_ADDRESS0_HIGH_AE                               (1UL)


/* bit positions */
#define RBA_ETH_MAC_ADDRESS_HIGH_AE_POS                           (31UL)
#define RBA_ETH_MAC_ADDRESS_HIGH_DCS_POS                          (16UL)
/* operation values */
#define RBA_ETH_MAC_ADDRESS_HIGH_AE                               (1UL)
#define RBA_ETH_MAC_ADDRESS_HIGH_DCS                              (1UL)
/* bit group masks */
#define RBA_ETH_MAC_ADDRESS_HIGH_AE_MASK                          (RBA_ETH_MAC_ADDRESS_HIGH_AE<<RBA_ETH_MAC_ADDRESS_HIGH_AE_POS)
#define RBA_ETH_MAC_ADDRESS_HIGH_DCS_MASK                         (RBA_ETH_MAC_ADDRESS_HIGH_DCS<<RBA_ETH_MAC_ADDRESS_HIGH_DCS_POS)


/******Registers specific to IFX Dev5-B step******/
/***** Eth GPCTL Register *****/

/* bit positions */
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
#define RBA_ETH_GPCTL_EPR_POS                   (22UL)
#define RBA_ETH_GPCTL_EPR_MIISELECT             (0x0UL)
#define RBA_ETH_GPCTL_EPR_RGMIISELECT           (0x1UL)
#define RBA_ETH_GPCTL_EPR_RMIISELECT            (0x4UL)


/***** Eth Kernel Reset Registers *****/

/* bit positions */

#define RBA_ETH_KRSTCLR_CLR_POS                        (0UL)
#define RBA_ETH_KRST0_RST_POS                          (0UL)
#define RBA_ETH_KRST1_RST_POS                          (0UL)
#define RBA_ETH_KRST0_RSTSTAT_POS                      (1UL)

/* operation values */
#define RBA_ETH_KRSTCLR_CLR                            (0x1UL)
#define RBA_ETH_KRST0_RST                              (0x1UL)
#define RBA_ETH_KRST1_RST                              (0x1UL)
#define RBA_ETH_KRST0_RSTSTAT                          (0x1UL)

#define RBA_ETH_RGMII_RX_CLK_DELAY_REQ_POS             (8UL)
#define RBA_ETH_RGMII_TX_CLK_DELAY_REQ_POS             (0UL)
#endif

/***************************************************************************************************
 ************************************ Descriptor Definitions ***************************************
***************************************************************************************************/


/**** Receive buffer descriptor bit definitions ****/

/* Normal Descriptor RDES0   */
/* Write-Back Format */
#define RBA_ETH_MAC_DMARXDES0_IVT_MASK                  (0xFFFF0000UL)  /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES0_IVT_POS                   (16UL)          /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES0_0VT_MASK                  (0x0000FFFFUL)  /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES0_0VT_POS                   (0UL)           /*!< Layer 3 and Layer 4 Filter Number Matched */

/* Normal Descriptor RDES1   */
/* Write-Back Format */
#define RBA_ETH_MAC_DMARXDES1_OPC_MASK                  (0xFFFF0000UL)  /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES1_OPC_POS                   (16UL)          /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES1_TD                        (0x00008000UL)  /*!< Timestamp Dropped */
#define RBA_ETH_MAC_DMARXDES1_TSA                       (0x00004000UL)  /*!< Timestamp Available */
#define RBA_ETH_MAC_DMARXDES1_PV                        (0x00002000UL)  /*!< PTP Version */
#define RBA_ETH_MAC_DMARXDES1_PFT                       (0x00001000UL)  /*!< PTP Packet Type */
#define RBA_ETH_MAC_DMARXDES1_PMT                       (0x00000F00UL)  /*!< PTP Message Type */
#define RBA_ETH_MAC_DMARXDES1_IPCE                      (0x00000080UL)  /*!< IP Payload Error */
#define RBA_ETH_MAC_DMARXDES1_IPCB                      (0x00000040UL)  /*!< IP Checksum Bypassed */
#define RBA_ETH_MAC_DMARXDES1_IPV6                      (0x00000020UL)  /*!< IPv6 header Present */
#define RBA_ETH_MAC_DMARXDES1_IPV4                      (0x00000010UL)  /*!< IPV4 Header Present */
#define RBA_ETH_MAC_DMARXDES1_IPHE                      (0x00000008UL)  /*!< IP Header Error */
#define RBA_ETH_MAC_DMARXDES1_PT                        (0x00000007UL)  /*!< Payload Type */

/* Normal Descriptor RDES2   */
/* Write-Back Format */
#define RBA_ETH_MAC_DMARXDES2_L3L4FM                   (0xE0000000UL)  /*!< Layer 3 and Layer 4 Filter Number Matched */
#define RBA_ETH_MAC_DMARXDES2_L4FM                     (0x10000000UL)  /*!< Layer 4 Filter Match */
#define RBA_ETH_MAC_DMARXDES2_L3FM                     (0x08000000UL)  /*!< Layer 3 Filter Match */
#define RBA_ETH_MAC_DMARXDES2_MADRM                    (0x07F80000UL)  /*!< MAC Address Match or Hash Value */
#define RBA_ETH_MAC_DMARXDES2_HF                       (0x00040000UL)  /*!< Hash Filter Status */
#define RBA_ETH_MAC_DMARXDES2_DAF                      (0x00020000UL)  /*!< Destination Address Filter Fail */
#define RBA_ETH_MAC_DMARXDES2_SAF                      (0x00010000UL)  /*!< SA Address Filter Fail */
#define RBA_ETH_MAC_DMARXDES2_VF                       (0x00008000UL)  /*!< VLAN Filter Status */
#define RBA_ETH_MAC_DMARXDES2_ARPNR                    (0x00000800UL)  /*!< ARP Reply Not Generated */

/* Normal Descriptor RDES3   */
#define RBA_ETH_MAC_DMARXDESC3_OWN                     (0x80000000UL)  /*!< Own bit*/

/* Read Format */
#define RBA_ETH_MAC_DMARXDESC3_IOC                     (0x40000000UL)  /*!< Interrupt Enabled on Completion */
#define RBA_ETH_MAC_DMARXDESC3_BUF2V                   (0x02000000UL) /*!< Buffer 2 Address Valid*/

/* Write-Back Format */
#define RBA_ETH_MAC_DMARXDESC3_CTXT                    (0x40000000UL)  /*!< Receive Context Descriptor */
#define RBA_ETH_MAC_DMARXDESC3_FD                      (0x20000000UL)  /*!< First Decriptor*/
#define RBA_ETH_MAC_DMARXDESC3_LD                      (0x10000000UL)  /*!< Last Descriptor*/
#define RBA_ETH_MAC_DMARXDESC3_RS2V                    (0x08000000UL)  /*!< Receive Status RDES2 Valid */
#define RBA_ETH_MAC_DMARXDESC3_RS1V                    (0x04000000UL)  /*!< Receive Status RDES1 Valid */
#define RBA_ETH_MAC_DMARXDESC3_RS0V                    (0x02000000UL)  /*!< Receive Status RDES0 Valid */
#define RBA_ETH_MAC_DMARXDESC3_CE                      (0x01000000UL)  /*!< CRC Error */
#define RBA_ETH_MAC_DMARXDESC3_GP                      (0x00800000UL)  /*!< Giant Packet */
#define RBA_ETH_MAC_DMARXDESC3_RWT                     (0x00400000UL)  /*!< Receive Watchdog Timeout */
#define RBA_ETH_MAC_DMARXDESC3_OE                      (0x00200000UL)  /*!< Overflow Error */
#define RBA_ETH_MAC_DMARXDESC3_RE                      (0x00100000UL)  /*!< Receive Error */
#define RBA_ETH_MAC_DMARXDESC3_DE                      (0x00080000UL)  /*!< Dribble Bit Error */
#define RBA_ETH_MAC_DMARXDESC3_LT                      (0x00070000UL)  /*!< Length/Type Field */
#define RBA_ETH_MAC_DMARXDESC3_ES                      (0x00008000UL)  /*!< Error Summary*/
#define RBA_ETH_MAC_DMARXDESC3_FL_MASK                 (0x00003FFFUL)  /*!< Frame Length  */
#define RBA_ETH_MAC_DMARXDESC3_FL_POS                  (0UL)           /*!< Frame Length Position */

/**** Transmit  buffer descriptor bit definitions ****/
/* Normal Descriptor TDES2   */
/* Read Format */
#define RBA_ETH_MAC_DMATXDESC2_IOC                      (0x80000000UL) /*!< Interrupt on Completion */
#define RBA_ETH_MAC_DMATXDESC2_TTSE                     (0x40000000UL) /*!< Transmit Timestamp Enable */
#define RBA_ETH_MAC_DMATXDESC2_B2L                      (0x3FFF0000UL) /*!< Buffer 2 Length */
#define RBA_ETH_MAC_DMATXDESC2_VTIR                     (0X0000C000UL) /*!< VLAN Tag Insertion or Replacement */
#define RBA_ETH_MAC_DMATXDESC2_B1L_MASK                 (0x00003FFFUL) /*!< Buffer 1 Length */
#define RBA_ETH_MAC_DMATXDESC2_B1L_POS                  (0UL) /*!< Buffer 1 position */

/* Normal Descriptor TDES3   */
#define RBA_ETH_MAC_DMATXDESC3_OWN                      (0x80000000UL) /*!< Own bit*/
#define RBA_ETH_MAC_DMATXDESC2_CTXT                     (0x40000000UL) /*!< Context Type*/
#define RBA_ETH_MAC_DMATXDESC3_FD                       (0x20000000UL) /*!< First Descriptor */
#define RBA_ETH_MAC_DMATXDESC3_LD                       (0x10000000UL) /*!< Last Descriptor */

/* Read Format */
#define RBA_ETH_MAC_DMATXDESC3_CPC                      (RBA_ETH_MAC_DMATXDESC3_CPC_CRC_PAD_INSERTION) /*!< CRC Pad Control */
#define RBA_ETH_MAC_DMATXDESC3_SAIC                     (0x01000000UL) /*!<SA Insertion Control : MAC address register0; Replace the source address */
#define RBA_ETH_MAC_DMATXDESC3_SLOTNUM                  (0x00780000UL) /*!<SLOTNUM: Slot Number Control Bits in AV Mode */
#define RBA_ETH_MAC_DMATXDESC3_CIC                      (0x00030000UL) /*!<Checksum Insertion Control */
#define RBA_ETH_MAC_DMATXDESC3_FL_MASK                  (0x00007FFFUL) /*!< Frame Length  */
#define RBA_ETH_MAC_DMATXDESC3_FL_POS                   (0UL)          /*!< Frame Length Position  */

#define RBA_ETH_MAC_DMATXDESC3_CPC_CRC_PAD_INSERTION       (0x00000000UL)  /*!< CRC and Pad Insertion */
#define RBA_ETH_MAC_DMATXDESC3_CPC_CRC_INSERTION           (0x04000000UL)  /*!< CRC Insertion (Disable Pad Insertion) */
#define RBA_ETH_MAC_DMATXDESC3_CPC_DISABLE_CRC_INSERTION   (0x08000000UL)  /*!< Disable CRC Insertion */
#define RBA_ETH_MAC_DMATXDESC3_CPC_CRC_REPLACEMENT         (0x0C000000UL)  /*!< CRC Replacement */

/* Write-Back Format */
#define RBA_ETH_MAC_DMATXDESC3_TTSS                     (0x00020000UL) /*!< Tx Timestamp Status */
#define RBA_ETH_MAC_DMATXDESC3_ES                       (0x00008000UL) /*!< Error Summary */
#define RBA_ETH_MAC_DMATXDESC3_JT                       (0x00004000UL) /*!< Jabber Timeout */
#define RBA_ETH_MAC_DMATXDESC3_FF                       (0x00002000UL) /*!< Packet Flushed */
#define RBA_ETH_MAC_DMATXDESC3_PCE                      (0x00001000UL) /*!< Payload Checksum Error */
#define RBA_ETH_MAC_DMATXDESC3_LOC                      (0x00000800UL) /*!< Loss of Carrier */
#define RBA_ETH_MAC_DMATXDESC3_NC                       (0x00000400UL) /*!< No Carrier */
#define RBA_ETH_MAC_DMATXDESC3_LC                       (0x00000200UL) /*!< Late Collision */
#define RBA_ETH_MAC_DMATXDESC3_EC                       (0x00000100UL) /*!< Excessive Collision */
#define RBA_ETH_MAC_DMATXDESC3_CC                       (0x000000F0UL) /*!< Collision Count */
#define RBA_ETH_MAC_DMATXDESC3_ED                       (0x00000008UL) /*!< Excessive Deferral */
#define RBA_ETH_MAC_DMATXDESC3_UF                       (0x00000004UL) /*!< Underflow Error */
#define RBA_ETH_MAC_DMATXDESC3_DB                       (0x00000002UL) /*!< Deferred Bit */
#define RBA_ETH_MAC_DMATXDESC3_IHE                      (0x00000001UL) /*!< IP Header Error */




/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */
/*Macros For Global Time Stamp Support*/
/*Bit Positions*/
#define RBA_ETH_MAC_TIME_STAMP_AV_802_1AS_POS   (28U)
#define RBA_ETH_MAC_TIME_STAMP_TX_STATUS_POS    (24U)
#define RBA_ETH_MAC_TIME_STAMP_MACADDR_POS      (18U)
#define RBA_ETH_MAC_TIME_STAMP_SNAPSEL_POS      (16U)
#define RBA_ETH_MAC_TIME_STAMP_MSTREENA_POS     (15U)
#define RBA_ETH_MAC_TIME_STAMP_EVENA_POS        (14U)
#define RBA_ETH_MAC_TIME_STAMP_IP4ENA_POS       (13U)
#define RBA_ETH_MAC_TIME_STAMP_IP6ENA_POS       (12U)
#define RBA_ETH_MAC_TIME_STAMP_IPENA_POS        (11U)
#define RBA_ETH_MAC_TIME_STAMP_VER2_POS         (10U)
#define RBA_ETH_MAC_TIME_STAMP_CTRLSR_POS       (9U)
#define RBA_ETH_MAC_TIME_STAMP_ENALL_POS        (8U)
#define RBA_ETH_MAC_TIME_STAMP_ADDREG_POS       (5U)
#define RBA_ETH_MAC_TIME_STAMP_TRIGGER_POS      (4U)
#define RBA_ETH_MAC_TIME_STAMP_UPDATE_POS       (3U)
#define RBA_ETH_MAC_TIME_STAMP_INIT_POS         (2U)
#define RBA_ETH_MAC_TIME_STAMP_FC_UPDATE_POS    (1U)
#define RBA_ETH_MAC_TIME_STAMP_EN_POS           (0U)

/*Mac SubSecond and Sub NanoSecond Increment  Register*/
#define RBA_ETH_MAC_SUB_NS_SEDCOND_INC_POS         (8U)
#define RBA_ETH_MAC_SUB_NS_SECOND_INC_MASK         (0x0000FF00UL)
#define RBA_ETH_MAC_SUB_SEC_SEDCOND_INC_POS        (16U)
#define RBA_ETH_MAC_SUB_SEC_SECOND_INC_MASK        (0x00FF0000UL)


/*Opeartion Values*/
#define RBA_ETH_MAC_TIME_STAMP_EN               (1UL)
#define RBA_ETH_MAC_TIME_STAMP_RECEIVE_ENALL    (1UL)
#define RBA_ETH_MAC_TIME_STAMP_INIT             (1UL)
#define RBA_ETH_MAC_TIME_STAMP_IPENA            (1UL)
#define RBA_ETH_MAC_TIME_STAMP_VER2             (1UL)
#define RBA_ETH_MAC_TIME_STAMP_CTRLSR           (1UL)
#define RBA_ETH_MAC_TIME_STAMP_SNAPSEL          (3UL)
#define RBA_ETH_MAC_TIME_STAMP_MACADDR          (1UL)
#define RBA_ETH_MAC_TIME_STAMP_UPDATE           (1UL)

/*Time Stamp Status Register*/
#define RBA_ETH_MAC_STATUS_TIME_SECOND_OVF              (0UL)
#define RBA_ETH_MAC_STATUS_TIME_TARGET_REACHED          (1UL)
#define RBA_ETH_MAC_STATUS_TIME_AUX_TRIGGER             (2UL)
#define RBA_ETH_MAC_STATUS_TIME_TARGET_ERR0             (3UL)
#define RBA_ETH_MAC_STATUS_TX_TIME_STAMP                (15UL)


#define RBA_ETH_MAC_STATUS_TIME_AUX_SNAPSHOT_TRG_IDFR_POS    (16U)
#define RBA_ETH_MAC_STATUS_TIME_AUX_SNAPSHOT_TRG_IDFR_MASK   (0x3U << RBA_ETH_MAC_STATUS_TIME_AUX_SNAPSHOT_TRG_IDFR_POS)
#define RBA_ETH_MAC_STATUS_TIME_AUX_SNAPSHOT_MISSED          (24U)
#define RBA_ETH_MAC_STATUS_TIME_NUMBEROF_AUX_SNAPSHOT_POS    (25U)
#define RBA_ETH_MAC_STATUS_TIME_NUMBEROF_AUX_SNAPSHOT_MASK   (0x1FU << RBA_ETH_MAC_STATUS_TIME_NUMBEROF_AUX_SNAPSHOT_POS)

#define RBA_ETH_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE       (31U)
#define RBA_ETH_MAC_SYSTEM_TIME_NANOSECONDS_ADD_SUB_MASK (1UL << RBA_ETH_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE)
#define RBA_ETH_MAC_TIMESTAMP_NS_UPRLIMIT                    (0x3B9AC9FFUL)

#define RBA_ETH_MAC_MAX_NS_UPDATE_TSCTRL_SET              (1000000000UL)
#define RBA_ETH_MAC_SECOND_CORRECTION                     (0xFFFFFFFFUL)

#define NORMAL_DESCRIPTOR_TYPE    (0U)
#define CONTEXT_DESCRIPTOR_TYPE   (1U)


/* This MACRO defines that data buffer address for context descriptor is 0 */
#define DATA_BUFFER_ADDRESS_FOR_CONTEXT_DESCRIPTOR (0U)

/*
 ***************************************************************************************************
 * Extern declarations
 ***************************************************************************************************
 */

#endif /* RBA_ETH_PRV_H_ */

