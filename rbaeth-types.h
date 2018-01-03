#ifndef RBA_ETH_TYPES_H_
#define RBA_ETH_TYPES_H_

/*
 ***************************************************************************************************
 * Includes
 ***************************************************************************************************
 */

#include "Eth_GeneralTypes.h"

/*
 *  SW version checks
 */

/*
 ***************************************************************************************************
 * Defines
 ***************************************************************************************************
 */

#define RBA_ETH_MAC_HWMACADDR_GROUP1           (32UL)

/*
 ***************************************************************************************************
 * Type definitions
 ***************************************************************************************************
 */

typedef struct
{
    volatile uint32 high_u32;
    volatile uint32 low_u32;
} rba_Eth_AddrRegisterType_tst;


/* register map structure of the Ethernet MAC (controller) */
typedef struct
{
    volatile uint32 MAC_CONFIGURATION;                                          /* [0x0000]  :   MAC Configuration register */
    volatile uint32 MAC_EXT_CONFIGURATION;                                      /* [0x0004]  :   MAC Extended Configuration register */
    volatile uint32 MAC_PACKET_FILTER;                                          /* [0x0008]  :   MAC Packet Filter register */
    volatile uint32 MAC_WATCHDOG_TIMEOUT;                                       /* [0x000C]  :   Watchdog Timeout register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_HASH_TABLE_REG0;                                        /* [0x0010]  :   Hash Table register 0 */
    volatile uint32 MAC_HASH_TABLE_REG1;                                        /* [0x0014]  :   Hash Table register 1 */
    volatile uint32 RESERVED1[14];                                              /* [0x0018...0x004F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED1_IFX5B[16];                                        /* [0x0010...0x004F]  :   Reserved */
#endif
    volatile uint32 MAC_VLAN_TAG;                                               /* [0x0050]  :   VLAN Tag register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED2;                                                  /* [0x0054...0x0057]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_VLAN_TAG_DATA;                                          /* [0x0054]  :   MAC VLAN Tag Data Register */
#endif
    volatile uint32 MAC_VLAN_HASH_TABLE;                                        /* [0x0058]  :   VLAN Hash Table register */
    volatile uint32 RESERVED3;                                                  /* [0x005C...0x005F]  :   Reserved */
    volatile uint32 MAC_VLAN_INCL;                                              /* [0x0060]  :   VLAN Tag Inclusion register */
    volatile uint32 MAC_INNER_VLAN_INCL;                                        /* [0x0064]  :   Inner VLAN Tag Inclusion register */
    volatile uint32 RESERVED4[2];                                               /* [0x0068...0x006F]  :   Reserved */
    volatile uint32 MAC_Q0_TX_FLOW_CTRL;                                        /* [0x0070]  :   MAC Q0 Flow Control register  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_Q1_TX_FLOW_CTRL;                                        /* [0x0074]  :   MAC Q1 Flow Control register  */
    volatile uint32 RESERVED5[6];                                               /* [0x0078...0x008F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED2_IFX5B[7];                                         /* [0x0074...0x008F]  :   Reserved */
#endif
    volatile uint32 MAC_RX_FLOW_CTRL;                                           /* [0x0090]  :   MAC Receive Flow Control register  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED6;                                                  /* [0x0094...0x0097]  :   Reserved */
    volatile uint32 MAC_TXQ_PRTY_MAP0;                                          /* [0x0098]  :   Transmit Queue Priority Mapping 0 register  */
    volatile uint32 RESERVED7;                                                  /* [0x009C...0x009F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED3_IFX5B[3];                                         /* [0x0094...0x009F]  :   Reserved */
#endif
    volatile uint32 MAC_RXQ_CTRL0;                                              /* [0x00A0]  :   Receive Queue Control 0 register  */
    volatile uint32 MAC_RXQ_CTRL1;                                              /* [0x00A4]  :   Receive Queue Control 1 register  */
    volatile uint32 MAC_RXQ_CTRL2;                                              /* [0x00A8]  :   Receive Queue Control 2 register  */
    volatile uint32 RESERVED8;                                                  /* [0x00AC...0x00AF]  :   Reserved */
    volatile uint32 MAC_INTERRUPT_STATUS;                                       /* [0x00B0]  :   Interrupt Status register  */
    volatile uint32 MAC_INTERRUPT_ENABLE;                                       /* [0x00B4]  :   Interrupt Enable register  */
    volatile uint32 MAC_RX_TX_STATUS;                                           /* [0x00B8]  :   Receive Transmit Status register  */
    volatile uint32 RESERVED9;                                                  /* [0x00BC...0x00BF]  :   Reserved */
    volatile uint32 MAC_PMT_CONTROL_STATUS;                                     /* [0x00C0]  :   MAC PMT Control and Status Register */
    volatile uint32 MAC_RWK_PACKET_FILTER;                                      /* [0x00C4]  :  MAC Wake-up Packet Filter Register */
    volatile uint32 RESERVED10[2];                                              /* [0x00C8...0x00CF]  :   Register set for Remote Wake Up handling ?? */
    volatile uint32 MAC_LPI_CONTROL_STATUS;                                     /* [0x00D0]  :   LPI Control and Status register  */
    volatile uint32 MAC_LPI_TIMERS_CONTROL;                                     /* [0x00D4]  :   LPI Timers Control register  */
    volatile uint32 MAC_LPI_ENTRY_TIMER;                                        /* [0x00D8]  :   LPI Entry Timer register  */
    volatile uint32 MAC_1US_TIC_COUNTER;                                        /* [0x00DC]  :   1US Tic Counter register  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED11[12];                                             /* [0x00E0...0x010F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED4_IFX5B[6];                                         /* [0x00E0...0x0F7]  :   Reserved */
    volatile uint32 MAC_PHYIF_CONTROL_STATUS;                                   /* [0x00F8]  :   MAC PHY Interface Control and Status Register  */
    volatile uint32 RESERVED5_IFX5B[5];                                         /* [0x00FC...0x010F]  :   Reserved */
#endif
    volatile uint32 MAC_VERSION;                                                /* [0x0110]-  :   Version register  */
    volatile uint32 MAC_DEBUG;                                                  /* [0x0114]  :   Debug register  */
    volatile uint32 RESERVED12;                                                 /* [0x0118...0x011B]  :   Reserved */
    volatile uint32 MAC_HW_FEATURE0;                                            /* [0x011C]  :   Hardware Feature0 register  */
    volatile uint32 MAC_HW_FEATURE1;                                            /* [0x0120]  :   Hardware Feature1 register  */
    volatile uint32 MAC_HW_FEATURE2;                                            /* [0x0124]  :   Hardware Feature2 register  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED13[54];                                             /* [0x0128...0x01FF]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_HW_FEATURE3;                                            /* [0x0128]  :   Hardware Feature2 register  */
    volatile uint32 RESERVED6_IFX5B[53];                                        /* [0x012C...0x01FF]  :   Reserved */
#endif
    volatile uint32 MAC_MDIO_ADDRESS;                                           /* [0x0200]  :   MDIO Address register  */
    volatile uint32 MAC_MDIO_DATA;                                              /* [0x0204]  :   MDIO Data register  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED14[2];                                              /* [0x0208...0x021F]  :   Reserved */
    volatile uint32 MAC_ARP_ADDRESS;                                            /* [0x0210]  :   ARP Address register  */
    volatile uint32 RESERVED15[59];                                             /* [0x0214...0x02FF]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED7_IFX5B[62];                                        /* [0x0208...0x02FF]  :   Reserved */
#endif
	volatile rba_Eth_AddrRegisterType_tst GMA_MAC_ADDR[RBA_ETH_MAC_HWMACADDR_GROUP1];  /* MAC Address register from 0 to 31 all clubbed in the array*/
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_ADDRESS32_HIGH63_32;                                    /* [0x0400]  :   MAC Address High register 32 */
    volatile uint32 MAC_ADDRESS32_LOW63_32;                                     /* [0x0404]  :   MAC Address Low register 32 */
    volatile uint32 MAC_ADDRESS33_HIGH63_32;                                    /* [0x0408]  :   MAC Address High register 33 */
    volatile uint32 MAC_ADDRESS33_LOW63_32;                                     /* [0x040C]  :   MAC Address Low register 33 */
    volatile uint32 MAC_ADDRESS34_HIGH63_32;                                    /* [0x0410]  :   MAC Address High register 34 */
    volatile uint32 MAC_ADDRESS34_LOW63_32;                                     /* [0x0414]  :   MAC Address Low register 34 */
    volatile uint32 MAC_ADDRESS35_HIGH63_32;                                    /* [0x0418]  :   MAC Address High register 35 */
    volatile uint32 MAC_ADDRESS35_LOW63_32;                                     /* [0x041C]  :   MAC Address Low register 35 */
    volatile uint32 MAC_ADDRESS36_HIGH63_32;                                    /* [0x0420]  :   MAC Address High register 36 */
    volatile uint32 MAC_ADDRESS36_LOW63_32;                                     /* [0x0424]  :   MAC Address Low register 36 */
    volatile uint32 MAC_ADDRESS37_HIGH63_32;                                    /* [0x0428]  :   MAC Address High register 37 */
    volatile uint32 MAC_ADDRESS37_LOW63_32;                                     /* [0x042C]  :   MAC Address Low register 37 */
    volatile uint32 MAC_ADDRESS38_HIGH63_32;                                    /* [0x0430]  :   MAC Address High register 38 */
    volatile uint32 MAC_ADDRESS38_LOW63_32;                                     /* [0x0434]  :   MAC Address Low register 38 */
    volatile uint32 MAC_ADDRESS39_HIGH63_32;                                    /* [0x0438]  :   MAC Address High register 39 */
    volatile uint32 MAC_ADDRESS39_LOW63_32;                                     /* [0x043C]  :   MAC Address Low register 39 */
    volatile uint32 MAC_ADDRESS40_HIGH63_32;                                    /* [0x0440]  :   MAC Address High register 40 */
    volatile uint32 MAC_ADDRESS40_LOW63_32;                                     /* [0x0444]  :   MAC Address Low register 40 */
    volatile uint32 MAC_ADDRESS41_HIGH63_32;                                    /* [0x0448]  :   MAC Address High register 41 */
    volatile uint32 MAC_ADDRESS41_LOW63_32;                                     /* [0x044C]  :   MAC Address Low register 41 */
    volatile uint32 MAC_ADDRESS42_HIGH63_32;                                    /* [0x0450]  :   MAC Address High register 42 */
    volatile uint32 MAC_ADDRESS42_LOW63_32;                                     /* [0x0454]  :   MAC Address Low register 42 */
    volatile uint32 MAC_ADDRESS43_HIGH63_32;                                    /* [0x0458]  :   MAC Address High register 43 */
    volatile uint32 MAC_ADDRESS43_LOW63_32;                                     /* [0x045C]  :   MAC Address Low register 43 */
    volatile uint32 MAC_ADDRESS44_HIGH63_32;                                    /* [0x0460]  :   MAC Address High register 44 */
    volatile uint32 MAC_ADDRESS44_LOW63_32;                                     /* [0x0464]  :   MAC Address Low register 44 */
    volatile uint32 MAC_ADDRESS45_HIGH63_32;                                    /* [0x0468]  :   MAC Address High register 45 */
    volatile uint32 MAC_ADDRESS45_LOW63_32;                                     /* [0x046C]  :   MAC Address Low register 45 */
    volatile uint32 MAC_ADDRESS46_HIGH63_32;                                    /* [0x0470]  :   MAC Address High register 46 */
    volatile uint32 MAC_ADDRESS46_LOW63_32;                                     /* [0x0474]  :   MAC Address Low register 46 */
    volatile uint32 MAC_ADDRESS47_HIGH63_32;                                    /* [0x0478]  :   MAC Address High register 47 */
    volatile uint32 MAC_ADDRESS47_LOW63_32;                                     /* [0x047C]  :   MAC Address Low register 47 */
    volatile uint32 MAC_ADDRESS48_HIGH63_32;                                    /* [0x0480]  :   MAC Address High register 48 */
    volatile uint32 MAC_ADDRESS48_LOW63_32;                                     /* [0x0484]  :   MAC Address Low register 48 */
    volatile uint32 MAC_ADDRESS49_HIGH63_32;                                    /* [0x0488]  :   MAC Address High register 49 */
    volatile uint32 MAC_ADDRESS49_LOW63_32;                                     /* [0x048C]  :   MAC Address Low register 49 */
    volatile uint32 MAC_ADDRESS50_HIGH63_32;                                    /* [0x0490]  :   MAC Address High register 50 */
    volatile uint32 MAC_ADDRESS50_LOW63_32;                                     /* [0x0494]  :   MAC Address Low register 50 */
    volatile uint32 MAC_ADDRESS51_HIGH63_32;                                    /* [0x0498]  :   MAC Address High register 51 */
    volatile uint32 MAC_ADDRESS51_LOW63_32;                                     /* [0x049C]  :   MAC Address Low register 51 */
    volatile uint32 MAC_ADDRESS52_HIGH63_32;                                    /* [0x04A0]  :   MAC Address High register 52 */
    volatile uint32 MAC_ADDRESS52_LOW63_32;                                     /* [0x04A4]  :   MAC Address Low register 52 */
    volatile uint32 MAC_ADDRESS53_HIGH63_32;                                    /* [0x04A8]  :   MAC Address High register 53 */
    volatile uint32 MAC_ADDRESS53_LOW63_32;                                     /* [0x04AC]  :   MAC Address Low register 53 */
    volatile uint32 MAC_ADDRESS54_HIGH63_32;                                    /* [0x04B0]  :   MAC Address High register 54 */
    volatile uint32 MAC_ADDRESS54_LOW63_32;                                     /* [0x04B4]  :   MAC Address Low register 54 */
    volatile uint32 MAC_ADDRESS55_HIGH63_32;                                    /* [0x04B8]  :   MAC Address High register 55 */
    volatile uint32 MAC_ADDRESS55_LOW63_32;                                     /* [0x04BC]  :   MAC Address Low register 55 */
    volatile uint32 MAC_ADDRESS56_HIGH63_32;                                    /* [0x04C0]  :   MAC Address High register 56 */
    volatile uint32 MAC_ADDRESS56_LOW63_32;                                     /* [0x04C4]  :   MAC Address Low register 56 */
    volatile uint32 MAC_ADDRESS57_HIGH63_32;                                    /* [0x04C8]  :   MAC Address High register 57 */
    volatile uint32 MAC_ADDRESS57_LOW63_32;                                     /* [0x04CC]  :   MAC Address Low register 57 */
    volatile uint32 MAC_ADDRESS58_HIGH63_32;                                    /* [0x04D0]  :   MAC Address High register 58 */
    volatile uint32 MAC_ADDRESS58_LOW63_32;                                     /* [0x04D4]  :   MAC Address Low register 58 */
    volatile uint32 MAC_ADDRESS59_HIGH63_32;                                    /* [0x04D8]  :   MAC Address High register 59 */
    volatile uint32 MAC_ADDRESS59_LOW63_32;                                     /* [0x04DC]  :   MAC Address Low register 59 */
    volatile uint32 MAC_ADDRESS60_HIGH63_32;                                    /* [0x04E0]  :   MAC Address High register 60 */
    volatile uint32 MAC_ADDRESS60_LOW63_32;                                     /* [0x04E4]  :   MAC Address Low register 60 */
    volatile uint32 MAC_ADDRESS61_HIGH63_32;                                    /* [0x04E8]  :   MAC Address High register 61 */
    volatile uint32 MAC_ADDRESS61_LOW63_32;                                     /* [0x04EC]  :   MAC Address Low register 61 */
    volatile uint32 MAC_ADDRESS62_HIGH63_32;                                    /* [0x04F0]  :   MAC Address High register 62 */
    volatile uint32 MAC_ADDRESS62_LOW63_32;                                     /* [0x04F4]  :   MAC Address Low register 62 */
    volatile uint32 MAC_ADDRESS63_HIGH63_32;                                    /* [0x04F8]  :   MAC Address High register 62 */
    volatile uint32 MAC_ADDRESS63_LOW63_32;                                     /* [0x04FC]  :   MAC Address Low register 62 */
    volatile uint32 RESERVED16[128];                                            /* [0x0500...0x6FC]  :   MAC Address High and Low registers 64 to 127 */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED8_IFX5B[192];                                       /* [0x0400...0x6FC]  :   Reserved */
#endif
    volatile uint32 MMC_CONTROL;                                                /* [0x0700] :   MMC Control Register */
    volatile uint32 MMC_RX_INTERRUPT;                                           /* [0x0704] :   MMC Receive Interrupt Register */
    volatile uint32 MMC_TX_INTERRUPT;                                           /* [0x0708] :   MMC Transmit Interrupt Register */
    volatile uint32 MMC_RX_INTERRUPT_MASK;                                      /* [0x070C] :   MMC Receive Interrupt Mask Register */
    volatile uint32 MMC_TX_INTERRUPT_MASK;                                      /* [0x0710] :   MMC Transmit Interrupt Mask Register */
    volatile uint32 TX_OCTET_COUNT_GOOD_BAD;                                    /* [0x0714] :   Transmit Octet Count for Good and Bad Frames */
    volatile uint32 TX_PACKET_COUNT_GOOD_BAD;                                   /* [0x0718] :   Transmit Packet Count Good Bad register */
    volatile uint32 TX_BROADCAST_PACKETS_GOOD;                                  /* [0x071C] :   Transmit Broadcast Packets Good register */
    volatile uint32 TX_MULTICAST_PACKETS_GOOD;                                  /* [0x0720] :   Transmit Multicast Packets Good register */
    volatile uint32 TX_64OCTETS_PACKETS_GOOD_BAD;                               /* [0x0724] :   Transmit 64Octets Packets Good Bad register */
    volatile uint32 TX_65TO127OCTETS_PACKETS_GOOD_BAD;                          /* [0x0728] :   Transmit 65to127Octets Packets Good Bad register */
    volatile uint32 TX_128TO255OCTETS_PACKETS_GOOD_BAD;                         /* [0x072C] :   Transmit 128To255Octets Packets Good Bad register */
    volatile uint32 TX_256TO511OCTETS_PACKETS_GOOD_BAD;                         /* [0x0730] :   Transmit 256To511Octets Packets Good Bad register */
    volatile uint32 TX_512TO1023OCTETS_PACKETS_GOOD_BAD;                        /* [0x0734] :   Transmit 512To1023Octets Packets Good Bad register */
    volatile uint32 TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD;                        /* [0x0738] :   Transmit 1024ToMaxOctets Packets Good Bad register */
    volatile uint32 TX_UNICAST_PACKETS_GOOD_BAD;                                /* [0x073C] :   Transmit Unicast Packets Good Bad register */
    volatile uint32 TX_MULTICAST_PACKETS_GOOD_BAD;                              /* [0x0740] :   Transmit Multicast Packets Good Bad register */
    volatile uint32 TX_BROADCAST_PACKETS_GOOD_BAD;                              /* [0x0744] :   Transmit Broadcast Packets Good Bad register */
    volatile uint32 TX_UNDERFLOW_ERROR_PACKETS;                                 /* [0x0748] :   Transmit Underflow Error Packets register */
    volatile uint32 TX_SINGLE_COLLISION_GOOD_PACKETS;                           /* [0x074C] :   Transmit Single Collision Good Packets register */
    volatile uint32 TX_MULTIPLE_COLLISION_GOOD_PACKETS;                         /* [0x0750] :   Transmit Multiple Collision Good Packets register */
    volatile uint32 TX_DEFERRED_PACKETS;                                        /* [0x0754] :   Transmit Deferred Packets register */
    volatile uint32 TX_LATE_COLLISION_PACKETS;                                  /* [0x0758] :   Transmit Late Collision Packets register */
    volatile uint32 TX_EXCESSIVE_COLLISION_PACKETS;                             /* [0x075C] :   Transmit Excessive Collision Packets register */
    volatile uint32 TX_CARRIER_ERROR_PACKETS;                                   /* [0x0760] :   Transmit Carrier Error Packets register */
    volatile uint32 TX_OCTET_COUNT_GOOD;                                        /* [0x0764] :   Transmit Octet Count Good register */
    volatile uint32 TX_PACKET_COUNT_GOOD;                                       /* [0x0768] :   Transmit Packet Count Good register */
    volatile uint32 TX_EXCESSIVE_DEFERRAL_ERROR;                                /* [0x076C] :   Transmit Excessive Deferral Error register */
    volatile uint32 TX_PAUSE_PACKETS;                                           /* [0x0770] :   Transmit Pause Packets register */
    volatile uint32 TX_VLAN_PACKETS_GOOD;                                       /* [0x0774] :   Transmit VLAN Packets Good register */
    volatile uint32 TX_OSIZE_PACKETS_GOOD;                                      /* [0x0778] :   Transmit OSize Packets Good register */
    volatile uint32 Reserved59;                                                 /* [0x077C....0x077F] : Reserved */
    volatile uint32 RX_PACKETS_COUNT_GOOD_BAD;                                  /* [0x0780] :   Receive Packets Count Good Bad register */
    volatile uint32 RX_OCTET_COUNT_GOOD_BAD;                                    /* [0x0784] :   Receive Octet Count Good Bad register */
    volatile uint32 RX_OCTET_COUNT_GOOD;                                        /* [0x0788] :   Receive Octet Count Good register */
    volatile uint32 RX_BROADCAST_PACKETS_GOOD;                                  /* [0x078C] :   Receive Broadcast Packets Good register */
    volatile uint32 RX_MULTICAST_PACKETS_GOOD;                                  /* [0x0790] :   Receive Multicast Packets Good register */
    volatile uint32 RX_CRC_ERROR_PACKETS;                                       /* [0x0794] :   Receive CRC Error Packets register */
    volatile uint32 RX_ALIGNMENT_ERROR_PACKETS;                                 /* [0x0798] :   Receive Alignment Error Packets register */
    volatile uint32 RX_RUNT_ERROR_PACKETS;                                      /* [0x079C] :   Receive Runt Error Packets register */
    volatile uint32 RX_JABBER_ERROR_PACKETS;                                    /* [0x07A0] :   Receive Jabber Error Packets register */
    volatile uint32 RX_UNDERSIZE_PACKETS_GOOD;                                  /* [0x07A4] :   Receive Undersize Packets Good register */
    volatile uint32 RX_OVERSIZE_PACKETS_GOOD;                                   /* [0x07A8] :   Receive Oversize Packets Good register */
    volatile uint32 RX_64OCTETS_PACKETS_GOOD_BAD;                               /* [0x07AC] :   Receive 64Octets Packets Good Bad register */
    volatile uint32 RX_65TO127OCTETS_PACKETS_GOOD_BAD;                          /* [0x07B0] :   Receive 65To127Octets Packets Good Bad register */
    volatile uint32 RX_128TO255OCTETS_PACKETS_GOOD_BAD;                         /* [0x07B4] :   Receive 128To255Octets Packets Good Bad register */
    volatile uint32 RX_256TO511OCTETS_PACKETS_GOOD_BAD;                         /* [0x07B8] :   Receive 256To511Octets Packets Good Bad register */
    volatile uint32 RX_512TO1023OCTETS_PACKETS_GOOD_BAD;                        /* [0x07BC] :   Receive 512To1023Octets Packets Good Bad register */
    volatile uint32 RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD;                        /* [0x07C0] :   Receive 1024ToMax Octets Packets Good bad register */
    volatile uint32 RX_UNICAST_PACKETS_GOOD;                                    /* [0x07C4] :   Receive Unicast Packets Good register */
    volatile uint32 RX_LENGTH_ERROR_PACKETS;                                    /* [0x07C8] :   Receive Length Error Packets register */
    volatile uint32 RX_OUT_OF_RANGE_TYPE_PACKETS;                               /* [0x07CC] :   Receive Out Of range Type Packets register */
    volatile uint32 RX_PAUSE_PACKETS;                                           /* [0x07D0] :   Receive Pause Packets register */
    volatile uint32 RX_FIFO_OVERFLOW_PACKETS;                                   /* [0x07D4] :   Receive FIFO Overflow Packets register */
    volatile uint32 RX_VLAN_PACKETS_GOOD_BAD;                                   /* [0x07D8] :   Receive VLAN Packets Good Bad register */
    volatile uint32 RX_WATCHDOG_ERROR_PACKETS;                                  /* [0x07DC] :   Receive Watchdog Error Packets register */
    volatile uint32 RX_RECEIVE_ERROR_PACKETS;                                   /* [0x07E0] :   Receive Error Packets register */
    volatile uint32 RX_CONTROL_PACKETS_GOOD;                                    /* [0x07E4] :   Receive Control Packets Good register */
    volatile uint32 Reserved54;                                                 /* [0x07E8...0x07EB]  :   Reserved */
    volatile uint32 TX_LPI_USEC_CNTR;                                           /* [0x07EC] :   Transmit LPI USEC Counter register */
    volatile uint32 TX_LPI_TRAN_CNTR;                                           /* [0x07F0] :   Transmit LPI Transaction Counter register */
    volatile uint32 RX_LPI_USEC_CNTR;                                           /* [0x07F4] :   Receive LPI USEC Counter register */
    volatile uint32 RX_LPI_TRAN_CNTR;                                           /* [0x07F8] :   Receive LPI Transaction Counter register */
    volatile uint32 Reserved55;                                                 /* [0x07FC...0x07FF] : Reserved */
    volatile uint32 MMC_IPC_RX_INTERRUPT_MASK;                                  /* [0x0800] :   MMC IPC Receive Interrupt Mask register */
    volatile uint32 Reserved56;                                                 /* [0x0804...0x0807] : Reserved */
    volatile uint32 MMC_IPC_RX_INTERRUPT;                                       /* [0x0808] :   MMC IPC Receive Interrupt register */
    volatile uint32 Reserved57;                                                 /* [0x080C...0x080F] : Reserved */
    volatile uint32 RXIPV4_GOOD_PACKETS;                                        /* [0x0810] :   Receive IPv4 Good Packets register */
    volatile uint32 RXIPV4_HEADER_ERROR_PACKETS;                                /* [0x0814] :   Receive IPv4 Header Error Packets */
    volatile uint32 RXIPV4_NO_PAYLOAD_PACKETS;                                  /* [0x0818] :   Receive IPv4 No Payload Packets register */
    volatile uint32 RXIPV4_FRAGMENTED_PACKETS;                                  /* [0x081C] :   Receive IPv4 Fragmented Packets register */
    volatile uint32 RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS;                       /* [0x0820] :   Receive IPv4 UDP Checksum Disabled Packets register */
    volatile uint32 RXIPV6_GOOD_PACKETS;                                        /* [0x0824] :   Receive IPv6 Good Packets register */
    volatile uint32 RXIPV6_HEADER_ERROR_PACKETS;                                /* [0x0828] :   Receive IPv6 Header Error Packets register */
    volatile uint32 RXIPV6_NO_PAYLOAD_PACKETS;                                  /* [0x082C] :   Receive IPv6 Payload Packets register */
    volatile uint32 RXUDP_GOOD_PACKETS;                                         /* [0x0830] :   Receive UDP Good Packets register */
    volatile uint32 RXUDP_ERROR_PACKETS;                                        /* [0x0834] :   Receive UDP Error Packets register */
    volatile uint32 RXTCP_GOOD_PACKETS;                                         /* [0x0838] :   Receive TCP Good Packets register */
    volatile uint32 RXTCP_ERROR_PACKETS;                                        /* [0x083C] :   Receive TCP Error Packets register */
    volatile uint32 RXICMP_GOOD_PACKETS;                                        /* [0x0840] :   Receive ICMP Good Packets register */
    volatile uint32 RXICMP_ERROR_PACKETS;                                       /* [0x0844] :   Receive ICMP Error Packets register */
    volatile uint32 Reserved58[2];                                                 /* [0x0848...0x084F] : Reserved */
    volatile uint32 RXIPV4_GOOD_OCTETS;                                         /* [0x0850] :   Receive IPv4 Good Octets register */
    volatile uint32 RXIPV4_HEADER_ERROR_OCTETS;                                 /* [0x0854] :   Receive IPv4 Header Error Octets register */
    volatile uint32 RXIPV4_NO_PAYLOAD_OCTETS;                                   /* [0x0858] :   Receive IPv4 No Payload Octets register */
    volatile uint32 RXIPV4_FRAGMENTED_OCTETS;                                   /* [0x085C] :   Receive IPv4 Fragmented Octets register */
    volatile uint32 RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS;                         /* [0x0860] :   Receive IPv4 UDP Checksum Disable Octets register */
    volatile uint32 RXIPV6_GOOD_OCTETS;                                         /* [0x0864] :   Receive IPv6 Good Octets register */
    volatile uint32 RXIPV6_HEADER_ERROR_OCTETS;                                 /* [0x0868] :   Receive Header Error Octets register */
    volatile uint32 RXIPV6_NO_PAYLOAD_OCTETS;                                   /* [0x086C] :   Receive IPv6 No Payload Octets register */
    volatile uint32 RXUDP_GOOD_OCTETS;                                          /* [0x0870] :   Receive UDP Good Octets register */
    volatile uint32 RXUDP_ERROR_OCTETS;                                         /* [0x0874] :   Receive UDP Error Octets register */
    volatile uint32 RXTCP_GOOD_OCTETS;                                          /* [0x0878] :   Receive TCP Good Octets register */
    volatile uint32 RXTCP_ERROR_OCTETS;                                         /* [0x087C] :   Receive TCP Error Octets register */
    volatile uint32 RXICMP_GOOD_OCTETS;                                         /* [0x0880] :   Receive ICMP Good Octets register */
    volatile uint32 RXICMP_ERROR_OCTETS;                                        /* [0x0884] :   Receive ICMP Error Octets register */
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED9_IFX5B[158];                                       /* [0x0888...0x0AFF]  :   Reserved */
#endif
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED18[30];                                             /* [0x0888...0x08FF]  :   Reserved */
    volatile uint32 MAC_L3_L4_CONTROL0;                                         /* [0x0900]  :   Layer 3 and Layer 4 Control 0 register  */
    volatile uint32 MAC_LAYER4_ADDRESS0;                                        /* [0x0904]  :   Layer 4 Address 0 register  */
    volatile uint32 RESERVED19[2];                                              /* [0x0908...0x090F]  :   Reserved */
    volatile uint32 MAC_LAYER3_ADDRESS0_REG0;                                   /* [0x0910]  :   Layer 3 Address 0 Register 0  */
    volatile uint32 MAC_LAYER3_ADDRESS1_REG0;                                   /* [0x0914]  :   Layer3 Address1 Register 0  */
    volatile uint32 MAC_LAYER3_ADDRESS2_REG0;                                   /* [0x0918]  :   Layer3 Address2 Register 0  */
    volatile uint32 MAC_LAYER3_ADDRESS3_REG0;                                   /* [0x091C]  :   Layer3 Address3 Register 0  */
    volatile uint32 RESERVED20[120];                                            /* [0x0920...0x0AFF]  :   Reserved */
#endif
    volatile uint32 MAC_TIMESTAMP_CONTROL;                                      /* [0x0B00]  :   Timestamp Control register  */
    volatile uint32 MAC_SUB_SECOND_INCREMENT;                                   /* [0x0B04]  :   Sub Second Increment register  */
    volatile uint32 MAC_SYSTEM_TIME_SECONDS;                                    /* [0x0B08]  :   System Time Seconds register  */
    volatile uint32 MAC_SYSTEM_TIME_NANOSECONDS;                                /* [0x0B0C]  :   System Time Nanoseconds register */
    volatile uint32 MAC_SYSTEM_TIME_SECONDS_UPDATE;                             /* [0x0B10]  :   System Time Seconds Update register */
    volatile uint32 MAC_SYSTEM_TIME_NANOSECONDS_UPDATE;                         /* [0x0B14]  :   System Time Nanoseconds Update register */
    volatile uint32 MAC_TIMESTAMP_ADDEND;                                       /* [0x0B18]  :   Timestamp Addend register  */
    volatile uint32 MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS;                        /* [0x0B1C]  :   System Time Higher Word Seconds register */
    volatile uint32 MAC_TIMESTAMP_STATUS;                                       /* [0x0B20]  :   MAC Timestamp Status register  */
    volatile uint32 RESERVED21[3];                                              /* [0x0B24...0x0B2F]  :   Reserved */
    volatile uint32 MAC_TX_TIMESTAMP_STATUS_NANOSECONDS;                        /* [0x0B30]  :   MAC Transmit Timestamp Status Nanoseconds register */
    volatile uint32 MAC_TX_TIMESTAMP_STATUS_SECONDS;                            /* [0x0B34]  :   MAC Tx Transmit Timestamp Status Seconds register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED22[2];                                              /* [0x0B38...0x0B3F]  :   Reserved */
    volatile uint32 MAC_AUXILIARY_CONTROL;                                      /* [0x0B40]  :   MAC Auxiliary Control register  */
    volatile uint32 RESERVED23;                                                 /* [0x0B44...0x0B47]  :   Reserved */
    volatile uint32 MAC_AUXILIARY_TIMESTAMP_NANOSECONDS;                        /* [0x0B48]  :   MAC Auxiliary Timestamp Nanoseconds register */
    volatile uint32 MAC_AUXILIARY_TIMESTAMP_SECONDS;                            /* [0x0B4C]  :   MAC Auxiliary Timestamp Seconds register */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED10_IFX5B[6];                                        /* [0x0B38...0x0B4F]  :   Reserved */
#endif
    volatile uint32 MAC_TIMESTAMP_INGRESS_ASYM_CORR;                            /* [0x0B50]  :   MAC Timestamp Ingress Asymmetry Correction register */
    volatile uint32 MAC_TIMESTAMP_EGRESS_ASYM_CORR;                             /* [0x0B54]  :   MAC Timestamp Egress Asymmetry Correction register */
    volatile uint32 MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND;                      /* [0x0B58]  :   MAC Timestamp Ingress Correction Nanosecond register */
    volatile uint32 MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND;                       /* [0x0B5C]  :   MAC Timestamp Egress Correction Nanosecond register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED24[4];                                              /* [0x0B60...0x0B6F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC;                      /* [0x0B60]  :   MAC Timestamp Ingress Correction Subnanoseconds Register */
    volatile uint32 MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC;                       /* [0x0B64]  :   MAC Timestamp Egress Correction Subnanoseconds Register */
    volatile uint32 RESERVED11_IFX5B[2];                                        /* [0x0B68 - 0x0B6F] :Reserved*/
#endif
    volatile uint32 MAC_PPS_CONTROL;                                            /* [0x0B70]  :   MAC_PPS_Control PPS Control register  */
    volatile uint32 RESERVED25[3];                                              /* [0x0B74...0x0B7F]  :   Reserved */
    volatile uint32 MAC_PPS0_TARGET_TIME_SECONDS;                               /* [0x0B80]  :   PPS0 Target Time Seconds register */
    volatile uint32 MAC_PPS0_TARGET_TIME_NANOSECONDS;                           /* [0x0B84]  :   PPS0 Target Time Nanoseconds register */
    volatile uint32 MAC_PPS0_INTERVAL;                                          /* [0x0B88]  :   PPS0 Interval register  */
    volatile uint32 MAC_PPS0_WIDTH;                                             /* [0x0B8C]  :   PPS0 Width register  */
    volatile uint32 RESERVED26[28];                                             /* [0x0B90...0x0BFF]  :   Reserved */
    volatile uint32 MTL_OPERATION_MODE;                                         /* [0x0C00]  :   Operation Mode register (MTL_OPERATION_MODE)  */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED27;                                                 /* [0x0C04...0x0C07]  :   Reserved */
    volatile uint32 MTL_DBG_CTL;                                                /* [0x0C08]  :   Debug Access Control register */
    volatile uint32 MTL_DBG_STS;                                                /* [0x0C0C]  :   Debug Status register */
    volatile uint32 MTL_FIFO_DEBUG_DATA;                                        /* [0x0C10]  :   FIFO Debug Data register */
    volatile uint32 RESERVED28[3];                                              /* [0x0C14...0x0C1F]  :   Reserved */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED12_IFX5B[7];                                        /* [0x0C04...0x0C1F]  :   Reserved */
#endif
    volatile uint32 MTL_INTERRUPT_STATUS;                                       /* [0x0C20]  :   Interrupt Status register */
    volatile uint32 RESERVED29[3];                                              /* [0x0C24...0x0C2F]  :   Reserved */
    volatile uint32 MTL_RXQ_DMA_MAP0;                                           /* [0x0C30]  :   Receive Queue and DMA Channel Mapping 0 register */
    volatile uint32 RESERVED30[51];                                             /* [0x0C34...0x0CFF]  :   Reserved */
    volatile uint32 MTL_TXQ0_OPERATION_MODE;                                    /* [0x0D00]  :   Queue 0 Transmit Operation Mode register */
    volatile uint32 MTL_TXQ0_UNDERFLOW;                                         /* [0x0D04]  :   Queue 0 Underflow Counter register */
    volatile uint32 MTL_TXQ0_DEBUG;                                             /* [0x0D08]  :   Queue 0 Transmit Debug register */
    volatile uint32 RESERVED31[2];                                              /* [0x0D0C...0x0D13]  :   Reserved */
    volatile uint32 MTL_TXQ0_ETS_STATUS;                                        /* [0x0D14]  :   Queue 0 ETS Status register */
    volatile uint32 MTL_TXQ0_QUANTUM_WEIGHT;                                    /* [0x0D18]  :   Queue 0 Quantum or Weights register */
    volatile uint32 RESERVED32[4];                                              /* [0x0D1C...0x0D2B]  :   Reserved */
    volatile uint32 MTL_Q0_INTERRUPT_CONTROL_STATUS;                            /* [0x0D2C]  :   Interrupt Control Status register */
    volatile uint32 MTL_RXQ0_OPERATION_MODE;                                    /* [0x0D30]  :   Queue 0 Receive Operation Mode register */
    volatile uint32 MTL_RXQ0_MISSED_PACKET_OVERFLOW_CNT;                        /* [0x0D34]  :   Queue 0 Missed Packet and Overflow Counter register */
    volatile uint32 MTL_RXQ0_DEBUG;                                             /* [0x0D38]  :   Queue 0 Receive Debug register */
    volatile uint32 MTL_RXQ0_CONTROL;                                           /* [0x0D3C]  :   Queue Receive Control register */
    volatile uint32 MTL_TXQ1_OPERATION_MODE;                                    /* [0x0D40]  :   Transmit Q1 Operation Mode register */
    volatile uint32 MTL_TXQ1_UNDERFLOW;                                         /* [0x0D44]  :   Transmit Q1 Underflow register */
    volatile uint32 MTL_TXQ1_DEBUG;                                             /* [0x0D48]  :   Transmit Q1 Debug register */
    volatile uint32 RESERVED33;                                                 /* [0x0D4C...0x0D4F]  :   Reserved */
    volatile uint32 MTL_TXQ1_ETS_CONTROL;                                       /* [0x0D50]  :   Transmit Q1 ETS Control register */
    volatile uint32 MTL_TXQ1_ETS_STATUS;                                        /* [0x0D54]  :   Transmit Q1 ETS Status register */
    volatile uint32 MTL_TXQ1_QUANTUM_WEIGHT;                                    /* [0x0D58]  :   Transmit Q1 Quantum Weight register */
    volatile uint32 MTL_TXQ1_SENDSPLOPECREDIT;                                  /* [0x0D5C]  :   Transmit Q1 Send Slope Credit register */
    volatile uint32 MTL_TXQ1_HICREDIT;                                          /* [0x0D60]  :   Transmit Q1 High Credit register */
    volatile uint32 MTL_TXQ1_LOCREDIT;                                          /* [0x0D64]  :   Transmit Qn Low Credit register */
    volatile uint32 RESERVED34;                                                 /* [0x0D68...0x0D6B]  :   Reserved */
    volatile uint32 MTL_Q1_INTERRUPT_CONTROL_STATUS;                            /* [0x0D6C]  :   Qn Interrupt Control Status register */
    volatile uint32 MTL_RXQ1_OPERATION_MODE;                                    /* [0x0D70]  :   Receive Qn Operation Mode register */
    volatile uint32 MTL_RXQ1_MISSED_PACKET_OVERFLOW_CNT;                        /* [0x0D74]  :   Receive Qn Missed Packet and Overflow Counter register */
    volatile uint32 MTL_RXQ1_DEBUG;                                             /* [0x0D78]  :   Receive Qn Debug register */
    volatile uint32 MTL_RXQ1_CONTROL;                                           /* [0x0D7C]  :   Receive Qn Control register */
    volatile uint32 RESERVED35[160];                                            /* [0x0D80...0x0FFF]  :   Reserved */
    volatile uint32 DMA_MODE;                                                   /* [0x1000]  :   DMA_Mode register */
    volatile uint32 DMA_SYSBUS_MODE;                                            /* [0x1004]  :   System Bus Mode register */
    volatile uint32 DMA_INTERRUPT_STATUS;                                       /* [0x1008]  :   Interrupt Status register */
    volatile uint32 DMA_DEBUG_STATUS0;                                          /* [0x100C]  :   Debug Status 0 register */
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 DMA_DEBUG_STATUS1;                                          /* [0x1010]  :   Debug Status 1 register */
    volatile uint32 RESERVED13_IFX5B[59];                                       /* [0x1014...0x10FF]  :   Reserved */
#endif
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED36[60];                                             /* [0x1010...0x10FF]  :   Reserved */
#endif
    volatile uint32 DMA_CH0_CONTROL;                                            /* [0x1100]  :   DMA Channel 0 Control register */
    volatile uint32 DMA_CH0_TX_CONTROL;                                         /* [0x1104]  :   DMA Channel 0 Transmit Control register */
    volatile uint32 DMA_CH0_RX_CONTROL;                                         /* [0x1108]  :   DMA Channel 0 Receive Control register */
    volatile uint32 RESERVED37[2];                                              /* [0x110C...0x1113]  :   Reserved */
    volatile uint32 DMA_CH0_TXDESC_LIST_ADDRESS;                                /* [0x1114]  :   DMA Channel 0 Transmit Descriptor List Address register */
    volatile uint32 RESERVED38;                                                 /* [0x1118...0x111B]  :   Reserved */
    volatile uint32 DMA_CH0_RXDESC_LIST_ADDRESS;                                /* [0x111C]  :   DMA Channel 0 Receive Descriptor List Address register */
    volatile uint32 DMA_CH0_TXDESC_TAIL_POINTER;                                /* [0x1120]  :   DMA Channel 0 Transmit Descriptor Tail Pointer register */
    volatile uint32 RESERVED39;                                                 /* [0x1124...0x1127]  :   Reserved */
    volatile uint32 DMA_CH0_RXDESC_TAIL_POINTER;                                /* [0x1128]  :   DMA Channel 0 Receive Descriptor Tail Pointer */
    volatile uint32 DMA_CH0_TXDESC_RING_LENGTH;                                 /* [0x112C]  :   DMA Channel 0 Transmit Descriptor Ring Length register */
    volatile uint32 DMA_CH0_RXDESC_RING_LENGTH;                                 /* [0x1130]  :   DMA Channel 0 Receive Descriptor Ring Length register */
    volatile uint32 DMA_CH0_INTERRUPT_ENABLE;                                   /* [0x1134]  :   DMA Channel 0 Interrupt Enable register */
    volatile uint32 DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER;                        /* [0x1138]  :   DMA Channel 0 Receive Interrupt Watchdog Timer register */
    volatile uint32 DMA_CH0_SLOT_FUNCTION_CONTROL_STATUS;                       /* [0x113C]  :   DMA Channel 0 Slot Function Control Status register */
    volatile uint32 RESERVED40;                                                 /* [0x1140...0x1143]  :   Reserved */
    volatile uint32 DMA_CH0_CURRENT_APP_TXDESC;                                 /* [0x1144]  :   DMA Channel 0 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED41;                                                 /* [0x1148...0x114B]  :   Reserved */
    volatile uint32 DMA_CH0_CURRENT_APP_RXDESC;                                 /* [0x114C]  :   DMA Channel 0 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED42;                                                 /* [0x1150...0x1153]  :   Reserved */
    volatile uint32 DMA_CH0_CURRENT_APP_TXBUFFER;                               /* [0x1154]  :   DMA Channel 0 Current Application Transmit Buffer register */
    volatile uint32 RESERVED43;                                                 /* [0x1158...0x115B]  :   Reserved */
    volatile uint32 DMA_CH0_CURRENT_APP_RXBUFFER;                               /* [0x115C]  :   DMA Channel 0 Current Application Receive Buffer register */
    volatile uint32 DMA_CH0_STATUS;                                             /* [0x1160]  :   DMA Channel 0 Status register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED44[2];                                              /* [0x1164...0x116B]  :   Reserved */
    volatile uint32 DMA_CH0_MISS_FRAME_CNT;                                     /* [0x116C]  :   DMA Channel 0 Miss Frame Counter register */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 DMA_CH0_MISS_FRAME_CNT;                                     /* [0x1164]  :   DMA Channel 0 Miss Frame Counter register */
    volatile uint32 RESERVED14_IFX5B[2];                                        /* [0x1168...0x116F]  :   DMA Channel 0 Miss Frame Counter register */
#endif
    volatile uint32 RESERVED45[4];                                              /* [0x1170...0x117F]  :   Reserved */
    volatile uint32 DMA_CH1_CONTROL;                                            /* [0x1180]  :   DMA Channel 1 Control register */
    volatile uint32 DMA_CH1_TX_CONTROL;                                         /* [0x1184]  :   DMA Channel 1 Transmit Control register */
    volatile uint32 DMA_CH1_RX_CONTROL;                                         /* [0x1188]  :   DMA Channel 1 Receive Control register */
    volatile uint32 RESERVED46[2];                                              /* [0x118C...0x1193]  :   Reserved */
    volatile uint32 DMA_CH1_TXDESC_LIST_ADDRESS;                                /* [0x1194]  :   DMA Channel 1 Transmit Descriptor List Address register */
    volatile uint32 RESERVED47;                                                 /* [0x1198...0x119B]  :   Reserved */
    volatile uint32 DMA_CH1_RXDESC_LIST_ADDRESS;                                /* [0x119C]  :   DMA Channel 1 Receive Descriptor List Address register */
    volatile uint32 DMA_CH1_TXDESC_TAIL_POINTER;                                /* [0x11A0]  :   DMA Channel 1 Transmit Descriptor Tail Pointer register */
    volatile uint32 RESERVED48;                                                 /* [0x11A4...0x11A7]  :   Reserved */
    volatile uint32 DMA_CH1_RXDESC_TAIL_POINTER;                                /* [0x11A8]  :   DMA Channel 1 Receive Descriptor Tail Pointer */
    volatile uint32 DMA_CH1_TXDESC_RING_LENGTH;                                 /* [0x11AC]  :   DMA Channel 1 Transmit Descriptor Ring Length register */
    volatile uint32 DMA_CH1_RXDESC_RING_LENGTH;                                 /* [0x11B0]  :   DMA Channel 1 Receive Descriptor Ring Length register */
    volatile uint32 DMA_CH1_INTERRUPT_ENABLE;                                   /* [0x11B4]  :   DMA Channel 1 Interrupt Enable register */
    volatile uint32 DMA_CH1_RX_INTERRUPT_WATCHDOG_TIMER;                        /* [0x11B8]  :   DMA Channel 1 Receive Interrupt Watchdog Timer register */
    volatile uint32 DMA_CH1_SLOT_FUNCTION_CONTROL_STATUS;                       /* [0x11BC]  :   DMA Channel 1 Slot Function Control Status register */
    volatile uint32 RESERVED49;                                                 /* [0x11C0...0x11C3]  :   Reserved */
    volatile uint32 DMA_CH1_CURRENT_APP_TXDESC;                                 /* [0x11C4]  :   DMA Channel 1 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED50;                                                 /* [0x11C8...0x11CB]  :   Reserved */
    volatile uint32 DMA_CH1_CURRENT_APP_RXDESC;                                 /* [0x11CC]  :   DMA Channel 1 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED51;                                                 /* [0x11D0...0x11D3]  :   Reserved */
    volatile uint32 DMA_CH1_CURRENT_APP_TXBUFFER;                               /* [0x11D4]  :   DMA Channel 1 Current Application Transmit Buffer register */
    volatile uint32 RESERVED52;                                                 /* [0x11D8...0x11DB]  :   Reserved */
    volatile uint32 DMA_CH1_CURRENT_APP_RXBUFFER;                               /* [0x11DC]  :   DMA Channel 1 Current Application Receive Buffer register */
    volatile uint32 DMA_CH1_STATUS;                                             /* [0x11E0]  :   DMA Channel 1 Status register */
#ifndef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 RESERVED53[2];                                              /* [0x11E4...0x11EB]  :   Reserved */
    volatile uint32 DMA_CH1_MISS_FRAME_CNT;                                     /* [0x11EC]  :   DMA Channel 1 Miss Frame Counter register */
#endif
#ifdef RBA_ETH_MCU_RB_IFX_UC1_DEV4_DEV5_40NM_UNICON
    volatile uint32 DMA_CH1_MISS_FRAME_CNT;                                     /* [0x11E4]  :   DMA Channel 1 Miss Frame Counter register */
    volatile uint32 RESERVED15_IFX5B[2];                                        /* [0x11E8...0x11EF]  :   DMA Channel 1 Miss Frame Counter register */
    volatile uint32 RESERVED16_IFX5B[4];                                        /* [0x11F0...0x11FF]  :   Reserved */
    volatile uint32 DMA_CH2_CONTROL;                                            /* [0x1200]  :   DMA Channel 2 Control register */
    volatile uint32 DMA_CH2_TX_CONTROL;                                         /* [0x1204]  :   DMA Channel 2 Transmit Control register */
    volatile uint32 DMA_CH2_RX_CONTROL;                                         /* [0x1208]  :   DMA Channel 2 Receive Control register */
    volatile uint32 RESERVED17_IFX5B[2];                                        /* [0x120C...0x1213]  :   Reserved */
    volatile uint32 DMA_CH2_TXDESC_LIST_ADDRESS;                                /* [0x1214]  :   DMA Channel 2 Transmit Descriptor List Address register */
    volatile uint32 RESERVED18_IFX5B;                                           /* [0x1218...0x121B]  :   Reserved */
    volatile uint32 DMA_CH2_RXDESC_LIST_ADDRESS;                                /* [0x121C]  :   DMA Channel 2 Receive Descriptor List Address register */
    volatile uint32 DMA_CH2_TXDESC_TAIL_POINTER;                                /* [0x1220]  :   DMA Channel 2 Transmit Descriptor Tail Pointer register */
    volatile uint32 RESERVED19_IFX5B;                                           /* [0x1224...0x1227]  :   Reserved */
    volatile uint32 DMA_CH2_RXDESC_TAIL_POINTER;                                /* [0x1228]  :   DMA Channel 2 Receive Descriptor Tail Pointer */
    volatile uint32 DMA_CH2_TXDESC_RING_LENGTH;                                 /* [0x122C]  :   DMA Channel 2 Transmit Descriptor Ring Length register */
    volatile uint32 DMA_CH2_RXDESC_RING_LENGTH;                                 /* [0x1230]  :   DMA Channel 2 Receive Descriptor Ring Length register */
    volatile uint32 DMA_CH2_INTERRUPT_ENABLE;                                   /* [0x1234]  :   DMA Channel 2 Interrupt Enable register */
    volatile uint32 DMA_CH2_RX_INTERRUPT_WATCHDOG_TIMER;                        /* [0x1238]  :   DMA Channel 2 Receive Interrupt Watchdog Timer register */
    volatile uint32 DMA_CH2_SLOT_FUNCTION_CONTROL_STATUS;                       /* [0x123C]  :   DMA Channel 2 Slot Function Control Status register */
    volatile uint32 RESERVED20_IFX5B;                                           /* [0x1240...0x1243]  :   Reserved */
    volatile uint32 DMA_CH2_CURRENT_APP_TXDESC;                                 /* [0x1244]  :   DMA Channel 2 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED21_IFX5B;                                           /* [0x1248...0x124B]  :   Reserved */
    volatile uint32 DMA_CH2_CURRENT_APP_RXDESC;                                 /* [0x124C]  :   DMA Channel 2 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED22_IFX5B;                                           /* [0x1250...0x1253]  :   Reserved */
    volatile uint32 DMA_CH2_CURRENT_APP_TXBUFFER;                               /* [0x1254]  :   DMA Channel 2 Current Application Transmit Buffer register */
    volatile uint32 RESERVED523_IFX5B;                                          /* [0x1258...0x125B]  :   Reserved */
    volatile uint32 DMA_CH2_CURRENT_APP_RXBUFFER;                               /* [0x125C]  :   DMA Channel 2 Current Application Receive Buffer register */
    volatile uint32 DMA_CH2_STATUS;                                             /* [0x1260]  :   DMA Channel 2 Status register */
    volatile uint32 DMA_CH2_MISS_FRAME_CNT;                                     /* [0x1264]  :   DMA Channel 2 Miss Frame Counter register */
    volatile uint32 RESERVED24_IFX5B[2];                                        /* [0x1268...0x126F]  :   Reserved */
    volatile uint32 RESERVED25_IFX5B[4];                                        /* [0x1270...0x127F]  :   Reserved */
    volatile uint32 DMA_CH3_CONTROL;                                            /* [0x1280]  :   DMA Channel 3 Control register */
    volatile uint32 DMA_CH3_TX_CONTROL;                                         /* [0x1284]  :   DMA Channel 3 Transmit Control register */
    volatile uint32 DMA_CH3_RX_CONTROL;                                         /* [0x1288]  :   DMA Channel 3 Receive Control register */
    volatile uint32 RESERVED26_IFX5B[2];                                        /* [0x128C...0x1293]  :   Reserved */
    volatile uint32 DMA_CH3_TXDESC_LIST_ADDRESS;                                /* [0x1294]  :   DMA Channel 3 Transmit Descriptor List Address register */
    volatile uint32 RESERVED27_IFX5B;                                           /* [0x1298...0x129B]  :   Reserved */
    volatile uint32 DMA_CH3_RXDESC_LIST_ADDRESS;                                /* [0x129C]  :   DMA Channel 3 Receive Descriptor List Address register */
    volatile uint32 DMA_CH3_TXDESC_TAIL_POINTER;                                /* [0x12A0]  :   DMA Channel 3 Transmit Descriptor Tail Pointer register */
    volatile uint32 RESERVED28_IFX5B;                                           /* [0x12A4...0x12A7]  :   Reserved */
    volatile uint32 DMA_CH3_RXDESC_TAIL_POINTER;                                /* [0x12A8]  :   DMA Channel 3 Receive Descriptor Tail Pointer */
    volatile uint32 DMA_CH3_TXDESC_RING_LENGTH;                                 /* [0x12AC]  :   DMA Channel 3 Transmit Descriptor Ring Length register */
    volatile uint32 DMA_CH3_RXDESC_RING_LENGTH;                                 /* [0x12B0]  :   DMA Channel 3 Receive Descriptor Ring Length register */
    volatile uint32 DMA_CH3_INTERRUPT_ENABLE;                                   /* [0x12B4]  :   DMA Channel 3 Interrupt Enable register */
    volatile uint32 DMA_CH3_RX_INTERRUPT_WATCHDOG_TIMER;                        /* [0x12B8]  :   DMA Channel 3 Receive Interrupt Watchdog Timer register */
    volatile uint32 DMA_CH3_SLOT_FUNCTION_CONTROL_STATUS;                       /* [0x12BC]  :   DMA Channel 3 Slot Function Control Status register */
    volatile uint32 RESERVED29_IFX5B;                                           /* [0x12C0...0x12C3]  :   Reserved */
    volatile uint32 DMA_CH3_CURRENT_APP_TXDESC;                                 /* [0x12C4]  :   DMA Channel 3 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED30_IFX5B;                                           /* [0x12C8...0x12CB]  :   Reserved */
    volatile uint32 DMA_CH3_CURRENT_APP_RXDESC;                                 /* [0x12CC]  :   DMA Channel 3 Current Application Transmit Descriptor register */
    volatile uint32 RESERVED31_IFX5B;                                           /* [0x12D0...0x12D3]  :   Reserved */
    volatile uint32 DMA_CH3_CURRENT_APP_TXBUFFER;                               /* [0x12D4]  :   DMA Channel 3 Current Application Transmit Buffer register */
    volatile uint32 RESERVED32_IFX5B;                                           /* [0x12D8...0x12DB]  :   Reserved */
    volatile uint32 DMA_CH3_CURRENT_APP_RXBUFFER;                               /* [0x12DC]  :   DMA Channel 3 Current Application Receive Buffer register */
    volatile uint32 DMA_CH3_STATUS;                                             /* [0x12E0]  :   DMA Channel 3 Status register */
    volatile uint32 DMA_CH3_MISS_FRAME_CNT;                                     /* [0x12E4]  :   DMA Channel 3 Miss Frame Counter register */
    volatile uint32 RESERVED33_IFX5B[2];                                        /* [0x12E8...0x12EF]  :   Reserved */
    volatile uint32 RESERVED34_IFX5B[836];                                      /* [0x12F0...0x1FFF] : Reserved */
    volatile uint32 CLC;                                                        /* [0x2000]  : Clock Control Register */
    volatile uint32 ID;                                                         /* [0x2004]  : Module Identification Register */
    volatile uint32 GPCTL;                                                      /* [0x2008]  : Input and Output Control Register */
    volatile uint32 ACCEN[2];                                                   /* [0x200C]  : Access Enable Register 0 */
    volatile uint32 KRST[2];                                                    /* [0x2014]  : Kernel Reset Register 0 */
    volatile uint32 KRSTCLR;                                                    /* [0x201C]  : Kernel Reset Status Clear Register */
    volatile uint32 ACCEND1;                                                    /* [0x2028]  : Access Enable Register 0 for DMA1 */
    volatile uint32 SKEWCTL;                                                    /* [0x2040]  : Skew Control Register */
    volatile uint32 MAC_TEST;                                                   /* [0x2044]  : ETHERMAC RGMII Test Register */
    volatile uint32 MAC_TEST_DLL;                                               /* [0x2048]  : ETHERMAC RGMII Test Register */
    volatile uint32 DLLCTL0;                                                    /* [0x204C]  : DLL Control Register 0  */
    volatile uint32 DLLCTL1;                                                    /* [0x2050]  : DLL Control Register 1  */
#endif

} rba_Eth_RegisterMapType_tst;

/*pointer to register map structure of the Ethernet MAC (controller) */
typedef rba_Eth_RegisterMapType_tst *rba_Eth_RegisterMapRefType_t;

/* structure of a descriptor; exchange structure between HW and SW which is used to set/read out
 * the descriptor registers. Internal structure of RX and TX descriptors differs, Also the size is 8 * 16bytes for Rx and 8 * 4 for Tx */
typedef struct
{
    volatile uint32 Des0_DataBuff1Addr_VlanTag_u32;
    volatile uint32 Des1_Reserved_Status_u32;
    volatile uint32 Des2_DataBuff2Addr_Status__u32;
    volatile uint32 Des3_CtrlStatus_u32;
} rba_Eth_RxBufferDescriptorType_tst;

/*Pointer to structure of a descriptor */
typedef rba_Eth_RxBufferDescriptorType_tst *rba_Eth_RxBufferDescriptorRefType_t;

typedef struct
{
    volatile uint32 Des0_DataBuff1Addr_TimeStampLow_u32;
    volatile uint32 Des1_DataBuff2Addr_TimeStampHigh_u32;
    volatile uint32 Des2_Ctrl_u32;
    volatile uint32 Des3_CtrlStatus_u32;
} rba_Eth_TxBufferDescriptorType_tst;

/*Pointer to structure of a descriptor */
typedef rba_Eth_TxBufferDescriptorType_tst *rba_Eth_TxBufferDescriptorRefType_t;

#if defined(RBA_ETH_EN_MII) && defined(RBA_ETH_ASYNCMII_SUPPORT)
#if ((RBA_ETH_EN_MII == STD_ON) && (RBA_ETH_ASYNCMII_SUPPORT == STD_ON))
/* enum for the type of function called */
typedef enum
{
    RBA_ETH_NONE = 0,
    RBA_ETH_WRITEMII = 1,
    RBA_ETH_READMII = 2
} rba_Eth_MiiOperationType_ten;

/* structure for storing controller index, function called, and tick value */
typedef struct
{
    uint8 rba_Eth_MiiCtrlIdx_u8;
    rba_Eth_MiiOperationType_ten rba_Eth_MiiOpType_en;
    uint32 rba_Eth_ReadWriteMiiGptClkTick_u32;
} rba_Eth_ReadWriteMiiManag_tst;

/* pointer to structure of readwrite mii management */
typedef rba_Eth_ReadWriteMiiManag_tst* rba_Eth_ReadWriteMiiManagRefType_t;
#endif
#endif

/*
 ***************************************************************************************************
 * Extern declarations
 ***************************************************************************************************
 */

#endif /* RBA_ETH_TYPES_H_ */
