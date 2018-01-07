#ifndef ETH_H_INCLUDED
#define ETH_H_INCLUDED

#include<stdio.h>
#include<stdlib.h>
#include<stdint-gcc.h>
#include<stdbool.h>

typedef enum
{
    ETH_BUFFER_FREE =0,
    ETH_BUFFER_LOCKED=1,
    ETH_BUFFER_LINKED=2
}EthTxBuffStates;

typedef enum
{
    ETH_UNINIT=0,
    ETH_STATE_ACTIVE=1
}EthStateType;

EthStateType Eth_ControllerState[1];
typedef enum
{
    ETH_MODE_DOWN=0,
    ETH_MODE_ACTIVE=1
}EthModeype;
EthStateType Eth_ConrollerMode[1];
typedef struct
{
    int *Eth_Phy_addr;
    int *Eth_Rx_Desc;
    int *Eth_Tx_Desc;
    int *Eth_Rx_Buff;
    int *Eth_Tx_Buff;
    bool *EthTxConfFlag;
    EthTxBuffStates *TxBuffStateTable;
    int Eth_RxBuff_Len;
    int Eth_TxBuff_Len;
    int Eth_RxBuff_Total;
    int Eth_TxBuff_Total;
    int EthCtrlIdx;
    int *DescToBuffLink;
    int *BuffToDescLink;
}Eth_CtrlConfigType;

typedef struct
{
    int CLC;
    int GPCTL;
    int SKEWCTL;
}Eth_RegisterMap;

typedef Eth_RegisterMap *Eth_RegisterMapRefType;

typedef struct
{
    int Des0_DataBuff1Addr_Vlan;
    int Des1_Reserved_Status;
    int Des2_DataBuff2Addr_Status;
    int Des3_CtrlStatus;
}Eth_RxBuffDesQueue_st;
typedef Eth_RxBuffDesQueue_st *Eth_RxBuffDesQueueRefType;
typedef struct
{
    int Des0_DataBuff1Addr_TsLow;
    int Des1_DataBuffAddr_TsHigh;
    int Des2_Ctrl;
    int Des3_CtrlStatus;
}Eth_TxBuffDesQueue_st;
typedef Eth_TxBuffDesQueue_st *Eth_TxBuffDesQueueRefType;

typedef struct
{
    Eth_RegisterMapRefType Registers_pst;
    Eth_RxBuffDesQueue_st RxDescQueue_st;
    Eth_TxBuffDesQueue_st TxDescQueue_st;
}Eth_ControllerType_st;
Eth_ControllerType_st Eth_Controllers_ast[1];

int Eth_RegisterMapBaseAddr[1];
int Eth_PhyAddr_Config[6];
int rba_Eth_Port_ConfigSets[1];

static int Eth_DmaDescRxTx[112];
int Eth_DmaBuffRxTx[21504];
static bool Eth_TxConfFlag[7];
static EthTxBuffStates Eth_BuffStateTable[7];
static int Eth_txDesctoBuffLink[7];
static int Eth_txBufftoDescLink[7];

void Eth_Init();
#endif // ETH_H_INCLUDED
