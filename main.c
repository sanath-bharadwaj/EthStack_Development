#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include"../Eth/api/Eth.h"


int main()
{

    Eth_CtrlConfigType Eth_CtrlConfig_ao[1];
    Eth_CtrlConfigType *Eth_ConfigRef_type=(Eth_CtrlConfigType*)malloc(sizeof(Eth_CtrlConfigType));

    Eth_CtrlConfig_ao[0].Eth_Phy_addr=&Eth_PhyAddr_Config[0];
    Eth_CtrlConfig_ao[0].Eth_Rx_Desc=&Eth_DmaDescRxTx[0];
    Eth_CtrlConfig_ao[0].Eth_Tx_Desc=&Eth_DmaDescRxTx[56];
    Eth_CtrlConfig_ao[0].Eth_Rx_Buff=&Eth_DmaBuffRxTx[0];
    Eth_CtrlConfig_ao[0].Eth_Tx_Buff=&Eth_DmaBuffRxTx[10752];
    Eth_CtrlConfig_ao[0].EthTxConfFlag=&Eth_TxConfFlag[0];
    Eth_CtrlConfig_ao[0].TxBuffStateTable=&Eth_BuffStateTable[0];
    Eth_CtrlConfig_ao[0].Eth_RxBuff_Len=1536U;
    Eth_CtrlConfig_ao[0].Eth_TxBuff_Len=1536U;
    Eth_CtrlConfig_ao[0].Eth_RxBuff_Total=7U;
    Eth_CtrlConfig_ao[0].Eth_TxBuff_Total=7U;
    Eth_CtrlConfig_ao[0].EthCtrlIdx=0U;
    Eth_CtrlConfig_ao[0].DescToBuffLink=&Eth_txDesctoBuffLink[0];
    Eth_CtrlConfig_ao[0].BuffToDescLink=&Eth_txBufftoDescLink[0];
    Eth_Init(&Eth_CtrlConfig_ao);
    return 0;
}
