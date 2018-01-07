#include<stdio.h>
#include<stdlib.h>
#include<stdint-gcc.h>
#include"../api/Eth.h"

Eth_PhyAddr_Config[6] =
{
    0xFCU,0xD6U,
    0xBDU,0x00U,
    0x00U,0x01U
};

rba_Eth_Port_ConfigSets[1]=
{
        0x00000C04UL
};
Eth_RegisterMapBaseAddr[1]={0x0000};
void Eth_Init(Eth_CtrlConfigType *ConfPtr)
{
    int CtrlIdx=0;
    int ClcReadVal;
    Eth_ControllerState[CtrlIdx]= ETH_UNINIT;
    Eth_ConrollerMode[CtrlIdx]= ETH_MODE_DOWN;

    Eth_Controllers_ast[0].Registers_pst = (Eth_RegisterMapRefType)Eth_RegisterMapBaseAddr[0];

    Eth_Controllers_ast[0].Registers_pst->CLC=0;
    ClcReadVal=Eth_Controllers_ast[0].Registers_pst->CLC;
    printf("%d",Eth_ControllerState[CtrlIdx]);

    return;
}
