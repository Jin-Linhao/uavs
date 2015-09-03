#ifndef SSUWB_H
#define SSUWB_H
#include "rcmIf.h"
#include "rcm.h"


class ssUWB
{
public:
    int dis[4];
    int* measure();
    ssUWB(int n,const int* ID,const char* filename);
    ssUWB(char * name, char * port);
    void initial();
private:

    char* dev;
    int nodes[4];//nodesID of each node
    int num;//num of node in location network

    rcmIfType   rcmIf;
    rcmConfiguration config;
    rcmMsg_GetStatusInfoConfirm statusInfo;
    rcmMsg_RangeInfo rangeInfo;
    rcmMsg_DataInfo dataInfo;
    rcmMsg_ScanInfo scanInfo;
    rcmMsg_FullScanInfo fullScanInfo;
public:
    int uwb(int destNodeId);
};



#endif // SSUWB_H
