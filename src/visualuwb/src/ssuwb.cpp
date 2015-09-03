#include "ssuwb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rcm.h"
#include "rcmIf.h"
#include <iostream>
using namespace std;

ssUWB::ssUWB(int n,const int* ID, const char* filename)
{
    if(n<=3)
        cout<<"UWB devices not enough!"<<endl;
    else
        num=n;
    for(int i=0; i<num;i++)
    {
        if(nodes[i]<100||nodes[i]>107)
            cout<<"Nodes ID fault!"<<endl;
        else
            nodes[i]=ID[i];
    }
    memcpy(dev,filename,sizeof(filename));
    rcmIf=rcmIfUsb;
    initial();
}

ssUWB::ssUWB(char * name, char * port)
{
    
    num=4;
    dev = name;

    nodes[0]=303;
    nodes[1]=303;
    nodes[2]=303;
    nodes[3]=303;

    if(!strcmp(port, "serial"))
        rcmIf = rcmIfSerial;
    else
        rcmIf = rcmIfUsb;
    initial();
}

int*ssUWB:: measure()
{
	int i=0;
    for(i=0;i<num;i++)
    {
        dis[i]=uwb(nodes[i]);
    }
    return dis;
}

//#define DEFAULT_DEST_NODE_ID    101

void ssUWB::initial()
{
    int status;
    if (rcmIfInit(rcmIf, dev) != OK)
    {
        printf("Initialization failed.\n");
        //exit(0);
    }

    // Make sure RCM is awake
    if (rcmSleepModeSet(RCM_SLEEP_MODE_ACTIVE) != 0)
    {
        printf("Time out waiting for sleep mode set.\n");
        //exit(0);
    }

    // Make sure opmode is RCM
    if (rcmOpModeSet(RCM_OPMODE_RCM) != 0)
    {
        printf("Time out waiting for opmode set.\n");
        //exit(0);
    }

    // execute Built-In Test - verify that radio is healthy
    if (rcmBit(&status) != 0)
    {
        printf("Time out waiting for BIT.\n");
        //exit(0);
    }

    if (status != OK)
    {
        printf("Built-in test failed - status %d.\n", status);
        //exit(0);
    }
    else
    {
        printf("Radio passes built-in test.\n\n");
    }

    // retrieve config from RCM
    if (rcmConfigGet(&config) != 0)
    {
        printf("Time out waiting for config confirm.\n");
        //exit(0);
    }

    // print out configuration
    printf("Configuration:\n");
    printf("\tnodeId: %d\n", config.nodeId);
    printf("\tintegrationIndex: %d\n", config.integrationIndex);
    printf("\tantennaMode: %d\n", config.antennaMode);
    printf("\tcodeChannel: %d\n", config.codeChannel);
    printf("\telectricalDelayPsA: %d\n", config.electricalDelayPsA);
    printf("\telectricalDelayPsB: %d\n", config.electricalDelayPsB);
    printf("\tflags: 0x%X\n", config.flags);
    printf("\ttxGain: %d\n", config.txGain);

    // retrieve status/info from RCM
    if (rcmStatusInfoGet(&statusInfo) != 0)
    {
        printf("Time out waiting for status info confirm.\n");
        //exit(0);
    }

    // print out status/info
    printf("\nStatus/Info:\n");
    printf("\tRCM version: %d.%d build %d\n", statusInfo.appVersionMajor,
            statusInfo.appVersionMinor, statusInfo.appVersionBuild);
    printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo.uwbKernelVersionMajor,
            statusInfo.uwbKernelVersionMinor, statusInfo.uwbKernelVersionBuild);
    printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo.firmwareMonth,
            statusInfo.firmwareDay, statusInfo.firmwareYear,
            statusInfo.firmwareVersion);
    printf("\tSerial number: %08X\n", statusInfo.serialNum);
    printf("\tBoard revision: %c\n", statusInfo.boardRev);
    printf("\tTemperature: %.2f degC\n\n", statusInfo.temperature/4.0);
}



void usage(void)
{
    printf("usage: rcmSampleApp -i <IP address> | -s <COM port> | -u <USB COM port>\n");
    printf("\nTo connect to radio at IP address 192.168.1.100 via Ethernet:\n");
    printf("\trcmSampleApp -i 192.168.1.100\n");
    printf("\nTo connect to radio's serial port using USB-to-TTL serial converter at COM3:\n");
    printf("\trcmSampleApp -s COM3\n");
    printf("\nTo connect to radio's USB port at COM10:\n");
    printf("\trcmSampleApp -u COM10\n");
    exit(0);
}


int ssUWB::uwb(int destNodeId)
{
        // Determine range to a radio. May also get data and scan packets.
        if (rcmRangeTo(destNodeId, RCM_ANTENNAMODE_TXA_RXA, 0, NULL,
                &rangeInfo, &dataInfo, &scanInfo, &fullScanInfo) == 0)
        {
            // we always get a range info packet
//            printf("RANGE_INFO: responder %d, msg ID %u, range status %d, "
//                    "stopwatch %d ms, channelRiseTime %d, vPeak %d, measurement type %d\n",
//                    rangeInfo.responderId, rangeInfo.msgId, rangeInfo.rangeStatus,
//                    rangeInfo.stopwatchTime, rangeInfo.channelRiseTime, rangeInfo.vPeak,
//                    rangeInfo.rangeMeasurementType);

            // The RANGE_INFO can provide multiple types of ranges
            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION)
            {
//                printf("Precision range: %d mm, error estimate %d mm\n",
//                        rangeInfo.precisionRangeMm, rangeInfo.precisionRangeErrEst);
            }

            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_COARSE)
            {
//                printf("Coarse range: %d mm, error estimate %d mm\n",
//                        rangeInfo.coarseRangeMm, rangeInfo.coarseRangeErrEst);
            }

            if (rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_FILTERED)
            {
//                printf("Filtered range: %d mm, error estimate %d mm\n",
//                        rangeInfo.filteredRangeMm, rangeInfo.filteredRangeErrEst);
//                printf("Filtered velocity: %d mm/s, error estimate %d mm/s\n",
//                        rangeInfo.filteredRangeVel, rangeInfo.filteredRangeVelErrEst);
            }


            // only get a data info packet if the responder sent data
            // dataSize will be non-zero if we there is data
            if (dataInfo.dataSize)
                printf("DATA_INFO from node %d: msg ID %u, channelRiseTime %d, vPeak %d, %d bytes\ndata: %*s\n",
                        dataInfo.sourceId, dataInfo.msgId, dataInfo.channelRiseTime, dataInfo.vPeak,
                        dataInfo.dataSize, dataInfo.dataSize, dataInfo.data);

            // only get a scan info packet if the SEND_SCAN bit is set in the config
            // numSamples will be non-zero if there is scan data
            // we don't do anything with the scan data itself here
            if (scanInfo.numSamples)
                printf("SCAN_INFO from node %d: msg ID %u, %d samples, channelRiseTime %d, vPeak %d\n",
                        scanInfo.sourceId, scanInfo.msgId, scanInfo.numSamples,
                        scanInfo.channelRiseTime, scanInfo.vPeak);

            // only get a full scan info packet if the FULL_SCAN bit is set in the config
            // numSamplesInMessage will be non-zero if there is scan data
            // we don't do anything with the scan data itself here
            if (fullScanInfo.numSamplesInMessage)
                printf("FULL_SCAN_INFO from node %d: msg ID %u, %d samples, channelRiseTime %d, vPeak %d\n",
                        fullScanInfo.sourceId, fullScanInfo.msgId, fullScanInfo.numSamplesInMessage,
                        fullScanInfo.channelRiseTime, fullScanInfo.vPeak);

            // a rangeStatus of 0 means the range was calculated successfully
//            if (rangeInfo.rangeStatus == 0)
//            {
//                // now broadcast the range in a data packet
//                sprintf(str, "The range from %d to %d is %d mm.",
//                        config.nodeId, destNodeId,
//                        rangeInfo.precisionRangeMm);
//                rcmDataSend(RCM_ANTENNAMODE_TXA_RXA, strlen(str), str);
//            }
        }


    // perform cleanup
    //rcmIfClose();
    return rangeInfo.precisionRangeMm;
}

