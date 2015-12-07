//============================================================================
// Name        : serialDemo.cpp
// Author      : britsk nguyen
// Version     :
// Copyright   : Your copyright notice
// Description : ros node to communicate with vicon
//============================================================================
#define ROS_MASTER_URI		"http://localhost:11311"
#define ROS_ROOT		"/opt/ros/indigo/share/ros"
#include <iostream>
#include "stdio.h"
#include <stdlib.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <signal.h>
#include <math.h>
#include <viconXbee/viconPoseMsg.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>

//#include <mavros/Imu.h>
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define BAUDRATE		57600
#define BAUD_MACRO		B57600
#define DFLT_PORT	"/dev/ttyUSB0"
#define RCV_THRESHOLD	40					//Receive frame's length
#define FPS				40					//Frames per second
#define DFLT_NODE_RATE	"20"			//Update frequency
#define	UP				0xFF
#define DOWN			0x00
#define HEADER_NAN		0x7FAAAAAA

using namespace std;

uint8_t		frame32[RCV_THRESHOLD];
uint8_t		rcvdFrame[RCV_THRESHOLD];
uint8_t		backupFrame[RCV_THRESHOLD];
uint8_t		flagIncompleteFrame = 0;
int			rcvdBytesCount = 0;
uint8_t		backupRcvBytesCount = 0;
uint8_t		misAlgnFlag = DOWN;
uint8_t		msgFlag = DOWN;
uint8_t		viconUpdateFlag = DOWN;
uint8_t		publishFlag = DOWN;
uint8_t		headerIndex = 0xFF;
//uint8_t		serialPortName[13] = {'/','d', 'e','v','/', 't', 't', 'y', 'U', 'S', 'B', '0'};
uint32_t	seqCount = 0;
//float viconX , viconY, viconZ, viconXd, viconYd, viconZd, viconPitchd, viconRolld, viconYawd;
uint32_t	nodeRate = 40;

long validCount = 0;
long faultCount = 0;
long freeCount = 0;
long loopCount = 0;

void signal_handler_IO(int status)
{
    msgFlag = UP;
}

int main(int argc, char **argv)
{
    //Create ros handler to node
    ros::init(argc, argv, "viconXbeeNode");
    ros::NodeHandle viconXbeeNode("~");

    string serialPortName = string(DFLT_PORT);
    string viconNodeRate = string(DFLT_NODE_RATE);

    bool handshakingMode = true;
    bool asynMode = false;

    if(viconXbeeNode.getParam("viconSerialPort", serialPortName))
        printf(KBLU"Retrieved value %s for param 'viconSerialPort'!\n"RESET, serialPortName.data());
    else
    {
        printf(KRED "Couldn't retrieve param 'viconSerialPort', program closed!\n"RESET);
        return 0;
    }

    if(viconXbeeNode.getParam("viconNodeRate", viconNodeRate))
        printf(KBLU"Retrieved value %s for param 'viconnNodeRate'\n"RESET, viconNodeRate.data());
    else
        printf(KYEL "Couldn't retrieve param 'viconnNodeRate', applying default value %sHz\n"RESET, viconNodeRate.data());

    if(viconXbeeNode.getParam("viconAsyncMode", asynMode))
    {
        if(asynMode)
            printf(KBLU"Retrieved value 'true' for param 'viconAsyncMode.'\n"RESET);
        else
            printf(KBLU"Retrieved value 'false' for param 'viconAsyncMode.'\n"RESET);
    }
    else
        printf(KYEL"Couldn't retrieve param 'viconAsyncMode'. Using default synchrous mode.\n"RESET);

    if(viconXbeeNode.getParam("viconHandshakingModeEnabled", handshakingMode))
    {
        if(handshakingMode)
            printf(KBLU"Retrieved value 'true' for param 'viconHandshakingModeEnabled.'\n"RESET);
        else
            printf(KBLU"Retrieved value 'false' for param 'viconHandshakingModeEnabled.'\n"RESET);
    }
    else
        printf(KYEL"Couldn't retrieve param 'viconHandshakingModeEnabled'. Using default mode with handshake.\n"RESET);


    nodeRate = atoi(viconNodeRate.data());
    ros::Publisher viconPosePublisher = viconXbeeNode.advertise<viconXbee::viconPoseMsg>("viconPoseTopic", 1);
    ros::Publisher viconMocapPublisher = viconXbeeNode.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);
    ros::Time timeStamp;

    //-------------------Initialize Serial Connection---------------------------------------------------
    int fd = -1;
    struct termios newtio;
    struct sigaction saio;           //definition of signal action
    FILE *fpSerial = NULL;

    //Open the serial port as a file descriptor for low level configuration
    //read/write, not controlling terminal for process,
    //fd = open(serialPortName.data(), O_RDWR | O_NOCTTY | O_NDELAY);	//program won't be blocked during read
    fd = open(serialPortName.data(), O_RDWR | O_NOCTTY);    			//program will be blocked in during read
    if ( fd < 0 )
    {
        printf(KRED "serialInit: Could not open serial device %s\n" RESET, serialPortName.data());
        return 0;
    }

    if(asynMode)
    {
        printf("Initializing asynchronous mode for Vicon Xbee\n");
        //Install the signal handler before making the device asynchronous
        saio.sa_handler = signal_handler_IO;
        sigemptyset(&saio.sa_mask);
        saio.sa_flags = 0;
        saio.sa_restorer = NULL;
        sigaction(SIGIO,&saio,NULL);

        //allow the process to receive SIGIO
        fcntl(fd, F_SETOWN, getpid());
        //Make the file descriptor asynchronous
        fcntl(fd, F_SETFL, O_ASYNC|O_NONBLOCK);
    }

    //Set up serial settings
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag =  CS8 | CLOCAL | CREAD;		//no parity, 1 stop bit
    newtio.c_iflag |= IGNBRK;  					//ignore break condition
    newtio.c_oflag = 0;							//all options off
    //set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    //non-canonical condition, RCV_THRESHOLD is the number of bytes to return
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = RCV_THRESHOLD;

    //Flush the residual data and activate new settings
    tcflush(fd, TCIFLUSH);
    if (cfsetispeed(&newtio, BAUD_MACRO) < 0 || cfsetospeed(&newtio, BAUD_MACRO) < 0)
    {
        printf(KRED"Cannot set baudrate for %s\n", serialPortName.data());
        close(fd);
        return 0;
    }
    else
    {
        tcsetattr(fd, TCSANOW, &newtio);
        tcflush(fd, TCIOFLUSH);
        printf(KBLU "Connection established\n\n" RESET);
    }
    //Open file as a standard I/O stream
    fpSerial = fdopen(fd, "r+");
    if (!fpSerial)
    {
        printf(KRED "serialInit: Failed to open %s as serial stream\n" RESET, serialPortName.data());
        fpSerial = NULL;
    }
    tcflush(fd, TCIOFLUSH);
    //-----------------------------------Serial Initialization-------------------------------------

    //ros Rate object to control the update rate
    ros::Rate rate(nodeRate);
    tcflush(fd, TCIOFLUSH);
    while(ros::ok())
    {
        if(handshakingMode)
            write(fd,"d", 1);

        //Check if vicon data has arrived
        if(msgFlag == UP || !asynMode)
        {
            //            ros::Duration tempoSleep(0.020);
            //            tempoSleep.sleep();
            msgFlag = DOWN;
            rcvdBytesCount = read(fd,rcvdFrame, RCV_THRESHOLD);
            tcflush(fd, TCIOFLUSH);
            //First, check if read is successful
            if(rcvdBytesCount == -1)
            {
                printf(KRED"Buffer not ready... :( \n" RESET);
                ros::Time recoveryTime = ros::Time::now();
                uint8_t residue = RCV_THRESHOLD;
                while(true)
                {
                    rcvdBytesCount = read(fd,rcvdFrame, residue);

                    /*                    if(rcvdBytesCount == RCV_THRESHOLD)
                    {
                        printf(KGRN"Frame comes after waiting\n" RESET);
                        for(int i = 0; i < rcvdBytesCount; i++)
                        {
                            backupFrame[i] = rcvdFrame[i];
                            printf(KGRN"%2x ", rcvdFrame[i]);
                        }
                        printf("\n"RESET);
                    }
                    else*/ if(rcvdBytesCount != -1)
                    {
                        printf(KYEL"Fragment of %d byte(s) arrives\n"RESET, rcvdBytesCount);
                        //                        for(int i = 0; i < rcvdBytesCount; i++)
                        //                        {
                        //                            backupFrame[RCV_THRESHOLD - residue + i] = rcvdFrame[i];
                        //                            printf(KGRN"%2x ", rcvdFrame[i]);
                        //                        }
                        //                        printf("\n"RESET);
                        residue = residue - rcvdBytesCount;
                        if(residue == 0)
                        {
                            viconUpdateFlag = UP;
                            //                            printf(KGRN"Concantenated frame: "RESET);
                            //                            for(int i = 0; i < RCV_THRESHOLD; i++)
                            //                                printf(KYEL"%2x ", backupFrame[i]);
                            //                            printf("\n"RESET);
                            break;
                        }
                        //(fd, TCIOFLUSH);
                        //break;
                    }
                    else if((ros::Time::now() - recoveryTime).toSec() > 0.01)
                    {
                        printf(KRED"Time out waiting for this frame!\n"RESET);
                        tcflush(fd, TCIOFLUSH);
                        break;
                    }
                }
            }
            //Second check if read returns full frame
            else if(rcvdBytesCount < RCV_THRESHOLD)
            {
                //printf(KRED "%d\n" RESET, rcvdBytesCount);
                printf(KRED "Only %d/%d bytes! Attempting to fetch the rest.. >,< \n" RESET, rcvdBytesCount, RCV_THRESHOLD);
                for(int i = 0; i < rcvdBytesCount; i++)
                    //{
                    backupFrame[i] = rcvdFrame[i];
                //printf(KYEL"%2x ", rcvdFrame[i]);
                //}
                //printf("\n"RESET);
                //                ros::Duration tempoSleep(0.02);
                //                tempoSleep.sleep();
                ros::Time recoveryTime = ros::Time::now();
                uint8_t residue = RCV_THRESHOLD - rcvdBytesCount;
                uint8_t miniBuff = 0, retrievalCount = 0;
                for (int i = 0; i < residue; i++)
                {
                    rcvdBytesCount = read(fd,&miniBuff, 1);
                    if(rcvdBytesCount == -1)
                    {
                        if((ros::Time::now() - recoveryTime).toSec() > 0.01)
                            i = residue;
                        else
                            i--;
                    }
                    else
                    {
                        retrievalCount++;
                        backupFrame[RCV_THRESHOLD - residue + i] = miniBuff;
                        //printf(KYEL"%2x ", miniBuff);
                    }
                }
                //printf("\n"RESET);
                if(retrievalCount == residue)
                {
                    printf(KGRN"Full retrival \n"RESET);
                    //                    for(int i = 0; i < RCV_THRESHOLD; i++)
                    //                        printf(KGRN"%2x ", backupFrame[i]);
                    //                    printf("\n"RESET);
                    viconUpdateFlag = UP;
                }
                else
                {
                    printf(KRED"Frame segment lost forever!\n\n"RESET);
                    tcflush(fd, TCIOFLUSH);
                }

            }
            //Third, check if frame is valid
            else
            {
                //Tracing the header
                if(*(uint32_t *)(rcvdFrame) != (HEADER_NAN))
                {
                    printf(KYEL"Jumbled frame: "RESET);
                    for(int i = 0; i < RCV_THRESHOLD; i++)
                    {
                        backupFrame[i] = rcvdFrame[i];
                        printf(KYEL"%2x ", rcvdFrame[i]);
                    }
                    printf("\n"RESET);
                    viconUpdateFlag = UP;
                }
                else // header = HEADER_NAN
                {
                    printf(KBLU "Recieved well-aligned message... ^,^!\n");
                    for(int i = 0; i< RCV_THRESHOLD; i++)
                        backupFrame[i] = rcvdFrame[i];
                    //A precautious flush to remove any possible buffered message
                    tcflush(fd, TCIOFLUSH);
                    viconUpdateFlag = UP;
                }
            }
        }
        //If no vicon data has arrived then notify and go on
        else
        {
            printf(KWHT "No message has arrived...\n\n\n" RESET);
        }

        //        backupFrame[20] = 0xAA;
        //        backupFrame[21] = 0xAA;
        //        backupFrame[22] = 0x7F;
        //        backupFrame[23] = 1;
        //        backupFrame[24] = 2;
        //        backupFrame[25] = 3;
        //        backupFrame[26] = 4;
        //        backupFrame[27] = 5;
        //        backupFrame[28] = 6;
        //        backupFrame[29] = 7;
        //        backupFrame[30] = 8;
        //        backupFrame[31] = 9;
        //        backupFrame[32] = 10;
        //        backupFrame[33] = 11;
        //        backupFrame[34] = 12;
        //        backupFrame[35] = 13;
        //        backupFrame[36] = 14;
        //        backupFrame[37] = 15;
        //        backupFrame[38] = 16;
        //        backupFrame[39] = 17;
        //        backupFrame[0] = 18;
        //        backupFrame[1] = 19;
        //        backupFrame[2] = 20;
        //        backupFrame[3] = 21;
        //        backupFrame[4] = 22;
        //        backupFrame[5] = 23;
        //        backupFrame[6] = 24;
        //        backupFrame[7] = 25;
        //        backupFrame[8] = 26;
        //        backupFrame[9] = 27;
        //        backupFrame[10] = 28;
        //        backupFrame[11] = 29;
        //        backupFrame[12] = 30;
        //        backupFrame[13] = 31;
        //        backupFrame[14] = 32;
        //        backupFrame[15] = 33;
        //        backupFrame[16] = 34;
        //        backupFrame[17] = 35;
        //        backupFrame[18] = 36;
        //        backupFrame[19] = 0xAA;

        if(viconUpdateFlag == UP)
        {
            viconUpdateFlag = DOWN;

            //Tracing the header
            headerIndex = 0xFF;
            //check if header is in the middle of frame
            for(int i = 0; i < RCV_THRESHOLD - 4 + 1; i++)
                if(*(uint32_t *)(backupFrame + i) == HEADER_NAN)
                {
                    headerIndex = i;
                    //swapping segments
                    for(int j = 0; j < RCV_THRESHOLD - headerIndex; j++)
                        rcvdFrame[j] = backupFrame[headerIndex + j];
                    for(int j = RCV_THRESHOLD - headerIndex; j < RCV_THRESHOLD; j++)
                        rcvdFrame[j] = backupFrame[j - RCV_THRESHOLD + headerIndex];
                    for(int j = 0; j < RCV_THRESHOLD; j++)
                        backupFrame[j] = rcvdFrame[j];

                    //                    for(int j = 0; j < RCV_THRESHOLD; j++)
                    //                        printf(KYEL"%2x ", backupFrame[j]);
                    //                    printf("\n"RESET);

                    break;
                }
            if(headerIndex == 0xFF)
            {
                printf(KYEL"Possible seperated header!\n");
                for(int j = 0; j < 3; j++)
                {
                    //check if header is seperated to the two ends, only three cases
                    uint8_t backupFrameLastByte = backupFrame[RCV_THRESHOLD-1];
                    for(int i = RCV_THRESHOLD - 1; i > 0; i--)
                        backupFrame[i] = backupFrame[i-1];
                    backupFrame[0] = backupFrameLastByte;

                    if(*(uint32_t *)(backupFrame) == HEADER_NAN)
                    {
                        headerIndex = RCV_THRESHOLD - j - 1;
                        break;
                    }
                }
            }
            printf(KGRN"Header index: %d\n"RESET, headerIndex);

            printf(KGRN"#");
            printf("%#8x$", *(uint32_t *)(backupFrame));
            bool validValues = true;
            for(int i = 1; i < RCV_THRESHOLD/4; i++)
            {
                printf("%f$", *(float *)(backupFrame + i*4));
                if(abs(*(float *)(backupFrame + i*4)) > 20)
                    validValues = false;
            }
            printf("\n"RESET);

            if(validValues)
            {
                viconXbee::viconPoseMsg viconPose;
                viconPose.time_stamp = ros::Time::now();
                viconPose.x = *(float *)(backupFrame + 4);
                viconPose.y = *(float *)(backupFrame + 8);
                viconPose.z = *(float *)(backupFrame + 12);
                viconPose.dx = *(float *)(backupFrame + 16);
                viconPose.dy = *(float *)(backupFrame + 20);
                viconPose.dz = *(float *)(backupFrame + 24);
                viconPose.roll = *(float *)(backupFrame + 28);
                viconPose.pitch = *(float *)(backupFrame + 32);
                viconPose.yaw = *(float *)(backupFrame + 36);
                viconPosePublisher.publish(viconPose);

                //publish to mocap/pose topic
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.seq = seqCount;
                poseStamped.header.stamp = viconPose.time_stamp;
                poseStamped.header.frame_id = "/vicon_global_frame";
                poseStamped.pose.position.x = viconPose.x;
                poseStamped.pose.position.y = viconPose.y;
                poseStamped.pose.position.z = viconPose.z;
                tf::Quaternion q = tf::createQuaternionFromRPY(viconPose.roll, viconPose.pitch, viconPose.yaw);
                poseStamped.pose.orientation.x = q.x();
                poseStamped.pose.orientation.y = q.y();
                poseStamped.pose.orientation.z = q.z();
                poseStamped.pose.orientation.w = q.w();
                viconMocapPublisher.publish(poseStamped);
                seqCount++;
            }
        }
        rate.sleep();
    }
    return 0;
}

