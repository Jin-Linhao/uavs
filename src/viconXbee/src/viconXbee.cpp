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
#define DFLT_NODE_RATE		"40"					//Update frequency
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
	if(viconXbeeNode.getParam("viconSerialPort", serialPortName))
		printf(KBLU"Retrieved value %s for param 'viconSerialPort'!\n"RESET, serialPortName.data());
	else
	{
		//serialPortName = string(DFLT_PORT);
		printf(KRED "Couldn't retrieve param 'viconSerialPort', program closed!\n"RESET);
		return 0;
	}

	if(viconXbeeNode.getParam("viconNodeRate", viconNodeRate))
		printf(KBLU"Retrieved value %s for param 'viconnNodeRate'\n"RESET, viconNodeRate.data());
	else
		printf(KYEL "Couldn't retrieve param 'viconnNodeRate', applying default value %sHz\n"RESET, viconNodeRate.data());

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
	fd = open(serialPortName.data(), O_RDWR | O_NOCTTY | O_NDELAY);	//program won't be blocked during read
	//fd = open(DEFAULT_PORT, O_RDONLY | O_NOCTTY);			//program will be blocked in during read
	if ( fd < 0 )
	{
		printf(KRED "serialInit: Could not open serial device %s\n" RESET, serialPortName.data());
		return 0;
	}

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
	//-----------------------------------Serial Initialization-------------------------------------

	//ros Rate object to control the update rate
	ros::Rate rate(nodeRate);
	tcflush(fd, TCIOFLUSH);
	while(ros::ok())
	{
		if(loopCount == nodeRate*5)
		{
			loopCount = 0;
			validCount = 0;
			freeCount = 0;
			faultCount = 0;

		}
		loopCount++;
		printf("Waiting for message...\n");
		if(msgFlag == UP)
		{
			msgFlag = DOWN;
			rcvdBytesCount = read(fd,rcvdFrame, RCV_THRESHOLD);
			if(rcvdBytesCount == -1)
			{
				faultCount++;
				printf(KBLU "Error reading message... :( \n" RESET);

			}
			else if(rcvdBytesCount < RCV_THRESHOLD)
			{
				printf(KRED "%d\n" RESET, rcvdBytesCount);
				faultCount++;
				printf(KRED "Only %d/%d bytes! There's some problem with communication.. >,< \n" RESET, rcvdBytesCount, RCV_THRESHOLD);

			}
			else
			{
				//Tracing the header
				if(*(uint32_t *)(rcvdFrame) != (HEADER_NAN))
				{
					faultCount++;
					printf(KYEL "Lost alignment...\n" RESET);
//					for(int i = 0; i < RCV_THRESHOLD/4; i++)
//						printf("%f$", *(uint32_t *)(rcvdFrame + i*4));
					//tracing the header
					headerIndex = 0xFF;
					for(int i = 0; i < RCV_THRESHOLD - 4; i++)
						if(*(uint32_t *)(rcvdFrame + i) == HEADER_NAN)
							headerIndex = i;
					if(headerIndex == 0xFF)
					{
						printf(KRED "No Header detected T_T...\n" RESET);
					}
					else
					{
						printf(KYEL "Header detected, reconstructing frame...\n");
						for(int i = 0; i < headerIndex; i++)
							frame32[i] = backupFrame[i + headerIndex];
						for(int i = headerIndex; i < RCV_THRESHOLD; i++)
							frame32[i] = rcvdFrame[i - headerIndex];

						printf("#");
						printf("%#8x$", *(uint32_t *)(frame32));
						for(int i = 1; i < RCV_THRESHOLD/4; i++)
							printf("%f$", *(float *)(frame32 + i*4));
						printf("# %f # %f \n\n" RESET,
								(float)validCount/(loopCount - freeCount),
								(float)faultCount/(loopCount - freeCount)
								);

						for(int i = 0; i < RCV_THRESHOLD; i++)
							backupFrame[i] = rcvdFrame[i];
					}
					//A magical flush that appears to solve the misalignment...
					tcflush(fd, TCIOFLUSH);
				}
				else // header = HEADER_NAN
				{
					validCount++;
					printf(KBLU "Got well-aligned message... ^,^!\n");
					if(misAlgnFlag == DOWN)
						misAlgnFlag = UP;
					printf("#");
					printf("%#8x$", *(uint32_t *)(rcvdFrame));
					for(int i = 1; i < RCV_THRESHOLD/4; i++)
						printf("%f$", *(float *)(rcvdFrame + i*4));
					printf("# %f # %f \n\n" RESET,
							(float)validCount/(loopCount - freeCount),
							(float)faultCount/(loopCount - freeCount)
					);
					publishFlag = UP;
				}
			}
		}
		else
		{
			freeCount++;
			if(freeCount >= (nodeRate*5/4))
			{
				//A magical flush that appears to solve the misalignment...
				tcflush(fd, TCIOFLUSH);
				printf(KWHT "No message has arrived...\n" RESET);
				printf(KYEL"Magic flush...\n\n"RESET);
			}
			else
				printf(KWHT "No message has arrived...\n\n\n" RESET);
		}
		if(publishFlag == UP)
		{
			publishFlag = DOWN;
			viconXbee::viconPoseMsg viconPose;
			viconPose.time_stamp = ros::Time::now();
			viconPose.x = *(float *)(rcvdFrame + 4);
			viconPose.y = *(float *)(rcvdFrame + 8);
			viconPose.z = *(float *)(rcvdFrame + 12);
			viconPose.dx = *(float *)(rcvdFrame + 16);
			viconPose.dy = *(float *)(rcvdFrame + 20);
			viconPose.dz = *(float *)(rcvdFrame + 24);
			viconPose.roll = *(float *)(rcvdFrame + 28);
			viconPose.pitch = *(float *)(rcvdFrame + 32);
			viconPose.yaw = *(float *)(rcvdFrame + 36);
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
		rate.sleep();
	}
	return 0;
}

