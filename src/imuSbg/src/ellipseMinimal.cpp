/*!
 *	\file		ellipseMinimal.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		28/03/2014
 *
 *	\brief		C example that simply opens an Ellipse interface and reads Euler Angles from it.
 *
 *	This small example demonstrates how to initialize the sbgECom library
 *	to read data from an Ellipse using callbacks.
 *
 *	\section CodeCopyright Copyright Notice
 *	Copyright (C) 2007-2014, SBG Systems SAS. All rights reserved.
 *
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */
#define ROS_MASTER_URI		"http://localhost:11311"
#define ROS_ROOT		"/opt/ros/indigo/share/ros"
#include "sbgEComLib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>
#include <ros/ros.h>

/*Program specific definitions
 * Textcolor
 * Default parameters for node operation
 */
//Text color
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

//Node operation
#define DFLT_PORT       "/dev/ttyUSB0"
#define DFLT_NODE_RATE	"40"            //Update frequency

//Parameter
#define ORI_COV 1e-3
#define GYR_COV 1e-3
#define ACC_COV 1e-3


uint32_t            nodeRate = 40;
ros::Publisher      imuSbgPublisher;
ros::Time           timeStamp;
sensor_msgs::Imu    ImuMsg;
long                seqCount = 0;
uint8_t             dataFlag[2] = {0, 0};

using namespace std;
//----------------------------------------------------------------------//
//  Call backs                                                          //
//----------------------------------------------------------------------//

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	logCmd									Contains the binary received log command id.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComCmdId logCmd, const SbgBinaryLogData *pLogData, void *pUserArg)
{
    //
    // Handle separately each received data according to the log ID
    //
    switch (logCmd)
    {
    case SBG_ECOM_LOG_EKF_QUAT:
    {
        //Mark that there is new angle info
        dataFlag[0] = 0xFF;

//        printf("Euler Angles: %3.1f\t%3.1f\t%3.1f\tStd Dev:%3.1f\t%3.1f\t%3.1f   \n",
//               pLogData->ekfEulerData.euler[0]*180.0/SBG_PI, pLogData->ekfEulerData.euler[1]*180.0/SBG_PI, pLogData->ekfEulerData.euler[2]*180.0/SBG_PI,
//                pLogData->imuData.accelerometers[0],pLogData->imuData.accelerometers[1], pLogData->imuData.accelerometers[2]);
        //orientation
//        tf::Quaternion q = tf::createQuaternionFromRPY(pLogData->ekfEulerData.euler[0],
//                pLogData->ekfEulerData.euler[1],
//                pLogData->ekfEulerData.euler[2]);

        //Orientation in quaternion
        ImuMsg.orientation.x = pLogData->ekfQuatData.quaternion[0];
        ImuMsg.orientation.y = pLogData->ekfQuatData.quaternion[1];
        ImuMsg.orientation.z = pLogData->ekfQuatData.quaternion[2];
        ImuMsg.orientation.w = pLogData->ekfQuatData.quaternion[3];
        //orientation covariance
        for(int i=0; i<9; i++)
            ImuMsg.orientation_covariance[i] = pLogData->ekfQuatData.eulerStdDev[i]*pLogData->ekfEulerData.eulerStdDev[i];
        break;
    }
    case SBG_ECOM_LOG_IMU_DATA:
    {
        dataFlag[1] = 0xFF;
        //angular velocity
        ImuMsg.angular_velocity.x = pLogData->imuData.gyroscopes[0];
        ImuMsg.angular_velocity.y = pLogData->imuData.gyroscopes[1];
        ImuMsg.angular_velocity.z = pLogData->imuData.gyroscopes[2];
        //angular velocity variance
        for(int i = 0; i<9; i++)
            ImuMsg.angular_velocity_covariance[i] = 0;
        ImuMsg.angular_velocity_covariance[0] = 5e-5;
        ImuMsg.angular_velocity_covariance[4] = 5e-5;
        ImuMsg.angular_velocity_covariance[8] = 5e-5;

        //linear acceleration
        ImuMsg.linear_acceleration.x = pLogData->imuData.accelerometers[0];
        ImuMsg.linear_acceleration.y = pLogData->imuData.accelerometers[1];
        ImuMsg.linear_acceleration.z = pLogData->imuData.accelerometers[2];
        //linear acceleration variance
        for(int i = 0; i<9; i++)
            ImuMsg.linear_acceleration_covariance[i] = 0;
        ImuMsg.linear_acceleration_covariance[0] = 5e-5;
        ImuMsg.linear_acceleration_covariance[4] = 5e-5;
        ImuMsg.linear_acceleration_covariance[8] = 5e-5;
        break;
    }
    default:
        break;
    }
    //Check if both pieces of info have been acquired
    if(dataFlag[0] == 0xFF && dataFlag[1] == 0xFF)
    {
        dataFlag[0] = 0;
        dataFlag[1] = 0;
        seqCount++;
        //header
        ImuMsg.header.seq = seqCount;
        ImuMsg.header.stamp = ros::Time::now();
        ImuMsg.header.frame_id = "imuSbg_frame";
        imuSbgPublisher.publish(ImuMsg);
    }
    return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

/*!
 *	Main entry point.
 *	\param[in]	argc		Number of input arguments.
 *	\param[in]	argv		Input arguments as an array of strings.
 *	\return					0 if no error and -1 in case of error.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imuSbgNode");
    ros::NodeHandle imuSbgNode("~");

    string serialPortName = string(DFLT_PORT);
    string imuSbgNodeRate = string(DFLT_NODE_RATE);
    if(imuSbgNode.getParam("imuSbgSerialPort", serialPortName))
        printf(KBLU"Retrieved value %s for param 'imuSbgSerialPort'!\n"RESET, serialPortName.data());
    else
    {
        //serialPortName = string(DFLT_PORT);
        printf(KRED "Couldn't retrieve param 'imuSbgSerialPort', program closed!\n"RESET);
        return 0;
    }

    if(imuSbgNode.getParam("imuSbgNodeRate", imuSbgNodeRate))
        printf(KBLU"Retrieved value %s for param 'imuSbgNodeRate'\n"RESET, imuSbgNodeRate.data());
    else
        printf(KYEL "Couldn't retrieve param 'imuSbgNodeRate', applying default value %sHz\n"RESET, imuSbgNodeRate.data());

    nodeRate = atoi(imuSbgNodeRate.data());
    imuSbgPublisher = imuSbgNode.advertise<sensor_msgs::Imu>("imuSbgTopic", 1);

    SbgEComHandle			comHandle;
    SbgErrorCode			errorCode;
    SbgInterface			sbgInterface;
    int32					retValue = 0;
    SbgEComDeviceInfo		deviceInfo;

    //
    // Create an interface:
    // We can choose either a serial for real time operation, or file for previously logged data parsing
    // Note interface closing is also differentiated !
    //
    errorCode = sbgInterfaceSerialCreate(&sbgInterface, serialPortName.data(), 921600);		// Example for Unix using a FTDI Usb2Uart converter
    //errorCode = sbgInterfaceSerialCreate(&sbgInterface, "COM8", 115200);				// Example for Windows serial communication

    //
    // Test that the interface has been created
    //
    if (errorCode == SBG_NO_ERROR)
    {
        //
        // Create the sbgECom library and associate it with the created interfaces
        //
        errorCode = sbgEComInit(&comHandle, &sbgInterface);

        //
        // Test that the sbgECom has been initialized
        //
        if (errorCode == SBG_NO_ERROR)
        {
            //
            // Get device inforamtions
            //
            errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);

            //
            // Display device information if no error
            //
            if (errorCode == SBG_NO_ERROR)
            {
                printf("Device : %0.9u found\n", deviceInfo.serialNumber);
            }
            else
            {
                fprintf(stderr, "imuSbg: Unable to get device information.\n");
            }

            SbgEComOutputMode outputModeDiv;
            switch(200/nodeRate)
            {
            case 1:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_MAIN_LOOP;
                break;
            }
            case 2:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_2;
                break;
            }
            case 4:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_4;
                break;
            }
            case 8:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_8;
                break;
            }
            case 10:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_10;
                break;
            }
            case 20:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_20;
                break;
            }
            case 40:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_40;
                break;
            }
            case 200:
            {
                outputModeDiv = SBG_ECOM_OUTPUT_MODE_DIV_200;
                break;
            }
            default:
            {
                printf("Unsupported rate, program closed!.\n");
                return -1;
                break;
            }
            }

            //
            // Configure some output logs to specified rate
            //
            if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, outputModeDiv) != SBG_NO_ERROR)
            {
                fprintf(stderr, "imuSbg: Unable to configure output log SBG_ECOM_LOG_IMU_DATA.\n");
            }
            if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, outputModeDiv) != SBG_NO_ERROR)
            {
                fprintf(stderr, "imuSbg: Unable to configure output log SBG_ECOM_LOG_EKF_EULER.\n");
            }

            //
            // Display a message for real time data display
            //
            printf("sbgECom properly Initialized.\n\nEuler Angles display with estimated standard deviation.\n");

            //
            // Define callbacks for received data
            //
            sbgEComSetReceiveCallback(&comHandle, onLogReceived, NULL);

            //
            // Loop until the user exist
            //
            ros::Rate rate(nodeRate);
            while (ros::ok())
            {
                //
                // Try to read a frame
                //
                errorCode = sbgEComHandle(&comHandle);

                //
                // Test if we have to release some CPU (no frame received)
                //
                if (errorCode == SBG_NOT_READY)
                {
                    //
                    // Release CPU
                    //
                    rate.sleep();
                }
                else
                {
                    fprintf(stderr, "Error\n");
                }
            }

            //
            // Close the sbgEcom library
            //
            sbgEComClose(&comHandle);
        }
        else
        {
            //
            // Unable to initialize the sbgECom
            //
            fprintf(stderr, "imuSbg: Unable to initialize the sbgECom library.\n");
            retValue = -1;
        }

        //
        // Close the interface
        //
        sbgInterfaceSerialDestroy(&sbgInterface);
        //sbgInterfaceFileClose(&sbgInterface);

    }
    else
    {
        //
        // Unable to create the interface
        //
        fprintf(stderr, "imuSbg: Unable to create the interface.\n");
        retValue = -1;
    }

    //
    // Returns -1 if we have an error
    //
    return retValue;
}
