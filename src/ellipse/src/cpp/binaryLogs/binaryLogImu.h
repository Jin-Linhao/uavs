/*!
 *	\file		binaryLogImu.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		25 February 2013
 *
 *	\brief		This file is used to parse received IMU binary logs.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
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
#ifndef __BINARY_LOG_IMU_H__
#define __BINARY_LOG_IMU_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log Inertial Data definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * Log inertial data status mask definitions
 */
#define	SBG_ECOM_IMU_COM_OK				(0x00000001u << 0)		/*!< Set to 1 if the communication with the IMU is ok. */
#define	SBG_ECOM_IMU_STATUS_BIT			(0x00000001u << 1)		/*!< Set to 1 if the IMU passes general Built in Tests (calibration, CPU, ...). */

#define	SBG_ECOM_IMU_ACCEL_X_BIT		(0x00000001u << 2)		/*!< Set to 1 if the accelerometer X passes Built In Test. */
#define	SBG_ECOM_IMU_ACCEL_Y_BIT		(0x00000001u << 3)		/*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define	SBG_ECOM_IMU_ACCEL_Z_BIT		(0x00000001u << 4)		/*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define	SBG_ECOM_IMU_GYRO_X_BIT			(0x00000001u << 5)		/*!< Set to 1 if the gyroscope X passes Built In Test. */
#define	SBG_ECOM_IMU_GYRO_Y_BIT			(0x00000001u << 6)		/*!< Set to 1 if the gyroscope Y passes Built In Test. */
#define	SBG_ECOM_IMU_GYRO_Z_BIT			(0x00000001u << 7)		/*!< Set to 1 if the gyroscope Z passes Built In Test. */

#define	SBG_ECOM_IMU_ACCELS_IN_RANGE	(0x00000001u << 8)		/*!< Set to 1 if all accelerometers are within operating range. */
#define SBG_ECOM_IMU_GYROS_IN_RANGE		(0x00000001u << 9)		/*!< Set to 1 if all gyroscopes are within operating range. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 *	Structure that stores data for the SBG_ECOM_LOG_IMU_DATA message.
 */
typedef struct _SbgLogImuData
{
	uint32	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16	status;						/*!< IMU status bitmask. */
	float	accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
	float	gyroscopes[3];				/*!< X, Y, Z gyroscopes in rad.s^-1. */
	float	temperature;				/*!< Internal temperature in °C. */	
	float	deltaVelocity[3];			/*!< X, Y, Z delta velocity in m.s^-2. */
	float	deltaAngle[3];				/*!< X, Y, Z delta angle in rad.s^-1. */
} SbgLogImuData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_IMU_DATA message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseImuData(const void *pPayload, uint32 payloadSize, SbgLogImuData *pOutputData);

#endif
