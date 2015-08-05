/*!
 *	\file		binaryLogMag.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		12 March 2013
 *
 *	\brief		This file is used to parse received magnetometer binary logs.
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
#ifndef __BINARY_LOG_MAG_H__
#define __BINARY_LOG_MAG_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log magnetometers status definitions                               -//
//----------------------------------------------------------------------//

/*!
 * Log magnetometer data status mask definitions
 */
#define	SBG_ECOM_MAG_MAG_X_BIT			(0x00000001u << 0)		/*!< Set to 1 if the magnetometer X passes Built In Test. */
#define	SBG_ECOM_MAG_MAG_Y_BIT			(0x00000001u << 1)		/*!< Set to 1 if the magnetometer Y passes Built In Test. */
#define	SBG_ECOM_MAG_MAG_Z_BIT			(0x00000001u << 2)		/*!< Set to 1 if the magnetometer Z passes Built In Test. */

#define	SBG_ECOM_MAG_ACCEL_X_BIT		(0x00000001u << 3)		/*!< Set to 1 if the accelerometer X passes Built In Test. */
#define	SBG_ECOM_MAG_ACCEL_Y_BIT		(0x00000001u << 4)		/*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define	SBG_ECOM_MAG_ACCEL_Z_BIT		(0x00000001u << 5)		/*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define	SBG_ECOM_MAG_MAGS_IN_RANGE		(0x00000001u << 6)		/*!< Set to 1 if all magnetometers are within operating range. */
#define SBG_ECOM_MAG_ACCELS_IN_RANGE	(0x00000001u << 7)		/*!< Set to 1 if all accelerometers are within operating range. */

#define SBG_ECOM_MAG_CALIBRATION_OK		(0x00000001u << 8)		/*!< Set to 1 if the magnetometers seems to be calibrated. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 *	Structure that stores data for the SBG_ECOM_LOG_MAG message.
 */
typedef struct _SbgLogMag
{
	uint32	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16	status;						/*!< Magnetometer status bitmask. */
	float	magnetometers[3];			/*!< X, Y, Z magnetometer data in A.U. */
	float	accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
} SbgLogMag;

/*!
 *	Structure that stores data for the SBG_ECOM_LOG_MAG_CALIB message.
 */
typedef struct _SbgLogMagCalib
{
	uint32	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16	reserved;					/*!< Reserved for future use. */
	uint8	magData[16];				/*!< Magnetometers calibration data. */
} SbgLogMagCalib;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_MAG message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagData(const void *pPayload, uint32 payloadSize, SbgLogMag *pOutputData);

/*!
 *	Parse data for the SBG_ECOM_LOG_MAG_CALIB message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagCalibData(const void *pPayload, uint32 payloadSize, SbgLogMagCalib *pOutputData);

#endif
