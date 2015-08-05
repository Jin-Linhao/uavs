/*!
 *	\file		binaryLogs.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		06 February 2013
 *
 *	\brief		This file is used to parse received binary logs.
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
#ifndef __BINARY_LOGS_H__
#define __BINARY_LOGS_H__

#include "../sbgCommon.h"
#include "../sbgEComIds.h"
#include "binaryLogStatus.h"
#include "binaryLogEkf.h"
#include "binaryLogGps.h"
#include "binaryLogImu.h"
#include "binaryLogOdometer.h"
#include "binaryLogUtc.h"
#include "binaryLogMag.h"
#include "binaryLogShipMotion.h"
#include "binaryLogDvl.h"
#include "binaryLogPressure.h"
#include "binaryLogUsbl.h"
#include "binaryLogEvent.h"
#include "binaryLogDebug.h"

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 *	Union used to store received logs data.
 */
typedef union _SbgBinaryLogData
{
	SbgLogStatusData		statusData;			/*!< Stores data for the SBG_ECOM_LOG_STATUS message. */
	SbgLogImuData			imuData;			/*!< Stores data for the SBG_ECOM_LOG_IMU_DATA message. */
	SbgLogEkfEulerData		ekfEulerData;		/*!< Stores data for the SBG_ECOM_LOG_EKF_EULER message. */
	SbgLogEkfQuatData		ekfQuatData;		/*!< Stores data for the SBG_ECOM_LOG_EKF_QUAT message. */
	SbgLogEkfNavData		ekfNavData;			/*!< Stores data for the SBG_ECOM_LOG_EKF_NAV message. */
	SbgLogShipMotionData	shipMotionData;		/*!< Stores data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message. */
	SbgLogOdometerData		odometerData;		/*!< Stores data for the SBG_ECOM_LOG_ODO_VEL message. */
	SbgLogUtcData			utcData;			/*!< Stores data for the SBG_ECOM_LOG_UTC_TIME message. */
	SbgLogGpsPos			gpsPosData;			/*!< Stores data for the SBG_ECOM_LOG_GPS_POS message. */
	SbgLogGpsVel			gpsVelData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_VEL message. */
	SbgLogGpsHdt			gpsHdtData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_HDT message. */
	SbgLogGpsRaw			gpsRawData;			/*!< Stores data for the SBG_ECOM_LOG_GPS#_RAW message. */
	SbgLogMag				magData;			/*!< Stores data for the SBG_ECOM_LOG_MAG message. */
	SbgLogMagCalib			magCalibData;		/*!< Stores data for the SBG_ECOM_LOG_MAG_CALIB message. */
	SbgLogDvlData			dvlData;			/*!< Stores data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK message. */
	SbgLogPressureData		pressureData;		/*!< Stores data for the SBG_ECOM_LOG_PRESSURE message. */
	SbgLogUsblData			usblData;			/*!< Stores data for the SBG_ECOM_LOG_USBL message. */
	SbgLogEvent				eventMarker;		/*!< Stores data for the SBG_ECOM_LOG_EVENT_# message. */
	SbgLogDebug0Data		debug0Data;			/*!< Stores debug information */
} SbgBinaryLogData;

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Test if the command is a binary log one.
 *	\param[in]	cmdId					The command id to test.
 *	\return								TRUE if the command id corresponds to a binary log.
 */
SBG_INLINE bool sbgEComBinaryLogIsCmdValid(uint16 cmdId)
{
	//
	// Test if this command id is part of the enum
	//
	if ( (cmdId >= SBG_ECOM_LOG_STATUS) && (cmdId <= SBG_ECOM_LOG_DEBUG_0) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*!
 *	Parse an incoming log and fill the output union.
 *	\param[in]	command						Received command that should be a log id.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output union that stores parsed data.
 */
SbgErrorCode sbgEComBinaryLogParse(uint16 command, const void *pPayload, uint32 payloadSize, SbgBinaryLogData *pOutputData);

#endif
