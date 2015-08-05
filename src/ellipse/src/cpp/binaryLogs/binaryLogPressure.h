/*!
 *	\file		binaryLogPressure.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		30 April 2014
 *
 *	\brief		This file is used to parse received Pressure binary logs.
 *
 *	Pressure binary logs contains both the pressure and altitude/depth for a sensor
 *	such as an altimeter or a subsea depth sensor.
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
#ifndef __BINARY_LOG_PRESSURE_H__
#define __BINARY_LOG_PRESSURE_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log Pressure status definitions                                    -//
//----------------------------------------------------------------------//

/*!
 * Pressure sensor status mask definitions
 */
#define SBG_ECOM_PRESSURE_TIME_SYNC			(0x0001u << 0)			/*!< Set to 1 if the Pressure sensor data is correctly time synchronized. */
#define	SBG_ECOM_PRESSURE_PRESSURE_VALID	(0x0001u << 1)			/*!< Set to 1 if the pressure field is filled and valid. */
#define	SBG_ECOM_PRESSURE_HEIGHT_VALID		(0x0001u << 2)			/*!< Set to 1 if the height field is filled and valid. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for Pressure data.
 */
typedef struct _SbgLogPressureData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16	status;					/*!< Pressure sensor status bitmask. */
	float	pressure;				/*!< Pressure value in Pascals. */
	float	height;					/*!< Altitude or depth in meters (positive up). */
} SbgLogPressureData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_PRESSURE message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParsePressureData(const void *pPayload, uint32 payloadSize, SbgLogPressureData *pOutputData);

#endif
