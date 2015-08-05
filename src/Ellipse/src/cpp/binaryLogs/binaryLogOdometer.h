/*!
 *	\file		binaryLogOdometer.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		25 February 2013
 *
 *	\brief		This file is used to parse received odometer binary logs.
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
#ifndef __BINARY_LOG_ODOMETER_H__
#define __BINARY_LOG_ODOMETER_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log odometer status definitions                                    -//
//----------------------------------------------------------------------//

#define	SBG_ECOM_ODO_REAL_MEAS			(0x00000001 << 0)			/*!< Set to 1 if this log comes from a real pulse measurement or from a timeout. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for odometer data.
 */
typedef struct _SbgLogOdometerData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16	status;					/*!< Odometer velocity status bitmask. */
	float	velocity;				/*!< Velocity in m.s^-1 in the odometer direction. */
} SbgLogOdometerData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_ODO_VEL message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseOdometerData(const void *pPayload, uint32 payloadSize, SbgLogOdometerData *pOutputData);

#endif
