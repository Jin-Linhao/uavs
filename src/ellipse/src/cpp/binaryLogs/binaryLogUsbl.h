/*!
 *	\file		binaryLogUsbl.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		02 June 2014
 *
 *	\brief		This file is used to parse received USBL binary logs.
 *
 *	USBL binary logs contains underwater positioning data of a USBL beacon.
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
#ifndef __BINARY_LOG_USBL_H__
#define __BINARY_LOG_USBL_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log USBL status definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * USBL sensor status mask definitions
 */
#define SBG_ECOM_USBL_TIME_SYNC				(0x0001u << 0)			/*!< Set to 1 if the USBL sensor data is correctly time synchronized. */
#define	SBG_ECOM_USBL_DATA_VALID			(0x0001u << 1)			/*!< Set to 1 if the USB data represents a valid position. */


//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for USBL data.
 */
typedef struct _SbgLogUsblData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16	status;					/*!< Pressure sensor status bitmask. */

	double	latitude;				/*!< Latitude in degrees, positive north. */
	double	longitude;				/*!< Longitude in degrees, positive east. */
	double	depth;					/*!< Depth in meters below mean sea level (positive down). */

	float	latitudeAccuracy;		/*!< 1 sigma latitude accuracy in meters. */
	float	longitudeAccuracy;		/*!< 1 sigma longitude accuracy in meters. */
	float	depthAccuracy;			/*!< 1 sigma depth accuracy in meters. */
} SbgLogUsblData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_USBL message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUsblData(const void *pPayload, uint32 payloadSize, SbgLogUsblData *pOutputData);

#endif
