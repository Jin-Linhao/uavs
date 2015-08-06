/*!
 *	\file		binaryLogDebug.h
 *  \author		SBG Systems (Alexis Guinamard)
 *	\date		03 October 2013
 *
 *	\brief		This file is used to parse received debug frames
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
#ifndef __BINARY_LOG_DEBUG_H__
#define __BINARY_LOG_DEBUG_H__

#include "../sbgCommon.h"


//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for debug data.
 */
typedef struct _SbgLogDebug0
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	float	data[64];				/*!< Debug data */
} SbgLogDebug0Data;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the debug message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDebug0Data(const void *pPayload, uint32 payloadSize, SbgLogDebug0Data *pOutputData);

#endif
