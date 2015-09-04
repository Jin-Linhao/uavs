/*!
 *	\file		commandsCommon.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file groups all common definitions required by all commands.
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
#ifndef __COMMANDS_COMMON_H__
#define __COMMANDS_COMMON_H__

#include "../sbgECom.h"

//----------------------------------------------------------------------//
//- Defintions                                                         -//
//----------------------------------------------------------------------//

#define SBG_ECOM_DEFAULT_CMD_TIME_OUT	(500)			/*!< Default time out in ms for commands reception. */

/*!
 * List of all rejection modes for aiding inputs.
 */
typedef enum _SbgEComRejectionMode
{
	SBG_ECOM_NEVER_ACCEPT_MODE		= 0,		/*!< Measurement is not taken into account. */
	SBG_ECOM_AUTOMATIC_MODE			= 1,		/*!< Measurement is rejected if inconsistent with current estimate (depending on error model). */
	SBG_ECOM_ALWAYS_ACCEPT_MODE		= 2			/*!< Measurement is always accepted. */
} SbgEComRejectionMode;

/*!
 * List of all axis directions for modules/sensor alignment.
 */
typedef enum _SbgEComAxisDirection
{
	SBG_ECOM_ALIGNMENT_FORWARD		= 0,		/*!< IMU/module Axis is turned in vehicle's forward direction. */
	SBG_ECOM_ALIGNMENT_BACKWARD		= 1,		/*!< IMU/module Axis is turned in vehicle's backward direction. */
	SBG_ECOM_ALIGNMENT_LEFT			= 2,		/*!< IMU/module Axis is turned in vehicle's left direction. */
	SBG_ECOM_ALIGNMENT_RIGHT		= 3,		/*!< IMU/module Axis is turned in vehicle's right direction. */
	SBG_ECOM_ALIGNMENT_UP			= 4,		/*!< IMU/module Axis is turned in vehicle's up direction. */
	SBG_ECOM_ALIGNMENT_DOWN			= 5			/*!< IMU/module Axis is turned in vehicle's down direction. */
} SbgEComAxisDirection;

/*!
 * Common model information structure.
 * This is used for motion profile or Magnetometer,Gps, or other aiding sensor error model.
 */
typedef struct _SbgEComModelInfo
{
	uint32	id;				/*!< Identifier of the model */
	uint32	revision;		/*!< Revision of the model */
} SbgEComModelInfo;

//----------------------------------------------------------------------//
//- Generic command definitions                                        -//
//----------------------------------------------------------------------//

/*!
 *	Generic function to retrieve error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						Original command.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGenericGetModelInfo(SbgEComHandle *pHandle, uint16 command, SbgEComModelInfo *pModelInfo);

//----------------------------------------------------------------------//
//- Ack commands				                                       -//
//----------------------------------------------------------------------//

/*!
 *	Wait for an ACK for a specified amount of time.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						The command we would like the ACK for.
 *	\param[in]	timeOut						Time out in ms during which we can receive the ACK.
 *	\return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint16 command, uint32 timeOut);

#endif
