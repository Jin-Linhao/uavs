/*!
 *	\file		commandsGnss.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to GNSS module.
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
#ifndef __COMMANDS_GNSS_H__
#define __COMMANDS_GNSS_H__

#include "commandsCommon.h"

//----------------------------------------------------------------------//
//- GNSS definitions												   -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- GNSS configuration												   -//
//----------------------------------------------------------------------//

/*!
 * Holds all necessary information for GNSS module alignment.
 */
typedef struct _SbgEComGnssAlignmentInfo
{
	float	leverArmX;			/*!< GNSS antenna lever arm in IMU X axis */
	float	leverArmY;			/*!< GNSS antenna lever arm in IMU Y axis */
	float	leverArmZ;			/*!< GNSS antenna lever arm in IMU Z axis */
	float	pitchOffset;		/*!< Pitch offset for dual antenna GNSS */
	float	yawOffset;			/*!< Pitch offset for dual antenna GNSS */
	float	antennaDistance;	/*!< Distance between two GNSS antennas */
} SbgEComGnssAlignmentInfo;

/*!
 * Holds all necessary information for GNSS module data rejection.
 */
typedef struct _SbgEComGnssRejectionConf
{
	SbgEComRejectionMode	position;		/*!< Rejection mode for position. */
	SbgEComRejectionMode	velocity;		/*!< Rejection mode for velocity. */
	SbgEComRejectionMode	course;			/*!< Rejection mode for course over ground. */
	SbgEComRejectionMode	hdt;			/*!< Rejection mode for true heading. */
} SbgEComGnssRejectionConf;

//----------------------------------------------------------------------//
//- GNSS public commands		                                       -//
//----------------------------------------------------------------------//

/*!
 *	Set the error model for the first GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size);

/*!
 *	Retrieve GNSS error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo);

/*!
 *	Retrieve the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf);

/*!
 *	Set the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf);

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf);

/*!
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf);

#endif
