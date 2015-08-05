/*!
 *	\file		commandsOutput.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to outputs.
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
#ifndef __COMMANDS_OUTPUT_H__
#define __COMMANDS_OUTPUT_H__

#include "commandsCommon.h"
#include "../SbgECanId.h"

//----------------------------------------------------------------------//
//- Output definitions												   -//
//----------------------------------------------------------------------//

/*!
 * List of ouput port available.
 */
typedef enum _SbgEComOutputPort
{
	SBG_ECOM_OUTPUT_PORT_A = 0		/*!< Main output port. */
} SbgEComOutputPort;

/*!
 * List of available legacy output format.
 */
typedef enum _SbgEComLegacyFormat
{
	SBG_ECOM_LEGACY_FORMAT_FLOAT = 0,		/*!< Data is output in floating point. */
	SBG_ECOM_LEGACY_FORMAT_FIXED			/*!< Data is output in fixed point. */
} SbgEComLegacyFormat;

/*!
 * List of available legacy output endiannesses.
 */
typedef enum _SbgEComLegacyEndian
{
	SBG_ECOM_LEGACY_LITTLE_ENDIAN = 0,		/*!< Data is output in little endian format. */
	SBG_ECOM_LEGACY_BIG_ENDIAN				/*!< Data is output in big endian format. */
} SbgEComLegacyEndian;		

/*!
 * List of output modes available.
 */
typedef enum _SbgEComOutputMode
{
	SBG_ECOM_OUTPUT_MODE_DISABLED 		= 0,		/*!< This output is disabled. */
	SBG_ECOM_OUTPUT_MODE_MAIN_LOOP 		= 1,		/*!< Output the message every main loop. */
	SBG_ECOM_OUTPUT_MODE_DIV_2 			= 2,		/*!< Output the message every 2 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_4 			= 4,		/*!< Output the message every 4 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_8 			= 8,		/*!< Output the message every 8 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_10 		= 10,		/*!< Output the message every 10 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_20 		= 20,		/*!< Output the message every 20 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_40 		= 40,		/*!< Output the message every 40 main loops. */
	SBG_ECOM_OUTPUT_MODE_DIV_200 		= 200,		/*!< Output the message every 200 main loops. */
	SBG_ECOM_OUTPUT_MODE_PPS 			= 10000,	/*!< Output the message on a Pulse Per Second event. */
	SBG_ECOM_OUTPUT_MODE_NEW_DATA 		= 10001,	/*!< Output sent when a new data is available. */
	SBG_ECOM_OUTPUT_MODE_VIRTUAL_ODO	= 10002,	/*!< Output is generated when a new virtual odometer event occurs. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_A		= 10003,	/*!< Output the message when a Sync A is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_B		= 10004,	/*!< Output the message when a Sync B is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_C		= 10005,	/*!< Output the message when a Sync C is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_D		= 10006		/*!< Output the message when a Sync D is received. */
} SbgEComOutputMode;

//----------------------------------------------------------------------//
//- Helper structure definitions									   -//
//----------------------------------------------------------------------//

/*!
 * Helper structure to configure legacy output.
 */
typedef struct _SbgEComLegacyConf
{
	uint32					mask;		/*!< Legacy output bit mask. */
	SbgEComLegacyFormat		format;		/*!< Format of the output. */
	SbgEComLegacyEndian		endian;		/*!< Endianness of the output. */
	SbgEComOutputMode		mode;		/*!< Mode of output. */
} SbgEComLegacyConf;

//----------------------------------------------------------------------//
//- Output commands                                                    -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[in]	classId						The class of the concerned log.
 *	\param[in]	msgId						The id of the concerned log.
 *	\param[out]	pMode						Pointer to a SbgEComOutputMode to contain the current output mode of the message.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, SbgEComMsgId msgId, SbgEComOutputMode *pMode);

/*!
 *	Set the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[in]	classId						The class of the concerned log.
 *	\param[in]	msgId						The id of the concerned log.
 *	\param[in]	mode						New output mode to set.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, SbgEComMsgId msgId, SbgEComOutputMode mode);

/*!
 *	Retrieve the configuration of one the message on the CAN interface.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	internalId					The internal message id.
 *	\param[out]	pMode						Pointer to a SbgEComOutputMode to contain the current output mode of the message.
 *	\param[out]	pUserId						The user defined message id.
 *	\param[out]	pExtended					TRUE if the user id uses the extended format.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdCanOutputGetConf(SbgEComHandle *pHandle, SbgECanMsgId internalId, SbgEComOutputMode *pMode, uint32 *pUserId, bool *pExtended);

/*!
 *	Set the configuration of one the message on the CAN interface
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	internalId					The internal message id.
 *	\param[in]	mode						Pointer to a SbgEComOutputMode containing the new output mode of the message.
 *	\param[in]	userId						The user defined message id.
 *	\param[in]	extended					TRUE if the user id uses the extended format.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdCanOutputSetConf(SbgEComHandle *pHandle, SbgECanMsgId internalId, SbgEComOutputMode mode, uint32 userId, bool extended);

/*!
 *	Retrieve the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *  \param[out] pConf						Pointer to a SbgEComLegacyConf structure to contain legacy configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetLegacyConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComLegacyConf *pConf);

/*!
 *	Set the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *  \param[in]  pConf						Pointer to a SbgEComLegacyConf structure containing new legacy configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetLegacyConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, const SbgEComLegacyConf *pConf);

/*!
 *	Retrieve the NMEA talker id of one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[out]	nmeaTalkerId				A 2-char array to contain the nmea talker id.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetNmeaTalkerId(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, char nmeaTalkerId[2]);

/*!
 *	Set the NMEA talker id of one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[out]	nmeaTalkerId				A 2-char array containint the new nmea talker id.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetNmeaTalkerId(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, const char nmeaTalkerId[2]);

#endif
