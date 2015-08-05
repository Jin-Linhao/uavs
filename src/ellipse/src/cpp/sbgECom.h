/*!
 *	\file		sbgECom.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 February 2013
 *
 *	\brief		Contains main sbgECom methods.
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

/*!
 *	\mainpage SBG Systems Enhanced Communication library documentation
 *	Welcome to the sbgECom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgECom library.
 */

#ifndef __SBG_ECOM_H__
#define __SBG_ECOM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgCommon.h"
#include "sbgEComIds.h"
#include "protocol/protocol.h"
#include "binaryLogs/binaryLogs.h"

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgEComHandle SbgEComHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	logCmd									Contains the binary received log command id.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
typedef SbgErrorCode (*SbgEComReceiveFunc)(SbgEComHandle *pHandle, SbgEComCmdId logCmd, const SbgBinaryLogData *pLogData, void *pUserArg);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 */
struct _SbgEComHandle
{
	SbgEComProtocol				  protocolHandle;			/*!< Handle on the protocol system. */
	SbgEComReceiveFunc			 pReceiveCallback;			/*!< Pointer on the method called each time a new binary log is received. */
	void						*pUserArg;					/*!< Optional user supplied argument for callbacks. */
};

//----------------------------------------------------------------------//
//- Public methods declarations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the protocol system used to communicate with the product and return the created handle.
 *	\param[out]	pHandle							Pointer used to store the allocated and initialized sbgECom handle.
 *	\param[in]	pInterface						Interface to use for read/write operations.
 *	\return										SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface);

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pHandle							A valid sbgECom handle to close.
 *	\return										SBG_NO_ERROR if we have closed and released the sbgECom system.
 */
SbgErrorCode sbgEComClose(SbgEComHandle *pHandle);

/*!
 *	Handle incoming logs.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming logs parsing.
 */
SbgErrorCode sbgEComHandle(SbgEComHandle *pHandle);

/*!
 *	Wait until any command that is not a output log is recevied during a specific time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	pCommand				Pointer used to hold the received command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveAnyCmd(SbgEComHandle *pHandle, uint16 *pCommand, void *pData, uint32 *pSize, uint32 maxSize, uint32 timeOut);

/*!
 *	Wait for a specific command to be received given a time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[in]	command					The command we would like to receive.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveCmd(SbgEComHandle *pHandle, uint16 command, void *pData, uint32 *pSize, uint32 maxSize, uint32 timeOut);

/*!
 *	Define the callback that should be called each time a new binary log is received.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\param[in]	pReceiveCallback				Pointer on the callback to call when a new log is received.
 *	\param[in]	pUserArg						Optional user argument that will be passed to the callback method.
 *	\return										SBG_NO_ERROR if the callback and user argument have been defined successfully.
 */
SbgErrorCode sbgEComSetReceiveCallback(SbgEComHandle *pHandle, SbgEComReceiveFunc pReceiveCallback, void *pUserArg);

/*!
 *	Returns an integer representing the version of the sbgCom library.
 *	\return										An integer representing the version of the sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgEComGetVersion(void);

/*!
 *	Retreive the sbgCom library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCom library version.
 */
const char *sbgEComGetVersionAsString(void);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgEComErrorToString(SbgErrorCode errorCode, char errorMsg[256]);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	/* __SBG_ECOM_H__ */
