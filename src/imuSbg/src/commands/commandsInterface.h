/*!
 *	\file		commandsInterface.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to interfaces.
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
#ifndef __COMMANDS_INTERFACE_H__
#define __COMMANDS_INTERFACE_H__

#include "commandsCommon.h"

//----------------------------------------------------------------------//
//- Serial interface definitions									   -//
//----------------------------------------------------------------------//

/*!
 * List of serial interfaces available.
 */
typedef enum _SbgEComPortId
{
	SBG_ECOM_IF_COM_A = 0,		/*!< Main communication interface. Full duplex. */
	SBG_ECOM_IF_COM_B = 1,		/*!< Auxiliary input interface for RTCM. */
	SBG_ECOM_IF_COM_C = 2,		/*!< Auxiliary communication interface. Full duplex. */
	SBG_ECOM_IF_COM_D = 3		/*!< Auxiliary input interface. */
} SbgEComPortId;

/*!
 * List of serial modes available.
 */
typedef enum _SbgEComPortMode
{
	SBG_ECOM_UART_MODE_OFF = 0,		/*!< This interface is turned OFF. */
	SBG_ECOM_UART_MODE_232 = 1,		/*!< This interface is using RS-232 communications. */
	SBG_ECOM_UART_MODE_422 = 2		/*!< This interface is using RS-422 communications. */
} SbgEComPortMode;

//----------------------------------------------------------------------//
//- Serial interface configuration									   -//
//----------------------------------------------------------------------//

/*!
 * Helper structure to configure a serial interface
 */
typedef struct _SbgEComInterfaceConf
{
	uint32				baudRate;	/*!< The baud rate of the interface */
	SbgEComPortMode		mode;		/*!< The mode of the interface */
} SbgEComInterfaceConf;

//----------------------------------------------------------------------//
//- CAN interface definitions										   -//
//----------------------------------------------------------------------//

/*!
 * Enum containing the list of all available bit rates (in KB/s).
 */
typedef enum _SbgEComCanBitRate
{
	SBG_ECOM_CAN_BITRATE_DISABLED 	= 0,			/*!< The CAN interface is disabled. */
	SBG_ECOM_CAN_BITRATE_10	 		= 10,			/*!< 10Kb/s. */
	SBG_ECOM_CAN_BITRATE_20 		= 20,			/*!< 20Kb/s. */
	SBG_ECOM_CAN_BITRATE_25	 		= 25,			/*!< 25Kb/s. */
	SBG_ECOM_CAN_BITRATE_50	 		= 50,			/*!< 50Kb/s. */
	SBG_ECOM_CAN_BITRATE_100 		= 100,			/*!< 100Kb/s. */
	SBG_ECOM_CAN_BITRATE_125 		= 125,			/*!< 125Kb/s. */
	SBG_ECOM_CAN_BITRATE_250 		= 250,			/*!< 250Kb/s. */
	SBG_ECOM_CAN_BITRATE_500 		= 500,			/*!< 500Kb/s. */
	SBG_ECOM_CAN_BITRATE_750 		= 750,			/*!< 750Kb/s. */
	SBG_ECOM_CAN_BITRATE_1000 		= 1000			/*!< 1Mb/s. */
} SbgEComCanBitRate;

//----------------------------------------------------------------------//
//- Interface commands                                                 -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	interfaceId					The interface from which the configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComInterfaceConf struct to hold configuration of the interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceGetUartConf(SbgEComHandle *pHandle, SbgEComPortId interfaceId, SbgEComInterfaceConf *pConf);

/*!
 *	Set the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	interfaceId					The interface from which the configuration is to be retrieved.
 *	\param[in]	pConf						Pointer to a SbgEComInterfaceConf struct that holds the new configuration for the interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceSetUartConf(SbgEComHandle *pHandle, SbgEComPortId interfaceId, const SbgEComInterfaceConf *pConf);

/*!
 *	Retrieve the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pBitRate					The bitrate of the CAN interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceGetCanConf(SbgEComHandle *pHandle, SbgEComCanBitRate *pBitrate);

/*!
 *	Set the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	bitRate						The bitrate of the CAN interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceSetCanConf(SbgEComHandle *pHandle, SbgEComCanBitRate bitrate);

#endif
