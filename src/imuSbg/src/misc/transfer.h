/*!
 *	\file		sbgSplitBuffer.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		19 November 2013
 *
 *	\brief		Handle large send/receive transfer for specific ECom Protocol commands.
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

#ifndef __TRANSFER_H__
#define __TRANSFER_H__

#include "../sbgCommon.h"
#include "../sbgECom.h"

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//

#define SBG_ECOM_PACKET_SIZE			(1300)		/*!< Max packet size transmitted in a single frame */

//----------------------------------------------------------------------//
//- Communication protocol structs and definitions                     -//
//----------------------------------------------------------------------//

/*!
 * Defines the ECom transfer commands
 */
typedef enum _EComTransferCmd
{
	ECOM_TRANSFER_START = 0,						/*!< Command to initiate a transfer. */
	ECOM_TRANSFER_DATA,								/*!< Command to transmit/receive data. */
	ECOM_TRANSFER_END								/*!< Command to end a transfer. */
} EComTransferCmd;

//----------------------------------------------------------------------//
//- Protocol transfer operations				                       -//
//----------------------------------------------------------------------//

/*!
 * Specific method to handle a large send into multiple frames.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	command					Original command asking for upload.
 * \param[in]	pBuffer					Pointer to the buffer containing the data to send.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful upload.
 */
SbgErrorCode sbgEComTransferSend(SbgEComHandle *pHandle, uint16 command, const void *pBuffer, uint32 size);

/*!
 * Specific method to handle a large receive from the device.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	command					Original command asking for download.
 * \param[in]	pBuffer					Pointer to the buffer where to write data.
 * \param[out]	pActualSize				The final size written into the buffer.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful download.
 */
SbgErrorCode sbgEComTransferReceive(SbgEComHandle *pHandle, uint16 command, void *pBuffer, uint32 *pActualSize, uint32 bufferSize);

#endif
