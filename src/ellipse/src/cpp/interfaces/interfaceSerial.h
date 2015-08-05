/*!
 *	\file		interfaceSerial.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		06 February 2013
 *
 *	\brief		This file implements a serial interface.
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

#ifndef __INTERFACE_SERIAL_H__
#define __INTERFACE_SERIAL_H__

#include "interface.h"

//----------------------------------------------------------------------//
//- Definitions                                                        -//
//----------------------------------------------------------------------//
#define SBG_IF_SERIAL_TX_BUFFER_SIZE			(4096u)					/*!< Define the transmission buffer size for the serial port. */
#define SBG_IF_SERIAL_RX_BUFFER_SIZE			(4096u)					/*!< Define the reception buffer size for the serial port. */

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Initialize a serial interface for read and write operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	deviceName						Serial interface location (COM21 , /dev/ttys0, depending on platform).
 *	\param[in]	baudRate						Serial interface baud rate in bps.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceSerialCreate(SbgInterface *pHandle, const char *deviceName, uint32 baudRate);

/*!
 *	Destroy an interface initialized using sbgInterfaceSerialCreate.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SbgErrorCode sbgInterfaceSerialDestroy(SbgInterface *pHandle);

/*!
 * Flush the RX and TX buffers (remove all old data)
 * \param[in]	handle				Valid handle on an initialized interface.
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgInterfaceSerialFlush(SbgInterface *pHandle);

/*!
 * Change the serial interface baud rate immediatly.
 * \param[in]	handle				Valid handle on an initialized interface.
 * \param[in]	baudRate			The new baudrate to apply in bps.
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgInterfaceSerialChangeBaudrate(SbgInterface *pHandle, uint32 baudRate);

//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceSerialWrite(SbgInterface *pHandle, const void *pBuffer, uint32 bytesToWrite);

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceSerialRead(SbgInterface *pHandle, void *pBuffer, uint32 *pReadBytes, uint32 bytesToRead);

#endif /* __INTERFACE_SERIAL_H__ */
