/*!
 *	\file		sbgStreamBuffer.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		02 January 2013
 *
 *	\brief		Used to read/write data from/to a memory buffer stream.
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

#ifndef __SBG_STREAM_BUFFER_H__
#define __SBG_STREAM_BUFFER_H__

#include "../sbgCommon.h"
#include "sbgSwap.h"
#include <string.h>
#include "../interfaces/interfaceSerial.h"

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Stream buffer modes.
 */
typedef enum _SbgSBMode
{
	SB_MODE_READ,						/*!< This stream buffer can perform read operations. */
	SB_MODE_WRITE						/*!< This stream buffer can perform write operations. */
} SbgSBMode;

/*!
 * Enum used to define all seek modes
 */
typedef enum _SbgSBSeekOrigin
{
	SB_SEEK_SET,							/*!< The offset is referenced to the begining of the stream. */
	SB_SEEK_CUR_INC,						/*!< The offset is referenced to the current cursor position and increment the current cursor. */
	SB_SEEK_CUR_DEC,						/*!< The offset is referenced to the current cursor position and decrement the current cursor. */
	SB_SEEK_END								/*!< The offset is referenced to the end of the stream. */
} SbgSBSeekOrigin;

/*!
 * Defines a stream buffer.
 */
typedef struct _SbgStreamBuffer
{
	SbgSBMode			 modes;				/*!< Defines the stream buffer modes (read/write). */
	uint32				 bufferSize;		/*!< Size in bytes of the linked buffer. */
	uint8				*pBufferPtr;		/*!< Pointer to the buffer linked with this stream. */
	uint8				*pCurrentPtr;		/*!< Current pointer within the buffer. */
	SbgErrorCode		 errorCode;			/*!< Current error code on stream buffer. */
} SbgStreamBuffer;

//----------------------------------------------------------------------//
//- Common operations methods                                          -//
//----------------------------------------------------------------------//

/*!
 * Initialize a stream buffer for both read and write operations and link it to a buffer.
 * \param[in]	pHandle									Handle on an allocated stream buffer.
 * \param[in]	pLinkedBuffer							Pointer on an allocated buffer to link with this stream.
 * \param[in]	bufferSize								Size in bytes of the linked buffer.
 * \return												SBG_NO_ERROR if the stream buffer has been initialized successfully.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferInitForWrite(SbgStreamBuffer *pHandle, void *pLinkedBuffer, uint32 bufferSize)
{
	//
	// Initialize stream parameters
	//
	pHandle->modes = SB_MODE_WRITE;
	pHandle->bufferSize = bufferSize;
	pHandle->errorCode = SBG_NO_ERROR;

	//
	// Initialize the buffer
	//
	pHandle->pBufferPtr = (uint8*)pLinkedBuffer;
	pHandle->pCurrentPtr = (uint8*)pLinkedBuffer;

	//
	// For now, we don't handle any error, maybe we could add checks in debug mode only
	//
	return SBG_NO_ERROR;
}

/*!
 * Initialize a stream buffer for both read and write operations and link it to a buffer.
 * \param[in]	pHandle									Handle on an allocated stream buffer.
 * \param[in]	pLinkedBuffer							Pointer on an allocated buffer to link with this stream.
 * \param[in]	bufferSize								Size in bytes of the linked buffer.
 * \return												SBG_NO_ERROR if the stream buffer has been initialized successfully.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferInitForRead(SbgStreamBuffer *pHandle, const void *pLinkedBuffer, uint32 bufferSize)
{
	//
	// Initialize stream parameters
	//
	pHandle->modes = SB_MODE_READ;
	pHandle->bufferSize = bufferSize;
	pHandle->errorCode = SBG_NO_ERROR;

	//
	// Initialize the buffer
	//
	pHandle->pBufferPtr = (uint8*)pLinkedBuffer;
	pHandle->pCurrentPtr = (uint8*)pLinkedBuffer;

	//
	// For now, we don't handle any error, maybe we could add checks in debug mode only
	//
	return SBG_NO_ERROR;
}

/*!
 * Return the error code that has occurred on the last stream buffer operation.
 * \param[in]	pHandle					Pointer to a valid Stream Buffer handle
 * \return								Last stream buffer error code
 */
SBG_INLINE SbgErrorCode sbgStreamBufferGetLastError(SbgStreamBuffer *pHandle)
{
	//
	// Return error code
	//
	return pHandle->errorCode;
}

/*!
 * Clear the last error code that has occurred on the last stream buffer operation.
 * \param[in]	pHandle					Pointer to a valid Stream Buffer handle
 */
SBG_INLINE void sbgStreamBufferClearLastError(SbgStreamBuffer *pHandle)
{
	//
	// Return error code
	//
	pHandle->errorCode = SBG_NO_ERROR;
}

/*!
 * Returns the size in bytes of this stream.
 * The size is the linked buffer total size in bytes.
 * For example, for a SbgStreamBuffer linked with a buffer of 256 bytes,
 * this method will always returns 256 even if no data has been written or read.
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \return								The allocated size of the linked buffer in bytes.
 */
SBG_INLINE uint32 sbgStreamBufferGetSize(SbgStreamBuffer *pHandle)
{
	//
	// Return the linked buffer size
	//
	return pHandle->bufferSize;
}

/*!
 * Returns the length in bytes of this stream.
 * The length is computed using the current cursor position.
 * If no data has been read or written, this method will return 0.
 * If 4 uint32 has been written, it should return 16.
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \return								The current cursor position in bytes.
 */
SBG_INLINE uint32 sbgStreamBufferGetLength(SbgStreamBuffer *pHandle)
{
	//
	// Return the number of bytes between the begin of the stream and the current pointer
	//
	//return ((uint32)pHandle->pCurrentPtr - (uint32)pHandle->pBufferPtr);
	return (uint32)(pHandle->pCurrentPtr - pHandle->pBufferPtr);
}

/*!
 * Returns the available space in this stream.
 * The available space is just the delta between the linked buffer size
 * and the current buffer length (cursor position).
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \return								The space available in this stream buffer in bytes.
 */
SBG_INLINE uint32 sbgStreamBufferGetSpace(SbgStreamBuffer *pHandle)
{
	//
	// Return the space left in bytes
	//
	return sbgStreamBufferGetSize(pHandle) - sbgStreamBufferGetLength(pHandle);
}

/*!
 * Move the current cursor position.
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \param[in]	offset					Offset in bytes to apply (only positive).
 * \param[in]	origin					Origin reference point to apply the offset from.
 * \return								SBG_NO_ERROR if the stream current cursor position has been moved.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferSeek(SbgStreamBuffer *pHandle, uint32 offset, SbgSBSeekOrigin origin)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// TODO: test for buffer overflow
	//

	//
	// According to the origin reference point
	//
	switch (origin)
	{
	case SB_SEEK_SET:
		pHandle->pCurrentPtr = pHandle->pBufferPtr + offset;
		break;
	case SB_SEEK_CUR_INC:
		pHandle->pCurrentPtr += offset;
		break;
	case SB_SEEK_CUR_DEC:
		pHandle->pCurrentPtr -= offset;
		break;
	case SB_SEEK_END:
		pHandle->pCurrentPtr = pHandle->pBufferPtr + (pHandle->bufferSize - offset);
		break;
	default:
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Returns the current offset in bytes from the beginning of the stream.
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \return								Current offset in bytes from the beginning.
 */
SBG_INLINE uint32 sbgStreamBufferTell(SbgStreamBuffer *pHandle)
{
	//return (uint32)pHandle->pCurrentPtr - (uint32)pHandle->pBufferPtr;
	return (uint32)(pHandle->pCurrentPtr - pHandle->pBufferPtr);
}

/*!
 * Returns a pointer on the internal buffer.
 * \param[in]	pHandle					Valid handle on a stream buffer.
 * \return								Pointer on the begining of the internal buffer.
 */
SBG_INLINE void *sbgStreamBufferGetLinkedBuffer(SbgStreamBuffer *pHandle)
{
	return pHandle->pBufferPtr;
}

/*!
 *	Returns a pointer on the internal buffer at the current cursor.
 *	\param[in]	pHandle					Valid handle on a stream buffer.
 *	\return								Pointer on the current cursor of the internal buffer.
 */
SBG_INLINE void *sbgStreamBufferGetCursor(SbgStreamBuffer *pHandle)
{
	return pHandle->pCurrentPtr;
}

//----------------------------------------------------------------------//
//- Read operations methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Read an int8 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int8 sbgStreamBufferReadInt8(SbgStreamBuffer *pHandle)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int8))
	{
		//
		// Read the byte
		//
		return *((int8*)(pHandle->pCurrentPtr++));
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an uint8 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint8 sbgStreamBufferReadUint8(SbgStreamBuffer *pHandle)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8))
	{
		//
		// Read the byte
		//
		return *(pHandle->pCurrentPtr++);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an int16 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int16 sbgStreamBufferReadInt16(SbgStreamBuffer *pHandle)
{
	int16 bytesValues[2];

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16))
	{
		//
		// Read the each bytes
		//
		bytesValues[0] = *(pHandle->pCurrentPtr++);
		bytesValues[1] = *(pHandle->pCurrentPtr++);

		//
		// Data are stored in little endian
		//
		return bytesValues[0] | (bytesValues[1] << 8);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an uint16 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint16 sbgStreamBufferReadUint16(SbgStreamBuffer *pHandle)
{
	uint16 bytesValues[2];

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16))
	{
		//
		// Read the each bytes
		//
		bytesValues[0] = *(pHandle->pCurrentPtr++);
		bytesValues[1] = *(pHandle->pCurrentPtr++);

		//
		// Data are stored in little endian
		//
		return bytesValues[0] | (bytesValues[1] << 8);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an int32 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int32 sbgStreamBufferReadInt32(SbgStreamBuffer *pHandle)
{
	int32 bytesValues[4];

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32))
	{
		//
		// Read the each bytes
		//
		bytesValues[0] = *(pHandle->pCurrentPtr++);
		bytesValues[1] = *(pHandle->pCurrentPtr++);
		bytesValues[2] = *(pHandle->pCurrentPtr++);
		bytesValues[3] = *(pHandle->pCurrentPtr++);

		//
		// Data are stored in little endian
		//
		return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an uint32 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint32 sbgStreamBufferReadUint32(SbgStreamBuffer *pHandle)
{
	uint32 bytesValues[4];

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32))
	{
		//
		// Read the each bytes
		//
		bytesValues[0] = *(pHandle->pCurrentPtr++);
		bytesValues[1] = *(pHandle->pCurrentPtr++);
		bytesValues[2] = *(pHandle->pCurrentPtr++);
		bytesValues[3] = *(pHandle->pCurrentPtr++);

		//
		// Data are stored in little endian
		//
		return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0;
	}
}

/*!
 * Read an int64 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int64 sbgStreamBufferReadInt64(SbgStreamBuffer *pHandle)
{
	int64 lowPart;
	int64 highPart;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64))
	{
		//
		// Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
		//
		lowPart = sbgStreamBufferReadUint32(pHandle);
		highPart = sbgStreamBufferReadUint32(pHandle);

		//
		// Data are stored in little endian
		//
		return lowPart | (highPart << 32);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0ll;
	}
}

/*!
 * Read an uint64 from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint64 sbgStreamBufferReadUint64(SbgStreamBuffer *pHandle)
{
	uint64 lowPart;
	uint64 highPart;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64))
	{
		//
		// Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
		//
		lowPart = sbgStreamBufferReadUint32(pHandle);
		highPart = sbgStreamBufferReadUint32(pHandle);

		//
		// Data are stored in little endian
		//
		return lowPart | (highPart << 32);
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0ull;
	}
}

/*!
 * Read an float from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE float sbgStreamBufferReadFloat(SbgStreamBuffer *pHandle)
{
	FloatNint floatInt;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(float))
	{
		//
		// Read the float as an uint32
		//
		floatInt.valU = sbgStreamBufferReadUint32(pHandle);

		//
		// Return the float using an union to avoid compiller cast
		//
		return floatInt.valF;
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0.0f;
	}
}

/*!
 * Read an double from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE double sbgStreamBufferReadDouble(SbgStreamBuffer *pHandle)
{
	DoubleNint doubleInt;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(double))
	{
		//
		// Read the float as an uint64
		//
		doubleInt.valU = sbgStreamBufferReadUint64(pHandle);

		//
		// Return the double using an union to avoid compiller cast
		//
		return doubleInt.valF;
	}
	else
	{
		//
		// We have a buffer overflow so return 0
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return 0.0;
	}
}

/*!
 * Read a buffer from a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \param[out]	pBuffer				Allocated buffer used to hold read data.
 * \param[in]	numBytesToRead		Number of bytes to read from the stream buffer and to store in pBuffer.
 * \return							SBG_NO_ERROR if the data has been read.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferReadBuffer(SbgStreamBuffer *pHandle, void *pBuffer, uint32 numBytesToRead)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test if enough bytes in stream
	//
	if (sbgStreamBufferGetSpace(pHandle) >= numBytesToRead)
	{
		//
		// Copy from the stream buffer to the output buffer
		//
		memcpy(pBuffer, pHandle->pCurrentPtr, numBytesToRead);

		//
		// Update the current pointer
		//
		pHandle->pCurrentPtr += numBytesToRead;
	}
	else
	{
		//
		// Not enough data in stream
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		errorCode = SBG_BUFFER_OVERFLOW;
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Write operations methods                                           -//
//----------------------------------------------------------------------//

/*!
 * Write an int8 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt8(SbgStreamBuffer *pHandle, int8 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int8))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an uint8 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint8(SbgStreamBuffer *pHandle, uint8 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint8))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}
/*!
 * Write an int16 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt16(SbgStreamBuffer *pHandle, int16 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an uint16 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint16(SbgStreamBuffer *pHandle, uint16 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an int32 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt32(SbgStreamBuffer *pHandle, int32 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an uint32 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint32(SbgStreamBuffer *pHandle, uint32 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an int64 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt64(SbgStreamBuffer *pHandle, int64 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an uint64 into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint64(SbgStreamBuffer *pHandle, uint64 value)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64))
	{
		//
		// Write each byte
		//
		*(pHandle->pCurrentPtr++) = (uint8)(value);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
		*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);

		//
		// The data has been successfully written
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an float into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteFloat(SbgStreamBuffer *pHandle, float value)
{
	FloatNint floatInt;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(float))
	{
		//
		// We use an union to avoid compiler cast
		//
		floatInt.valF = value;

		//
		// Write this float as an uint32
		//
		return sbgStreamBufferWriteUint32(pHandle, floatInt.valU);
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

/*!
 * Write an double into a stream buffer
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteDouble(SbgStreamBuffer *pHandle, double value)
{
	DoubleNint doubleInt;

	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= sizeof(double))
	{
		//
		// We use an union to avoid compiler cast
		//
		doubleInt.valF = value;

		//
		// Write this float as an uint64
		//
		return sbgStreamBufferWriteUint64(pHandle, doubleInt.valU);
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}


/*!
 * Write a buffer to a stream buffer.
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[out]	pBuffer				Buffer to write into the stream buffer.
 * \param[in]	numBytesToRead		Number of bytes to write to the stream buffer.
 * \return							SBG_NO_ERROR if the data has been written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteBuffer(SbgStreamBuffer *pHandle, const void *pBuffer, uint32 numBytesToWrite)
{
	//
	// Test if we can access this item
	//
	if (sbgStreamBufferGetSpace(pHandle) >= numBytesToWrite)
	{
		//
		// Copy from the stream buffer to the output buffer
		//
		memcpy(pHandle->pCurrentPtr, pBuffer, numBytesToWrite);

		//
		// Update the current pointer
		//
		pHandle->pCurrentPtr += numBytesToWrite;

		return SBG_NO_ERROR;
	}
	else
	{
		//
		// We are accessing a data that is outside the stream buffer
		//
		pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		return SBG_BUFFER_OVERFLOW;
	}
}

#endif /* __SBG_STREAM_BUFFER_H__ */
