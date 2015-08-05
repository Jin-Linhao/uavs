/*!
 *	\file		sbgSplitBuffer.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		19 November 2013
 *
 *	\brief		Helper methods used to handle a splittable buffer.
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

#ifndef __SBG_SPLIT_BUFFER_H__
#define __SBG_SPLIT_BUFFER_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Structs definitions                                                -//
//----------------------------------------------------------------------//

/*!
 * Defines the ECom transfer states
 */
typedef struct _SbgSplitBuffer
{
	uint8	*pLinkedBuffer;					/*!< Pointer on the buffer that needs to be split */
	uint32	 linkedBufferSize;				/*!< Size of the original buffer */
	uint32	 subBufferSize;					/*!< The size of the sub buffers */
	uint32	 subBufferNbr;					/*!< The number of sub buffers in this split buffer */
} SbgSplitBuffer;

//----------------------------------------------------------------------//
//- Public methods definitions					                       -//
//----------------------------------------------------------------------//

/*!
 * Initialize a split buffer for read operations.
 * \param[in] pSplitBuffer			Pointer to an allocated split buffer instance.
 * \param[in] pBuffer				The buffer that needs to be split (doesn't take the ownership).
 * \param[in] bufferSize			The total size of the buffer in bytes.
 * \param[in] subBufferSize			The size of each sub buffer in bytes.
 * \return							SBG_NO_ERROR if no error occurred.
 */
SBG_INLINE SbgErrorCode sbgSplitBufferInitForRead(SbgSplitBuffer *pSplitBuffer, const void *pBuffer, uint32 bufferSize, uint32 subBufferSize)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test input parameters
	//
	if (pSplitBuffer)
	{
		//
		// Initialize split buffer parameters
		//
		pSplitBuffer->pLinkedBuffer = (uint8*)pBuffer;
		pSplitBuffer->linkedBufferSize = bufferSize;
		pSplitBuffer->subBufferSize = subBufferSize;

		//
		// Compute sub buffer number
		//
		pSplitBuffer->subBufferNbr = (bufferSize + (subBufferSize - 1)) / subBufferSize;
	}
	else
	{
		//
		// Invalid handle
		//
		errorCode = SBG_ERROR;
	}

	return errorCode;
}

/*!
 * Initialize a split buffer for write operations.
 * \param[in] pSplitBuffer			Pointer to an allocated split buffer instance.
 * \param[in] pBuffer				The buffer that needs to be split (doesn't take the ownership).
 * \param[in] bufferSize			The total size of the buffer in bytes.
 * \param[in] subBufferSize			The size of each sub buffer in bytes.
 * \return							SBG_NO_ERROR if no error occurred.
 */
SBG_INLINE SbgErrorCode sbgSplitBufferInitForWrite(SbgSplitBuffer *pSplitBuffer, void *pBuffer, uint32 bufferSize, uint32 subBufferSize)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test input parameters
	//
	if (pSplitBuffer)
	{
		//
		// Initialize split buffer parameters
		//
		pSplitBuffer->pLinkedBuffer = (uint8*)pBuffer;
		pSplitBuffer->linkedBufferSize = bufferSize;
		pSplitBuffer->subBufferSize = subBufferSize;

		//
		// Compute sub buffer number
		//
		pSplitBuffer->subBufferNbr = (bufferSize + (subBufferSize - 1)) / subBufferSize;
	}
	else
	{
		//
		// Invalid handle
		//
		errorCode = SBG_ERROR;
	}

	return errorCode;
}

/*!
 * Returns the number of sub buffers that compose the whole buffer.
 * \param[in] pSplitBuffer			Valid pointer to a Split Buffer instance.
 * \return							The number of sub buffer the buffer has or 0 if there is an error.
 */
SBG_INLINE uint32 sbgSplitBufferGetSubBufferNbr(const SbgSplitBuffer *pSplitBuffer)
{
	//
	// Test input parameters
	//
	if (pSplitBuffer)
	{
		//
		// Return subBufferNbr parameter
		//
		return pSplitBuffer->subBufferNbr;
	}
	else
	{
		//
		// Invalid handle
		//
		return 0;
	}
}

/*!
 * Get one sub buffer given its index.
 * \param[in] pSplitBuffer			Valid pointer to a Split Buffer instance.
 * \param[in] subBufferIdx			Index of the sub buffer required.
 * \return							Pointer to the sub buffer or NULL if the subBuffer index is invalid.
 */
SBG_INLINE void *sbgSplitBufferGetSubBuffer(const SbgSplitBuffer *pSplitBuffer, uint32 subBufferIdx)
{
	//
	// Test input parameters
	//
	if ( (pSplitBuffer) && (subBufferIdx < pSplitBuffer->subBufferNbr))
	{
		//
		// Return pointer to buffer
		//
		return ((uint8*)pSplitBuffer->pLinkedBuffer + pSplitBuffer->subBufferSize*subBufferIdx);
	}
	else
	{
		//
		// Invalid index
		//
		return NULL;
	}
}

/*!
 * Return the offset in bytes of a sub buffer from the start of the buffer.
 * \param[in] pSplitBuffer			Valid pointer to a Split Buffer instance.
 * \param[in] subBufferIdx			Index of the sub buffer required.
 * \return							Offset to the sub buffer or 0 if the subBuffer index is invalid.
 */
SBG_INLINE uint32 sbgSplitBufferGetSubBufferOffset(const SbgSplitBuffer *pSplitBuffer, uint32 subBufferIdx)
{
	//
	// Test input parameters
	//
	if ((pSplitBuffer) && (subBufferIdx < pSplitBuffer->subBufferNbr))
	{
		//
		// Return pointer to buffer
		//
		return (pSplitBuffer->subBufferSize * subBufferIdx);
	}
	else
	{
		//
		// Invalid index
		//
		return 0;
	}
}

/*!
 * Get the size of a sub buffer given its index.
 * \param[in] pSplitBuffer			Valid pointer to a Split Buffer instance.
 * \param[in] subBufferIdx			Index of the sub buffer required.
 * \return							The size of the sub buffer of index subBufferIdx, or 0 if the subBuffer index is invalid.
 */
SBG_INLINE uint32 sbgSplitBufferGetSubBufferSize(const SbgSplitBuffer *pSplitBuffer, uint32 subBufferIdx)
{
	uint32 subBufferSize = 0;

	//
	// Test input parameters
	//
	if ( (pSplitBuffer) && (pSplitBuffer->subBufferNbr > 0) )
	{
		//
		// Test that the sub buffer index is not the last one
		//
		if (subBufferIdx < (pSplitBuffer->subBufferNbr-1))
		{
			//
			// We can just return the sub buffer size because it's not the last sub buffer
			//
			subBufferSize =  pSplitBuffer->subBufferSize;
		}
		else if (subBufferIdx == (pSplitBuffer->subBufferNbr-1) )
		{
			//
			// It's the last sub buffer so return the remaining size
			//
			subBufferSize = pSplitBuffer->linkedBufferSize - (subBufferIdx * pSplitBuffer->subBufferSize);
		}
	}

	//
	// Return computed size
	//
	return subBufferSize;
}

#endif
