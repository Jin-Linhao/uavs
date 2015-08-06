#include "binaryLogStatus.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_STATUS message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseStatusData(const void *pPayload, uint32 payloadSize, SbgLogStatusData *pOutputData)
{
	SbgStreamBuffer inputStream;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->generalStatus	= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->reserved1		= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->comStatus		= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->aidingStatus	= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->reserved2		= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->reserved3		= sbgStreamBufferReadUint16(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}
