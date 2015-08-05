#include "binaryLogUtc.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_UTC_DATA message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUtcData(const void *pPayload, uint32 payloadSize, SbgLogUtcData *pOutputData)
{
	SbgStreamBuffer inputStream;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp = sbgStreamBufferReadUint32(&inputStream);
	pOutputData->status = sbgStreamBufferReadUint16(&inputStream);
	pOutputData->year = sbgStreamBufferReadUint16(&inputStream);
	pOutputData->month = sbgStreamBufferReadInt8(&inputStream);
	pOutputData->day = sbgStreamBufferReadInt8(&inputStream);
	pOutputData->hour = sbgStreamBufferReadInt8(&inputStream);
	pOutputData->minute = sbgStreamBufferReadInt8(&inputStream);
	pOutputData->second = sbgStreamBufferReadInt8(&inputStream);
	pOutputData->nanoSecond = sbgStreamBufferReadInt32(&inputStream);
	pOutputData->gpsTimeOfWeek = sbgStreamBufferReadUint32(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}
