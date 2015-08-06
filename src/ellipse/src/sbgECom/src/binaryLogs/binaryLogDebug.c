#include "binaryLogDebug.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the debug message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDebug0Data(const void *pPayload, uint32 payloadSize, SbgLogDebug0Data *pOutputData)
{
	SbgStreamBuffer	inputStream;
	uint32			i;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload and return
	//
	pOutputData->timeStamp = sbgStreamBufferReadUint32(&inputStream);

	//
	// Read each value in the array
	//
	for (i = 0; i < 64; i++)
	{
		pOutputData->data[i] = sbgStreamBufferReadFloat(&inputStream);
	}

	return SBG_NO_ERROR;
}
