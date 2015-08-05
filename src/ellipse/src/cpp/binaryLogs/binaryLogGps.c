#include "binaryLogGps.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_GPS#_VEL message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsVelData(const void *pPayload, uint32 payloadSize, SbgLogGpsVel *pOutputData)
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
	pOutputData->status			= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->timeOfWeek		= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->velocity[0]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->velocity[1]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->velocity[2]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->velocityAcc[0]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->velocityAcc[1]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->velocityAcc[2]	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->course			= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->courseAcc		= sbgStreamBufferReadFloat(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}

/*!
 *	Parse data for the SBG_ECOM_LOG_GPS#_POS message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsPosData(const void *pPayload, uint32 payloadSize, SbgLogGpsPos *pOutputData)
{
	SbgStreamBuffer inputStream;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->status				= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->timeOfWeek			= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->latitude			= sbgStreamBufferReadDouble(&inputStream);
	pOutputData->longitude			= sbgStreamBufferReadDouble(&inputStream);
	pOutputData->altitude			= sbgStreamBufferReadDouble(&inputStream);
	pOutputData->undulation			= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->latitudeAccuracy	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->longitudeAccuracy	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->altitudeAccuracy	= sbgStreamBufferReadFloat(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}

/*!
 *	Parse data for the SBG_ECOM_LOG_GPS#_HDT message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsHdtData(const void *pPayload, uint32 payloadSize, SbgLogGpsHdt *pOutputData)
{
	SbgStreamBuffer inputStream;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->status				= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->timeOfWeek			= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->heading			= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->headingAccuracy	= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->pitch				= sbgStreamBufferReadFloat(&inputStream);
	pOutputData->pitchAccuracy		= sbgStreamBufferReadFloat(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}

/*!
 *	Parse data for the SBG_ECOM_LOG_GPS#_RAW message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsRawData(const void *pPayload, uint32 payloadSize, SbgLogGpsRaw *pOutputData)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;

	//
	// This buffer is different from other because only a variable size raw buffer is stored
	// We use the payload size (read from the low level protocol) to know the buffer size
	// Check that the received buffer can stored in the RAW message log
	//
	if (payloadSize <= SBG_ECOM_GPS_RAW_MAX_BUFFER_SIZE)
	{
		//
		// Copy the buffer
		//
		memcpy(pOutputData->rawBuffer, pPayload, payloadSize);
		pOutputData->bufferSize = payloadSize;
	}
	else
	{
		//
		// Unable to store the received buffer due to buffer overflow
		//
		errorCode = SBG_BUFFER_OVERFLOW;
	}

	return errorCode;
}