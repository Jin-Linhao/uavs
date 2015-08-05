#include "binaryLogMag.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_MAG message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagData(const void *pPayload, uint32 payloadSize, SbgLogMag *pOutputData)
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

	pOutputData->magnetometers[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->magnetometers[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->magnetometers[2] = sbgStreamBufferReadFloat(&inputStream);
				
	pOutputData->accelerometers[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->accelerometers[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->accelerometers[2] = sbgStreamBufferReadFloat(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}

/*!
 *	Parse data for the SBG_ECOM_LOG_MAG_CALIB message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagCalibData(const void *pPayload, uint32 payloadSize, SbgLogMagCalib *pOutputData)
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
	pOutputData->reserved = sbgStreamBufferReadUint16(&inputStream);

	//
	// Read the raw magnetic calibration data buffer
	//
	return sbgStreamBufferReadBuffer(&inputStream, pOutputData->magData, sizeof(pOutputData->magData));
}
