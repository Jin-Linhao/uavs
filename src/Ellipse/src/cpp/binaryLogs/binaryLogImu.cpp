#include "binaryLogImu.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_IMU_DATA message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseImuData(const void *pPayload, uint32 payloadSize, SbgLogImuData *pOutputData)
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
				
	pOutputData->accelerometers[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->accelerometers[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->accelerometers[2] = sbgStreamBufferReadFloat(&inputStream);

	pOutputData->gyroscopes[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->gyroscopes[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->gyroscopes[2] = sbgStreamBufferReadFloat(&inputStream);

	pOutputData->temperature = sbgStreamBufferReadFloat(&inputStream);

	pOutputData->deltaVelocity[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->deltaVelocity[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->deltaVelocity[2] = sbgStreamBufferReadFloat(&inputStream);
				
	pOutputData->deltaAngle[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->deltaAngle[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->deltaAngle[2] = sbgStreamBufferReadFloat(&inputStream);

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}
