#include "binaryLogShipMotion.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_SHIP_MOTION_# message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseShipMotionData(const void *pPayload, uint32 payloadSize, SbgLogShipMotionData *pOutputData)
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

	//
	// Read the main heave period in seconds
	//
	pOutputData->mainHeavePeriod = sbgStreamBufferReadFloat(&inputStream);

	//
	// Read the surge, sway and heave ship motion
	//
	pOutputData->shipMotion[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->shipMotion[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->shipMotion[2] = sbgStreamBufferReadFloat(&inputStream);
	
	//
	// Read the ship accelerations
	//
	pOutputData->shipAccel[0] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->shipAccel[1] = sbgStreamBufferReadFloat(&inputStream);
	pOutputData->shipAccel[2] = sbgStreamBufferReadFloat(&inputStream);

	//
	// Check if we have other outputs in this frame
	//
	if (payloadSize >= 46)
	{
		//
		// Read new outputs
		//
		pOutputData->shipVel[0] = sbgStreamBufferReadFloat(&inputStream);
		pOutputData->shipVel[1] = sbgStreamBufferReadFloat(&inputStream);
		pOutputData->shipVel[2] = sbgStreamBufferReadFloat(&inputStream);

		pOutputData->status = sbgStreamBufferReadUint16(&inputStream);
	}
	else
	{
		//
		// Those outputs are not available in previous versions
		//
		pOutputData->shipVel[0] = 0.0f;
		pOutputData->shipVel[1] = 0.0f;
		pOutputData->shipVel[2] = 0.0f;

		pOutputData->status = 0;
	}

	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}
