#include "commandsCommon.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Generic command definitions                                        -//
//----------------------------------------------------------------------//

/*!
 *	Generic function to retrieve error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						Original command.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGenericGetModelInfo(SbgEComHandle *pHandle, uint16 command, SbgEComModelInfo *pModelInfo)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointers are valid
	//
	if ((pHandle) && (pModelInfo))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Send the command only since this is a no payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, command, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedCmd, receivedBuffer, &receivedSize, sizeof(receivedBuffer), SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a the specified command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == command))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					//
					pModelInfo->id = sbgStreamBufferReadUint32(&inputStream);
					pModelInfo->revision = sbgStreamBufferReadUint32(&inputStream);

					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Null pointer
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Ack commands				                                       -//
//----------------------------------------------------------------------//

/*!
 *	Wait for an ACK for a specified amount of time.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						The command we would like the ACK for.
 *	\param[in]	timeOut						Time out in ms during which we can receive the ACK.
 *	\return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint16 command, uint32 timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint8				payload[2*sizeof(uint16)];
	SbgStreamBuffer		inputStream;
	uint32				receivedSize;
	SbgEComCmdId		cmd;

	//
	// Build command
	//
	cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ACK);

	//
	// Try to receive the ACK
	//
	errorCode = sbgEComReceiveCmd(pHandle, cmd, payload, &receivedSize, sizeof(payload), timeOut);

	//
	// Test if an ACK frame has been received
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Validate the received ACK frame
		//
		if (receivedSize == 2*sizeof(uint16))
		{
			//
			// Initialize a stream buffer to parse the received payload
			//
			sbgStreamBufferInitForRead(&inputStream, payload, sizeof(payload));

			//
			// The ACK frame contains two uint16, the command id of the acknoledge frame and the return error code
			// We make sure that the ACK is for the correct command
			//
			if (sbgStreamBufferReadUint16(&inputStream) == command)
			{
				//
				// Parse the error code and return it
				//
				errorCode = (SbgErrorCode)sbgStreamBufferReadUint16(&inputStream);
			}
			else
			{
				//
				// We have received an ACK but not for this frame!
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
		else
		{
			//
			// The ACK is invalid
			//
			errorCode = SBG_INVALID_FRAME;
		}
	}	

	return errorCode;
}