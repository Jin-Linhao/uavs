#include "commandsInfo.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Info commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the device information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pInfo						A pointer to a structure to hold device information.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGetInfo(SbgEComHandle *pHandle, SbgEComDeviceInfo *pInfo)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointers are valid
	//
	if ((pHandle) && (pInfo))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO);

			//
			// Send the command only since this is a no-payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, NULL, 0);

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
				// Test if we have received a SBG_ECOM_CMD_INFO command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == cmd))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					//
					sbgStreamBufferReadBuffer(&inputStream, pInfo->productCode, SBG_ECOM_INFO_PRODUCT_CODE_LENGTH);
					pInfo->serialNumber		= sbgStreamBufferReadUint32(&inputStream);
					pInfo->calibationRev	= sbgStreamBufferReadUint32(&inputStream);
					pInfo->calibrationYear	= sbgStreamBufferReadUint16(&inputStream);
					pInfo->calibrationMonth = sbgStreamBufferReadUint8(&inputStream);
					pInfo->calibrationDay	= sbgStreamBufferReadUint8(&inputStream);
					pInfo->hardwareRev		= sbgStreamBufferReadUint32(&inputStream);
					pInfo->firmwareRev		= sbgStreamBufferReadUint32(&inputStream);

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