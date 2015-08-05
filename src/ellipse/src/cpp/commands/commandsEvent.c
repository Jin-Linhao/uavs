#include "commandsEvent.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Event commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of a Sync In.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncInId					The id of the sync whose configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComSyncInConf to contain the current configuration of the sync in.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncInGetConf(SbgEComHandle *pHandle, SbgEComSyncInId syncInId, SbgEComSyncInConf *pConf)
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
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF);

			//
			// Send the command with syncInId as a 1-byte payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, &syncInId, 1);

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
				// Test if we have received a SBG_ECOM_CMD_SYNC_IN_CONF command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == cmd))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned the id of the sync, then the sensitivity and the delay at last.
					//
					syncInId = (SbgEComSyncInId)sbgStreamBufferReadUint8(&inputStream);					
					pConf->sensitivity = (SbgEComSyncInSensitivity)sbgStreamBufferReadUint8(&inputStream);
					pConf->delay = sbgStreamBufferReadInt32(&inputStream);

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

/*!
 *	Set the configuration of a Sync In.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncInId					The id of the sync whose configuration is to be set.
 *	\param[in]	pConf						Pointer to a SbgEComSyncInConf that contains the new configuration for the sync in.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncInSetConf(SbgEComHandle *pHandle, SbgEComSyncInId syncInId, const SbgEComSyncInConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8(&outputStream, (uint8)syncInId);			
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pConf->sensitivity);
			sbgStreamBufferWriteInt32(&outputStream, pConf->delay);

			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, cmd, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
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
		// Null pointer.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the configuration of a Sync Out.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncOutId					The id of the sync whose configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComSyncOutConf to contain the current configuration of the sync out.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncOutGetConf(SbgEComHandle *pHandle, SbgEComSyncOutId syncOutId, SbgEComSyncOutConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	uint8				reserved;
	SbgStreamBuffer		inputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointers are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF);

			//
			// Send the command with syncOutId as a 1-byte payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, &syncOutId, 1);

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
				// Test if we have received a SBG_ECOM_CMD_SYNC_OUT_CONF command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == cmd))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned the id of the sync, then a reserved field, the output function, polarity and the duration at last.
					//
					syncOutId = (SbgEComSyncOutId)sbgStreamBufferReadUint8(&inputStream);
					reserved = sbgStreamBufferReadUint8(&inputStream);
					pConf->outputFunction = (SbgEComSyncOutFunction)sbgStreamBufferReadUint16(&inputStream);
					pConf->polarity = (SbgEComSyncOutPolarity)sbgStreamBufferReadUint8(&inputStream);
					pConf->duration = sbgStreamBufferReadUint32(&inputStream);

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

/*!
 *	Set the configuration of a Sync Out.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncOutId					The id of the sync whose configuration is to be set.
 *	\param[in]	pConf						Pointer to a SbgEComSyncOutConf that contains the new configuration for the sync Out.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncOutSetConf(SbgEComHandle *pHandle, SbgEComSyncOutId syncOutId, const SbgEComSyncOutConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8(&outputStream, (uint8)syncOutId);
			sbgStreamBufferWriteUint8(&outputStream, 0);
			sbgStreamBufferWriteUint16(&outputStream, (uint16)pConf->outputFunction);
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pConf->polarity);
			sbgStreamBufferWriteUint32(&outputStream, pConf->duration);

			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, cmd, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
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

/*!
 *	Retrieve the configuration of the virtual odometer.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pDistance					The distance between two pulses.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdVirtualOdometerGetConf(SbgEComHandle *pHandle, float *pDistance)
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
	if ((pHandle) && (pDistance))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_VIRTUAL_ODOMETER_CONF);

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
				// Test if we have received a SBG_ECOM_CMD_VIRTUAL_ODOMETER_CONF command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == cmd))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// The only parameter returned is the distance between two pulses.
					//
					*pDistance = (SbgEComSyncOutId)sbgStreamBufferReadFloat(&inputStream);

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

/*!
 *	Set the configuration of the virtual odometer.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	distance					The distance between two pulses.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdVirtualOdometerSetConf(SbgEComHandle *pHandle, float distance)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if (pHandle)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_VIRTUAL_ODOMETER_CONF);

			//
			// Send the payload over ECom as a 4-byte payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, &distance, sizeof(float));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, cmd, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
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