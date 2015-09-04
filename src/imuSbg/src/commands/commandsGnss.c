#include "commandsGnss.h"
#include "../misc/sbgStreamBuffer.h"
#include "../misc/transfer.h"

//----------------------------------------------------------------------//
//- GNSS private commands                                              -//
//----------------------------------------------------------------------//

/*!
 *	Set the Error model for a GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the motion profile buffer.
 *	\param[in]	size						Size of the buffer.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size, SbgEComCmd cmdId)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgEComCmdId cmd;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{	
		//
		// Build command
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

		//
		// Call function that handle data transfer
		//
		errorCode = sbgEComTransferSend(pHandle, cmd, pBuffer, size);
	}
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}


/*!
 *	Retrieve GNSS error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo, SbgEComCmd cmdId)
{
	SbgEComCmdId cmd;

	//
	// Build command
	//
	cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericGetModelInfo(pHandle, cmd, pModelInfo);
}

/*!
 *	Retrieve the alignment configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pAlignConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

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
				// Test if we have received a SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT command
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
					pAlignConf->leverArmX = sbgStreamBufferReadFloat(&inputStream);
					pAlignConf->leverArmY = sbgStreamBufferReadFloat(&inputStream);
					pAlignConf->leverArmZ = sbgStreamBufferReadFloat(&inputStream);
					pAlignConf->pitchOffset = sbgStreamBufferReadFloat(&inputStream);
					pAlignConf->yawOffset = sbgStreamBufferReadFloat(&inputStream);
					pAlignConf->antennaDistance = sbgStreamBufferReadFloat(&inputStream);

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
 *	Set the alignment configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pAlignConf))
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
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->leverArmX);
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->leverArmY);
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->leverArmZ);
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->pitchOffset);
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->yawOffset);
			sbgStreamBufferWriteFloat(&outputStream, pAlignConf->antennaDistance);

			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

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
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pRejectConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

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
				// Test if we have received a SBG_ECOM_CMD_GNSS_1_REJECT_MODES command
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
					pRejectConf->position = (SbgEComRejectionMode)sbgStreamBufferReadUint8(&inputStream);
					pRejectConf->velocity = (SbgEComRejectionMode)sbgStreamBufferReadUint8(&inputStream);
					pRejectConf->course = (SbgEComRejectionMode)sbgStreamBufferReadUint8(&inputStream);
					pRejectConf->hdt = (SbgEComRejectionMode)sbgStreamBufferReadUint8(&inputStream);

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
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;
	SbgEComCmdId		cmd;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pRejectConf))
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
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pRejectConf->position);
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pRejectConf->velocity);
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pRejectConf->course);
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pRejectConf->hdt);

			//
			// Build command
			//
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, cmdId);

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
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}


//----------------------------------------------------------------------//
//- GNSS public commands		                                       -//
//----------------------------------------------------------------------//

/*!
 *	Set the error model for a GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size)
{
	return sbgEComCmdGnssSetModel(pHandle, pBuffer, size, SBG_ECOM_CMD_GNSS_1_SET_MODEL);
}

/*!
 *	Retrieve GNSS 1 error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo)
{
	return sbgEComCmdGnssGetModelInfo(pHandle, pModelInfo, SBG_ECOM_CMD_GNSS_1_MODEL_INFO);
}

/*!
 *	Retrieve the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf)
{
	return sbgEComCmdGnssGetLeverArmAlignment(pHandle, pAlignConf, SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT);
}

/*!
 *	Set the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf)
{
	return sbgEComCmdGnssSetLeverArmAlignment(pHandle, pAlignConf, SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT);
}

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf)
{
	return sbgEComCmdGnssGetRejection(pHandle, pRejectConf, SBG_ECOM_CMD_GNSS_1_REJECT_MODES);
}

/*!
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf)
{
	return sbgEComCmdGnssSetRejection(pHandle, pRejectConf, SBG_ECOM_CMD_GNSS_1_REJECT_MODES);
}
