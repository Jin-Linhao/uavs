#include "commandsMag.h"
#include "../misc/sbgStreamBuffer.h"
#include "../misc/transfer.h"

//----------------------------------------------------------------------//
//- Magnetometer commands                                              -//
//----------------------------------------------------------------------//

/*!
 *	Set the error model for the magnetometer.
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgEComCmdId cmd;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{	
		//
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_SET_MODEL);

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
 *	Retrieve magnetometer error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current magnetometer error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo)
{
	SbgEComCmdId cmd;

	//
	// Build frame identifier based on message class and command id
	//
	cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_MODEL_INFO);

	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericGetModelInfo(pHandle, cmd, pModelInfo);
}

/*!
 *	Send a command that set the magnetometers calibration parameters.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	offset						Magnetometers calibration offset vector.
 *	\param[in]	matix						Magnetometers calibration 3x3 matrix.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetCalibData(SbgEComHandle *pHandle, const float offset[3], const float matrix[9])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outputStream;
	uint8				payload[12*sizeof(float)];
	uint32				trial;
	uint32				i;
	SbgEComCmdId		cmd;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Initialize a stream buffer to write the command payload
		//
		errorCode = sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

		//
		// Write the offset vector
		//
		sbgStreamBufferWriteFloat(&outputStream, offset[0]);
		sbgStreamBufferWriteFloat(&outputStream, offset[1]);
		sbgStreamBufferWriteFloat(&outputStream, offset[2]);

		//
		// Write the matrix
		//
		for (i = 0; i < 9; i++)
		{
			sbgStreamBufferWriteFloat(&outputStream, matrix[i]);
		}

		//
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SET_MAG_CALIB);

		//
		// Make sure that the stream buffer has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the command three times
			//
			for (trial = 0; trial < 3; trial++)
			{
				//
				// Send the command
				//
				errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, payload, sbgStreamBufferGetLength(&outputStream));

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
 *	Retrieve the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct to hold rejection configuration of the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetRejection(SbgEComHandle *pHandle, SbgEComMagRejectionConf *pRejectConf)
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
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE);

		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
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
				// Test if we have received a SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE command
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
					pRejectConf->magneticField = (SbgEComRejectionMode)sbgStreamBufferReadUint8(&inputStream);

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
 *	Set the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct holding rejection configuration for the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetRejection(SbgEComHandle *pHandle, const SbgEComMagRejectionConf *pRejectConf)
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
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE);

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
			sbgStreamBufferWriteUint8(&outputStream, (uint8)pRejectConf->magneticField);

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
//- Magnetometer onboard calibration commands	                       -//
//----------------------------------------------------------------------//

/*!
 *	Start the magnetic calibration process.
 *	As soon as this command is sent, the device will start logging magnetic field data internally.
 *	This set of data will be used later by the magnetic calibration algorithms to map the surrounding magnetic field.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	mode						Define which magnetic calibration type to perform. It could be 3D or 2D.
 *	\param[in]	bandwidth					Tell the device that we should have low, medium or high dynamics during the magnetic calibration process.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagStartCalib(SbgEComHandle *pHandle, SbgEComMagCalibMode mode, SbgEComMagCalibBandwidth bandwidth)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outputStream;
	uint8				payload[2];
	uint32				trial;
	SbgEComCmdId		cmd;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Initialize a stream buffer to write the command payload
		//
		errorCode = sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

		//
		// Write the calibration mode and bandwith
		//
		sbgStreamBufferWriteUint8(&outputStream, (uint8)mode);
		sbgStreamBufferWriteUint8(&outputStream, (uint8)bandwidth);

		//
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_START_MAG_CALIB);
		
		//
		// Make sure that the stream buffer has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the command three times
			//
			for (trial = 0; trial < 3; trial++)
			{
				//
				// Send the command
				//
				errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, payload, sbgStreamBufferGetLength(&outputStream));

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
 *	This command computes a magnetic calibration solution based on the magnetic field logged since the last call to the command SBG_ECOM_CMD_START_MAG_CALIB (15).
 *	As soon as the computations are done, the device will answer with quality indicators, status flags and if possible a valid magnetic calibration matrix and offset.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pCalibResults				Pointer on a SbgEComMagCalibResults structure that can hold onboard magnetic calibration results and status.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagComputeCalib(SbgEComHandle *pHandle, SbgEComMagCalibResults *pCalibResults)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint16				receivedCmd;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	SbgEComCmdId		cmd;
	uint32				i;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pCalibResults))
	{
		//
		// Build frame identifier based on message class and command id
		//
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_COMPUTE_MAG_CALIB);

		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
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
				// Try to read the device answer for 5 s because the onboard magnetic computation can take some time
				//
				errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedCmd, receivedBuffer, &receivedSize, sizeof(receivedBuffer), 5000);

				//
				// Test if we have received the correct command
				//
				if ((errorCode == SBG_NO_ERROR) && (receivedCmd == cmd))
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read quality and status parameters
					//
					pCalibResults->quality			= (SbgEComMagCalibQuality)sbgStreamBufferReadUint8(&inputStream);
					pCalibResults->confidence		= (SbgEComMagCalibConfidence)sbgStreamBufferReadUint8(&inputStream);
					pCalibResults->advancedStatus	= sbgStreamBufferReadUint16(&inputStream);

					pCalibResults->beforeMeanError	= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->beforeStdError	= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->beforeMaxError	= sbgStreamBufferReadFloat(&inputStream);

					pCalibResults->afterMeanError	= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->afterStdError	= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->afterMaxError	= sbgStreamBufferReadFloat(&inputStream);

					pCalibResults->meanAccuracy		= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->stdAccuracy		= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->maxAccuracy		= sbgStreamBufferReadFloat(&inputStream);

					pCalibResults->numPoints		= sbgStreamBufferReadUint16(&inputStream);
					pCalibResults->maxNumPoints		= sbgStreamBufferReadUint16(&inputStream);

					//
					// Read the computed hard iron offset vector
					//
					pCalibResults->offset[0]		= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->offset[1]		= sbgStreamBufferReadFloat(&inputStream);
					pCalibResults->offset[2]		= sbgStreamBufferReadFloat(&inputStream);

					//
					// Read the computed soft iron matrix
					//
					for (i = 0; i < 9; i++)
					{
						pCalibResults->matrix[i]	= sbgStreamBufferReadFloat(&inputStream);
					}

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
