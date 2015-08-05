#include "sbgECom.h"
#include "sbgEComVersion.h"
#include "time/sbgTime.h"
#include <string.h>
#include <stdio.h>

//----------------------------------------------------------------------//
//- Private methods declarations                                       -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Public methods declarations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the protocol system used to communicate with the product and return the created handle.
 *	\param[out]	pHandle							Pointer used to store the allocated and initialized sbgECom handle.
 *	\param[in]	pInterface						Interface to use for read/write operations.
 *	\return										SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	
	//
	// Check input parameters
	//
	if ( (pInterface) && (pHandle) )
	{
		//
		// Initialize the sbgECom handle
		//
		pHandle->pReceiveCallback = NULL;
		pHandle->pUserArg = NULL;

		//
		// Initialize the protocol 
		//
		errorCode = sbgEComProtocolInit(&pHandle->protocolHandle, pInterface);
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pHandle							A valid sbgECom handle to close.
 *	\return										SBG_NO_ERROR if we have closed and released the sbgECom system.
 */
SbgErrorCode sbgEComClose(SbgEComHandle *pHandle)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Close the protocol
		//
		errorCode = sbgEComProtocolClose(&pHandle->protocolHandle);
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Handle incoming logs.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming logs parsing.
 */
SbgErrorCode sbgEComHandle(SbgEComHandle *pHandle)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgBinaryLogData	logData;
	uint16				receivedCmd;
	uint32				payloadSize;
	uint8				payloadData[SBG_ECOM_MAX_PAYLOAD_SIZE];
	char				errorMsg[4096];
	
	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Try to read all received frames
		//
		do
		{
			//
			// Read a received frame
			//
			errorCode = sbgEComProtocolReceive(&pHandle->protocolHandle, &receivedCmd, payloadData, &payloadSize, sizeof(payloadData));

			//
			// Test if we have received a valid frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Test if the received frame is a binary log
				//
				if (sbgEComBinaryLogIsCmdValid(receivedCmd))
				{
					//
					// The received frame is a binary log one
					//
					errorCode = sbgEComBinaryLogParse(receivedCmd, payloadData, payloadSize, &logData);

					//
					// Test if the incoming log has been parsed successfully
					//
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// Test if we have a valid callback to handle received logs
						//
						if (pHandle->pReceiveCallback)
						{
							//
							// Call the binary log callback
							//
							pHandle->pReceiveCallback(pHandle, (SbgEComCmdId)receivedCmd, &logData, pHandle->pUserArg);
						}
					}
					else
					{
						//
						// Call the on error callback
						//
					}
				}
				else
				{
					//
					// We have received a command, it shouldn't happen
					//
				}
			}
			else if (errorCode != SBG_NOT_READY)
			{
				//
				// We have received an invalid frame
				//
				sbgEComErrorToString(errorCode, errorMsg);
				fprintf(stderr, "Error[%u]: %s\n", receivedCmd, errorMsg);
			}
		} while (errorCode != SBG_NOT_READY);
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Wait until any command that is not a output log is recevied during a specific time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	pCommand				Pointer used to hold the received command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveAnyCmd(SbgEComHandle *pHandle, uint16 *pCommand, void *pData, uint32 *pSize, uint32 maxSize, uint32 timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgBinaryLogData	logData;
	uint16				receivedCmd;
	uint32				payloadSize;
	uint8				payloadData[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32				lastValidTime;
	
	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Compute the last valid time according to the time out
		//
		lastValidTime = sbgGetTime() + timeOut;

		//
		// Try to receive the desired frame within the specified time out
		//
		do
		{
			//
			// Read a received frame
			//
			errorCode = sbgEComProtocolReceive(&pHandle->protocolHandle, &receivedCmd, payloadData, &payloadSize, sizeof(payloadData));

			//
			// Test if we have received a valid frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Test if the received frame is a binary log
				//
				if (sbgEComBinaryLogIsCmdValid(receivedCmd))
				{
					//
					// The received frame is a binary log one
					//
					errorCode = sbgEComBinaryLogParse(receivedCmd, payloadData, payloadSize, &logData);

					//
					// Test if the incoming log has been parsed successfully
					//
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// Test if we have a valid callback to handle received logs
						//
						if (pHandle->pReceiveCallback)
						{
							//
							// Call the binary log callback
							//
							pHandle->pReceiveCallback(pHandle, (SbgEComCmdId)receivedCmd, &logData, pHandle->pUserArg);
						}
					}
					else
					{
						//
						// Call the on error callback
						//
					}
				}
				else
				{
					//
					// Return the received command
					//
					if (pCommand)
					{
						*pCommand = receivedCmd;
					}

					//
					// We have received a command so return the payload size
					//
					if (pSize)
					{
						*pSize = payloadSize;
					}

					//
					// Test if we have a payload to return
					//
					if (payloadSize > 0)
					{
						//
						// Make sure that the payload can be stored and fit in the destination buffer
						//
						if ( (pData) && (payloadSize <= maxSize) )
						{
							//
							// Copy the payload
							//
							memcpy(pData, payloadData, payloadSize);
						}
						else
						{
							//
							// We have a buffer overflow
							//
							return SBG_BUFFER_OVERFLOW;
						}
					}

					//
					// We have received the frame we are looking for so return
					//
					return SBG_NO_ERROR;
				}
			}
			else if (errorCode == SBG_NOT_READY)
			{
				//
				// No more data are present in the reception buffer so release some CPU before the next try
				//
				sbgSleep(1);
			}
		} while (lastValidTime >= sbgGetTime());

		//
		// The time out has expired so return time out error
		//
		errorCode = SBG_TIME_OUT;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Wait for a specific command to be received given a time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[in]	command					The command we would like to receive.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveCmd(SbgEComHandle *pHandle, uint16 command, void *pData, uint32 *pSize, uint32 maxSize, uint32 timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgBinaryLogData	logData;
	uint16				receivedCmd;
	uint32				payloadSize;
	uint8				payloadData[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32				lastValidTime;
	
	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Compute the last valid time according to the time out
		//
		lastValidTime = sbgGetTime() + timeOut;

		//
		// Try to receive the desired frame within the specified time out
		//
		do
		{
			//
			// Read a received frame
			//
			errorCode = sbgEComProtocolReceive(&pHandle->protocolHandle, &receivedCmd, payloadData, &payloadSize, sizeof(payloadData));

			//
			// Test if we have received a valid frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Test if the received frame is a binary log
				//
				if (sbgEComBinaryLogIsCmdValid(receivedCmd))
				{
					//
					// The received frame is a binary log one
					//
					errorCode = sbgEComBinaryLogParse(receivedCmd, payloadData, payloadSize, &logData);

					//
					// Test if the incoming log has been parsed successfully
					//
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// Test if we have a valid callback to handle received logs
						//
						if (pHandle->pReceiveCallback)
						{
							//
							// Call the binary log callback
							//
							pHandle->pReceiveCallback(pHandle, (SbgEComCmdId)receivedCmd, &logData, pHandle->pUserArg);
						}
					}
					else
					{
						//
						// Call the on error callback
						//
					}
				}
				else if (receivedCmd == command)
				{
					//
					// We have received the command we are looking for so return the payload size
					//
					if (pSize)
					{
						*pSize = payloadSize;
					}

					//
					// Test if we have a payload to return
					//
					if (payloadSize > 0)
					{
						//
						// Make sure that the payload can be stored and fit in the destination buffer
						//
						if ( (pData) && (payloadSize <= maxSize) )
						{
							//
							// Copy the payload
							//
							memcpy(pData, payloadData, payloadSize);
						}
						else
						{
							//
							// We have a buffer overflow
							//
							return SBG_BUFFER_OVERFLOW;
						}
					}

					//
					// We have received the frame we are looking for so return
					//
					return SBG_NO_ERROR;
				}
			}
			else if (errorCode == SBG_NOT_READY)
			{
				//
				// No more data are present in the reception buffer so release some CPU before the next try
				//
				sbgSleep(1);
			}
		} while (lastValidTime >= sbgGetTime());

		//
		// The time out has expired so return time out error
		//
		errorCode = SBG_TIME_OUT;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Define the callback that should be called each time a new binary log is received.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\param[in]	pReceiveCallback				Pointer on the callback to call when a new log is received.
 *	\param[in]	pUserArg						Optional user argument that will be passed to the callback method.
 *	\return										SBG_NO_ERROR if the callback and user argument have been defined successfully.
 */
SbgErrorCode sbgEComSetReceiveCallback(SbgEComHandle *pHandle, SbgEComReceiveFunc pReceiveCallback, void *pUserArg)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Define the callback and the user argument
		//
		pHandle->pReceiveCallback = pReceiveCallback;
		pHandle->pUserArg = pUserArg;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Returns an integer representing the version of the sbgCom library.
 *	\return										An integer representing the version of the sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgEComGetVersion(void)
{
	return SBG_E_COM_VERSION_U;
}

/*!
 *	Retreive the sbgCom library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCom library version.
 */
const char *sbgEComGetVersionAsString(void)
{
	return SBG_E_COM_VERSION;
}

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgEComErrorToString(SbgErrorCode errorCode, char errorMsg[256])
{
	if (errorMsg)
	{
		//
		// For each error code, copy the error msg
		//
		switch (errorCode)
		{
		case SBG_NO_ERROR:
			strcpy(errorMsg, "SBG_NO_ERROR: No error."); 
			break;
		case SBG_ERROR:
			strcpy(errorMsg, "SBG_ERROR: Generic error."); 
			break;
		case SBG_NULL_POINTER:
			strcpy(errorMsg, "SBG_NULL_POINTER: A pointer is null."); 
			break;
		case SBG_INVALID_CRC:
			strcpy(errorMsg, "SBG_INVALID_CRC: The received frame has an invalid CRC.");
			break;
		case SBG_INVALID_FRAME:
			strcpy(errorMsg, "SBG_INVALID_FRAME: The received frame is invalid.");
			break;
		case SBG_TIME_OUT:
			strcpy(errorMsg, "SBG_TIME_OUT: We have a time out during frame reception.");
			break;
		case SBG_WRITE_ERROR:
			strcpy(errorMsg, "SBG_WRITE_ERROR: All bytes hasn't been written.");
			break;
		case SBG_READ_ERROR:
			strcpy(errorMsg, "SBG_READ_ERROR: All bytes hasn't been read.");
			break;
		case SBG_BUFFER_OVERFLOW:
			strcpy(errorMsg, "SBG_BUFFER_OVERFLOW: A buffer is too small to contain so much data.");
			break;
		case SBG_INVALID_PARAMETER:
			strcpy(errorMsg, "SBG_INVALID_PARAMETER: An invalid parameter has been founded.");
			break;
		case SBG_NOT_READY:
			strcpy(errorMsg, "SBG_NOT_READY: A device isn't ready (Rx isn't ready for example).");
			break;
		case SBG_MALLOC_FAILED:
			strcpy(errorMsg, "SBG_MALLOC_FAILED: Failed to allocate a buffer.");
			break;
		case SGB_CALIB_MAG_NOT_ENOUGH_POINTS:
			strcpy(errorMsg, "SGB_CALIB_MAG_NOT_ENOUGH_POINTS: Not enough points were available to perform magnetometers calibration.");
			break;
		case SBG_CALIB_MAG_INVALID_TAKE:
			strcpy(errorMsg, "SBG_CALIB_MAG_INVALID_TAKE: The calibration procedure could not be properly executed due to insufficient precision.");
			break;
		case SBG_CALIB_MAG_SATURATION:
			strcpy(errorMsg, "SBG_CALIB_MAG_SATURATION: Saturation were detected when attempt to calibrate magnetos.");
			break;
		case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE:
			strcpy(errorMsg, "SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE: 2D calibration procedure could not be performed.");
			break;
		case SBG_DEVICE_NOT_FOUND:
			strcpy(errorMsg, "SBG_DEVICE_NOT_FOUND: A device couldn't be founded or opened.");
			break;
		case SBG_OPERATION_CANCELLED:
			strcpy(errorMsg, "SBG_OPERATION_CANCELLED: An operation has been cancelled by a user.");
			break;
		case SBG_NOT_CONTINUOUS_FRAME:
			strcpy(errorMsg, "SBG_NOT_CONTINUOUS_FRAME: We have received a frame that isn't a continuous one.");
			break;
		case SBG_INCOMPATIBLE_HARDWARE:
			strcpy(errorMsg, "SBG_INCOMPATIBLE_HARDWARE: Hence valid, the configuration cannot be executed because of incompatible hardware.");
			break;
		default:
			sprintf(errorMsg, "Undefined error code: %u", errorCode);
			break;
		}
	}
}
