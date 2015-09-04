#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../commands/commands.h"
#include "sbgStreamBuffer.h"
#include "sbgSplitBuffer.h"
#include "transfer.h"


//----------------------------------------------------------------------//
//- Internal transfer method definitions			                   -//
//----------------------------------------------------------------------//

/*!
 * Initiates an upload transfer sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] command					Original protocol command asking for transfer.
 * \param[in] size						Total size of the upload.
 * \return								SBG_NO_ERROR when the transfer was initiated successfully.
 */
SbgErrorCode sbgEComTransferSendInit(SbgEComHandle *pHandle, uint16 protocolCommand, uint32 size)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		streamBuffer;
	uint8				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32				i;

	//
	// Test input parameter
	//
	if (pHandle)
	{
		//
		// Initialize stream buffer that will contain payload
		//
		sbgStreamBufferInitForWrite(&streamBuffer, outputBuffer, sizeof(outputBuffer));

		//
		// Build transfer payload (a ECOM_TRANSFER_START command and the total size of the upload)
		//
		sbgStreamBufferWriteUint16(&streamBuffer, ECOM_TRANSFER_START);
		sbgStreamBufferWriteUint32(&streamBuffer, size);

		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send transfer payload encapsulated in ECom protocol
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&streamBuffer), sbgStreamBufferGetLength(&streamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// If the device accepts the transfer, it returns an ack, wait for the answer.
				//
				errorCode = sbgEComWaitForAck(pHandle, protocolCommand, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if the response is positive from device
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Ack received, no need for other trial.
					//
					break;
				}
			}
		}
	}
	else
	{
		//
		// Handle pointer is null
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Send one packet of data on a initiated upload transfer.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] protocolCommand			Original protocol command asking for upload.
 * \param[in] pBuffer					Pointer to the buffer containing the data to send.
 * \param[in] offset					The offset from the start of the transfer.
 * \param[in] packetSize				The size of this packet.
 * \return								SBG_NO_ERROR if the packet was sent and acknowledged by the device.
 */
SbgErrorCode sbgEComTransferSendData(SbgEComHandle *pHandle, uint16 protocolCommand, const void *pBuffer, uint32 offset, uint32 packetSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		streamBuffer;
	uint8				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32				i;
	
	//
	// Test input parameters
	//
	if ((pHandle) && (pBuffer) && (packetSize))
	{
		//
		// Initialize stream buffer for output
		//
		sbgStreamBufferInitForWrite(&streamBuffer, outputBuffer, sizeof(outputBuffer));

		//
		// Build payload: a ECOM_TRANSFER_DATA command, the offset from the start of the transfer, and the data
		//
		sbgStreamBufferWriteUint16(&streamBuffer, ECOM_TRANSFER_DATA);
		sbgStreamBufferWriteUint32(&streamBuffer, offset);
		sbgStreamBufferWriteBuffer(&streamBuffer, pBuffer, packetSize);

		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send transfer payload encapsulated in a ECom protocol frame
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&streamBuffer), sbgStreamBufferGetLength(&streamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// If the device receives the frame successfully received, it responds with an ACK, wait for the answer
				//
				errorCode = sbgEComWaitForAck(pHandle, protocolCommand, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if the response is positive from device
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Ack received, no need for other trial
					//
					break;
				}
			}
		}
	}
	else
	{
		//
		// Invalid input parameters
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Ends ongoing upload transfer sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] command					Original protocol command asking for transfer.
 * \return								SBG_NO_ERROR when the transfer ended successfully.
 */
SbgErrorCode sbgEComTransferSendEnd(SbgEComHandle *pHandle, uint16 protocolCommand)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	uint8				outputBuffer[sizeof(uint16)];
	uint32				i;

	//
	// Test input parameter
	//
	if (pHandle)
	{
		//
		// Initialize stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

		//
		// Build payload, only a ECOM_TRANSFER_END cmd
		//
		sbgStreamBufferWriteUint16(&outStreamBuffer, ECOM_TRANSFER_END);
		
		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send upload end payload encapsulated in a ECom protocol frame
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// If the device finishes the sequence successfully, it responds with an ACK, wait for answer
				//
				errorCode = sbgEComWaitForAck(pHandle, protocolCommand, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if the response is positive from device
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// ACK received, no need for other trial
					//
					break;
				}
			}
		}
	}
	else
	{
		//
		// Handle pointer is null
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Initiates a download sequences with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] command					Original protocol command asking for transfer.
 * \param[out] pSize					Size of the transfer initiated, returned from the device.
 * \return								SBG_NO_ERROR when the transfer initiated successfully.
 */
SbgErrorCode sbgEComTransferReceiveInit(SbgEComHandle *pHandle, uint16 protocolCommand, uint32 *pSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	SbgStreamBuffer		inStreamBuffer;
	uint8				inputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint8				outputBuffer[sizeof(uint16)];
	uint16				transferCmd;
	uint16				receivedCmd;
	uint32				inputSize;
	uint32				transferSize;
	uint32				i;

	//
	// Test input parameter
	//
	if (pHandle)
	{
		//
		// Initialize stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

		//
		// Build payload, only a ECOM_TRANSFER_START cmd
		//
		sbgStreamBufferWriteUint16(&outStreamBuffer, ECOM_TRANSFER_START);

		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send transfer payload encapsulated in an ECom protocol frame
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Wait for reponse, the device should respond with a ECOM_TRANSFER_START command and the transfer size
				// If it can not initiate the transfer, it will respond with a NACK
				//
				errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedCmd, inputBuffer, &inputSize, SBG_ECOM_MAX_PAYLOAD_SIZE, SBG_ECOM_DEFAULT_CMD_TIME_OUT);
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Test if the command received is the one expected
					// 
					if (receivedCmd == protocolCommand)
					{
						//
						// Init stream buffer on received payload to process it
						//
						sbgStreamBufferInitForRead(&inStreamBuffer, inputBuffer, inputSize);

						//
						// Retrieve parameters, the first one is the transfer command
						// The second one is the total transfer size
						//
						transferCmd = sbgStreamBufferReadUint16(&inStreamBuffer);
						transferSize = sbgStreamBufferReadUint32(&inStreamBuffer);

						//
						// The device should have answered with ECOM_TRANSFER_START transfer command
						//
						if (transferCmd == ECOM_TRANSFER_START)
						{
							//
							// Update output variable with the transfer size
							//
							*pSize = transferSize;

							//
							// No need for other trials, exit loop/
							//
							break;
						}
						else
						{
							//
							// Invalid transfer command response
							//
							errorCode = SBG_ERROR;
						}
					}
					else
					{
						//
						// This is not the command expected
						// 
						errorCode = SBG_ERROR;
					}
				}
			}
		}
	}
	else
	{
		//
		// Protocol pointer is null
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Receive one packet of data on a initiated download transfer.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] protocolCommand			Original command asking for download.
 * \param[in] pBuffer					Pointer to the buffer where to write the packet.
 * \param[in] offset					The offset from the start of the buffer.
 * \param[in] packetSize				The size of the data asked to the device.
 * \return								SBG_NO_ERROR if the packet was successfully received.
 */
SbgErrorCode sbgEComTransferReceiveData(SbgEComHandle *pHandle, uint16 protocolCommand, void *pBuffer, uint32 offset, uint32 packetSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	SbgStreamBuffer		inStreamBuffer;
	uint8				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint8				inputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint16				transferCmd;
	uint16				receivedCmd;
	uint32				rcvdOffset;
	uint32				inputSize;
	uint32				i;

	//
	// Test input parameters
	//
	if ((pHandle) && (pBuffer) && (packetSize))
	{
		//
		// Initialize stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outputBuffer));

		//
		// Build payload: an ECOM_TRANSFER_DATA transfer command, the offset from the start of the transfer, the size of the packet the device must send
		//
		sbgStreamBufferWriteUint16(&outStreamBuffer, ECOM_TRANSFER_DATA);
		sbgStreamBufferWriteUint32(&outStreamBuffer, offset);
		sbgStreamBufferWriteUint32(&outStreamBuffer, packetSize);

		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send transfer payload encapsulated in an ECom protocol frame
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Wait for reponse, the device should respond with a ECOM_TRANSFER_DATA, the offset from the start of the transfer and the data payload
				// If it can not provide the data, it will respond with a NACK
				//
				errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedCmd, inputBuffer, &inputSize, SBG_ECOM_MAX_PAYLOAD_SIZE, SBG_ECOM_DEFAULT_CMD_TIME_OUT);
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Test if this is the protocol command expected
					//
					if (protocolCommand == receivedCmd)
					{
						//
						// Initialize stream buffer for read on input buffer
						//
						sbgStreamBufferInitForRead(&inStreamBuffer, inputBuffer, inputSize);

						//
						// Read response fields, first is the transfer command, second is the offset
						//
						transferCmd = sbgStreamBufferReadUint16(&inStreamBuffer);
						rcvdOffset = sbgStreamBufferReadUint32(&inStreamBuffer);

						//
						// Test that it's a ECOM_TRANSFER_DATA command
						// The data is at the offset asked
						// And the size corresponds
						//
						if ( (transferCmd == ECOM_TRANSFER_DATA) && (offset == rcvdOffset) && (packetSize == (inputSize - (sizeof(uint16) + sizeof(uint32)))) )
						{
							//
							// Read then all the buffer
							//
							sbgStreamBufferReadBuffer(&inStreamBuffer, pBuffer, inputSize - (sizeof(uint16) + sizeof(uint32)));

							//
							// No need for other trials, exit loop
							//
							break;
						}
					}
					else
					{
						//
						// Not the command expected
						//
						errorCode = SBG_ERROR;
					}
				}
			}
		}
	}
	else
	{
		//
		// Invalid input parameters
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Function that ends a download sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] command					Original protocol command asking for transfer.
 * \return								SBG_NO_ERROR when the transfer ended successfully.
 */
SbgErrorCode sbgEComTransferReceiveEnd(SbgEComHandle *pHandle, uint16 protocolCommand)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	uint8				outputBuffer[sizeof(uint16)];
	uint32				i;

	//
	// Test input parameter
	//
	if (pHandle)
	{
		//
		// Initialize stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

		//
		// Build payload, only a ECOM_TRANSFER_END cmd
		//
		sbgStreamBufferWriteUint16(&outStreamBuffer, ECOM_TRANSFER_END);
		
		//
		// Send command (multiple times in case of failures)
		//
		for (i = 0; i < 3; i++)
		{
			//
			// Send upload end payload encapsulated in a ECom protocol frame
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, protocolCommand, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// If the device is able to finish transfer sequence, it responds with an ACK
				//
				errorCode = sbgEComWaitForAck(pHandle, protocolCommand, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if the response is positive from device
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// No need for other trial, exit loop
					//
					break;
				}
			}
		}
	}
	else
	{
		//
		// Handle pointer is null
		// 
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Public transfer method definitions			                       -//
//----------------------------------------------------------------------//

/*!
 * Specific method to handle a large send into multiple frames.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	command					Original command asking for upload.
 * \param[in]	pBuffer					Pointer to the buffer containing the data to send.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful upload.
 */
SbgErrorCode sbgEComTransferSend(SbgEComHandle *pHandle, uint16 command, const void *pBuffer, uint32 size)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgSplitBuffer	splitBuffer;
	uint32			i;

	//
	// Test input parameters
	//
	if ( (pHandle) && (pBuffer) && (size) )
	{
		//
		// Initiate data transfer
		//
		errorCode = sbgEComTransferSendInit(pHandle, command, size);
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Initialize split buffer that will help with splitting up provided buffer
			//
			sbgSplitBufferInitForRead(&splitBuffer, pBuffer, size, SBG_ECOM_PACKET_SIZE);

			//
			// Transfer sub buffer one by one
			//
			for (i = 0; i < sbgSplitBufferGetSubBufferNbr(&splitBuffer); i++)
			{
				//
				// Send a sub buffer
				//
				errorCode = sbgEComTransferSendData(pHandle, command, sbgSplitBufferGetSubBuffer(&splitBuffer, i), sbgSplitBufferGetSubBufferOffset(&splitBuffer, i), sbgSplitBufferGetSubBufferSize(&splitBuffer, i));
				if (errorCode != SBG_NO_ERROR)
				{
					//
					// Unable to send a sub buffer, abort send operation.
					//
					break;
				}
			}

			//
			// Test if any error occurred during data transfer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// End data transfer
				//
				errorCode = sbgEComTransferSendEnd(pHandle, command);
			}
		}
	}
	else
	{
		//
		// One of the provided parameters is invalid
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Specific method to handle a large receive from the device.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	command					Original command asking for download.
 * \param[in]	pBuffer					Pointer to the buffer where to write data.
 * \param[out]	pActualSize				The final size written into the buffer.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful download.
 */
SbgErrorCode sbgEComTransferReceive(SbgEComHandle *pHandle, uint16 command, void *pBuffer, uint32 *pActualSize, uint32 bufferSize)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgSplitBuffer	splitBuffer;
	uint32			transferSize;
	uint32			i;

	//
	// Test input parameters
	//
	if ( (pHandle) && (pBuffer) && (pActualSize) && (bufferSize) )
	{
		//
		// initiate data transfer
		//
		errorCode = sbgEComTransferReceiveInit(pHandle, command, &transferSize);
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Test that the provided buffer is large enough to receive all data
			//
			if (transferSize <= bufferSize)
			{
				//
				// Initialize Split buffer to help with sub buffer receive
				//
				sbgSplitBufferInitForWrite(&splitBuffer, pBuffer, transferSize, SBG_ECOM_PACKET_SIZE);

				//
				// Receive buffers one by one 
				//
				for (i = 0; i < sbgSplitBufferGetSubBufferNbr(&splitBuffer); i++)
				{
					//
					// Receive a sub buffer
					//																									  
					errorCode = sbgEComTransferReceiveData(pHandle, command, sbgSplitBufferGetSubBuffer(&splitBuffer, i), sbgSplitBufferGetSubBufferOffset(&splitBuffer, i), sbgSplitBufferGetSubBufferSize(&splitBuffer, i));
					if (errorCode != SBG_NO_ERROR)
					{
						//
						// An error occurred, abort data transfer
						//
						break;
					}
				}

				//
				// Test if any error occurred during transfer
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// End data transfer
					//
					errorCode = sbgEComTransferReceiveEnd(pHandle, command);
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// Since the transfer was successful update output variable pActualSize
						//
						*pActualSize = transferSize;
					}
				}
			}
			else
			{
				//
				// Provided buffer is too small
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}
	else
	{
		//
		// One of the provided parameters is invalid
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}