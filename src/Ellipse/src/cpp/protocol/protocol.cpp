#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "protocol.h"
#include "../misc/sbgCrc.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the protocol system used to communicate with the product and return the created handle.
 *	\param[in]	pHandle					Pointer on an allocated protocol structure to initialize.
 *	\param[in]	pInterface				Interface to use for read/write operations.
 *	\return								SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgEComProtocolInit(SbgEComProtocol *pHandle, SbgInterface *pInterface)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	
	//
	// Check input parameters
	//
	if ( (pInterface) && (pHandle) )
	{
		//
		// Initialize the created protocol handle
		//
		pHandle->pLinkedInterface = pInterface;
		pHandle->rxBufferSize = 0;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Close the protocol system.
 *	\param[in]	pHandle					A valid protocol handle to close.
 *	\return								SBG_NO_ERROR if we have closed and released the protocol system.
 */
SbgErrorCode sbgEComProtocolClose(SbgEComProtocol *pHandle)
{
	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Send a frame to the device (size should be less than 4086 bytes).
 *	\param[in]	pHandle					A valid protocol handle.
 *	\param[in]	cmd						Command number to send (0 to 65535).
 *	\param[in]	pData					Pointer on the data payload to send or NULL if no payload.
 *	\param[in]	size					Size in bytes of the data payload (less than 4086).
 *	\return								SBG_NO_ERROR if the frame has been sent.
 */
SbgErrorCode sbgEComProtocolSend(SbgEComProtocol *pHandle, uint16 cmd, const void *pData, uint32 size)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	uint8 outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer outputStream;
	uint16 frameCrc;

	//
	// First, test input parameters
	//
	if ( (pHandle) && (size <= SBG_ECOM_MAX_PAYLOAD_SIZE) && ( ((size > 0) && (pData)) || (size == 0) ) )
	{
		//
		// Create a stream buffer to write the frame
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

		//
		// Write the header
		//
		sbgStreamBufferWriteUint8(&outputStream, SBG_ECOM_SYNC_1);
		sbgStreamBufferWriteUint8(&outputStream, SBG_ECOM_SYNC_2);

		//
		// Write the command field
		//
		sbgStreamBufferWriteUint16(&outputStream, cmd);

		//
		// Write the length field
		//
		sbgStreamBufferWriteUint16(&outputStream, (uint16)size);

		//
		// Write the payload part
		//
		sbgStreamBufferWriteBuffer(&outputStream, pData, size);

		//
		// Compute the CRC, we skip the two sync chars
		//
		frameCrc = sbgCrc16Compute(((uint8*)sbgStreamBufferGetLinkedBuffer(&outputStream)) + 2, sbgStreamBufferGetLength(&outputStream) - 2);

		//
		// Write the CRC
		//
		sbgStreamBufferWriteUint16(&outputStream, frameCrc);

		//
		// Write ETX char
		//
		sbgStreamBufferWriteUint8(&outputStream, SBG_ECOM_ETX);

		//
		// The frame has been generated so send it
		//
		errorCode = sbgInterfaceWrite(pHandle->pLinkedInterface, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));
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
 *	Try to receive a frame from the device and returns the cmd, data and size of data field.
 *	\param[in]	pHandle					A valid protocol handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 */
SbgErrorCode sbgEComProtocolReceive(SbgEComProtocol *pHandle, uint16 *pCmd, void *pData, uint32 *pSize, uint32 maxSize)
{
	SbgErrorCode errorCode = SBG_NOT_READY;
	SbgStreamBuffer inputStream;
	bool syncFound;
	uint32 payloadSize = 0;
	uint16 frameCrc;
	uint16 computedCrc;
	uint32 i;
	uint32 numBytesRead;
	uint16 receivedCmd;
	uint32 payloadOffset;

	//
	// First, test input parameters
	//
	if (pHandle)
	{
		// 
		// Set the return size to 0 in order to avoid possible bugs
		//
		if (pSize)
		{
			*pSize = 0;
		}

		//
		// Check if we can receive some new data (the receive buffer isn't full)
		//
		if (pHandle->rxBufferSize < SBG_ECOM_MAX_BUFFER_SIZE)
		{
			//
			// First try to read as much data as we can
			//
			if (sbgInterfaceRead(pHandle->pLinkedInterface, pHandle->rxBuffer + pHandle->rxBufferSize, &numBytesRead, SBG_ECOM_MAX_BUFFER_SIZE - pHandle->rxBufferSize) == SBG_NO_ERROR)
			{
				//
				// No error during reading so increment the number of bytes stored in the rx buffer
				//
				pHandle->rxBufferSize += numBytesRead;
			}
		}

		//
		// We have read all available data and stored them into the rx buffer
		// We will try to process all received data until we have found a valid frame.
		//
		while (pHandle->rxBufferSize > 0)
		{
			//
			// For now, we haven't found any start of frame
			//
			syncFound = FALSE;

			//
			// To find a valid start of frame we need at least 2 bytes in the reception buffer
			//
			if (pHandle->rxBufferSize >= 2)
			{
				//
				// Try to find a valid start of frame by looking for SYNC_1 and SYNC_2 chars
				//
				for (i = 0; i < pHandle->rxBufferSize-1; i++)
				{
					//
					// A valid start of frame should begin with SYNC and when STX chars
					//
					if ( (pHandle->rxBuffer[i] == SBG_ECOM_SYNC_1) && (pHandle->rxBuffer[i+1] == SBG_ECOM_SYNC_2) )
					{
						//
						// We have found the sync char, test if we have dummy bytes at the begining of the frame
						//
						if (i > 0)
						{
							//
							// Remove all dumy received bytes before the begining of the frame
							//
							memmove(pHandle->rxBuffer, pHandle->rxBuffer+i, pHandle->rxBufferSize-i);
							pHandle->rxBufferSize = pHandle->rxBufferSize-i;
						}

						//
						// The sync has been found
						//
						syncFound = TRUE;
						break;
					}
				}
			}

			//
			// Check if a valid start of frame has been found
			//
			if (syncFound)
			{
				//
				// A valid start of frame has been found, try to extract the frame if we have at least a whole frame.
				//
				if (pHandle->rxBufferSize < 8)
				{
					//
					// Don't have enough data for a valid frame
					//
					return SBG_NOT_READY;
				}

				//
				// Initialize an input stream buffer to parse the received frame
				//
				sbgStreamBufferInitForRead(&inputStream, pHandle->rxBuffer, pHandle->rxBufferSize);

				//
				// Skip both the Sync 1 and Sync 2 chars
				//
				sbgStreamBufferSeek(&inputStream, sizeof(uint8)*2, SB_SEEK_CUR_INC);

				//
				// Read the command
				//
				receivedCmd = sbgStreamBufferReadUint16(&inputStream);

				//
				// Read the payload size
				//
				payloadSize = (uint16)sbgStreamBufferReadUint16(&inputStream);

				//
				// Check that the payload size is valid
				//
				if (payloadSize <= SBG_ECOM_MAX_PAYLOAD_SIZE)
				{
					//
					// Check if we have received the whole frame
					//
					if (pHandle->rxBufferSize < payloadSize+9)
					{
						//
						// Don't have received the whole frame
						//
						return SBG_NOT_READY;
					}

					//
					// We should have the whole frame so for now, skip the payload part but before store the current cursor
					//
					payloadOffset = sbgStreamBufferTell(&inputStream);
					sbgStreamBufferSeek(&inputStream, payloadSize, SB_SEEK_CUR_INC);

					//
					// Read the frame CRC
					//
					frameCrc = sbgStreamBufferReadUint16(&inputStream);

					//
					// Read and test the frame ETX
					//
					if (sbgStreamBufferReadUint8(&inputStream) == SBG_ECOM_ETX)
					{
						//
						// Go back at the beginning of the payload part
						//
						sbgStreamBufferSeek(&inputStream, payloadOffset, SB_SEEK_SET);

						//
						// We have a frame so return the received command if needed even if the CRC is still not validated
						//
						if (pCmd)
						{
							*pCmd = receivedCmd;
						}

						//
						// Compute the CRC of the received frame (Skip SYNC 1 and SYNC 2 chars)
						//
						computedCrc = sbgCrc16Compute(((uint8*)sbgStreamBufferGetLinkedBuffer(&inputStream)) + 2, payloadSize + 4);
						
						//
						// Check if the received frame has a valid CRC
						//
						if (frameCrc == computedCrc)
						{
							//
							// Extract the payload if needed
							//
							if (payloadSize > 0)
							{
								//
								// Check if input parameters are valid
								//
								if ( (pData) && (pSize) )
								{
									//
									// Check if we have enough space to store the payload
									//
									if (payloadSize <= maxSize)
									{
										//
										// Copy the payload and return the payload size
										//
										*pSize = payloadSize;
										memcpy(pData, sbgStreamBufferGetCursor(&inputStream), payloadSize);
										errorCode = SBG_NO_ERROR;
									}
									else
									{
										//
										// Not enough space to store the payload, we will just drop the received data
										//
										errorCode = SBG_BUFFER_OVERFLOW;
									}									
								}
								else
								{
									errorCode = SBG_NULL_POINTER;
								}
							}
							else
							{
								//
								// No payload but the frame has been read successfully
								//
								errorCode = SBG_NO_ERROR;
							}
						}
						else
						{
							//
							// We have an invalid frame CRC and we will directly return this error
							//
							errorCode = SBG_INVALID_CRC;
						}

						//
						// We have read a whole valid frame so remove it from the buffer
						// First, test if the reception buffer contains more than just the current frame
						//
						if (pHandle->rxBufferSize > payloadSize+9)
						{
							//
							// We remove the read frame but we keep the remaining data
							//
							pHandle->rxBufferSize = pHandle->rxBufferSize-(payloadSize+9);
							memmove(pHandle->rxBuffer, pHandle->rxBuffer+payloadSize+9, pHandle->rxBufferSize);
						}
						else
						{
							//
							// We have parsed the whole received buffer so just empty it
							//
							pHandle->rxBufferSize = 0;
						}

						//
						// We have at least found a complete frame
						//
						return errorCode;
					}
				}
				
				//
				// Frame size invalid or the found frame is invalid so we should have incorrectly detected a start of frame.
				// Remove the SYNC 1 and SYNC 2 chars to retry to find a new frame
				//
				pHandle->rxBufferSize -= 2;
				memmove(pHandle->rxBuffer, pHandle->rxBuffer+2, pHandle->rxBufferSize);
			}
			else
			{
				//
				// Unable to find a valid start of frame so check if the last byte is a SYNC char in order to keep it for next time
				//
				if (pHandle->rxBuffer[pHandle->rxBufferSize-1] == SBG_ECOM_SYNC_1)
				{
					//
					// Report the SYNC char and discard all other bytes in the buffer
					//
					pHandle->rxBuffer[0] = SBG_ECOM_SYNC_1;
					pHandle->rxBufferSize = 1;
				}
				else
				{
					//
					// Discard the whole buffer
					//
					pHandle->rxBufferSize = 0;
				}

				//
				// Unable to find a frame
				//
				return SBG_NOT_READY;
			}
		}

		//
		// The whole buffer has been paresed and no valid frame has been found
		//
		return SBG_NOT_READY;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}
