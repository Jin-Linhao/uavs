#include "commandsSettings.h"
#include "../misc/transfer.h"

//----------------------------------------------------------------------//
//- Settings commands                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Execute one of the available settings action : <BR>
 *			- SBG_ECOM_REBOOT_ONLY : Only reboot the device.<BR>
 *			- SBG_ECOM_SAVE_SETTINGS : Save the settings to non-volatile memory and then reboot the device.<BR>
 *			- SBG_ECOM_RESTORE_DEFAULT_SETTINGS : Restore default settings, save them to non-volatile memory and reboot the device.<BR>
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	action						One of the available SbgEComSettingsAction.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSettingsAction(SbgEComHandle *pHandle, SbgEComSettingsAction action)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	uint32			trial;
	SbgEComCmdId	cmd;

	//
	// Test that the protocol handle is valid
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
			cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SETTINGS_ACTION);

			//
			// Send the command and the action as a 1-byte payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, cmd, &action, 1);

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
 *	Send a complete set of settings to the device and store them into the FLASH memory.
 *	The device will reboot automatically to use the new settings.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the settings.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdImportSettings(SbgEComHandle *pHandle, const void *pBuffer, uint32 size)
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
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_IMPORT_SETTINGS);

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
 *	Retrieve a complete set of settings from the device as a buffer.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Allocated buffer that can hold the received settings.
 *	\param[out]	pSize						The number of bytes that have been stored into pBuffer.
 *	\param[in]	maxSize						The maximum buffer size in bytes that can be stored into pBuffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdExportSettings(SbgEComHandle *pHandle, void *pBuffer, uint32 *pSize, uint32 maxSize)
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
		cmd = SBG_ECOM_BUILD_ID(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_EXPORT_SETTINGS);

		//
		// Call function that handle data transfer
		//
		errorCode = sbgEComTransferReceive(pHandle, cmd, pBuffer, pSize, maxSize);
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