#include "interfaceFile.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
//- Internal methods declarations                                      -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Open a file as an interface for read only operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	filePath						File path to open.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceFileOpen(SbgInterface *pHandle, const char *filePath)
{
	SbgErrorCode	 errorCode = SBG_NO_ERROR;
	FILE			*pInputFile;
			
	//
	// First check if we have a valid pHandle
	//
	if (pHandle)
	{
		//
		// First, initialize the interface handle to zero
		//
		sbgInterfaceZeroInit(pHandle);

		//
		// Try to open the file
		//
		pInputFile = fopen(filePath, "rb");

		//
		// Test if the input file has been opened
		//
		if (pInputFile)
		{
			//
			// Define the interface members
			//
			pHandle->handle = pInputFile;
			pHandle->type = SBG_IF_TYPE_FILE;
			pHandle->pReadFunc = sbgInterfaceFileRead;
			pHandle->pWriteFunc = sbgInterfaceFileWriteFake;
		}
		else
		{
			//
			// Unable to open the input file
			//
			errorCode = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// Invalid input handle
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Open a file as an interface for write only operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	filePath						File path to open.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceFileWriteOpen(SbgInterface *pHandle, const char *filePath)
{
	SbgErrorCode	 errorCode = SBG_NO_ERROR;
	FILE			*pInputFile;
			
	//
	// First check if we have a valid pHandle
	//
	if (pHandle)
	{
		//
		// Try to open the file
		//
		pInputFile = fopen(filePath, "wb");

		//
		// Test if the input file has been opened
		//
		if (pInputFile)
		{
			//
			// Define the interface members
			//
			pHandle->handle = pInputFile;
			pHandle->type = SBG_IF_TYPE_FILE;
			pHandle->pReadFunc = sbgInterfaceFileReadFake;
			pHandle->pWriteFunc = sbgInterfaceFileWrite;
		}
		else
		{
			//
			// Unable to open the input file
			//
			errorCode = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// Invalid input handle
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Destroy an interface initialized using sbgInterfaceFileOpen.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SbgErrorCode sbgInterfaceFileClose(SbgInterface *pHandle)
{
	FILE *pInputFile;

	//
	// Test that we have a valid interface
	//
	if (pHandle)
	{
		//
		// Get the internal FILE handle
		//
		pInputFile = (FILE*)(pHandle->handle);

		//
		// Close the input file only if opened
		//
		if (pInputFile)
		{
			fclose(pInputFile);
			pHandle->handle = NULL;
		}

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Returns the file size in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The file size in bytes.
 */
uint32 sbgInterfaceFileGetSize(SbgInterface *pHandle)
{
	FILE	*pInputFile;
	uint32	 cursorPos;
	uint32	 fileSize;

	//
	// Initialize the file size to 0 in case of error
	//
	fileSize = 0;

	//
	// Test that we have a valid interface
	//
	if (pHandle)
	{
		//
		// Get the internal FILE handle
		//
		pInputFile = (FILE*)(pHandle->handle);

		//
		// Test if the file handle is valid
		//
		if (pInputFile)
		{
			//
			// Compute the file size
			//
			cursorPos = ftell(pInputFile);
			fseek(pInputFile, 0, SEEK_END);
			fileSize = ftell(pInputFile);
			fseek(pInputFile, cursorPos, SEEK_SET);
		}		
	}
	
	return fileSize;
}

/*!
 *	Returns the current cursor position in the file in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The current cursor position in bytes.
 */
uint32 sbgInterfaceFileGetCursor(const SbgInterface *pHandle)
{
	FILE	*pInputFile;

	//
	// Test that we have a valid interface
	//
	if (pHandle)
	{
		//
		// Get the internal FILE handle
		//
		pInputFile = (FILE*)(pHandle->handle);

		//
		// Test if the file handle is valid
		//
		if (pInputFile)
		{
			//
			// Return the current cursor position
			//
			return ftell(pInputFile);
		}		
	}
	
	return 0;
}


//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceFileWrite(SbgInterface *pHandle, const void *pBuffer, uint32 bytesToWrite)
{
	FILE *pOutputFile;
	uint32 bytesWritten;

	//
	// Test input parameters
	//
	if ( (pHandle) && (pBuffer) )
	{
		//
		// Get the internal FILE handle
		//
		pOutputFile = (FILE*)(pHandle->handle);

		//
		// Write as much bytes as we can
		//
		bytesWritten = fwrite(pBuffer, sizeof(uint8), bytesToWrite, pOutputFile);

		//
		// Check if we could successfuly write our bytes
		//
		if (bytesWritten == bytesToWrite)
		{

			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_WRITE_ERROR;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceFileRead(SbgInterface *pHandle, void *pBuffer, uint32 *pReadBytes, uint32 bytesToRead)
{
	FILE *pInputFile;

	//
	// Test input parameters
	//
	if ( (pHandle) && (pBuffer) && (pReadBytes) )
	{
		//
		// Get the internal FILE handle
		//
		pInputFile = (FILE*)(pHandle->handle);

		//
		// Read some bytes from the file
		//
		*pReadBytes = fread(pBuffer, sizeof(uint8), bytesToRead, pInputFile);

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}


/*!
 * Fake write function for read only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceFileWriteFake(SbgInterface *pHandle, const void *pBuffer, uint32 bytesToWrite)
{
	//
	// Simply avoid warnings and return an error
	//
	(void)pHandle;
	(void)pBuffer;
	(void)bytesToWrite;

	return SBG_ERROR;
}

/*!
 * Fake read function for write only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceFileReadFake(SbgInterface *pHandle, void *pBuffer, uint32 *pReadBytes, uint32 bytesToRead)
{
	//
	// Simply avoid warnings and return an error
	//
	(void)pHandle;
	(void)pBuffer;
	(void)pReadBytes;
	(void)bytesToRead;

	return SBG_ERROR;
}