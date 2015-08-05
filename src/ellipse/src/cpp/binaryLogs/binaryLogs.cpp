#include "binaryLogs.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Parse an incoming log and fill the output union.
 *	\param[in]	command						Received command that should be a log id.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output union that stores parsed data.
 */
SbgErrorCode sbgEComBinaryLogParse(uint16 command, const void *pPayload, uint32 payloadSize, SbgBinaryLogData *pOutputData)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test input parameters
	//
	if ( (pPayload) && (payloadSize > 0) && (pOutputData) )
	{
		//
		// Parse the incoming log according to its type
		//
		switch (command)
		{
		case SBG_ECOM_LOG_STATUS:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseStatusData(pPayload, payloadSize, &pOutputData->statusData);
			break;
		case SBG_ECOM_LOG_IMU_DATA:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseImuData(pPayload, payloadSize, &pOutputData->imuData);
			break;
		case SBG_ECOM_LOG_EKF_EULER:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfEulerData(pPayload, payloadSize, &pOutputData->ekfEulerData);
			break;
		case SBG_ECOM_LOG_EKF_QUAT:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfQuatData(pPayload, payloadSize, &pOutputData->ekfQuatData);
			break;
		case SBG_ECOM_LOG_EKF_NAV:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfNavData(pPayload, payloadSize, &pOutputData->ekfNavData);
			break;
		case SBG_ECOM_LOG_SHIP_MOTION_0:
		case SBG_ECOM_LOG_SHIP_MOTION_1:
		case SBG_ECOM_LOG_SHIP_MOTION_2:
		case SBG_ECOM_LOG_SHIP_MOTION_3:
		case SBG_ECOM_LOG_SHIP_MOTION_HP_0:
		case SBG_ECOM_LOG_SHIP_MOTION_HP_1:
		case SBG_ECOM_LOG_SHIP_MOTION_HP_2:
		case SBG_ECOM_LOG_SHIP_MOTION_HP_3:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseShipMotionData(pPayload, payloadSize, &pOutputData->shipMotionData);
			break;
		case SBG_ECOM_LOG_ODO_VEL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseOdometerData(pPayload, payloadSize, &pOutputData->odometerData);
			break;
		case SBG_ECOM_LOG_UTC_TIME:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseUtcData(pPayload, payloadSize, &pOutputData->utcData);
			break;
		case SBG_ECOM_LOG_GPS1_VEL:
		case SBG_ECOM_LOG_GPS2_VEL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsVelData(pPayload, payloadSize, &pOutputData->gpsVelData);
			break;
		case SBG_ECOM_LOG_GPS1_POS:
		case SBG_ECOM_LOG_GPS2_POS:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsPosData(pPayload, payloadSize, &pOutputData->gpsPosData);
			break;
		case SBG_ECOM_LOG_GPS1_HDT:
		case SBG_ECOM_LOG_GPS2_HDT:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsHdtData(pPayload, payloadSize, &pOutputData->gpsHdtData);
			break;
		case SBG_ECOM_LOG_GPS1_RAW:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsRawData(pPayload, payloadSize, &pOutputData->gpsRawData);
			break;
		case SBG_ECOM_LOG_MAG:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseMagData(pPayload, payloadSize, &pOutputData->magData);
			break;
		case SBG_ECOM_LOG_MAG_CALIB:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseMagCalibData(pPayload, payloadSize, &pOutputData->magCalibData);
			break;
		case SBG_ECOM_LOG_DVL_BOTTOM_TRACK:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDvlData(pPayload, payloadSize, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_DVL_WATER_TRACK:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDvlData(pPayload, payloadSize, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_PRESSURE:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParsePressureData(pPayload, payloadSize, &pOutputData->pressureData);
			break;
		case SBG_ECOM_LOG_USBL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseUsblData(pPayload, payloadSize, &pOutputData->usblData);
			break;
		case SBG_ECOM_LOG_EVENT_A:
		case SBG_ECOM_LOG_EVENT_B:
		case SBG_ECOM_LOG_EVENT_C:
		case SBG_ECOM_LOG_EVENT_D:
		case SBG_ECOM_LOG_EVENT_E:
			//
			// Parse all events markers logs
			//
			errorCode = sbgEComBinaryLogParseEvent(pPayload, payloadSize, &pOutputData->eventMarker);
			break;
		case SBG_ECOM_LOG_DEBUG_0:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDebug0Data(pPayload, payloadSize, &pOutputData->debug0Data);
			break;
		default:
			//
			// This log isn't handled
			//
			errorCode = SBG_ERROR;
		}
	}
	else
	{
		//
		// Invalid input parameters
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}
