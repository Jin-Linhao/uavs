/*!
 *	\file		sbgEComIds.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		25 February 2013
 *
 *	\brief		Defines all sbgECom commands identifiers.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */

/*!
 *	\mainpage SBG Systems Enhanced Communication library documentation
 *	Welcome to the sbgECom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgECom library.
 */

#ifndef __SBG_ECOM_IDS_H__
#define __SBG_ECOM_IDS_H__

//----------------------------------------------------------------------//
//- Macro definitions						                           -//
//----------------------------------------------------------------------//

/*!
 * Helper macro to build an id with its class
 */
#define SBG_ECOM_BUILD_ID(classId, logId)			(((uint16)classId << 8) | (uint8)logId)

//----------------------------------------------------------------------//
//- Constants and types definitions                                    -//
//----------------------------------------------------------------------//

/*!
 * Redefine SbgEComCmdId as a simple 16-bit unsigned integer.
 * The Id now has to be built using SBG_ECOM_BUILD_ID macro to build an ID with its associated class.
 */
typedef uint16	SbgEComCmdId;

/*!
 *	This type defines any message identifier.
 *	Because message identifiers enum will be different with each class id, we use a generic uint8 rather than an enum.
 */
typedef uint8	SbgEComMsgId;

//----------------------------------------------------------------------//
//- Definition of all class id for sbgECom                             -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the message classes available.
 */
typedef enum _SbgEComClass
{
	SBG_ECOM_CLASS_LOG_ECOM_0			= 0x00,		/*!< Class that contains sbgECom protocol input/output log messages. */

	SBG_ECOM_CLASS_LOG_NMEA_0			= 0x02,		/*!< Class that contains NMEA (and NMEA like) output logs. <br>
														 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_THIRD_PARTY_0	= 0x04,		/*!< Class that contains third party output logs.
														 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_CMD_0			= 0x10		/*!< Class that contains sbgECom protocol commands */
} SbgEComClass;

//----------------------------------------------------------------------//
//- Definition of all messages id for sbgECom                          -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the available ECom output logs from the sbgECom library.
 */
typedef enum _SbgEComLog
{
	SBG_ECOM_LOG_STATUS 			= 1,			/*!< Status general, clock, com aiding, solution, heave */

	SBG_ECOM_LOG_UTC_TIME 			= 2,			/*!< Provides UTC time reference */

	SBG_ECOM_LOG_IMU_DATA 			= 3,			/*!< Includes IMU status, acc., gyro, temp delta speeds and delta angles values */

	SBG_ECOM_LOG_MAG 				= 4,			/*!< Magnetic data with associated accelerometer on each axis */
	SBG_ECOM_LOG_MAG_CALIB 			= 5,			/*!< Magnetometer calibration data (raw buffer) */

	SBG_ECOM_LOG_EKF_EULER 			= 6,			/*!< Includes roll, pitch, yaw and their accuracies on each axis */
	SBG_ECOM_LOG_EKF_QUAT 			= 7,			/*!< Includes the 4 quaternions values */
	SBG_ECOM_LOG_EKF_NAV 			= 8,			/*!< Position and velocities in NED coordinates with the accuracies on each axis */

	SBG_ECOM_LOG_SHIP_MOTION_0		= 9,			/*!< Heave, surge and sway and accelerations on each axis for up to 4 points */
	SBG_ECOM_LOG_SHIP_MOTION_1		= 10,			/*!< Return ship motion such as surge, sway, heave at the first monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_2		= 11,			/*!< Return ship motion such as surge, sway, heave at the second monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_3		= 12,			/*!< Return ship motion such as surge, sway, heave at the third monitoring point. */

	SBG_ECOM_LOG_GPS1_VEL 			= 13,			/*!< GPS velocities from primary or secondary GPS receiver */
	SBG_ECOM_LOG_GPS1_POS 			= 14,			/*!< GPS positions from primary or secondary GPS receiver */
	SBG_ECOM_LOG_GPS1_HDT 			= 15,			/*!< GPS true heading from dual antenna system */
	SBG_ECOM_LOG_GPS1_RAW			= 31,			/*!< GPS 1 raw data for post processing. */

	SBG_ECOM_LOG_GPS2_VEL			= 16,			/*!< GPS 2 velocity log data. */
	SBG_ECOM_LOG_GPS2_POS			= 17,			/*!< GPS 2 position log data. */
	SBG_ECOM_LOG_GPS2_HDT			= 18,			/*!< GPS 2 true heading log data. */

	SBG_ECOM_LOG_ODO_VEL 			= 19,			/*!< Provides odometer velocity */

	SBG_ECOM_LOG_USER_HEADING		= 20,			/*!< User true heading aiding data. */
	SBG_ECOM_LOG_USER_VEL_NED		= 21,			/*!< User velocity aiding data expressed in the NED local frame. */
	SBG_ECOM_LOG_USER_VEL_XYZ		= 22,			/*!< User velocity aiding data expressed in the device local frame. */
	SBG_ECOM_LOG_USER_POS_LLA		= 23,			/*!< User position aiding data expressed in latitude, longitude, altitude frame. */

	SBG_ECOM_LOG_EVENT_A 			= 24,			/*!< Event markers sent when events are detected on sync in A pin */
	SBG_ECOM_LOG_EVENT_B 			= 25,			/*!< Event markers sent when events are detected on sync in B pin */
	SBG_ECOM_LOG_EVENT_C			= 26,			/*!< Event markers sent when events are detected on sync in C pin */
	SBG_ECOM_LOG_EVENT_D 			= 27,			/*!< Event markers sent when events are detected on sync in D pin */
	SBG_ECOM_LOG_EVENT_E			= 28,			/*!< Event marker for the synchronization E input signal. */

	SBG_ECOM_LOG_DVL_BOTTOM_TRACK	= 29,			/*!< Doppler Velocity Log for bottom tracking data. */
	SBG_ECOM_LOG_DVL_WATER_TRACK	= 30,			/*!< Doppler Velocity log for water layer data. */

	SBG_ECOM_LOG_SHIP_MOTION_HP_0	= 32,			/*!< Return delayed ship motion such as surge, sway, heave at the main monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_1	= 33,			/*!< Return delayed ship motion such as surge, sway, heave at the first monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_2	= 34,			/*!< Return delayed ship motion such as surge, sway, heave at the second monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_3	= 35,			/*!< Return delayed ship motion such as surge, sway, heave at the third monitoring point. */

	SBG_ECOM_LOG_PRESSURE			= 36,			/*!< Pressure sensor such as depth sensor or altimeter. */

	SBG_ECOM_LOG_USBL				= 37,			/*!< Raw USBL position data for subsea navigation. */

	SBG_ECOM_LOG_DEBUG_0			= 38,			/*!< */

	SBG_ECOM_LOG_ECOM_NUM_MESSAGES					/*!< Helper definition to know the number of ECom messages */
} SbgEComLog;

/*!
 * Enum that defines all the available Nmea output logs from the sbgECom library.
 */
typedef enum _SbgEComNmeaLog
{
	SBG_ECOM_LOG_NMEA_GGA 			= 0,			/*!< Latitude, Longitude, Altitude, Quality indicator. */
	SBG_ECOM_LOG_NMEA_RMC 			= 1,			/*!< Latitude, Longitude, velocity, course over ground. */
	SBG_ECOM_LOG_NMEA_ZDA 			= 2,			/*!< UTC Time. */
	SBG_ECOM_LOG_NMEA_HDT 			= 3,			/*!< Heading (True). */
	SBG_ECOM_LOG_NMEA_GST			= 4,			/*!< GPS Pseudorange Noise Statistics. */
	SBG_ECOM_LOG_NMEA_VBW			= 5,			/*!< Water referenced and ground referenced speed data. */

	SBG_ECOM_LOG_NMEA_PRDID 		= 6,			/*!< RDI proprietary sentence. Pitch, Roll, Heading */
	SBG_ECOM_LOG_NMEA_NUM_MESSAGES					/*!< Helper definition to know the number of NMEA messages */
} SbgEComNmeaLog;

/*!
 * Enum that defines all the available Proprietary output logs from the sbgECom library.
 */
typedef enum _SbgEComThirdPartyLog
{
	SBG_ECOM_THIRD_PARTY_TSS1 		= 0,			/*!< Roll, pitch and heave data */
	SBG_ECOM_LOG_THIRD_PARTY_NUM_MESSAGES			/*!< Helper definition to know the number of third party messages */
} SbgEComThirdPartyLog;

/*!
 * Enum that defines all the available commands for the sbgECom library.
 */
typedef enum _SbgEComCmd
{
	/* Acknowledge */
	SBG_ECOM_CMD_ACK			 			= 0,			/*!< Acknowledge */

	/* Special settings commands */
	SBG_ECOM_CMD_SETTINGS_ACTION 			= 1,			/*!< Performs various settings actions */
	SBG_ECOM_CMD_IMPORT_SETTINGS 			= 2,			/*!< Imports a new settings structure to the sensor */
	SBG_ECOM_CMD_EXPORT_SETTINGS 			= 3,			/*!< Export the whole configuration from the sensor */

	/* Device info */
	SBG_ECOM_CMD_INFO 						= 4,			/*!< Get basic device information */

	/* Sensor parameters */
	SBG_ECOM_CMD_INIT_PARAMETERS 			= 5,			/*!< Initial configuration */
	SBG_ECOM_CMD_SET_MOTION_PROFILE 		= 6,			/*!< Set a new motion profile */
	SBG_ECOM_CMD_MOTION_PROFILE_INFO 		= 7,			/*!< Get motion profile information */
	SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM	= 8,			/*!< Sensor alignment and lever arm on vehicle configuration */
	SBG_ECOM_CMD_AIDING_ASSIGNMENT 			= 9,			/*!< Aiding assignments such as RTCM / GPS / Odometer configuration */

	/* Magnetometer configuration */
	SBG_ECOM_CMD_MAGNETOMETER_SET_MODEL 	= 10,			/*!< Set a new magnetometer error model */
	SBG_ECOM_CMD_MAGNETOMETER_MODEL_INFO 	= 11,			/*!< Get magnetometer error model information */
	SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE 	= 12,			/*!< Magnetometer aiding rejection mode */
	SBG_ECOM_CMD_SET_MAG_CALIB 				= 13,			/*!< Set magnetic soft and hard Iron calibration data */

	/* Magnetometer onboard calibration */
	SBG_ECOM_CMD_START_MAG_CALIB			= 14,			/*!< Start / reset internal magnetic field logging for calibration. */
	SBG_ECOM_CMD_COMPUTE_MAG_CALIB			= 15,			/*!< Compute a magnetic calibration based on previously logged data. */

	/* GNSS configuration */
	SBG_ECOM_CMD_GNSS_1_SET_MODEL 			= 16,			/*!< Set a new GNSS model */
	SBG_ECOM_CMD_GNSS_1_MODEL_INFO 			= 17,			/*!< Get GNSS model information */
	SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT = 18,			/*!< GNSS installation configuration (lever arm, antenna alignments) */
	SBG_ECOM_CMD_GNSS_1_REJECT_MODES 		= 19,			/*!< GNSS aiding rejection modes configuration. */

	/* Odometer configuration */
	SBG_ECOM_CMD_ODO_CONF 					= 20,			/*!< Odometer gain, direction configuration */
	SBG_ECOM_CMD_ODO_LEVER_ARM 				= 21,			/*!< Odometer installation configuration (lever arm) */
	SBG_ECOM_CMD_ODO_REJECT_MODE 			= 22,			/*!< Odometer aiding rejection mode configuration. */

	/* Interfaces configuration */
	SBG_ECOM_CMD_UART_CONF 					= 23,			/*!< UART interfaces configuration */
	SBG_ECOM_CMD_CAN_BUS_CONF 				= 24,			/*!< CAN bus interface configuration */
	SBG_ECOM_CMD_CAN_OUTPUT_CONF			= 25,			/*!< CAN identifiers configuration */

	/* Events configuration */
	SBG_ECOM_CMD_SYNC_IN_CONF 				= 26,			/*!< Synchronization inputs configuration */
	SBG_ECOM_CMD_SYNC_OUT_CONF 				= 27,			/*!< Synchronization outputs configuration */
	SBG_ECOM_CMD_VIRTUAL_ODOMETER_CONF 		= 28,			/*!< Virtual Odometer distance configuration */

	/* Output configuration */
	SBG_ECOM_CMD_NMEA_TALKER_ID 			= 29,			/*!< NMEA talker ID configuration */
	SBG_ECOM_CMD_OUTPUT_CONF 				= 30,			/*!< Output configuration */
	SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF 	= 31,			/*!< Legacy serial output mode configuration */

	/* Avanced configuration */
	SBG_ECOM_CMD_ADVANCED_CONF 				= 32,			/*!< Advanced settings configuration */

	/* Misc. */
	SBG_ECOM_LOG_ECOM_NUM_CMDS								/*!< Helper definition to know the number of commands */
} SbgEComCmd;

#endif	/* __SBG_ECOM_CMDS_H__ */
