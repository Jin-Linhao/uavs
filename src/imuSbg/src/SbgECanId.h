/*!
 *	\file		sbgECanIds.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		10 October 2014
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

#ifndef __SBG_ECAN_IDS_H__
#define __SBG_ECAN_IDS_H__

//----------------------------------------------------------------------//
//- Definition of all messages id for sbgECan                          -//
//----------------------------------------------------------------------//

/*!
 * Enum containing the list of messages that can be output on the can interface.
 */
typedef enum _SbgECanMsgId
{
	//
	// Output Messages
	//
	SBG_CAN_LOG_STATUS_01 = 0,
	SBG_CAN_LOG_STATUS_02,
	SBG_CAN_LOG_STATUS_03,
	SBG_CAN_LOG_UTC_0,
	SBG_CAN_LOG_UTC_1,
	SBG_CAN_LOG_IMU_INFO,
	SBG_CAN_LOG_IMU_ACCEL,
	SBG_CAN_LOG_IMU_GYRO,
	SBG_CAN_LOG_IMU_DELTA_VEL,
	SBG_CAN_LOG_IMU_DELTA_ANGLE,
	SBG_CAN_LOG_EKF_INFO,
	SBG_CAN_LOG_EKF_QUAT,
	SBG_CAN_LOG_EKF_EULER,
	SBG_CAN_LOG_EKF_ORIENTATION_ACC,
	SBG_CAN_LOG_EKF_POS,
	SBG_CAN_LOG_EKF_ALTITUDE,
	SBG_CAN_LOG_EKF_POS_ACC,
	SBG_CAN_LOG_EKF_VEL,
	SBG_CAN_LOG_EKF_VEL_ACC,
	SBG_CAN_LOG_SHIP_MOTION_INFO,
	SBG_CAN_LOG_SHIP_MOTION_0_0,
	SBG_CAN_LOG_SHIP_MOTION_1_0,
	SBG_CAN_LOG_SHIP_MOTION_2_0,
	SBG_CAN_LOG_SHIP_MOTION_3_0,
	SBG_CAN_LOG_SHIP_MOTION_0_1,
	SBG_CAN_LOG_SHIP_MOTION_1_1,
	SBG_CAN_LOG_SHIP_MOTION_2_1,
	SBG_CAN_LOG_SHIP_MOTION_3_1,
	SBG_CAN_LOG_SHIP_MOTION_0_2,		/*!< Ship motion velocity on monitoring point 0 (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_1_2,		/*!< Ship motion velocity on monitoring point 1 (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_2_2,		/*!< Ship motion velocity on monitoring point 2 (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_3_2,		/*!< Ship motion velocity on monitoring point 3 (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_INFO,	/*!< Delayed heave general information (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_0_0,		/*!< Delayed heave on monitoring 0 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_1_0,		/*!< Delayed heave on monitoring 1 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_2_0,		/*!< Delayed heave on monitoring 2 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_3_0,		/*!< Delayed heave on monitoring 3 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_0_1,		/*!< Delayed heave on monitoring 0 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_1_1,		/*!< Delayed heave on monitoring 1 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_2_1,		/*!< Delayed heave on monitoring 2 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_3_1,		/*!< Delayed heave on monitoring 3 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_0_2,		/*!< Delayed heave on monitoring 0 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_1_2,		/*!< Delayed heave on monitoring 1 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_2_2,		/*!< Delayed heave on monitoring 2 point (added since settings v1). */
	SBG_CAN_LOG_SHIP_MOTION_HP_3_2,		/*!< Delayed heave on monitoring 3 point (added since settings v1). */
	SBG_CAN_LOG_MAG_0,
	SBG_CAN_LOG_MAG_1,
	SBG_CAN_LOG_MAG_2,
	SBG_CAN_LOG_ODO_INFO,
	SBG_CAN_LOG_ODO_VEL,
	SBG_CAN_LOG_GPS1_VEL_INFO,
	SBG_CAN_LOG_GPS1_VEL,
	SBG_CAN_LOG_GPS1_VEL_ACC,
	SBG_CAN_LOG_GPS1_VEL_COURSE,
	SBG_CAN_LOG_GPS1_POS_INFO,
	SBG_CAN_LOG_GPS1_POS,
	SBG_CAN_LOG_GPS1_POS_ALT,
	SBG_CAN_LOG_GPS1_POS_ACC,
	SBG_CAN_LOG_GPS1_HDT_INFO,
	SBG_CAN_LOG_GPS1_HDT,
	SBG_CAN_LOG_GPS2_VEL_INFO,
	SBG_CAN_LOG_GPS2_VEL,
	SBG_CAN_LOG_GPS2_VEL_ACC,
	SBG_CAN_LOG_GPS2_VEL_COURSE,
	SBG_CAN_LOG_GPS2_POS_INFO,
	SBG_CAN_LOG_GPS2_POS,
	SBG_CAN_LOG_GPS2_POS_ALT,
	SBG_CAN_LOG_GPS2_POS_ACC,
	SBG_CAN_LOG_GPS2_HDT_INFO,
	SBG_CAN_LOG_GPS2_HDT,
	SBG_CAN_LOG_USER_HEADING_INFO,
	SBG_CAN_LOG_USER_HEADING,
	SBG_CAN_LOG_USER_VEL_NED_INFO,
	SBG_CAN_LOG_USER_VEL_NED,
	SBG_CAN_LOG_USER_VEL_NED_ACC,
	SBG_CAN_LOG_USER_VEL_XYZ_INFO,
	SBG_CAN_LOG_USER_VEL_XYZ,
	SBG_CAN_LOG_USER_VEL_XYZ_ACC,
	SBG_CAN_LOG_USER_POS_0,
	SBG_CAN_LOG_USER_POS_1,
	SBG_CAN_LOG_USER_POS_2,
	SBG_CAN_LOG_EVENT_INFO_A,
	SBG_CAN_LOG_EVENT_TIME_A,
	SBG_CAN_LOG_EVENT_INFO_B,
	SBG_CAN_LOG_EVENT_TIME_B,
	SBG_CAN_LOG_EVENT_INFO_C,
	SBG_CAN_LOG_EVENT_TIME_C,
	SBG_CAN_LOG_EVENT_INFO_D,
	SBG_CAN_LOG_EVENT_TIME_D,
	SBG_CAN_LOG_EVENT_INFO_E,
	SBG_CAN_LOG_EVENT_TIME_E,
	SBG_CAN_LOG_PRESSURE,				/*!< Pressure sensor such as Depth or Altimeter aiding equipment data (added since settings v2). */
	SBG_CAN_NUM_TX_MESSAGES,			/*!< Just used to know how many TX messages we have. */
} SbgECanMsgId;


#endif	/* __SBG_ECOM_CMDS_H__ */
