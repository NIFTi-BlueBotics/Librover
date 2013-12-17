 /***************************************************************************
 *   Copyright (C) 2011 BlueBotics SA                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef TYPES_H_
#define TYPES_H_


#define TWO_PI					(2.0 * M_PI)
#define RAD2DEG(a)				((a) * 180.0 / M_PI)
#define DEG2RAD(a)				((a) * M_PI / 180.0)
#define MIN(a,b)				((a) < (b) ? (a):(b))

#define BAT_CRITICAL			2
#define BAT_WARNING				1
#define BAT_OK					0
#define CALIB_LOW_VOLTAGE		45.0
#define CALIB_HIGH_VOLTAGE		60.0
#define TORQUE_MAX			8.0
#define TORQUE_THRES_H			5.0
#define TORQUE_THRES_L			3.0

#define ID_CORE 				0
#define ID_TRACK_LEFT			1
#define ID_TRACK_RIGHT			2
#define ID_FLIPPER_FRONT_LEFT	3
#define ID_FLIPPER_FRONT_RIGHT  4
#define ID_FLIPPER_REAR_LEFT	5
#define ID_FLIPPER_REAR_RIGHT	6
#define ID_CTRL_MAX				7

const char CTRL_NAMES[][32] = {"CORE", "TRACK_LEFT", "TRACK_RIGHT", "FLIP_FRONT_LEFT",\
							   "FLIP_FRONT_RIGHT", "FLIP_REAR_LEFT", "FLIP_REAR_RIGHT"};

// Masks to interpret controller status
#define SR_ERROR	 	   		(1 << 0)
#define SR_SERVO_DRIVE_STATUS   0xE
#define SR_MOTOR_ON	  	   		(1 << 4)
#define SR_REFERENCE_MODE  		(1 << 5)
#define SR_MOTOR_FAILURE   		(1 << 6)
#define SR_UNIT_MODE       		0x380
#define SR_GAIN_SCHEDULING 		(1 << 10)
#define SR_PROGRAM_RUNNING 		(1 << 12)
#define SR_CURRENT_LIMIT   		(1 << 13)
#define SR_MOTION_STATUS   		(1 << 14 | 1 << 15)
#define SR_RECORDER_STATUS 		(1 << 16 | 1 << 17)
#define SR_CPU_STATUS      		(1 << 27)
#define SR_STOPPED_LIMIT   		(1 << 28)
#define SR_ERROR_USER_PRG  		(1 << 29)

#define SR_GET_ERROR(status)			   ((status) & SR_ERROR)
#define SR_GET_SERVO_DRIVE_STATUS(status)  (((status) & SR_SERVO_DRIVE_STATUS) >> 1)
#define SR_GET_MOTOR_ON(status)			   (((status) & SR_MOTOR_ON) >> 4)
#define SR_GET_MOTION_STATUS(status)	   (((status) & SR_MOTION_STATUS) >> 14)

#define SD_OK			 	0
#define SD_UNDER_VOLTAGE 	1
#define SD_OVER_VOLTAGE  	2
#define SD_SHORT_CIRCUIT 	5
#define SD_OVERHEAT      	6
#define SD_FLIPPERMODE		7			/**< \brief Flipper are set in incompatible mode. */


#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif
/** \brief Geometrical parameters. */
typedef struct RoverParams{
	double trackDistance;				/**< \brief The distance between the two tracks. */
	double trackWheelRadius;			/**< \brief Radius of the track wheels. */
	double trackLength;					/**< \brief Distance between the track wheel axes. */
	double trackWidth;					/**< \brief Width of the track. */
	double flipperLength;				/**< \brief Distance between the track wheel center and the flipper extremity. */
	double flipperWidth;				/**< \brief Width of the flipper. */
	double flipperOffset;				/**< \brief Angle offset between the angle reference axis and the symetry axis of the flipper [rad]. */
	double laserX, laserY, laserZ;		/**< \brief The coordinates of the laser center. */
	double imuX, imuY, imuZ;			/**< \brief IMU coordinate center. */
	double omniX, omniY, omniZ;			/**< \brief Omni-camera coordinate center. */
	double omniAngleOffset;				/**< \brief Omni-camera #0 offset angle [rad] regarding the x-axis. */
	double referentialX,referentialY;
	double referentialZ;				/**< \brief Robot referential coordinate center (referentialZ: distance to the ground). */
	double vMax, wMax;					/**< \brief Max translational and rotational speeds. */

}RoverParams;


#endif /* TYPES_H_ */
