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
/** \file rover.h
 *  \brief Regrouping communication and motion hardware.
 */

#ifndef ROVER_H_
#define ROVER_H_

#include "types.h"
#include "core.h"
#include "controller.h"
#include "flipper.h"
#include "track.h"
#include <unistd.h>

extern "C" {
#include <tulibs/param.h>
#include <tulibs/config.h>
#include <libcan/can.h>
#include <libcan/can_cpc.h>
}
/** \class Rover
 *  \brief A top level class controlling communication and motion hardware.
 *
 *  This class supervises and manages the platform internal variables and settings.
 *  It regroups all degree of freedom, internal variables and settings of the entire platform.
 *  It also manages the USB-CAN hardware in order to communicate to all Elmos controllers.
 */
class Rover {

	public:
		static const double calibAngle;
		static void GetDefaultParams(RoverParams *params);
		/** \brief Constructor.
		 *  \param[in] canDevPath CAN device path. Ex.: /dev/usb/cpc_usb0.
		 *  \param[in] *params Rover geometrical parameters.
		 *  \param[in] bestInit Initialize the flipper more accurately otherwise roughly.*/
		Rover(const char* canDevPath, RoverParams *params, bool bestInit = false);
		/** \brief Destructor. Cleanup and close CAN device. */
		~Rover();
		/** \brief Initialize/Unfold chassis/flippers with interference management */
		bool InitFlippers(bool bestInit);
		int Restart3D(); /**< \brief Restart the initialisation of 3D laser scanner */
		int EnableMotion(int enable);	/**< \brief Enable controllers power stage. */
		Controller* GetController(int idx); /**< \brief Get controller object. \param[in] idx The controller CAN-ID. */
		/** \brief Get platform geometrical params. \param[in] *params The controller CAN-ID. */
		void GetRoverParams(RoverParams *params);
		int EnableFlippers(bool enable); /**< \brief Enable/disable flippers power stage. */
		/** \brief Move flippers to a corresponding angle. */
		int SetFlippersAngle(double frontLeft, double frontRight,  double rearLeft, double rearRight);
		/** \brief Move flipper to a corresponding angle.
		 * \param[in] angle Angle [\a rad] to set.
		 * \param[in] object Possible object id are: ID_FLIPPER_FRONT_LEFT, ID_FLIPPER_FRONT_RIGHT, ID_FLIPPER_REAR_LEFT, ID_FLIPPER_REAR_RIGHT.*/
		int SetFlipperAngle(double angle, int object);
		/** \brief Read actual corresponding flippers angle. */
		int ReadFlippersAngle(double *frontLeft, double *frontRight,  double *rearLeft, double *rearRight);
		/** \brief Read actual corresponding flippers angle. */
		int ReadFlippersEncoder(int *frontLeft, int*frontRight,  int *rearLeft, int *rearRight);
		/** \brief Read analog/potentiometer corresponding angle. */
		int ReadAnalogFlippersAngle(double *frontLeft, double *frontRight,  double *rearLeft, double *rearRight);
		/** \brief Set flippers torques while conserving the positions of the flippers. */
		int SetFlippersTorque(double frontTorque = 3.0, double rearTorque = 3.0);
		/** \brief Check all flippers motion status. */
		int FlippersAngleReached(bool *reached);
		int WaitFlippers();/**< \brief Wait until all flippers has reached destination angle. */

		int SetSpeed(double v, double omega); /**< \brief Set translation and rotation speed of the platform. */
		int GetSpeed(double *v, double *omega); /**< \brief Get translation and rotation speed of the platform. */
		int SetSpeedLR(double left, double right); /**< \brief Set translation speed of each track. */
		int GetSpeedLR(double *left, double *right); /**< \brief Get translation speed of each track. */
		int GetEncodersLR(int *left, int *right); /**< \brief Get tracks encoders. */

		int SetScanningSpeed(double radPerSec); /**< \brief Set scanner rotation speed [\a rad/s]. */
		int GetScanningSpeed(double *val); /**< \brief Get scanner actual rotation speed [\a rad/s]. */
		int GetScannerAngle(double *val);/**< \brief Get scanner actual angle [\a rad]. */
		int GetEstop(int *eStop);
		int GoMiddlePos(double angleOffset);/**< \brief Set scanner to the middle position with an offset in [\a rad]. */

		int ReadDOFCurrent(double *iq, int object);
		int ReadBatteryLevel(double *level, int *state); /**< \brief Get battery level [\a 0.0 - 100.0%]. */
		int ReadBatteryVoltage(double *val);/**< \brief Read battery voltage [\a V]. */
		int GetDifferentialAngles(double *left, double *right); /**< \brief Get tracks angle according to the ground [\a rad]. */
		int GetMotorsFailure(int status[]); /**< \brief Get all actual motor failure register. */
		int GetControllersStatus(int status[]); /**< \brief Get all actual controller status register. */
		int GetControllersError(int error[]); /**< \brief Get all actual controller error register. */
		int SetBrake(int on);  /**< \brief Enable/disable brake. */
		int GetBrake(int *val); /**< \brief Get actual status of the brake. */

		int CalibrateChassis();  /**< \brief Calibrate angles of flippers and tracks on flat ground. */
		int SetVoltageCalibration(double offset, double gain); /**< \brief Calibrate voltage scaling. */
		/** \brief Get flipper interference status during start-up phase. */
		int GetFlipperInterference(int flipInter[], bool analogRead = true, double angles[] = NULL);
		bool GetFlippersInitState(); /**< \brief Actual initialisation state: 1 = initialised; 0 = NOT initialised */


	private:
		RoverParams roverParams;
		can_device_t canDev;

		Core* core;
		Flipper* flipperRearRight;
		Flipper* flipperFrontRight;
		Track* trackRight;
		Flipper* flipperRearLeft;
		Flipper* flipperFrontLeft;
		Track* trackLeft;
		Controller *dof[ID_CTRL_MAX];
		Track *track[2];
		Flipper *flipper[4];
		bool isFlippersInit;

		int InitCan(const char* canDevPath); /**< \brief Intitialize CAN communication. */
		 /**< \brief Move flippers with analog/potentiometer feedback. */
		int SetAnalogFlippersAngle(double frontLeft, double frontRight,  double rearLeft, double rearRight);

};

#endif /* ROVER_H_ */
