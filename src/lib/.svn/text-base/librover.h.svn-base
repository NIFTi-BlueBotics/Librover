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

/** \file librover.h
 *  \brief API controlling the low level hardware.
 *
 * \attention The int returned by the functions is a communication error code. A code
 * different from zero is an error (see can.h for more details).
 * All unit used throughout the API are SI e.g. rad, meter, second, Hz etc.
 *
 * \mainpage BlueBotics Librover Source Code Documentation
 * \section Platform Overview
 * \image html 00123-90-02_SOFTWARE.JPG
 * \author Copyright (C) 2011 BlueBotics SA
 */


#ifndef LIBROVER_H_
#define LIBROVER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"



/** \param[out] *params Fill in params with factory setting. */
void nrGetDefaultParams(RoverParams *params);

/** \brief Initialize the library with device path, rover parameters and bestInit flag.
 *  \param[in] *device USB-Can device path.
 *  \param[in] *params Rover parameters to be initialized.
 *  \param[in] bestInit If bestInit is != 0 the flippers move up to \a 90 deg for optimal flipper angle calibration. */
void nrInit(const char *device, RoverParams *params, int bestInit);

/** \brief Initialise flippers.
 *  \return 1 = SUCCESS; 0 = ESTOP is pressed */
int nrInitFlippers(int bestInit);

/** \brief Restart 3D laser scanner.
 *  \return CAN error ID. */
int nrRestart3D();

/** \brief Retrieve the current rover parameters (refer to types.h for RoverParams \a *params members information).
 *  \param[out] *params *params will be filled in with current rover parameters. */
void nrGetRoverParams(RoverParams *params);

/** \brief Clean up library. */
void nrDestroy();

/** \brief Enable/disable rover controllers.
 *  \return CAN error ID.
 *  \param[in] enable 1 to enable and 0 to disable. */
int nrEnable(int enable);

/** \brief Set translation [\e m/s] and rotation [\e rad/s] speed.
 *  \return CAN error ID.
 *  \param[in] v Platform translation speed  [\e m/s].
 *  \param[in] w Platform rotation speed [\e rad/s]. */
int nrSetSpeed(double v, double w);

/** \brief Get the current translation [\a m/s] and rotation speed [\e rad/s].
 *  \return CAN error ID. */
/*  \param[out] *v Filled in with platform translation speed [\e m/s].
 *  \param[out] *w Filled in with platform rotation speed [\e rad/s]. */
int nrGetSpeed(double *v, double *w);

/** \brief Set the left and right track speed [\a m/s].
 *  \return CAN error ID.
 *  \param[in] left Left track speed [\a m/s].
 *  \param[in] right Right track speed [\a m/s]. */
int nrSetSpeedLR(double left, double right);

/** \brief Get the current left and right track speed [\a m/s].
 *  \return CAN error ID.
 *  \param[out] *left Filled in with left track speed [\a m/s].
 *  \param[out] *right Filled in with right track speed [\a m/s].*/
int nrGetSpeedLR(double *left, double *right);

/** \brief Get the left and right track encoder value [\a ticks]. There are \a 60 \a ticks per motor-revolution.
 *  The tracks reduction-ratio is \a 100.
 *  \return CAN error ID.
 *  \param[out] *left Filled in with left track speed [\a m/s].
 *  \param[out] *right Filled in with right track speed [\a m/s].
 * */
int nrGetEncodersLR(int *left, int *right);

/** \brief Set flipper angles [\a rad]. All flippers has the same referential coordinate. They all rotate around the \a Y-axis.
 *  That means negative values for front flippers makes the flipper to move up.
 *  \return CAN error ID.
 *  \param[in] frontLeft Front left flipper angle [\a rad].
 *  \param[in] frontRight Front right flipper angle [\a rad].
 *  \param[in] rearLeft Rear left flipper angle [\a rad].
 *  \param[in] rearRight Rear right flipper angle [\a rad]. */
int nrSetFlippers(double frontLeft, double frontRight, double rearLeft, double rearRight);

/** \brief Set flipper torques [\a A] while conserving the positions of the flippers.
 *   Maximum torques are truncated at a maximum of 6A (TORQUE_MAX).
 *   Return values are CAN error ID + '7' = incompatible controller mode.
 *   Front flippers has the same torque and rear flippers has the same torque.
 *  \param[in] frontTorque flippers torque [\a A].
 *  \param[in] rearTorque flippers torque [\a A]. */
int nrSetFlippersTorque(double frontTorque, double rearTorque);

/** \brief Set flipper angles [\a rad]. All flippers has the same referential coordinate. They all rotate around the \a Y-axis.
 *  That means negative values for front flippers makes the flipper to move up.
 *  \return CAN error ID.
 *  \param[in] angle Angle to be set [\a rad].
 *  \param[in] object Object id. Possible object id are: ID_FLIPPER_FRONT_LEFT, ID_FLIPPER_FRONT_RIGHT,
 *  ID_FLIPPER_REAR_LEFT, ID_FLIPPER_REAR_RIGHT.*/
int nrSetFlipper(double angle, int object);

/** \brief  Get current flipper angles [\a rad].
 *  \return CAN error ID.
 *  \param[out] frontLeft Filled in with actual front left flipper angle [\a rad].
 *  \param[out] frontRight Filled in with actual front right flipper angle [\a rad].
 *  \param[out] rearLeft Filled in with actual rear left flipper angle [\a rad].
 *  \param[out] rearRight Filled in with actual rear right flipper angle [\a rad]. */
int nrGetFlippers(double *frontLeft, double *frontRight, double *rearLeft, double *rearRight);

/** \brief Say if the flippers has reached their target angle.
 *  \return CAN error ID.
 *  \param[out] *reached Reached = 1, not reached = 0 */
int nrFlippersAngleReached(int *reached);

/** \brief Enable/disable differential brake. on = 1 to enable.
 *  \param[in] on Brake = 1, unbrake = 0.*/
int nrSetBrake(int on);

/** \brief Get the current brake state.
 *  \return CAN error ID.
 *  \param[out] *val Brake = 1, unbrake = 0. */
int nrGetBrake(int *val);

/** \brief Get the motor failure register (MF register) for all controllers (see controller.h for details).
 *  \return CAN error ID.
 *  \param[out] status[] Array of int to be filled in with controller status (SR register). */
int nrGetMotorsFailure(int status[]);

/** \brief Get the status register (SR register) for all controllers (see controller.h for details).
 *  \param[out] status[] Array of int to be filled in with controller status (SR register). */
int nrGetControllersStatus(int status[]);

/** \brief Get the error code register for all controllers (see controller.h for details).
 *  \return CAN error ID.
 *  \param[out] error[] Array of int to be filled in with controller error code (EC register). */
int nrGetControllersError(int error[]);

/** \brief Get the current differential angles [\a rad].
 *  \return CAN error ID.
 *  \param[out] *left Filed in with actual left track angle [\a rad].
 *  \param[out] *right Filed in with actual right track angle [\a rad].*/
int nrGetDifferentialAngles(double *left, double *right);

/** \brief Set the 3D laser scanning rate. Maximal rate is \a 1.25 [\a rad/s].
 *  \return CAN error ID.
 *  \param[in] val Rotation speed [\a rad].*/
int nrSetScanningSpeed(double val);

/** \brief Get the current scanning rate [\a rad/s].
 *  \return CAN error ID.
 *  \param[out] *val Filled in with actual rotation speed [\a rad/s]. */
int nrGetScanningSpeed(double *val);

/** \brief Get the current scanner angle in [\a -PI/2, PI/2].
 *  \return CAN error ID.
 *  \param[out] *val Get actual laser angle [\a rad]. */
int nrGetScannerAngle(double *val);

/** \brief Get the active current of the motors [\a A]. Use the object ID_xxx defined in types.h.
 *
 * ID_CORE; ID_TRACK_LEFT; ID_TRACK_RIGHT; ID_FLIPPER_FRONT_LEFT;
 * ID_FLIPPER_FRONT_RIGHT; ID_FLIPPER_REAR_LEFT; ID_FLIPPER_REAR_RIGHT; ID_CTRL_MAX
 *
 *  \return CAN error ID.
 *  \param[out] *iq Actual active current of the motors in [\a A].
 *  \param[in] object The objects id are defined in the types.h only. */
int nrReadDOFCurrent(double *iq, int object);

/** \brief Get the current battery level in \a % and state {\a BAT_OK, BAT_WARNING, BAT_CRITICAL}.
 *  \return CAN error ID.
 *  \param[out] *level Actual battery level in \a %.
 *  \param[out] *state Actual state {\a BAT_OK, BAT_WARNING, BAT_CRITICAL}. */
int nrGetBatteryLevel(double *level, int *state);

/** \brief Get the actual E-Stop state.
 *  \return CAN error ID.
 *  \param[out] *eStop Button state is Enable = 1, Disable = 0. */
int nrGetEstop(int *eStop);

/** \brief Actual initialisation state:
 *  \return 1 = initialised; 0 = NOT initialised */
int nrGetFlippersInitState();

/** \brief Tell the 3D laser scanner to go to the predefined middle position with an user offset.
 *   Max range is \a [-pi/4; pi/4] [\a rad/s].
 *  \return CAN error ID.
 *  \param[out] angleOffset Offset compared to initial middle position [\a rad].*/
int nrGoMiddlePos(double angleOffset);

#ifdef __cplusplus
}
#endif

#endif /* LIBROVER_H_ */
