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

#ifndef CORE_H_
#define CORE_H_

#include "controller.h"


#define COUNT_PER_REV_CORE 			512
#define GEAR_3D						516
#define MIDDLE_COUNTER				123381
#define BRAKE_LOCK					1
#define BRAKE_UNLOCK				0
#define SCALE_PER_VOLT_DEFAULT		0.05
#define BAT_FULL_VOLTAGE			58.0
#define BAT_LOW_VOLTAGE				48.0
#define BAT_WARNING_VOLT			47.0
#define BAT_CRITICAL_VOLT			38.0


class Core : public Controller {

	public:
		Core(int elmoId, const char *name, can_device_p device, bool inverted = false);
		int Enable(bool enable);
		int ReadBatteryVoltage(double *val);
		int ReadBatteryLevel(double *level, int *state);
		int SetBrake(int on);
		int ReadBrakeState(int *val);
		int Restart();
		int SetScanningSpeed(double speed); // final output rad/s
		int ReadScanningSpeed(double *val);
		int ReadScannerAngle(double *val);
		int GoMiddlePos(double angleOffset);
};

#endif /* CORE_H_ */
