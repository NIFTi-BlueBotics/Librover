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

#ifndef FLIPPER_H_
#define FLIPPER_H_

#include "controller.h"


#define FLIPPER_COUNT_PER_REV	60.0f
#define FLIPPER_GEAR			100.0f
#define FLIPPER_CNT_OUTPUT		(FLIPPER_COUNT_PER_REV * FLIPPER_GEAR)
#define FLIPPER_DIAMETER		0.18f
#define FLIPPER_RAD_PER_PULSE	(2.0 * M_PI / FLIPPER_COUNT_PER_REV / FLIPPER_GEAR)
#define INTERF_ANGLE_POS		(M_PI / 2.0)
#define INTERF_ANGLE_MIN		(-M_PI / 2.0)
//#define POS_TARGET_RADIUS			(DEG2RAD(4.0) / FLIPPER_RAD_PER_PULSE)


class Flipper : public Controller{

	public:
		Flipper(int elmo_id, const char *name, can_device_p device, bool isFrontF, bool inverted = false);
		bool IsFront();
		int SetAngle(double angle, double omega = M_PI / 4);
		int SetAngleAnalog(double angle, double omega = M_PI / 8);
		int SetTorque(double flipperTorque); // Torque in [A]
		int ReadAngle(double *val);
		int ReadAnalogAngle(double *val, int samples = -1);
		int InitEncoderWithAnalogAngle();
		bool InInterferenceZone(double angle);

	private:
		bool isFront;
		double ToZero2Pi(double value);
		double ToMinPlusPi(double value);

};

#endif /* FLIPPER_H_ */
