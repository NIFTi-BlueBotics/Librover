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

#ifndef TRACK_H_
#define TRACK_H_

#include "controller.h"

/* define const max speed here */
#define TRACK_COUNT_PER_REV			60
#define TRACK_GEAR					100
#define TRACK_CNT_OUTPUT			(TRACK_COUNT_PER_REV * TRACK_GEAR)
#define TRACK_RAD_PER_PULSE			(2.0 * M_PI / 60.0 / 100.0)
#define TRACK_DFLT_GAIN				2000.0f

class Track : public Controller {

	public:
		Track(int elmo_id, const char *name, can_device_p device, double wRadius, bool inverted = false);
		int SetSpeed(double speed);
		int ReadSpeed(double *val);
		int ReadAngle(double *angle, int samples = -1);

	private:
		double radius;
};

#endif /* TRACK_H_ */
