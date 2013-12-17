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


#include "track.h"


Track::Track(int elmoId, const char *name, can_device_p device, double wRadius, bool inverted) : Controller(elmoId, name, device, inverted){
	SetMode(VELOCITY_MODE);
	WriteFloatObject(PDO2_RECEIVE, "kp", 2, TRACK_DFLT_GAIN);
	radius = wRadius;
}

int Track::SetSpeed(double speed){
	int speedInt, e, it;
	speedInt = (int)(speed / TWO_PI / radius * TRACK_CNT_OUTPUT); // SP is in cnt/s
	if(e = WriteIntObject(PDO2_RECEIVE, "jv", 0, speedInt)) return e;
//	printf("Set speed: %d\n", speedInt);
	return ReadIntObject(PDO2_RECEIVE, "bg", 0, &it);
}

int Track::ReadSpeed(double *val){
	int vx;
	int e = ReadIntObject(PDO2_RECEIVE, "vx", 0, &vx);
	*val = vx * TWO_PI * radius / TRACK_CNT_OUTPUT;// vx is in cnt/s
//	printf("Read speed: %d\n", vx);
	return e;
}

int Track::ReadAngle(double *angle, int samples){
	double input;
	int e = ReadAnalogInput(&input, samples);
	*angle = (input - analogOffset) * analogGain;
	return e;
}
