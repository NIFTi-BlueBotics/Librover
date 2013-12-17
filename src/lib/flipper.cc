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

#include <unistd.h>
#include "flipper.h"

Flipper::Flipper(int elmoId, const char *name, can_device_p device, bool isFrontF, bool inverted) : Controller(elmoId, name, device, inverted){
	isFront = isFrontF;
	SetMode(TORQUE_MODE);
	InitEncoderWithAnalogAngle();
}

bool Flipper::IsFront(){
	return isFront;
}

// Angle in Radian, speed in rad/s
int Flipper::SetAngle(double angle, double omega){
	int speedInt, position, e;

	speedInt = (int)(FLIPPER_CNT_OUTPUT / TWO_PI * omega); // SP is in cnt/s
	position = (int)(FLIPPER_CNT_OUTPUT / TWO_PI * angle);
	if(invert){
		speedInt = -speedInt;
		position = -position;
	}
	if(e = WriteIntObject(PDO2_RECEIVE, "sp", 0, speedInt)) return e;
	if(e = WriteIntObject(PDO2_RECEIVE, "pa", 0, position)) return e;
	int it;
	return ReadIntObject(PDO2_RECEIVE, "bg", 0, &it);
}

int Flipper::SetAngleAnalog(double angle, double omega){
	const double radius = DEG2RAD(1.0);
	double a;
	int e, speedInt, actualMode;
	actualMode = elmoMode;
	if(e = ReadAnalogAngle(&a)) return e;
	if(fabs(angle - a) < radius) return e;

	speedInt = (int)(FLIPPER_CNT_OUTPUT / TWO_PI * omega);
	if(angle - a < 0) speedInt = -speedInt;
	if(e = SetMode(VELOCITY_MODE)) return e;
	if(e = StartJog(speedInt)) return e;

	bool reach = false;
	while(!reach) {
		if(e = ReadAnalogAngle(&a)) return Enable(false);
		bool overshoot = (a > angle && speedInt > 0) || (a < angle && speedInt < 0);
		if(fabs(angle - a) < radius || overshoot) {
			if(e = Enable(false)) return e;
			reach = true;
		}
	}
	return SetMode(actualMode);
}

int Flipper::SetTorque(double flipperTorque){
	int e;
	if (elmoMode == TORQUE_MODE){
		flipperTorque = flipperTorque * (flipperTorque < 0 ? -1.0 : 1.0);
		if ((flipperTorque >TORQUE_MAX) || (flipperTorque >TORQUE_THRES_H)) flipperTorque = 2*TORQUE_MAX;
		if ((flipperTorque >TORQUE_THRES_L) && (flipperTorque < TORQUE_THRES_H)) flipperTorque = TORQUE_THRES_L;
		if(e = WriteFloatObject(PDO2_RECEIVE, "cl", 1, TORQUE_MAX)) return e; // maximum allowed continuous motor phase current
		if(e = WriteFloatObject(PDO2_RECEIVE, "pl", 1, (float)(flipperTorque))) return e; // peak current limitations
	}
	else e = SD_FLIPPERMODE; // return flipper mode error
	return e;
}

int Flipper::ReadAngle(double *val){
	int e, enc;
	if(e = ReadEncoder(&enc)) return e;
	*val = FLIPPER_RAD_PER_PULSE * enc * (invert ? -1.0 : 1.0);
	return e;
}

int Flipper::ReadAnalogAngle(double *val, int samples){
	int e;
	if(e = ReadAnalogInput(val, samples)) return e;
	//printf("%s input %f\n", GetName(), *val);
	*val = (*val - analogOffset) * analogGain;
	return e;
}

int Flipper::InitEncoderWithAnalogAngle(){
	double angle;
	int e;
	if(e = ReadAnalogAngle(&angle, CALIB_SAMPLES)) return e;
	SetEncoder((int)(angle / FLIPPER_RAD_PER_PULSE * (invert ? -1.0 : 1.0)));
	return e;
}

bool Flipper::InInterferenceZone(double angle){
	angle = ToMinPlusPi(angle);
	if(isFront) return angle > fabs(INTERF_ANGLE_MIN) || angle < -INTERF_ANGLE_POS;
	else return angle > INTERF_ANGLE_POS || angle < INTERF_ANGLE_MIN;
}

// Wrap an angle in [0, 2 * pi[ interval
double Flipper::ToZero2Pi(double value){
    int mod = (int)(value / TWO_PI);
    double ret = value - mod * TWO_PI;
    if(fabs(ret) == TWO_PI)
        return 0.0;
    if(ret < 0)
        return TWO_PI + ret;
    return ret;
}

// Wrap an angle in ]-pi, pi] interval
double Flipper::ToMinPlusPi(double value){
    double ret = ToZero2Pi(value);
    if(ret > M_PI)
        return - (2.0 * M_PI - ret);

    return ret;
}



