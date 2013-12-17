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

#include "core.h"
#include <unistd.h>

Core::Core(int elmoId, const char *name, can_device_p device, bool inverted) : Controller(elmoId, name, device, inverted){
	WriteFloatObject(PDO2_RECEIVE, "cl", 1, 0.1);//permanent current limitations
	WriteFloatObject(PDO2_RECEIVE, "pl", 1, 0.2); //peak current limitations
	WriteIntObject(PDO2_RECEIVE, "pl", 2, 2); // peak duration
	WriteIntObject(PDO2_RECEIVE, "tr", 1, 2000);
	WriteIntObject(PDO2_RECEIVE, "er", 3, 10000);
}

int Core::Enable(bool enable){
	int e;

	if (enable) e = WriteIntObject(PDO2_RECEIVE, "mo", 0, 1);
	else e = WriteIntObject(PDO2_RECEIVE, "mo", 0, 0);
	usleep(150000); // To make sure the controller has executed the code
	return e;
}

int Core::ReadBatteryVoltage(double *val){
	int e = ReadAnalogInput(val, 4);
	*val = (*val - analogOffset) * analogGain + CALIB_LOW_VOLTAGE;
	return e;
}

int Core::ReadBatteryLevel(double *level, int *state){
	double voltage;
	int e = ReadBatteryVoltage(&voltage);
	if(voltage < BAT_CRITICAL_VOLT)
		*state = BAT_CRITICAL;
	else if(voltage < BAT_WARNING_VOLT)
		*state = BAT_WARNING;
	else
		*state = BAT_OK;

	*level = (voltage - BAT_CRITICAL_VOLT) / (BAT_FULL_VOLTAGE - BAT_CRITICAL_VOLT) * 100.0;
	*level = *level < 0.0 ? 0.0: MIN(*level, 100.0);
	return e;
}

int Core::Restart(){
	int it;
	WriteIntObject(PDO2_RECEIVE, "mo", 0, 1);
	WriteIntObject(PDO2_RECEIVE, "jv", 0, 10000);
	ReadIntObject(PDO2_RECEIVE, "bg", 0, &it);
	return WriteIntObject(PDO2_RECEIVE, "ui", 5, true);
}

int Core::SetBrake(int on){
	return WriteIntObject(PDO2_RECEIVE, "ui", 3, on);
}

// 0 = Brake is released; 1 = 1 Brake is locked
int Core::ReadBrakeState(int *val){
	return ReadIntObject(PDO2_RECEIVE, "ui", 3, val);
}

// Speed in rad/s
int Core::SetScanningSpeed(double speed){
	return WriteFloatObject(PDO2_RECEIVE, "uf", 1, speed);
}

// return speed in rad/s
int Core::ReadScanningSpeed(double *val){
	float flt;
	int e = ReadFloatObject(PDO2_RECEIVE, "uf", 1, &flt);
	*val = flt;
	return e;
}

// return the laser angle in radian
int Core::ReadScannerAngle(double *val){
	int enc;
	int e = ReadEncoder(&enc);
	*val = (TWO_PI / COUNT_PER_REV_CORE / GEAR_3D) * (enc + MIDDLE_COUNTER);
	return e;
}

// Go to the init position with an offset
int Core::GoMiddlePos(double angleOffset){
	int e = WriteFloatObject(PDO2_RECEIVE, "uf", 3, angleOffset); // Write angleOffset into user float#3
	e = e || WriteIntObject(PDO2_RECEIVE, "ui", 4, 1); // Set user integer #4 to 1
	return e;
}


