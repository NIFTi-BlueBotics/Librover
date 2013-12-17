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



#include <stdio.h>
#include "types.h"
#include "rover.h"

static Rover *rover;

extern "C" void nrGetDefaultParams(RoverParams *params){
	rover->GetDefaultParams(params);
}

extern "C" void nrInit(const char *device, RoverParams *params, int bestInit){
	if(device)
		rover = new Rover(device, params, bestInit != 0);
	else
		rover = new Rover("/dev/usb/cpc_usb0", params, bestInit != 0);
}

extern "C" int nrInitFlippers(int bestInit){
	return rover->InitFlippers(bestInit);
}

extern "C" int nrRestart3D(){
	return rover->Restart3D();
}

extern "C" void nrGetRoverParams(RoverParams *params){
	rover->GetRoverParams(params);
}

extern "C" void nrDestroy(){
	delete rover;
}

int nrEnable(int enable){
	rover->EnableMotion(enable);
}

extern "C" int nrSetSpeed(double v, double w){
	return rover->SetSpeed(v, w);
}

extern "C" int nrGetSpeed(double *v, double *w){
	return rover->GetSpeed(v, w);
}

extern "C" int nrSetSpeedLR(double left, double right){
	return rover->SetSpeedLR(left, right);
}

extern "C" int nrGetSpeedLR(double *left, double *right){
	return rover->GetSpeedLR(left, right);
}

extern "C" int nrGetEncodersLR(int *left, int *right){
	return rover->GetEncodersLR(left, right);
}

extern "C" int nrSetFlippers(double frontLeft, double frontRight, double rearLeft, double rearRight){
	return rover->SetFlippersAngle(frontLeft, frontRight, rearLeft, rearRight);
}
extern "C" int nrSetFlippersTorque(double frontTorque, double rearTorque){
	return rover->SetFlippersTorque(frontTorque, rearTorque);
}

extern "C" int nrSetFlipper(double angle, int id){
	return rover->SetFlipperAngle(angle, id);
}

extern "C" int nrGetFlippers(double *frontLeft, double *frontRight, double *rearLeft, double *rearRight){
	return rover->ReadFlippersAngle(frontLeft, frontRight, rearLeft, rearRight);
}

extern "C" int nrFlippersAngleReached(int *reached){
    bool reach;
    int res = rover->FlippersAngleReached(&reach);
    *reached = (int) reach;
    return res;
}

extern "C" int nrSetBrake(int on){
	return rover->SetBrake(on);
}

extern "C" int nrGetBrake(int *val){
	return rover->GetBrake(val);
}

extern "C" int nrGetMotorsFailure(int status[]){
	return rover->GetMotorsFailure(status);
}

extern "C" int nrGetControllersStatus(int status[]){
	return rover->GetControllersStatus(status);
}

extern "C" int nrGetControllersError(int error[]){
	return rover->GetControllersError(error);
}

extern "C" int nrGetDifferentialAngles(double *left, double *right){
	return rover->GetDifferentialAngles(left, right);
}

extern "C" int nrSetScanningSpeed(double val){
	return rover->SetScanningSpeed(val);
}

extern "C" int nrGetScanningSpeed(double *val){
	return rover->GetScanningSpeed(val);
}

extern "C" int nrGetScannerAngle(double *val){
	return rover->GetScannerAngle(val);
}

extern "C" int nrReadDOFCurrent(double *iq, int object){
	return rover->ReadDOFCurrent(iq, object);
}

extern "C" int nrGetBatteryLevel(double *level, int *state){
	return rover->ReadBatteryLevel(level, state);
}

extern "C" int nrGetEstop(int *eStop){
	return rover->GetEstop(eStop);
}

extern "C" int nrGetFlippersInitState(void){
	return rover->GetFlippersInitState();
}

extern "C" int nrGoMiddlePos(double angleOffset){
	return rover->GoMiddlePos(angleOffset);
}

