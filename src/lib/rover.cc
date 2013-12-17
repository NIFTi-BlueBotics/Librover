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

#include "rover.h"
#include <string.h>

const double Rover::calibAngle = 3 * M_PI / 4.0;

typedef struct {
	int id;
	bool interf;
	double angle;
} SortFlip;

int flipCompare(const void *a, const void *b) {
	double va = fabs(((SortFlip *)a)->angle);
	double vb = fabs(((SortFlip *)b)->angle);
	double diff = va - vb;
	if(diff > 0) return 1;
	if(diff < 0) return -1;
	return 0;
}

void Rover::GetDefaultParams(RoverParams *params){
	params->trackDistance = 0.397;
	params->trackWheelRadius = 0.089;
	params->trackLength = 0.5;
	params->trackWidth = 0.097;
	params->flipperLength = 0.34;
	params->flipperWidth = 0.05;
	params->flipperOffset = 0.193731547;
	params->laserX = 0.2502;
	params->laserY = 0.0;
	params->laserZ = 0.1427;
	params->imuX = 0;
	params->imuY = 0;
	params->imuZ = 0.1694;
	params->omniX = 0.070;
	params->omniY = 0.068;
	params->omniZ = 0.2707;
	params->omniAngleOffset = 0.942477796;
	params->referentialX = 0.0;
	params->referentialY = 0.0;
	params->referentialZ = 0.0705;
	params->vMax = 0.6;
	params->wMax = 2.0 * params->vMax / params->trackDistance;
}

Rover::Rover(const char* canDevPath, RoverParams *params, bool bestInit) {
	if(params == NULL) GetDefaultParams(&roverParams);
	else memcpy(&roverParams, params, sizeof(RoverParams));
	InitCan(canDevPath);

	core = new Core(1, "core", &canDev);
	trackRight = new Track(3, "track right", &canDev, roverParams.trackWheelRadius);
	trackLeft = new Track(6, "track left", &canDev, roverParams.trackWheelRadius);
	flipperRearRight = new Flipper(2, "rear right", &canDev, false);
	flipperFrontRight = new Flipper(4, "front right", &canDev, true);
	flipperRearLeft = new Flipper(5, "rear left", &canDev, false, true);
	flipperFrontLeft = new Flipper(7, "front left", &canDev, true, true);

	dof[ID_CORE] = core;
	dof[ID_FLIPPER_FRONT_LEFT] = flipperFrontLeft;
	dof[ID_TRACK_LEFT] = trackLeft;
	dof[ID_FLIPPER_REAR_LEFT] = flipperRearLeft;
	dof[ID_FLIPPER_REAR_RIGHT] = flipperRearRight;
	dof[ID_TRACK_RIGHT] = trackRight;
	dof[ID_FLIPPER_FRONT_RIGHT] = flipperFrontRight;

	track[0] = trackLeft;
	track[1] = trackRight;

	flipper[0] = flipperFrontLeft;
	flipper[1] = flipperFrontRight;
	flipper[2] = flipperRearLeft;
	flipper[3] = flipperRearRight;

	isFlippersInit = InitFlippers(bestInit);
}

Rover::~Rover(){
	for(int i = 0; i < ID_CTRL_MAX; i++)
		delete dof[i];

	can_close(&canDev);
	can_destroy(&canDev);
}

bool Rover::InitFlippers(bool bestInit){
	// Unfold chassis with interference management
	int eStop = 0;
	GetEstop(&eStop);
	if(eStop){
		isFlippersInit = false;
		return false;
	}
	else{
		if(bestInit){
			int inter[4];
			double angles[4];
			GetFlipperInterference(inter, true, angles);
			SortFlip list[4];
			for(int i = 0; i < 4; i++){
				list[i].id = i;
				list[i].angle = angles[i];
				list[i].interf = inter[i];
			}
			qsort(list, 4, sizeof(SortFlip), flipCompare);

			for(int i = 0; i < 4; i++){
				Flipper *flip = flipper[list[i].id];
				if(list[i].interf) {
					flip->SetAngleAnalog(flip->IsFront() ? -calibAngle : calibAngle);
				}
			}
			SetAnalogFlippersAngle(-calibAngle, -calibAngle, calibAngle, calibAngle);
			for(int i = 0; i < 4; i++)
				flipper[i]->InitEncoderWithAnalogAngle();
		}
		isFlippersInit = true;
		return true;
	}
}

int Rover::Restart3D(){
	return core->Restart();
}

int Rover::EnableMotion(int enable){
	for(int i = 0; i < 4; i++)
		flipper[i]->Enable(enable);

	for(int i = 0; i < 2; i++)
		track[i]->Enable(enable);
}

Controller* Rover::GetController(int idx){
	return dof[idx];
}

void Rover::GetRoverParams(RoverParams *params){
	memcpy(params, &roverParams, sizeof(RoverParams));
}

int Rover::EnableFlippers(bool enable){
	int e;
	for(int i = 0; i < 4; i++)
		if(e = flipper[i]->Enable(enable)) return e;
	return e;
}

int Rover::SetFlippersAngle(double frontLeft, double frontRight, double rearLeft, double rearRight){
	int e;
	if(e = flipperFrontLeft->SetAngle(frontLeft)) return e;
	if(e = flipperFrontRight->SetAngle(frontRight)) return e;
	if(e = flipperRearLeft->SetAngle(rearLeft)) return e;
	return flipperRearRight->SetAngle(rearRight);
}

int Rover::SetFlippersTorque(double frontTorque, double rearTorque){
	int e;
	if(e = flipperFrontLeft->SetTorque(frontTorque)) return e;
	if(e = flipperFrontRight->SetTorque(frontTorque)) return e;
	if(e = flipperRearLeft->SetTorque(rearTorque)) return e;
	return flipperRearRight->SetTorque(rearTorque);

}

int Rover::SetFlipperAngle(double angle, int id){

	switch(id){
	case ID_FLIPPER_FRONT_LEFT:
		return flipperFrontLeft->SetAngle(angle);

	case ID_FLIPPER_FRONT_RIGHT:
		return flipperFrontRight->SetAngle(angle);

	case ID_FLIPPER_REAR_LEFT:
		return flipperRearLeft->SetAngle(angle);

	case ID_FLIPPER_REAR_RIGHT:
		return flipperRearRight->SetAngle(angle);

	default:
		DEBUG_PRINT(("ID not found. Cannot SetFlippersAngle\n"));
		return CAN_ERROR_SETUP;
	}
}

int Rover::ReadFlippersAngle(double *frontLeft, double *frontRight, double *rearLeft, double *rearRight){
	int e;
	if(e = flipperFrontLeft->ReadAngle(frontLeft)) return e;
	if(e = flipperFrontRight->ReadAngle(frontRight)) return e;
	if(e = flipperRearLeft->ReadAngle(rearLeft)) return e;
	return flipperRearRight->ReadAngle(rearRight);
}

int Rover::ReadFlippersEncoder(int *frontLeft, int*frontRight,  int *rearLeft, int *rearRight){
	int e;
	if(e = flipperFrontLeft->ReadEncoder(frontLeft)) return e;
	if(e = flipperFrontRight->ReadEncoder(frontRight)) return e;
	if(e = flipperRearLeft->ReadEncoder(rearLeft)) return e;
	return flipperRearRight->ReadEncoder(rearRight);

}
int Rover::ReadAnalogFlippersAngle(double *frontLeft, double *frontRight,  double *rearLeft, double *rearRight){
	int e;
	if(e = flipperFrontLeft->ReadAnalogAngle(frontLeft)) return e;
	if(e = flipperFrontRight->ReadAnalogAngle(frontRight)) return e;
	if(e = flipperRearLeft->ReadAnalogAngle(rearLeft)) return e;
	return flipperRearRight->ReadAnalogAngle(rearRight);
}

int Rover::FlippersAngleReached(bool *reached){
	int all = 0, status, e;
	*reached = false;
	for(int i = 0; i < 4; i++){
		if(e = flipper[i]->ReadMotionStatus(&status)) return e;
		all += status;
	}
	*reached = (all == 0);
	return e;
}

int Rover::WaitFlippers(){
	bool reached = false;
	int e;
	while(!reached){
		if(e = FlippersAngleReached(&reached)) return e;
		sleep(0.05);
	}
	sleep(0.2);
	return e;
}

int Rover::SetSpeed(double v, double omega){
	int e;
	double vl = (2 * v - roverParams.trackDistance * omega) / 2.0;
	if(e = trackLeft->SetSpeed(vl)) return e;
	return trackRight->SetSpeed(2 * v - vl);
}

int Rover::GetSpeed(double *v, double *omega){
	int e;
	double vl, vr;

	if(e = GetSpeedLR(&vl, &vr)) return e;
	*v = (vl + vr) / 2.0;
	*omega = (vr - vl) / roverParams.trackDistance;
	return e;
}

// speed in m/s
int Rover::SetSpeedLR(double left, double right){
	int e;
	if(e = trackLeft->SetSpeed(left)) return e;
	return trackRight->SetSpeed(right);
}

int Rover::GetSpeedLR(double *left, double *right){
	int e;
	if(e = trackLeft->ReadSpeed(left)) return e;
	return trackRight->ReadSpeed(right);
}

int Rover::GetEncodersLR(int *left, int *right){
	int e;
	if(e = trackLeft->ReadEncoder(left)) return e;
	return trackRight->ReadEncoder(right);
}

// return in rad/s
int Rover::SetScanningSpeed(double radPerSec){
	return core->SetScanningSpeed(radPerSec);
}

// return in rad/s
int Rover::GetScanningSpeed(double *val){
	return core->ReadScanningSpeed(val);
}

int Rover::GetScannerAngle(double *val){
	return core->ReadScannerAngle(val);
}
// Estop is connected to tracks and flippers in parallele
int Rover::GetEstop(int *eStop){
	return trackLeft->ReadEstop(eStop);
}
// Make the laser scanner to the middle position
int Rover::GoMiddlePos(double angleOffset){
	return core->GoMiddlePos(angleOffset);
}
// return 0-100%
int Rover::ReadBatteryLevel(double *level, int *state){
	return core->ReadBatteryLevel(level, state);
}

int Rover::ReadDOFCurrent(double *iq, int object){
	if (object >= ID_CTRL_MAX){
		DEBUG_PRINT(("ID not found. Cannot ReadDOFCurrent\n"));
		return CAN_ERROR_SETUP;
	}
	return dof[object]->ReadActiveCurrent(iq);
}

int Rover::ReadBatteryVoltage(double *val){
	return core->ReadBatteryVoltage(val);
}

int Rover::GetDifferentialAngles(double *left, double *right){
	int e;
	if(e = trackLeft->ReadAngle(left)) return e;
	return trackRight->ReadAngle(right);
}

int Rover::SetBrake(int on){ // Status 1 = lock, 0 = unlock
	return core->SetBrake(on);
}

int Rover::GetBrake(int *val){
	return core->ReadBrakeState(val);
}

// Refer to SimpleIQ Command Reference manual (MAN-SIMCR) p.3-135
int Rover::GetMotorsFailure(int status[]){
	int e;
	for(int i = 0; i < ID_CTRL_MAX; i++)
		if(e = dof[i]->ReadMotorFailure(&status[i])) return e;

	return e;
}

int Rover::GetControllersStatus(int status[]){
	int e;
	for(int i = 0; i < ID_CTRL_MAX; i++)
		if(e = dof[i]->ReadStatusRegister(&status[i])) return e;

	return e;
}

int Rover::GetControllersError(int error[]){
	int e;
	for(int i = 0; i < ID_CTRL_MAX; i++)
		if(e = dof[i]->ReadErrorCodeRegister(&error[i])) return e;

	return e;
}

// WARNING: Avoid using this procedure frequently. Flash could be destroyed!!!
int Rover::CalibrateChassis(){
	int e, samples = CALIB_SAMPLES;
	double offset, input, angle, inLeft, inRight;
	int encoder, error = 0;

	for(int i = 0; i < 4; i++){
		if(e = flipper[i]->ReadAnalogInput(&input, samples)){
			DEBUG_PRINT(("ERROR %d reading analog input of flipper[%d] \n", e, i));
			return e;
		}
		if(e = flipper[i]->SetAnalogParams(input, RAD_PER_VOLT)){
			DEBUG_PRINT(("ERROR %d setting analog params of flipper[%d] \n", e, i));
			return e;
		}
		if(e = flipper[i]->ClearEncoder()){
			DEBUG_PRINT(("ERROR %d clearing encoder of flipper[%d] \n", e, i));
			return e;
		}
	}
	// Read angles when fully deployed
	if(e = trackLeft->ReadAnalogInput(&inLeft, samples)) return e;
	if(e = trackRight->ReadAnalogInput(&inRight, samples)) return e;
	for(int i = 0; i < 4; i++){
		if(e = flipper[i]->SetAngle(flipper[i]->IsFront() ? -calibAngle : calibAngle)) return e;
	}
	if(e = WaitFlippers()) return e;

	double meanGain = 0;
	for(int i = 0; i < 4; i++) {
		offset = flipper[i]->GetAnalogOffset();
		if(e = flipper[i]->ReadEncoder(&encoder)) return e;
		if(e = flipper[i]->ReadAnalogInput(&input, samples)) return e;
		if(e = flipper[i]->ReadAngle(&angle)) return e;
		if(e = flipper[i]->SetAnalogParams(offset, angle / (input - offset), true)) return e;
		if(e = flipper[i]->SetEncoder(encoder)) return e;
		meanGain += fabs(flipper[i]->GetAnalogGain());
	}
	meanGain /= 4;

	// Calibrate tracks using meanGain of flippers and average analog input
	if(e = trackLeft->ReadAnalogInput(&input, samples)) return e;
	if(e = trackLeft->SetAnalogParams((input + inLeft) / 2, meanGain, true)) return e;
	if(e = trackRight->ReadAnalogInput(&input, samples)) return e;
	if(e = trackRight->SetAnalogParams((input + inRight) / 2, -meanGain, true)) return e;
	return e;
}

int Rover::SetVoltageCalibration(double offset, double gain){
	return core->SetAnalogParams(offset, gain, true);
}

int Rover::GetFlipperInterference(int flipInter[], bool analogRead, double angles[]){
	double angle;
	int e;
	for(int i = 0; i < 4; i++){
		if(analogRead) e = flipper[i]->ReadAnalogAngle(&angle, 3);
		else e = flipper[i]->ReadAngle(&angle);
		if(e) return e;
		/* FIXME: wrap angle in -pi;pi range*/
		flipInter[i] = flipper[i]->InInterferenceZone(angle);
		if(angles != NULL) angles[i] = angle;
	}

	return e;
}
bool Rover::GetFlippersInitState(){
	return isFlippersInit;
}


int Rover::SetAnalogFlippersAngle(double frontLeft, double frontRight,  double rearLeft, double rearRight){
	const double radius = DEG2RAD(1.0);
	const double omega = M_PI / 8;
	double target[4], reached[4], speed[4] = {0, 0, 0, 0};
	int e, speedInt, actualMode;
	double a;

	actualMode = flipper[0]->elmoMode;
	target[0] = frontLeft; target[1] = frontRight; target[2] = rearLeft; target[3] = rearRight;

	for(int i = 0; i < 4; i++){
		if(e = flipper[i]->ReadAnalogAngle(&a)) return e;
		reached[i] = fabs(target[i] - a) < radius;
		if(!reached[i]){
			speedInt = (int)(FLIPPER_CNT_OUTPUT / TWO_PI * omega);
			if(target[i] - a < 0) speedInt = -speedInt;
			speed[i] = speedInt;
			if(e = flipper[i]->SetMode(VELOCITY_MODE)) return e;
			if(e = flipper[i]->StartJog(speedInt)) return e;
		}
	}

	int reach = 0;
	while(reach < 4) {
		reach = 0;
		for(int i = 0; i < 4; i++){
			if(e = flipper[i]->ReadAnalogAngle(&a))
				return flipper[i]->Enable(false);

			if(!reached[i]){
				bool overshoot = (a > target[i] && speed[i] > 0) || (a < target[i] && speed[i] < 0);
				if(fabs(target[i] - a) < radius || overshoot) {
					if(e = flipper[i]->Enable(false)) return e;
					fflush(stdout);
					reached[i] = true;
					reach++;
				}
			} else reach++;
		}
	}

	for(int i = 0; i < 4; i++)
		if(e = flipper[i]->SetMode(actualMode)) return e;

	return CAN_ERROR_NONE;
}

int Rover::InitCan(const char* canDevPath){
	int e;
	can_message_t canMsg, canMsgRcv;
	param_t canParams[] = {
			  {CAN_CPC_PARAMETER_DEVICE, ""},
			  {CAN_CPC_PARAMETER_BITRATE, "500"},
			  {CAN_CPC_PARAMETER_QUANTA_PER_BIT, "8"},
			  {CAN_CPC_PARAMETER_SAMPLING_POINT, "0.75"},
			  {CAN_CPC_PARAMETER_TIMEOUT, "0.01"},
			};

	memcpy(&canParams[0].value, canDevPath, strlen(canDevPath) + 1);
	config_t canConfig = {
				canParams,
				sizeof(canParams)/sizeof(param_t),
			};
	can_init(&canDev, &canConfig);
	// Opening CAN
	if (can_open(&canDev)) {
		DEBUG_PRINT(("Open_can_device: open FAILED\n"));
		exit(1);
	}

	canMsg.length=4;
	canMsg.id= 0;
	canMsg.content[0]=1; //Set Elmo to remote access
	canMsg.content[1]= 0; //message sent to all Elmos
	canMsg.content[2] = 0;
	canMsg.content[3] = 0;

	if(e = can_send_message(&canDev, &canMsg)){
		DEBUG_PRINT(("FAIL to initialize remote access; Send error code: %x\n", e));
		return e;
	}
	while(1){
		if(e = can_receive_message(&canDev, &canMsgRcv)){
			DEBUG_PRINT(("No msg left in buffer!\n"));
			return e;
		}
	}
}


