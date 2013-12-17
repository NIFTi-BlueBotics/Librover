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

#include "rover.h"
#include "librover.h"

void printUsage(){
	puts("");
	puts("Usage: test <option> [device]");
	puts("");
	puts("");
}

void PrintStatus(int status){
	printf("Raw value: %X\n", status);
	printf("Error: %i\n", SR_GET_ERROR(status));
	printf("Servo drive status: %i\n", SR_GET_SERVO_DRIVE_STATUS(status));
	printf("Motor on: %i\n", SR_GET_MOTOR_ON(status));
}

int main (int argc, char **argv)
{
	int state = -1; int i;
	int enc[4];
	double angleFL,angleFR, angleRL, angleRR;
	if (argc < 2 || argc > 3 || !strcmp("--help", argv[1])) {
		printUsage();
		return 1;
	}

	Rover *rover;
	if(argc > 2)
		rover = new Rover(argv[2], NULL);
	else
		rover = new Rover("/dev/usb/cpc_usb0", NULL, false);

	if(!strcmp("--state", argv[1])){
		double level;
		rover->ReadBatteryLevel(&level, &state);
		printf("-- Rover state\n");
		double voltage;
		rover->ReadBatteryVoltage(&voltage);
		printf("Battery voltage: %.2fV\n", voltage);
		printf("Battery level: %.1f%% (%s)\n", level, state == BAT_OK ? "OK": (state == BAT_WARNING ? "WARNING": "CRITICAL"));
		int eStop;
		rover->GetEstop(&eStop);
		printf("Emergency Stop button %s\n", eStop == 1 ? "Pushed." : "Released.");
		double angles[4];
		rover->ReadFlippersAngle(&angles[0], &angles[1], &angles[2], &angles[3]);
		printf("Flipper angles\n front-left: %.3f\n front-right: %.3f\n rear-left: %.3f\n rear-right: %.3f\n", \
				angles[0], angles[1], angles[2], angles[3]);

		double left, right;
		rover->GetDifferentialAngles(&left, &right);
		printf("Differential angles\n left: %.3f (%.3f deg)\n right: %.3f (%.3f deg)\n",\
				left, RAD2DEG(left), right, RAD2DEG(right));
		double angle;
		rover->GetScannerAngle(&angle);
		printf("Scanner angle: %.3f\n", angle);
		int leftEnc, rightEnc;
		rover->GetEncodersLR(&leftEnc, &rightEnc);
		printf("Track encoders\n left: %i\n right: %i\n", leftEnc, rightEnc);
		int state;
		rover->GetBrake(&state);
		printf("Brake status: %s\n", state ? "ON":"OFF");
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);

		int inter[4];
		rover->GetFlipperInterference(inter, true, angles);
		for(int i = 0; i < 4; i++) {
			Controller *ctrl = rover->GetController(ID_FLIPPER_FRONT_LEFT + i);
			printf("Flipper %i interference: %i (angle=%.3f) mode: %i\n", i, inter[i], angles[i], ctrl->elmoMode);
		}
		Controller *track = rover->GetController(ID_TRACK_LEFT);
		float gain;
		track->ReadRegulatorVelocityGain(&gain);
		printf("Left track gain: %f\n", gain);

		Controller *flipper = rover->GetController(ID_FLIPPER_FRONT_RIGHT);
		flipper->ReadRegulatorPositionGain(&gain);
		printf("Front right flipper: %f\n", gain);
		int filter;
		flipper->ReadMainEncoderFilter(&filter);
		printf("Main encoder filter: %i\n", filter);
		flipper->ReadAuxEncoderFilter(&filter);
		printf("Aux encoder filter: %i\n", filter);


		double current;
		Controller *core = rover->GetController(ID_CORE);
		core->ReadActiveCurrent(&current);
		printf("Core active current: %.3f\n", current);
	}
	else if(!strcmp("--restart3D", argv[1])){
		rover->Restart3D();
	}
	else if(!strcmp("--scan-midP45", argv[1])){
		rover->GoMiddlePos(M_PI / 4);
	}
	else if(!strcmp("--scan-midM45", argv[1])){
		rover->GoMiddlePos(-M_PI / 4);
	}
	else if(!strcmp("--scan-mid0", argv[1])){
		rover->GoMiddlePos(0);
	}
	else if(!strcmp("--scan-on", argv[1])){
		rover->SetScanningSpeed(M_PI / 4);
	}
	else if(!strcmp("--scan-off", argv[1])){
		rover->SetScanningSpeed(0.0);
	}
	else if(!strcmp("--brake-on", argv[1])){
		rover->SetBrake(1);
	}
	else if(!strcmp("--brake-off", argv[1])){
		rover->SetBrake(0);
	}
	else if(!strcmp("--initFlippers", argv[1])){
		rover->InitFlippers(true);
	}
	else if(!strcmp("--flippersTorque2A", argv[1])){
		rover->SetFlippersTorque(2.0, 2.0);
	}
	else if(!strcmp("--flat", argv[1])){
		for(i = 0; i < 4; i++){
			rover->SetFlipperAngle(0, ID_FLIPPER_FRONT_LEFT + i);
		}
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
	}
	else if(!strcmp("--90", argv[1])){
		for(i = 0; i < 4; i++){
			if(i < 2) rover->SetFlipperAngle(-M_PI / 2, ID_FLIPPER_FRONT_LEFT + i);
			else  rover->SetFlipperAngle(M_PI / 2, ID_FLIPPER_FRONT_LEFT + i);
		}
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
	}
	else if(!strcmp("--45", argv[1])){
		for(i = 0; i < 4; i++){
			if(i < 2) rover->SetFlipperAngle(-M_PI / 4, ID_FLIPPER_FRONT_LEFT + i);
			else  rover->SetFlipperAngle(M_PI / 4, ID_FLIPPER_FRONT_LEFT + i);
		}
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
	}
	else if(!strcmp("--180", argv[1])){
		rover->SetFlipperAngle(-M_PI, ID_FLIPPER_FRONT_LEFT);
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
	}
	else if(!strcmp("--300", argv[1])){
		rover->SetFlipperAngle(-2 * M_PI + M_PI / 6, ID_FLIPPER_FRONT_LEFT);
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
		}
	else if(!strcmp("--360", argv[1])){
		rover->SetFlipperAngle(-2 * M_PI, ID_FLIPPER_FRONT_LEFT);
		rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
		rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
		printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
		printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
	}
	else if(!strcmp("--720", argv[1])){
		rover->SetFlipperAngle(-4 * M_PI, ID_FLIPPER_FRONT_LEFT);
		for (i =0; i < 4000; i++){
			rover->ReadFlippersEncoder(&enc[0], &enc[1], &enc[2], &enc[3]);
			rover->ReadFlippersAngle(&angleFL,&angleFR, &angleRL, &angleRR);
			printf("Flippers angle: %f, %f, %f, %f\n", angleFL,angleFR, angleRL, angleRR);
			printf("Flippers enc: %d, %d, %d, %d\n", enc[0],enc[1], enc[2], enc[3]);
		}
	}
	else if(!strcmp("--enable-flippers", argv[1])){
		rover->EnableFlippers(true);
	}
	else if(!strcmp("--disable-flippers", argv[1])){
		rover->EnableFlippers(false);
	}
	else if(!strcmp("--enable", argv[1])){
		rover->EnableMotion(true);
	}
	else if(!strcmp("--disable", argv[1])){
		rover->EnableMotion(false);
	}
	else if(!strcmp("--ctrl-status", argv[1])){
		int status[ID_CTRL_MAX];
		rover->GetControllersStatus(status);
		for(int i = 0; i < ID_CTRL_MAX; i++){
			printf("-- Status %s\n", CTRL_NAMES[i]);
			PrintStatus(status[i]);
		}
	}
	else if(!strcmp("--ctrl-errors", argv[1])){
		int errors[ID_CTRL_MAX];
		int failures[ID_CTRL_MAX];
		int status[ID_CTRL_MAX];
		rover->GetControllersStatus(status);
		rover->GetControllersError(errors);
		rover->GetMotorsFailure(failures);
		for(int i = 0; i < ID_CTRL_MAX; i++) {
			printf("-- Controller %s\n", CTRL_NAMES[i]);
			printf("Error: 0x%x\n", errors[i]);
			printf("Failure: 0x%x\n", failures[i]);
			PrintStatus(status[i]);
		}
	}
	else if(!strcmp("--analog-angle", argv[1])){
		for(i = 0; i < 4; i++){
			rover->SetFlipperAngle(0, ID_FLIPPER_FRONT_LEFT + i);
		}
		rover->WaitFlippers();
		rover->SetFlipperAngle(M_PI / 2, ID_FLIPPER_REAR_LEFT);

		bool reached = false;
		double frontLeft, frontRight, rearLeft, rearRight;
		double aFrontLeft, aFrontRight, aRearLeft, aRearRight;
		while(!reached){
			rover->FlippersAngleReached(&reached);
			sleep(0.1);
			rover->ReadFlippersAngle(&frontLeft, &frontRight, &rearLeft, &rearRight);
			rover->ReadAnalogFlippersAngle(&aFrontLeft, &aFrontRight, &aRearLeft, &aRearRight);
			printf("%.6f %.6f\n", frontRight, aFrontRight);
		}
	}
	else {
		printf("no option %s\n", argv[1]);
	}

	delete rover;
	return 0;
}
