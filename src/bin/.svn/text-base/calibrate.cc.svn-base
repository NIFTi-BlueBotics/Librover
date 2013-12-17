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

void printUsage(){
	puts("");
	puts("Usage: calibrate <option> [device]");
	puts("");
	puts("");
}

int main (int argc, char **argv)
{
	if (argc < 2 || argc > 3 || !strcmp("--help", argv[1])) {
		printUsage();
		return 1;
	}

	Rover *rover;
	if(argc > 2)
		rover = new Rover(argv[2], NULL);
	else
		rover = new Rover("/dev/usb/cpc_usb0", NULL);

	/* Calibrate the analog inputs of the chassis state sensors. Before calling this function
	 * make sure that:
	 * 1. the flippers must be fully deployed on a flat ground and disabled (not controlled).
	 * 2. the brake is disabled.
	 * The calibration parameters are stored in the flash memory of the controllers. */
	if(!strcmp("--chassis", argv[1])){
		rover->EnableMotion(false);
		rover->SetBrake(false);
		printf("Make sure the flippers are fully deployed, on a flat surface!\n");
		printf("Press Enter to start calibration...");
		getc(stdin);
		rover->CalibrateChassis();
		printf("Chassis calibration done!\n");
	}
	else if(!strcmp("--voltage", argv[1])){
		printf("Make sure the platform is powered with the external power supply.\n");
		printf("Set voltage to exactly %.1fV\n", CALIB_LOW_VOLTAGE);
		printf("Press Enter ...");
		getc(stdin);

		Controller *core = rover->GetController(ID_CORE);
		double low, high;
		core->ReadAnalogInput(&low, 50);

		printf("Set voltage to exactly %.1fV\n", CALIB_HIGH_VOLTAGE);
		printf("Press Enter ...");
		getc(stdin);

		core->ReadAnalogInput(&high, 50);
		rover->SetVoltageCalibration(low, (CALIB_HIGH_VOLTAGE - CALIB_LOW_VOLTAGE) / (high - low));
		printf("Voltage calibration done!\n");
	}

	delete rover;
	return 0;
}
