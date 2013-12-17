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

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <signal.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "librover.h"

#define NAME_LENGTH 	128
#define INC_ANGLE	(45.0 / 2 * M_PI / 180.0)
#define ANGLE_45	(45.0 * M_PI / 180.0)
#define INC_RATE	0.1
#define MAX_RATE	1.2


/* Joystick channel and buttons definition */
#define MAX_CHAN_VAL 	32767
#define BTN_FRONT_LEFT 	3
#define BTN_FRONT_RIGHT 2
#define BTN_REAR_LEFT 	0
#define BTN_REAR_RIGHT 	1
#define BTN_FLAT	8
#define BTN_45		9
#define BTN_BRAKE_ON    5
#define BTN_BRAKE_OFF	4
#define BTN_ENABLE	7
#define BTN_DISABLE	6


typedef struct Axis{
	int channel;
	int reversed;
	int inputMax;
	double deadZone;
	double outputMax;
} Axis;

typedef struct ButtonState{
	int size;
	char *pressed;
	char *rising;
	char *falling;
	char *changed;
	char *previous;
} ButtonState;


/* Global variables */
static const btnFlipperMap[4] = {BTN_FRONT_LEFT, BTN_FRONT_RIGHT, BTN_REAR_LEFT, BTN_REAR_RIGHT};
static Axis axisVel = {3, 1, MAX_CHAN_VAL, 0.03, 1.0};
static Axis axisRotVel = {2, 1, MAX_CHAN_VAL, 0.03, 0.2};
static Axis axisIncAngle = {5, 1, MAX_CHAN_VAL, 0.0, 1};
static Axis axisIncScan = {4, 0, MAX_CHAN_VAL, 0.0, 1};
static Axis axisVelStraight = {1, 1, MAX_CHAN_VAL, 0.04, 0};
static struct ButtonState btnState;
static struct RoverParams params;

static int prevFailure[ID_CTRL_MAX];
static int prevStatus[ID_CTRL_MAX];
static int prevError[ID_CTRL_MAX];


void allocButtonState(ButtonState *bstate, int size){
	bstate->size = size;
	bstate->previous = calloc(size, sizeof(char));
	bstate->rising = calloc(size, sizeof(char));
	bstate->falling = calloc(size, sizeof(char));
	bstate->changed = calloc(size, sizeof(char));
	bstate->pressed = calloc(size, sizeof(char));
}

/* Update buttons states with the current state */
void updateButtonState(ButtonState *bstate, char *newState){
	int i;
	for(i = 0; i < bstate->size; i++){
		bstate->changed[i] = bstate->previous[i] ^ newState[i];
		bstate->rising[i] = bstate->changed[i] & newState[i];
		bstate->falling[i] = bstate->changed[i] & ~bstate->rising[i];
		bstate->pressed[i] = newState[i];
	}
	memcpy(bstate->previous, newState, bstate->size);
}

/* Apply the axis mapping on the axis input value */
double applyMapping(Axis *axis, int *input){
	double slope, offset, oIn;
	int in = input[axis->channel];
	if(axis->reversed) in = -in;
	if(in > axis->inputMax) in = axis->inputMax;
	if(in < -axis->inputMax) in = -axis->inputMax;

	oIn = axis->deadZone * axis->inputMax;
	if(fabs(in) < oIn) return 0;
	slope = axis->outputMax / (axis->inputMax - oIn);
	offset = axis->outputMax - slope * axis->inputMax;
	if(in > 0) return slope * in + offset;
	return slope * in - offset;
}

/* Limit translation and rotation speed using the max platform speeds */
void limitSpeed(double *v, double *w, double vMax, double wMax){
	if(*v == 0 || *w == 0) {
		if(*v > vMax) *v = vMax;
		if(*w > wMax) *w = wMax;
		return;
	}
	double m0 = fabs(*v) / fabs(*w);
	double m1 = -vMax / wMax;
	double aw = -m1 * wMax / (m0 - m1);
	double av = m0 * aw;
	double normIn = (*v) * (*v) + (*w) * (*w);
	double normLim = av * av + aw * aw;
	double sv = *v > 0 ? 1.0 : -1.0;
	double sw = *w > 0 ? 1.0 : -1.0;
	if(normIn <= normLim) return;

	*v = sv * av;
	*w = sw * aw;
}

/* Drive the platform */
void drive(char *button, int *axis){
	int i;
	double rate;
	double flipper[4];
	double iq[4];
	double v = applyMapping(&axisVel, axis);
	double w = applyMapping(&axisRotVel, axis);
	double vs = applyMapping(&axisVelStraight, axis);
	static double torqueFront = 1.0, torqueRear = 1.0;
	int pressed = 0;

	limitSpeed(&v, &w, axisVel.outputMax, axisRotVel.outputMax);
	if(vs != 0.0) {
		nrSetSpeed(vs, 0.0);
	}
	else nrSetSpeed(v, w);

	/* Handle flippers */
	updateButtonState(&btnState, button);
	int inc = (int) applyMapping(&axisIncAngle, axis);
	if(inc != 0){
		nrGetFlippers(&flipper[0], &flipper[1], &flipper[2], &flipper[3]);
		torqueFront += inc*0.5; torqueRear += inc*0.5;
		if (torqueFront < 0.0) torqueFront = 0.0;
		if (torqueFront > TORQUE_MAX) torqueFront = TORQUE_MAX;
		if (torqueRear < 0.0) torqueRear = 0.0;
		if (torqueRear > TORQUE_MAX) torqueRear = TORQUE_MAX;
		for(i = 0; i < 4; i++){

			if(btnState.pressed[btnFlipperMap[i]]){
				pressed = 1;
				if(i < 2) flipper[i] -= inc * INC_ANGLE;
				else flipper[i] += inc * INC_ANGLE;
			}
		}

		if (!pressed){
			int e = nrSetFlippersTorque(torqueFront, torqueRear);
			printf("Torque set to %f [A]; Error: %d\n", torqueFront, e);
		}

		for(i = 0; i < 4; i++){
			nrSetFlipper(flipper[i], ID_FLIPPER_FRONT_LEFT + i);
		}
		pressed = 0;
	}

	inc = (int) applyMapping(&axisIncScan, axis);
	if(inc != 0){
		nrGetScanningSpeed(&rate);
		rate += inc * INC_RATE;
		if(rate < 0) rate = 0;
		if(rate > MAX_RATE) rate = MAX_RATE;
		nrSetScanningSpeed(rate);
	}

	if(btnState.rising[BTN_ENABLE]){
		printf("enable controllers\n");
		nrEnable(1);
	}

	if(btnState.rising[BTN_DISABLE]){
		printf("disable controllers\n");
		nrEnable(0);
	}

	if(btnState.rising[BTN_45]) {
		for(i = 0; i < 4; i++){
			if(i < 2) nrSetFlipper(-ANGLE_45, ID_FLIPPER_FRONT_LEFT + i);
			else nrSetFlipper(ANGLE_45, ID_FLIPPER_FRONT_LEFT + i);
		}
	}
	if(btnState.rising[BTN_FLAT]){
		for(i = 0; i < 4; i++){
			nrSetFlipper(0, ID_FLIPPER_FRONT_LEFT + i);
			nrReadDOFCurrent(&iq[i], ID_FLIPPER_FRONT_LEFT + i);
		}
		printf("Active current: %f, %f, %f, %f\n\n", iq[0], iq[1], iq[2], iq[3]);
	}
	if(btnState.rising[BTN_BRAKE_ON]){
		printf("chassis blocked\n");
		nrSetBrake(1);
	}
	if(btnState.rising[BTN_BRAKE_OFF]){
		printf("chassis released\n");
		nrSetBrake(0);
	}

/*	int failure[ID_CTRL_MAX];
	int status[ID_CTRL_MAX];
	int error[ID_CTRL_MAX];
	int diff = 0;int diffEC = 0; int diffFailure = 0;
	nrGetMotorsFailure(failure);
	nrGetControllersStatus(status);
	nrGetControllersError(error);

	for(i = 0; i < ID_CTRL_MAX; i++)
		if(status[i] != prevStatus[i]) diff = 1;
	for(i = 0; i < ID_CTRL_MAX; i++)
		if(error[i] != prevError[i]) diffEC = 1;
	for(i = 0; i < ID_CTRL_MAX; i++)
			if(failure[i] != prevFailure[i]) diffFailure = 1;

	if(diffFailure) {
		printf("MF 0x: ");
		for(i = 0; i < ID_CTRL_MAX; i++) printf("%X ", failure[i]);
		printf("\n");
		}
	if(diff) {
		printf("SR 0x: ");
		for(i = 0; i < ID_CTRL_MAX; i++) printf("%X ", status[i]);
		printf("\n");
	}
	if(diffEC) {
		printf("EC dec: ");
		for(i = 0; i < ID_CTRL_MAX; i++) printf("%d ", error[i]);
		printf("\n");
	}
	memcpy(prevFailure, failure, sizeof(int) * ID_CTRL_MAX);
	memcpy(prevStatus, status, sizeof(int) * ID_CTRL_MAX);
	memcpy(prevError, error, sizeof(int) * ID_CTRL_MAX);
*/
}

void interrupt(int sig){
	nrDestroy();
	exit(0);
}

void testTime(){
	struct timeval start, end;
	double diff;
	double v, w;
	double left, right, scannerAngle, battLevel, scanningSpeed;
	double flipper[4];
	int status[ID_CTRL_MAX];
	int brake, battState;

	gettimeofday(&start, NULL);

	nrGetSpeed(&v, &w);
	nrGetDifferentialAngles(&left, &right);
	nrGetFlippers(&flipper[0], &flipper[1], &flipper[2], &flipper[3]);
	nrGetScannerAngle(&scannerAngle);
	nrGetBatteryLevel(&battLevel, &battState);
	nrGetControllersStatus(status);
	nrGetBrake(&brake);
	nrGetScanningSpeed(&scanningSpeed);

	gettimeofday(&end, NULL);
	diff = (end.tv_sec - start.tv_sec)*1000.0 + (end.tv_usec - start.tv_usec)/1000.0;
	printf("\n\nexecuting time: %.2f ms\n\n", diff);
}

int openJoystick(int argc, char **argv){
	if(argc > 1)
		return open(argv[argc-1], O_RDONLY);
	else 
		return open("/dev/input/js0", O_RDONLY);
}

int main (int argc, char **argv)
{
	int fd, i;
	unsigned char axes = 2;
	unsigned char buttons = 2;
	char name[NAME_LENGTH] = "Unknown";

	if (argc > 2 || (argc > 1 && !strcmp("--help", argv[1]))) {
		puts("");
		puts("Usage: jsdrive [device]");
		puts("");
		puts("");
		return 1;
	}


	fd = openJoystick(argc, argv);
	if (fd < 0){
                perror("jsdrive");
                return 1;
        }

	(void) signal(SIGINT, interrupt);
	ioctl(fd, JSIOCGAXES, &axes);
	ioctl(fd, JSIOCGBUTTONS, &buttons);
	ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);


	struct js_event js;
	int *axis;
	char *button;

	/* Init rover */
	printf("Initializing rover...\n");
	nrInit(NULL, NULL, 1);
	nrGetRoverParams(&params);
	params.wMax /= 1;

	/* Init joystick related variables */
	allocButtonState(&btnState, buttons);
	axisVel.outputMax = params.vMax;
	axisVelStraight.outputMax = params.vMax;
	axisRotVel.outputMax = params.wMax;
	axis = calloc(axes, sizeof(int));
	button = calloc(buttons, sizeof(char));

	printf("Rover ready (press any button to enable rover)\n");
	int event;
	while (1) {
		if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
			 perror("\njsdrive: error reading");
			 nrSetSpeed(0,0);
		}
		else {
			event =  0;
			switch(js.type) {
			case JS_EVENT_BUTTON:
				 button[js.number] = js.value;
				 event = 1;
				 break;
			case JS_EVENT_AXIS:
				 axis[js.number] = js.value;
				 event = 1;
				 break;
			}

			if(event) {
				//printf("type: %d num: %d value: %d\n", js.type, js.number, js.value);
				drive(button, axis);
			}
		}
		usleep(1000);

//		testTime();
	}
	return 0;
}

