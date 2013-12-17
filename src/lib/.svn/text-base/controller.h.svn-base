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

/** \file controller.h
 *  \brief Configure motor controller (Elmos)
 *
 */

/** \fn Controller::Enable(bool enable)
 * \brief Enable motor controller power stage
 *
 */

/** \fn int Controller::SetMode(int mode, int radius)
    \brief Set controller mode.
    \param mode Position or velocity mode.
    \param radius The target radius considered as reached.
*/

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "librover.h"

extern "C" {
#include <libcan/can.h>
#include <libcpc/cpclib.h>
#include <libcan/can_cpc.h>
}

#define ID_ALL					0
#define TORQUE_MODE				1
#define POSITION_MODE			5
#define VELOCITY_MODE			2
#define FLOAT_MSG				1
#define INT_MSG					0
#define GET_INT					64
#define SET_INT					0
#define GET_FLOAT				64
#define SET_FLOAT				128
#define PDO2_RECEIVE			0x300 // Process Data Objects
#define ANALOG_MAX				5.0
#define RAD_PER_VOLT			(DEG2RAD(340.0) / ANALOG_MAX)

#define DFLT_REG_OFFSET			24	// default register index for storing offset
#define DFLT_REG_GAIN			23	// default register index for storing gain
#define CALIB_SAMPLES			25	// number of samples for calibration/initialization

typedef struct {
	unsigned char cmd[4];
	float value;
} MsgToFloat;

typedef struct {
	unsigned char cmd[4];
} FloatToMsg;

typedef struct {
	unsigned char cmd[4];
	int value;
} MsgToInt;


class Controller {

	public:
		int elmoMode;
		Controller(int elmoId, const char *name, can_device_p device, bool inverted = false);
		const char *GetName();
		int Enable(bool enable);
		int SetMode(int mode, int radius = -1);
		bool IsInverted();
		int ReadEstop(int *eStop);
		int ReadMotionStatus(int *val);
		int ReadMotorFailure(int *val);
		int ReadStatusRegister(int *val);
		int ReadErrorCodeRegister(int *val);
		int SetEncoder(int val);
		int ReadEncoder(int *val);
		int ClearEncoder();
		int ReadAnalogInput(double *val, int samples = -1);
		int SetAnalogParams(double offset, double gain, bool store = false);
		int StoreCalibration(double offset, double gain, int regOffset = DFLT_REG_OFFSET, int regGain = DFLT_REG_GAIN);
		int ReadCalibration(double *offset, double *gain, int regOffset = DFLT_REG_OFFSET, int regGain = DFLT_REG_GAIN);
		int ReadActiveCurrent(double *iq);
		int ReadRegulatorVelocityGain(float *val);
		int ReadRegulatorPositionGain(float *val);
		int ReadMainEncoderFilter(int *val);
		int WriteMainEncoderFilter(int val);
		int ReadAuxEncoderFilter(int *val);
		int WriteAuxEncoderFilter(int val);
		double GetAnalogOffset();
		double GetAnalogGain();
		int StartJog(int speed);

	protected:
		int elmoId;
		bool invert;
		double analogOffset;
		double analogGain;

		int WriteIntObject(int elmoCmdId, const char* cmd, unsigned char index, int cmdValue);
		int ReadIntObject(int baseCmdId, const char* cmd, unsigned char index, int *val);
		int WriteFloatObject(int elmoCmdId, const char* cmd, unsigned char index, float cmdValue);
		int ReadFloatObject(int baseCmdId, const char* cmd, unsigned char index, float *val);
		int GetMsgQueueCnt();
		int ClearMSGQueue();

	private:
		can_device_p canDev;
		const char *name;
		int MatchReceive(int msgIdSnd, can_message_p canMsgRcv);
		int ReceiveFloat(int msgIdSnd, float *val);
		int ReceiveInt(int msgIdSnd, int *val);
		int CheckMismatch(can_device_p dev, int msgIdSnd, can_message_p canMsgRcv);
	//	int DoMatching(can_device_p dev, can_message_p canMsgRcv);
		void PrintContent(int id, unsigned char * content);
		void TransmitMsg(int baseCmdId, const char* cmdType, unsigned char index, int cmdValue, int msgLength, int *error);
		unsigned char* ReceiveMsg(int msgIdSnd, int *error);

};


#endif /* CONTROLLER_H_ */
