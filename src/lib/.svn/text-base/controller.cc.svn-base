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

#include "controller.h"
#include <unistd.h>

#include <sys/time.h>


Controller::Controller(int elmoId, const char *name, can_device_p device, bool inverted){
	this->elmoId = elmoId;
	this->name = name;
	canDev = device;
	invert = inverted;
	ReadCalibration(&analogOffset, &analogGain);
	WriteMainEncoderFilter(0);
	WriteAuxEncoderFilter(0);
}

const char *Controller::GetName(){
	return name;
}

int Controller::Enable(bool enable){
	int e;
	if (elmoId !=1){
		if (enable) e = WriteIntObject(PDO2_RECEIVE, "ui", 1, 1);
		else e = WriteIntObject(PDO2_RECEIVE, "ui", 2, 1);
	}
	else{
		if (enable) e = WriteIntObject(PDO2_RECEIVE, "mo", 0, 1);
		else e = WriteIntObject(PDO2_RECEIVE, "mo", 0, 0);
	}
	usleep(150000); // To make sure the controller has executed the code
	return e;
}

int Controller::SetMode(int mode, int radius){
	int e;
	if(e = Enable(false)) return e;
	elmoMode = mode;
	if (mode == POSITION_MODE){
		if(e = WriteIntObject(PDO2_RECEIVE, "um", 0, POSITION_MODE)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "hm", 1, 0)) return e; // Disarm homing process
		if(e = WriteFloatObject(PDO2_RECEIVE, "cl", 1, TORQUE_MAX)) return e; //permanent current limitation
		if(e = WriteIntObject(PDO2_RECEIVE, "cl", 2, 0)) return e; // % current limitation
		if(e = WriteIntObject(PDO2_RECEIVE, "cl", 3, 6)) return e; // speed threshold
		if(e = WriteFloatObject(PDO2_RECEIVE, "pl", 1, 2*TORQUE_MAX)) return e; //peak current limitations
		if(e = WriteIntObject(PDO2_RECEIVE, "pl", 2, 3)) return e; // peak duration
		if(e = WriteIntObject(PDO2_RECEIVE, "xm", 1, -1000000)) return e; // Modulo enc avoid multi turn
		if(e = WriteIntObject(PDO2_RECEIVE, "xm", 2, 1000000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "er", 3, 45)) return e; // 3% position error tolerance
		if(e = WriteIntObject(PDO2_RECEIVE, "kp", 3, 10)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "ki", 2, 60000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "gs", 2, 0)) return e; // not using scheduled gain table
		if(radius > 0){
			if(e = WriteIntObject(PDO2_RECEIVE, "tr", 1, radius)) return e;
		}
	}else if (mode == VELOCITY_MODE){
		if(e = WriteIntObject(PDO2_RECEIVE, "ac", 0, 15000)) return e; // Acceleration rate
		if(e = WriteIntObject(PDO2_RECEIVE, "dc", 0, 20000)) return e; // Deceleration rate
		if(e = WriteIntObject(PDO2_RECEIVE, "sf", 0, 10)) return e; // Smooth factor [ms] for profile mode
		if(e = WriteIntObject(PDO2_RECEIVE, "um", 0, VELOCITY_MODE)) return e;
		if(e = WriteFloatObject(PDO2_RECEIVE, "cl", 1, TORQUE_MAX)) return e; //permanent current limitation
		if(e = WriteFloatObject(PDO2_RECEIVE, "pl", 1, 2*TORQUE_MAX)) return e; //peak current limitations
		if(e = WriteIntObject(PDO2_RECEIVE, "pl", 2, 3)) return e;	// peak duration
		if(e = WriteIntObject(PDO2_RECEIVE, "pm", 0, 1)) return e; // Profiler mode for using scheduled gain
		if(e = WriteIntObject(PDO2_RECEIVE, "er", 2, 5000)) return e; //velocity error tolerance
		if(e = WriteIntObject(PDO2_RECEIVE, "kp", 2, 4000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "ki", 2, 8000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "gs", 2, 64)) return e; // use scheduled gain table
	}else if (mode == TORQUE_MODE){// Torque are expressed in [A]
		if(e = WriteIntObject(PDO2_RECEIVE, "um", 0, POSITION_MODE)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "hm", 1, 0)) return e; // Disarm homing process
		if(e = WriteFloatObject(PDO2_RECEIVE, "cl", 1, TORQUE_MAX)) return e; //permanent current limitation
		if(e = WriteIntObject(PDO2_RECEIVE, "cl", 2, 0)) return e; // % current limitation
		if(e = WriteIntObject(PDO2_RECEIVE, "cl", 3, 6)) return e; // speed threshold
		if(e = WriteFloatObject(PDO2_RECEIVE, "pl", 1, 2.0*TORQUE_MAX)) return e; //peak current limitations
		if(e = WriteIntObject(PDO2_RECEIVE, "pl", 2, 3)) return e; // peak duration
		if(e = WriteIntObject(PDO2_RECEIVE, "xm", 1, -1000000)) return e; // Modulo enc avoid multi turn
		if(e = WriteIntObject(PDO2_RECEIVE, "xm", 2, 1000000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "er", 3, 100000)) return e; // 3% position error tolerance
		if(e = WriteIntObject(PDO2_RECEIVE, "kp", 3, 1)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "ki", 2, 60000)) return e;
		if(e = WriteIntObject(PDO2_RECEIVE, "gs", 2, 0)) return e; // not using scheduled gain table
	}
	return Enable(true);
}

bool Controller::IsInverted(){
	return invert;
}

int Controller::ReadEstop(int *eStop){
	return ReadIntObject(PDO2_RECEIVE, "ib", 16, eStop);
}

int Controller::ReadMotionStatus(int *val){
	return ReadIntObject(PDO2_RECEIVE, "ms", 0, val);
}

int Controller::ReadRegulatorVelocityGain(float *val) {
	return ReadFloatObject(PDO2_RECEIVE, "kp", 2, val);
}

int Controller::ReadRegulatorPositionGain(float *val) {
	return ReadFloatObject(PDO2_RECEIVE, "kp", 3, val);
}

int Controller::ReadMotorFailure(int *val){
	return ReadIntObject(PDO2_RECEIVE, "mf", 0, val);
}

int Controller::ReadStatusRegister(int *val){
	return ReadIntObject(PDO2_RECEIVE, "sr", 0, val);
}

int Controller::ReadErrorCodeRegister(int *val){
	return ReadIntObject(PDO2_RECEIVE, "ec", 0, val);
}

int Controller::ReadMainEncoderFilter(int *val){
	return ReadIntObject(PDO2_RECEIVE, "ef", 1, val);
}

int Controller::WriteMainEncoderFilter(int val){
	return WriteIntObject(PDO2_RECEIVE, "ef", 1, val);
}

int Controller::ReadAuxEncoderFilter(int *val){
	return ReadIntObject(PDO2_RECEIVE, "ef", 2, val);
}

int Controller::WriteAuxEncoderFilter(int val){
	return WriteIntObject(PDO2_RECEIVE, "ef", 2, val);
}

int Controller::SetEncoder(int val){
	Enable(false);
	WriteIntObject(PDO2_RECEIVE, "px", 0, val);
	return Enable(true);
}

int Controller::ReadEncoder(int *val){
	return ReadIntObject(PDO2_RECEIVE, "px", 0, val);
}

int Controller::ClearEncoder(){
	return SetEncoder(0);
}

int Controller::ReadAnalogInput(double *val, int samples){
	int e;
	float read;
	if(samples > 0){
		*val = 0;
		for(int i = 0; i < samples; i++){
			e = ReadFloatObject(PDO2_RECEIVE, "an", 1, &read);
			*val += read;
		}

		*val /= samples;
		return e;
	}
	e = ReadFloatObject(PDO2_RECEIVE, "an", 1, &read);
	*val = read;
	return e;
}

int Controller::SetAnalogParams(double offset, double gain, bool store){
	analogOffset = offset;
	analogGain = gain;
	if(store) return StoreCalibration(offset, gain);
	return CAN_ERROR_NONE;
}

// WARNING: Avoid using this procedure frequently. Flash could be destroyed!!!
int Controller::StoreCalibration(double offset, double gain, int regOffset, int regGain){
	int e, tmp;

	if(e = Enable(false)){
		DEBUG_PRINT(("ERROR disabling controller\n"));
		return e;
	}
	sleep(1); // Avoid communication problems

	if(e = ReadIntObject(PDO2_RECEIVE, "hp", 0, &tmp)){
		DEBUG_PRINT(("ERROR halting user program (elmo firmware)\n"));
		return e;
	}
	if(e = WriteFloatObject(PDO2_RECEIVE, "uf", regOffset, (float)offset)) return e;
	if(e = WriteFloatObject(PDO2_RECEIVE, "uf", regGain, (float)gain)) return e;
	if(e = ReadIntObject(PDO2_RECEIVE, "sv", 0, &tmp)){
		DEBUG_PRINT(("ERROR saving into FLASH\n"));
		return e;
	}
	sleep(1); // Avoid communication problems
	if(e = ReadIntObject(PDO2_RECEIVE, "xc", 0, &tmp)){
		DEBUG_PRINT(("ERROR restarting user program\n"));
		return e;
	}
	DEBUG_PRINT(("Store calibration controller id %d into FLASH (off: %f gain: %f)\n", elmoId, offset, gain));
	return Enable(true);
}

int Controller::ReadCalibration(double *offset, double *gain, int regOffset, int regGain){
	int e;
	float off, g;
	if(e = ReadFloatObject(PDO2_RECEIVE, "uf", regOffset, &off)) return e;
	*offset = off;
	if(e = ReadFloatObject(PDO2_RECEIVE, "uf", regGain, &g)) return e;
	*gain = g;
	DEBUG_PRINT(("%s : %f %f\n", name, analogOffset, analogGain));
	return e;
}

int Controller::ReadActiveCurrent(double *iq){
	float iqTmp; int e;
	if(e = ReadFloatObject(PDO2_RECEIVE, "iq", 0, &iqTmp)) return e;
	*iq = iqTmp;
	return e;
}

double Controller::GetAnalogOffset(){
	return analogOffset;
}

double Controller::GetAnalogGain(){
	return analogGain;
}

int Controller::StartJog(int speed){
	int e, tmp;
	if(IsInverted()) speed = -speed;
	if(e = WriteIntObject(PDO2_RECEIVE, "jv", 0, speed)) return e;
	return ReadIntObject(PDO2_RECEIVE, "bg", 0, &tmp);
}

// baseCmdId: cf SimpleIQ Command reference Manual §2.7 p.2-3 (MAN-CAN301IG.pdf)
// cmd: 3bytes (1st char elmo cmd, 2nd char, index); cmdType: set or request
// cmdType : bit6=(0:set; 1:request); bit7=(0:int;1:float)
// int cmdValue: 4 bytes for commands value
// cf SimpleIQ Command reference Manual (MAN-CAN301IG.pdf)
int Controller::WriteIntObject(int baseCmdId, const char* cmd, unsigned char index, int cmdValue){

	int e = 0;
	int rcvInt;
	can_message_t canMsg;
	int sendId = baseCmdId + elmoId;

	canMsg.content[0] = cmd[0];
	canMsg.content[1] = cmd[1];
	canMsg.content[2] = index;
	canMsg.content[3] = SET_INT; // 0 for int and set cmd

	// transform into little endian format
	for (int i=0; i<4; i++)
		canMsg.content[4+i] = (cmdValue>>(i*8)) & 0xff;

	if(baseCmdId == ID_ALL){
		canMsg.length=4;
		canMsg.id= 0;
		canMsg.content[0]=1; 	//Set Elmo to remote access
		canMsg.content[1]= 0; 	//message sent to all Elmos
		}
	else{
		canMsg.length=8;
		canMsg.id= sendId;
	}

	if(e = can_send_message(canDev, &canMsg)){
		DEBUG_PRINT(("Controller::WriteIntObject ERROR sending: %x\n", e));
		return e;
	}

	if(baseCmdId != ID_ALL) {
		e = ReceiveInt(sendId, &rcvInt);
	}

	return e;
}

// baseCmdId: cf SimpleIQ Command reference Manual §2.7 p.2-3 (MAN-CAN301IG.pdf)
// cmd: 3bytes (1st char elmo cmd, 2nd char, index); cmdType: set or request
// cmdType : bit6=(0:set; 1:request); bit7=(0:int;1:float)
// int cmdValue: 4 bytes for commands value
// cf SimpleIQ Command reference Manual (MAN-CAN301IG.pdf)
int Controller::ReadIntObject(int baseCmdId, const char* cmd, unsigned char index, int *val){

	int e = 0;
	int sendId = baseCmdId + elmoId;
	can_message_t canMsg;

	canMsg.length = 4;
	canMsg.id = sendId;
	canMsg.content[0] = cmd[0];
	canMsg.content[1] = cmd[1];
	canMsg.content[2] = index;
	canMsg.content[3] = GET_INT;

	if(e = can_send_message(canDev, &canMsg)) {
		DEBUG_PRINT(("Controller::ReadIntObject ERROR sending: %x\n", e));
		return e;
	}
	return ReceiveInt(sendId, val);
}

// baseCmdId: cf SimpleIQ Command reference Manual §2.7 p.2-3 (MAN-CAN301IG.pdf)
// cmd: 3bytes (1st char elmo cmd, 2nd char, index); cmdType: set or request
// cmdType : bit6=(0:set; 1:request); bit7=(0:int;1:float)
// int cmdValue: 4 bytes for commands value
// cf SimpleIQ Command reference Manual (MAN-CAN301IG.pdf)
int Controller::WriteFloatObject(int baseCmdId, const char* cmd, unsigned char index, float cmdValue){

	int e = 0;
	int sendId = baseCmdId + elmoId;
	float flt;
	can_message_t canMsg;

	canMsg.content[0] = cmd[0];
	canMsg.content[1] = cmd[1];
	canMsg.content[2] = index;
	canMsg.content[3] = SET_FLOAT;

	// transform into little endian format
	for(int i=0; i<4; i++)
		canMsg.content[4+i] = ((FloatToMsg*)&cmdValue)->cmd[i];

	canMsg.length=8;
	canMsg.id= sendId;

	if(e = can_send_message(canDev, &canMsg)){
		DEBUG_PRINT(("Controller::WriteFloatObject ERROR send: %x\n", e));
		return e;
	}
	return ReceiveFloat(sendId, &flt);
}

// baseCmdId: cf SimpleIQ Command reference Manual §2.7 p.2-3 (MAN-CAN301IG.pdf)
// cmd: 3bytes (1st char elmo cmd, 2nd char, index); cmdType: set or request
// cmdType : bit6=(0:set; 1:request); bit7=(0:int;1:float)
// int cmdValue: 4 bytes for commands value
// cf SimpleIQ Command reference Manual (MAN-CAN301IG.pdf)
int Controller::ReadFloatObject(int baseCmdId, const char* cmd, unsigned char index, float *val){

	int e=0; int i=0;
	float rcvFloat; int sendId = baseCmdId + elmoId;
	can_message_t canMsg;

	canMsg.length=4;
	canMsg.id= sendId;
	canMsg.content[0] = cmd[0];
	canMsg.content[1] = cmd[1];
	canMsg.content[2] = index;
	canMsg.content[3] = GET_FLOAT;

	if(e = can_send_message(canDev, &canMsg)){
		DEBUG_PRINT(("Controller::ReadFloatObject ERROR send: %x\n", e));
		return e;
	}
	return ReceiveFloat(sendId, val);
}

int Controller::MatchReceive(int msgIdSnd, can_message_p canMsgRcv){
	int e; int timeout = 0;
	struct timeval start, end;

	if(e = can_receive_message(canDev, canMsgRcv)){
		return e;
	}
	else{
		int count = 0;
		gettimeofday(&start, NULL);
		while((CheckMismatch(canDev, msgIdSnd, canMsgRcv)) && !timeout){
			count++;
			DEBUG_PRINT(("... Retrying %d time(s)...\n", count));
			canMsgRcv->id = -1;
			if(e = can_receive_message(canDev, canMsgRcv)) {
				DEBUG_PRINT(("ERROR receive msg: %d\n", e));
				return e;
			}
			DEBUG_PRINT(("Received CAN id: 0x%x ", canMsgRcv->id));
			if(msgIdSnd - 128 - canMsgRcv->id) DEBUG_PRINT(("is NOT MATCHING\n"));
			else DEBUG_PRINT(("is MATCHING\n"));
			if(canMsgRcv->id == -1) {
				timeout = 1;
				DEBUG_PRINT(("TIMEOUT retrying %d time(s)\n", count));
				return false;
			}
		}
	}
	return true;
}


int Controller::ReceiveFloat(int msgIdSnd, float *val){
	int timeout = 0;
	struct timeval start, end;
	can_message_t canMsgRcv;

	if(MatchReceive(msgIdSnd, &canMsgRcv)){
		*val = ((MsgToFloat*)canMsgRcv.content)->value;
		return CAN_ERROR_NONE;
	}
	return CAN_ERROR_RECEIVE;
}

int Controller::ReceiveInt(int msgIdSnd, int *val){
	int e; int timeout = 0;
	struct timeval start, end;
	can_message_t canMsgRcv;

	if(MatchReceive(msgIdSnd, &canMsgRcv)){
		*val = ((MsgToInt*)canMsgRcv.content)->value;
		return CAN_ERROR_NONE;
	}
	return CAN_ERROR_RECEIVE;
}

int Controller::CheckMismatch(can_device_p dev, int msgIdSnd, can_message_p canMsgRcv){
	int e;
	if(msgIdSnd - 128 - canMsgRcv->id){
		DEBUG_PRINT(("WARNING: received ID(0x%x) does not match sent ID(0x%x)\n", canMsgRcv->id, msgIdSnd));
		if (canMsgRcv->id < 0x188){
			DEBUG_PRINT(("Motor %d reports errors:\n", (canMsgRcv->id) & 15));
			for(int i=0; i<8; i++){
				if((canMsgRcv->content[0] >> i) & 0x01 ){
					switch (i){
						case 0: DEBUG_PRINT(("Ready to switch on\n"));
							break;
						case 1: DEBUG_PRINT(("Switched on\n"));
							break;
						case 2: DEBUG_PRINT(("Operation enabled\n"));
							break;
						case 3: DEBUG_PRINT(("Fault\n"));
							break;
						case 4: DEBUG_PRINT(("Voltage enabled\n"));
							break;
						case 5: DEBUG_PRINT(("Quick stop\n"));
							break;
						case 6: DEBUG_PRINT(("Switch on disabled\n"));
							break;
						case 7: DEBUG_PRINT(("Warning \n"));
							break;
					}
				}
			}
			for(int i=0; i<8; i++){
				if((canMsgRcv->content[1] >> i) & 0x01 ){
					switch (i){
						case 0: DEBUG_PRINT(("Manufacturer specific\n"));
							break;
						case 1: DEBUG_PRINT(("Remote mode\n"));
							break;
						case 2: DEBUG_PRINT(("Target reached\n"));
							break;
						case 3: DEBUG_PRINT(("Internal limit active\n"));
							break;
						case 4: DEBUG_PRINT(("Operation mode specific\n"));
							break;
						case 5: DEBUG_PRINT(("Operation mode specific\n"));
							break;
						case 6: DEBUG_PRINT(("Manufacturer specific\n"));
							break;
						case 7: DEBUG_PRINT(("Manufacturer specific\n"));
							break;
					}
				}
			}
		}
		PrintContent(canMsgRcv->id, &canMsgRcv->content[0]);
		return true;
	}
	return false;
}
/*
int Controller::DoMatching(can_device_p dev, can_message_p canMsgRcv){
	int e;
	DEBUG_PRINT(("... Do matching ...\n"));
	if(e = can_receive_message(canDev, canMsgRcv)) return e;
	PrintContent(canMsgRcv->id, &canMsgRcv->content[0]);
	return 0;
}
*/
void Controller::PrintContent(int id, unsigned char * content){
	DEBUG_PRINT(("Received hex(char) ID 0x%x: ", id));
	for (int i=0; i<8;i++)
		DEBUG_PRINT(("%x(%c) ", content[i], content[i]));
	DEBUG_PRINT(("\n"));
}

//________________________ FOR TESTS PURPOSES _______________________________

/*
// int ElmoCmdId: cf SimpleIQ Command reference Manual (MAN-CAN301IG.pdf)
//  char* commands_type: each char have to be separated by a comma, 7 chars in total
// int commands_value: 4 bytes for commands value
void Controller::TransmitMsg(int baseCmdId, const char* cmdType, unsigned char index, int cmdValue, int msgLength, int *error){

	int rcvInt;
	can_message_t canMsg;
	int sendId = baseCmdId + elmoId;

	canMsg.content[0] = cmdType[0];
	canMsg.content[1] = cmdType[1];
	canMsg.content[2] = index;
	canMsg.content[3] = SET_INT; // 0 for int and set cmd

	// transform into little endian format
	for (int i = 0; i < 4; i++)
		canMsg.content[4+i] = (cmdValue>>(i*8)) & 0xff;

	canMsg.length=msgLength;
	canMsg.id= sendId;

	if(*error = can_send_message(canDev, &canMsg)){
		DEBUG_PRINT(("Controller::TransmitMsg ERROR sending: %x\n", *error));
	}
}
*/
// _______________________________ FOR TESTS PURPOSES __________________________________
/*
unsigned char* Controller::ReceiveMsg(int msgIdSnd, int *error){
	int e=0; int i=0;
	can_message_t canMsgRcv;
	unsigned char* rcvMsg;

	if(e = can_receive_message(canDev, &canMsgRcv)){
		DEBUG_PRINT(("Controller::ReceiveMsg ERROR: %x\n", e));
		for (i = 0; i < 8; i++){
			rcvMsg[i] = 0;
		}
	}
	else{
		DEBUG_PRINT(("Controller::ReceiveMsg: id=0x%x. ", canMsgRcv.id));
		CheckMismatch(canDev, msgIdSnd, &canMsgRcv);
	}
	if(CheckMismatch(canDev, msgIdSnd, &canMsgRcv))
		*error = true;
	else *error = false;

	return rcvMsg;
}

*/
int Controller::GetMsgQueueCnt(){
	printf("CanState: %d\n", CPC_GetCANState(((can_cpc_device_p)(canDev->comm_dev))->handle));
	return CPC_GetMSGQueueCnt(((can_cpc_device_p)(canDev->comm_dev))->handle);
}
int Controller::ClearMSGQueue(){
	return CPC_ClearMSGQueue(((can_cpc_device_p)(canDev->comm_dev))->handle);
}
