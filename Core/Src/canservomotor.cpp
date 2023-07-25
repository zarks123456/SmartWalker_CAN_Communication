#include "CANServoMotor.h"

CANServoMotor::CANServoMotor(uint8_t id) {
	motorID = id;
}

CANServoMotor::~CANServoMotor() {
	// Destructor implementation (if required)
}

void CANServoMotor::Reset() {
	txHeader.DLC = 2;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = static_cast<uint32_t>(COBID::NMT);

	uint8_t reset[] = { 82, motorID };
	HAL_CAN_AddTxMessage(this->hcan, &txHeader, reset, &txMailbox);
}

void CANServoMotor::Init(CAN_HandleTypeDef *hcan) {
	this->hcan = hcan;
}

void CANServoMotor::StartCommand() {
	txHeader.DLC = 2;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = static_cast<uint32_t>(COBID::NMT);

	uint8_t operationalmode[] = { 01, motorID };
	HAL_CAN_AddTxMessage(hcan, &txHeader, operationalmode, &txMailbox);
}



void CANServoMotor::GetVelocity() {
	txHeader.DLC = 8;
	txHeader.StdId = 0x601;
	uint8_t speed[] = { 0x40, 0x6C, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 };
	HAL_CAN_AddTxMessage(hcan, &txHeader, speed, &txMailbox);
}

CANServoMotor::Data CANServoMotor::ParseData(uint8_t *rxData,
		CAN_RxHeaderTypeDef &rxHeader) {
	Data parseData;
	uint16_t registrarAddress = (static_cast<uint16_t>(rxData[2]) << 8)
			| rxData[1];

	switch (registrarAddress){
	case 0x606C:
		parseData.Error=this->parseVelocity(rxData, parseData.leftValue, parseData.rightValue);
		break;
	}
	// Check if the data length is valid (less than or equal to 7, considering RxData[0] is the data length)

	parseData.Address=registrarAddress;
	return parseData;
}

bool CANServoMotor::parseVelocity(uint8_t *rxData, int &leftVelocity,
		int &rightVelocity) {
	if (rxData[0] == 0x43) {
		switch (rxData[3]){
			case 0x01:
				this->LeftVelocity= ((static_cast<int32_t>(rxData[7]) << 24) | (rxData[6] << 16)
				| (rxData[5] << 8) | rxData[4])*0.1;
				break;
			case 0x02:
				this->RightVelocity=((static_cast<int32_t>(rxData[7]) << 24) | (rxData[6] << 16)
						| (rxData[5] << 8) | rxData[4])*0.1;
				break;
		}
		leftVelocity=0;
		rightVelocity=0;
	} else {
		// Invalid data length, set default values
		leftVelocity = 0;
		rightVelocity = 0;
	}
}
