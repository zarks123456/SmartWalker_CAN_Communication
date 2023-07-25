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
	uint8_t dataLengthCode = rxData[0];
	uint8_t dataLength;
	switch (dataLengthCode) {
	case 0x4F:
		dataLength = 1;
		break;
	case 0x4B:
		dataLength = 2;
		break;
	case 0x47:
		dataLength = 3;
		break;
	case 0x43:
		dataLength = 4;
		break;
	case 128:
		dataLength = 5;
		break;
	default:
		dataLength = 10; // Return -1 if the input is not recognized
	}

	if (rxData[0] == 0x43) {
		leftVelocity = (rxData[5] << 8) | rxData[4];
		rightVelocity = (rxData[7] << 8) | rxData[6];
	} else {
		// Invalid data length, set default values
		leftVelocity = 0;
		rightVelocity = 0;
	}
}
