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
	uint8_t lspeed[] = { 0x40, 0x6C, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 };
	HAL_CAN_AddTxMessage(hcan, &txHeader, lspeed, &txMailbox);
	uint8_t rspeed[] = { 0x40, 0x6C, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 };
	HAL_CAN_AddTxMessage(hcan, &txHeader, rspeed, &txMailbox);
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
			default:
				this->LeftVelocity=888;
				this->RightVelocity=888;
				break;
		}
		leftVelocity=999;
		rightVelocity=999;
	} else {
		// Invalid data length, set default values
		leftVelocity = 999;
		rightVelocity = 999;
	}
}

void CANServoMotor::Run(){
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x601;

	uint8_t synchronousMode[] =
			{ 0x2B, 0x0F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t velocityMode[] = { 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t accOfLeftMotor[] =
			{ 0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t accOfRightMotor[] =
			{ 0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t decOfLeftMotor[] =
			{ 0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t decOfRightMotor[] =
			{ 0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t enable1[] = { 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t enable2[] = { 0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t enable3[] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 }; // set asynchronous mode
	uint8_t speedOfLeftMotor[] = { 0x23, 0xFF, 0x60, 0x01, 0x0A, 0x00, 0x00,
			0x00 }; // set asynchronous mode
	//uint8_t speedOfRightMotor[] = { 0x23, 0xFF, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00 };// set asynchronous mode
	uint8_t stopLeftMotor[] = { 0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };// set asynchronous mode

	HAL_CAN_AddTxMessage(hcan, &txHeader, synchronousMode, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, velocityMode, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfLeftMotor, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfRightMotor, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfLeftMotor, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfRightMotor, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable1, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable2, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable3, &txMailbox);
	HAL_Delay(1000);
	HAL_CAN_AddTxMessage(hcan, &txHeader, speedOfLeftMotor, &txMailbox);
	HAL_Delay(5000);
//	uint8_t syncMode[] = { 0x2B, 0x0F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }; //async Mode
//	uint8_t ctrMode[] = { 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 };
//
//	uint8_t accOfLeftMotor[] =
//			{ 0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 };
//	uint8_t accOfRightMotor[] =
//			{ 0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 };
//	uint8_t decOfLeftMotor[] =
//			{ 0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 };
//	uint8_t decOfRightMotor[] =
//			{ 0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 };
//	uint8_t enable1[] = { 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 }; // set asynchronous mode
//	uint8_t enable2[] = { 0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00 }; // set asynchronous mode
//	uint8_t enable3[] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 }; // s
//
//	uint8_t speedOfLeftMotor[] = { 0x23, 0xFF, 0x60, 0x01, 0x0A, 0x00, 0x00,
//			0x00 };
//	uint8_t speedOfRightMotor[] = { 0x23, 0xFF, 0x60, 0x01, 0x00, 0x00, 0x00,
//			0x00 };
//
//	HAL_CAN_AddTxMessage(hcan, &txHeader, syncMode, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, ctrMode, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfLeftMotor, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfRightMotor, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfLeftMotor, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfRightMotor, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, enable1, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, enable2, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, enable3, &txMailbox);
//	HAL_CAN_AddTxMessage(hcan, &txHeader, speedOfLeftMotor, &txMailbox);
	//HAL_CAN_AddTxMessage(hcan, &txHeader, speedOfRightMotor, &txMailbox);
}

void CANServoMotor::Enable(){
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x601;
	uint8_t enable1[] = { 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00 };// set asynchronous mode
	uint8_t enable2[] = { 0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00 };// set asynchronous mode
	uint8_t enable3[] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 };// set asynchronous mode
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable1, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable2, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, enable3, &txMailbox);
}

void CANServoMotor::Stop(){
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x601;
	uint8_t stopMotor[] = { 0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
	HAL_CAN_AddTxMessage(hcan, &txHeader, stopMotor, &txMailbox);
}

void CANServoMotor::SetMode(bool synchronous,int mode=0){
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x601;
	uint8_t syncMode[]={ 0x2B, 0x0F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }; //async Mode
	uint8_t ctrMode[]={ 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 }; //velocity mode
	ControlMode=mode;

	if (synchronous) {
		SynchronousMode=true;
		syncMode[4]=01;
	}
	else{
		SynchronousMode=false;
	}
	switch (mode){
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	}
	HAL_CAN_AddTxMessage(hcan, &txHeader, syncMode, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, ctrMode, &txMailbox);
}

void CANServoMotor::SetParameters(int32_t acceleration[], int32_t deceleration[],int32_t velocity[],int32_t position[],int32_t torque[]){
	txHeader.DLC = 8;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x601;
	uint8_t accOfLeftMotor[] = { 0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 };
	uint8_t accOfRightMotor[] = { 0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 };
	uint8_t decOfLeftMotor[] = { 0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00 };
	uint8_t decOfRightMotor[] = { 0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00 };
	switch(ControlMode){
	case 0:

		break;
	}
	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfLeftMotor, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, accOfLeftMotor, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfLeftMotor, &txMailbox);
	HAL_CAN_AddTxMessage(hcan, &txHeader, decOfLeftMotor, &txMailbox);
}
