#ifndef CANSERVOMOTOR_H
#define CANSERVOMOTOR_H

#include "stm32f1xx_hal.h"

class CANServoMotor {

public:
	CANServoMotor(uint8_t id);
	~CANServoMotor();

	void Reset();
	void StartCommand();
	void GetVelocity();
	void Init(CAN_HandleTypeDef *hcan);
	void Run();
	void Enable();
	void Stop();
	void SetMode(bool synchronous,int mode);
	void SetParameters(int32_t acceleration[], int32_t deceleration[],int32_t velocity[],int32_t position[],int32_t torque[]);
	
	double LeftVelocity;
	double RightVelocity;
	bool SynchronousMode;
	int ControlMode;
	
	int32_t CommandVelocity[2];
	int32_t CommandPosition[2];
	int32_t CommandTorque[2];

private:
	uint8_t motorID;

	CAN_HandleTypeDef *hcan;

	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;

	bool parseVelocity(uint8_t *rxData, int &leftVelocity,
			int &rightVelocity);
public:
	enum class COBID {
		NMT = 0x000,
		SYNC = 0x080,
		Emergency = 0x081,
		TPDO0 = 0x181,
		RPDO0 = 0x201,
		TPDO1 = 0x281,
		RPDO1 = 0x301,
		TPDO2 = 0x381,
		RPDO2 = 0x401,
		TPDO3 = 0x481,
		RPDO3 = 0x501,
		RSDOServerSend = 0x581,
		TSDOClientRespond = 0x601,
		NMTErrorControl = 0x701
	// Add other code numbers as needed...
	};

	struct Data {
		COBID dataType;
		bool Error;
		uint16_t Address;
		int leftValue;
		int rightValue;
	};
	CANServoMotor::Data ParseData(uint8_t *rxData,
			CAN_RxHeaderTypeDef &rxHeader);

};

#endif // CANSERVOMOTOR_H
