#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>

// ID of each CAN message
// Byte the message starts at (out of 8)
// ASCII length of data (1 data bytes -> 3 ASCII bytes, 2 data bytes -> 5 ASCII
// bytes)
#define SOC_ID 0x6B0
#define SOC_byte 4
#define SOC_length 3

#define FPV_ID 0x6B0
#define FPV_byte 2
#define FPV_length 5

#define highTempID 0x6B4
#define highTempByte 0
#define lowTempID 0x6B4
#define lowTempByte 2
#define tempLength 5

#define auxVoltageID 0x700
#define auxVoltageLength 4

#define motorTempID 0x0A2
#define motorTempByte 0
#define motorCtrlTempID 0x0A0
#define motorCtrlTempByte 6
#define motorTempLength 6  // Motor temperature ranges from -3276.8 to +3276.7 C
#define motorCtrlTempLen 6

#define dcBusCurrentID 0x0A6
#define dcBusCurrentByte 6
#define dcBusCurrentLen 6  // DC Bus current ranges from -3276.8 to +3276.7 Amps

#define motorTorqueID 0x0AC
#define motorTorqueByte 0
#define motorTorqueLen 6  // Motor torque ranges from -3276.8 to +3276.7 N-m

#define highVoltageID 0x6B3
#define highVoltageByte 2
#define lowVoltageID 0x6B3
#define lowVoltageByte 4
#define voltageLength 5
#define imuLength 9

#define RPM_ID 0x0A5
#define RPM_BYTE 2
#define RPM_LEN 6  // due to possible negative, will have leading 0 if positive

/* CAN variables */
extern bool rxMsg;
extern bool errFlag;
extern uint32_t msgCount;

// CAN data struct
typedef struct {  // Multiplication factors (units) from the BMS utility manual
  uint8_t SOC[SOC_length];       // 0.5 (%)
  uint8_t FPV[FPV_length];       // 0.1 (V)
  uint8_t highTemp[tempLength];  // 1 (degrees C) (BOLT3 data indicates 0.1)
  uint8_t lowTemp[tempLength];   // 1 (degrees C) (BOLT3 data indicates 0.1)
  uint8_t highVoltage[voltageLength];  // 0.0001 (V)
  uint8_t lowVoltage[voltageLength];   // 0.0001 (V)
  uint8_t RPM[RPM_LEN];                // 1 (rpm)
  uint8_t motorTemp[motorTempLength];
  uint8_t motorCtrlTemp[motorCtrlTempLen];
  uint8_t motorTorque[motorTorqueLen];
  uint8_t dcBusCurrent[dcBusCurrentLen];
} CANTransmitData_t;

void CANSendData(int id, int data);  // send AUX battery voltage over CAN
void CANSetup(tCANMsgObject* message);
void configureCAN();
void CANReceive(tCANMsgObject* sCANMessage, CANTransmitData_t* CANdata,
                uint8_t msgDataIndex, uint8_t* msgData);
void CAN0_IRQHandler(void);
