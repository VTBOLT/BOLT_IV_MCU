#ifndef UART_COMMS_INCLUDED
#define UART_COMMS_INCLUDED

#define UART_2_EN
//#define XBEE_PLACEHOLDER_DATA
#define IGNIT_CUTOFF_DELAY  70
#define IMU_RECEIVE_BAUD 57600
#define XBEE_BAUD_RATE 57600


//Function Prototypes

//void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void UARTSendCharNonBlocking(uint32_t, const char);
void UARTSendStrNonBlocking(uint32_t, const uint8_t*, uint32_t);
void UARTSendStr(uint32_t, const uint8_t*, uint32_t);
void UARTSendChar(uint32_t, const char);
//void UART_SendComma();
void UART6Setup(void);
void UART7Setup();
void UART6_IRQHandler(void);
void enableUARTprintf();




#endif
