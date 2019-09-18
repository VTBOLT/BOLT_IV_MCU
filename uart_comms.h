#ifndef UART_COMMS_INCLUDED
#define UART_COMMS_INCLUDED

#define UART_2_EN
//#define XBEE_PLACEHOLDER_DATA


//Function Prototypes

//void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
void UARTSendChar(uint32_t, const char);
void UARTSendCharNonBlocking(uint32_t, const char);
void UARTSendStr(uint32_t, const uint8_t*, uint32_t);
void UARTSendStrNonBlocking(uint32_t, const uint8_t*, uint32_t);
//void UART_SendComma();
void UART7Setup();
void UART6Setup(void);
void UART6_IRQHandler(void);
void enableUARTprintf();


#endif
