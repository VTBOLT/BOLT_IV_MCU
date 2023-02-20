/******************************************************************************
 *
 * MSP432 BOLT IV MCU PROGRAM
 *
 * Authors:
 * William Campbell
 * Ethan Brooks
 * Patrick Graybeal
 * Logan Richardson
 *
 ******************************************************************************
 *
 * This file is the empty main for the BOLT MCU. See can_example.c for details
 * when implementing CAN functionality.
 *
 ******************************************************************************
 * Potentially useful functions:
 * GPIOPinTypeADC(uint32_t ui32Port, uint8_t ui8Pins)
 * GPIOPinTypeCAN(uint32_t ui32Port, uint8_t ui8Pins)
 * GPIOPinTypeGPIOInput(uint32_t ui32Port, uint8_t ui8Pins)
 * GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
 *
 ******************************************************************************
 * Notes:
 * Must include "MAP_" before driverlib function calls, I don't know why, but
 *   otherwise it doesn't actually see the function and doesn't compile.
 *
 * So that UART2 is connected to the XDS110 and CAN0 is enabled to RX/TX,
 *   the jumpers on the MCU must be horizontal (see silkscreen graphic)
 *
 ******************************************************************************/

#include "msp.h"

/* Standard driverlib include - can be more specific if needed */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#include "can.h"
#include "uart.h"
#include "uartstdio.h"
#include "helper.h"

#define UART_2_EN
// #define XBEE_PLACEHOLDER_DATA

#define IGNIT_CUTOFF_DELAY 70
#define XBEE_BAUD_RATE 57600

/* Configure system clock for 120 MHz */
uint32_t systemClock;

// IMU data struct
typedef struct {
  uint8_t xAcc[imuLength];
  uint8_t yAcc[imuLength];
  uint8_t zAcc[imuLength];
  uint8_t xGyro[imuLength];
  uint8_t yGyro[imuLength];
  uint8_t zGyro[imuLength];
  uint8_t pitch[imuLength];
  uint8_t roll[imuLength];
  uint8_t yaw[imuLength];
  uint8_t compass[imuLength];
} IMUTransmitData_t;

// Instantiate IMU object
IMUTransmitData_t gIMUData;

// IMU receive buffer
char gIMUReceiveBuf[imuLength];
uint32_t gIMUBufIndex = 0;

// Pump Voltage
uint8_t pumpVoltage[voltageLength];

// AUX pack voltage
uint8_t auxVoltage[voltageLength];

// Global variable to count milliseconds
uint8_t msCount = 0;

// ISR Flags
uint8_t g_ui8xbeeFlag = 0;
uint32_t g_ui32Flags;

/* Temp */
uint8_t CANCount = 0;
uint8_t g_ui8canFlag = 0;

// Timer 1 flag
uint8_t intTimer1_flag = 0;

// Debounce counter
uint32_t debounceCounter = 0;

typedef enum states { PCB, ACC, IGN, MAX_STATES } states_t;
typedef enum ignitState { IGNIT_OFF = 0, IGNIT_ON = 1 } ignitState_t;
// Required second count to delay switching off ACC and IGN if voltage dips
// below for a brief second. We want to calculate the # of cycles to delay x
// seconds from the formula below: x seconds delay = (# of Cycles) *
// (1/frequency) So plug in the second count that you want delayed into the
// formula below: (120MHz)*(x second delay) = # of cycles The current value is
// for a 3 second delay: (120MHz)*(3s)=360000000
#define REQSECCOUNT 360000000

// Function prototypes
void ADCSetup();
uint32_t auxADCSend(uint32_t* auxBatVoltage);
uint32_t pumpADCSend(uint32_t* pumpVoltage);
void accIgnDESetup(void);
void timerSetup();
void timerRun();
bool ignitDebounce(bool, uint32_t*, uint8_t*);
bool ignitPoll(void);
bool accPoll(void);
bool DEPoll(void);
void UART6_IRQHandler(void);
// This function can handle signed and unsigned from -32767 to +32767
void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num);
void initTimers();
void TIMER1A_IRQHandler();
void xbeeTransmit(CANTransmitData_t, IMUTransmitData_t, uint8_t*, uint8_t*);
void imuParse(char c);

int main(void) {
  /* Configure system clock for 120 MHz */
  systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                       120000000);
  // Set up the timer system
  timerSetup();
  initTimers(systemClock);

  // Instantiate CAN objects
  static CANTransmitData_t CANData;
  tCANMsgObject sCANMessage;
  uint8_t msgDataIndex;
  uint8_t msgData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Initialize IMU object
  memset(gIMUData.xAcc, '0', imuLength);
  memset(gIMUData.yAcc, '0', imuLength);
  memset(gIMUData.zAcc, '0', imuLength);
  memset(gIMUData.xGyro, '0', imuLength);
  memset(gIMUData.yGyro, '0', imuLength);
  memset(gIMUData.zGyro, '0', imuLength);
  memset(gIMUData.roll, '0', imuLength);
  memset(gIMUData.pitch, '0', imuLength);
  memset(gIMUData.yaw, '0', imuLength);
  memset(gIMUData.compass, '0', imuLength);

  memset(auxVoltage, '0', sizeof(auxVoltage));
  uint32_t auxBatVoltage[1];
  uint32_t auxBatAdjusted;  // no decimal, accurate value

  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))) ADCSetup();
  UART7Setup(systemClock);
  UART6Setup(systemClock);
  accIgnDESetup();
  configureCAN(systemClock);
  CANSetup(&sCANMessage);

  // Enable interrupts globally
  MAP_IntMasterEnable();

  enableUARTprintf(systemClock);
  UARTprintf("UARTprintf enabled\n");

  states_t present = PCB;

#ifdef XBEE_PLACEHOLDER_DATA
  strncpy((char*)CANData.SOC, "50", sizeof(CANData.SOC));
  strncpy((char*)CANData.FPV, "300", sizeof(CANData.FPV));
  strncpy((char*)CANData.highTemp, "80", sizeof(CANData.highTemp));
  strncpy((char*)CANData.lowTemp, "70", sizeof(CANData.lowTemp));
  strncpy((char*)CANData.highVoltage, "6.8", sizeof(CANData.highVoltage));
  strncpy((char*)CANData.lowVoltage, "6.4", sizeof(CANData.lowVoltage));
  strncpy((char*)IMUData.xAcc, "1.5", sizeof(IMUData.xAcc));
  strncpy((char*)IMUData.yAcc, "1.2", sizeof(IMUData.yAcc));
  strncpy((char*)IMUData.zAcc, "1", sizeof(IMUData.zAcc));
  strncpy((char*)IMUData.xGyro, "20", sizeof(IMUData.xGyro));
  strncpy((char*)IMUData.yGyro, "3", sizeof(IMUData.yGyro));
  strncpy((char*)IMUData.zGyro, "0", sizeof(IMUData.zGyro));
  strncpy((char*)pumpVoltage, "12", sizeof(pumpVoltage));
  strncpy((char*)auxVoltage, "16", sizeof(auxVoltage));
#endif

  // UARTprintf("Entering loop");
  // Loop forever.
  while (1) {
    if (g_ui8xbeeFlag) {
      xbeeTransmit(CANData, gIMUData, pumpVoltage, auxVoltage);
      g_ui8xbeeFlag = 0;
    }

    // As long as the PCB is on, CAN should be read
    if (g_ui8canFlag) {
      CANSendData(31, auxADCSend(auxBatVoltage));
      CANReceive(&sCANMessage, &CANData, msgDataIndex, msgData);
      g_ui8canFlag = 0;
      // UARTprintf("Compare value: %i\n", auxADCSend(auxBatVoltage));
      // UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));
    }

    switch (present) {
      /* ACC state of FSM */
      case ACC:

        // Output HIGH to ACC Relay
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
        // Output HIGH to ACC Dash
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);

        // Output LOW to IGN Relay
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
        // Output LOW to IGN Dash
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

        // Output HIGH to PSI LED
        MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);
        // Output HIGH to PSI Dash
        MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);

        // Aux battery voltage stored as volts * 1000
        auxBatAdjusted = auxADCSend(auxBatVoltage);

        // UARTprintf("Aux Bat: %i", auxBatAdjusted);
        // UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));

        if (auxBatAdjusted <= 1200) {
          //  Wait 3 seconds to check value, ensure constant value
          timerRun();

          auxBatAdjusted = auxADCSend(auxBatVoltage);
          if (auxBatAdjusted <= 1200) {
            present = PCB;
          }

        } else if (!accPoll()) {
          present = PCB;

        } else if (ignitPoll()) {
          present = IGN;
        }
        break;

      /* IGN state of FSM */
      case IGN:

        if (!DEPoll()) {
          // Output HIGH to ACC Relay
          MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);
          // Output HIGH to ACC Dash
          MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);

          // Output HIGH to IGN Relay
          MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0);
          // Output HIGH to IGN Dash
          MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);

          // Output HIGH to PSI LED
          MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);
          // Output HIGH to PSI LED
          MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);

          auxBatAdjusted = auxADCSend(auxBatVoltage);

          // UARTprintf("Pump: %i", pumpADCSend(pumpVoltage));

          if (auxBatAdjusted <= 1200) {
            //  Wait 3 seconds to check value, ensure constant value
            timerRun();

            auxBatAdjusted = auxADCSend(auxBatVoltage);
            if (auxBatAdjusted <= 1200) {
              present = ACC;
            }

            //} else if (/*Low pump current*/) {

            // present = ACC;

          } else if (!ignitPoll()) {
            present = ACC;

          } else if (!accPoll()) {
            present = ACC;
          }

        } else {
          present = ACC;
        }
        break;

      /* PCB state of FSM */
      default:

        // Output LOW to ACC Relay
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~GPIO_PIN_4);
        // Output LOW to ACC Dash
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);

        // Output LOW to IGN Relay
        MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
        // Output LOW to IGN Dash
        MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);

        // Output LOW to PSI LED
        MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
        // Output LOW to PSI Dash
        MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

        if (accPoll()) {
          present = ACC;
        }
        break;
    }
  }
}

void accIgnDESetup(void) {
  // ACC K6, IGN K7, PSI M2
  /* Enables pins for ACC Tx/Rx, IGN Tx/Rx, ACC/IGN relays and reading from BMS
   * Discharge Enable  */

  /* Enable clock to peripherals used (H, K, M, P, Q) */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    ;
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)) {
  };

  // Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

  /*Enable the GPIO Pins for Ignition and Accessory Tx as outputs.
  ( PM6 and PQ1, respectively) */
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_6);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_1);

  /* Set them to HIGH */
  MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_PIN_6);
  MAP_GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_PIN_1);

  /* Enable the GPIO Pins for the Ignition and Accessory relays as outputs.
  (PH0 and PK4, respectively) */
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_0);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);

  /* Then, set them to LOW */
  MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
  MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, ~GPIO_PIN_4);

  /* Enable the GPIO Pins for the Accessory, Ignition, and PSI Dash outputs.
  (PK6, PK7, PM2, respectively) */
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);

  /* Then set them to LOW */
  MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, ~GPIO_PIN_6);
  MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, ~GPIO_PIN_7);
  MAP_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

  /* Enable the GPIO pins for Ignition and Accessory Rx as inputs.
  (PP3 and PH1, respectively) */
  MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_3);
  MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

  /* Enable the GPIO pin for BMS Discharge Enable as input.
  (PP2) */
  MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_1);

  /* Then, enable the MCU's pull-down resistors on each to prevent noise */
  GPIOP->PDR |= GPIO_PIN_3;
  GPIOH->PDR |= GPIO_PIN_1;

  /* Enable MCU's pull-up resistor on PM1 for reading from BMS Discharge Enable
   */
  GPIOM->PUR |= GPIO_PIN_1;
}

bool accPoll(void) {
  /* Return TRUE if acc Rx reads HIGH and FALSE if acc Rx reads LOW.
     As a note: ACC Rx: PH1, ACC Relay: PK4 */

  if (MAP_GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
    return true;
  } else {
    return false;
  }
}

bool ignitDebounce(bool btn, uint32_t* count, uint8_t* flag) {
  static ignitState_t ignitState = IGNIT_OFF;
  static uint32_t bounceCount =
      0;  // count number of ms that ignition switch has bounced
  static bool bounceFlag = false;

  switch (ignitState) {
    case IGNIT_OFF:
      if (btn == true) {
        ignitState = IGNIT_ON;
      }
      // count number of times the ignition switch is bouncing
      if (bounceFlag) {
        bounceCount++;
        bounceFlag = false;
      }
      break;

    case IGNIT_ON:
      if (*flag) {
        *flag = 0;
        if (btn == false) {
          bounceFlag = true;
          if (*count > IGNIT_CUTOFF_DELAY) {
            *count = 0;
            ignitState = IGNIT_OFF;
          } else {
            (*count)++;
          }
        } else {
          *count = 0;  // reset counter if ignition switch turns back on
        }
      }
      break;
  }
  // UARTprintf("%d\n", *count);
  return ignitState;
}

bool ignitPoll(void) {
  /* Return TRUE if ignition Rx reads HIGH and FALSE if ignition Rx reads LOW
     As a note: IGN Rx: PP3, IGN Relay: PH0 */

  //    if (  MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
  //        return true;
  //    } else {
  //        return false;
  //    }
  return ignitDebounce((bool)(MAP_GPIOPinRead(GPIO_PORTP_BASE, GPIO_PIN_3)),
                       &debounceCounter, &intTimer1_flag);
}

bool DEPoll(void) {
  /* Poll if the bike is able to be discharged.
     Return TRUE if so, return FALSE otherwise. */

  /* DE is read from PM1. Pull-up resistor is enabled on PM1 since
     the BMS grounds PM1 when the bike can discharge. */
  if (MAP_GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
    return false;
  } else {
    return true;
  }
}

// Handles interrupts by reading in data from UART
void UART6_IRQHandler(void) {
  // UARTprintf("Entered UART6 ISR\n");
  uint32_t ui32Status;

  // Get the interrupt status.
  ui32Status = MAP_UARTIntStatus(UART6_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  MAP_UARTIntClear(UART6_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  while (MAP_UARTCharsAvail(UART6_BASE)) {
    char c = MAP_UARTCharGetNonBlocking(UART6_BASE);
    // UARTprintf(c);
    // MAP_UARTCharPutNonBlocking(UART0_BASE, c);
    imuParse(c);
  }
}

void ADCSetup() {
  /* AUX ADC SETUP - built using adc0_singleended_singlechannel_singleseq */

  /* Enable the clock to GPIO Ports E & D and wait for it to be ready */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))) {
  };

  /* Configure PE0 as ADC input channel */
  MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);  // aux battery
  MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);  // pump

  /* Enable the clock to ADC0 and ADC1 and wait for it to be ready */
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))) {
  };
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))) {
  };

  /* Configure Sequencer 3 to sample a single analog channel: AIN3 */
  MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                               ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
  /* Configure sequencer 3 on ADC1 */
  MAP_ADCSequenceStepConfigure(ADC1_BASE, 3, 0,
                               ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

  /* Configure and enable sample sequence 3 with a processor signal trigger */
  MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  MAP_ADCSequenceEnable(ADC0_BASE, 3);
  MAP_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  MAP_ADCSequenceEnable(ADC1_BASE, 3);

  /* Clear interrupt status flag */
  MAP_ADCIntClear(ADC0_BASE, 3);
  MAP_ADCIntClear(ADC1_BASE, 3);
}

uint32_t auxADCSend(uint32_t* auxBatVoltage) {
  /* BUG
   * @ 12 V in, this function outputs 1120 (11.2V)
   * This is safe so long as we're underestimating
   * Consider for future revisions
   */

  /* AUX ADC */
  MAP_ADCProcessorTrigger(ADC0_BASE, 3);
  while (!MAP_ADCIntStatus(ADC0_BASE, 3, false)) {
  }
  MAP_ADCIntClear(ADC0_BASE, 3);
  MAP_ADCSequenceDataGet(ADC0_BASE, 3, auxBatVoltage);

  float tempFloat = auxBatVoltage[0];

  // From ((v/1000)/1.265)/.1904 - see spreadsheet
  // tempFloat *= 0.004719;
  // tempFloat *= 0.004104;
  // uint32_t temp = tempFloat * 100;
  // uint32_t toReturn = temp;

  uint32_t tempTrueVoltage = (tempFloat / 218.587) * 100;  // see spreadsheet
  uint32_t compareVoltage = tempTrueVoltage;

  // UARTprintf("Raw Value: %i\n", auxBatVoltage[0]);
  // UARTprintf("Adjusted Value: %i\n", tempTrueVoltage);

  convertToASCII(auxVoltage, voltageLength - 1, tempTrueVoltage);

  // Adjust for decimal point in the middle to transmit
  auxVoltage[4] = auxVoltage[3];
  auxVoltage[3] = auxVoltage[2];
  auxVoltage[2] = '.';

  /*
  uint32_t tempToTransmit = auxBatVoltage[0] * .461;
  UARTprintf("Aux bat to transmit: %i\n", tempToTransmit);


  asciiChars[4] = tempToTransmit % 10 + '0';
  tempToTransmit /= 10;
  asciiChars[3] = tempToTransmit % 10 + '0';
  tempToTransmit /= 10;
  asciiChars[2] = '.';
  asciiChars[1] = tempToTransmit % 10 + '0';
  tempToTransmit /= 10;
  asciiChars[0] = tempToTransmit % 10 + '0';
  */

  // UARTprintf("Put into chars: %c%c%c%c%c\n", auxVoltage[0], auxVoltage[1],
  // auxVoltage[2], auxVoltage[3], auxVoltage[4]);

  /*
  uint8_t i = 0;
  for ( ; i < 5; i++) {
      auxVoltage[i] = asciiChars[i];
  }
  */

  return compareVoltage;
}

uint32_t pumpADCSend(uint32_t* pumpVoltage) {
  /* AUX ADC */
  MAP_ADCProcessorTrigger(ADC1_BASE, 3);
  while (!MAP_ADCIntStatus(ADC1_BASE, 3, false)) {
  }
  MAP_ADCIntClear(ADC1_BASE, 3);
  MAP_ADCSequenceDataGet(ADC1_BASE, 3, pumpVoltage);
  return pumpVoltage[0];
}

void initTimers(uint32_t sysClock) {
  // Enable the peripherals used by this example.
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

  // Configure timer 1
  MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, sysClock / 1000);

  // Setup the interrupts for the timer timeouts.
  MAP_IntEnable(INT_TIMER1A);
  MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

  // Enable the timers.
  MAP_TimerEnable(TIMER1_BASE, TIMER_A);
}

void timerSetup() {
  // Set the 32-bit timer Peripheral.
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  // Configure the timer to be one-shot.
  MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
}

void timerRun() {
  // Load the required second count into the timer.
  MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, REQSECCOUNT);

  // Enable the timer.
  MAP_TimerEnable(TIMER0_BASE, TIMER_A);

  // Wait for 3 seconds to have the timer delay the system
  while (MAP_TimerValueGet(TIMER0_BASE, TIMER_A) != REQSECCOUNT) {
  }
}

void xbeeTransmit(CANTransmitData_t CANData, IMUTransmitData_t IMUData,
                  uint8_t* pumpVoltage, uint8_t* auxVoltage) {
  // Send CAN Data
  UARTSendStr(UART7_BASE, CANData.SOC, sizeof(CANData.SOC));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.FPV, sizeof(CANData.FPV));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.highTemp, sizeof(CANData.highTemp));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.lowTemp, sizeof(CANData.lowTemp));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.highVoltage, sizeof(CANData.highVoltage));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.lowVoltage, sizeof(CANData.lowVoltage));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.RPM, sizeof(CANData.RPM));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.motorTemp, sizeof(CANData.motorTemp));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.dcBusCurrent, sizeof(CANData.dcBusCurrent));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.motorTorque, sizeof(CANData.motorTorque));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, CANData.motorCtrlTemp, sizeof(CANData.motorCtrlTemp));
  UARTSendChar(UART7_BASE, ',');

  // Send AUX pack voltage
  UARTSendStr(UART7_BASE, auxVoltage, sizeof(auxVoltage));
  UARTSendChar(UART7_BASE, ',');

  // Send IMU Data
  UARTSendStr(UART7_BASE, IMUData.xAcc, sizeof(IMUData.xAcc));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.yAcc, sizeof(IMUData.yAcc));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.zAcc, sizeof(IMUData.zAcc));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.xGyro, sizeof(IMUData.xGyro));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.yGyro, sizeof(IMUData.yGyro));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.zGyro, sizeof(IMUData.zGyro));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.roll, sizeof(IMUData.roll));
  UARTSendChar(UART7_BASE, ',');
  UARTSendStr(UART7_BASE, IMUData.pitch, sizeof(IMUData.pitch));
}

void imuParse(char c) {
#define DEFAULT 0
  enum {
    ACCEL = 'a',
    GYRO = 'g',
    EULER = 'e',
    ACCEL_X = 'a' + 'x',
    ACCEL_Y = 'a' + 'y',
    ACCEL_Z = 'a' + 'z',
    GYRO_X = 'g' + 'x',
    GYRO_Y = 'g' + 'y',
    GYRO_Z = 'g' + 'z',
    EULER_R = 'e' + 'r',
    EULER_Y = 'e' + 'y',
    EULER_P = 'e' + 'p',
    COMPASS = 'c'
  };
  static int state = DEFAULT;  // initial state will be default state

  switch (state) {
    case ACCEL:
      state = 'a' + c;
      break;

    case ACCEL_X:
      memcpy(gIMUData.xAcc, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Accel-X\n\r");
      state = DEFAULT;
      break;

    case ACCEL_Y:
      memcpy(gIMUData.yAcc, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Accel-Y\n\r");
      state = DEFAULT;
      break;

    case ACCEL_Z:
      memcpy(gIMUData.zAcc, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Accel-Z\n\r");
      state = DEFAULT;
      break;

    case GYRO:
      state = 'g' + c;
      break;

    case GYRO_X:
      memcpy(gIMUData.xGyro, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Gyro-X\n\r");
      state = DEFAULT;
      break;

    case GYRO_Y:
      memcpy(gIMUData.yGyro, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Gyro-Y\n\r");
      state = DEFAULT;
      break;

    case GYRO_Z:
      memcpy(gIMUData.zGyro, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Gyro-Z\n\r");
      state = DEFAULT;
      break;

    case EULER:
      state = 'e' + c;
      break;

    case EULER_R:
      memcpy(gIMUData.roll, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Roll\n\r");
      state = DEFAULT;
      break;

    case EULER_Y:
      memcpy(gIMUData.yaw, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Yaw\n\r");
      state = DEFAULT;
      break;

    case EULER_P:
      memcpy(gIMUData.pitch, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Pitch\n\r");
      state = DEFAULT;
      break;

    case COMPASS:
      memcpy(gIMUData.compass, gIMUReceiveBuf, imuLength);
      memset(gIMUReceiveBuf, '0', imuLength);
      gIMUBufIndex = 0;
      // UARTprintf(" -- Compass\n\r");
      state = DEFAULT;
      break;

    default:
      if ((c >= 0x30 && c <= 0x39) || c == '.' ||
          c == '-') {  // Check if the character is an ASCII number
        // Protect against buffer overflow
        if (gIMUBufIndex >= imuLength) {
          memset(gIMUReceiveBuf, '0', imuLength);
          // UARTprintf("Buffer overflow detected\n\r");
          gIMUBufIndex = 0;
        } else {
          gIMUReceiveBuf[gIMUBufIndex++] = c;
          // UARTprintf("%c", c);
        }
      } else {
        state = c;
      }
      break;
  }
}


void TIMER1A_IRQHandler() {
  // Clear the timer interrupt.
  intTimer1_flag = 1;
  MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

  // toggle LED every 0.05 seconds
  if (CANCount >= 50) {
    g_ui8canFlag = 1;
    CANCount = 0;
  }
  if (msCount >= 250) {
    // Toggle the flag for the first timer.
    HWREGBITW(&g_ui32Flags, 1) ^= 1;
    g_ui8xbeeFlag = 1;

    if (g_ui32Flags) {
      MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
    } else {
      MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
    }

    msCount = 0;
  }
  msCount++;
  CANCount++;
}
