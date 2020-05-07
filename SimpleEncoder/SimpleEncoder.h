// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Button.
// Supports Click and Hold functions.
// 
// Timer-based rotary encoder logic by Brad George ©2019-2020
// ----------------------------------------------------------------------------

#ifndef __SimpleEncoder_h__
#define __SimpleEncoder_h__

// ----------------------------------------------------------------------------

#define ENC_BUTTONINTERVAL    10  //  Check button every x milliseconds, also debouce time.
#define ENC_HOLDTIME        1200  //  Report held state after 1.2s.

// ----------------------------------------------------------------------------

#define ENC_ACCEL              1  //  Comment out to disable acceleration.
#define ENC_ACCELRATE        128  //  Accel speed climbing rate. Maximum 255.
#define ENC_DECELRATE          2  //  Decel per millisecond.
#define ENC_ACCELMAX        2560  //  10x (ENC_ACCELMAX >> 8 = 10)

// ----------------------------------------------------------------------------

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"


class SimpleEncoder
{
public:
  enum eBtnState {
    bsOpen = 0,
    bsHeld,
    bsReleased,
    bsClicked
  };

public:
  SimpleEncoder(uint8_t pinLeft, uint8_t pinRight, uint8_t pinBtn, bool bActive = LOW);

  void isr_DoPolling(void);  
  int8_t iGetValue(void);
  eBtnState bsGetButton(void);

private:
  //  The code this class was based on had uiHoldTicks & ulLastBtnCheck as "static" declarations inside
  //  of the isr_DoPolling() method. This caused issues when trying to use more than one instance of
  //  this class at a time. Declaring them here and initializing in the constructor solved the issue.
  uint16_t uiHoldTicks;
  unsigned long ulLastBtnCheck;
  const uint8_t uPinLeft;
  const uint8_t uPinRight;
  const uint8_t uPinBtn;
  const bool bActiveState;
  volatile int8_t iDelta;
  volatile eBtnState bsButton;

#ifdef ENC_ACCEL
  volatile int16_t iAccel;
#endif
};

// ----------------------------------------------------------------------------

#endif // __SimpleEncoder_h__
