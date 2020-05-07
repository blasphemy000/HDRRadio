// ----------------------------------------------------------------------------
// Simple Timer-Polled Button Library.
// Supports Click and Hold functions.
// 
// Timer-based button logic by Brad George ©2019-2020
// ----------------------------------------------------------------------------

#ifndef __SimpleButton_h__
#define __SimpleButton_h__

// ----------------------------------------------------------------------------

#define BTN_INTERVAL    10  //  Check button every x milliseconds, also debouce time.
//#define BTN_HOLDTIME  1200  //  Report held state after 1.2s. (Replaced with uiHoldTime constant).

// ----------------------------------------------------------------------------

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"


class SimpleButton
{
public:
  enum eBtnState {
    bsOpen = 0,
    bsHeld,
    bsReleased,
    bsClicked
  };

public:
  SimpleButton(uint8_t pinBtn, uint16_t holdTime = 1200, bool bActive = LOW);

  void isr_DoPolling(void);  
  eBtnState bsGetButton(void);

private:
  //  The code this class was based on had uiHoldTicks & ulLastBtnCheck as "static" declarations inside
  //  of the isr_DoPolling() method. This caused issues when trying to use more than one instance of
  //  this class at a time. Declaring them here and initializing in the constructor solved the issue.
  uint16_t uiHoldTicks;
  unsigned long ulLastBtnCheck;
  const uint8_t uiPinBtn;
  const uint16_t uiHoldTime;
  const bool bActiveState;
  volatile eBtnState bsButton;
};

// ----------------------------------------------------------------------------

#endif // __SimpleButton_h__
