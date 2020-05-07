// ----------------------------------------------------------------------------
// Simple Timer-Polled Button Library.
// Supports Click and Hold functions.
// 
// Timer-based button logic by Brad George ©2019-2020
// ----------------------------------------------------------------------------

#include "SimpleButton.h"

// ----------------------------------------------------------------------------

SimpleButton::SimpleButton(uint8_t pinBtn, uint16_t holdTime, bool bActive)
  : bsButton(bsOpen), uiPinBtn(pinBtn), uiHoldTime(holdTime), bActiveState(bActive), uiHoldTicks(0), ulLastBtnCheck(0)
{
  uint8_t uiInputType = (bActiveState == LOW) ? INPUT_PULLUP : INPUT;
  pinMode(uiPinBtn, uiInputType);
}

// ----------------------------------------------------------------------------
// Call this every 1 millisecond via timer ISR.
//
void SimpleButton::isr_DoPolling(void)
{
  //  Get current tick count.
  unsigned long ulNow = millis();

  //  Handle button presses.
  //  Only check button if the pin is defined and the interval has ellapsed. (Once for every 10ms.)
  if (uiPinBtn > 0 && (ulNow - ulLastBtnCheck) >= BTN_INTERVAL) { 
    ulLastBtnCheck = ulNow;
    
    if (digitalRead(uiPinBtn) == bActiveState) { //  Key is down.
      if (bsButton != bsHeld) { uiHoldTicks++; }
      if (uiHoldTicks > (uiHoldTime / BTN_INTERVAL)) {
        bsButton = bsHeld;
      }
    }

    if (digitalRead(uiPinBtn) != bActiveState) { // key is now up
      if (uiHoldTicks) { //  This is only >0 if the button has been held longer than the check interval. 
        if (bsButton == bsHeld) {
          bsButton = bsReleased;
        } else {
          bsButton = bsClicked;
        }
        
        uiHoldTicks = 0;
      }
    }
  }
}

// ----------------------------------------------------------------------------

SimpleButton::eBtnState SimpleButton::bsGetButton(void)
{
  cli();
  SimpleButton::eBtnState bsRetVal = bsButton;
  if (bsButton != SimpleButton::bsHeld) {
    bsButton = SimpleButton::bsOpen; //  Reset button state.
  }
  sei();

  return bsRetVal;
}
