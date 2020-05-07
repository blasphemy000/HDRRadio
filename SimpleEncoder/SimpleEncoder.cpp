// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Button.
// Supports Click and Hold functions.
// 
// Timer-based rotary encoder logic by Brad George ©2019-2020
// ----------------------------------------------------------------------------

#include "SimpleEncoder.h"

// ----------------------------------------------------------------------------

SimpleEncoder::SimpleEncoder(uint8_t pinLeft, uint8_t pinRight, uint8_t pinBtn, bool bActive)
  : iDelta(0), bsButton(bsOpen), uPinLeft(pinLeft), uPinRight(pinRight),
    uPinBtn(pinBtn), bActiveState(bActive), uiHoldTicks(0), ulLastBtnCheck(0)
{
  uint8_t uiInputType = (bActiveState == LOW) ? INPUT_PULLUP : INPUT;
  pinMode(uPinLeft, uiInputType);
  pinMode(uPinRight, uiInputType);
  pinMode(uPinBtn, uiInputType);
}

// ----------------------------------------------------------------------------
// Call this every 1 millisecond via timer ISR.
//
void SimpleEncoder::isr_DoPolling(void)
{
  //  Get current tick count.
  unsigned long ulNow = millis();

#ifdef ENC_ACCEL
  iAccel -= ENC_DECELRATE;
  if (iAccel & 0x8000) { iAccel = 0; }
#endif

  //  Handle encoder movement.
  //static unsigned long ulLastEncCheck = 0;
  static uint8_t uiMovedBits = 0;
  //  Bit0 = Moved; Bit1 = Direction;
  //      Moved Bit0: 0 = Nope; 1 = Moved;
  //  Direction Bit1: 0 = Left; 1 = Right;

  if (uiMovedBits == 1) {
    if (digitalRead(uPinLeft) != bActiveState) {
      iDelta = -1;
      uiMovedBits = 0;
#ifdef ENC_ACCEL
      if (iAccel <= (ENC_ACCELMAX - ENC_ACCELRATE)) { iAccel += ENC_ACCELRATE; }
#endif
    }
  } else if (uiMovedBits == 3) {
    if (digitalRead(uPinRight) != bActiveState) {
      iDelta = 1;
      uiMovedBits = 0;
#ifdef ENC_ACCEL
      if (iAccel <= (ENC_ACCELMAX - ENC_ACCELRATE)) { iAccel += ENC_ACCELRATE; }
#endif
    }
  } else {
    if (digitalRead(uPinLeft) == bActiveState) {
      uiMovedBits = 1;  //  Bits: 01; Left + Moved.
    }

    if (digitalRead(uPinRight) == bActiveState) {
      uiMovedBits = 3;  //  Bits: 11; Right + Moved.
    }
  }


  //  Handle button presses.
  //  Only check button if the pin is defined and the interval has ellapsed. (Once for every 10ms.)
  if (uPinBtn > 0 && (ulNow - ulLastBtnCheck) >= ENC_BUTTONINTERVAL) { 
    ulLastBtnCheck = ulNow;
    
    if (digitalRead(uPinBtn) == bActiveState) { //  Key is down.
      if (bsButton != bsHeld) { uiHoldTicks++; }
      if (uiHoldTicks > (ENC_HOLDTIME / ENC_BUTTONINTERVAL)) {
        bsButton = bsHeld;
      }
    }

    if (digitalRead(uPinBtn) != bActiveState) { // key is now up
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

int8_t SimpleEncoder::iGetValue(void)
{
  int8_t iRetVal;
  
  cli();  //  Temporarily disable interrupts.
    iRetVal = iDelta;
    iDelta = 0;
  sei();  //  Re-enable interrupts.
  
#ifdef ENC_ACCEL
  int16_t iRetAccel = (iAccel >> 8);

  if (iRetVal > 0) {
    iRetVal += iRetAccel;
  } else if (iRetVal < 0) {
    iRetVal -= iRetAccel;
  }
#endif

  return iRetVal;
}

// ----------------------------------------------------------------------------

SimpleEncoder::eBtnState SimpleEncoder::bsGetButton(void)
{
  cli();
  SimpleEncoder::eBtnState bsRetVal = bsButton;
  if (bsButton != SimpleEncoder::bsHeld) {
    bsButton = SimpleEncoder::bsOpen; //  Reset button state.
  }
  sei();

  return bsRetVal;
}
