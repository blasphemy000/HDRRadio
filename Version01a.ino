/*  Homebrew Superheterodyne Amature Radio Controller V0.1a by KC3MOP (Brad George).
 *  This version is designed to use a simple 4-wire rotary encoder (1 pulse/click, direction by which wire receives the pulses).
 *  Frequency synthesis is provided via an Adafruit Si5351 clock generator module. The display is an Adafruit 320x240 TFT LCD w/SD card.
 */


/*  TODO: Stuff that still needs to be implemented into this software for v0.1a.
 *  1.) Enable the outputs for the modes and PTT lines.
 *  2.) Decide how CW will be keyed. (Enable MOX and manual keying of a low power stage or something.)
 */


//  Define statements for different configuration options.
//#define HBR_USBDEBUG 1      //  USB Debugging Output. Currently not needed. Disabled to save program space.

#define HBR_SPLASH_SCREEN 1   //  Comment to disable.

#define TFT_DC 9              //  Data/Command Pin for TFT Display.
#define TFT_CS 10             //  Chip Select Pin for TFT Display.
#define TFT_ROTATION 3        //  For landscape display either 1 or 3 is used depending on mounting orientation.
//#define SD_CS xx            //  Chip Select Pin for SD Card. Currently not used.

//  User input pin configuration. All digital inputs are active LOW for ease of hardware implementation.
#define ENCODE_I A0           //  Encoder Pin 1
#define ENCODE_Q A1           //  Encoder Pin 2
#define VFO_BUTTON A2         //  VFO Button Pin | Click = Toggle A <-> B VFO. | Hold = Toggle split opperation between A/B.
#define STEP_BUTTON 3         //  Step Button Pin | Click = Cycle step size. | Hold = VFO tunes IF-Shift.
#define MODE_BUTTON 4         //  Mode Button Pin | Click = Cycle modes. | Hold = VFO changes band.
#define PTT_BUTTON 2          //  Push-to-Talk Button Pin

//  Control IO Lines.
#define POWER_IN A3           //  Analog input to read value of power adjustment knob. Might implement power sampling during transmit?
#define SSB_MODE_OUT 5        //  Active when in USB/LSB mode.
#define CW_MODE_OUT 6         //  Active when in CW mode.
#define AM_MODE_OUT 7         //  Active when in AM mode.
#define TX_MODE_OUT 8         //  Active to enable transmit. Low for receive.

//  Output active states for above outputs. Default = Active High.
//  These are configurable to allow for different hardware switching configurations.
//  Default usage would be to have the pin driving a small NPN transistor(2N3904) swtich
//  or a small MOSFET(2N7000) to activate the hardware switching circuitry. If the active
//  state is LOW, the pin can be used to drive a PNP transistor or P-Type MOSFET to do the switching.
#define SSB_ACTIVE HIGH       //  SSB_MODE_OUT state when active.
#define CW_ACTIVE HIGH        //  CW_MODE_OUT state when active.
#define AM_ACTIVE HIGH        //  AM_MODE_OUT state when active.
#define TX_ACTIVE HIGH        //  TX_MODE_OUT state when active.


//  Library inclusions.
#include <TimerOne.h>
#include <SimpleButton.h>
#include <SimpleEncoder.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Wire.h>
#include <si5351.h>


//  Functions with default parameters require prototypes.
void vRedrawVFOs(bool bSetup = false);
void vCycleMode(bool bDrawOnly = false);
void vUpdateRXLOs(bool bModeChange = false, bool bShiftIF = false);


//  Global Object Instances.
//  Button for Step size. Holding allows VFO to change IF Shift.
SimpleButton  sbStepButton = SimpleButton(STEP_BUTTON);
//  Button to change USB/LSB/CW/AM. Holding allows VFO knob to change band for selected VFO.
SimpleButton  sbModeButton = SimpleButton(MODE_BUTTON);
//  Push-to-Talk button on the microphone.
SimpleButton  sbPTTButton = SimpleButton(PTT_BUTTON, 250);
//  Encoder for VFO tuning. Includes a button to change the active VFO and enable/disable split functionality.
SimpleEncoder seEncoder = SimpleEncoder(ENCODE_I, ENCODE_Q, VFO_BUTTON);
//  This is the object instance for the TFT LCD display.
Adafruit_ILI9341 tDisp = Adafruit_ILI9341(TFT_CS, TFT_DC);
//  This is the object instance for the Si5351 clock generator.
Si5351 siFreqGen;


//  Frequency definitions.
//  Must cast frequencies to 64-bit when setting Si5351: (uint64_t)uiLocalOsc * SI5351_FREQ_MULT
//  These two "VFO" frequencies are actually the frequency that the radio is tuned to. The actual
//  frequency of the Local Oscilators are calculated based on the config specified here.
//  This allows flexibility to use almost any IF frequency(s). Just set them here and the outputs
//  of the Si5351 are calculated mathmatically on the fly. Details are below.
#define IF1FREQ 0               //  First IF Frequency. Center frequency of first IF crystal filter. 0 for single conversion.
#define IF1SHIFT 0              //  Amount to shift the carrier to get to IF1 filter's edge. Is 1/2 the filter bandwidth.
#define IF2FREQ 8830000         //  Final IF Frequency. Center frequency of second IF crystal filter.
#define IF2SHIFT 1500           //  Amount to shift the carrier to get to IF2 filter's edge. Is 1/2 the filter bandwidth.
#define CWTONE 700              //  Amount to shift the carrier when keying CW.
//  This (IF2OFFSET) allows the filters for IF1 and IF2 to have different bandwidths.
#define IF2OFFSET IF1SHIFT - IF2SHIFT


//  Global Variables.
//  Maximum allowable band ID. The 11m (CB) band and the nearby "freeband" is disabled by default. Set this to 7 to allow 11m+ tuning.
//  USE ON 11m, OR ANY FREQUENCY OUTSIDE OF THE AMATUER BANDS IS NOT LEGAL IN THE UNITED STATES!!! CHECK LOCAL LAWS IF YOU LIVE ELSEWHERE.
#define BANDMAX 7

//  TODO: Unfortunately band filters must be manually switched right now.
//  I need an MCU with more pins than an UNO, or a port expander/shift register.
//  Looking at using a xPCF8574x MCP23017 (half the price) from Microchip. It is a 16-Bit I2C port expander.
//  Could be used to drive filter switching outputs or all of the outputs except the PTT output line.

//  Band limit definitions. Third and forth columns are to save tuned frequency when changing bands back and forth.
//  { Lower Frequency Limit, Upper Frequency Limit, Last Tuned VFOA Frequency, Last Tuned VFOB Frequency }
uint32_t uiBands[8][4] = {
  { 1800000,  2000000,  1800000,  1800000}, //  160m (0)
  { 3500000,  4000000,  3500000,  3500000}, //   80m (1)
  { 7000000,  7300000,  7000000,  7000000}, //   40m (2)
//{10100000, 10150000, 10100000, 10100000}, //   30m (x) - Not implemented in my prototype hardware.
  {14000000, 14350000, 14000000, 14000000}, //   20m (3)
  {18068000, 18168000, 18068000, 18068000}, //   17m (4)
  {21000000, 21450000, 21000000, 21000000}, //   15m (5)
//{24890000, 24990000, 24890000, 24890000}, //   12m (x) - Not implemented in my prototype hardware.
  {28000000, 29700000, 28000000, 28000000}, //   10m (6)
  {26000000, 27999999, 27205000, 27205000}  //  +11m (7) - Disabled by default. ILLEGAL TO USE IN USA AND PROBABLY ELSEWHERE ALSO!
};

volatile bool bTransmitting = false;  //  True when transmitting.
int8_t   iBandA = 2;            //  Default VFOA to 40m band. (Can be changed.)
int8_t   iBandB = 3;            //  Default VFOB to 20m band. (Can be changed.)
uint32_t uiVFOAFreq = 7175000;  //  Default to bottom of the General class phone portion of 40m. (Can be changed.)
uint32_t uiVFOBFreq = 14225000; //  Default to bottom of the General class phone portion of 20m. (Can be changed.)
uint32_t uiIF1Freq = 0;         //  This is the first IF oscillator (second LO) in dual-conversion receiver mode. (Calculated. / CLK1)
uint32_t uiIF2Freq = 0;         //  This is the BFO for the product detector to down-convert to audio frequencies. (Calculated. / CLK2)
uint32_t uiLocalOsc = 0;        //  This is the main/first LO for either configuration. (Calculated. / CLK0)
uint16_t uiTuneStep = 100;      //  Default to 1KHz tuning steps. (Adds ten when update function called at startup.)
int16_t  iIFShift = 0;          //  How much to shift the VFO & IF to vary the passband of the crystal filter(s). (User control.)

//  For LO frequency calculations when IF1FREQ = 0. This is for a single conversion receiver.
//  Transmit modulation for SSB/AM and carrier injection for CW is done at IF2FREQ +/- the required shift in this mode.
//  Set IF2SHIFT positive for USB/CW, set negative for LSB. (Sidebands are inverted at first mixer stage.)
//  iIFShift is subtracted for USB/CW to allow for same bandwidth edge clipping when changing modes.
//  Example: IF = 8.83 MHz | IF2SHIFT 1.5 KHz for Filter = 3 kHz BW

//  Receive:
//  uiIF2Freq = IF2FREQ - IF2SHIFT + iIFShift   //  LSB Mode.
//  uiIF2Freq = IF2FREQ + IF2SHIFT - iIFShift   //  USB/CW Mode.

//  LocalOSC  = VFOFreq + uiIF2Freq

//  (IF FREQ) = LocalOSC - (RFIn)
//  (Audio)   = (IF FREQ) - uiIF2Freq

//  Transmit:
//  uiIF2Freq = IF2FREQ                     //  AM Mode.
//  uiIF2Freq = IF2FREQ - IF2SHIFT          //  LSB Mode.
//  uiIF2Freq = IF2FREQ + IF2SHIFT          //  USB Mode.
//  uiIF2Freq = IF2FREQ + IF2SHIFT - CWTONE //  CW Mode.

//  LocalOSC  = VFOFreq + uiIF2Freq           //  AM/LSB/USB Modes.
//  LocalOSC  = VFOFreq + IF2FREQ + IF2SHIFT  //  CW Mode Only. (This allows the IF to be shifted for the CW carrier w/o shifting the LO.)

//  (TXIF) = (Audio) +/- uiIF2Freq  //  "+/-" Means DSB Modulation. The IF2SHIFT causes the filter to chop off the unwanted sideband.
//  (TXRF) = LocalOSC - (TXIF)      //  Sideband inversion happens at this TX Mixer stage to produce the final output RF signal.

//  -------------------------------------------------------------------------------------------------------
//  For LO frequency calculations when IF1FREQ > 0. This would be for a dual conversion receiver.
//  Transmit modulation for SSB/AM and carrier injection for CW is done at IF1FREQ +/- the required shift in this mode.
//  Set IF2SHIFT positive for USB/CW, set negative for LSB. (Sidebands are inverted at first mixer stage.)
//  iIFShift essentially works as a "Width" control in double conversion mode.
//  Example: First IF = 10.7 Mhz | Second IF(BFO) = 455 KHz | IF1 & IF2 Filters = 3kHz BW

//  Receive:
//  uiIF2Freq = IF2FREQ - IF2SHIFT  //  LSB Mode.
//  uiIF2Freq = IF2FREQ + IF2SHIFT  //  USB/CW Mode.

//  uiIF1Freq = (IF1FREQ - IF2FREQ) - IF2OFFSET + iIFShift  //  LSB Mode.
//  uiIF1Freq = (IF1FREQ - IF2FREQ) + IF2OFFSET - iIFShift  //  USB/CW Mode.

//  LocalOSC  = uiIF2Freq + uiIF1Freq + VFOFreq

//  (IF1FREQ) = LocalOSC - (RFIn)
//  (IF2FREQ) = (IF1FREQ) - uiIF1Freq
//  (Audio)   = (IF2FREQ) - uiIF2Freq

//  Transmit:
//  uiIF1Freq = IF1FREQ                     //  AM Mode.
//  uiIF1Freq = IF1FREQ - IF1SHIFT          //  LSB Mode.
//  uiIF1Freq = IF1FREQ + IF1SHIFT          //  USB Mode.
//  uiIF1Freq = IF1FREQ + IF1SHIFT - CWTONE //  CW Mode.

//  LocalOSC  = VFOFreq + uiIF1Freq           //  AM/LSB/USB Modes.
//  LocalOSC  = VFOFreq + IF1FREQ + IF1SHIFT  //  CW Mode Only. (This allows the IF to be shifted for the CW carrier w/o shifting the LO.)

//  (TXIF) = (Audio) +/- uiIF1Freq  //  "+/-" Means DSB Modulation. The IF2SHIFT causes the filter to chop off the unwanted sideband.
//  (TXRF) = LocalOSC - (TXIF)      //  Sideband inversion happens at this TX Mixer stage to produce the final output RF signal.

//  -------------------------------------------------------------------------------------------------------
//  Global Bit Flags. Defaults are startup conditions. These states will be saved to EEPROM in the future to allow startup in last state.
//  Bit: 7 = USB Mode (Default)
//  Bit: 6 = LSB Mode
//  Bit: 5 =  CW Mode
//  Bit: 4 =  AM Mode
//  Bit: 3 = VFO B TX
//  Bit: 2 = VFO B RX
//  Bit: 1 = VFO A TX (Default)
//  Bit: 0 = VFO A RX (Default)
uint8_t uiFlags = 0b10000011;
//  VFO Toggling bit-math explanation.
//  uiFlags ^= 0b00001111;  This will toggle from one VFO to the other. If in split mode, the RX/TX will be flipped on both VFOs.
//  uiFlags ^= 0b00001010;  This will enable split operation by moving the TX to the currently inactive VFO.
//                          It will also disable split operation and the active VFO will be the one currently assigned to RX.

//  Used for "run-once" triggering. Mainly used for split button detection,
//  but could be used for anything else if care is used to avoid interference.
bool bTriggered = false;


//  -------------------------------------------------------------
//  ------ Callback function that polls the user controls. ------
//  -------------------------------------------------------------
//  The "Timer1" library is setup to generate a timer interrupt
//  every 1ms. This is the callback function that gets called
//  every 1ms when the timer interrupt is triggered. Inside the
//  polling functions of the encoder/button classes are called.
//  The polling functions read the states of the hardware pins
//  and set the proper states that are checked on every
//  itteration of the main program loop.
//  The user controls are polled in this manner to eliminate the
//  the need to use the two hardware interrupts (only pins 2 & 3)
//  on the Arduino Uno (used for prototyping) while also ensuring
//  a fast response time and no missed presses of the buttons.
//  -------------------------------------------------------------
void tmrPollUI() {
  if (!bTransmitting) {
    seEncoder.isr_DoPolling();
    sbStepButton.isr_DoPolling();
    sbModeButton.isr_DoPolling();
  }

  sbPTTButton.isr_DoPolling();
}


//  ---------------------------------------
//  ------ Arduino "setup" function. ------
//  ---------------------------------------
void setup() {
  #ifdef HBR_USBDEBUG
  //  Setup serial port to print debug info.
    Serial.begin(9600);
    Serial.println("");
    Serial.println(F("//-------------------------\\\\"));
    Serial.println(F("  Start of \"Setup\" routine."));
  #endif
    
  //  Initialize the ILI9341 display class.
  tDisp.begin();
  tDisp.setRotation(TFT_ROTATION);
  //  End display initialization.

  #ifdef HBR_USBDEBUG
    //  Debug info for TFT Display.
    //  This requires the MISO (Master In/Slave Out) pin to be connected
    //  so that data can be read from the display. The screen will function
    //  normally with only the MOSI (Master Out/Slave In) pin connected
    //  but you can only send data to the display. This is an optional feature.
    Serial.println("");
    uint8_t uiRetVal = tDisp.readcommand8(ILI9341_RDMODE);
    Serial.print(F("  Display Power Mode: 0x")); Serial.println(uiRetVal, HEX);
    uiRetVal = tDisp.readcommand8(ILI9341_RDMADCTL);
    Serial.print(F("  MADCTL Mode: 0x")); Serial.println(uiRetVal, HEX);
    uiRetVal = tDisp.readcommand8(ILI9341_RDPIXFMT);
    Serial.print(F("  Pixel Format: 0x")); Serial.println(uiRetVal, HEX);
    uiRetVal = tDisp.readcommand8(ILI9341_RDIMGFMT);
    Serial.print(F("  Image Format: 0x")); Serial.println(uiRetVal, HEX);
    uiRetVal = tDisp.readcommand8(ILI9341_RDSELFDIAG);
    Serial.print(F("  Self Diagnostic: 0x")); Serial.println(uiRetVal, HEX);   
    Serial.println("");
    delay(10);
  #endif

  //  Setup UI polling timer and ISR callback function.
  //  Do this step last in the setup routine.
  Timer1.initialize(1000);  //  Poll every 1ms.
  Timer1.attachInterrupt(tmrPollUI);  

  //  Print final debug info for "setup" routine before exiting into main loop.
  #ifdef HBR_USBDEBUG
    Serial.println(F("  End of \"Setup\" routine."));
    Serial.println(F("\\\\-------------------------//"));
    Serial.println("");
  #endif

    //  Display splash screen and allow 5 seconds to view it.
  #ifdef HBR_SPLASH_SCREEN
    vHBR_SplashScreen();
    delay(5000);
  #endif

  //  Setup output pin modes and set all to off.
  pinMode(POWER_IN, INPUT); analogRead(POWER_IN);
  pinMode(SSB_MODE_OUT, OUTPUT); digitalWrite(SSB_MODE_OUT, !SSB_ACTIVE);
  pinMode(CW_MODE_OUT, OUTPUT); digitalWrite(CW_MODE_OUT, !CW_ACTIVE);
  pinMode(AM_MODE_OUT, OUTPUT); digitalWrite(AM_MODE_OUT, !AM_ACTIVE);
  pinMode(TX_MODE_OUT, OUTPUT); digitalWrite(TX_MODE_OUT, !TX_ACTIVE);

  //  Initialize the Si5351.
  bool I2C_Comm = siFreqGen.init(SI5351_CRYSTAL_LOAD_8PF, 0, 7900);  //  Frequency correction is last parameter. 1000 = 10Hz.
  if (!I2C_Comm) {
    tDisp.fillScreen(ILI9341_BLACK);
    tDisp.setTextSize(3);
    tDisp.setTextColor(ILI9341_YELLOW, 0);
    tDisp.setCursor(6, 8);
    tDisp.println(F("Si5351 Failed!!!"));
    while(1);
  }

  siFreqGen.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  siFreqGen.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);  //  Changed to 8ma (Max drive) for use with diode ring mixer for TX.
  siFreqGen.set_ms_source(SI5351_CLK2, SI5351_PLLB);

  #if IF1FREQ > 0   //  Only setup CLK1 in dual conversion mode.
    siFreqGen.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
    siFreqGen.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  #endif

  //  Uncomment the next 2 lines and measure CLK0 for 10MHz.
  //  Take the difference (10MHz - Measured Value) multiplied by 100 for the correction in the "init" above.
  //  Example: My Si5351 board measured 10,000,079Hz. So 79 * 100 = 7900 for the correction factor.
  //siFreqGen.set_freq(SI5351_DEFAULT_CLK, SI5351_CLK0);  //  Set CLK0 to 10MHz and enable.
  //while(1); //  Endless loop.

  //  Draw the main screen.
  vDrawMainBackground();
  vRedrawVFOs(true);
  vRedrawVFOLabels();
  vRedrawIFShift();
  vCycleStepSize();
  vCycleMode(true);     //  This will enable the proper mode selection outputs for startup mode.
  vUpdateRXLOs(true);   //  Calculate all clock frequencies and enable them.
}


//  --------------------------------------
//  ------ Arduino "loop" function. ------
//  --------------------------------------
void loop() {
  //  Get PTT button status first. When transmitting, all other panel buttons are not read.
  SimpleButton::eBtnState bsPTTButton = sbPTTButton.bsGetButton();
  if (bsPTTButton == SimpleButton::bsOpen) {
    //  Not transmitting. Do normal functions.

    //  Get step button status.
    SimpleButton::eBtnState bsStepButton = sbStepButton.bsGetButton();
    if (bsStepButton == SimpleButton::bsClicked) { vCycleStepSize(); }

    //  Get VFO button status.
    //  This is normally the button when you push the VFO knob, but it can be a regular panel button if you wish.
    SimpleEncoder::eBtnState bsVFOButton = seEncoder.bsGetButton();
    if (bsVFOButton == SimpleEncoder::bsClicked) {
      uiFlags ^= 0b00001111;  //  Switch VFOs.
      vRedrawVFOLabels();
      vUpdateRXLOs(); //  LO Only.
    } else if (bsVFOButton == SimpleEncoder::bsHeld) {
      //  Changed this to do the enable/disable when "Held" is triggered instead of on "Released."
      //  This required this bit of extra code to prevent constant cycling while the button was held.
      if (!bTriggered) {
        uiFlags ^= 0b00001010;  //  Enable/Disable VFO Split operation.
        vRedrawVFOLabels();
        vUpdateRXLOs(); //  LO Only.
        bTriggered = true;
      }
    } else if (bsVFOButton == SimpleEncoder::bsReleased) { bTriggered = false; }   //  Reset the "run-once" condition.

    SimpleButton::eBtnState bsModeButton = sbModeButton.bsGetButton();
    if (bsModeButton == SimpleButton::bsClicked) {
      vCycleMode();
      vUpdateRXLOs(true); //  Update everything.
    }

    //  Get encoder rotation status.
    int8_t iEncoder = seEncoder.iGetValue();
    if (iEncoder != 0) {
      if (bsStepButton == SimpleButton::bsHeld) {
        //  Step button held. (Tune IF Shift.)
        iIFShift += (iEncoder * 10);  //  Shift in 10Hz steps. Acceleration goes up to 100Hz steps for faster tuning.
        iIFShift = constrain(iIFShift, -3000, 3000);
        vRedrawIFShift();
        vUpdateRXLOs(false, true);  //  LO and IFShift.
      } else if (bsModeButton == SimpleButton::bsHeld) {
        //  Code for Mode button held. (Band cycle.)
        vChangeBand((iEncoder > 0) ? 1 : -1);   //  This 1 or -1 conditional disables encoder acceleration for band changes.
        vRedrawVFOs();
        vUpdateRXLOs(); //  LO Only.
      } else {
        //  Normal VFO Tuning.
        vTuneActiveVFO(iEncoder);   //  Tune in selected step size with acceleration up to 10x selected step size.
        vRedrawVFOs();
        vUpdateRXLOs(); //  LO Only.
      }
    }
  } else if (bsPTTButton == SimpleButton::bsHeld) {
    //  Transmitting. Handle TX Functions.
    bTransmitting = true;
    vRedrawVFOLabels();
    vUpdateTXLOs();
    digitalWrite(TX_MODE_OUT, TX_ACTIVE);
  } else if (bsPTTButton == SimpleButton::bsReleased) {
    //  Just stopped transmitting. Reset to receive mode.
    digitalWrite(TX_MODE_OUT, !TX_ACTIVE);
    bTransmitting = false;
    vRedrawVFOLabels();
    vUpdateRXLOs(false, true);  //  LO and IFShift.
  }
}


//  -------------------------------------------
//  ------ Homebrew Radio Splash screen. ------
//  -------------------------------------------
#ifdef HBR_SPLASH_SCREEN
void vHBR_SplashScreen()
{
  tDisp.fillScreen(ILI9341_BLACK);
  tDisp.setTextSize(4);
  tDisp.setTextColor(ILI9341_GREEN, 0);
  tDisp.setCursor(16, 32);
  tDisp.println(F("Damien v0.1a"));
  
  tDisp.setTextSize(2);
  tDisp.setTextColor(0xB800, 0);  //  Dark Red (184, 0, 0)
  tDisp.setCursor(16, 96);
  tDisp.println(F("George Family Radios"));
  tDisp.setCursor(16, 128);
  tDisp.println(F("Prototype HDR by: KC3MOP"));
}
#endif


//  -------------------------------------------
//  ------ Draw out the main background. ------
//  -------------------------------------------
//  This should only need to be done at startup
//  or if exiting a menu or something that had
//  been covering up the entire screen. Normal
//  screen updates will only redraw the parts
//  of the screen that need to be redrawn.
//  -------------------------------------------
void vDrawMainBackground() {
  //  Clear the screen to a black background.
  tDisp.fillScreen(ILI9341_BLACK);

  //  Draw bounding boxes.
  tDisp.drawRect( 0,  0, 320, 240, 0x051D);
  tDisp.drawRect( 1,  1, 318, 238, 0x051D);
  tDisp.drawRect(96, 64, 128, 112, 0x051D);
  tDisp.drawRect(97, 65, 126, 110, 0x051D);
  tDisp.drawFastHLine(0,  64, 320, 0x051D);
  tDisp.drawFastHLine(0,  65, 320, 0x051D);
  tDisp.drawFastHLine(0, 119, 320, 0x051D);
  tDisp.drawFastHLine(0, 120, 320, 0x051D);
  tDisp.drawFastHLine(0, 174, 320, 0x051D);
  tDisp.drawFastHLine(0, 175, 320, 0x051D);

  //  Draw callsign in center of screen. You can change this to whatever static text you want.
  //  Limit of 6 characters for each of the 2 lines.
  //  Might implement some kind of graph to read the voltage at the defined pin "POWER_IN".
  //  This was my original idea for an S-Meter/Power Output display. We'll see what happens.
  tDisp.setTextSize(3);
  tDisp.setTextColor(0x53F6, 0);
  tDisp.setCursor(108, 70);
  tDisp.print(F("HDR By"));
  tDisp.setCursor(108, 95);
  tDisp.print(F("KC3MOP"));

  //  Draw "Shift" label.
  tDisp.setTextSize(3);
  tDisp.setTextColor(ILI9341_LIGHTGREY, 0);
  tDisp.setCursor(126, 125);
  tDisp.print(F("Shift"));
  tDisp.setCursor(180, 150);
  tDisp.print(F("Hz"));

  //  Draw "Step" label.
  tDisp.setTextSize(3);
  tDisp.setTextColor(ILI9341_ORANGE, 0);
  tDisp.setCursor(246, 125);
  tDisp.print(F("Step"));

  //  Draw static decimal points in the VFO display areas.
  tDisp.fillRect( 90,  53, 4, 4, ILI9341_ORANGE);
  tDisp.fillRect(222,  39, 4, 4, ILI9341_ORANGE);
  tDisp.fillRect( 90, 227, 4, 4, ILI9341_ORANGE);
  tDisp.fillRect(222, 213, 4, 4, ILI9341_ORANGE);  
}


//  -------------------------------------
//  ------ Redraw the VFO display. ------
//  -------------------------------------
//  This should get called once from the
//  setup() routine and any time that
//  either of the VFOs change frequency.
//  -------------------------------------
void vRedrawVFOs(bool bSetup) {
  uint8_t uiMegs = 0;
  uint16_t uiKilos = 0;
  uint16_t uiHertz = 0;
  char cTemp[] = "000";

  tDisp.setTextColor(ILI9341_ORANGE, 0);
  if (bitRead(uiFlags, 0) || bSetup) {
    //  Draw and update VFO A.
    uiMegs = (uint8_t)(uiVFOAFreq / 1000000);
    uiKilos = (uint16_t)((uiVFOAFreq / 1000) % 1000);
    uiHertz = (uint16_t)(uiVFOAFreq % 1000);

    tDisp.setTextSize(7);
    tDisp.setCursor(6, 8);
    if (uiMegs < 10) { tDisp.print(" "); }
    tDisp.print(uiMegs);
    sprintf(cTemp, "%.3u", uiKilos);
    tDisp.setCursor(96, 8); tDisp.print(cTemp);
    tDisp.setTextSize(5);
    sprintf(cTemp, "%.3u", uiHertz);
    tDisp.setCursor(228, 8); tDisp.print(cTemp);
  }

  if (bitRead(uiFlags, 2) || bSetup) {
    //  Draw and update VFO B.
    uiMegs = (uint8_t)(uiVFOBFreq / 1000000);
    uiKilos = (uint16_t)((uiVFOBFreq / 1000) % 1000);
    uiHertz = (uint16_t)(uiVFOBFreq % 1000);

    tDisp.setTextSize(7);
    tDisp.setCursor(6, 182);
    if (uiMegs < 10) { tDisp.print(" "); }
    tDisp.print(uiMegs);
    sprintf(cTemp, "%.3u", uiKilos);
    tDisp.setCursor(96, 182); tDisp.print(cTemp);
    tDisp.setTextSize(5);
    sprintf(cTemp, "%.3u", uiHertz);
    tDisp.setCursor(228, 182); tDisp.print(cTemp);
  }
}


//  ----------------------------------------------
//  ------ Redraw the VFO and RX/TX labels. ------
//  ----------------------------------------------
//  This gets called when the VFO button is used
//  to either swtich VFOA <-> VFOB or enable or
//  disable the split mode of opperation. When in
//  split mode, this function will also be called
//  when the PTT switch is closed to highlight the
//  VFO change while transmitting.
//  ----------------------------------------------
void vRedrawVFOLabels() {
  bool bVFOB = bGetActiveVFO();

  tDisp.setTextSize(3);
  tDisp.setCursor(6, 70);
  tDisp.setTextColor((bVFOB) ? ILI9341_DARKGREY : ILI9341_YELLOW);
  tDisp.print("VFO-A");

  tDisp.setCursor(12, 95);
  tDisp.setTextColor((uiFlags & 1) ? ILI9341_GREEN : ILI9341_DARKGREY);
  tDisp.print("RX");

  tDisp.setCursor(54, 95);
  tDisp.setTextColor((uiFlags & 2) ? ILI9341_RED : ILI9341_DARKGREY);
  tDisp.print("TX");

  tDisp.setCursor(12, 125);
  tDisp.setTextColor((uiFlags & 4) ? ILI9341_GREEN : ILI9341_DARKGREY);
  tDisp.print("RX");

  tDisp.setCursor(54, 125);
  tDisp.setTextColor((uiFlags & 8) ? ILI9341_RED : ILI9341_DARKGREY);
  tDisp.print("TX");

  tDisp.setCursor(6, 150);
  tDisp.setTextColor((bVFOB) ? ILI9341_YELLOW : ILI9341_DARKGREY);
  tDisp.print("VFO-B");
}


//  ------------------------------------------
//  ------ Redraw the IF Shift display. ------
//  ------------------------------------------
//  This gets called when the IF Shift is
//  changed and at startup. The display is
//  updated with the new value.
//  -------------------------------------------
void vRedrawIFShift() {
  tDisp.setTextSize(3);
  tDisp.setTextColor(ILI9341_LIGHTGREY, 0);

  tDisp.setCursor(108, 125);
  if (iIFShift == 0) {
    tDisp.print(" ");
  } else if (iIFShift > 0) {
    tDisp.print("+");
  } else {
    tDisp.print("-");
  }

  tDisp.setCursor(108, 150);
  tDisp.print(abs(iIFShift));
  if (abs(iIFShift) < 1000) { tDisp.print(" "); }
  if (abs(iIFShift) < 100)  { tDisp.print(" "); }
  if (abs(iIFShift) < 10)   { tDisp.print(" "); }
}


//  -----------------------------------------------
//  ------ Cycle through the 4 tuning steps. ------
//  -----------------------------------------------
void vCycleStepSize() {
  tDisp.setTextSize(3);
  tDisp.setTextColor(ILI9341_ORANGE, 0);

  if (uiTuneStep == 10) {
    uiTuneStep = 100;
    tDisp.setCursor(228, 150);
    tDisp.print(F("100Hz"));
  } else if (uiTuneStep == 100) {
    uiTuneStep = 1000;
    tDisp.setCursor(228, 150);
    tDisp.print(F("1 kHz"));
  } else if (uiTuneStep == 1000) {
    uiTuneStep = 10000;
    tDisp.setCursor(228, 150);
    tDisp.print(F("10kHz"));
  } else {
    uiTuneStep = 10;
    tDisp.setCursor(228, 150);
    tDisp.print(F("10 Hz"));
  }
}


//  --------------------------------------------
//  ------ Gets the currently active VFO. ------
//  --------------------------------------------
//  Returns: false = VFOA / true = VFOB
//  Active VFO depends on the uiFlags VFO bits
//  and whether or not we are transmitting.
//  --------------------------------------------
bool bGetActiveVFO() {
  if (bTransmitting) {
    return ((uiFlags & 0b00001010) > 3) ? true : false;
  } else {
    return ((uiFlags & 0b00000101) > 3) ? true : false;
  }
}


//  -----------------------------------------------------
//  ------ Tune the active VFO and set the Si5351. ------
//  -----------------------------------------------------
void vTuneActiveVFO(int8_t iAmount) {
  int32_t iTune = (int32_t)iAmount * uiTuneStep;

  if (bitRead(uiFlags, 0)) {
    uiVFOAFreq += iTune;
    uiVFOAFreq = constrain(uiVFOAFreq, uiBands[iBandA][0],uiBands[iBandA][1]);
  } else if (bitRead(uiFlags, 2)) {
    uiVFOBFreq += iTune;
    uiVFOBFreq = constrain(uiVFOBFreq, uiBands[iBandB][0],uiBands[iBandB][1]);
  }
}


//  ------------------------------------------
//  ------ Change band for current VFO. ------
//  ------------------------------------------
//  Changes the band for the current VFO and
//  sets the frequency to the lower end of
//  the selected band.
//  ------------------------------------------
void vChangeBand(int8_t iAmount) {
  if (bitRead(uiFlags, 0)) {
    uiBands[iBandA][2] = uiVFOAFreq;
    iBandA += iAmount;
    iBandA = constrain(iBandA, 0, BANDMAX);
    uiVFOAFreq = uiBands[iBandA][2];
  } else if (bitRead(uiFlags, 2)) {
    uiBands[iBandB][3] = uiVFOBFreq;
    iBandB += iAmount;
    iBandB = constrain(iBandB, 0, BANDMAX);
    uiVFOBFreq = uiBands[iBandB][3];
  }
}


// -------------------------------------------------------------
// ------ Cycles operating modes and draws to the screen. ------
// -------------------------------------------------------------
void vCycleMode(bool bDrawOnly) {
  //  Cycle to next mode.
  if(!bDrawOnly) {
    if (uiFlags & 128) { uiFlags &= ~(128); uiFlags |= 64; }
    else if (uiFlags & 64) { uiFlags &= ~(64); uiFlags |= 32; }
    else if (uiFlags & 32) { uiFlags &= ~(32); uiFlags |= 16; }
    else if (uiFlags & 16) { uiFlags &= ~(16); uiFlags |= 128; }
  }

  //  Disable all outputs.
  digitalWrite(SSB_MODE_OUT, !SSB_ACTIVE);
  digitalWrite(CW_MODE_OUT, !CW_ACTIVE);
  digitalWrite(AM_MODE_OUT, !AM_ACTIVE);
  delay(100); //  Wait 100ms to ensure turn off.

  //  Draw mode to the screen and set output pins.
  tDisp.setTextSize(5);
  tDisp.setTextColor(ILI9341_YELLOW, 0);
  tDisp.setCursor(228, 74);
  if (uiFlags & 128) { tDisp.print(F("USB")); digitalWrite(SSB_MODE_OUT, SSB_ACTIVE); }
  else if (uiFlags & 64) { tDisp.print(F("LSB")); digitalWrite(SSB_MODE_OUT, SSB_ACTIVE); }
  else if (uiFlags & 32) { tDisp.print(F(" CW")); digitalWrite(CW_MODE_OUT, CW_ACTIVE); }
  else if (uiFlags & 16) { tDisp.print(F(" AM")); digitalWrite(AM_MODE_OUT, AM_ACTIVE); }
}


//  -----------------------------------------------------------------------------
//  ------ Calculates and updates the IF and LO frequencies to the Si5351. ------
//  -----------------------------------------------------------------------------
//  This is called when changing modes (USB/LSB/AM/CW), when changing iIFShift,
//  when exiting transmit mode, and when tuning the VFO. Both "bModeChange" and
//  "bShiftIF" default to "false" so VFO only updates can be called easier. The
//  "bShiftIF" is set to "true" when exiting transmit mode also.
//  -----------------------------------------------------------------------------
void vUpdateRXLOs(bool bModeChange, bool bShiftIF) {
  #if IF1FREQ == 0
    //  Single conversion mode.
    if (bShiftIF || bModeChange) {
      if ((uiFlags & 128) || (uiFlags & 32)) { uiIF2Freq = IF2FREQ + IF2SHIFT - iIFShift; } //  USB/CW Mode.
      else if (uiFlags & 64) { uiIF2Freq = IF2FREQ - IF2SHIFT + iIFShift; }                 //  LSB Mode.
      else if (uiFlags & 16) { uiIF2Freq = IF2FREQ - iIFShift; }                            //  AM Mode.
      //  Push uiIF2Freq to CLK2 of Si5351.
      siFreqGen.set_freq((uint64_t)uiIF2Freq * SI5351_FREQ_MULT, SI5351_CLK2);
    }

    //  Always update Main LO Frequency.
    if (bGetActiveVFO()) { uiLocalOsc = uiVFOBFreq + uiIF2Freq; }
    else { uiLocalOsc = uiVFOAFreq + uiIF2Freq; }
    //  Push uiLocalOsc to CLK0 of Si5351.
    siFreqGen.set_freq((uint64_t)uiLocalOsc * SI5351_FREQ_MULT, SI5351_CLK0);
  #elif IF1FREQ > 0
    //  Dual conversion mode.
    if (bModeChange) {
      if ((uiFlags & 128) || (uiFlags & 32)) { uiIF2Freq = IF2FREQ + IF2SHIFT; } //  USB/CW Mode.
      else if (uiFlags & 64) { uiIF2Freq = IF2FREQ - IF2SHIFT; } //  LSB Mode.
      else if (uiFlags & 16) { uiIF2Freq = IF2FREQ; } //  AM Mode.
      //  Push uiIF2Freq to CLK2 of Si5351.
      siFreqGen.set_freq((uint64_t)uiIF2Freq * SI5351_FREQ_MULT, SI5351_CLK2);
    }

    if (bShiftIF || bModeChange) {
      if ((uiFlags & 128) || (uiFlags & 32)) { uiIF1Freq = (IF1FREQ - IF2FREQ) + IF2OFFSET - iIFShift; } //  USB/CW Mode.
      else if (uiFlags & 64) { uiIF1Freq = (IF1FREQ - IF2FREQ) - IF2OFFSET + iIFShift; } //  LSB Mode.
      else if (uiFlags & 16) { uiIF1Freq = (IF1FREQ - IF2FREQ); } //  AM Mode.
      //  Push uiIF1Freq to CLK1 of Si5351.
      siFreqGen.set_freq((uint64_t)uiIF1Freq * SI5351_FREQ_MULT, SI5351_CLK1);
    }

    //  Always update Main LO Frequency.
    if (bGetActiveVFO()) { uiLocalOsc = uiVFOBFreq + uiIF1Freq + uiIF2Freq; }
    else { uiLocalOsc = uiVFOAFreq + uiIF1Freq + uiIF2Freq; }
    //  Push uiLocalOsc to CLK0 of Si5351.
    siFreqGen.set_freq((uint64_t)uiLocalOsc * SI5351_FREQ_MULT, SI5351_CLK0);
  #endif
}


//  -----------------------------------------------------------------------------
//  ------ Calculates and updates the IF and LO frequencies to the Si5351. ------
//  -----------------------------------------------------------------------------
//  This is called when entering transmit mode.
//  -----------------------------------------------------------------------------
void vUpdateTXLOs() {
  #if IF1FREQ == 0
    //  Single conversion mode.
    if (uiFlags & 128) { uiIF2Freq = IF2FREQ + IF2SHIFT; }              //  USB Mode.
    else if (uiFlags & 64) { uiIF2Freq = IF2FREQ - IF2SHIFT; }          //  LSB Mode.
    else if (uiFlags & 32) { uiIF2Freq = IF2FREQ + IF2SHIFT - CWTONE; } //  CW Mode.
    else if (uiFlags & 16) { uiIF2Freq = IF2FREQ; }                     //  AM Mode.
    //  Push uiIF2Freq to CLK2 of Si5351.
    siFreqGen.set_freq((uint64_t)uiIF2Freq * SI5351_FREQ_MULT, SI5351_CLK2);

    //  Always update Main LO Frequency.
    if (bGetActiveVFO()) { uiLocalOsc = uiVFOBFreq + uiIF2Freq; }
    else { uiLocalOsc = uiVFOAFreq + uiIF2Freq; }

    if (uiFlags & 32) { uiLocalOsc += CWTONE; }   //  Fix offset for CW carrier.
    //  Push uiLocalOsc to CLK0 of Si5351.
    siFreqGen.set_freq((uint64_t)uiLocalOsc * SI5351_FREQ_MULT, SI5351_CLK0);
  #elif IF1FREQ > 0
    //  Dual conversion mode.
    if (uiFlags & 128) { uiIF1Freq = IF1FREQ + IF1SHIFT; } //  USB Mode.
    else if (uiFlags & 64) { uiIF1Freq = IF1FREQ - IF1SHIFT; } //  LSB Mode.
    else if (uiFlags & 32) { uiIF1Freq = IF1FREQ + IF1SHIFT - CWTONE; } //  CW Mode.
    else if (uiFlags & 16) { uiIF1Freq = IF1FREQ; } //  AM Mode.
    //  Push uiIF1Freq to CLK1 of Si5351.
    siFreqGen.set_freq((uint64_t)uiIF1Freq * SI5351_FREQ_MULT, SI5351_CLK1);

    //  Always update Main LO Frequency.
    if (bGetActiveVFO()) { uiLocalOsc = uiVFOBFreq + uiIF1Freq; }
    else { uiLocalOsc = uiVFOAFreq + uiIF1Freq; }

    if (uiFlags & 32) { uiLocalOsc += CWTONE; }   //  Fix offset for CW carrier.
    //  Push uiLocalOsc to CLK0 of Si5351.
    siFreqGen.set_freq((uint64_t)uiLocalOsc * SI5351_FREQ_MULT, SI5351_CLK0);
  #endif
}










