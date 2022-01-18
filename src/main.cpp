#include "main.h"
#include "i2cEncoderLibV2.h"
#include <AccelStepper.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "LeoNerdEncoder.h"
#include <math.h>

#define __ESP32__ 1
#define PANEL_DISPLAY_ADDRESS 0x3C // I2C adddress
#define PANEL_ENCODER_ADDRESS 0x3D // I2C adddress
#define LARGE_ENCODER_ADDRESS 0x04 // I2C adddress

#define BASE_FONT u8g2_font_logisoso16_tr //u8g2_font_9x15B_tf //u8g2_font_6x12_t_symbols

#define stepPinStepper 26
#define dirPinStepper 27
#define enablePinStepper 25
#define EncChangePin 18

#define I2C_SDA 21
#define I2C_SCL 22
#define i2C_Freq 400000

#define LeftMargin 04     //4 pixels from left hand side on display
#define Line1 16        //Line start coordinated for 16pixel font
#define Line2 38
#define Line3 60

// simple wrapper for determining the array elements count
#define ArraySize(arr) (sizeof(arr) / sizeof(arr[0]))

Preferences preferences;

i2cEncoderLibV2 LargeEncoder(LARGE_ENCODER_ADDRESS);
LeoNerdEncoder PanelEncoder(PANEL_ENCODER_ADDRESS);

U8G2_SH1106_128X64_NONAME_F_HW_I2C PanelDisplay(U8G2_R2, /* reset=*/U8X8_PIN_NONE, /* clock=*/I2C_SCL, /* data=*/I2C_SDA); // ESP32 Thing, HW I2C with pin remapping

AccelStepper stepper(AccelStepper::DRIVER, stepPinStepper, dirPinStepper); //pin 26 step, pin 27 direction

// runtime variables
ButtonState wheelBtn,
    mainBtn,
    leftBtn,
    rightBtn;
int16_t encoderPos = 0, lastEncoderPos = 0;

int PosCounter = 0;
int EncCount = 1;
int EncStep = 1;
int StartTime = 0;
int Period = 0;
int EditValue = 0;
int BootCount = 0;

int MotorStepsPerRev = 0;
int ControllerMicroSteps = 0;
int TableRatio = 0;
int JogSpeed = 0;
int Accelleration = 0;

int StepsPerRev = 0;
int StepsPerDegree = 0;
float DegreePerStep = 0;

void setup()
{

  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL, i2C_Freq); //Configure I2C bus with pins 22(SDA), 21(SCL) and frequesncy of 400KHz

  Serial.println("**** Setup Started****");

  //Read paramters from NVRAM\FLASH RAM
  Serial.println("**** Setting up Preferences ****");
  StartTime = micros();
  preferences.begin("RotaryTable", false);         //Start NameSpace with read\write access
  BootCount = preferences.getUInt("BootCount", 0); //read Boot Count
  BootCount++;                                     //Increment by one
                                                   //Read Physical parameters
  MotorStepsPerRev = preferences.getUInt("MSPR", 200);
  ControllerMicroSteps = preferences.getUInt("CMS", 1);
  TableRatio = preferences.getUInt("TableRatio", 1);
  JogSpeed = preferences.getUInt("JogSpeed", 100);
  Accelleration = preferences.getUInt("Accelleration", 100);

  //Calculate derived constants from physical parameters
  StepsPerRev = MotorStepsPerRev * ControllerMicroSteps * TableRatio; //eg 200 * 8 *90 = 144000 Steps per rev
  StepsPerDegree = StepsPerRev / 360;                                 //eg 144000/360 = 400 Steps per degree
  DegreePerStep = 360 / StepsPerRev;                                  //eg 360 / 144000 = 0.0025 Degrees per step

  preferences.putUInt("BootCount", BootCount); //store value in Non Volatile Memory (NVM)
  preferences.end();
  Period = micros() - StartTime;
  Serial.printf(" *** Setting up Preferences Complete **** > %d uS\n", Period);
  Serial.printf(" Boot Count > %d \n", BootCount);

  scanI2CDevices(); //scan and print to debug window devices found on I2C bus

  // initialize the Panel display
  Serial.println("**** Setting up Panel Display ****");
  StartTime = micros();
  PanelDisplay.setI2CAddress(PANEL_DISPLAY_ADDRESS << 1); // Set I2C address of Panel Display
  PanelDisplay.begin();
  PanelDisplay.enableUTF8Print(); //enable UTF8 support for the print() statement
  PanelDisplay.clearDisplay();    //clear Display
  Period = micros() - StartTime;
  Serial.printf(" *** Setting up Panel Display COMPLETE **** > %d uS\n", Period);

  // initialize the Panel encoder
  Serial.println("**** Setting up Panel Encoder ****");
  StartTime = micros();
  PanelEncoder.begin();
  uint8_t ver = PanelEncoder.queryVersion(); // Please notice: This library needs at least version 2!
  Serial.printf("Encoder version is: %d", ver);
  uint8_t opt = PanelEncoder.queryOptions(); // check the options
  Serial.printf("Options set: 0x%02X\n", opt);
  // enable encoder wheel to beep on rotation only
  PanelEncoder.setKeyBeep(1400, 10);
  PanelEncoder.setKeyBeepMask(BEEP_WHEEL_ROTATION);
  Period = micros() - StartTime;
  Serial.printf(" *** etting up Panel Encoder COMPLETE **** > %d uS\n", Period);

  // Set up Stepper Motor Values
  Serial.println("**** Setting up Stepper ****");
  StartTime = micros();
  stepper.setEnablePin(enablePinStepper);      //Set Enable Pin
  stepper.setPinsInverted(false, false, true); //(Direction, Step, Enable) i.e Invert Enable Pin
  stepper.disableOutputs();                    //Disable outputs via En pin to reduce standby current
  stepper.setMinPulseWidth(20);                //Step min pulse width - 20uS (standard minimum for 6600 based microstepper drivers)
  stepper.setMaxSpeed(10000);                  //Set Maximum speed pulses\second
  stepper.setAcceleration(50000);              //Set accelleration in pulses\second\second
  stepper.setCurrentPosition(0);               //Set current position to Absolute Position 0

  Period = micros() - StartTime;
  Serial.printf(" *** Setting up Stepper COMPLETE **** > %d uS\n", Period);

  //Setup large encoder
  Serial.println("**** Setting up Large Encoder ****");
  pinMode(EncChangePin, INPUT); // define pin that signals changes to large encoder dial low=change
  StartTime = micros();
  LargeEncoder.reset();
  LargeEncoder.begin(
      i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_ENABLE |
      i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
      i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeCounter((int32_t)0);    //Reset the counter value
  LargeEncoder.writeInterruptConfig(0x18);  //Enable just rotation interrupts */
  LargeEncoder.writeAntibouncingPeriod(20); //Set an anti-bouncing of 200ms */
  Period = micros() - StartTime;
  Serial.printf(" *** Setting up Large Encoder COMPLETE **** > %d uS\n", Period);

  IntroScreen(); //Display SW Version Info

  disableCore0WDT();                                                        //Stop watchdog on processor that step.run() is running on
  xTaskCreatePinnedToCore(MoveStepper, "TaskOne", 10000, NULL, 1, NULL, 0); //Ensure .run() task is running at a higher level then core loop.
}

void loop()
{

  for (;;)
  {                              //Continuous Loop
    int MenuChoice = MainMenu(); //Display main menu and return selected choice]
    Serial.printf(" Menu Choice  >%d\n", MenuChoice);
    switch (MenuChoice)
    {
    case 0:
      Jog(); // Jog to specific position in increments of 0.01,0.1,1,10 and 15 degrees
      break;

    case 1:
      StepAngle(); // Jog repeatedly a specified number of degrees
      break;

    case 2:
    {
      int NumDivisions = GetDivisions(); // Obtain number of divisions of 360 degrees
      Serial.printf(" Menu  >%d\n", NumDivisions);
      Divisions(NumDivisions); //Jog repeatedly through number of divisions (bolt holes\gear teeth etc)
    }
    break;

    case 3:
      RunContinuous(); // Run continuously with varying speed and direction
      break;

    case 4:
      Oscillate(); //jog set number of degrees at varying speed
      break;

    case 5:
      ParameterSetUp(); // Edit and store physical parameters of hardware and config
      break;
    }
  }
}

//Setup task to run on second processor freeing other processer for UI functions.
void MoveStepper(void *parameter)
{

  for (;;)
  { //Set loop to run forever.
    if (stepper.distanceToGo() != 0)
    { //If Stepper has distance to move then move.
      stepper.run();
    }
  }
}

void scanI2CDevices()
{ //scan for I2C devices on bus and display those found.
  for (uint8_t i2cAdr = 1; i2cAdr < 127; i2cAdr++)
  {
    // request a transmission for the device address
    Wire.beginTransmission(i2cAdr);
    // if something answered, the device is basically available
    if (Wire.endTransmission() == 0)
    {
      Serial.printf("I2C device found at address 0x%2x\n", i2cAdr);
      Serial.printf("Frequency %d\n", Wire.getClock()); //display clock frequency of I2C bus.
    }
  }
}

void IntroScreen()
{ //Display about Info
  char tmp[20];
  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);
  PanelDisplay.clearBuffer();
  PanelDisplay.drawStr(LeftMargin, Line1, "ERT Ver:1.0");
  PanelDisplay.drawStr(LeftMargin, Line2, "R.Firth 2022");
  sprintf(tmp, "Boot: %d", BootCount); //Display Step value
  PanelDisplay.drawStr(LeftMargin, Line3, tmp);
  PanelDisplay.sendBuffer();
  delay(3000); //Wait 3 seconds
}

int MainMenu()
{                             //Display main menu items and select an item.
  const int NumMenuItems = 6; //Holds number of menu items
  int MenuSelection = 0;
  const char *MenuText[NumMenuItems] = {"1:Jog", "2:Step Angle", "3:Divisions", "4:Run", "5:Oscillate", "9:Setup"};
  int v = 0;              //Holds rotary button value -1,0 or 1
  int encoderPos = 0;     //Current Cumulative rotary button count
  int lastEncoderPos = 0; //Previous Cumulative rotary button count

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  PanelEncoder.begin();
  PanelEncoder.setLED(LED_GREEN, true); //Set main button Green
  PanelEncoder.setLED(LED_RED, false);
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state
  PanelEncoder.loop();                  //Query Panel status

  lastEncoderPos = -1; //last encoder position to -1 so do executes at least once.

  do
  { //query main encoder and panel button, exit on main button press.

    PanelEncoder.loop();                          //Query Panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get button status

    v = PanelEncoder.getValue(); //Get Button step direction -1, 0 or 1
    encoderPos += v;             //Calculate cumulative position
    if (encoderPos == -1)
      encoderPos = NumMenuItems - 1;

    if (encoderPos != lastEncoderPos)
    {                                                 //Execute if rotary button changed
      lastEncoderPos = encoderPos;                    //Keep track of current and previous
      MenuSelection = abs(encoderPos % NumMenuItems); //Calculate index postion from Cumulative button position

      PanelDisplay.clearBuffer();
      PanelDisplay.drawStr(LeftMargin, Line1, "Main Menu");             //Display Main Menu
      PanelDisplay.drawStr(LeftMargin, Line2, MenuText[MenuSelection]); //Display text indext by button index position
      PanelDisplay.sendBuffer();

      Serial.printf("Menu Option Value inside menu change %d\n", MenuSelection);
      Serial.printf("Dial Counter value %d\n", v);
    }

  } while (mainBtn != Clicked); //Exit if main button pressed.

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Turn main button Red (Why?)
  return MenuSelection;               // Return menu item choosen
}

void Jog()
{

  char tmp[20];
  float AbsoluteAngle = 0.0; //Stores Angular position after each jog;
  int EncoderAngle = AbsoluteAngle * 100;
  int OptValue = 0;
  float AnglePerMenuOpt[5] = {1.0, 10.0, 15.0, 0.01, 0.1};
  int StepValue = 0;
  float StepValuef = 0.0;
  int AbsStepsPos = 0;
  int v = 0;
  int lastEncoderPos = -1; //Last encoder position - set to -1 to monitor loop runs at least once to set display
  int encoderPos = 0;

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)100000);           //Set max dial value to an arbitrary 100000
  LargeEncoder.writeMin((int32_t)-100000);          //Set min dial value to an arbitrary -10000
  LargeEncoder.writeCounter((int32_t)EncoderAngle); //Set cencoder value to Angle
  LargeEncoder.writeStep((int32_t)100);             //Set Steps to 100
  LargeEncoder.writeInterruptConfig(0x18);          //Limit events to encoder turning
  LargeEncoder.writeAntibouncingPeriod(20);         //Set antiBounce to 200mS

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Set main button to Red

  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  stepper.enableOutputs();       //Enable Stepper Driver
  stepper.setCurrentPosition(0); //Set current motor position to Zero

  do
  {

    PanelEncoder.loop();                          //Query state of panel
    mainBtn = PanelEncoder.getButton(MainButton); //Get state of main button

    v = PanelEncoder.getValue(); //Get value of encoder -1, 0 or 1
    encoderPos += v;             //Add to cumulative counter
    if (encoderPos != lastEncoderPos)
    {                                                 //If change to rotary dial...
      lastEncoderPos = encoderPos;                    //last = current to keep track of change
      OptValue = abs(encoderPos % 5);                 //Create Option value from 0 to 4 that will index increments
      StepValuef = AnglePerMenuOpt[OptValue] * 100.0; //Convert step value to integer by multiplying by 100
      StepValue = static_cast<int>(StepValuef);       //Convert step value to integer by multiplying by 100
      LargeEncoder.writeStep((int32_t)StepValue);     //Write to large encoder value for it to handle increments

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Step: %05.2f", AnglePerMenuOpt[OptValue]); //Display Step selected
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "Pos: %.2f", AbsoluteAngle); //Display Total angle
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      PanelDisplay.sendBuffer();
      Serial.printf("Menu Option Value inside menu change %d\n", OptValue);
      Serial.printf("Step inside {}: %f\n", AnglePerMenuOpt[OptValue]);
      Serial.printf("Step inside {} *100: %f\n", AnglePerMenuOpt[OptValue] * 100.0);
    }

    if (digitalRead(EncChangePin) == LOW)
    {                                                //Execute of Large dial moved
      EncoderAngle = LargeEncoder.readCounterLong(); //Read new Integer angle
      LargeEncoder.updateStatus();                   //Clear Events
      LargeEncoder.readStatus();                     //Clear Events

      AbsoluteAngle = EncoderAngle / 100.0; //Convert integer angle to real by div 100

      Serial.printf("Menu Option Value inside encoder change %d\n", OptValue);
      Serial.printf("EncoderAngle: %d\n", EncoderAngle);
      Serial.printf("AboluteAngle: %f\n", AbsoluteAngle);
      Serial.printf("Step: %f\n", AnglePerMenuOpt[OptValue] * 100.0);

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Step: %05.2f", AnglePerMenuOpt[OptValue]); //Display Step value
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "Pos: %.2f", AbsoluteAngle); //Display absolute total angle
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      PanelDisplay.sendBuffer();

      AbsStepsPos = StepsPerDegree * EncoderAngle / 100; //Convert angle to steps {need to calculate from steps per angle)}
      stepper.moveTo(AbsStepsPos);                       //Move to absolute steps away from zero
    }
  } while (mainBtn != Clicked); //repeat until cancel is pressed

  PanelEncoder.setLED(LED_GREEN, true); //Set main button LED to Green
  PanelEncoder.setLED(LED_RED, false);
  stepper.stop();           //Stop the motor
  stepper.disableOutputs(); //Disables output from Stepper controller to save current\heating
}

void StepAngle()
{
  char tmp[20];                                                              //String to hold Underscore marker
  char uString[10] = " ^^^     ";                                            //Underscore Marker Initial Position
  int JogAngle = 0;                                                          //10000 times real Step Angle to remove decimals and make integer
  int UnitJogAngle = int(JogAngle / 10000);                                  //Unit amount of Step Angle
  int TenthsJogAngle = int(JogAngle / 100) - UnitJogAngle * 100;             //Tenths amount of Step Angle
  int ThouJogAngle = JogAngle - TenthsJogAngle * 100 - UnitJogAngle * 10000; //Thousandths amount of Step Angle
  int MaxValue = 360;                                                        //Max value of large encoder
  int MinValue = -360;                                                       //Min value of large encoder
  int EditValue = 0;                                                         //Hold current editable value
  int UnderscoreIndex = 0;                                                   //Position of underscore or currently editable portion of Step Angle
  float StepAngle = 0.0;                                                     //Real Step angle in Degrees
  int StepCount = 0;                                                         //Number of steps from start
  int Revolutions = 0;                                                       //Count of complete revolutions to display Rev and steps (Angle mod 360)
  float CumulativeAngle = 0.0;                                               //Absolute Angle position
  int AbsStepperPosition = 0;                                                //Absolute Stepper position in steps
  int lastEncoderPos = -1;                                                   //Last encoder position - set to -1 to monitor loop runs at least once to set display

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  LargeEncoder.writeCounter((int32_t)UnitJogAngle); //Reset the counter value
  LargeEncoder.writeMax((int32_t)MaxValue);         //Set the maximum threshold
  LargeEncoder.writeMin((int32_t)MinValue);         //Set the minimum threshold
  LargeEncoder.writeStep((int32_t)1);               //Set increment to 1

  PanelEncoder.setLED(LED_GREEN, true); //Set main button to Green
  PanelEncoder.setLED(LED_RED, false);
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  stepper.enableOutputs();       //Enable Stepper Driver
  stepper.setCurrentPosition(0); //Set current motor position to Zero

  do
  { // repeat until main botton pressed to return to main menu

    PanelEncoder.loop();                          //Get Panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get status of main button
    int v = PanelEncoder.getValue();              //Get value of wheel -1 CCW, 0 No movement or 1 CW)
    encoderPos += v;                              //update wheel counter position

    if (encoderPos != lastEncoderPos)
    {                                        //detect change in wheel position - only execute on change
      lastEncoderPos = encoderPos;           //store current in last to enable check for change
      UnderscoreIndex = abs(encoderPos % 3); //convert incremental counter to values between 0 and 2
      switch (UnderscoreIndex)
      {
      case 0:
        sprintf_P(uString, PSTR(" ___.")); //Underline Units
        MaxValue = 360;                    //Set max value of encoder to 360 degrees
        MinValue = -360;                   //Set min value of encoder to -360 degrees
        EditValue = UnitJogAngle;          //Edit Unit angle
        EncStep = 1;                       //Set steps to 1
        break;
      case 1:
        sprintf_P(uString, PSTR("    .__")); //Underline Tenths value
        MaxValue = 99;                       //Set MAx value to 99
        MinValue = 0;                        //Set Min value to 0
        EditValue = TenthsJogAngle;          //Edit Tenths Angle
        EncStep = 1;                         //Set steps to 1
        break;
      case 2:
        sprintf_P(uString, PSTR("    .  __")); //Underline Thousandths value
        MaxValue = 99;                         //Set MAx to 99
        MinValue = 0;                          //Set Min to 0
        EditValue = ThouJogAngle;              //Edit Thousandths angle
        EncStep = 25;                          //Set step to max resolution (Needs to be calculated from hardware Parameters)
        break;
      }
      LargeEncoder.writeCounter((int32_t)EditValue); //Update Encoder with Editable value
      LargeEncoder.writeMax((int32_t)MaxValue);      //Set encoder Max Value
      LargeEncoder.writeMin((int32_t)MinValue);      //Set Encoder Min Value
      LargeEncoder.writeStep((int32_t)EncStep);      //Set encoder step value

      PanelDisplay.clearBuffer();                                                  //Clear display buffer
      sprintf(tmp, "%+04d,%02d%02d°", UnitJogAngle, TenthsJogAngle, ThouJogAngle); //Convert numerics to string
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);                                //Display Angle
      PanelDisplay.drawStr(LeftMargin, 18, uString);                               //Display Underline Markers
      PanelDisplay.sendBuffer();                                                   //Display Buffer
      Serial.printf("Menu Option Value inside menu change %d\n", UnderscoreIndex);
    } //End of Wheel change section

    if (digitalRead(EncChangePin) == LOW)
    {                                            //Check for change in large encoder
      EditValue = LargeEncoder.readCounterInt(); //Read New large encoder value (Editable value)
      LargeEncoder.updateStatus();               //Clear Events
      LargeEncoder.readStatus();                 //Clear Events
      switch (UnderscoreIndex)
      { //Select actions dependant upon which value is being edited
      case 0:
        UnitJogAngle = EditValue; //Set Unit angle from dial position
        break;
      case 1:
        TenthsJogAngle = EditValue; //Set Tenths angle from dial position
        break;
      case 2:
        ThouJogAngle = EditValue; //Set Thousandths angle from dial position
        break;
      }
      Serial.printf("Menu Option Value inside encoder change %d\n", UnderscoreIndex);
      StartTime = micros();
      PanelDisplay.clearBuffer();                                                  //Clear display buffer
      sprintf(tmp, "%+04d,%02d%02d°", UnitJogAngle, TenthsJogAngle, ThouJogAngle); //Convert numbers to string
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);                                //Display Angle
      PanelDisplay.drawStr(LeftMargin, 18, uString);                               //Display Underline
      PanelDisplay.sendBuffer();                                                   //Display buffer
      Period = micros() - StartTime;
      Serial.printf(" Elapsed time >%d uS\n", Period);
      StepAngle = UnitJogAngle + TenthsJogAngle / 100.0 + ThouJogAngle / 10000.0; //Calculate StepAngle from units, tenths and thousandths
    }

  } while (mainBtn != Clicked); //repeat until main button pressed

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Turn button Red

  PanelDisplay.clearBuffer(); //Display Angle step, count
  sprintf(tmp, "Step:%3.4f", StepAngle);
  PanelDisplay.drawStr(LeftMargin, Line1, tmp);
  sprintf(tmp, "Rev:%2d Step#:%2d", Revolutions, StepCount);
  PanelDisplay.drawStr(LeftMargin, Line2, tmp);
  sprintf(tmp, "Angle:%3.3f", CumulativeAngle);
  PanelDisplay.drawStr(LeftMargin, Line3, tmp);
  PanelDisplay.sendBuffer();

  LargeEncoder.writeCounter((int32_t)0); // Reset the encoder value
  LargeEncoder.writeMax((int32_t)1000);  //Set max to arbitrary 10000 angle steps
  LargeEncoder.writeMin((int32_t)-1000); //Set min to arbitrary -10000 angle steps
  LargeEncoder.writeStep((int32_t)1);    //Set step value to 1

  do
  { //Monitor main encoder for changes and step motor on each change

    PanelEncoder.loop();                          //Query panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get main button state (to detect if canceled)

    if (digitalRead(EncChangePin) == LOW)
    {                                            //Check if main dial changed, only execute on change
      StepCount = LargeEncoder.readCounterInt(); //Get position of large encoder
      LargeEncoder.updateStatus();               //Clear Events
      LargeEncoder.readStatus();                 //Clear Events

      CumulativeAngle = StepAngle * StepCount; //Calculate Cumulative angle, add  in new encoder postion * steps per position
      Revolutions = CumulativeAngle / 360;     //Calculate revolutions

      PanelDisplay.clearBuffer(); //Update Display
      sprintf(tmp, "Step:%3.4f", StepAngle);
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "R:%2d S#:%2d", Revolutions, StepCount);
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      sprintf(tmp, "Ang:%3.3f", CumulativeAngle);
      PanelDisplay.drawStr(LeftMargin, Line3, tmp);
      PanelDisplay.sendBuffer();

      AbsStepperPosition = StepsPerDegree * CumulativeAngle; //Calculate Absolute Stepper position using hardware parameters (steps per degree
      stepper.moveTo(AbsStepperPosition);                    //move stepper to desired steps away from origin
    }
  } while (mainBtn != Clicked); //repeat until main butten pressed to return to main menu
  stepper.stop();
  stepper.disableOutputs();             //Disables output from Stepper controller to save current\heating
  PanelEncoder.setLED(LED_GREEN, true); //Main button to Green
  PanelEncoder.setLED(LED_RED, false);
}

int GetDivisions()
{ //Edit\Capture number of required divisions

  char tmp[20];      //temp string to hold num to char conversions
  int Divisions = 0; //initial Divisions to 0

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  PanelDisplay.clearBuffer();
  PanelDisplay.drawStr(LeftMargin, Line1, "Enter divisions");
  sprintf(tmp, "Div: %2d", Divisions);
  PanelDisplay.drawStr(LeftMargin, Line2, tmp);
  PanelDisplay.sendBuffer();

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)100000);        //Max to arbitrary 10,000
  LargeEncoder.writeMin((int32_t)0);             //Min to 0
  LargeEncoder.writeCounter((int32_t)Divisions); //Set large encoder to Divisions
  LargeEncoder.writeStep((int32_t)1);            //Set step to 1
  LargeEncoder.writeInterruptConfig(0x18);       //Change on rotation only
  LargeEncoder.writeAntibouncingPeriod(20);      //Debounce steps

  PanelEncoder.begin();
  PanelEncoder.setLED(LED_GREEN, true); //Turn main Button Green
  PanelEncoder.setLED(LED_RED, false);
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  do
  { //Monitor for large encoder movement and main button press

    PanelEncoder.loop();                          //Query panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get main button status

    if (digitalRead(EncChangePin) == LOW)
    {                                             //Execute if dial moved
      Divisions = LargeEncoder.readCounterLong(); //Read dial value and update Divisions
      LargeEncoder.updateStatus();                //Clear change event
      LargeEncoder.readStatus();                  //Clear status

      Serial.printf("Divisions %d\n", Divisions);
      Serial.printf("Main but: %d\n", mainBtn);
      Serial.printf("Wheel but: %d\n", wheelBtn);

      PanelDisplay.clearBuffer();
      PanelDisplay.drawStr(LeftMargin, Line1, "Enter divisions"); //Display Enter Divisions
      sprintf(tmp, "Div: %2d", Divisions);                        //Display Divisions
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      PanelDisplay.sendBuffer();
    }
  } while (mainBtn != Clicked); //Exit on main button press

  return Divisions; //Return Divisions dialed in.
}

void Divisions(int Divisions)
{ //Step set number of divisions of a circle

  char tmp[20];                        //Hold temp string for formatting from number ot string
  int Division = 0;                    //Set default current division to 0
  float StepAngle = 360.0 / Divisions; //Calculate Stepangle from number of divisions
  float Angle = 0;                     //Angle in degrees
  int AngleInSteps = 0;                //Angle in steps

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  PanelDisplay.clearBuffer();
  sprintf(tmp, "Divs: %2d", Divisions);
  PanelDisplay.drawStr(LeftMargin, Line1, tmp);
  sprintf(tmp, "Div: %2d", Division);
  PanelDisplay.drawStr(LeftMargin, Line2, tmp);
  sprintf(tmp, "Angle: %3.3f", Angle);
  PanelDisplay.drawStr(LeftMargin, Line3, tmp);
  PanelDisplay.sendBuffer();

  LargeEncoder.reset(); //reset large encoder
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_ENABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER); //Start new encoder, Wrap values, CW increase
  LargeEncoder.writeMax((int32_t)Divisions);                                   //Max counter to Divisions
  LargeEncoder.writeMin((int32_t)0);                                           //min to 0
  LargeEncoder.writeCounter((int32_t)Division);                                //Large encoder value to Division
  LargeEncoder.writeStep((int32_t)1);                                          //Set Steps to 1
  LargeEncoder.writeInterruptConfig(0x18);                                     //Only trigger activity on dial rotation
  LargeEncoder.writeAntibouncingPeriod(20);                                    //DeBounce signals

  PanelEncoder.begin();
  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true);   //Turn main button Red
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  stepper.enableOutputs();       //Enable Stepper Driver
  stepper.setCurrentPosition(0); //Set current motor position to 0 (Home)

  do
  { //Increment steps for large encoder value, stop when button pressed

    PanelEncoder.loop();                          //Query Panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Query Main button state

    if (digitalRead(EncChangePin) == LOW)
    {                                            //Execute if large encoder moved
      Division = LargeEncoder.readCounterLong(); //Read encoder value and set as current Division
      Angle = Division * StepAngle;              //Absolute angle = Current Dision number * step angle (consider accuracy)
      AngleInSteps = StepsPerDegree * Angle;     //Convert absolute angle to Steps from Home Needs calculating from Params

      LargeEncoder.updateStatus(); //CLear Events
      LargeEncoder.readStatus();   //Clear Events

      Serial.printf("Division: %d\n", Division);
      Serial.printf("Angle: %.2f\n", Angle);
      Serial.printf("Angle in Steps: %d\n", AngleInSteps);

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Divs:%2d", Divisions);
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "Div:%2d", Division);
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      sprintf(tmp, "Angle:%3.3f", Angle);
      PanelDisplay.drawStr(LeftMargin, Line3, tmp);
      PanelDisplay.sendBuffer();

      stepper.moveTo(AngleInSteps); //Move motor to absolute Angle in Steps from Home
    }
  } while (mainBtn != Clicked); //Exit if main button pressed.

  PanelEncoder.setLED(LED_GREEN, true); //Turn main button Green
  PanelEncoder.setLED(LED_RED, false);
  stepper.stop();           //Stop Motor
  stepper.disableOutputs(); //Disables output from Stepper controller to save current\heating
}

void RunContinuous()
{ //Run motor continuous varyinf speed with main dial until button pressed
  char tmp[20];
  int Speed = 0;            //Desired Motor Speed Steps\second
  int SpeedStepValue = 100; //Speed Step value in steps

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  PanelDisplay.clearBuffer();
  sprintf(tmp, "Speed: %d", Speed); //Display Speed
  PanelDisplay.drawStr(LeftMargin, Line1, tmp);
  PanelDisplay.sendBuffer();

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)100000);          //Max to  10,000
  LargeEncoder.writeMin((int32_t)-100000);         //Min to -10,000
  LargeEncoder.writeCounter((int32_t)Speed);       //Set main dial value to Speed
  LargeEncoder.writeStep((int32_t)SpeedStepValue); //Set step value to Speed Step Value
  LargeEncoder.writeInterruptConfig(0x18);         //Change event only on dial rotation
  LargeEncoder.writeAntibouncingPeriod(20);        //Debounce steps

  PanelEncoder.begin();                 //reset Panel encoder
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  stepper.enableOutputs();       //Enable Stepper Driver
  stepper.setCurrentPosition(0); //Reset motor to 0 (Home)

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Set main button to Red

  do
  { //Monitor main button press and main dial for rotation

    PanelEncoder.loop();                          //Query Panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get Panel Main button Status

    if (digitalRead(EncChangePin) == LOW)
    {                                         //Execute if Main encoder changed
      Speed = LargeEncoder.readCounterLong(); //Read encoder value and set Speed
      LargeEncoder.updateStatus();            //Clear Encoder Status
      LargeEncoder.readStatus();              //Clear Encoder Status

      Serial.printf("Encoder Speed: %d\n", Speed);

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Speed: %+d", Speed); //Update display with Speed
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      PanelDisplay.sendBuffer();
      stepper.setMaxSpeed(Speed); //Set maz speed of Motor to new Speed
      if (Speed > 0)
        stepper.move(1000000); //Move motor CW if Max Speed > 0
      if (Speed < 0)
        stepper.move(-1000000); //Move motor CCW if max speed < 0 (arbitrary 10000000 for constant run)
    }
  } while (mainBtn != Clicked); //Exit of main button pressed

  stepper.stop();                       //Stop Motor
  stepper.disableOutputs();             //Disables output from Stepper controller to save current\heating
  PanelEncoder.setLED(LED_GREEN, true); //Turn main button Green
  PanelEncoder.setLED(LED_RED, false);
}

void Oscillate()
{ //Move back and forward choosen angle.
  char tmp[20];
  float AbsoluteAngle = 0.0; //Stores Angular position after each jog;
  int EncoderAngle = AbsoluteAngle * 100;
  int OptValue = 0;
  float AnglePerMenuOpt[5] = {1.0, 10.0, 15.0, 0.01, 0.1};
  int StepValue = 0;
  float StepValuef = 0.0;
  int v = 0;
  int lastEncoderPos = -1; //Last encoder position - set to -1 to monitor loop runs at least once to set display
  int encoderPos = 0;
  int Speed = 0;
  int SpeedStepValue = 100;
  int MoveSteps = 0; //Holds absolute steps to move calculated from desired angle

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)99999);            //Set max dial value to an arbitrary 100000
  LargeEncoder.writeMin((int32_t)-99999);           //Set min dial value to an arbitrary -10000
  LargeEncoder.writeCounter((int32_t)EncoderAngle); //Set cencoder value to Angle
  LargeEncoder.writeStep((int32_t)100);             //Set Steps to 100
  LargeEncoder.writeInterruptConfig(0x18);          //Limit events to encoder turning
  LargeEncoder.writeAntibouncingPeriod(20);         //Set antiBounce to 200mS

  PanelEncoder.setLED(LED_GREEN, true);
  PanelEncoder.setLED(LED_RED, false); //Set main button to Green

  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  stepper.setCurrentPosition(0); //Set current motor position to Zero

  do
  {

    PanelEncoder.loop();                          //Query state of panel
    mainBtn = PanelEncoder.getButton(MainButton); //Get state of main button

    v = PanelEncoder.getValue(); //Get value of encoder -1, 0 or 1
    encoderPos += v;             //Add to cumulative counter
    if (encoderPos != lastEncoderPos)
    {                                                 //If change to rotary dial...
      lastEncoderPos = encoderPos;                    //last = current to keep track of change
      OptValue = abs(encoderPos % 5);                 //Create Option value from 0 to 4 that will index increments
      StepValuef = AnglePerMenuOpt[OptValue] * 100.0; //Convert step value to integer by multiplying by 100
      StepValue = static_cast<int>(StepValuef);       //Convert step value to integer by multiplying by 100
      LargeEncoder.writeStep((int32_t)StepValue);     //Write to large encoder value for it to handle increments

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Step: %05.2f", AnglePerMenuOpt[OptValue]); //Display Step selected
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "Angle:%+07.2f", AbsoluteAngle); //Display Total angle
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      PanelDisplay.sendBuffer();
      Serial.printf("Menu Option Value inside menu change %d\n", OptValue);
      Serial.printf("Step inside {}: %f\n", AnglePerMenuOpt[OptValue]);
      Serial.printf("Step inside {} *100: %f\n", AnglePerMenuOpt[OptValue] * 100.0);
    }

    if (digitalRead(EncChangePin) == LOW)
    {                                                //Execute of Large dial moved
      EncoderAngle = LargeEncoder.readCounterLong(); //Read new Integer angle
      LargeEncoder.updateStatus();                   //Clear Events
      LargeEncoder.readStatus();                     //Clear Events

      AbsoluteAngle = EncoderAngle / 100.0; //Convert integer angle to real by div 100

      Serial.printf("Menu Option Value inside encoder change %d\n", OptValue);
      Serial.printf("EncoderAngle: %d\n", EncoderAngle);
      Serial.printf("AboluteAngle: %f\n", AbsoluteAngle);
      Serial.printf("Step: %f\n", AnglePerMenuOpt[OptValue] * 100.0);

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Step: %05.2f", AnglePerMenuOpt[OptValue]); //Display Step value
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      sprintf(tmp, "Angle:%+07.2f", AbsoluteAngle); //Display absolute total angle
      PanelDisplay.drawStr(LeftMargin, Line2, tmp);
      PanelDisplay.sendBuffer();
    }
  } while (mainBtn != Clicked); //repeat until main button is pressed

  //Main button pressed and have desired angle in Float AbsoluteAngle or Int(*100) in EncoderAngle
  //Move between 0 and desired angle anf vary speed bu main dial, Main button cancels\stop returns to main menu

  PanelEncoder.setLED(LED_GREEN, false); //Set main button LED to Red
  PanelEncoder.setLED(LED_RED, true);

  PanelDisplay.clearBuffer();
  sprintf(tmp, "Speed: %d", Speed); //Display Speed
  PanelDisplay.drawStr(LeftMargin, Line1, tmp);
  PanelDisplay.sendBuffer();

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)100000);          //Max to  10,000
  LargeEncoder.writeMin((int32_t)0);               //Min to 0
  LargeEncoder.writeCounter((int32_t)Speed);       //Set main dial value to Speed
  LargeEncoder.writeStep((int32_t)SpeedStepValue); //Set step value to Speed Step Value
  LargeEncoder.writeInterruptConfig(0x18);         //Change event only on dial rotation
  LargeEncoder.writeAntibouncingPeriod(20);        //Debounce steps

  PanelEncoder.begin();                 //reset Panel encoder
  PanelEncoder.resetButton(MainButton); //Reset main button

  stepper.enableOutputs();
  stepper.setCurrentPosition(0); //Reset motor to 0 (Home)
  stepper.setMaxSpeed(Speed);    //Set maz speed of Motor to new Speed
  MoveSteps = StepsPerDegree * EncoderAngle / 100;

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Set main button to Red

  do
  { //Monitor main button press and main dial for rotation

    PanelEncoder.loop();                          //Query Panel status
    mainBtn = PanelEncoder.getButton(MainButton); //Get Panel Main button Status

    if (digitalRead(EncChangePin) == LOW)
    {                                         //Execute if Main encoder changed
      Speed = LargeEncoder.readCounterLong(); //Read encoder value and set Speed
      LargeEncoder.updateStatus();            //Clear Encoder Status
      LargeEncoder.readStatus();              //Clear Encoder Status

      Serial.printf("Encoder Speed: %d\n", Speed);

      PanelDisplay.clearBuffer();
      sprintf(tmp, "Speed: %d", Speed); //Update display with Speed
      PanelDisplay.drawStr(LeftMargin, Line1, tmp);
      PanelDisplay.sendBuffer();
      stepper.setMaxSpeed(Speed); //Set maz speed of Motor to new Speed
    }
    if (stepper.currentPosition() == 0)
      stepper.moveTo(MoveSteps); //Move motor to prescribed angle
    if (stepper.currentPosition() == MoveSteps)
      stepper.moveTo(0); //Move motor back to zero
    PanelDisplay.clearBuffer();
    sprintf(tmp, "Speed: %d", Speed); //Update display with Speed
    PanelDisplay.drawStr(LeftMargin, Line1, tmp);
    sprintf(tmp, "Angle:%+07.2f", float(stepper.currentPosition() / (StepsPerDegree * 1.0))); //Update display with Speed
    PanelDisplay.drawStr(LeftMargin, Line2, tmp);
    PanelDisplay.sendBuffer();

  } while (mainBtn != Clicked); //Exit if main button pressed

  stepper.stop();                       //Stop Motor
  stepper.disableOutputs();             //Disables output from Stepper controller to save current\heating
  PanelEncoder.setLED(LED_GREEN, true); //Turn main button Green
  PanelEncoder.setLED(LED_RED, false);
}
void ParameterSetUp()
{                                                                                                                       //Edit and store hardware parameters (EEPROM limit 200,000 R|W)
  char tmp[20];                                                                                                         //Temp string t hold char version of formatted numbers
  const int NumMenuItems = 5;                                                                                           //Number of menu items
  int MenuSelection = 0;                                                                                                //Current menu item selected
  const char *MenuText[NumMenuItems] = {"Motor Steps:", "Micro Steps:", "Table Ratio:", "Jog Speed:", "Acceleration:"}; //Array of parameters
  int v = 0;                                                                                                            //Holds rotary button value -1, 0 or 1
  int encoderPos = 0;                                                                                                   //Holds current cumulative position of rotary button
  int lastEncoderPos = 0;                                                                                               //Holds last rotary button position to detect change

  int CurMotorStepsPerRev = MotorStepsPerRev; //Make copy of current parameters so that only changes need to be saved
  int CurControllerMicroSteps = ControllerMicroSteps;
  int CurTableRatio = TableRatio;
  int CurJogSpeed = JogSpeed;
  int CurAccelleration = Accelleration;

  LargeEncoder.reset();
  LargeEncoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE |
                     i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE |
                     i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);
  LargeEncoder.writeMax((int32_t)100000);   //Max value to arbitrary 100000
  LargeEncoder.writeMin((int32_t)0);        //Min value to 0
  LargeEncoder.writeCounter((int32_t)0);    //Set encoder position to 0
  LargeEncoder.writeStep((int32_t)1);       //Set step to 1
  LargeEncoder.writeInterruptConfig(0x18);  //Changes on rotation only
  LargeEncoder.writeAntibouncingPeriod(20); //Debounce changes 200ms

  PanelDisplay.setFont(BASE_FONT);
  PanelDisplay.setFontMode(0);
  PanelDisplay.setDrawColor(1);

  PanelEncoder.begin();
  PanelEncoder.setLED(LED_GREEN, true); //Turn Main button green
  PanelEncoder.setLED(LED_RED, false);
  PanelEncoder.flushFifo();             //clear any queued events
  PanelEncoder.resetButton(MainButton); //reset main button state

  lastEncoderPos = -1; //Set last position to -1 to ensure loops runs through at least once

  do
  {

    PanelEncoder.loop();                          //Query status of panel
    mainBtn = PanelEncoder.getButton(MainButton); //Read status of Main button

    v = PanelEncoder.getValue(); //Read button encoder value -1,0 or 1
    encoderPos += v;             //Current position = curent pos + rotary position
    if (encoderPos != lastEncoderPos)
    {                                                 //If current different from previous
      lastEncoderPos = encoderPos;                    //last = current
      MenuSelection = abs(encoderPos % NumMenuItems); //Create menu index number from cumulative position

      PanelDisplay.clearBuffer();
      PanelDisplay.drawStr(LeftMargin, Line1, "Setup Menu");            //Display Menu
      PanelDisplay.drawStr(LeftMargin, Line2, MenuText[MenuSelection]); //Display Menu item
      switch (MenuSelection)
      { //Display menu item and value depending on index value
      case 0:
        LargeEncoder.writeCounter((int32_t)CurMotorStepsPerRev);
        LargeEncoder.writeStep((int32_t)100); //set steps to 100
        sprintf(tmp, "%d", CurMotorStepsPerRev);
        PanelDisplay.drawStr(LeftMargin, Line3, tmp);
        break;
      case 1:
        LargeEncoder.writeCounter((int32_t)CurControllerMicroSteps);
        LargeEncoder.writeStep((int32_t)1); //set Steps to 1
        sprintf(tmp, "%d", CurControllerMicroSteps);
        PanelDisplay.drawStr(LeftMargin, Line3, tmp);
        break;
      case 2:
        LargeEncoder.writeCounter((int32_t)CurTableRatio);
        LargeEncoder.writeStep((int32_t)1); //set steps to 1
        sprintf(tmp, "%d", CurTableRatio);
        PanelDisplay.drawStr(LeftMargin, Line3, tmp);
        break;
      case 3:
        LargeEncoder.writeCounter((int32_t)CurJogSpeed);
        LargeEncoder.writeStep((int32_t)100); //Set steps to 100
        sprintf(tmp, "%d", CurJogSpeed);
        PanelDisplay.drawStr(LeftMargin, Line3, tmp);
        break;
      case 4:
        LargeEncoder.writeCounter((int32_t)CurAccelleration);
        LargeEncoder.writeStep((int32_t)100); //Set steps to 100
        sprintf(tmp, "%d", CurAccelleration);
        PanelDisplay.drawStr(LeftMargin, Line3, tmp);
        break;
      }
      PanelDisplay.sendBuffer(); //Display buffer
      Serial.printf("Menu Option Vinside paramsetup %d\n", MenuSelection);
    }

    if (digitalRead(EncChangePin) == LOW)
    {                                                //Check for main dial movement
      int DialValue = LargeEncoder.readCounterInt(); //Read dial value
      LargeEncoder.updateStatus();                   //Clear change event
      LargeEncoder.readStatus();                     //Clear change event

      PanelDisplay.clearBuffer();
      PanelDisplay.drawStr(LeftMargin, Line1, "Setup Menu");            //Display menu
      PanelDisplay.drawStr(LeftMargin, Line2, MenuText[MenuSelection]); //Display parameter from index

      switch (MenuSelection)
      { //Set parameter from menu index position
      case 0:
        CurMotorStepsPerRev = DialValue;
        break;
      case 1:
        CurControllerMicroSteps = DialValue;
        break;
      case 2:
        CurTableRatio = DialValue;
        break;
      case 3:
        CurJogSpeed = DialValue;
        break;
      case 4:
        CurAccelleration = DialValue;
        break;
      }
      sprintf(tmp, "%d", DialValue); //display value being edited
      PanelDisplay.drawStr(LeftMargin, Line3, tmp);
      PanelDisplay.sendBuffer();
    }

  } while (mainBtn != Clicked); //Exit if main button pressed.

  PanelEncoder.setLED(LED_GREEN, false);
  PanelEncoder.setLED(LED_RED, true); //Turn main button Red?

  preferences.begin("RotaryTable", false); //Start NameSpace with read\write access
  if (CurMotorStepsPerRev != MotorStepsPerRev)
  {
    preferences.putUInt("MSPR", CurMotorStepsPerRev);
    MotorStepsPerRev = CurMotorStepsPerRev;
  }
  //Only Store Motor steps per Rev if changed
  if (CurControllerMicroSteps != ControllerMicroSteps)
  {
    preferences.putUInt("CMS", CurControllerMicroSteps); //Only Store Microsteps if changed
    ControllerMicroSteps = CurControllerMicroSteps;
  }
  if (CurTableRatio != TableRatio)
  {
    preferences.putUInt("TableRatio", CurTableRatio); //Only Store Table Ratio if changed
    TableRatio = CurTableRatio;
  }
  if (CurJogSpeed != JogSpeed)
  {
    preferences.putUInt("JogSpeed", CurJogSpeed); //Only Store Jog Speed if changed
    JogSpeed = CurJogSpeed;
    stepper.setMaxSpeed(JogSpeed);
  }

  if (CurAccelleration != Accelleration)
  {
    preferences.putUInt("Accelleration", CurAccelleration); //Only Store Accelleration (steps\s\s) if changed
    Accelleration = CurAccelleration;
    stepper.setAcceleration(Accelleration);
  }
  preferences.end();

  //Recalculate derived constants from physical parameters
  StepsPerRev = MotorStepsPerRev * ControllerMicroSteps * TableRatio; //eg 200 * 8 *90 = 144000 Steps per rev
  StepsPerDegree = StepsPerRev / 360;                                 //eg 144000/360 = 400 Steps per degree
  DegreePerStep = 360 / StepsPerRev;                                  //eg 360 / 144000 = 0.0025 Degrees per step
}