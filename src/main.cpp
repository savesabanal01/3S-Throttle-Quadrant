#include <Arduino.h>
/** Project CPP includes. */
#include "TLx493D_inc.hpp"

// Include Joystick library
#include <Joystick.h>

// Include Moving Averages library
#include <movingAvg.h>

using namespace ifx::tlx493d;

/** Declaration of three sensor objects with the same default address (A0).
 *
 *  ATTENTION
 *  All Kit2Go/S2Go boards have pull-up resistors attached to the IIC SCL and SDA lines.
 *  In case of sensors dut2 and dut3 these resistors actually pull-down the SCL/SDA lines while
 *  the sensors are NOT powered. Therefore the pull-up resistors of dut2 and dut3 have to be removed
 *  for this example to work. Otherwise not even dut1 can be initialized properly. Alternatively,
 *  pull-up resistors for all devices are removed and external pull-ups are added to SCL/SDA.
 */
TLx493D_A1B6 dut1(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 dut2(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 dut3(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 dut4(Wire, TLx493D_IIC_ADDR_A0_e);

// Variable declarations
int maxUnchangedValue = 50; // number of unchanged values to detect if sensor has stuck

int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int counter4 = 0;

double prevVal1 = 0;
double prevVal2 = 0;
double prevVal3 = 0;
double prevVal4 = 0;

// Initial values for minimum and maximum angles
double minAngle = -163;
double maxAngle = -65;

// Angles to be sent to the joystick library after taking out outliers
double finalAngle1 = minAngle;
double finalAngle2 = minAngle;
double finalAngle3 = minAngle;
double finalAngle4 = minAngle;

// Function declarations
double scaleValue(float x, float in_min, float in_max, float out_min, float out_max);
void checkSensorsAndReset();
void checkToggleSwitches();

// Moving average objects take the average for 50 readings
movingAvg mAvg1(15);
movingAvg mAvg2(15);
movingAvg mAvg3(15);
movingAvg mAvg4(15);

int sw1State = LOW;
int sw2State = LOW;
int sw3State = LOW;
int sw4State = LOW;

// Debounce variables

// Buttons for A3144 HES, by default all buttons are off
int button1 = 0;
int button2 = 0;
int button3 = 0;
int button4 = 0;

// Previous state of buttons
int prevButton1 = 0;
int prevButton2 = 0;
int prevButton3 = 0;
int prevButton4 = 0;

void setup()
{
  Serial.begin(115200);
  delay(3000);

  /** In this example we're using the XMC4700-Relax-Kit.
   *  Here we're using three GPIOs to power up the sensors, one
   *  after the other. This is done by defining the pins with the
   *  Board Support Class and its functions. The power up pins are
   *  enabled during the begin() call to the sensor object as done below.
   */

  dut1.setPowerPin(0, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax P1.10
  dut2.setPowerPin(1, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax XMC P1.11
  dut3.setPowerPin(2, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax XMC P1.11
  dut4.setPowerPin(3, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax XMC P3.10

  /** Here we're enabling one sensor after another. This procedure
   *  is necessary in order to avoid an interrupt to occur on the SCL line before
   *  interrupts are disabled in the sensor's begin() method. Additionally, the IIC address
   *  is changed from the default A0 to a distinct one for each sensor.
   */

  dut1.begin(true, false, false, true);
  dut1.setIICAddress(TLx493D_IIC_ADDR_A3_e);
  dut1.printRegisters();

  dut2.begin(true, false, false, false);
  dut2.setIICAddress(TLx493D_IIC_ADDR_A2_e);
  dut2.printRegisters();

  dut3.begin(true, false, false, false);
  dut3.setIICAddress(TLx493D_IIC_ADDR_A1_e);
  dut3.printRegisters();

  dut4.begin(true, false, false, false);
  dut4.setIICAddress(TLx493D_IIC_ADDR_A0_e);
  dut4.printRegisters();

  // Initialize moving average function
  mAvg1.begin();
  mAvg2.begin();
  mAvg3.begin();
  mAvg4.begin();

  // Buttons on pins setting
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  // Analog pins 26 to 28 used as digital inputs
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);

  // Initialize joystick library
  Joystick.begin();

  Serial.println("setup done.\n");
}

/** In the loop we're reading out the temperature value as well as the magnetic values in X, Y, Z-direction
 *  of all three sensors. After that they're all printed to the serial output.
 */
void loop()
{

  double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0, temp4 = 0.0;
  double valX1 = 0, valY1 = 0, valZ1 = 0, valX2 = 0, valY2 = 0, valZ2 = 0, valX3 = 0, valY3 = 0, valZ3 = 0, valX4 = 0, valY4 = 0, valZ4 = 0;
  double angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0;

  int val1 = 0, val2 = 0, val3 = 0, val4 = 0;
  int avgVal1 = 0, avgVal2 = 0, avgVal3 = 0, avgVal4 = 0; // moving averages

  dut1.getMagneticFieldAndTemperature(&valX1, &valY1, &valZ1, &temp1);
  angle1 = atan2(valY1, valX1) * RAD_TO_DEG;
  dut2.getMagneticFieldAndTemperature(&valX2, &valY2, &valZ2, &temp2);
  angle2 = atan2(valY2, valX2) * RAD_TO_DEG;
  dut3.getMagneticFieldAndTemperature(&valX3, &valY3, &valZ3, &temp3);
  angle3 = atan2(valY3, valX3) * RAD_TO_DEG;
  dut4.getMagneticFieldAndTemperature(&valX4, &valY4, &valZ4, &temp4);
  angle4 = atan2(valY4, valX4) * RAD_TO_DEG;

  checkToggleSwitches(); // Check toggle switches first
  // Check if the sensor readings are changing for the next 50 counts. If not, the sensor is stuck and needs to be reset

  if (prevVal1 != valX1 || valX1 == 0)
  {
    prevVal1 = valX1;
    counter1 = 0;
  }
  else
  {
    counter1++;
  }

  if (prevVal2 != valX2 || valX1 == 0)
  {
    prevVal2 = valX2;
    counter2 = 0;
  }
  else
  {
    counter2++;
  }

  if (prevVal3 != valX3 || valX3 == 0)
  {
    prevVal3 = valX3;
    counter3 = 0;
  }
  else
  {
    counter3++;
  }

  if (prevVal4 != valX4 || valX4 == 0)
  {
    prevVal4 = valX4;
    counter4 = 0;
  }
  else
  {
    counter4++;
  }

  checkSensorsAndReset(); // check if sensors are stuck andr reset if needed

  // Ensure that all angles are within minAngle and maxAngle range to remove outliers

  if (angle1 >= minAngle && angle1 <= maxAngle)
  {
    finalAngle1 = angle1;
  }

  if (angle2 >= minAngle && angle2 <= maxAngle)
  {
    finalAngle2 = angle2;
  }

  if (angle3 >= minAngle && angle3 <= maxAngle)
  {
    finalAngle3 = angle3;
  }

  if (angle4 >= minAngle && angle4 <= maxAngle)
  {
    finalAngle4 = angle4;
  }

  // Scale values and get moving average
  val1 = (int)scaleValue(finalAngle1, minAngle, maxAngle, 0, 1023);
  avgVal1 = mAvg1.reading(val1);
  val2 = (int)scaleValue(finalAngle2, minAngle, maxAngle, 0, 1023);
  avgVal2 = mAvg2.reading(val2);
  val3 = (int)scaleValue(finalAngle3, minAngle, maxAngle, 0, 1023);
  avgVal3 = mAvg3.reading(val3);
  val4 = (int)scaleValue(finalAngle4, minAngle, maxAngle, 0, 1023);
  avgVal4 = mAvg4.reading(val4);

  // Below is only for debugging purposes

  Serial.println("========================================");
  Serial.print("Angle 1:\t");
  Serial.print(finalAngle1);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");
  Serial.print("Angle 2:\t");
  Serial.print(finalAngle2);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");
  Serial.print("Angle 3:\t");
  Serial.print(finalAngle3);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");
  Serial.print("Angle 4:\t");
  Serial.print(finalAngle4);
  Serial.println(" degrees");
  Serial.println("----------------------------------------");

  Joystick.useManualSend(true);
  Joystick.use8bit(false);

  // Send axis values to joystick
  Joystick.X(avgVal1);
  delay(2); // little delay
  Joystick.Y(avgVal2);
  delay(2); // little delay
  Joystick.Z(avgVal3);
  delay(2); // little delay
  Joystick.sliderLeft(avgVal4);
  delay(2); // little delay

  // Send the button state from the A3144 Hall Effect Sensor
  Joystick.button(1, sw1State);
  Joystick.button(2, sw2State);
  Joystick.button(3, sw3State);
  Joystick.button(4, sw4State);

  // Send the button state from all other pins
  Joystick.button(5, !digitalRead(10));
  Joystick.button(6, !digitalRead(11));
  Joystick.button(7, !digitalRead(12));
  Joystick.button(8, !digitalRead(13));
  Joystick.button(9, !digitalRead(14));
  Joystick.button(10, !digitalRead(15));
  Joystick.button(11, !digitalRead(16));
  Joystick.button(12, !digitalRead(17));
  Joystick.button(13, !digitalRead(18));
  Joystick.button(14, !digitalRead(19));
  Joystick.button(15, !digitalRead(20));
  Joystick.button(16, !digitalRead(21));
  Joystick.button(17, !digitalRead(22));
  Joystick.button(18, !digitalRead(26));
  Joystick.button(19, !digitalRead(27));
  Joystick.button(20, !digitalRead(28));
  Joystick.send_now();
  delay(15);
}

double scaleValue(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkSensorsAndReset()
{
  if (counter1 >= maxUnchangedValue)
  {
    Serial.println("Sensor1 Error!!!");
    dut1.reset();
    dut1.setPowerPin(0, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax P1.10
    dut1.begin(true, false, false, true);
    dut1.setIICAddress(TLx493D_IIC_ADDR_A3_e);
    counter1 = 0;
  }
  else if (counter2 >= maxUnchangedValue)
  {
    Serial.println("Sensor2 Error!!!");
    dut2.reset();
    dut2.setPowerPin(1, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax P1.10
    dut2.begin(true, false, false, true);
    dut2.setIICAddress(TLx493D_IIC_ADDR_A2_e);
    counter2 = 0;
  }
  else if (counter3 >= maxUnchangedValue)
  {
    Serial.println("Sensor3 Error!!!");
    dut3.reset();
    dut3.setPowerPin(2, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax P1.10
    dut3.begin(true, false, false, true);
    dut3.setIICAddress(TLx493D_IIC_ADDR_A1_e);
    counter3 = 0;
  }
  else if (counter4 >= maxUnchangedValue)
  {
    Serial.println("Sensor4 Error!!!");
    dut4.reset();
    // dut4.setPowerPin(3, OUTPUT, INPUT, HIGH, LOW, 0, 250000); // XMC4700 Relax P1.10
    // dut4.begin(true, false, false, true);
    // dut4.setIICAddress(TLx493D_IIC_ADDR_A0_e);
    counter4 = 0;
  }
}
// Check the A3144 HES switches status
void checkToggleSwitches()
{
  int reading;
  // Serial.println("I am here!");

  // check button 1
  reading = digitalRead(6);
  if (reading == LOW)
    button1 = 1;
  else
    button1 = 0;
  if (button1 == 1 && prevButton1 == 0)
  { // If Button1 is 1 and prevButton is 0 (This means basically means "If the button was just pressed"
    // Put what you want to happen when the "Switch" is on here
    sw1State = !sw1State;
  }
  prevButton1 = button1;

  // check button 2
  reading = digitalRead(7);
  if (reading == LOW)
    button2 = 1;
  else
    button2 = 0;
  if (button2 == 1 && prevButton2 == 0)
  { // If Button1 is 1 and prevButton is 0 (This means basically means "If the button was just pressed"
    // Put what you want to happen when the "Switch" is on here
    sw2State = !sw2State;
  }
  prevButton2 = button2;

  // check button 3
  reading = digitalRead(8);
  if (reading == LOW)
    button3 = 1;
  else
    button3 = 0;
  if (button3 == 1 && prevButton3 == 0)
  { // If Button1 is 1 and prevButton is 0 (This means basically means "If the button was just pressed"
    // Put what you want to happen when the "Switch" is on here
    sw3State = !sw3State;
  }
  prevButton3 = button3;

  // check button 4
  reading = digitalRead(9);
  if (reading == LOW)
    button4 = 1;
  else
    button4 = 0;
  if (button4 == 1 && prevButton4 == 0)
  { // If Button1 is 1 and prevButton is 0 (This means basically means "If the button was just pressed"
    // Put what you want to happen when the "Switch" is on here
    sw4State = !sw4State;
  }
  prevButton4 = button4;
  // delay for debounce
  delay(20);
}