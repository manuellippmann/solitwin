// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <LSM303.h>
#include <math.h>

// DEFINE SENSORS
Adafruit_MLX90393 WindSensor = Adafruit_MLX90393();
Adafruit_LSM303_Accel_Unified AccelSensor = Adafruit_LSM303_Accel_Unified(54321);
LSM303 compass;

// DEFINE SERVOS
Servo sailServo;
Servo rudderServo;

// DATA DECLARATIONS
int throttleSensData;
int rudderSensData;
float throttleData;
float rudderData;
int switch_A_data;
int switch_D_data;
int switch_A;
int switch_D;
int tempSwitch;
float xData, yData, zData, windDirection;
char windSide;
float heading;
float hValue;
int currentHeading;
int course;
int newHeading;
unsigned long timeStamp;

// I/O DECLARATIONS
int sailIn = 9;
int sailOut = 7;
int rudderIn = 10;
int rudderOut = 8;
int switch_A_In = 11;
int switch_D_In = 12;

// SMOOTHING DECLARATIONS
const int numReadings = 11;

// TODO: REMOVE FLOATS FROM SMOOTHING OPERATIONS
int sReadings[numReadings]; // sail
int sReadIndex = 0;
float sTotal = 0;
float sAverage = 0;

int rReadings[numReadings]; // rudder
int rReadIndex = 0;
float rTotal = 0;
float rAverage = 0;

const int magNumReadings = 15;

int magyReadings[magNumReadings]; // magnetometer y
int magyReadIndex = 0;
float magyTotal = 0;
float magyAverage = 0;

int magzReadings[magNumReadings]; // magnetometer z
int magzReadIndex = 0;
float magzTotal = 0;
float magzAverage = 0;

int compReadings[magNumReadings]; // compass
int compReadIndex = 0;
float compTotal = 0;
float compAverage = 0;

int accReadings[magNumReadings]; // acceleration
int accReadIndex = 0;
float accTotal = 0;
float accAverage = 0;

// FUCTION DECLARATIONS

int calcRudderServo();
int calcSailServo();
void updateSwitches();
void calcWindData();
float getCompassDir();
float getHeelingAngle();
int calcMagY(int data);
int calcMagZ(int data);
void navigate(int wantedheading);
int calcRudderAmp(int heading);
int calcSailAngle();

void setup()
{
  // INIT SERIAL TO DEBUG
  Serial.begin(9600);
  // INIT DATA IN
  pinMode(sailIn, INPUT);
  pinMode(rudderIn, INPUT);
  pinMode(switch_A_In, INPUT);
  pinMode(switch_D_In, INPUT);
  // INIT SERVOS
  sailServo.attach(sailOut);
  rudderServo.attach(rudderOut);
  // INIT SENOSRS

  if (!WindSensor.begin())
  {
    while (1)
      ;
  }
  if (!AccelSensor.begin())
  {
    while (1)
      ;
  }

  compass.init();
  compass.enableDefault();
  // compass.m_min = (LSM303::vector<int16_t>){-247, -644, -406};
  // compass.m_max = (LSM303::vector<int16_t>){+278, +437, +14};
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  //INIT SMOOTHING
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    sReadings[thisReading] = 0;
    rReadings[thisReading] = 0;
  }

  for (int thisReading = 0; thisReading < magNumReadings; thisReading++)
  {
    magyReadings[thisReading] = 0;
    magzReadings[thisReading] = 0;
    compReadings[thisReading] = 0;
    accReadings[thisReading] = 0;
  }
}

void loop()
{
  // check for receiver values and set current switch positions
  updateSwitches();
  currentHeading = getCompassDir();

  if (switch_A == 0)
  { // AUTONOMOUS SECTION
    calcWindData();

    if (switch_D == 0 && tempSwitch == 0)
    {
      course = 1; //"closeHauled"
      navigate(-5);
    }
    else if (switch_D == 1 && tempSwitch == 0)
    {
      course = 2; //"beamReach"
      navigate(-5);
    }
    else if (switch_D == 2 && tempSwitch == 0)
    {
      course = 3; //"broadReach"
      navigate(-5);
    }
    else if (switch_D == 0 && tempSwitch == 1)
    {
      course = 3; //"go north"
      navigate(10);
  }

    else if (switch_D == 1 && tempSwitch == 1)
    {
      course = 3; //"go east"
      navigate(90);
    }

    else if (switch_D == 2 && tempSwitch == 1)
    {
      course = 3; //"go south"
      navigate(180);
    }

    Serial.println("windDirection");
    Serial.println(windDirection);
    Serial.println("compassDirection");
    Serial.println(currentHeading);
    Serial.println("----");
    Serial.println("----");
  }
  else if (switch_A == 1)
  { // MANUAL MODE
    sailServo.write(calcSailServo());
    rudderServo.write(calcRudderServo());

    // Serial.println(calcSailServo());
    // calcWindData();
    // Serial.print("windDirection: ");
    // Serial.println(windDirection);
    // Serial.println("--");

    // WIND SENSOR CALIBRATION:
    // calcWindData();
    // WindSensor.readData(&xData, &yData, &zData);
    // Serial.print("windz: ");
    // Serial.println(calcMagZ(zData));
    // Serial.print("windy: ");
    // Serial.println(calcMagY(yData));
    // Serial.println("--");
    // Serial.println(windDirection);
    // Serial.println("--");
  }
};

void updateSwitches()
{
  // get input data for switch A
  switch_A_data = (pulseIn(switch_A_In, HIGH)) - 1000;
  // set switch A value
  if (switch_A_data > 500)
  {
    switch_A = 0;

    throttleSensData = (pulseIn(sailIn, HIGH)) - 1000;
    if (throttleSensData > 400)
    {
      tempSwitch = 0;
    }
    else if (throttleSensData < 400)
    {
      tempSwitch = 1;
    }
  }
  else if (switch_A_data < 150)
  {
    switch_A = 1;
  }

  // get input data for switch D
  switch_D_data = (pulseIn(switch_D_In, HIGH)) - 1000;
  // set switch D data
  if (switch_D_data > 600)
  {
    switch_D = 0;
  }
  else if (switch_D_data > 400 && switch_D_data < 599)
  {
    switch_D = 1;
  }
  else if (switch_D_data < 150)
  {
    switch_D = 2;
  }
}

void calcWindData()
{
  // read data from windsensor, store in variables: &xData, &yData, &zData
  WindSensor.readData(&xData, &yData, &zData);

  // read y curve to determine the side the wind is coming from, set windSide
  if (calcMagY(yData) < 0)
  {
    windSide = 's';
  }
  else
  {
    windSide = 'b';
  }

  // calculate wind direction between 0 deg and 180 deg, set to windDirection
  windDirection = calcMagZ(zData);
  windDirection = (1 - ((windDirection - 600) / 600)) * 180;
};

float getCompassDir()
{
  // get data from sensor
  compass.read();

  heading = compass.heading((LSM303::vector<int>){1, 0, 0});

  // smooth data, the last 15 values are added and a median is found
  compTotal = compTotal - compReadings[compReadIndex];
  compReadings[compReadIndex] = heading;
  compTotal = compTotal + compReadings[compReadIndex];
  compReadIndex = compReadIndex + 1;

  if (compReadIndex >= magNumReadings)
  {
    compReadIndex = 0;
  }

  compAverage = compTotal / magNumReadings;
  // return data
  return compAverage;
}

float getHeelingAngle()
{ // get accelsensor data
  sensors_event_t event;
  AccelSensor.getEvent(&event);
  hValue = event.acceleration.y;

  // smooth data, the last 15 values are added and a median is found
  accTotal = accTotal - accReadings[accReadIndex];
  accReadings[accReadIndex] = hValue;
  accTotal = accTotal + accReadings[accReadIndex];
  accReadIndex = accReadIndex + 1;

  if (accReadIndex >= magNumReadings)
  {
    accReadIndex = 0;
  }

  accAverage = accTotal / magNumReadings;
  accAverage = accAverage - 3;

  // accAverage is up to -10 heeling left, is up to 6.3 heeling right
  // return data

  accAverage = abs(accAverage);
  accAverage = constrain(accAverage, 0, 5);

  return accAverage;
}

void navigate(int wantedHeading)
{
  if (wantedHeading >= 0 && wantedHeading <= 360) // if a compass heading is set
  {
    rudderServo.write(calcRudderAmp(wantedHeading)); // steer heading
  }
  else // steer course to the wind
  {
    int windAngle;

    switch (course) // set windAngle for course to the wind
    {
    case 1: //"closeHauled"
      windAngle = 50;
      break;
    case 2: //"beamReach"
      windAngle = 90;
      break;
    case 3: //"broadReach"
      windAngle = 135;
      break;
    default:
      break;
    }
    if (windDirection >= windAngle - 5 && windDirection <= windAngle + 5) // if windDirection is +-5 deg from desired windAngle (GO STRAIGHT)
    {
      rudderServo.write(calcRudderAmp(currentHeading));
    }
    else if (windDirection < windAngle - 5) // if windDirection is smaller than desired (BEAR AWAY)
    {
      if (windSide == 's')
      { //wind from starbord
        rudderServo.write(calcRudderAmp((windAngle - windDirection) - currentHeading));
      }
      else if (windSide == 'b')
      { //wind from port
        rudderServo.write(calcRudderAmp((windAngle - windDirection) + currentHeading));
      }
    }
    else if (windDirection > windAngle + 5) // if windDirection is greater than desired (LUFF UP)
    {
      if (windSide == 's')
      { //wind from starbord
        rudderServo.write(calcRudderAmp((windDirection - windAngle) + currentHeading));
      }
      else if (windSide == 'b')
      { //wind from port
        rudderServo.write(calcRudderAmp((windDirection - windAngle) - currentHeading));
      }
    }
  }
  sailServo.write(calcSailAngle());
}

int calcRudderAmp(int heading) // calc the rudder position according to desired compass heading
{
  float servoValue; // define calculation parameters
  int rudderThreshold;
  int rudderCenter;
  int currentHeel;

  rudderThreshold = 5;
  rudderCenter = 98; // servo position of rudder in center

  if (heading < 0) // normalize negative headings
  {
    heading = 360 + heading;
  }
  else if (heading > 360) // normalize headings above 360
  {
    heading = heading - 360;
  }

  // TODO: FIX function below: every thrirty seconds, it sets heading to 270 deg. why????

  // if (windDirection < 30) // make boat tack against the wind, when heading === wind direction
  // {
  //   timeStamp = millis(); // timestamp the last call of this section

  //   if (windSide == 's')
  //   { //wind from starbord
  //     heading = heading - (45 - windDirection);
  //   }
  //   else if (windSide == 'b')
  //   { //wind from port
  //     heading = heading + (45 - windDirection);
  //   }

  //   if (heading < 0)
  //   {
  //     heading = 360 + heading;
  //   }
  //   else if (heading > 360)
  //   {
  //     heading = heading - 360;
  //   }

  //   newHeading = heading;

  //   Serial.print("timeStamp");
  //   Serial.println(timeStamp);
  //   Serial.print("newHeading");
  //   Serial.println(newHeading);
  // }
  // else if (windDirection > 30 && timeStamp > 0) // if millis is defined
  // {
  //   heading = newHeading;
  //   Serial.print("wait for it! ");
  //   if ((millis() - timeStamp) > 30000) // every thirty seconds, switch direction
  //   {
  //     Serial.print("IF CHECKER ");
  //     Serial.println((millis() - timeStamp));

  //     Serial.print("MILLIS ");
  //     Serial.println((millis()));

  //     Serial.print("TIMESTAMP ");
  //     Serial.println((timeStamp));
  //     if (windSide == 's')
  //     { //
  //       newHeading = newHeading + 90;
  //     }

  //     if (windSide == 'b')
  //     { //
  //       newHeading = newHeading - 90;
  //     }

  //     if (newHeading < 0)
  //     {
  //       newHeading = 360 + newHeading;
  //     }
  //     else if (newHeading > 360)
  //     {
  //       newHeading = newHeading - 360;
  //     }
  //     Serial.print("newHeading ");
  //     Serial.println(newHeading);
  //     timeStamp = millis();
  //   }
  // }

  currentHeel = getHeelingAngle();

  Serial.print("currentHeel: ");
  Serial.println(currentHeel);

  if (currentHeading >= heading - rudderThreshold && currentHeading <= heading + rudderThreshold) // if currentHeading is +-rudderThreshold from desired heading (GO STRAIGHT)
  {
    if (windSide == 's')
    {                                          //wind from starbord
      servoValue = rudderCenter - currentHeel; // include heelingAngle to compensate force of heel (helps going straight);
    }
    else if (windSide == 'b')
    { //wind from port
      servoValue = rudderCenter + currentHeel;
    }
  }
  // TODO: implement checking for which direction to turn, to acieve short angle (e.g. turn from 345deg currentheading to 010deg desiredheading in a right curve, not a left one. )
  else if (currentHeading < heading - rudderThreshold) // if currentHeading is smaller than desired heading (TURN BOAT RIGHT)
  {
    servoValue = rudderCenter - abs(heading - currentHeading);
    servoValue = constrain(servoValue, 55, rudderCenter);
  }

  else if (currentHeading > heading + rudderThreshold) // if currentHeading is greater than desired heading (TURN BOAT LEFT)
  {
    servoValue = rudderCenter + abs(currentHeading - heading);
    servoValue = constrain(servoValue, rudderCenter, 130);
  }
  Serial.print("rudderAmp: ");
  Serial.println(servoValue);
  Serial.print("heading: ");
  Serial.println(heading);
  return servoValue;
}

int calcSailAngle() // calculate position of sailServo according to a logistic equotation.
// This equotation represents the correct positon of the sails according to the current windDirection
{
  float angle;
  float servoValue;
  float e;

  e = exp(0.45 * (windDirection / 10));
  angle = 105 / (1 + 0.01 * e);
  angle = constrain(angle, 0, 100);
  servoValue = ((1 - (angle / 100)) * 30) + 60;

  return servoValue;
}

int calcSailServo()
{
  // get input of receiver, calc data
  throttleSensData = (pulseIn(sailIn, HIGH)) - 1000;
  throttleData = constrain(throttleSensData / 10, 0, 1000) * 1.8;

  // smooth data, the last 15 values are added and a median is found
  sTotal = sTotal - sReadings[sReadIndex];
  sReadings[sReadIndex] = throttleData;
  sTotal = sTotal + sReadings[sReadIndex];
  sReadIndex = sReadIndex + 1;

  if (sReadIndex >= numReadings)
  {
    sReadIndex = 0;
  }

  sAverage = sTotal / numReadings;
  // return data to set servo, limit between 0 and 90 deg
  return constrain(sAverage, 60, 90);
}

int calcRudderServo()
{
  // get input of receiver, calc data
  rudderSensData = (pulseIn(rudderIn, HIGH)) - 1000;
  rudderData = constrain(rudderSensData / 10, 0, 1000) * 1.8;

  // smooth data, the last 15 values are added and a median is found
  rTotal = rTotal - rReadings[rReadIndex];
  rReadings[rReadIndex] = rudderData;
  rTotal = rTotal + rReadings[rReadIndex];
  rReadIndex = rReadIndex + 1;

  if (rReadIndex >= numReadings)
  {
    rReadIndex = 0;
  }

  rAverage = rTotal / numReadings;
  // return data to set servo, limit between 0 and 180 deg
  return constrain(rAverage, 0, 180);
}

int calcMagY(int data)
{
  // smooth MagY data, the last 15 values are added and a median is found
  magyTotal = magyTotal - magyReadings[magyReadIndex];
  magyReadings[magyReadIndex] = data;
  magyTotal = magyTotal + magyReadings[magyReadIndex];
  magyReadIndex = magyReadIndex + 1;

  if (magyReadIndex >= magNumReadings)
  {
    magyReadIndex = 0;
  }

  magyAverage = magyTotal / magNumReadings;
  return magyAverage;
}

int calcMagZ(int data)
{
  // smooth MagZ data, the last 15 values are added and a median is found
  magzTotal = magzTotal - magzReadings[magzReadIndex];
  magzReadings[magzReadIndex] = data;
  magzTotal = magzTotal + magzReadings[magzReadIndex];
  magzReadIndex = magzReadIndex + 1;

  if (magzReadIndex >= magNumReadings)
  {
    magzReadIndex = 0;
  }

  magzAverage = magzTotal / magNumReadings;
  return magzAverage;
}