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

// I/O DECLARATIONS
int sailIn = 9;
int sailOut = 7;
int rudderIn = 10;
int rudderOut = 8;
int switch_A_In = 11;
int switch_D_In = 12;

// SMOOTHING DECLARATIONS
const int numReadings = 11;

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
int bearAway(float amount);
int luffUp(float amount);
int calcMagY(int data);
int calcMagZ(int data);
void navigate(int wantedheading);
int calcRudderAmp(int heading);
int calcSailAngle();

void setup()
{
  // INIT SERIAL TO DEBUG
  Serial.begin(115200);
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
    }
  else if (switch_A == 1)
  { // MANUAL MODE
    sailServo.write(calcSailServo());
    rudderServo.write(calcRudderServo());
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
  windDirection = (1 - ((windDirection - 560) / 715)) * 180;
};

float getCompassDir()
{
  // get data from sensor
  compass.read();

  heading = compass.heading();

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
      windAngle = 60;
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

  currentHeel = getHeelingAngle();

  if (currentHeading >= heading - rudderThreshold && currentHeading <= heading + rudderThreshold) // if currentHeading is +-rudderThreshold from desired heading (GO STRAIGHT)
{
  if (windSide == 's')
    {                                              //wind from starbord
      servoValue = rudderCenter - currentHeel * 2; // include heelingAngle to compensate force of heel (helps going straight);
  }
  else if (windSide == 'b')
  { //wind from port
      servoValue = rudderCenter + currentHeel * 2;
    }
  }

  else if (currentHeading < heading - rudderThreshold) // if currentHeading is smaller than desired heading (TURN BOAT RIGHT)
  {
    servoValue = rudderCenter - abs(heading - currentHeading);
    servoValue = constrain(servoValue, 55, 98);
  }

  else if (currentHeading > heading + rudderThreshold) // if currentHeading is greater than desired heading (TURN BOAT LEFT)
  {
    servoValue = rudderCenter + abs(currentHeading - heading);
    servoValue = constrain(servoValue, 98, 130);
  }

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
  servoValue = ((1 - (angle / 100)) * 25) + 50;

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
  // return data to set servo, limit between 0 and 180 deg
  return constrain(sAverage, 0, 90);
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