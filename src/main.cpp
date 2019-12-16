// INCLUDE LIBRARIES
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <LSM303.h>

// DEFINE SENSORS
Adafruit_MLX90393 WindSensor = Adafruit_MLX90393();
Adafruit_LSM303_Mag_Unified MagSensor = Adafruit_LSM303_Mag_Unified(1);
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
  // INIT WINDSENSOR
  WindSensor.begin();

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
}

void loop()
{
  // check for receiver values and set current switch positions
  updateSwitches();

  if (switch_A == 0)
  { // AUTONOMOUS SECTION
    calcWindData();
    if (switch_D == 0)
    { // AM WIND
      if (windDirection > 30 && windDirection < 60)
      {
        rudderServo.write(98); // center rudder
        sailServo.write(50);   // TODO: CHECK SAIL SERVO VALUES!!!

        Serial.println("Am Wind SRAIGHT");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      }

      if (windDirection < 30)
      {
        sailServo.write(50); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(bearAway((30 - windDirection) / 30));

        Serial.println("Am Wind BEARING AWAY");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((40 - windDirection) / 40);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      }
      else if (windDirection > 63)
      {
        sailServo.write(53); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(luffUp((windDirection - 63) / 57));

        Serial.println("Am Wind LUFFING UP");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((windDirection - 75) / 105);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      };
    }
    else if (switch_D == 1)
    { // HALBWIND
      if (windDirection > 85 && windDirection < 100)
      {
        rudderServo.write(98);
        sailServo.write(60); // TODO: CHECK SAIL SERVO VALUES!!!

        Serial.println("Halb Wind Straight");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("----");
        Serial.println("----");
      }

      if (windDirection < 85)
      {
        sailServo.write(58); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(bearAway((85 - windDirection) / 85));

        Serial.println("Halb Wind BEAR AWAY");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((85 - windDirection) / 85);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      }
      else if (windDirection > 100)
      {
        sailServo.write(62); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(luffUp((windDirection - 100) / 80));

        Serial.println("Halb Wind LUFF UP");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((windDirection - 100) / 80);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      };
    }
    else if (switch_D == 2)
    { // RAUMWIND
      if (windDirection > 105 && windDirection < 165)
      {
        rudderServo.write(98);
        sailServo.write(70); // TODO: CHECK SAIL SERVO VALUES!!!

        Serial.println("Raum Wind Straight");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("----");
        Serial.println("----");
      }

      if (windDirection < 105)
      {
        sailServo.write(68); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(bearAway((105 - windDirection) / 105));

        Serial.println("Raum Wind BEAR AWAY");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((105 - windDirection) / 105);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      }
      else if (windDirection > 165)
      {
        sailServo.write(72); // TODO: CHECK SAIL SERVO VALUES!!!
        rudderServo.write(luffUp((windDirection - 165) / 15));

        Serial.println("Raum Wind LUFF UP");
        Serial.println("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println("Wind Direction Value: ");
        Serial.print((windDirection - 165) / 15);
        Serial.println("Wind Side: ");
        Serial.print(windSide);
        Serial.println("----");
        Serial.println("----");
      };
    }
  }
  else if (switch_A == 1)
  { // MANUAL MODE
    calcWindData();
    getCompassDir();
    getHeelingAngle();
    sailServo.write(calcSailServo());
    rudderServo.write(calcRudderServo());

    Serial.println("----");
    Serial.println("----");
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

int luffUp(float amount)
{
  // amount is between 0 and 1
  int servoData;
  //check which side the wind is coming from to determine which direction luffing up is
  if (windSide == 's')
  { //wind from starbord
    servoData = 98 - (43 * amount);
  }
  else if (windSide == 'b')
  { //wind from port
    servoData = 98 + (32 * amount);
  }
  // return number to set servo
  return servoData;
}

int bearAway(float amount)
{
  // amount is between 0 and 1
  int servoData;
  //check which side the wind is coming from to determine which direction bearing away is
  if (windSide == 's')
  { //wind from starbord
    servoData = 98 + (32 * amount);
  }
  else if (windSide == 'b')
  { //wind from port
    servoData = 98 - (43 * amount);
  }
  // return number to set servo
  return servoData;
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
  return constrain(sAverage, 0, 180);
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