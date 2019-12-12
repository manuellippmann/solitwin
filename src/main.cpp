#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Adafruit_MLX90393 WindSensor = Adafruit_MLX90393();
Adafruit_LSM303_Mag_Unified MagSensor = Adafruit_LSM303_Mag_Unified();

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
float xData, yData, zData, windDirection;
char windSide;

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

int magxReadings[magNumReadings]; // magnetometer x
int magxReadIndex = 0;
float magxTotal = 0;
float magxAverage = 0;

int magyReadings[magNumReadings]; // magnetometer y
int magyReadIndex = 0;
float magyTotal = 0;
float magyAverage = 0;

int magzReadings[magNumReadings]; // magnetometer z
int magzReadIndex = 0;
float magzTotal = 0;
float magzAverage = 0;

// FUCTION DECLARATIONS

int calcRudderServo();
int calcSailServo();
void updateSwitches();
void calcWindData();
void getCompassDir();
int bearAway(float amount);
int luffUp(float amount);
int calcMagY(int data);
int calcMagZ(int data);

void setup()
{
  Serial.begin(9600);
  pinMode(sailIn, INPUT);
  pinMode(rudderIn, INPUT);
  pinMode(switch_A_In, INPUT);
  pinMode(switch_D_In, INPUT);
  sailServo.attach(sailOut);
  rudderServo.attach(rudderOut);
  WindSensor.begin();

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    sReadings[thisReading] = 0;
    rReadings[thisReading] = 0;
  }
}

void loop()
{
  updateSwitches();

  if (switch_A == 0)
  { // autonomous section
    calcWindData();
    if (switch_D == 0)
    { // AM WIND
      if (windDirection > 30 && windDirection < 60)
      {
        rudderServo.write(98); // center
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
  { // manual section

    //getCompassDir();

    sailServo.write(calcSailServo());
    rudderServo.write(calcRudderServo());
    calcWindData();
    Serial.print("windDirection: ");
    Serial.println(windDirection);
    Serial.print("windSide: ");
    Serial.println(windSide);
    Serial.print("RudderServo: ");
    Serial.println(calcRudderServo());

    Serial.print("SailServo: ");
    Serial.println(calcSailServo());

    Serial.println("----");
    Serial.println("----");
  }
};

void updateSwitches()
{
  switch_A_data = (pulseIn(switch_A_In, HIGH)) - 1000;
  if (switch_A_data > 500)
  {
    switch_A = 0;
  }
  else if (switch_A_data < 150)
  {
    switch_A = 1;
  }

  switch_D_data = (pulseIn(switch_D_In, HIGH)) - 1000;
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
  WindSensor.readData(&xData, &yData, &zData);

  if (calcMagY(yData) < 0)
  {
    windSide = 's';
  }
  else
  {
    windSide = 'b';
  }

  windDirection = calcMagZ(zData);
  windDirection = (1 - ((windDirection - 560) / 715)) * 180;
};

void getCompassDir()
{
  sensors_event_t event;
  MagSensor.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
}

int luffUp(float amount)
{
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
  return servoData;
}

int bearAway(float amount)
{
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
  return servoData;
}

int calcSailServo()
{
  throttleSensData = (pulseIn(sailIn, HIGH)) - 1000;
  throttleData = constrain(throttleSensData / 10, 0, 1000) * 1.8;

  sTotal = sTotal - sReadings[sReadIndex];
  sReadings[sReadIndex] = throttleData;
  sTotal = sTotal + sReadings[sReadIndex];
  sReadIndex = sReadIndex + 1;

  if (sReadIndex >= numReadings)
  {
    sReadIndex = 0;
  }

  sAverage = sTotal / numReadings;
  return constrain(sAverage, 0, 180);
}

int calcRudderServo()
{
  rudderSensData = (pulseIn(rudderIn, HIGH)) - 1000;
  rudderData = constrain(rudderSensData / 10, 0, 1000) * 1.8;

  rTotal = rTotal - rReadings[rReadIndex];
  rReadings[rReadIndex] = rudderData;
  rTotal = rTotal + rReadings[rReadIndex];
  rReadIndex = rReadIndex + 1;

  if (rReadIndex >= numReadings)
  {
    rReadIndex = 0;
  }

  rAverage = rTotal / numReadings;
  return constrain(rAverage, 0, 180);
}

int calcMagY(int data)
{

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