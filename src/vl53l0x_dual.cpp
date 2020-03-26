#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include <PID_v1.h>

// #define DEBUG
#define MOCK

#ifdef DEBUG
// https://vmaker.tw/archives/13258
int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printRam() {
  Serial.print(F("RAM: "));
  Serial.println(freeRam());
}
#endif

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#if defined(__AVR__)
// set the pins to shutdown
#define SHT_LOX2 A3

#define PIN_MOTOR_X_ENABLE 4
#define PIN_MOTOR_X_DIRECTION 5

#define PIN_MOTOR_Y_ENABLE 6
#define PIN_MOTOR_Y_DIRECTION 7
#elif defined(ESP8266)
// set the pins to shutdown
#define SHT_LOX2 D3

#define PIN_MOTOR_X_ENABLE D8
#define PIN_MOTOR_X_DIRECTION D7

#define PIN_MOTOR_Y_ENABLE D6
#define PIN_MOTOR_Y_DIRECTION D5
#endif

uint16_t x = 100; //measure1.RangeMilliMeter;
uint16_t y = 100; //measure2.RangeMilliMeter;

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID()
{
  // all reset
  digitalWrite(SHT_LOX2, LOW);
  delay(100);
  // all unreset
  digitalWrite(SHT_LOX2, HIGH);
  delay(100);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(VL53L0X_BEST_ACCURACY_MODE, LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1) {
#if defined(ESP8266)
      ESP.wdtFeed();
#endif
    }
  }
#ifdef DEBUG
  Serial.println(F("Successful to boot first VL53L0X"));
#endif

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(100);

  //initing LOX2
  if (!lox2.begin(VL53L0X_BEST_ACCURACY_MODE, LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1) {
#if defined(ESP8266)
      ESP.wdtFeed();
#endif
    }
  }
#ifdef DEBUG
  Serial.println(F("Successful to boot second VL53L0X"));
#endif
}

#ifdef MOCK
uint16_t X_START = 80; // RIGHT
uint16_t X_END = 120;  // LEFT
uint16_t Y_START = 90; // TOP
uint16_t Y_END = 110;  // BOTTOM
#else
uint16_t X_START = 200; // RIGHT
uint16_t X_END = 500;   // LEFT
uint16_t Y_START = 100;  // TOP
uint16_t Y_END = 270;   // BOTTOM
#endif

double Setpoint, YSetpoint, Input, Output;
PID xPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
PID yPID(&Input, &Output, &YSetpoint, 2, 5, 1, DIRECT);
int WindowSize = 2000;
unsigned long windowStartTime;

void setup_PID()
{
  windowStartTime = millis();
  Setpoint = 100;

  // 告诉 PID 在从 0 到窗口大小的范围内取值
  xPID.SetOutputLimits(0, WindowSize);
  yPID.SetOutputLimits(0, WindowSize);
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
}

void read_dual_sensors()
{

  lox1.rangingTest(&measure1); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("1: ");
  if (measure1.RangeStatus != 4)
  { // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  }
  else
  {
    Serial.print(F("Out of range"));
  }

  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if (measure2.RangeStatus != 4)
  {
    Serial.print(measure2.RangeMilliMeter);
  }
  else
  {
    Serial.print("Out of range");
  }

  Serial.println();
}

void setup()
{
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (!Serial)
  {
    delay(10);
  }

  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));

  Serial.println(F("Starting..."));

#ifndef MOCK
  setID();
#endif

  setup_PID();

  pinMode(PIN_MOTOR_X_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_Y_ENABLE, OUTPUT);

  pinMode(PIN_MOTOR_X_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_Y_DIRECTION, OUTPUT);

  digitalWrite(PIN_MOTOR_X_ENABLE, HIGH);
  digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);
}

// -1
// 10 11
// 00 01
// bitRead(direction, 0); => toX 0 => toLeft 1 => toRight
// bitRead(direction, 1); => toY 0 => toBottom 1=> toTop
// state
//   bitWrite(direction, 0, !bitRead(direction, 0))
//   bitWrite(direction, 1, !bitRead(direction, 1))
int8_t direction = -1;

#define DIRECTION_LEFT LOW
#define DIRECTION_RIGHT HIGH
#define DIRECTION_TOP HIGH
#define DIRECTION_BOTTOM LOW

short getNearOrigin(uint16_t x, uint16_t y)
{
#ifdef DEBUG
  Serial.println(F("getNearOrigin"));
#endif
  byte pos = 0;
  if (x - X_START < X_END - x)
  {
    // near X_START
    pos = 2;
  }

  if (y - Y_START < Y_END - y)
  {
    // near Y_START
    pos += 1;
  }
  return pos;
}

void resetToOrigin(uint16_t x, uint16_t y)
{
  direction = getNearOrigin(x, y);
}

void moveX(uint16_t dir)
{
  xPID.Compute();
  if (millis() - windowStartTime > WindowSize)
  {
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime)
  {
#ifdef DEBUG
    Serial.print(F("DIRECTION:  "));
    Serial.print(dir ? F("RIGHT") : F("LEFT"));
#endif
#ifdef MOCK
    if (dir)
    {
      x--;
    }
    else
    {
      x++;
    }
#endif
    digitalWrite(PIN_MOTOR_X_DIRECTION, dir);
    delay(100);
#ifdef DEBUG
    Serial.println(F(" ENABLE"));
#endif
    digitalWrite(PIN_MOTOR_X_ENABLE, LOW);
  }
  else
  {
#ifdef DEBUG
    Serial.println(F("DISABLE"));
#endif
    digitalWrite(PIN_MOTOR_X_ENABLE, HIGH);
  }
}

void moveY(uint16_t dir)
{
  yPID.Compute();
  if (millis() - windowStartTime > WindowSize)
  {
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime)
  {
#ifdef DEBUG
    Serial.print(F("DIRECTION:  "));
    Serial.print(dir ? F("TOP") : F("BOTTOM"));
#endif
#ifdef MOCK
    if (dir)
    {
      y++;
    }
    else
    {
      y--;
    }
#endif
    digitalWrite(PIN_MOTOR_Y_DIRECTION, dir);
    delay(100);
#ifdef DEBUG
    Serial.println(F(" ENABLE"));
#endif
    digitalWrite(PIN_MOTOR_Y_ENABLE, LOW);
  }
  else
  {
  #ifdef DEBUG
    Serial.println(F("DISABLE"));
  #endif
    digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);
  }
}

// to0
void toLeft()
{
#ifdef DEBUG
  Serial.println(F("toLeft"));
#endif
  Setpoint = Input;
  Input = X_START;
  moveX(DIRECTION_LEFT);
}

void toRight()
{
#ifdef DEBUG
  Serial.println(F("toRight"));
#endif
  Setpoint = X_END;
  moveX(DIRECTION_RIGHT);
}

bool y_moving = false;

void toTop()
{
  if (YSetpoint >= Input)
  {
#ifdef DEBUG
    Serial.println(F("toTop"));
#endif
    moveY(DIRECTION_TOP);
  }
  else
  {
    digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);
  }
}

// to0
void toBottom()
{
  if (YSetpoint <= Input)
  {
#ifdef DEBUG
    Serial.println(F("toBottom"));
#endif
    moveY(DIRECTION_BOTTOM);
  }
  else
  {
    digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);
  }
}

void boundaryDetection(uint16_t x, uint16_t y)
{
#ifdef DEBUG
  Serial.println(F("boundaryDetection"));
  Serial.print(F("direction: "));
  Serial.print(direction);
#endif
  if (x < X_START)
  {
    bitWrite(direction, 0, 1);
  }

  if (x > X_END)
  {
    bitWrite(direction, 0, 0);
  }
  if (y < Y_START)
  {
    bitWrite(direction, 1, 1);
  }

  if (y > Y_END)
  {
    bitWrite(direction, 1, 0);
  }
#ifdef DEBUG
  Serial.print(F(" to: "));
  Serial.println(direction);
#endif
}

void loop()
{
  // digitalWrite(PIN_MOTOR_X_ENABLE, LOW);
  // digitalWrite(PIN_MOTOR_Y_ENABLE, LOW);

#ifndef MOCK
  read_dual_sensors();
  uint16_t x = measure1.RangeMilliMeter;
  uint16_t y = measure2.RangeMilliMeter;
#else
  Serial.print(x);
  Serial.print(',');
  Serial.println(y);
#endif
#ifdef DEBUG
  Serial.print(F("X:"));
  Serial.print(x);
  Serial.print(F(" Y:"));
  Serial.println(y);
#endif

  if (~direction)
  {
    byte _direction = direction;
    boundaryDetection(x, y);
    Input = x;
    if (bitRead(direction, 0))
    {
      toLeft();
    }
    else
    {
      toRight();
    }

    Input = y;
    // 当 X 发生换向，Y 需要更新 Y 目标
    if (bitRead(_direction, 0) != bitRead(direction, 0))
    {
#ifdef DEBUG
      Serial.print(F("Y need move"));
#endif
      if (bitRead(direction, 1))
      {
        YSetpoint = Input + 4;
      }
      else
      {
        YSetpoint = Input - 4;
      }
#ifdef DEBUG
      Serial.print(F("YSetpoint: "));
      Serial.println(YSetpoint);
#endif
    }

    if (bitRead(direction, 1))
    {
      toTop();
    }
    else
    {
      toBottom();
    }
  }
  else
  {
    Serial.println(F("ResetToOrigin"));
    resetToOrigin(x, y);
  }

  delay(10);
}
