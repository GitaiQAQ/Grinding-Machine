#include "Arduino.h"
#include <EEPROM.h>

bool status = true;
int address = 0;

// #define DEBUG
// #define MOCK

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

#define PIN_LED_DIN 11
#define PIN_LED_CS 12
#define PIN_LED_CLK 13

#elif defined(ESP8266)
// set the pins to shutdown
#define SHT_LOX2 D3

#define PIN_MOTOR_X_ENABLE D8
#define PIN_MOTOR_X_DIRECTION D7

#define PIN_MOTOR_Y_ENABLE D6
#define PIN_MOTOR_Y_DIRECTION D5
#endif

// uint16_t x = 200; //measure1.RangeMilliMeter;
// uint16_t y = 200; //measure2.RangeMilliMeter;
uint16_t x = 310; //measure1.RangeMilliMeter;
uint16_t y = 72; //measure2.RangeMilliMeter;

#include "LedControl.h"

LedControl lc = LedControl(PIN_LED_DIN, PIN_LED_CLK, PIN_LED_CS, 1);

void setup_led() {
    lc.shutdown(0, false);
    lc.setIntensity(0, 5);
    lc.clearDisplay(0);
    lc.printDigit(0, PI);
}

void updateXY() {
    lc.printDigit(0, y);
    lc.printDigit(0, x, 4);
}

#define STATUS_BOOTLOAD 1000
#define STATUS_SETUP_VL53L0X_FIRST 1001
#define STATUS_SETUP_VL53L0X_SECOND 1002
#define STATUS_SETUP_OK 1003
#define STATUS_SUCCESSFUL_TO_BOOT_FIRST_VL53L0X 1101
#define STATUS_SUCCESSFUL_TO_BOOT_SECOND_VL53L0X 1102
#define STATUS_RESET_TO_ORIGIN 1201

#define STATUS_FIRST_VL53L0X_TIME_OUT 1301
#define STATUS_FIRST_VL53L0X_OUT_OF_RANGE 1302
#define STATUS_FIRST_VL53L0X_ERR_CODE_BASE 1400

#define STATUS_FIRST_VL53L0X_ERROR_CALIBRATION_WARNING (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_CALIBRATION_WARNING)
#define STATUS_FIRST_VL53L0X_ERROR_UNDEFINED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_UNDEFINED)
#define STATUS_FIRST_VL53L0X_ERROR_INVALID_PARAMS (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_INVALID_PARAMS)
#define STATUS_FIRST_VL53L0X_ERROR_NOT_SUPPORTED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_NOT_SUPPORTED)
#define STATUS_FIRST_VL53L0X_ERROR_RANGE_ERROR (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_RANGE_ERROR)
#define STATUS_FIRST_VL53L0X_ERROR_TIME_OUT (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_TIME_OUT)
#define STATUS_FIRST_VL53L0X_ERROR_MODE_NOT_SUPPORTED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_MODE_NOT_SUPPORTED)
#define STATUS_FIRST_VL53L0X_ERROR_BUFFER_TOO_SMALL (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_BUFFER_TOO_SMALL)
#define STATUS_FIRST_VL53L0X_ERROR_GPIO_NOT_EXISTING (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_GPIO_NOT_EXISTING)
#define STATUS_FIRST_VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED)
#define STATUS_FIRST_VL53L0X_ERROR_INTERRUPT_NOT_CLEARED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_INTERRUPT_NOT_CLEARED)
#define STATUS_FIRST_VL53L0X_ERROR_CONTROL_INTERFACE (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_CONTROL_INTERFACE)
#define STATUS_FIRST_VL53L0X_ERROR_INVALID_COMMAND (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_INVALID_COMMAND)
#define STATUS_FIRST_VL53L0X_ERROR_DIVISION_BY_ZERO (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_DIVISION_BY_ZERO)
#define STATUS_FIRST_VL53L0X_ERROR_REF_SPAD_INIT (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_REF_SPAD_INIT)
#define STATUS_FIRST_VL53L0X_ERROR_NOT_IMPLEMENTED (STATUS_FIRST_VL53L0X_ERR_CODE_BASE + VL53L0X_ERROR_NOT_IMPLEMENTED)

#define STATUS_FIRST_VL53L0X_RANGESTATUS_RANGEVALID (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 0)
// 范围内有效测距测量有效
#define STATUS_FIRST_VL53L0X_RANGESTATUS_SIGMA (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 1)
// 环境光，环境光太多噪声
#define STATUS_FIRST_VL53L0X_RANGESTATUS_SIGNAL (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 2)
// 限制或RIT（范围忽略阈值）
#define STATUS_FIRST_VL53L0X_RANGESTATUS_MINRANGE (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 3)
// 最小范围失败默认情况下未启用
#define STATUS_FIRST_VL53L0X_RANGESTATUS_PHASE (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 4)
// 噪声过高
#define STATUS_FIRST_VL53L0X_RANGESTATUS_HW (STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + 5)
// 硬件故障硬件故障

#define STATUS_SECOND_VL53L0X_TIME_OUT 1401
#define STATUS_SECOND_VL53L0X_OUT_OF_RANGE 1402
#define STATUS_SECOND_VL53L0X_ERR_CODE_BASE 1500

#define ERR_FAILED_TO_BOOT_FIRST_VL53L0X 2301
#define ERR_FAILED_TO_BOOT_SECOND_VL53L0X 2302

void printStatus(uint16_t id) {
    lc.printDigit(0, id);
    lc.printDigit(0, 5, 7);
}

void printError(uint16_t id) {
    lc.printDigit(0, id);
    lc.printBuffer(0, "E", 7);
    while (1) {
#if defined(ESP8266)
      ESP.wdtFeed();
#endif
    }
}

#ifndef MOCK
#include "Adafruit_VL53L0X.h"

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

  printStatus(STATUS_SETUP_VL53L0X_FIRST);
  // initing LOX1
  if (!lox1.begin(VL53L0X_GOOD_ACCURACY_MODE, LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    printError(ERR_FAILED_TO_BOOT_FIRST_VL53L0X);
  }
  printStatus(STATUS_SUCCESSFUL_TO_BOOT_FIRST_VL53L0X);
#ifdef DEBUG
  Serial.println(F("Successful to boot first VL53L0X"));
#endif

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(100);

  printStatus(STATUS_SETUP_VL53L0X_SECOND);
  //initing LOX2
  if (!lox2.begin(VL53L0X_BEST_ACCURACY_MODE, LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    printError(ERR_FAILED_TO_BOOT_SECOND_VL53L0X);
  }
  printStatus(STATUS_SUCCESSFUL_TO_BOOT_SECOND_VL53L0X);
#ifdef DEBUG
  Serial.println(F("Successful to boot second VL53L0X"));
#endif
}

void read_dual_sensors()
{
  VL53L0X_Error err = lox1.rangingTest(&measure1); // pass in 'true' to get debug data printout!
  if (err != VL53L0X_ERROR_NONE) {
    // VL53L0X_define_Error_group
    printStatus(STATUS_FIRST_VL53L0X_ERR_CODE_BASE + err);
    delay(500);
  }
  if (measure1.RangeStatus != 0)
  {
    // VL53L0X_get_range_status_string
    printStatus(STATUS_FIRST_VL53L0X_ERR_CODE_BASE - 50 + measure1.RangeStatus);
    delay(500);
  }

  err = lox2.rangingTest(&measure2); // pass in 'true' to get debug data printout!
  if (err != VL53L0X_ERROR_NONE) {
    // VL53L0X_define_Error_group
    printStatus(STATUS_SECOND_VL53L0X_ERR_CODE_BASE + err);
    delay(500);
  }
  if (measure2.RangeStatus != 0)
  {
    // VL53L0X_get_range_status_string
    printStatus(STATUS_SECOND_VL53L0X_ERR_CODE_BASE - 50 + measure1.RangeStatus);
    delay(500);
  }
}
#endif

#ifdef MOCK
uint16_t X_START = 180; // RIGHT
uint16_t X_END = 220;  // LEFT
uint16_t Y_START = 190; // TOP
uint16_t Y_END = 210;  // BOTTOM
// uint16_t X_START = 200; // RIGHT
// uint16_t X_END = 500;   // LEFT
// uint16_t Y_START = 50;  // TOP
// uint16_t Y_END = 270;   // BOTTOM

bool X_DIRECTION = false;
bool Y_DIRECTION = false;

void _digitalWrite(uint8_t pin, uint8_t val) {
  switch (pin)
  {
  case PIN_MOTOR_X_ENABLE:
    val || (X_DIRECTION ? x++: x--);
    break;
  case PIN_MOTOR_Y_ENABLE:
    val || (Y_DIRECTION ? y++: y--);
    break;
  case PIN_MOTOR_X_DIRECTION:
    X_DIRECTION = val;
    break;
  case PIN_MOTOR_Y_DIRECTION:
    Y_DIRECTION = val;
    break;
  }
  digitalWrite(pin, val);
}
#else
uint16_t X_START = 200; // RIGHT
uint16_t X_END = 500;   // LEFT
uint16_t Y_START = 100;  // TOP
uint16_t Y_END = 200;   // BOTTOM

void (*_digitalWrite)(uint8_t pin, uint8_t val) = digitalWrite;
#endif

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

void onPause()
{
    status = !status;
}

void setup()
{
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
#ifdef DEBUG
  while (!Serial)
  {
    delay(10);
  }
#endif

  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));

  Serial.println(F("Starting..."));

  setup_led();

  printStatus(STATUS_BOOTLOAD);
#ifndef MOCK
  setID();
#endif

  pinMode(PIN_MOTOR_X_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_Y_ENABLE, OUTPUT);

  pinMode(PIN_MOTOR_X_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_Y_DIRECTION, OUTPUT);

  _digitalWrite(PIN_MOTOR_X_ENABLE, HIGH);
  _digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);

  direction = EEPROM.read(address);

  printStatus(STATUS_SETUP_OK);

  pinMode(A2, INPUT);
  attachInterrupt(digitalPinToInterrupt(A2), onPause, RISING);
}

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

uint16_t YSetpoint = 0;

void loop()
{
  if (!status) {
    _digitalWrite(PIN_MOTOR_X_ENABLE, HIGH);
    _digitalWrite(PIN_MOTOR_Y_ENABLE, HIGH);
    return;
  }

#ifndef MOCK
  read_dual_sensors();
  x = measure1.RangeMilliMeter;
  y = measure2.RangeMilliMeter;
#endif
Serial.print(direction);
Serial.print(',');
Serial.print(x);
Serial.print(',');
Serial.print(y);
updateXY();
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

    if (bitRead(direction, 0) != bitRead(_direction, 0)) {
      EEPROM.write(address, direction);
      _digitalWrite(PIN_MOTOR_X_ENABLE, HIGH);
      delay(1000);
    }
    _digitalWrite(PIN_MOTOR_X_DIRECTION, bitRead(direction, 0));

    // 当 X 发生换向，Y 需要更新 Y 目标
    if (bitRead(_direction, 0) != bitRead(direction, 0))
    {
#ifdef DEBUG
      Serial.print(F("Y need move"));
#endif
      if (bitRead(direction, 1))
      {
        YSetpoint = y + 4;
      }
      else
      {
        YSetpoint = y - 4;
      }
#ifdef DEBUG
      Serial.print(F("YSetpoint: "));
      Serial.println(YSetpoint);
#endif
    }

    _digitalWrite(PIN_MOTOR_Y_DIRECTION, bitRead(direction, 1));
    if (YSetpoint) {
      bool enable = (bitRead(direction, 1) ? y > YSetpoint : y < YSetpoint);
      _digitalWrite(PIN_MOTOR_Y_ENABLE, enable);
      Serial.print(',');
      Serial.print(YSetpoint);
      Serial.print(',');
      Serial.print(enable);
      if (enable) {
        YSetpoint = 0;
      }
    } else {
      _digitalWrite(PIN_MOTOR_X_ENABLE, LOW);
    }
  }
  else
  {
  #ifdef DEBUG
    Serial.println(F("ResetToOrigin"));
  #endif
    printStatus(STATUS_RESET_TO_ORIGIN);
    resetToOrigin(x, y);
  }

  Serial.println("");
  delay(100);
}
