//#define DEBUG
#ifdef DEBUG
#include <avr8-stub.h>
//#include <app_api.h> //only needed with flash breakpoints
#else
char printBuffer[128];
#endif

//#define L04// RGB LED
//#define L05// Buttons
//#define L06// Buzzer active
//#define L07// Buzzer passive
//#define L08// Tilt Ball Switch
//#define L09// SG90 (Servo)
//#define L10// SR04 (Ultrasonic Sensor)
//#define L11// Membrane Switch Module
//#define L12// DHT11 (Temperature and Humidity Sensor)
//#define L13// Analog Joystick Module
//#define L14// IR Receiver Module
//#define L15// MAX7219 LED Dot Matrix Module
//#define L16// MPU-6050 (GY-521 Module)
//#define L17// HC-SR501 (PIR Sensor)
//#define L18// Water Level Detection Sensor Module
//#define L19// DS1307 RTC (Real Time Clock Module)
//#define L20// KY-037 (Sound Sensor Module)
#define L21// RC522 RFID Module
//#define L22// LCD1602 (LCD Display)

#if defined(L06) || defined(L07) || defined(L09) || defined(L10) || defined(L12) || defined(L16) || defined(L17) || defined(L18) || defined(L19) || defined(L20) || defined(L21)
#define L05
#endif

/* PINS
-------------------------PWM-----------------------
0-2   : AJM
3-5   : MAX7219
6     : RC522 RST
7-11  : -----free to use
12    : LED
13    : LED_BUILTIN
-------------------------DGT-----------------------
14-21 : -----predefined
22-24 : RGB
25-26 : Buttons
27    : Buzzers
28    : TBS
29    : SG90 (Servo)
30-31 : SR04 (Ultrasonic)
32-39 : MSM (Keyboard)
40    : DHT11 (Temperature and Humidity)
41    : IR Reciver
42    : HC-SR501 (PIR)
43    : Sound Sensor Module (digital output)
44-49 : -----free to use
50-53 : RC522 (MOSI, MISO, SCK, SDA(SS))
-------------------------ANALOG--------------------
A0    : Water Level Detection Sensor Module
A1    : Sound Sensor Module (analog output)
A2-A13: -----free to use
*/

#ifdef L04 /* DEF L04 (RGB LED) */
#define rLedPin 22
#define gLedPin 23
#define bLedPin 24
#endif

#ifdef L05 /* DEF L05 (Buttons) */
#define LED_PIN 12
#define BTN1_PIN 25
#define BTN2_PIN 26
bool btn1Status;
bool btn2Status;
#endif

#if defined(L06) || defined(L07) // Buzzer Active, Passive
#include <anyrtttl.h>
#include <binrtttl.h>
//#include <pitches.h>
#define BZR_PIN 27
const char tetris[] PROGMEM = "tetris:d=4,o=5,b=160:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a,8p,d6,8f6,a6,8g6,8f6,e6,8e6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,a";
const char arkanoid[] PROGMEM = "Arkanoid:d=4,o=5,b=140:8g6,16p,16g.6,2a#6,32p,8a6,8g6,8f6,8a6,2g6";
const char mario[] PROGMEM = "mario:d=4,o=5,b=100:16e6,16e6,32p,8e6,16c6,8e6,8g6,8p,8g,8p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,16p,8c6,16p,8g,16p,8e,16p,8a,8b,16a#,8a,16g.,16e6,16g6,8a6,16f6,8g6,8e6,16c6,16d6,8b,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16c7,16p,16c7,16c7,p,16g6,16f#6,16f6,16d#6,16p,16e6,16p,16g#,16a,16c6,16p,16a,16c6,16d6,8p,16d#6,8p,16d6,8p,16c6";
// James Bond theme defined in inline code below (also stored in flash memory) 
byte songIndex = 0; //which song to play when the previous one finishes
#endif

#ifdef L07 /* DEF L07 (Buzzer Passive) */
#include "pitches.h"
int bzrPassMelody[] = {NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6};
#endif

#ifdef L08 /* DEF L08 (Tilt Ball Switch) */
#define TBS_PIN 28
#endif

#ifdef L09 /* DEF L09 (Servo SG90) */
#include "servo.h"
#define SRV_PIN 29
byte srvPos = 0;
Servo mySrv;
#endif

#ifdef L10 /* DEF L10 (Ultrasonic Sensor (SR04)) */
#include "HCSR04.h"
#define ECHO_PIN 30
#define TRIG_PIN 31
#endif

#ifdef L11 /* DEF L11 (Membrane Switch Module) */
#include <Keypad.h>

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
//Keypad pins:
//1-4 pins - columns (right to left)
//5-8 pins - rows up till down
byte colPins[COLS] = {35, 34, 33, 32};
byte rowPins[ROWS] = {39, 38, 37, 36};

Keypad myKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

char codeCorrect[5] = "";// = "0000";//{'0', '0', '0', '0', '\0'};
char codeUser[5] = {'1', '1', '1', '1', '\0'};
byte codeIndex = 0;
#endif

#ifdef L12 /* DEF L12 (DHT11 Temperature and Humidity Sensor) */
//#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTTYPE DHT11
#define DHTPIN 40
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
#endif

#ifdef L13 /* DEF L13 (Analog Joystick Module) */
#define AJM_X_PIN 0 //TX0
#define AJM_Y_PIN 1 //RX0
#define AJM_S_PIN 2 //digital pin connected to switch output
#endif

#ifdef L14 /* DEF L14 (IR Receiver Module) */
#include "IRremote.hpp"
//#include "PinDefinitionsAndMore.h"
#define IR_PIN 41
IRrecv irrecv(IR_PIN);
#endif

#ifdef L15 /* DEF L15 (MAX7219 LED Dot Matrix Module)*/
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#define MAX7219_DATA 3
#define MAX7219_CS   4
#define MAX7219_CLK  5
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, /* clk=*/ MAX7219_CLK, /* data=*/ MAX7219_DATA, /* cs=*/ MAX7219_CS, /* dc=*/ U8X8_PIN_NONE, /* reset=*/ U8X8_PIN_NONE);
String text = "Hello MAX7219";
#endif

#ifdef L16 /* DEF L16 (MPU-6050 (GY-521 Module))*/
#include "I2Cdev.h"
#include "MPU6050.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu6050;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

bool blinkState = false;
#endif

#ifdef L17 /* DEF L17 (HC-SR501 (PIR Sensor))*/
#define RIP_PIN 42
#endif

#ifdef L18 /* DEF L18 (Water Level Detection Sensor Module)*/
  #define ADC_PIN A0
  int HistoryValue = 0;  
#endif

#ifdef L19 /* DEF L19 (DS1307 RTC (Real Time Clock Module))*/
#include "RTClib.h"
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
#endif

#ifdef L20 /* DEF L20 (KY-037 (Sound Sensor Module))*/
#define SSM_DO_PIN 43 //sensor DO
#define SSM_AO_PIN A1 //sensor AO
#endif

#ifdef L21 /* DEF L21 (RC522 RFID Module)*/
#include <SPI.h>
#include <MFRC522.h>

#define RC522_RST_PIN 6
#define RC522_SS_PIN 53 //SDA(SS)
MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
#endif

#ifdef L22 /* DEF L22 LCD1602 (LCD Display)*/

#endif

void setupL04(void);
void setupL05(void);
void setupL06(void);
void setupL07(void);
void setupL08(void);
void setupL09(void);
void setupL10(void);
void setupL11(void);
void setupL12(void);
void setupL13(void);
void setupL14(void);
void setupL15(void);
void setupL16(void);
void setupL17(void);
void setupL18(void);
void setupL19(void);
void setupL20(void);
void setupL21(void);
void setupL22(void);

void loopL04(void);
void loopL05(void);
void loopL06(void);
void loopL07(void);
void loopL08(void);
void loopL09(void);
void loopL10(void);
void loopL11(void);
void loopL12(void);
void loopL13(void);
void loopL14(void);
void irrcvTranslate(uint32_t val);
void loopL15(void);
void loopL16(void);
void loopL17(void);
void loopL18(void);
void loopL19(void);
void loopL20(void);
void loopL21(void);
void loopL22(void);