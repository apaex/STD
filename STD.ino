


#if defined(__AVR__)
#include <avr/pgmspace.h>
#include <util/delay.h>
#else
// Для совместимости с Arduino Due (пока не работает)
#define PROGMEM
#define PGM_P  const char *
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned char **)(addr))
#define strcpy_P(dest, src) strcpy((dest), (src))
#endif
#include <SimpleTimer.h>
//#include <ITDB02_Graph16.h>
#include <UTFT.h>
#include <UTouch.h>
#include <Wire.h>            // I2C library
#include <EEPROM.h>
#include "writeAnything.h"
//#include "pgmspace_big.h"
#include <DS1307new.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SdFat.h>
//#include <SdFatUtil.h>

#include "aquarium.h"
using namespace aquarium;
#define dataSerial Serial1

SimpleTimer timemillis;      // There must be one global SimpleTimer object.

//Default Controller Settings
boolean RECOM_RCD = true;               // For Mean Well drivers change "true" to "false"
//boolean RECOM_RCD = false;

//**********************ВЫБОР ТИПА ШИЛДА ********************************************************

#define Standard_shield         // Раскоментировать при использовании стандартоного шилда , 

//#define Aqua_shield_v2        // Раскоментировать при использовании АКВАшилда V2 ,
 
//#define Aqua_shield_v3        // Раскоментировать при использовании АКВАшилда V3 ,

#define PH_sensor_I2C         // Раскоментировать при наличии датчика РН на шине I2C
#define PH_sensor_ADC A0      // Раскоментировать при наличии датчика РН с аналоговым модулем (калибровка на модуле)

//**********************ВЫБОР МОРЕ - ПРЕСНЯК ****************************************************
#define freshwater              // Раскоментировать для пресняка

//#define seawater              // Раскоментировать для моря

//******************** Выбор частот ШИМ (PWM_FRQ_Value) *********************
//------------------ для пинов 7,8,9, 11, 12 ------------------
     byte PWM_FRQ_Value = 1;	     // PWM Frequency = 31.374 KHz                     
   //byte PWM_FRQ_Value = 2;	     // PWM Frequency = 3906 Hz 
   //byte PWM_FRQ_Value = 3;         // PWM Frequency = 488 Hz   
   //byte PWM_FRQ_Value = 4;	     // PWM Frequency = 122 Hz
   //byte PWM_FRQ_Value = 5;	     // PWM Frequency = 30.63 Hz
//-------------------- для пина 10 (таймер 2)-------------------
   //byte PWM_FRQ_ValueT2 = 1;	     // PWM Frequency = 31.374 KHz
   //byte PWM_FRQ_ValueT2 = 2;	     // PWM Frequency = 3.921 KHz
   //byte PWM_FRQ_ValueT2 = 3;	     // PWM Frequency = 980.3 Hz 
   //byte PWM_FRQ_ValueT2 = 4;	     // PWM Frequency = 490.1 Hz  				           
   //byte PWM_FRQ_ValueT2 = 5;	     // PWM Frequency = 245 hz  
   //byte PWM_FRQ_ValueT2 = 6;	     // PWM Frequency = 122 Hz 
   //byte PWM_FRQ_ValueT2 = 7;	     // PWM Frequency = 30.63 hz

//--------------- для пинов 44, 45, 46 (вентилятор) --------------
   //byte PWM_FRQ_Value_Fan = 1;        // PWM Frequency = 31.374 KHz
   //byte PWM_FRQ_Value_Fan = 2;        // PWM Frequency = 3.921 Khz
   //byte PWM_FRQ_Value_Fan = 3;        // PWM Frequency = 490.1 Hz 
   //byte PWM_FRQ_Value_Fan = 4;        // PWM Frequency = 122 Hz for Fans
     byte PWM_FRQ_Value_Fan = 5;        // PWM Frequency = 30 Hz for Fans
   
   

//********************** Цвета каналов ********************************************
const byte rgbCh0[] = {255, 255, 205};   //  Теплый белый		|
const byte rgbCh1[] = {255, 255, 255};   //  Холодный белый      	|
const byte rgbCh2[] = {58, 95, 205};     //  Синий	                |	
const byte rgbCh3[] = {255, 0, 0};       //  Красный	                |	
const byte rgbCh4[] = {224, 102, 255};   //  Фиолетовый			|
const byte rgbCh5[] = {255, 143, 32};    //  Оранжевый		        |
const byte rgbCh6[] = {0, 255, 0};       //  Зеленый			|
//const byte rgbCh7[] = {0, 0, 0};       //   		                |
const byte rgbCh8[] = {236, 214, 114};   //  Moon			|
//************************************************************************************************
   

//(Mega Shield utilizes pins 5V, 3V3, GND, 2-6, 20-41, & (50-53 for SD Card)) 
//ITDB02 myGLCD(38,39,40,41,ITDB32S);   // Uncomment this line for the SSD1289
UTFT myGLCD(ITDB32S, 38,39,40,41);
//UTFT myGLCD(HX8352A, 38,39,40,41);
#ifdef Standard_shield
UTouch myTouch(6,5,4,3,2);        // для стандартного шилда
#else
UTouch myTouch(42,49,47,48,43);   // для Аквашилда v3 и v2
#endif

byte xdate;                          // переменная даты

int rtcSetMin, rtcSetHr, rtcSetDy, rtcSetMon, rtcSetYr, rtcSetSec, rtcSetDw;

// аналоговые часы
int clockCenterX=159;       // координана положения по горизонтали
int clockCenterY=119;  
int oldsec=0;

int displayDOW = 0;                  // Hide=0 || Show=1 (change in prog)
int yTime;                           // Setting clock stuff

int timeDispH, timeDispM, xTimeH, xTimeM10, xTimeM1, xColon; 
String time, day; 

int setClockOrBlank = 0;             // Clock Screensaver=0 || Blank Screen=1 (change in prog)
int setScreensaverOnOff = 0;         // OFF=0 || ON=1 Turns it ON/OFF (change in prog)
int setScreensaverDOWonOff = 0;      // OFF=0 || ON=1 Shows/Hides DOW in Screensaver (change in prog) 

int digital = 0;                     // цифровые часы
int analog = 1;                      // аналоговые часы

int SS_DOW_x;                        // Moves the DOW to correct position
int setSSmintues;                    // Time in (minutes) before Screensaver comes on (change in program)
int TempSSminutes;                   // Temporary SetSSminutes used in calcs and prints, etc.
int setScreenSaverTimer;             // how long in (minutes) before Screensaver comes on (change in program)
int screenSaverCounter = 0;          // counter for Screen Saver
boolean SCREEN_RETURN = true;        // Auto Return to mainScreen() after so long of inactivity
int returnTimer = 0;                 // counter for Screen Return
int setReturnTimer;                  // Return to main screen 75% of time before the screensaver turns on


int setScreensaverTupe = 0;

// декларируем шрифты
extern uint8_t SmallFont[];           // маленький шрифт
extern uint8_t BigFont[];             // большой шрифт
extern uint8_t DotMatrix_M_Num[];     // большой матричный шрифт (только цифры)
extern uint8_t SevenSegNumFont[]; // большой шрифт
extern uint8_t RusFont1[];            // маленькая кириллица
extern uint8_t RusFont2[];            // большая кириллица
extern uint8_t RusFont3[];            // средняя кириллица
extern uint8_t RusFont6[];            // средняя кириллица (Small Font)

float linhaR;
float linhaG;
float linhaB;

// true - включено по умолчанию, false - выключено
#define LARGE true     // большой шрифт
#define SMALL false    // маленький шрифт (по умолчанию)

#define BlUE_BAC false // голубой (по умолчанию)
#define GREEN_BAC true // зеленый 


//************************* Define for 11bit timer *********************************
#ifndef cbi_mix
#define cbi_mix(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi_mix
#define sbi_mix(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Занятые контакты (50-53 for SD Card)  myTouch(46,45,44,43,42)
// Разрешение 11 бит, может работать на контактах (2, 3, 5, 6, 7, 8, 11, 12, 44, 45, 46)

//************************* Назначение пинов  *********************************

// ------------- ДЛЯ СТАНДАРТНОГО ШИЛДА MEGA2560 ------------Выводы 6,5,4,3,2 заняты под тач
// ------------------------можно использовать выводы 42,49,47,48,43 + 44,45,46 с ШИМ (паять) 
#ifdef Standard_shield

uint8_t ledPinWhite =   12;   // WWT  Теплый белый     
uint8_t ledPinBlue =     7;   // CWT  Холодный белый  
uint8_t ledPinRoyBlue = 11;   // RBL  Глубой 
uint8_t ledPinRed =      9;   // RED  Красный     
uint8_t ledPinUV =      A1;   // UVL  Фиолетовый    
uint8_t ledPinOrange =  A2;   // ORG  Оранжевый  
uint8_t ledPinGr =       8;   // GRN  Зеленый  
uint8_t ledPinMoon =    13;   // Moon Луна

uint8_t LCDbrightPin = 45;   // подсветка LCD  

uint8_t SensLight = A13;  // датчик освещения, на аналоговом пине 
uint8_t SensLevel = A14;  // датчик уровня, на аналоговом пине 

// Define the other DIGITAL and/or PWM PINS being used
uint8_t tempHeatPin       = 47;   // 47  Нагреватель вкл./выкл. 
uint8_t tempChillPin      = 48;   // 48  Холодильник вкл./выкл. 
uint8_t tempAlarmPin      = A3;   // A3  Buzzer Alarm 
uint8_t autoFeeder        = A4;   // Пин кормушки
uint8_t Heatsink1_FansPWM = 44;   // 44 Fan-PWM0 Heatsink1 Fan, Вентилятор на радиаторе 1 
uint8_t Heatsink2_FansPWM = 46;   // 46 Fan-PWM1 Heatsink2 Fan, Вентилятор на радиаторе 2 
  
// Таймеры
uint8_t timer1 = A5;    // pin analog 
uint8_t timer2 = A6;    // pin analog 
uint8_t timer3 = A7;    // pin analog 
uint8_t timer4 = A8;    // pin analog 
uint8_t timer5 = A9;    // pin analog 

uint8_t pump1 = A10;    // pin analog 
uint8_t pump2 = A11;    // pin analog 
uint8_t pump3 = A12;    // pin analog 
uint8_t pump4 = A13;    // pin analog 

uint8_t vacpump = 3;    // pin 3


// DS18B20 Temperature sensors plugged into pin 15 (Water, Hood) 
OneWire OneWireBus(A15);      // Датчик температуры  (Dallas pin)
uint8_t SDchipSelect = 53; // SD card attached to SPI bus
# endif


// ------------- FOR AQUA SHIELD V3 Распиновка для Аквашилда Олега-------------------------------


#ifdef Aqua_shield_v2

// ВХОДЫ
uint8_t SensLight   = A13;   // датчик освещения, на аналоговом пине 
uint8_t SensLevel    = A12;   // датчик уровня воды, на аналоговом пине 
OneWire OneWireBus(A15);     // Датчик температуры  (Dallas pin)
 
// ВЫХОДЫ
uint8_t ledPinWhite   =12;   // WWT  Теплый белый    (2) - 11 бит   
uint8_t ledPinBlue    = 5;   // CWT  Холодный белый  (3) - 11 бит
uint8_t ledPinRoyBlue =13;   // RBL  Глубой          (5) - 11 бит
uint8_t ledPinRed     = 6;   // RED  Красный         (6) - 11 бит
uint8_t ledPinUV      = 4;   // UVL  Фиолетовый      (7) - 11 бит
uint8_t ledPinOrange = 11;   // ORG  Оранжевый       (8) - 11 бит
uint8_t ledPinGr     = 10;   // GRN  Зеленый        (11) - 11 бит

uint8_t ledPinMoon    = 3;   // Moon Led pin     (4) - ( луна 8 бит - 255 )

uint8_t LCDbrightPin = 45;   // подсветка LCD   (13) - (8 бит)

uint8_t tempHeatPin       = A9;    // A5   Нагреватель вкл./выкл. 
uint8_t tempChillPin      =A11;    // 46   Холодильник вкл./выкл. 
uint8_t tempAlarmPin      =  7;    // A11  Buzzer Alarm 
uint8_t autoFeeder        = A2;    // А7   Пин кормушки
uint8_t Heatsink1_FansPWM = 44;    // 44   Вентилятор на радиаторе 1 
uint8_t Heatsink2_FansPWM = 46;     // 9    Вентилятор на радиаторе 2 
  
// Таймеры
uint8_t timer1 = A0;    // pin analog
uint8_t timer2 = A1;    // pin analog 
uint8_t timer3 = A3;    // pin analog
uint8_t timer4 = A7;    // pin analog
uint8_t timer5 = A4;    // pin analog
// Дозаторы
uint8_t pump1 =  A5;    // pin analog A5          Дозатор1
uint8_t pump2 =  A6;    // pin analog A6          Дозатор2
uint8_t pump3 = A10;    // pin analog A10         Дозатор3
uint8_t pump4 = A14;    // pin analog A14         Дозатор4

uint8_t vacpump = A6;   // pin analog A6  Вакуумный насос

uint8_t SDchipSelect = 53; // SD card attached to SPI bus


#endif


// ------------- FOR AQUA SHIELD V3 Распиновка для Аквашилда Олега-------------------------------


#ifdef Aqua_shield_v3

// ВХОДЫ
uint8_t SensLight   = A13;   // датчик освещения, на аналоговом пине 
uint8_t SensLevel    = A8;   // датчик уровня воды, на аналоговом пине 
OneWire OneWireBus(A15);     // Датчик температуры  (Dallas pin)
 
// ВЫХОДЫ
uint8_t ledPinWhite   = 2;   // WWT  Теплый белый    (2) - 11 бит   
uint8_t ledPinBlue    = 3;   // CWT  Холодный белый  (3) - 11 бит
uint8_t ledPinRoyBlue = 5;   // RBL  Глубой          (5) - 11 бит
uint8_t ledPinRed     = 6;   // RED  Красный         (6) - 11 бит
uint8_t ledPinUV      = 7;   // UVL  Фиолетовый      (7) - 11 бит
uint8_t ledPinOrange  = 8;   // ORG  Оранжевый       (8) - 11 бит
uint8_t ledPinGr     = 12;   // GRN  Зеленый        (11) - 11 бит

uint8_t ledPinMoon    = 4;   // Moon Led pin     (4) - ( луна 8 бит - 255 )

uint8_t LCDbrightPin = 13;   // подсветка LCD   (13) - (8 бит)

uint8_t tempHeatPin       = A5;    // A5   Нагреватель вкл./выкл. 
uint8_t tempChillPin      = 46;    // 46   Холодильник вкл./выкл. 
uint8_t tempAlarmPin     = A11;    // A11  Buzzer Alarm 
uint8_t autoFeeder        = A7;    // А7   Пин кормушки
uint8_t Heatsink1_FansPWM = 44;    // 44   Вентилятор на радиаторе 1 
uint8_t Heatsink2_FansPWM = 9;     // 9    Вентилятор на радиаторе 2 
  
// Таймеры
uint8_t timer1 = A0;    // pin analog 0 Аэрация
uint8_t timer2 = A1;    // pin analog 1 СО2
uint8_t timer3 = A3;    // pin analog 3 Фильтр
uint8_t timer4 = A2;    // pin analog 2 Уф лампа
uint8_t timer5 = A4;    // pin analog 4 Долив

uint8_t pump1 =  10;    // pin 10          Дозатор1
uint8_t pump2 =  A9;    // pin analog A9   Дозатор2
uint8_t pump3 =  45;    // pin 45          Дозатор3
uint8_t pump4 = A14;    // pin analog A14  Дозатор4

uint8_t vacpump = A6;   // pin analog A6  Вакуумный насос

uint8_t SDchipSelect = 53; // SD card attached to SPI bus


#endif

//======================================================================================


//const int maxModos = 6;  // колличество шагов для счетчика переключения режимов 
//int Mode1, Mode2, Mode3, Mode4, Mode5, Mode6;
//int ModeSel = 6;           // 6 режим работы по умолчанию - помпы выключены

//int ValPoten = 0;  // переменный резистор
 
long times = 0; 
long starttime = 0; 
int startvalue = 255;
//byte periodtime = 50000;   // время для переключения в режиме 3
//int value = 0;     // значение уровней
int Pump1PWM = 0;    // значение уровня для помпы 1 = 0
int Pump2PWM = 0;

byte value = 0;    // значение уровней
int periode;       // значение периодов для обычного режима (в setup-5000} 

int cmode = 0;     // переключение режимов 
int aclock;
int sec,sec1,sec2,sec3,sec4;
int waterlevel=1;

// PH
  #define PHADDRESS 0x4D // адрес PH датчика
  int RoomTempI2CAddress = B1001011; // адрес датчика температуры на модуле PH

   double SetvalPH;
   float volt7 = 0.6939;                     // Напряжения калибровки датчика PH
   float volt10 = 0.3846;
  
  float calibrationTempC = 20;
    
  double phVolt;    
  
  double voltsPerPH;
  
  double realPHVolt;
  double phUnits;
  double measuredPH;
 
  double roomTempC; 
  double roomTempCompensatedMeasuredPH;
  int sampleSize = 200;
  
  double avgMeasuredPH = 0;
  double avgRoomTempC = 0;
  double avgPHVolts = 0;
  double avgRoomTemperatureCompensatedMeasuredPH = 0;
  
  double tempAdjusted10;
  
  
   
 
float adjustPHBasedOnTemp(float PH, float temp)
{
   // http://www.omega.com/Green/pdf/pHbasics_REF.pdf
   // When the temperature is other than 25degC and the ph is other than 7
   // the temperature error is 0.03ph error/ph unit/10degC
   // which means error = 0.03*(ph away from 7)*(tempdiffC/10)
   
    float phDifference = abs(PH-7);
    float tempDifferenceC = abs(temp-25);
    float phAdjust = (0.03*phDifference)*(tempDifferenceC/10);
    
    if(PH>7 && temp<25)
      phAdjust=phAdjust;
 
    if(PH>7 && temp>25)
      phAdjust=phAdjust*-1;
 
    if(PH<7 && temp>25)
      phAdjust=phAdjust;
 
    if(PH<7 && temp<25)
      phAdjust=phAdjust*-1;
 
    float tempAdjustedPH = PH + phAdjust;
    return tempAdjustedPH;
}
 
 
double getPHVolts()
{
  byte ad_high;
  byte ad_low;
  
  Wire.requestFrom(PHADDRESS, 2);        //requests 2 bytes
  while(Wire.available() < 2);         //while two bytes to receive
    
  ad_high = Wire.read();           
  ad_low = Wire.read();
  double units = (ad_high * 256) + ad_low;
  
  double volts =  (units /4096)*3; 
  return volts;  
}
 
 
double getRoomTemperatureC()
{
  Wire.requestFrom(RoomTempI2CAddress,2);
  byte MSB = Wire.read();
  byte LSB = Wire.read();
 
  int TemperatureSum = ((MSB << 8) | LSB) >> 4;
  double celsius = TemperatureSum*0.0625;
  
  return celsius;
}
 
void SetRoomTemperataureResolutionBits(int ResolutionBits)
{
  if (ResolutionBits < 9 || ResolutionBits > 12) exit;
  Wire.beginTransmission(RoomTempI2CAddress);
  Wire.write(B00000001); //addresses the configuration register
  Wire.write((ResolutionBits-9) << 5); //writes the resolution bits
  Wire.endTransmission();
 
  Wire.beginTransmission(RoomTempI2CAddress); //resets to reading the temperature
  Wire.write((byte)0x00);
  Wire.endTransmission();
}
 

  


//кормушка

byte feedTime;
byte FEEDTime1, FEEDTime2,
     FEEDTime3, FEEDTime4;

byte feedFish1H, feedFish1M,          //Times to feed the fish
     feedFish2H, feedFish2M,
     feedFish3H, feedFish3M,
     feedFish4H, feedFish4M;
     
     boolean FeedWaveCtrl_1 = false;
     boolean FeedWaveCtrl_2 = false;
     boolean FeedWaveCtrl_3 = false;
     boolean FeedWaveCtrl_4 = false;
     
     
int AM_PM; 

int  xTimeAMPM;
//byte setAutoStop = 0;
byte fiveTillBackOn1, fiveTillBackOn2,
     fiveTillBackOn3, fiveTillBackOn4;
     
//дозатор удо  

byte dozTime;
byte DOZTime1, DOZTime2,
     DOZTime3, DOZTime4;

byte dozPump1H, dozPump1M,          
     dozPump2H, dozPump2M,
     dozPump3H, dozPump3M,
     dozPump4H, dozPump4M;
     
byte numDoz1,numDoz2,numDoz3,numDoz4,
     intDoz1,intDoz2,intDoz3,intDoz4;     
     
int dozVal1, dozVal2, dozVal3, dozVal4;
int dozPart1, dozPart2, dozPart3, dozPart4;
int dozCal1, dozCal2, dozCal3, dozCal4;
int DosSec1, DosSec2, DosSec3, DosSec4;
int CalMode;

byte shiftH12, shiftH13, shiftH14;
byte shiftH22, shiftH23, shiftH24; 
byte shiftH32, shiftH33, shiftH34; 
byte shiftH42, shiftH43, shiftH44;

int Doscalibrate1, Doscalibrate2 ,Doscalibrate3 ,Doscalibrate4;

// oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&OneWireBus); 

// SD карта
SdFat sd;
SdFile myFile;
#define error(s) sd.errorHalt_P(PSTR(s))


// Assign the addresses of temperature sensors.  Add/Change addresses as needed.
// ROM = 28 14 4F CC 3 0 0 D0 
DeviceAddress tempDeviceAddress;
DeviceAddress Heatsink1Thermometer; // датчик температуры радиатора 1
DeviceAddress Heatsink2Thermometer; // датчик температуры радиатора 2
DeviceAddress waterThermometer;     // датчик температуры воды в аквариуме

byte  resolution = 11; // разрешение датчика
unsigned long lastTempRequest = 0;
int  delayInMillis = 0;
//unsigned long logtempminutoantes = 0;  // Для записи лога температуры на флеш карту 

byte counterB1=0;            // счетчик 1
byte counterB2=0;            // счетчик 2
byte counterB3=0;
byte numberOfDevices=0;      // quantity of Dallas sensor connected to board

unsigned long previousMillis1sec;  

float tempW = 0;                          // Датчик температуры в аквариуме
float tempH1 = 0;                         // Датчик температуры на радиаторе 1
float tempH2 = 0;                         // Датчик температуры на радиаторе 2



float setTempToBeginHeatsink1FanC=0.0;    // Temperature to Turn on Heatsink1 Fans (in Degrees C)
float setTempToBeginHeatsink2FanC=0.0;    // Temperature to Turn on Heatsink2 Fan (in Degrees C)
int  setTempToSoundAlarmC=0;              // Temperature to Heatsink Sound alarm (in Degrees C)

float temp2beHFan;                   // Temporary Temperature Values
float temp2beSFan;                   // Temporary Temperature Values

float FanOn = 0.2;                   // Starts Fan(s) at 20% Duty Cycle (choose 0.2 or higher)

int Heatsink1TempInterval = 0;       // Used for PWM Duty calculations
int Heatsink2TempInterval = 0;       // Used for PWM Duty calculations
byte Heatsink1PWM = 0;               // Used for PWM Duty calculations
byte Heatsink2PWM = 0;               // Used for PWM Duty calculations

float setTempC = 0.0;                // Desired Water Temperature (User input in program)
float offTempC = 0.0;                // Desired Water Temp. Offsets for Heater & Chiller (User input in program)
float alarmTempC = 0.0;              // Temperature the Alarm will sound (User input in program)
boolean tempCoolflag = 0;            // 1 if cooling on
boolean tempHeatflag = 0;            // 1 if heating on
boolean tempAlarmflag = 0;           // 1 if alarm on

boolean tempAlarmflagH1 = 0;         // 1 if alarm on
boolean tempAlarmflagH2 = 0;         // 1 if alarm on
boolean AlarmflagON ;
boolean RefreshAfterError= false;

float temp2beS;                      // Temporary Temperature Values
float temp2beO;                      // Temporary Temperature Values
float temp2beA;                      // Temporary Temperature Values

float MaxTempW;       // максимумы температуры для датчика воды 
float MaxTempH1;      //  --//--               для датчика на радиаторе 1
float MaxTempH2;      //  --//--               для датчика на радиаторе 2

int DimmL=0;                         // диммирование вручную 1-да, 0-нет
int setLEDsDimPercentL = 0;
int TempsetLEDsDimPercentL;
float PercentDimL = 0.0;

// int ledON = 1; // при настройке пресетов каналы продолжают работать в штатном режиме

byte setDimLEDsOnOff = 0;             // If/When LEDs reach a certain temp they can dim down (feature off by default)
byte setLEDsDimTempC = 0;             // Default value is 0, set this value in program
byte TempLEDsDimTemp;                 // Temporary LED Dimming Temp
byte setLEDsDimPercent = 0;           // Choose value to failsafe dim LEDs in program
byte TempLEDsDimPercent;              // Temporary LED Dimming Percent
float PercentDim = 0.0;               // Converts saved value in EEPROM to a percentage
//int tempLED=setLEDsDimTempC+5;
float PercentSoftStart = 0.0;         // change from 0 to 1 with 0.1 increment, every 5sec after programm is started 
 
extern unsigned int up[0x5D0];        // картинка стрелка вверх для таймеров
extern unsigned int down[0x5D0];      // картинка стрелка вниз  для таймеров 
extern unsigned int clos[0x240];      // крестик (закрыть)  
extern unsigned int preset[0x195];    // preset

// Картинки фаз луны  http: //arduino.cc/forum/index.php/topic,134649.0.html
   uint_farptr_t MoonPic;              // Pointer to the Lunar Phase Pic
   extern unsigned int Full_Moon[0xAF9];         // Lunar Phase Pics   
   extern unsigned int First_Quarter[0xAF9];  

float LC = 29.53059;       // 1 Lunar Cycle = 29.53059 days
String LP;                 // LP = Lunar Phase - variable used to print out Moon Phase
double AG;
byte tMaxI;                // Maximum Illumination of Moon (User Defined/Set in Prog. -- Default = 0)
byte tMinI;                // Minimum Illumination of Moon (User Defined/Set in Prog. -- Default = 0)
byte MinI = 5;             // минимальная яркость луны по умолчанию
byte MaxI = 90;            // максимальная

float lunar_perc;
 
//================================= Текстовые надписи в printHeader (верхний баннер) ========================================
 PROGMEM const char Header_Text_string0[] ="CK@BMNE LEM^";                           // ГЛАВНОЕ МЕНЮ
 PROGMEM const char Header_Text_string1[] ="SQR@MNBJ@ BPELEMH H D@R[";               // УСТАНОВКА ВРЕМЕНИ И ДАТЫ
 PROGMEM const char Header_Text_string2[] ="SQR@MNBJ@ RELOEP@RSP[ BND[";             // УСТАНОВКА ТЕМПЕРАТУРЫ ВОДЫ
 PROGMEM const char Header_Text_string3[] ="@BRNL@RHWEQJNE REQRHPNB@MHE J@M@KNB";    // АВТОМАТИЧЕСКОЕ ТЕСТИРОВАНИЕ КАНАЛОВ
 PROGMEM const char Header_Text_string4[] ="PSWMNI REQR NQBEYEMH_";                  // РУЧНОЙ ТЕСТ ОСВЕЩЕНИЯ
 PROGMEM const char Header_Text_string5[] ="M@QRPNIJ@ J@M@KNB ON VBER@L";            // НАСТРОЙКА КАНАЛОВ ПО ЦВЕТАМ
 PROGMEM const char Header_Text_string6[] ="SQR@MNBJ@ _PJNQRH KSM[";                 // УСТАНОВКА ЯРКОСТИ ЛУНЫ
 PROGMEM const char Header_Text_string7[] ="@BRNDNKHB";                              // АВТОДОЛИВ        
 PROGMEM const char Header_Text_string8[] ="M@QRPNIJ@ DN3@RNPNB";                    // НАСТРОЙКА ДОЗАТОРОВ
 PROGMEM const char Header_Text_string9[] ="M@QRPNIJ@ OND@WH SCKEJHQKNR[";           // НАСТРОЙКА ПОДАЧИ УГЛЕКИСЛОТЫ 
 PROGMEM const char Header_Text_string10[] ="CK@BM[E M@QRPNIJH Qrp. 1";              // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 1
 PROGMEM const char Header_Text_string11[] ="CK@BM[E M@QRPNIJH Qrp. 2";              // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 2
 PROGMEM const char Header_Text_string12[] ="CK@BM[E M@QRPNIJH Qrp. 3";              // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 3 
 PROGMEM const char Header_Text_string13[] ="M@QRPNIJ@ _PJNQRH ]JP@M@";              // НАСТРОЙКА ЯРКОСТИ ЭКРАНА (подсветка)
 PROGMEM const char Header_Text_string14[] ="SQR@MNBJ@ RELOEP@RSP[ BJK.BEMRHK_RNPNB";// УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ
 PROGMEM const char Header_Text_string15[] ="SLEM&XEMHE _PJNQRH OPH OEPECPEBE";      // УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ
 PROGMEM const char Header_Text_string16[] ="SQR@MNBJH UP@MHREK_ ]JP@M@";            // УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА
 PROGMEM const char Header_Text_string17[] ="QSRNWM[E R@ILEP[";                      // УСТАНОВКИ ТАЙМЕРОВ
# ifdef freshwater
 PROGMEM const char Header_Text_string18[] ="SQR@MNBJH R@ILEP@ 1 @]P@VH_";           // УСТАНОВКИ ТАЙМЕРА 1 АЭРАЦИЯ
 PROGMEM const char Header_Text_string19[] ="SQR@MNBJH R@ILEP@ 2 QN2";               // УСТАНОВКИ ТАЙМЕРА 2 СО2
 PROGMEM const char Header_Text_string20[] ="SQR@MNBJH R@ILEP@ 3 THK&RP";            // УСТАНОВКИ ТАЙМЕРА 3 ФИЛЬТР
 PROGMEM const char Header_Text_string21[] ="SQR@MNBJH R@ILEP@ 4 ST K@LO@";          // УСТАНОВКИ ТАЙМЕРА 4 УФ ЛАМПА
 PROGMEM const char Header_Text_string22[] ="SQR@MNBJH R@ILEP@ 5 DNKHB";             // УСТАНОВКИ ТАЙМЕРА 5 ДОЛИВ
# endif
# ifdef seawater
 PROGMEM const char Header_Text_string18[] ="SQR@MNBJH R@ILEP@ 1";                   // УСТАНОВКИ ТАЙМЕРА 1
 PROGMEM const char Header_Text_string19[] ="SQR@MNBJH R@ILEP@ 2";                   // УСТАНОВКИ ТАЙМЕРА 2
 PROGMEM const char Header_Text_string20[] ="SQR@MNBJH R@ILEP@ 3";                   // УСТАНОВКИ ТАЙМЕРА 3
 PROGMEM const char Header_Text_string21[] ="SQR@MNBJH R@ILEP@ 4";                   // УСТАНОВКИ ТАЙМЕРА 4
 PROGMEM const char Header_Text_string22[] ="SQR@MNBJH R@ILEP@ 5";                   // УСТАНОВКИ ТАЙМЕРА 5
# endif
 PROGMEM const char Header_Text_string23[] ="PSWMNE SOP@BKEMHE R@ILEP@LH";           // РУЧНОЕ УПРАВЛЕНИЕ ТАЙМЕРАМИ                    
 PROGMEM const char Header_Text_string24[] ="@BRNONHQJ D@RWHJNB RELOEP@RSP[";        // АВТОПОИСК ДАТЧИКОВ ТЕМПЕРАТУРЫ
 PROGMEM const char Header_Text_string25[] ="QNUP@MHR&, 3@CPS3HR& M@QRPNIJH";        // СОХРАНИТЬ, ЗАГРУЗИТЬ НАСТРОЙКИ
 PROGMEM const char Header_Text_string26[] ="SQR@MNBJ@ BPELEMH DN3@RNPNB";           // УСТАНОВКА ВРЕМЕНИ ДОЗАТОРОВ//"PEFHL P@ANR[ THK&RP@";  // РЕЖИМ РАБОТЫ ФИЛЬТРА
 PROGMEM const char Header_Text_string27[] ="SQR@MNBJ@ BPELEMH JNPLKEMH_";           // УСТАНОВКА ВРЕМЕНИ КОРМЛЕНИЯ

const PROGMEM char* const PROGMEM Header_Text_table[]  ={ 
 Header_Text_string0, Header_Text_string1, Header_Text_string2, Header_Text_string3, Header_Text_string4, 
 Header_Text_string5, Header_Text_string6, Header_Text_string7, Header_Text_string8, Header_Text_string9, 
 Header_Text_string10, Header_Text_string11, Header_Text_string12, Header_Text_string13, Header_Text_string14, 
 Header_Text_string15, Header_Text_string16, Header_Text_string17, Header_Text_string18, Header_Text_string19, 
 Header_Text_string20, Header_Text_string21, Header_Text_string22, Header_Text_string23, Header_Text_string24, 
 Header_Text_string25, Header_Text_string26, Header_Text_string27, };
 
//====================== Текстовые надписи в меню, хранящиеся во флеш ==========================
 PROGMEM const char Text_string0[] ="BPEL_:";                 // ВРЕМЯ: 
 PROGMEM const char Text_string1[] ="D@R@:";                  // ДАТА:
 PROGMEM const char Text_string2[] =""; // ----------------------------------------------------- 
 PROGMEM const char Text_string3[] ="B TNPL@RE";              // В ФОРМАТЕ (в меню часов) 
 PROGMEM const char Text_string4[] ="BLANK";                  // BLANK (пустой экран)
 PROGMEM const char Text_string5[] ="CLOCK";                  // ЧАСЫ (CLOCK)
 PROGMEM const char Text_string6[] ="Rho }jp`m`";             // Тип экрана
 PROGMEM const char Text_string7[] ="ONJ@3[B@R& D@RS";        // ПОКАЗЫВАТЬ ДАТУ 
 PROGMEM const char Text_string8[] ="YES";                    // ДА
 PROGMEM const char Text_string9[] ="NO";                     // НЕТ
 PROGMEM const char Text_string10[] ="@.W@Q[";                // А.ЧАСЫ (аналоговые часы)
 PROGMEM const char Text_string11[] ="V.W@Q[";                // Ц.ЧАСЫ (цифровые часы)
 PROGMEM const char Text_string12[] ="D@RW.";                 // Датч. D`rw. // ДАТЧ.
 PROGMEM const char Text_string13[] ="LNMHRNP QNA[RHI";       // МОНИТОР СОБЫТИЙ
 PROGMEM const char Text_string14[] ="TIME SECTOR";           // СЕКТОР ВРЕМЕНИ
 PROGMEM const char Text_string15[] ="24HR";                  // 24ч.(формат)
 PROGMEM const char Text_string16[] ="C";                     // С (цельсий)
 PROGMEM const char Text_string17[] ="OK";                    // OK
 PROGMEM const char Text_string18[] ="SQR@M.RELO.";           // УСТАН.ТЕМП.
 PROGMEM const char Text_string19[] ="Qnup`mhr| Sqr`mnbjh";   // Сохранить Установки  
 PROGMEM const char Text_string20[] ="3`cpsghr| Sqr`mnbjh";   // Загрузить Установки   
 PROGMEM const char Text_string21[] ="M@QRPNHR;";             // НАСТРОИТЬ   
 PROGMEM const char Text_string22[] ="* SOP@BKEMHE @JB@PHSLNL *"; // УПРАВЛЕНИЕ АКВАРИУМОМ  
 PROGMEM const char Text_string23[] ="_PJNQR& J@M@KNB";       // ЯРКОСТЬ КАНАЛОВ
 PROGMEM const char Text_string24[] ="T@3@ KSM[";             // ФАЗА ЛУНЫ
 PROGMEM const char Text_string25[] ="RELOEP@R.";             // ТЕМПЕРАТ. (RELOEP@RSP@)  (reloep.) 
 PROGMEM const char Text_string26[] =" OFF ";                 // ( OFF )
 PROGMEM const char Text_string27[] ="BND@:";                 // ВОДА :  ТЕМП.ВОДЫ (RELO BND[:)  
 PROGMEM const char Text_string28[] ="P@D:1";                 // РАД:1   РАДИАТ Д1 (P@DH@R D1:) 
 PROGMEM const char Text_string29[] ="P@D:2";                 // РАД:2   РАДИАТ Д2 (P@DH@R D2:)
 PROGMEM const char Text_string30[] ="Err.";                  // Err. (Ошибка)
 PROGMEM const char Text_string31[] ="UNKNDHK&MHJ";           // ХОЛОДИЛЬНИК 
 PROGMEM const char Text_string32[] ="M@CPEB@REK&";           // НАГРЕВАТЕЛЬ 
 PROGMEM const char Text_string33[] ="ALARM!!";               // ТРЕВОГА ! 
 PROGMEM const char Text_string34[] ="(DD/LL/CCCC)";          // (ДД/MM/ГГГГ)  (день, месяц, год)
 PROGMEM const char Text_string35[] ="CP@THJ QBERNBNCN DM_";  // ГРАФИК СВЕТОВОГО ДНЯ
 PROGMEM const char Text_string36[] =""; // -------------------------------------------------------
 PROGMEM const char Text_string37[] ="3`d`mm`$ Reloep`rsp`:"; // Заданная Температура  
 PROGMEM const char Text_string38[] ="Chqrepeghq Reloep`rsp{";// Гистерезис Температуры
 PROGMEM const char Text_string39[] ="Chqrepeghq Rpebnch";    // Гистерезис Тревоги
 PROGMEM const char Text_string40[] ="Bpel$";                 // Время       
 PROGMEM const char Text_string41[] ="Boeped";                // Вперед     
 PROGMEM const char Text_string42[] ="M`g`d";                 // Назад
 PROGMEM const char Text_string43[] ="Test in Progress";      // Test in Progress
 PROGMEM const char Text_string44[] ="TIME";                  // ВРЕМЯ
 PROGMEM const char Text_string45[] ="SPNBMH B[UNDNB (0-100%)"; // УРОВНИ ВЫХОДОВ (0-100%)
 PROGMEM const char Text_string46[] ="000";                   // 000 (три нуля)
 PROGMEM const char Text_string47[] ="R.AEK[I";                 // БЕЛЫЙ
 PROGMEM const char Text_string48[] ="QHMHI";                 // СИНИЙ
 PROGMEM const char Text_string49[] ="JP@QM[I";               // КРАСНЫЙ   
 PROGMEM const char Text_string50[] ="KSM@";                  // ЛУНА
 PROGMEM const char Text_string51[] ="NP@MFEB[I";             // ОРАНЖЕВЫЙ
 PROGMEM const char Text_string52[] ="THNKERNB[I";            // ФИОЛЕТОВЫЙ
 PROGMEM const char Text_string53[] ="U.AEK[I";               // ГОЛУБОЙ   CNKSANI
 PROGMEM const char Text_string54[] ="GEKEM[I";               // ЗЕЛЕНЫЙ   
 PROGMEM const char Text_string55[] ="3M@WEMH_ _PJNQRH DK_ AEKNCN J@M@K@";     // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ БЕЛОГО КАНАЛА
 PROGMEM const char Text_string56[] ="DK_ OEPEUND@ J CP@THJ@L-M@F@R& M@ ]JP@M";// ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН
 PROGMEM const char Text_string57[] ="3M@WEMH_ _PJNQRH DK_ CNKSANCN J@M@K@";   // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ГОЛУБОГО КАНАЛА
 PROGMEM const char Text_string58[] ="3M@WEMH_ _PJNQRH DK_ QHMECN J@M@K@";     // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ СИНЕГО КАНАЛА
 PROGMEM const char Text_string59[] ="3M@WEMH_ _PJNQRH DK_ JP@QMNCN J@M@K@";   // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ КРАСНОГО КАНАЛА
 PROGMEM const char Text_string60[] ="3M@WEMH_ _PJNQRH DK_ THNKERNBNCN J@M@K@";// ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ФИОЛЕТОВОГО КАНАЛА
 PROGMEM const char Text_string61[] ="3M@WEMH_ _PJNQRH DK_ NP@MFEBNCN J@M@K@"; // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ОРАНЖЕВОГО КАНАЛА
 PROGMEM const char Text_string62[] ="3M@WEMH_ _PJNQRH DK_ 3EKEMNCN J@M@K@";   // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ЗЕЛЕНОГО КАНАЛА 
 PROGMEM const char Text_string63[] ="Lhmhl`k|m`=";           // Минимальная
 PROGMEM const char Text_string64[] ="L`jqhl`k|m`=";          // Максимальная
 PROGMEM const char Text_string65[] ="_pjnqr|";               // Яркость (в меню луны)
 PROGMEM const char Text_string66[] ="MNB@_ KSM@";            // НОВАЯ ЛУНА
 PROGMEM const char Text_string67[] ="ONKM@_ KSM@";           // ПОЛНАЯ ЛУНА
 PROGMEM const char Text_string68[] ="B BEPUMEI W@QRH ]JP@M@ B[AP@R&";   //  В ВЕРХНЕЙ ЧАСТИ ЭКРАНА ВЫБРАТЬ 
 PROGMEM const char Text_string69[] ="K^ANI DBSUW@QNBNI NRPE3NJ BPELEMH";// ЛЮБОЙ ДВУХЧАСОВОЙ ОТРЕЗОК ВРЕМЕНИ
 PROGMEM const char Text_string70[] ="H SQR@MNBHR& 3M@WEMH_ _PJNQRH";    //  И УСТАНОВИТЬ ЗНАЧЕНИЯ ЯРКОСТИ
 PROGMEM const char Text_string71[] ="DK_ B[AP@MMNCN J@M@K@";            //      ДЛЯ ВЫБРАННОГО КАНАЛА
 PROGMEM const char Text_string72[] ="DN3@RNP[";             // ДОЗАТОР УДО
 PROGMEM const char Text_string73[] ="BND[";                   // ВОДЫ 
 PROGMEM const char Text_string74[] ="JNPLSXJ@";               // КОРМУШКА
 PROGMEM const char Text_string75[] ="B[JK^WHR& THK&RP";       // ВЫКЛЮЧИТЬ ФИЛЬТР
 PROGMEM const char Text_string76[] ="SPNBEM&";         // УРОВЕНЬ // ТЕКУЩИЙ РЕЖИМ: REJSYHI PEFHL: 
 PROGMEM const char Text_string77[] ="QKHB";          // СЛИВ // НИ ОДИН РЕЖИМ НЕ ВЫБРАН ! "MH NDHM PEFHL ME B[AP@M !"
 PROGMEM const char Text_string78[] ="SQR@MNBJ@ SPNBM_ PM";  //  B{aephre pefhl p`anr{ onlo rewemh$"; // Выберите режим работы помп течения/ Установка уровня РН
 PROGMEM const char Text_string79[] ="D@RWHJ@ PM";          //     Переменный или Синхронный Oepelemm{i hkh Qhmupnmm{i
 PROGMEM const char Text_string80[] ="REJSYEE";         //  Текущее  /   Затем СОХРАНИТЕ Настройки g`rel QNUP@MHRE M`qrpnijh.
 PROGMEM const char Text_string81[] ="3M@WEMHE"; //Значение/ ПОМПЫ РАБОТАЮТ В ПЕРЕМЕННОМ РЕЖИМЕ(при настройке большой шрифт)ONLO[ P@ANR@^R B OEPELEMMNL PEFHLE
 PROGMEM const char Text_string82[] ="PM";                       // PH
 PROGMEM const char Text_string83[] ="hkh m`flhre jmnojs BJK^WHR& ONLO[";  //  или нажмите кнопку ВКЛЮЧИТЬ ПОМПЫ    
 PROGMEM const char Text_string84[] ="7";                // 7
 PROGMEM const char Text_string85[] ="10";                // 10 
 PROGMEM const char Text_string86[] ="BJK^WEM ONQRN_MMN";     // ВКЛЮЧЕНЫ ПОСТОЯННО (кнопка)
 PROGMEM const char Text_string87[] ="THK&RP";    // ФИЛЬТР
 PROGMEM const char Text_string88[] ="ON OPHMVHOS";       // ПО ПРИНЦИПУ  
 PROGMEM const char Text_string89[] ="BJK./ B[JK.";       // ВКЛ./ВЫКЛ.  
 PROGMEM const char Text_string90[] ="PEFHL OSK&Q@VHH";         // РЕЖИМ ПУЛЬСАЦИИ 
 PROGMEM const char Text_string91[] ="THK&RP BJK^WEM";          // ФИЛЬТР ВКЛЮЧЕН   
 PROGMEM const char Text_string92[] ="  THK&RP B[JK^WEM  ";         // ФИЛЬТР ВЫКЛЮЧЕН 
 PROGMEM const char Text_string93[] ="BJK^WHR& THK&RP";          // ВКЛЮЧИТЬ ФИЛЬТР
 PROGMEM const char Text_string94[] ="  THK&RP B[JK^WEM  ";          // ФИЛЬТР ВЫКЛЮЧЕН  
 PROGMEM const char Text_string95[] ="Wrna{ bjk~whr| Onlo{ h bngnamnbhr|"; // Чтобы включить Помпы и возобновить 
 PROGMEM const char Text_string96[] ="p`anrs b oepelemmnl pefhle";         //     работу в переменном режиме 
 PROGMEM const char Text_string97[] ="p`anrs b qhmupnmmnl pefhle";         //     работу в синхронном режиме
 PROGMEM const char Text_string98[] ="B{aephre pefhl jmnojni bbepus";      //    Выберите режим кнопкой вверху 
 PROGMEM const char Text_string99[] ="Feeding Mode";            // Режим Кормления
 PROGMEM const char Text_string100[] ="THK&RP B[JK^WEM !!";            // ФИЛЬТР ВЫКЛЮЧЕН !!   
 PROGMEM const char Text_string101[] ="THK&RP ASDER BJK^WEM";              // ФИЛЬТР БУДЕТ ВКЛЮЧЕН
 PROGMEM const char Text_string102[] ="wepeg:";                            //  через:  00:00 (большой шрифт)
 PROGMEM const char Text_string103[] ="h bngnamnbkem Oepelemm{i pefhl";    // и возобновлен Переменный режим
 PROGMEM const char Text_string104[] ="h bngnamnbkem Qhmupnmm{i pefhl";    // и возобновлен Синхронный режим
 PROGMEM const char Text_string105[] ="<< BACK";                // << НАЗАД (BACK)
 PROGMEM const char Text_string106[] =""; // --------------------------------------------------------------- 
 PROGMEM const char Text_string107[] ="SAVE";                              // СОХРАНИТЬ
 PROGMEM const char Text_string108[] ="SQR@MNBHRE OPNDNKF. P@ANR[ ONLO";   // УСТАНОВИТЕ ПРОДОЛЖ. РАБОТЫ ПОМП 
 PROGMEM const char Text_string109[] ="ONLO@ 1";                           // ПОМПА 1
 PROGMEM const char Text_string110[] ="ONLO@ 2";                           // ПОМПА 2
 PROGMEM const char Text_string111[] ="Lhmsr";                             // Минут
 PROGMEM const char Text_string112[] ="Qejsmd";                            // Секунд
 PROGMEM const char Text_string113[] =""; // ----------------------------------------------------------------
 PROGMEM const char Text_string114[] ="QBERNDHNDNB DN:";                        // CВЕТОДИОДОВ ДО:
 PROGMEM const char Text_string115[] ="JNK-BN ONPVHI   >";                         // КОЛИЧЕСТВО ДОЗ  //Выберите один из режимов работы, B{aephre ndhm hg pefhlnb p`anr{,
 PROGMEM const char Text_string116[] ="HMREPB@K (w`q{) >";                        // ИНТЕРВАЛ //Либо помпы будут постоянно включены  Khan onlo{ asdsr onqrn$mmn bjk~wem{
 PROGMEM const char Text_string117[] ="BBEDHRE NAZEL >";                          // ВВЕДИТЕ ОБЪЕМ      "Khan asdsr p`anr`r| b pefhle osk|q`vhh"; // либо будут работать в режиме пульсации
 PROGMEM const char Text_string118[] ="Qej.";                                   // Сек. "ONLO[ ASDSR BJK^WEM[ ONQRN_MMN";         // ПОМПЫ БУДУТ ВКЛЮЧЕНЫ ПОСТОЯННО 
 PROGMEM const char Text_string119[] ="NAZEL DN3[ (Lk) >";                        // ОБЪЕМ дозы  
 PROGMEM const char Text_string120[] ="BPEL_ DN3[";                  // Время работы дозатора     DN3@RNP@ 
 PROGMEM const char Text_string121[] ="Lk.";                                    // Mл   
 PROGMEM const char Text_string122[] ="J@KHAPNBJ@";                             // КАЛИБРОВКА //"ONLO[ ASDSR BJK^WEM[ B REWEMHH";    // ПОМПЫ БУДУТ ВКЛЮЧЕНЫ В ТЕЧЕНИИ
 PROGMEM const char Text_string123[] ="DN3@RNP";                                // ДОЗАТОР  //"ONLO[ ASDSR B[JK^WEM[ B REWEMHH";   // ПОМПЫ БУДУТ ВЫКЛЮЧЕНЫ В ТЕЧЕНИИ
 PROGMEM const char Text_string124[] ="W@Q[";                                   // ЧАСЫ 
 PROGMEM const char Text_string125[] ="LHMSR[";                                 // МИНУТЫ
 PROGMEM const char Text_string126[] ="OPNDNKF.";                               // ПРОДОЛЖ.
 PROGMEM const char Text_string127[] ="SQR@MNB. RELO.";                         // УСТАНОВ. ТЕМП.   
 PROGMEM const char Text_string128[] ="BJK.";                                   // ВКЛ.
 PROGMEM const char Text_string129[] ="Spnbmh";                                 // Уровни
 PROGMEM const char Text_string130[] ="QNUP@MHR&";                              // СОХРАНИТЬ
 PROGMEM const char Text_string131[] ="Servo";                                  // Servo
 PROGMEM const char Text_string132[] ="LHMSR";                                  // МИНУТ 
 PROGMEM const char Text_string133[] ="ONDQBERJ@ ]JP@M@";                       // ПОДСВЕТКА ЭКРАНА
 PROGMEM const char Text_string134[] ="RELO.BJK.BEMRHK_RNP@";                   // ТЕМП. ВКЛЮЧЕНИЯ ВЕНТИЛЯТОРА
 PROGMEM const char Text_string135[] ="@BRN-SLEM&X. _PJNQRH";                   // АВТО-УМЕНЬШ. ЯРКОСТИ
 PROGMEM const char Text_string136[] ="OPH OEPECPEBE";                          // ПРИ ПЕРЕГРЕВЕ 
 PROGMEM const char Text_string137[] ="SQR@MNBJH";                              // УСТАНОВКИ
 PROGMEM const char Text_string138[] ="UP@MHREK_ ]JP@M@";                       // ХРАНИТЕЛЯ ЭКРАНА
 PROGMEM const char Text_string139[] ="NCP@MHWEMHE LNYMNQRH";                   // ОГРАНИЧЕНИЕ МОЩНОСТИ  
 PROGMEM const char Text_string140[] ="BEMRHK_RNP NUK@FDEMH_ P@DH@RNP@-1";      // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-1 
 PROGMEM const char Text_string141[] ="BEMRHK_RNP NUK@FDEMH_ P@DH@RNP@-2";      // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-2 
 PROGMEM const char Text_string142[] ="Reloep`rsp` Bjk~wemh$";                  // ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ
 PROGMEM const char Text_string143[] ="BEKHWHM@ QHCM@K@ BQEU J@M@KNB";          // величина сигнала всех каналов
 PROGMEM const char Text_string144[] ="(25-50)";                                // (25-50) граница температур
 PROGMEM const char Text_string145[] ="OPH M@CPEBE P@DH@RNP@";                  // ПРИ НАГРЕВЕ РАДИАТОРА
 PROGMEM const char Text_string146[] ="DN RELOEP@RSP[:";                        // ДО ТЕМПЕРАТУРЫ:
 PROGMEM const char Text_string147[] ="SLEM&X@R& _PJNQR&";                      // УМЕНЬШАТЬ ЯРКОСТЬ
 PROGMEM const char Text_string148[] ="W@Q[ / OSQRNI ]JP@M";                    // ЧАСЫ / ПУСТОЙ ЭКРАН
 PROGMEM const char Text_string149[] ="@JRHB@VH_";                              // АКТИВАЦИЯ 
 PROGMEM const char Text_string150[] ="ONQKE:";                                 //   ПОСЛЕ
# ifdef freshwater
 PROGMEM const char Text_string151[] ="@]P@VH_";                               // АЭРАЦИЯ  (меню ручное управление)    
 PROGMEM const char Text_string152[] ="QN2";                                   // СО2 
 PROGMEM const char Text_string153[] ="THK&RP";                                // ФИЛЬТР
 PROGMEM const char Text_string154[] ="ST K@LO@";                              // УФ ЛАМПА   
 PROGMEM const char Text_string155[] ="DNKHB";                                 // ДОЛИВ
#endif
# ifdef seawater
 PROGMEM const char Text_string151[] ="R@ILEP-1";                               // Таймер 1  (меню ручное управление)    
 PROGMEM const char Text_string152[] ="R@ILEP-2";                               // Таймер 2  
 PROGMEM const char Text_string153[] ="R@ILEP-3";                               // Таймер 3 
 PROGMEM const char Text_string154[] ="R@ILEP-4";                               // Таймер 4    
 PROGMEM const char Text_string155[] ="R@ILEP-5";                               // Таймер 5 
#endif
 PROGMEM const char Text_string156[] ="AUTO";                      // АВТО
 PROGMEM const char Text_string157[] ="RELOEP@RSP@ BND[ 3@ QSRJH"; // ТЕМПЕРАТУРА ВОДЫ ЗА СУТКИ  
 PROGMEM const char Text_string158[] ="M`w`kn dbhf.";              // Начало движ.
 PROGMEM const char Text_string159[] ="Jnmev dbhf.";               // Конец движ.
 PROGMEM const char Text_string160[] ="P`anw`$ gnm`";              // надпись "рабочая зона"
 PROGMEM const char Text_string161[] ="Qjnpnqr| onbnpnr`";         // скорость поворота
 PROGMEM const char Text_string162[] ="Scnk onbnpnr`";             // шаг(угол) поворота 
 PROGMEM const char Text_string163[] ="O`sg` lefds onbnpnr`lh";    // Пауза между поворотами
 PROGMEM const char Text_string164[] ="Onknfemhe oph b{jk.";        // Положение при выкл.
 PROGMEM const char Text_string165[] ="M`qrpnijh bjk/b{jk b R@ILEPE 3"; // настройки вкл/выкл в таймере 3
 PROGMEM const char Text_string166[] ="(24HR)";                    // (24HR)
 PROGMEM const char Text_string167[] ="SQR@MNBJ@ 3BSJNBNI RPEBNCH";// УСТАНОВКА ЗВУКОВОЙ ТРЕВОГИ  
 PROGMEM const char Text_string168[] ="(Ndmnbpelemm`$ p`anr` onlo)"; //(Одновременная работа помп)  
 PROGMEM const char Text_string169[] ="R@ILEP[";                    // Таймеры
 PROGMEM const char Text_string170[] ="Changed!";                  // Изменено ! (меню часов)
 PROGMEM const char Text_string171[] ="B{j";                       // Вык (выключено)
 PROGMEM const char Text_string172[] ="Bjk";                       // Вкл (включено)
 PROGMEM const char Text_string173[] =""; //----------------------------------------------------
 PROGMEM const char Text_string174[] ="3`ohq| m`qrpnej m` tkex j`prs "; // Запись настроек на флеш карту
 PROGMEM const char Text_string175[] ="M`qrpnijh sqoexmn qnup`mem{.";   // Настройки успешно сохранены 
 PROGMEM const char Text_string176[] ="Nrjk~w.";                   // Отключ.  Отключить
 PROGMEM const char Text_string177[] ="D.Bnd{";                    // Д.Воды (Датчик в аквариуме)
 PROGMEM const char Text_string178[] ="D.P`d:1";                   // Д.Рад:1 (Радиатор Датчик 1)
 PROGMEM const char Text_string179[] ="D.P`d:2";                   // Д.Рад:2 (Радиатор Датчик 2)
 PROGMEM const char Text_string180[] ="ONJNPLHR& QEIW@Q";          // ПОКОРМИТЬ СЕЙЧАС
 PROGMEM const char Text_string181[] ="JNPLKEMHE";                 // КОРМЛЕНИЕ
 PROGMEM const char Text_string182[] ="BPEL_";                     // ВРЕМЯ
 PROGMEM const char Text_string183[] ="ME SQR@MNBKEMN";            // НЕ УСТАНОВЛЕНО
 PROGMEM const char Text_string184[] ="B[JK.";                     // ВЫКЛ.

const PROGMEM char* const PROGMEM Text_table[]  ={
  Text_string0, Text_string1, Text_string2, Text_string3, Text_string4, Text_string5, Text_string6, Text_string7,
  Text_string8, Text_string9, Text_string10, Text_string11, Text_string12, Text_string13, Text_string14, Text_string15,
  Text_string16, Text_string17, Text_string18, Text_string19, Text_string20, Text_string21, Text_string22, Text_string23,
  Text_string24, Text_string25, Text_string26, Text_string27, Text_string28, Text_string29, Text_string30, Text_string31,
  Text_string32, Text_string33, Text_string34, Text_string35, Text_string36, Text_string37, Text_string38, Text_string39,
  Text_string40, Text_string41, Text_string42, Text_string43, Text_string44, Text_string45, Text_string46, Text_string47,
  Text_string48, Text_string49, Text_string50,  Text_string51, Text_string52, Text_string53, Text_string54, Text_string55,
  Text_string56, Text_string57, Text_string58, Text_string59, Text_string60, Text_string61, Text_string62, Text_string63,
  Text_string64, Text_string65, Text_string66, Text_string67, Text_string68, Text_string69, Text_string70, Text_string71,
  Text_string72, Text_string73, Text_string74, Text_string75, Text_string76, Text_string77, Text_string78, Text_string79,
  Text_string80, Text_string81, Text_string82, Text_string83, Text_string84, Text_string85, Text_string86, Text_string87, 
  Text_string88, Text_string89, Text_string90, Text_string91, Text_string92, Text_string93, Text_string94, Text_string95,
  Text_string96,Text_string97,Text_string98,Text_string99,Text_string100, Text_string101, Text_string102, Text_string103,
  Text_string104, Text_string105, Text_string106, Text_string107, Text_string108, Text_string109, Text_string110,
  Text_string111, Text_string112, Text_string113, Text_string114, Text_string115, Text_string116, Text_string117,
  Text_string118, Text_string119, Text_string120, Text_string121, Text_string122, Text_string123, Text_string124, 
  Text_string125, Text_string126, Text_string127, Text_string128, Text_string129, Text_string130, Text_string131,  
  Text_string132, Text_string133, Text_string134, Text_string135, Text_string136, Text_string137, Text_string138,
  Text_string139, Text_string140, Text_string141, Text_string142, Text_string143, Text_string144, Text_string145,
  Text_string146, Text_string147, Text_string148, Text_string149, Text_string150, Text_string151, Text_string152,
  Text_string153, Text_string154, Text_string155, Text_string156, Text_string157, Text_string158, Text_string159,
  Text_string160, Text_string161, Text_string162, Text_string163, Text_string164, Text_string165, Text_string166, 
  Text_string167, Text_string168, Text_string169, Text_string170, Text_string171, Text_string172, Text_string173, 
  Text_string174, Text_string175, Text_string176, Text_string177, Text_string178, Text_string179, Text_string180,
  Text_string181, Text_string182, Text_string183, Text_string184, };   

char buffer[40];
int PrintStringIndex;
char* print_text[]  = { 
"<< LEM^",           // print_text[0]  МЕНЮ
"NRLEM@",            // print_text[1]  ОТМЕНА
"<< M@G@D",          // print_text[2]  НАЗАД
"QNUP@MHR&",         // print_text[3]  СОХРАНИТЬ
"STOP",              // print_text[4]  СТОП
"DPSCNI VBER",       // print_text[5]  ДРУГОЙ ЦВЕТ
"HGLEMHR&",          // print_text[6]  ИЗМЕНИТЬ
"MON",               // print_text[7]  ПОН.
"TUE",               // print_text[8]  ВТО. 
"WED",               // print_text[9]  СРД.
"THU",               // print_text[10] ЧТВ.
"FRI",               // print_text[11] ПТН.
"SAT",               // print_text[12] СУБ.
"SUN",               // print_text[13] ВСК.
"AUTO FEEDER",       // print_text[14]
"RAND",              // print_text[15]
"STORM",             // print_text[16]
"BOEPED>>",          // print_text[17]
"<<",                // print_text[18]
">>",                // print_text[19]
"WW",                // print_text[20]  (W) white  10 аква
"CW",                // print_text[21]  (B) blue   11 аква
"RB",                // print_text[22]  (RB)royal   5 аква
"R",                 // print_text[23]  (R) red     8 аква
"UV",                // print_text[24]  ultra       9 аква
"Or",                // print_text[25]  oLed
"Gr",                // print_text[26]  green
"+",                 // print_text[27]
"-",                 // print_text[28]
"<< QEJRNP",         // print_text[29]  << Сектор
"QEJRNP>>",          // print_text[30]  Сектор >>
"Preset 1",          // print_text[31]  Пресет 1  OPEQER 1
"Preset 2",          // print_text[32]  Пресет 2
"Preset 3",          // print_text[33]  Пресет 3
"Preset 4",          // print_text[34]  Пресет 4
"YES",               // print_text[35]  ДА
"NO",                // print_text[36]  НЕТ
"10",                // print_text[37]  10
"20",                // print_text[38]  20
"30",                // print_text[39]  30
"40",                // print_text[40]  40
"50",                // print_text[41]  50
"60",                // print_text[42]  60
"70",                // print_text[43]  70
"80" ,               // print_text[44]  80
"90",                // print_text[45]  90
"100",               // print_text[46]  100
"1",                 // print_text[47]  1  "@]P@VH_"
"2",                 // print_text[48]  2 "OND@W@ QN"
"3",                 // print_text[49]  3 "THK&RP"
"4",                 // print_text[50]  4 "ST K@LO@"
"5",                 // print_text[51]  5 "QKHB"
"O-1",               // print_text[52]
"O-2",               // print_text[53]
"O-3",               // print_text[54]
"O-4",               // print_text[55]
":",                 // print_text[56]  двоеточие
"C",                 // print_text[57]
"-10s",              // print_text[58]
"+10s",              // print_text[59]
"-1",                // print_text[60]
"+5",                // print_text[61]
# ifdef freshwater
"@]PN",              // print_text[62]                         Таймеры
"QN2",               // print_text[63]
"THK&RP",            // print_text[64]
"ST",                // print_text[65]
"DNKHB",             // print_text[66]
#endif
# ifdef seawater
"-1-",               // print_text[62]                         Таймеры
"-2-",               // print_text[63]
"-3-",               // print_text[64]
"-4-",               // print_text[65]
"-5-",               // print_text[66]
#endif
"BPEL_ BJK^WEMH_",   // print_text[67] время вкл.
"BPEL_ B[JK^WEMH_",  // print_text[68] время выкл.
"22",                // print_text[69]
"23",                // print_text[70] 
"24",                // print_text[71] 
"25",                // print_text[72] 
"26",                // print_text[73] 
"27",                // print_text[74]
"28",                // print_text[75]
"29",                // print_text[76]
"%",                 // print_text[77]  знак процентов
"31",                // print_text[78]
"6",                 // print_text[79]
"8",                 // print_text[80]
"12",                // print_text[81]
"14",                // print_text[82]
"16",                // print_text[83]
"18",                // print_text[84]
"E",                 // print_text[85]
"D",                 // print_text[86]
"D@RW1",             // print_text[87]  датчик температуры радиатора 1   ДАТЧ1
"D@RW2",             // print_text[88]  датчик температуры радиатора 2   ДАТЧ2
"Ksm`",              // print_text[89]  Луна 
"32",                // print_text[90]  32
"34",                // print_text[91]  34
"36",                // print_text[92]  36
"38",                // print_text[93]  38
"42",                // print_text[94]  42
"B[JK",              // print_text[95]   ВЫКЛ
"START",             // print_text[96]   СТАРТ
"R.AEK[I  :",        // print_text[97]   БЕЛЫЙ   канал    (главный экран)
"U.AEK[I  :",        // print_text[98]   ГОЛУБОЙ   канал
"QHMHI    :",        // print_text[99]   СИНИЙ   канал
"JP@QM[I  :",        // print_text[100]  КРАСНЫЙ   канал
"SK&RP@   :",        // print_text[101]  УЛЬТРА   канал
"NP@MFEB[I:",        // print_text[102]  ОРАНЖЕВЫЙ  канал
"3EKEM[I  :",        // print_text[103]  ЗЕЛЕНЫЙ   канал 
"R.AEK[I:",          // print_text[104]  БЕЛЫЙ (меню тест)
"U.AEK[I:",          // print_text[105]  ГОЛУБОЙ
"QHMHI  :",          // print_text[106]  СИНИЙ
"JP@QM[I:",          // print_text[107]  КРАСНЫЙ
"SK&RP@ :",          // print_text[108]  УЛЬТРА
"NP@MF. :",          // print_text[109]  ОРАНЖ.
"3EKEM[I:",          // print_text[110]  ЗЕЛЕНЫЙ
"   ",               // print_text[111]  (три пробела)
"Mnbnksmhe",         // print_text[112]  Новолуние 
" P`qrsy`_",         // print_text[113]  Растущая 
"1 Werbepr|",        // print_text[114]  1 Четверть
" P`qrsy`_",         // print_text[115]  Растущая
"Onkmnksmhe",        // print_text[116]  Полнолуние
" Sa{b`~y`$",        // print_text[117]  Убывающая
"3 Werbepr|",        // print_text[118]  3 Четверть
"@JB@PHSL",          // print_text[119]  АКВАРИУМ
"P@DH@RNP",          // print_text[120]  РАДИАТОР
"CP@THJ RELOEP@RSP", // print_text[121]  ГРАФИК ТЕМПЕРАТУР (menu)
"SQR@MNBJ@",         // print_text[122]   УСТАНОВКА    (главное меню)
"BPELEMH",           // print_text[123]   ВРЕМЕНИ 
"RELOEP@RSP[",       // print_text[124]   ТЕМПЕРАТУРЫ
"THK&RP",            // print_text[125]   ФИЛЬТР                        
"DN3@RNP",           // print_text[126]   ДОЗАТОР                            
"DNA@BNJ",           // print_text[127]   ДОБАВОК                       
"NQMNBM[E",          // print_text[128]   ОСНОВНЫЕ
"M@QRPNIJH",         // print_text[129]   НАСТРОЙКИ
"@BRN",              // print_text[130]   АВТО
"REQR",              // print_text[131]   ТЕСТ
"CP@THJ",            // print_text[132]   ГРАФИК
"J@M@KNB",           // print_text[133]   КАНАЛОВ
"_PJNQRH",           // print_text[134]   ЯРКОСТИ 
"QEJRNP",            // print_text[135]   CЕКТОР
"BPEL_",             // print_text[136]   ВРЕМЯ
"3@OHQ&",            // print_text[137]   ЗАПИСЬ
"OPEQERNB",          // print_text[138]   ПРЕСЕТОВ
"QSRNWM[E",          // print_text[139]   СУТОЧНЫЕ 
"R@ILEP[",           // print_text[140]   ТАЙМЕРЫ                         
"SCKEJHQKNR[",       // print_text[141]   УГЛЕКИСЛОТЫ                        
"%",                 // print_text[142]   % (проценты)
"p`qqber",           // print_text[143]   рассвет
"g`j`r",             // print_text[144]   закат
"L`jq",              // print_text[145]   Макс
"Rejsy",             // print_text[146]   Текущ
"BACKUP.TXT",        // print_text[147]   BACKUP.TXT
"B REJQRNBNL T@IKE", // print_text[148]   В ТЕКСТОВОМ ФАЙЛЕ
"BQE M@QRPNIJH M@UND_RQ_ M@ TKEX J@PRE", // print_text[149]  ВСЕ НАСТРОЙКИ НАХОДЯТСЯ НА ФЛЕШ КАРТЕ
"M`qrpnijh",         // print_text[150]   Настройки
"RELOEP@RSP@ P@DH@RNPNB 3@ QSRJH",       // print_text[151]  температура радиаторов за сутки
"Alarm",             // print_text[152]   Alarm (в меню установки температуры радиатора)
"  ",                // print_text[153]  (два пробела)  (Cегодня Qecndm$)
"TEST",              // print_text[154]   TEST
"|",                 // print_text[155]   // | /
"ONMEDEK&MHJ",       // print_text[156]   Понедельник (бар даты)
"BRNPMHJ",           // print_text[157]   Вторник
"QPED@",             // print_text[158]   Среда
"WERBEPC",           // print_text[159]   Четверг
"O_RMHV@",           // print_text[160]   Пятница
"QSAANR@",           // print_text[161]   Суббота
"BNQJPEQEM&E",       // print_text[162]   Воскресенье
"_MB@P_",            // print_text[163]   Января
"TEBP@K_",           // print_text[164]   Февраля
"L@PR@",             // print_text[165]   Марта
"@OPEK_",            // print_text[166]   Апреля
"L@_",               // print_text[167]   Мая
"H^M_",              // print_text[168]   Июня
"H^K_",              // print_text[169]   Июля
"@BCSQR@",           // print_text[170]   Августа
"QEMR_AP_",          // print_text[171]   Сентября
"NJR_AP_",           // print_text[172]   Октября
"MN_AP_",            // print_text[173]   Ноября
"DEJ@AP_",           // print_text[174]   Декабря
"PLEASE WAIT",       // print_text[175]   Пожалуйста подождите
"0...100 %",         // print_text[176]  (0--100) В меню настройки луны
"Moon",              // print_text[177]  Moon
"GRN",               // print_text[178]  Green (GRN)
"ORG",               // print_text[179]  Orange(ORG
"UVL",               // print_text[180]  UV    (UVL)
"RED",               // print_text[181]  Red   (RED)
"RBL",               // print_text[182]  Royal (RBL)
"CWT",               // print_text[183]  Blue  (BLU)
"WWT",               // print_text[184]  White (WHT)
"B[ UNRHRE QNUP@MHR; M@QRPNIJH,", // print_text[185]  ВЫ ХОТИТЕ СОХРАНИТЬ НАСТРОЙКИ,
"OEPED B[UNDNL HG LEM^ ?",        // print_text[186]      ПЕРЕД ВЫХОДОМ ИЗ МЕНЮ ?
"0",                 // print_text[187]  0 (один ноль)
"00",                // print_text[188]  00 (два нуля)
"000",               // print_text[189]  000 (три нуля)
"TESTING",           // print_text[190]  ТЕСТ
"ONDNFDHRE, hder nwhqrj` o`l$rh",   // print_text[191] ПОДОЖДИТЕ, идет очистка памяти 
"3@CPS3HR&",         // print_text[192]  ЗАГРУЗИТЬ
"QAPNQ",             // print_text[193]  СБРОС (RESET)
"AEJ@O",             // print_text[194]  БЕКАП(BACKUP)
"ONHQJ D@RW",        // print_text[195]  ПОИСК ДАТЧ. 
"Bnqqr`mnbkemhe m`qrpnej g`bepxemn.", // print_text[196] Восстановление настроек завершено
"Onf`ksiqr` ondnfdhre ...",           // print_text[197] Пожалуйста подождите....
"Wremhe t`ik` m`qrpnej m` tkex j`pre",// print_text[198] Чтение файла настроек на флеш карте
"Nxhaj` oph nrjp{rhh t`ik` M`qrpnej", // print_text[199] Ошибка при открытии файла Настроек
"QAPNQHR; M@QRPNIJH ?",               // print_text[200] СБРОСИТЬ НАСТРОЙКИ ?
"Hmhvh`khg`vh$ sqoexmn g`bepxem`",    // print_text[201] Инициализация успешно завершена
"Nxhaj` ! hmhvh`khg`vhh tkex j`pr{",  // print_text[202] Ошибка ! инициализации флеш карты
"Hmhvh`khg`vh$ tkex j`pr{..",         // print_text[203] Инициализация Флеш карты..
"ONHQJ D@RWHJNB",                     // print_text[204] ПОИСК ДАТЧИКОВ   ОБНОВИТЬ (NAMNBHR;)
"Ondjk~wem J",                        // print_text[205] Подключен К
"D`rwhj",                             // print_text[206] Датчик
"D`rwhjnb",                           // print_text[207] Датчиков
"M`idemn:",                           // print_text[208] Найдено:
"QAPNQHR& M@QRPNIJH",                 // print_text[209] СБРОСИТЬ НАСТРОЙКИ (в меню) 
"QNUP. M@QRP. M@ J@PRS",              // print_text[210] СОХР. НАСТР. НА КАРТУ
"ONHQJ D@RWHJNB RELOEP.",             // print_text[211] ПОИСК ДАТЧИКОВ ТЕМПЕР. 
"ON",                                 // print_text[212] ON
"OFF",                                // print_text[213] OFF
"RELOEP@RSP@ BND[",                   // print_text[214] Температура воды
"M@QRP.",                             // print_text[215] НАСТР.
"JNPLKEMH_",                          // print_text[216] КОРМЛЕНИЯ
};

int dispScreen=0; 
/* 
0-Main Screen, 
1-Menu, 
2-Clock Setup, 
3-Temp Control, 
4-LED Test Options (sector),
5-Test LED Arrays, 
6-Test Individual,
7-Choose LED Color,
8-View Selected Color, Values, 
9-Selected Color Array Values, 
10-Wavemaker, 
11-Wavemaker Settings, 
12-Servo, 
13-Led Preset, 
14-General Set.(Page1), 
15-General Set.(Page2)
16-Heatsink FAN Temp, 
17-Led Dim (temp), 
18-Set Screensaver, 
19-Timer,
20-T1, 21-T2, 22-T3, 23-T4, 24-T5,
25-Manual Timer control, 
26-Moon Seting, 
27-Log Temp Radiator,
28-Log Temp Water, 
29-Test Led Graph
30-Dallas auto-detect, 
31-Backup Setting, 
32-Dimm, 
33-LCDbright,
34-       , 
35-Sound Alarm
36-General Set.(Page3), 
37-
   */                                  
int x, y;                            // touch coordinates

long previousMillisLED = 0;          // Used in the Test LED Array Function
long previousMillisWave = 0;         // Used in the WaveMaker Function feed_output()
long previousMillisFive = 0;          // Used in the Main Loop (Checks Time,Temp,LEDs,Screen)
long previousMillisTen = 0;
long previousMillisLEDoff = 0;       // Used in Shut LEDs Off if Temp too high
long previousMillisAlarm = 0;        // Used in the Alarm
long previousMillisOne = 0;          // Used in the Main Loop (LEDs level) для уровней каналов в 11бит режиме

long previousMillisLewel = 0;        // check led lewel
long previousMillisAlarm2 = 0;
long previousMillisGraph = 0;

// вместо Delay
unsigned long currentTime;
unsigned long loopTime;
                                   // your feed seconds are doubled)    
long previousMillisCt = 0;           // stores the last update for the Countdown Timer
long intervalCt = 1000;              // One Second Interval for Countdown Timer
int countDown = 5*60 + 0;            // Countdown for 5 minutes and zero seconds
int MIN_O = 5;                       // Start the Time at 5 (for looks only)
int SEC_T = 0;
int SEC_O = 0;

int LedChangTime = 0;                // LED change page, time and values
byte oldLCT ;
int min_cnt;                         // Used to determine the place in the color arrays
int sec_cnt;
byte temp_sector;                    // used in led test arrays, min_cnt/15 

boolean LEDtestTick = false;         // for testing leds and speed up clock
//boolean firstTouch  = false;         // #oleg выход из скринсейва
boolean SliderSwitch  = false;       // slider

//byte LedShannelStatusByte =255;      // ON=1/OFF=0 channel, one bit - one channel
 byte LedShannelStatusByte =2000;  // 11 bit ON=1/OFF=0 channel, one bit - one channel
byte GlobalStatus2Byte=0x0;          // byte Led preset status

// bit 0 - preset 1 (ON=1/OFF), bit 1 - preset 2, bit 2 - preset 3, bit 3 - preset 4 
byte  AddressShift;
byte GlobalStatus1Byte;		     // bit 0 for Y/N button in reset default function
byte setTimeFormat = 0; 

int whiteLed, blueLed, rblueLed, redLed, uvLed, orLed, grLed;     // предыдущие значения яркости LED 
int wled_out, bled_out, rbled_out, rled_out, uvled_out, oLed_out, moonled_out, gled_out; // текущие значения яркости LED                                            
int wcol_out, bcol_out, rbcol_out, rcol_out, uvcol_out, ocol_out, grcol_out, mooncol_out; // Current LED output values for Test Ind. Color LEDs                                                           
int w_out, b_out, rb_out, r_out, uv_out, o_out, gr_out, moon_out;  // Current LED output values for Test Ind. Color LEDs    


byte COLOR=0, WHITE=1, BLUE=2, ROYAL=3, RED=4, ULTRA=5, ORANGE=6, GREEN=7, MOON=8; // View / Change Color LED Values
    
boolean colorLEDtest = false;         // To test individual color LEDs
//int sbR, sbG, sbB, sbX1, sbX2;        // Used in the Slider Bars
byte sbR, sbG, sbB; 
int sbX1, sbX2;                      // Used in the Slider Bars
//int tSlide=0;
int yWHT=0, yBLU=0, yRBL=0, yRED=0, yUVL=0, ySMP=0, yLUN=0, yGRN=0;

boolean DrawStaticElement = false;    // Allows selection of changing LED values

int yStore=0, k=0, tSlide=0;    
boolean TopRows = false;              // Allows selection of changing LED values
    
int x1Bar=0, x2Bar=0, xValue=0, yValue=0, LEDlevel, yBar; // Used in LED Output Chart on Test Ind. LEDs Screen
int setmode1=0, setmode2=0, setmode3=0;    

int timer;  // переменная таймеров
// Активация настроенных таймеров освещения 0=NO  1=YES
int prog1,prog2,prog3,prog4,prog5; // таймеры 1-5

// Время включения освещения
int on1,on2,on3,on4,on5;       // таймеры 1-5

// Время выключения освещения 
int off1,off2,off3,off4,off5;  // таймеры 1-5

//Временная переменная Время включения освещения
int tempon1,tempon2,tempon3,tempon4,tempon5;      // таймеры 1-5

// Время выключения освещения 
int tempoff1,tempoff2,tempoff3,tempoff4,tempoff5; // таймеры 1-5

// Статус вкл/выкл таймеров (ручное управление)
byte timer1Status;   // Статус таймера 1 
byte timer2Status;   // Статус таймера 2 
byte timer3Status;   // Статус таймера 3 
byte timer4Status;   // Статус таймера 4 
byte timer5Status;   // Статус таймера 5 

byte tmpLCDbright=40;    // подсветка экрана
byte LCDbright=40;  // яркость экрана по умолчанию

long previousMillisA = 0;  // Timer stuff
long previousMillisB = 0;
long previousMillisC = 0;

int DelayPeriodA = 200;    // Как долго ждать (в миллисекундах) между изменением яркости LED (скорость нарастания)
int DelayPeriodB = 100;
int DelayPeriodC = 50;

int DegreesCounterA = 30;  // Используется для прокрутки сигнала синус / 300
int DegreesCounterB = 30;
int DegreesCounterC = 30;

float RadiansCounterA = 0; // Radians version of DegreesCounter
float RadiansCounterB = 0;
float RadiansCounterC = 0;

float Sin_of_Rad_CtrA = 0; // Sine feed calculation stuff
float Sin_of_Rad_CtrB = 0;
float Sin_of_Rad_CtrC = 0;

int BrightnessA = 0;       // Значение яркости, LED
int BrightnessB = 0;
int BrightnessC = 0;

int StepSize = 1;  // шаг нарастания яркости

// Графики
byte StopTime;      
byte StartTime;     
int TopSldY ;	    // top slider
int BotSldY ;       // bot slider
boolean LedShannelFlag_on_off;   // flag led on/off channel status
int  EndScale;
byte LightDay;
boolean W = true;   // true - включено по умолчанию, false - выключено
boolean RB = true;
boolean B = true;
boolean R = true;
boolean UV = true; 
boolean SU = true; 
boolean OR = true; 

boolean F1 = true;  // датчик 1 на радиаторе  
boolean F2 = true;  // датчик 2
float Th;
int TimeW = 1;

// WHITE Dimming Values 
byte wled[96] = { // канал освещения White
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1 время суток
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 3,           // 8 - 9
  5, 10, 15, 20, 20, 25, 25, 30,    // 10 - 11
  35, 40, 45, 50, 50, 50, 55, 60,   // 12 - 13
  60, 60, 60, 55, 50, 50, 50, 47,   // 14 - 15
  45, 43, 40, 40, 40, 35, 33, 30,   // 16 - 17
  30, 25, 22, 15, 10, 5, 0, 0,      // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23

// BLUE Dimming Values
byte bled[96] = { // канал освещения Blue 
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,           // 8 - 9
  1, 2, 3, 4, 5, 8, 10, 15,         // 10 - 11
  25, 30, 35, 40, 45, 50, 55, 55,   // 12 - 13
  55, 55, 55, 50, 50, 50, 45, 40,   // 14 - 15 
  40, 35, 30, 30, 25, 25, 25, 20,   // 16 - 17
  20, 20, 20, 20, 17, 10, 5, 0,     // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23
 
// ROYAL BLUE Dimming Values
byte rbled[96] = { // канал освещения Royal Blue 
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 5, 8, 10,          // 8 - 9
  15, 15, 20, 20, 25, 30, 35, 40,   // 10 - 11
  45, 50, 50, 55, 60, 65, 70, 80,   // 12 - 13
  80, 80, 80, 75, 75, 75, 70, 70,   // 14 - 15
  65, 63, 57, 55, 55, 50, 47, 45,   // 16 - 17
  43, 40, 35, 35, 30, 30, 25, 20,   // 18 - 19
  15, 10, 5, 0, 0, 0, 0, 0,         // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0  };        // 22 - 23
 
// RED Dimming Values
byte rled[96] = { // канал освещения Red
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,           // 8 - 9
  0, 0, 0, 0, 0, 0, 0, 0,           // 10 - 11
  10, 15, 20, 30, 40, 50, 60, 70,   // 12 - 13
  80, 70, 60, 50, 40, 30, 20, 10,   // 14 - 15
  0, 0, 0, 0, 0, 0, 0, 0,           // 16 - 17
  0, 0, 0, 0, 0, 0, 0, 0,           // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23
  
// ULTRA VIOLET (UV) Dimming Values
byte uvled[96] = { // канал освещения Фиолетовый
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,           // 8 - 9
  1, 5, 20, 30, 40, 50, 60, 70,     // 10 - 11
  80, 90, 90, 90, 90, 90, 90, 90,   // 12 - 13
  90, 90, 90, 90, 80, 80, 70, 60,   // 14 - 15
  60, 60, 50, 50, 50, 50, 40, 40,   // 16 - 17
  40, 35, 30, 20, 15, 10, 5, 0,     // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23
  
// ORANGE Dimming Values
byte oLed[96] = {  // канал освещения Оранжевый
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,           // 8 - 9
  0, 0, 0, 0, 0, 0, 0, 0,           // 10 - 11
  5, 8, 10, 10, 15, 20, 25, 25,     // 12 - 13
  30, 35, 40, 40, 40, 35, 30, 30,   // 14 - 15
  25, 20, 20, 10, 5, 1, 0, 0,       // 16 - 17
  0, 0, 0, 0, 0, 0, 0, 0,           // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23
  
// GREEN Dimming Values  
byte gled[96] = {  // канал освещения Зеленый 
  0, 0, 0, 0, 0, 0, 0, 0,           // 0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,           // 2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,           // 4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,           // 6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,           // 8 - 9
  0, 0, 0, 0, 0, 0, 0, 0,           // 10 - 11
  0, 5, 10, 15, 18, 20, 25, 25,     // 12 - 13
  20, 20, 18, 15, 12, 10, 8, 5,     // 14 - 15
  0, 0, 0, 0, 0, 0, 0, 0,           // 16 - 17
  0, 0, 0, 0, 0, 0, 0, 0,           // 18 - 19
  0, 0, 0, 0, 0, 0, 0, 0,           // 20 - 21
  0, 0, 0, 0, 0, 0, 0, 0 };         // 22 - 23
 
byte tled[96];     // временный массив для хранения значений led
word tFA[48];      // временный массив для температуры в аквариуме
word media[48] ;
word tF1[48] ;     // временный массив для температуры (датчик 1)
word tF2[48] ;     // временный массив для температуры (датчик 2)

boolean refreshAll,  PlsMnsPress, ButtonDisable;
int XShift, YShift, ButDist, Min, Max, Step, PlsMnsCount;


    
//------------------ Виртуальные кнопки в главном Меню --------------------------
const int tanD[]={5, 21, 105, 57};         // Время и Дата
const int temC[]={5, 65, 105, 102};        // Температура Воды в Аквариуме
const int feed[]={5, 110, 105, 145};       // Установка времени кормления 10, 110, 155, 145
const int gSet[]={5, 153, 105, 189};       // Основные настройки  
const int tesT[]={110, 21, 210, 57};       // Тест установок яркости, за сутки (авто-тест обычный)
const int ledChM[]={110, 65, 210, 102};    // Настройка каналов по цветам
const int Sector[]={110, 110, 210, 145};   // Настройка каналов по секторам времени
const int timday[]={110, 153, 210, 189};   // Суточный Tаймер 165, 153, 310, 189
const int tesT2[]={215, 21, 315, 57};      // Тест установок яркости, за сутки (тест с графиками каналов)
const int Preset[]={215, 65, 315, 102};    // Запись присетов 
const int PHset[]={215, 110, 315, 145};    // Настройка PH
const int Pumpset[]={215, 153, 315, 189};  // Настройка дозирующих помп
const int logW[]={16, 208, 91, 223};       // график температуры воды за сутки
const int logH[]={110, 208, 185, 223};     // график температуры радиатора за сутки

//--------------- Виртуальные кнопки в Окне настройки времени и даты -----------------
const int houU[]={107, 22, 137, 44};       // Hour up
const int houD[]={107, 76, 137, 98};       // Hour down
const int minU[]={177, 22, 207, 44};       // Min up
const int minD[]={177, 76, 207, 98};       // Min down
const int dayU[]={107, 112, 137, 134};     // Day up
const int dayD[]={107, 166, 137, 188};     // Day down
const int monU[]={177, 112, 207, 134};     // Month up
const int monD[]={177, 166, 207, 188};     // Month down
const int yeaU[]={265, 112, 295, 134};     // Year up
const int yeaD[]={265, 166, 295, 188};     // Year down

//----------------- Виртуальные кнопки в Окне контроля температуры ------------------
const int temP[]={215, 49, 255, 79};           // Температура плюс 
const int temM[]={65, 49, 105, 79};            // Температура минус 
const int offP[]={215, 99, 255, 129};          // Offset plus
const int offM[]={65, 99, 105, 129};           // Offset minus 
const int SoundAlarmTp[]={215, 149, 255, 179}; // Sound Alarm Temp + 
const int SoundAlarmTm[]={65, 149, 105, 179};  // Sound Alarm Temp -  
const int weatH[]={5, 80, 39, 160};            // Лог температуры  

/**************************** LED TESTING MENU BUTTONS *******************************/
const int tstLA[]={40, 59, 280, 99};       // "Test LED Array Output" settings
const int cntIL[]={40, 109, 280, 149};     // "Control Individual Leds" settings

//---------------------- Виртуальные кнопки в меню тест LED ----------------------------
const int stsT[]={110, 105, 200, 175};     // Start/stop
const int tenM[]={20, 120, 90, 160};       // -10s
const int tenP[]={220, 120, 290, 160};     // +10s

/************************* CHANGE LED VALUES MENU BUTTONS ****************************/
const int btCIL[]={5, 188, 90, 220};       // Back to Change Individual LEDs Screen
const int ledChV[]={110, 200, 210, 220};   // LED Change Values
const int eeprom[]={215, 200, 315, 220};   // Save to EEPROM (Right Button)

//---------- кнопки установки яркости луны --------
const int MINiM[]={12, 142, 44, 172};      // MinI minus
const int MINiP[]={114, 142, 146, 172};    // MinI plus
const int MAXiM[]={174, 142, 206, 172};    // MaxI minus 
const int MAXiP[]={276, 142, 308, 172};    // MaxI plus

/************************ кнопки установки времени кормления ************************/
const int houP[]= {110, 38, 135, 63};       //hour up
const int minP[]= {180, 38, 205, 63};       //min up
const int ampmP[]= {265, 38, 290, 63};      //AM/PM up
const int houM[]= {110, 89, 135, 114};      //hour down
const int minM[]= {180, 89, 205, 114};      //min down
const int ampmM[]= {265, 89, 290, 114};     //AM/PM down

/*********************** кнопки дозатора удобрений***********************************/
const int dos1b[]={5, 31, 100, 56};         // дозатор1
const int dos2b[]={5, 75, 100, 100};         // дозатор2
const int dos3b[]={5, 120, 100, 145};       // дозатор3
const int dos4b[]={5, 163, 100, 188};       // дозатор4 
const int dosT1[]={10, 21, 210, 51};       // время дозатора 1
const int dosT2[]={10, 65, 210, 95};       // время дозатора 2
const int dosT3[]={10, 110, 210, 140};     // время дозатора 3
const int dosT4[]={10, 153, 210, 183};     // время дозатора 4
const int dosval1[]={230, 26, 315, 56};     // объем дозатора 1
const int dosval2[]={230, 70, 315, 100};     // объем дозатора 2
const int dosval3[]={230, 115, 315, 145};   // объем дозатора 3
const int dosval4[]={230, 158, 315, 188};   // объем дозатора 4
const int dosCal[]={16, 208, 91, 223};      // калибровка

const int valUP[]=  {170, 44, 178, 75};       // объем вверх
const int valDOWN[]={265, 44, 313, 75};     // объем вниз
const int numUP[]=  {170, 79, 178, 110};      // количество доз вверх
const int numDOWN[]={265, 79, 313, 110};    // количество доз вниз
const int intUP[]=  {170, 114, 178, 145};     // интервал часы вверх
const int intDOWN[]={265, 114, 313, 145};   // интервал часы вниз
const int calUP[]=  {170, 155, 178, 181};     // калибровка вверх
const int calDOWN[]={265, 150, 313, 181};   // калибровка вниз

/***************************** MISCELLANEOUS BUTTONS *********************************/
const int backGS[]={4, 200, 78, 223};      // BACK  (маленькая)
const int nextGS[]={83, 200, 157, 223};    // NEXT  (маленькая)
const int prSAVEgs[]={162, 200, 236, 223}; // SAVE  (маленькая)
const int canCgs[]={241, 200, 315, 223};   // CANCEL(маленькая)

const int HoodFanTm[]={80, 53, 115, 83};   // Hood Fan Temp -  температура радиатора
const int HoodFanTp[]={205, 53, 240, 83};  // Hood Fan Temp +
const int SumpFanTm[]={80, 144, 115, 174}; // Hood 2 Fan Temp -
const int SumpFanTp[]={205, 144, 240, 174};// Hood 2 Fan Temp +

const int SalaRm[]={7, 145, 53, 165};  // кнопка установки настройки звуковой тревоги
const int SoundATp[]={215, 117, 257, 151}; // Sound Alarm Temp + 
const int SoundATm[]={63, 117, 105, 151};  // Sound Alarm Temp -

const int back[]={5, 200, 105, 223};       // BACK   (большая)
const int prSAVE[]={110, 200, 210, 223};   // SAVE   (большая)
const int canC[]={215, 200, 315, 223};     // CANCEL (большая)

const int ButMns[]= {0, 0, 40, 30};           // universal +/- button counter, defaul coordinate, 24x24 size

// Графики освещенности LED 
const int Wg[]={37, 0, 67, 15};            // белый канал
const int Bg[]={77, 0, 107, 15};           // голубой канал
const int RBg[]={117, 0, 147, 15};         // канал роял 
const int Rg[]={157, 0, 187, 15};          // красный канал
const int UVg[]={197, 0, 227, 15};         // фиолет канал
const int ORg[]={237, 0, 267, 15};         // оранжевый канал
const int GRg[]={277, 0, 307, 15};         // зеленый канал

// кнопки утановки времени таймеров
const int btonhup[]={20, 44, 68, 75};          // часы вкл вверх
const int btonhdn[]={20, 149, 68, 180};         // часы вкл вниз 
const int btonmup[]={89, 44, 137, 75};         // минуты вкл вверх
const int btonmdn[]={89, 149, 137, 180};       // минуты вкл вниз
const int btofhup[]={185, 44, 233, 75};        // часы выкл вверх
const int btofhdn[]={185, 149, 233, 180};       // часы выкл вниз
const int btofmup[]={255, 44, 303, 75};        // минуты выкл вверх
const int btofmdn[]={255, 149, 303, 180};      // минуты выкл вниз

// начало светового дня, конец светового дня (рассвет, закат)
const int StartDay[]={5, 0, 74, 14};       // рассвет
const int StopDay[]={241, 0, 315, 14};     // закат
const int Psect[]={140, 200, 222, 220};    // Prev SECTOR (предыдущий сектор)
const int Nsect[]={233, 200, 315, 220};    // NEXT SECTOR (следующий сектор)
const int Yes[]={110, 125, 150, 145};      // Yes
const int No[]={170, 125, 210, 145};       // No

// пресеты
const int LedPres1[]={7, 0, 78, 14};        // Led Preset 1 button
const int LedPres2[]={85, 0, 156, 14};      // Led Preset 2 button
const int LedPres3[]={163, 0, 234, 14};     // Led Preset 3 button
const int LedPres4[]={241, 0, 312, 14};     // Led Preset 4 button

// кнопки в Окне ЛОГ температуры радиаторов 1,2
const int Fg1[]={112, 200, 157, 218};       // Датч1                                       
const int Fg2[]={163, 200, 208, 218};       // Датч2 

//------------------------ Подсветка экрана --------------------------------
const int gseB[]= {64, 44};                // brightness bar  

//----------------- кнопки YES  NO в меню главных настроек -----------------
const int ClocBlno[]={255, 51, 305, 71};   // setScreensaverDOWonOff  NO
const int ClocBlyes[]={185, 51, 235, 71};  // setScreensaverDOWonOff  YES
const int DledOn[]={195, 27, 235, 47};     // setDimLEDsOnOff ON
const int DledOff[]={255, 27, 295, 47};    // setDimLEDsOnOff OFF
const int SsavOn[]={195, 101, 235, 121};   // setScreensaverOnOff ON
const int SsavOff[]={255, 101, 295, 121};  // setScreensaverOnOff OFF
const int DimmLOn[]={195, 169, 235, 189};  // DimmL ON
const int DimmLOff[]={255, 169, 295, 189}; // DimmL OFF





void drawUpButtonSlide(int x, int y){ // кнопка в верх, со стрелкой в меню слайдера
   myGLCD.setColor(64, 64, 128);
   myGLCD.fillRoundRect(x, y, x+30, y+22);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+30, y+22);
   myGLCD.setColor(128, 128, 255);
   for (int i=0; i<15; i++) myGLCD.drawLine(x+5+(i/1.5), y+18-i, x+26-(i/1.5), y+18-i); }
  
void drawDownButtonSlide(int x, int y){
   myGLCD.setColor(64, 64, 128);
   myGLCD.fillRoundRect(x, y, x+30, y+22);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+30, y+22);
   myGLCD.setColor(128, 128, 255);
   for (int i=0; i<15; i++) myGLCD.drawLine(x+5+(i/1.5), y+4+i, x+26-(i/1.5), y+4+i);}

void printButton(char* text,int x1,int y1,int x2,int y2, boolean fontsize=false, boolean background=false){
     int stl = strlen(text);
     int fx, fy;
  
  if (background == true){                    // белый фонт, зелёный фон 
      myGLCD.setColor(0, 180, 86);            // цвет зеленый для ON  
      myGLCD.fillRoundRect (x1, y1, x2, y2);
      myGLCD.setColor(255, 255, 255);         // текст белый
      myGLCD.drawRoundRect (x1, y1, x2, y2);
      myGLCD.setBackColor(0, 180, 86);        // вокруг текста  зелёный фон (70, 200, 0);
      myGLCD.setColor(0, 0, 0);
    } else {  
          myGLCD.setColor(0, 0, 255);         // белый фонт, голубой фон 
	  myGLCD.fillRoundRect (x1, y1, x2, y2);
	  myGLCD.setColor(255, 255, 255);
          myGLCD.drawRoundRect (x1, y1, x2, y2);
          myGLCD.setBackColor(0, 0, 255);}
   
 if (fontsize) {
     myGLCD.setFont(BigFont);   // большой шрифт
     fx = x1+(((x2 - x1+1)-(stl*16))/2); fy = y1+(((y2 - y1+1)-16)/2);
     myGLCD.print(text, fx, fy);
    } else {
     myGLCD.setFont(SmallFont); // маленький шрифт
     fx = x1+(((x2 - x1)-(stl*8))/2); fy = y1+(((y2 - y1-1)-8)/2);
     myGLCD.print(text, fx+2, fy-1); } }
     
         
//--------------------- Кнопки на русском ----------------------- 
 void printButtonRUS(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false){
  int stl = strlen(text);
  int fx, fy;  
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2); 
  myGLCD.setBackColor(0, 0, 255);
  myGLCD.setFont(RusFont6); fx = x1+(((x2 - x1)-(stl*8))/2); fy = y1+(((y2 - y1-1)-6)/2); 
  myGLCD.print(text, fx+1, fy-2); } // fx - по горизонтали, fy - по вертикали     
            
// кнопки со стрелками для меню серво (маленькие)
void drawUpButton1(int x, int y){  // вверх
   myGLCD.setColor(0, 0, 255);
   myGLCD.fillRoundRect(x, y, x+18, y+18);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+18, y+18);
  for (int i=0; i<14; i++) myGLCD.drawLine(x+1+(i/1.5), y+16-i, x+17-(i/1.5), y+16-i); }

void drawDownButton1(int x, int y){  // вниз
   myGLCD.setColor(0, 0, 255);
   myGLCD.fillRoundRect(x, y, x+18, y+18);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+18, y+18);
  for (int i=0; i<14; i++) myGLCD.drawLine(x+1+(i/1.5), y+2+i, x+17-(i/1.5), y+2+i); }
  
// кнопки со стрелками (обычные) в меню настройки времени
void drawUpButton(int x, int y){   // вверх
   myGLCD.setColor(0, 0, 255);
   myGLCD.fillRoundRect(x, y, x+25, y+25);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+25, y+25);
  for (int i=0; i<15; i++) myGLCD.drawLine(x+3+(i/1.5), y+19-i, x+22-(i/1.5), y+19-i); }
  
void drawDownButton(int x, int y){  // вниз
   myGLCD.setColor(0, 0, 255);
   myGLCD.fillRoundRect(x, y, x+25, y+25);
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(x, y, x+25, y+25);
  for (int i=0; i<15; i++) myGLCD.drawLine(x+3+(i/1.5), y+6+i, x+22-(i/1.5), y+6+i); }  
  
// ----- виртуальные кнопки серые ------
void printButton100(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false){
  int stl = strlen(text);
  int fx, fy;  
  myGLCD.setColor(100, 100, 100);
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2); 
  myGLCD.setBackColor(100, 100, 100);
  if (fontsize) { myGLCD.setFont(BigFont); 
  fx = x1+(((x2 - x1+1)-(stl*16))/2); fy = y1+(((y2 - y1+1)-16)/2);
  myGLCD.print(text, fx, fy); } else { myGLCD.setFont(SmallFont); 
  fx = x1+(((x2 - x1)-(stl*8))/2); fy = y1+(((y2 - y1-1)-8)/2);
  myGLCD.print(text, fx+2, fy-1); } }
  
// ----- виртуальные кнопки R ------
void printButton104(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false){
  int stl = strlen(text);
  int fx, fy;  
  myGLCD.setColor(255, 0, 0);             // цвет внутреннего заполнения кнопки - красный
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);         // цвет рамки вокруг кнопки - бирюзовый (88, 255, 238);
  myGLCD.drawRoundRect (x1, y1, x2, y2); 
  myGLCD.setBackColor(255, 0, 0);
  if (fontsize) { myGLCD.setFont(BigFont); 
  fx = x1+(((x2 - x1+1)-(stl*16))/2); fy = y1+(((y2 - y1+1)-16)/2);
  myGLCD.print(text, fx, fy); } else { myGLCD.setFont(SmallFont); 
  fx = x1+(((x2 - x1)-(stl*8))/2); fy = y1+(((y2 - y1-1)-8)/2);
  myGLCD.print(text, fx+2, fy-1); } }
  
void printVBar(int val, int x1, int y1, const byte rgb[]){  // бар яркости подсветки LCD
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRect (x1-1, y1+21, x1+30, y1+125); // белая рамка  
  myGLCD.setColor(0, 0, 125); // цвет темно-синий
//                         V -  ширина активного бара
  myGLCD.drawRect (x1, y1+22, x1+29, y1+124);  
  myGLCD.setColor(rgb[0], rgb[1], rgb[2]);
//                                         v - ширина   
  myGLCD.fillRect (x1+1, y1+(124-val), x1+28, y1+124); // заполнение цветом при изменении уровня   
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect (x1+1, y1+22, x1+28, y1+(124-val));
   drawUpButtonSlide (x1, y1-15);     // кнопка вверх (x1, y1-10);
   drawDownButtonSlide (x1, y1+140);  // кнопка вниз  (x1, y1+134);
  setFont(SMALL, 255, 255, 255, 0, 0, 0);
//                                     V - яркость в %
if (val > -1) myGLCD.printNumI(val, x1+16-(intlen(val)*4), y1+66); } 

 int intlen(int number){    // number of digits
 int length;
 char tmpChar[5];    
 itoa(number, tmpChar, 10); length = strlen(tmpChar);
 return length; }
 
void printCoun(){
    myGLCD.setFont(RusFont3);
    myGLCD.setColor(255, 255, 255);       
    myGLCD.setBackColor(0, 0, 255); }
   //setFont(RUS3, 255, 255, 255, 0, 0, 255); }
       
void printFrame(){       // рамка при выборе цвета канала
    myGLCD.fillRect (1, 0, 318, 13);   // верх
    myGLCD.fillRect(2, 181, 317, 195); // низ
    myGLCD.setFont(RusFont3); 
    myGLCD.setColor(255, 255, 255); }

void printFrame2(){       // рамка при выборе цвета канала  
    myGLCD.fillRect (1, 0, 318, 13);   // верхний баннер
    myGLCD.setFont(RusFont3); 
    myGLCD.setColor(255, 255, 255); }  

void printFont(){ // шрифт и фонт в таймерах
    myGLCD.setFont(DotMatrix_M_Num);
    myGLCD.setColor(0, 255, 255);
    myGLCD.setBackColor(0, 0, 0); }

void printMes(){ // текст после выбора цвета какнала
    myGLCD.setFont(RusFont1);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[56]))); 
     myGLCD.print(buffer, CENTER, 184); }        // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН
                         
void waitForIt(int x1, int y1, int x2, int y2){    // Красная рамка при нажатии на экран
    myGLCD.setColor(255, 0, 0);
    myGLCD.drawRoundRect (x1, y1, x2, y2);
    waitForTouchRelease();
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (x1, y1, x2, y2); }
    
void OnOffTimer1(){ // графика on off для Таймера 1
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 25);        // AUTO 
       myGLCD.drawRect(102,17,171,49); 
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 25);  // OFF
     myGLCD.print(print_text[212], 192, 25);  // ON
       myGLCD.setColor(190, 190, 190);        // серый
       myGLCD.drawRect(177,19,242,47);        // on
       myGLCD.drawRect(175,17,244,49);        // ON     
       myGLCD.drawRect(248,17,317,49);        // OFF
       myGLCD.drawRect(250,19,315,47);}       // off
       
void OnOffTimer2(){ // графика on off для Таймера 2
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 61);        // AUTO
       myGLCD.drawRect(102,53,171,85);     // auto off
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 61);   // OFF
     myGLCD.print(print_text[212], 192, 61);   // ON
       myGLCD.setColor(190, 190, 190);         // серый
       myGLCD.drawRect(177,55,242,83);         // on
       myGLCD.drawRect(175,53,244,85);         // ON
       myGLCD.drawRect(250,55,315,83);         // off
       myGLCD.drawRect(248,53,317,85);}        // OFF
       
void OnOffTimer3(){ // графика on off для Таймера 3
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 97);        // AUTO 
       myGLCD.drawRect(102,89,171,121);    // auto on 
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 97);   // OFF 
     myGLCD.print(print_text[212], 192, 97);   //  ON
       myGLCD.setColor(190, 190, 190);         // серый 
       myGLCD.drawRect(177,91,242,119);        // on
       myGLCD.drawRect(175,89,244,121);        // ON
       myGLCD.drawRect(250,91,315,119);        // off 
       myGLCD.drawRect(248,89,317,121);}       // OFF    
       
void OnOffTimer4(){ // графика on off для Таймера 4
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 134);       // AUTO
       myGLCD.drawRect(102,125,171,157);   // auto on 
       myGLCD.setColor(80, 80, 80);  
     myGLCD.print(print_text[213], 259, 133);   // OFF  
     myGLCD.print(print_text[212], 192, 134);   // ON
       myGLCD.setColor(190, 190, 190);          // серый
       myGLCD.drawRect(177,127,242,155);        // on
       myGLCD.drawRect(175,125,244,157);        // ON
       myGLCD.drawRect(250,127,315,155);        // off 
       myGLCD.drawRect(248,125,317,157);}       // OFF
       
void OnOffTimer5(){ // графика on off для Таймера 5
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 168);       // AUTO 
       myGLCD.drawRect(102,161,171,193);   // auto on 
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 168);   // OFF 
     myGLCD.print(print_text[212], 192, 168);   // ON
       myGLCD.setColor(190, 190, 190);          // серый
       myGLCD.drawRect(177,163,242,191);        // on
       myGLCD.drawRect(175,161,244,193);        // ON
       myGLCD.drawRect(250,163,315,191);        // off
       myGLCD.drawRect(248,161,317,193);}       // OFF

 
/***************************** Олег / Кнопки плюс минус для целых чисел*******************************/

int PlusMinusCountI ( boolean refreshAll, boolean ButtonDisable, int XShift, int YShift, byte ButDist, int Min, int Max, int Step, int PlsMnsCount ) // =/- всех значений
{
/*
int XShift - X fist left "-" button coordinate / Первая левая координата по оси X
int YShift   Y fist left "-" button coordinate / Первая левая координата по оси Y
byte ButDist - distance between "-" and "+" buttons / Расстояние между кнопками + и -
int Min - minimum value of counter  / Минимальное значение счетчика
int Max - maximum counter  / Максимальное значение счетчика
int Step - step  / Шаг отсчета
int PlsMnsCount - input/output value / изменяемая переменнная
boolean refreshAll - if "false" - draw only static element, and display values without change / Только нарисовать (значения не изменяются)
boolean ButtonDisable - if "false" - disable touch button function  / Отключение чувствительности к нажатию тача
*/
// ---------------------------------------------    draw static element / Отрисовка статичных элементов
   if (refreshAll == false){									
   PlsMnsPress = false;
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect (ButMns[0]+XShift, ButMns[1]+YShift, ButMns[2]+XShift+ButDist, ButMns[3]+YShift); 
   printButton("-", (ButMns[0]+XShift), (ButMns[1]+YShift), (ButMns[2]+XShift), (ButMns[3]+YShift), LARGE);
   printButton("+", (ButMns[0]+XShift+ButDist), (ButMns[1]+YShift), (ButMns[2]+XShift+ButDist), (ButMns[3]+YShift), LARGE);
   setFont(LARGE, 255, 255, 255, 0, 0, 0);
//   myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+3);
   if (PlsMnsCount >=0 && PlsMnsCount <10 ) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-8),ButMns[1]+YShift+6);}
   if (PlsMnsCount >=10 && PlsMnsCount <100) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-16),ButMns[1]+YShift+6);} 
   if (PlsMnsCount >=100) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+6);}
                         }
//  ---------------------------wait for press button + and - / Ожидание  нажатия и отпускания кнопок + и -
 if (ButtonDisable == true){				// Minus button / Кнопка минус
      if (x>=(ButMns[0]+XShift) && x<=(ButMns[2]+XShift) && y>=(ButMns[1]+YShift) && y<=(ButMns[3]+YShift))  //press "-" / нажатие "-"
   { waitForIt(ButMns[0]+XShift, ButMns[1]+YShift, ButMns[2]+XShift, ButMns[3]+YShift);
	  PlsMnsCount -= Step;
	  PlsMnsPress = true; 
	  }											// Plus button / Кнопка плюс
      if (x>=(ButMns[0]+XShift+ButDist) && x<=(ButMns[2]+XShift+ButDist) && y>=(ButMns[1]+YShift) && y<=(ButMns[3]+YShift))  //press "+"  / Нажатие "+"
   { waitForIt(ButMns[0]+XShift+ButDist, ButMns[1]+YShift, ButMns[2]+XShift+ButDist, ButMns[3]+YShift);
	  PlsMnsCount += Step;
	  PlsMnsPress = true;
	  }}
 // ------------------------------ print counter values / вывод на экран значения счетчика --------------

   	  if (PlsMnsPress == true ){
//	  PlsMnsPress = false;
	  if (PlsMnsCount > Max) {PlsMnsCount = Max;}
	  if (PlsMnsCount < Min) {PlsMnsCount = Min;}
	  myGLCD.setColor(0, 0, 0);
          myGLCD.fillRect(ButMns[2]+XShift+2, ButMns[3]+YShift-2, ButMns[0]+XShift+ButDist-2, ButMns[1]+YShift+2);
          setFont(LARGE, 255, 255, 255, 0, 0, 0);
//	  myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+3);
	  if (PlsMnsCount >=0 && PlsMnsCount <10 ) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-8),ButMns[1]+YShift+6);}
          if (PlsMnsCount >=10 && PlsMnsCount <100) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-16),ButMns[1]+YShift+6);} 
          if (PlsMnsCount >=100) {myGLCD.printNumI(PlsMnsCount, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+6);}
	  }
	return PlsMnsCount;
	return PlsMnsPress;
 }

/***************************** Олег / Кнопки плюс минус для чисел с запятой *******************************/

float PlusMinusCountF ( boolean refreshAll, boolean ButtonDisable, int XShift, int YShift, byte ButDist, float Min, float Max, float Step, float PlsMnsCount ) // increase/decrease all values counter
{
/*
int XShift - X fist left "-" button coordinate / Первая левая координата по оси X
int YShift   Y fist left "-" button coordinate / Первая левая координата по оси Y
byte ButDist - distance between "-" and "+" buttons / Расстояние между кнопками + и -
int Min - minimum value of counter  / Минимальное значение счетчика
int Max - maximum counter  / Максимальное значение счетчика
int Step - step  / Шаг отсчета
int PlsMnsCount - input/output value / изменяемая переменнная
boolean refreshAll - if "false" - draw only static element, and display values without change / Только нарисовать (значения не изменяются)
boolean ButtonDisable - if "false" - disable touch button function  / Отключение чувствительности к нажатию тача
*/
// ---------------------------------------------    draw static element / Отрисовка статичных элементов
   if (refreshAll == false){									
   PlsMnsPress = false;
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect (ButMns[0]+XShift, ButMns[1]+YShift, ButMns[2]+XShift+ButDist, ButMns[3]+YShift); 
   printButton("-", (ButMns[0]+XShift), (ButMns[1]+YShift), (ButMns[2]+XShift), (ButMns[3]+YShift), LARGE);
   printButton("+", (ButMns[0]+XShift+ButDist), (ButMns[1]+YShift), (ButMns[2]+XShift+ButDist), (ButMns[3]+YShift), LARGE);
   setFont(LARGE, 255, 255, 255, 0, 0, 0);
//   myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-36),ButMns[1]+YShift+3);
   if (PlsMnsCount >=0 && PlsMnsCount <10 ) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-26),ButMns[1]+YShift+6);}
   if (PlsMnsCount >=10 && PlsMnsCount <100) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-28),ButMns[1]+YShift+6);} 
   if (PlsMnsCount >=100) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-36),ButMns[1]+YShift+6);}
                         }
//  --------------------------wait for press button + and - / Ожидание  нажатия и отпускания кнопок + и -
 if (ButtonDisable == true){				// Minus button / кнопка минус
      if (x>=(ButMns[0]+XShift) && x<=(ButMns[2]+XShift) && y>=(ButMns[1]+YShift) && y<=(ButMns[3]+YShift))  //press "-"
   { waitForIt(ButMns[0]+XShift, ButMns[1]+YShift, ButMns[2]+XShift, ButMns[3]+YShift);
	  PlsMnsCount -= Step;
	  PlsMnsPress = true; 
	  }
											// Plus button / кнопка плюс
      if (x>=(ButMns[0]+XShift+ButDist) && x<=(ButMns[2]+XShift+ButDist) && y>=(ButMns[1]+YShift) && y<=(ButMns[3]+YShift))  //press "+"
   { waitForIt(ButMns[0]+XShift+ButDist, ButMns[1]+YShift, ButMns[2]+XShift+ButDist, ButMns[3]+YShift);
	  PlsMnsCount += Step;
	  PlsMnsPress = true;
	  }
                            }
  // ------------------------------ print counter values / вывод на экран значения счетчика --------------
	  if (PlsMnsPress == true ){
//      PlsMnsPress = false;
	  if (PlsMnsCount > Max+0.05) {PlsMnsCount = Max;}
	  if (PlsMnsCount < Min) {PlsMnsCount = Min;}
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(ButMns[2]+XShift+2, ButMns[3]+YShift-2, ButMns[0]+XShift+ButDist-2, ButMns[1]+YShift+2);
      setFont(LARGE, 255, 255, 255, 0, 0, 0);
	  if (PlsMnsCount >=0 && PlsMnsCount <10 ) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-26),ButMns[1]+YShift+6);}
	  if (PlsMnsCount >=10 && PlsMnsCount <100) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-28),ButMns[1]+YShift+6);}
	  if (PlsMnsCount >=100) {myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-36),ButMns[1]+YShift+6);}
	  }
	return PlsMnsCount;
	return PlsMnsPress;
 }
                     
       
//******************Работа с памятью*****************EEPROM FUNCTIONS***************//
struct config_t {    // температура
  int tempset;
  int tempoff;
  int tempalarm;
  int tempSoundAlarmC; } tempSettings;
  
struct config_ph {    // PH
  float Pvolt7;
  float Pvolt10;
  float PSetvalPH; } PHsettings;  

struct config_m {    // луна
  int MinI_t;
  int MaxI_t; } MinMaxIsettings;
  
struct config_L {
  int DimLEDsOnOff;
  int LEDsDimTempC; 
  int LEDsDimPercent; } LEDsFailsafeSettings;

struct config_g {     // главные настройки
  int ShowHideDOW;
  int Heatsink1FanTempC;
  int Heatsink2FanTempC;
  int SCREENsaver;
  int ScreensaverClockOrBlank;
  int ScreensaverDOWonOff;
  int ScreensaverTimer; } GENERALsettings;
 
struct config_h { // таймеры
  int on1;
  int on2;
  int on3;
  int on4;
  int on5; 
  int off1;
  int off2;
  int off3;
  int off4;
  int off5; } TIMERsettings;

struct config_preset{ // пресеты
  byte wcol_out; 
  byte bcol_out;
  byte rbcol_out;
  byte rcol_out;
  byte uvcol_out;
  byte ocol_out;
  byte grcol_out; } LedPresetSetting;
  


struct config_f
{
  int feedFish1h;
  int feedFish1m;
  int feedFish2h;
  int feedFish2m;
  int feedFish3h;
  int feedFish3m;
  int feedFish4h;
  int feedFish4m;
  int feedTime1;
  int feedTime2;
  int feedTime3;
  int feedTime4;  
} FEEDERsettings;

struct config_d
{
  int DozTime1h;
  int DozTime1m;
  int DozTime2h;
  int DozTime2m;
  int DozTime3h;
  int DozTime3m;
  int DozTime4h;
  int DozTime4m;
  int dozTime1;
  int dozTime2;
  int dozTime3;
  int dozTime4; 
  int doz1sec;
  int doz2sec;
  int doz3sec;
  int doz4sec;
  int doz1num;
  int doz2num;
  int doz3num;
  int doz4num;
  int doz1int;
  int doz2int;
  int doz3int;
  int doz4int;
  int doz1cal;
  int doz2cal;
  int doz3cal;
  int doz4cal;
} DOSINGsettings;
  
  

  
struct config_D {    // Dimm
  int DimmL;
  int setLEDsDimPercentL; }  DIMMsettings;  
  
// функция очистки памяти   
void EraseAllEEPROM_SaveDefault(){  // check memory for my code format #oleg
 int k = EEPROM.read(0);
 if (k!=125) { // 127 проверочное число
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 0, 0);
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 255, 0, 0, 0, 0, 0);
    myGLCD.print(print_text[191], CENTER, 110);       // ПОДОЖДИТЕ, идет очистка памяти
 for (int i = 0; i < 4096; i++) { EEPROM.write(i, 0); 
 if (i ==0) {EEPROM.write(0,125);} 
 if ((i==0)|| (i==512)|| (i==1024)||(i==1536)||(i==2048)||(i==2560)||(i==3072)||(i==3584)||(i==4094)){
     setFont(LARGE, 200, 200, 200, 0, 0, 0);
     myGLCD.printNumI(i, CENTER, 140); } }  
        COLOR=0; SaveLEDToEEPROM();        // save default 
         EEPROM.write(780, 0xFF); } }      // enable all led channel
      
void SaveLEDToEEPROM(){      // сохранить настройки яркости
  EEPROM.write(0, 125);      // to determine if data available in EEPROM
 if ((COLOR==1)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*0),wled[i-1]);}} // xled[] address must be from 1 to 96     
 if ((COLOR==2)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*1),bled[i-1]);}} // xled[] address must be from 1 to 96		
 if ((COLOR==3)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*2),rbled[i-1]);}}      
 if ((COLOR==4)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*3),rled[i-1]);}}         
 if ((COLOR==5)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*4),uvled[i-1]);}}          
 if ((COLOR==6)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*5),oLed[i-1]);}}         
 if ((COLOR==7)||(COLOR==0)){for(int i=1;i<97;i++){EEPROM.write(i+(96*6),gled[i-1]);}}}           

void SaveOnOffLedStatus(boolean LedShannelFlag_on_off= false){   // вкл / вык канал 
  if (LedShannelFlag_on_off == true){
        if (COLOR==1){bitSet(LedShannelStatusByte,0);} 
        if (COLOR==2){bitSet(LedShannelStatusByte,1);} 
        if (COLOR==3){bitSet(LedShannelStatusByte,2);} 
        if (COLOR==4){bitSet(LedShannelStatusByte,3);}
	if (COLOR==5){bitSet(LedShannelStatusByte,4);} 
	if (COLOR==6){bitSet(LedShannelStatusByte,5);} 
	if (COLOR==7){bitSet(LedShannelStatusByte,6);} 
       } else {
         if (COLOR==1){bitClear(LedShannelStatusByte,0);} 
         if (COLOR==2){bitClear(LedShannelStatusByte,1);} 
         if (COLOR==3){bitClear(LedShannelStatusByte,2);} 
	 if (COLOR==4){bitClear(LedShannelStatusByte,3);}
	 if (COLOR==5){bitClear(LedShannelStatusByte,4);} 
         if (COLOR==6){bitClear(LedShannelStatusByte,5);} 
	 if (COLOR==7){bitClear(LedShannelStatusByte,6);}}
	     EEPROM.write(780, LedShannelStatusByte);}

void ReadOnOffLedStatus(){ // if reader bit =0 LedShannelFlag_on_off = false, else LedShannelFlag_on_off = true;
 	if (COLOR==1){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,0);} 
        if (COLOR==2){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,1);} 
        if (COLOR==3){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,2);} 
        if (COLOR==4){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,3);}
        if (COLOR==5){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,4);} 
	if (COLOR==6){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,5);}  
	if (COLOR==7){LedShannelFlag_on_off = bitRead(LedShannelStatusByte,6);}}

void SaveMoonLEDToEEPROM(){ // сохранение минимальной / макс. яркостей луны
     MinMaxIsettings.MinI_t = int(MinI);
     MinMaxIsettings.MaxI_t = int(MaxI);
     EEPROM_writeAnything(600+200, MinMaxIsettings); }  

void SaveLEDsFailsafeToEEPROM(){ // запись настроек авто-умен. яркости при перегреве
    LEDsFailsafeSettings.DimLEDsOnOff = int(setDimLEDsOnOff);
    LEDsFailsafeSettings.LEDsDimTempC = int(setLEDsDimTempC);
    LEDsFailsafeSettings.LEDsDimPercent = int(setLEDsDimPercent);
    EEPROM_writeAnything(610+200, LEDsFailsafeSettings); } 

void SaveTempToEEPROM(){
    tempSettings.tempset = int(setTempC*10);
    tempSettings.tempoff = int(offTempC*10);
    tempSettings.tempalarm = int(alarmTempC*10);
    tempSettings.tempSoundAlarmC =  int(setTempToSoundAlarmC);
    EEPROM_writeAnything(640+200, tempSettings); } 
    
 void SavePHsetToEEPROM() {   // запись настроек ph
   PHsettings.Pvolt7 = volt7*10000;
   PHsettings.Pvolt10 = volt10*10000;
   PHsettings.PSetvalPH = SetvalPH*10;
   EEPROM_writeAnything(2300, PHsettings);

 }    

void SaveGenSetsToEEPROM(){
    GENERALsettings.Heatsink1FanTempC = int(setTempToBeginHeatsink1FanC*10); 
    GENERALsettings.Heatsink2FanTempC = int(setTempToBeginHeatsink2FanC*10);  
    GENERALsettings.SCREENsaver = int(setScreensaverOnOff);
    GENERALsettings.ScreensaverClockOrBlank = int(setClockOrBlank);      
    GENERALsettings.ScreensaverDOWonOff = int(setScreensaverDOWonOff);    
    GENERALsettings.ScreensaverTimer = int(setSSmintues);    
    EEPROM_writeAnything(660+200, GENERALsettings); } 
    
void SaveFeedTimesToEEPROM()
{
  FEEDERsettings.feedFish1h = int(feedFish1H);
  FEEDERsettings.feedFish1m = int(feedFish1M);  
  FEEDERsettings.feedFish2h = int(feedFish2H);
  FEEDERsettings.feedFish2m = int(feedFish2M);  
  FEEDERsettings.feedFish3h = int(feedFish3H);
  FEEDERsettings.feedFish3m = int(feedFish3M);  
  FEEDERsettings.feedFish4h = int(feedFish4H);
  FEEDERsettings.feedFish4m = int(feedFish4M);
  FEEDERsettings.feedTime1 = int(FEEDTime1);  
  FEEDERsettings.feedTime2 = int(FEEDTime2);  
  FEEDERsettings.feedTime3 = int(FEEDTime3);  
  FEEDERsettings.feedTime4 = int(FEEDTime4);    
  EEPROM_writeAnything(2000+200, FEEDERsettings);//680
}  

void SaveDoseTimesToEEPROM()
{
  DOSINGsettings.DozTime1h = int(dozPump1H);
  DOSINGsettings.DozTime1m = int(dozPump1M);  
  DOSINGsettings.DozTime2h = int(dozPump2H);
  DOSINGsettings.DozTime2m = int(dozPump2M);  
  DOSINGsettings.DozTime3h = int(dozPump3H);
  DOSINGsettings.DozTime3m = int(dozPump3M);  
  DOSINGsettings.DozTime4h = int(dozPump4H);
  DOSINGsettings.DozTime4m = int(dozPump4M);
  DOSINGsettings.dozTime1 = int(DOZTime1);  
  DOSINGsettings.dozTime2 = int(DOZTime2);  
  DOSINGsettings.dozTime3 = int(DOZTime3);  
  DOSINGsettings.dozTime4 = int(DOZTime4); 
  DOSINGsettings.doz1sec = int(dozVal1);
  DOSINGsettings.doz2sec = int(dozVal2);  
  DOSINGsettings.doz3sec = int(dozVal3);
  DOSINGsettings.doz4sec = int(dozVal4);
  DOSINGsettings.doz1num = int(numDoz1);
  DOSINGsettings.doz2num = int(numDoz2);
  DOSINGsettings.doz3num = int(numDoz3);
  DOSINGsettings.doz4num = int(numDoz4);
  DOSINGsettings.doz1int = int(intDoz1);
  DOSINGsettings.doz2int = int(intDoz2);
  DOSINGsettings.doz3int = int(intDoz3);
  DOSINGsettings.doz4int = int(intDoz4); 
  DOSINGsettings.doz1cal = int(dozCal1);  
  DOSINGsettings.doz2cal = int(dozCal2);  
  DOSINGsettings.doz3cal = int(dozCal3);  
  DOSINGsettings.doz4cal = int(dozCal4);  
  EEPROM_writeAnything(1900+200, DOSINGsettings);//680
}    
    
void SaveTimerEEPROM(){    // запись в память настроек времени 
    TIMERsettings.on1 = int(on1);    
    TIMERsettings.on2 = int(on2);    
    TIMERsettings.on3 = int(on3);    
    TIMERsettings.on4 = int(on4);    
    TIMERsettings.on5 = int(on5);    
    TIMERsettings.off1 = int(off1);  
    TIMERsettings.off2 = int(off2);  
    TIMERsettings.off3 = int(off3);  
    TIMERsettings.off4 = int(off4);  
    TIMERsettings.off5 = int(off5);  
    EEPROM_writeAnything(1200, TIMERsettings); } // 700+200
               
// запись в память датчиков температуры 
void SaveDallasAddress (){
  for (byte i = 0; i < 8; i++){
   EEPROM.write(900+i, Heatsink1Thermometer [i]);  // sensor address
   EEPROM.write(900+i+9, Heatsink2Thermometer [i]);
   EEPROM.write(900+i+18, waterThermometer [i]); }
   EEPROM.write(900+8, counterB1);		 // config byte	
   EEPROM.write(900+17, counterB2);
   EEPROM.write(900+26, counterB3); } 
   
 
 void SaveLEDPresetToEEPROM(){ // сохранение настроек пресетов
   LedPresetSetting.wcol_out = wcol_out;
   LedPresetSetting.bcol_out = bcol_out;
   LedPresetSetting.rbcol_out = rbcol_out;  
   LedPresetSetting.rcol_out =  rcol_out;
   LedPresetSetting.uvcol_out = uvcol_out;
   LedPresetSetting.ocol_out = ocol_out;  
   LedPresetSetting.grcol_out = grcol_out;
   EEPROM_writeAnything(1000 + AddressShift, LedPresetSetting); }
   
   


void SaveDimmLEDToEEPROM(){        // Dimm
   DIMMsettings.DimmL = int(DimmL);
   DIMMsettings.setLEDsDimPercentL = int(setLEDsDimPercentL);  
   EEPROM_writeAnything(1500, DIMMsettings); } 
 
void SaveLCDbrightToEEPROM(){   // сохранение настроек подсветки
EEPROM.write(1900, LCDbright); }


void ReadLEDPresetFromEEPROM(){
    EEPROM_readAnything(1000 + AddressShift, LedPresetSetting); 
    wcol_out = LedPresetSetting.wcol_out;
    bcol_out = LedPresetSetting.bcol_out;
    rbcol_out = LedPresetSetting.rbcol_out;
    rcol_out = LedPresetSetting.rcol_out; 
    uvcol_out = LedPresetSetting.uvcol_out;
    ocol_out= LedPresetSetting.ocol_out;
    grcol_out = LedPresetSetting.grcol_out; }
    
    
void ReadLedFromEEPROM(){  // COLOR=0 - read all colours 
  byte k = EEPROM.read(0);
     int Temp;
 if (k==125){     
   if ((COLOR==1) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*0));
     if (Temp >100){Temp = 100;} wled[i-1] = Temp;}}
   if ((COLOR==2) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*1));
     if (Temp >100){Temp = 100;} bled[i-1] = Temp;}} 
   if ((COLOR==3) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*2));
     if (Temp >100){Temp = 100;} rbled[i-1] = Temp;}}
   if ((COLOR==4) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*3));
     if (Temp >100){Temp = 100;} rled[i-1] = Temp;}}  
   if ((COLOR==5) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*4));
     if (Temp >100){Temp = 100;} uvled[i-1] = Temp;}}
   if ((COLOR==6) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*5));
     if (Temp >100){Temp = 100;} oLed[i-1] = Temp;}} 
   if ((COLOR==7) || (COLOR==0)){for (byte i=1; i<97; i++){Temp = EEPROM.read(i+(96*6));
     if (Temp >100){Temp = 100;} gled[i-1] = Temp;}}
     
  if ((COLOR==8)||(COLOR==0)){ // moon
       EEPROM_readAnything(600+200, MinMaxIsettings); // 200 байт для мин. макс. значений яркости луны         
       MinI = MinMaxIsettings.MinI_t;         
  if ( MinI>100 ) {MinI= 10;}      // set default = 10
       MaxI = MinMaxIsettings.MaxI_t; 
  if ( MaxI>100 ) {MaxI= 90;} }    // set default = 90   
	 LedShannelStatusByte = EEPROM.read(780); }

       EEPROM_readAnything(610+200, LEDsFailsafeSettings); //температура авто-уменьшения яркости при перегреве     
       setDimLEDsOnOff = LEDsFailsafeSettings.DimLEDsOnOff;      
  if ( setDimLEDsOnOff >1) {setDimLEDsOnOff=1;}					 // set default = 1 (on / off)
       setLEDsDimTempC = LEDsFailsafeSettings.LEDsDimTempC;
  if ( setLEDsDimTempC>99 || setLEDsDimTempC<40 ) {setLEDsDimTempC=50;}          // set default = 50C   
       setLEDsDimPercent = LEDsFailsafeSettings.LEDsDimPercent;
  if ( setLEDsDimPercent>80 || setLEDsDimPercent<10 ) {setLEDsDimPercent=50;} }  // set default = 50%  

void ReadFromEEPROM(){      // read from eeprom all data exclude LED SETTING
  int k = EEPROM.read(0);
  char tempString[3]; 

// предустановки темперетуры по умолчанию     
      EEPROM_readAnything(640+200, tempSettings);     
     setTempC = tempSettings.tempset; setTempC /=10; 
 if (setTempC <10||setTempC >40){setTempC=26.0;}        // set default = 26 C
     offTempC=tempSettings.tempoff; offTempC /=10;   
 if (offTempC <0||offTempC >5){offTempC=1.0;}           // set default = 1 C
     alarmTempC=tempSettings.tempalarm; alarmTempC /= 10;     
 if (alarmTempC == 255){alarmTempC = 255;} else {
 if (alarmTempC <0||alarmTempC >9.9){alarmTempC=5.0;}}  // set default = 5 C
    
      setTempToSoundAlarmC = tempSettings.tempSoundAlarmC;              // звуковая тревога
  if (setTempToSoundAlarmC ==255){setTempToSoundAlarmC = 255;} else { 
  if (setTempToSoundAlarmC <40 || setTempToSoundAlarmC >99){setTempToSoundAlarmC=50;}} // set default = 50 C
  
    EEPROM_readAnything(2300, PHsettings); // ph
    volt7 = PHsettings.Pvolt7;volt7 /=10000.0;
        
    if (volt7 <0.5 || volt7 >0.9) {volt7 = 0.6939;}
   volt10 = PHsettings.Pvolt10;volt10 /=10000.0;
  
    if (volt10 <0.2 || volt10 >0.5) {volt10 = 0.3846;}
    SetvalPH = PHsettings.PSetvalPH;SetvalPH /=10.0;
 
    if (SetvalPH <5 || SetvalPH >11) {SetvalPH = 7.0;}
 
  
      EEPROM_readAnything(660+200, GENERALsettings); // температура вентеляторов на радиаторе	660+200	  
      setTempToBeginHeatsink1FanC = GENERALsettings.Heatsink1FanTempC; setTempToBeginHeatsink1FanC /= 10;  
  if (setTempToBeginHeatsink1FanC <25 || setTempToBeginHeatsink1FanC >50){setTempToBeginHeatsink1FanC=30.0;} // set default = 30 C  
      setTempToBeginHeatsink2FanC = GENERALsettings.Heatsink2FanTempC; setTempToBeginHeatsink2FanC /= 10;    
  if (setTempToBeginHeatsink2FanC <25 || setTempToBeginHeatsink2FanC >50){setTempToBeginHeatsink2FanC=30.0;} // set default = 30 C 
  
      setScreensaverOnOff = GENERALsettings.SCREENsaver;		 // OFF=0 || ON=1   
  if (setScreensaverOnOff >1) {setScreensaverOnOff=1;} 
      setClockOrBlank = GENERALsettings.ScreensaverClockOrBlank;         // Clock Screensaver=0 || Blank Screen=1    
  if (setClockOrBlank >0) {setClockOrBlank=1;} 
      setScreensaverDOWonOff = GENERALsettings.ScreensaverDOWonOff;      // OFF=0 || ON=1 Shows/Hides DOW   
  if (setScreensaverDOWonOff >1) {setScreensaverDOWonOff=1;} 
      setSSmintues = GENERALsettings.ScreensaverTimer;			 // 	    
  if (setSSmintues <1 || setSSmintues >99) {setSSmintues=10;}		 // 1....99 min
 
 EEPROM_readAnything(2000+200, FEEDERsettings);  //680
  feedFish1H = FEEDERsettings.feedFish1h;
  if (feedFish1H >24) {feedFish1H=20;}		    // 0....24 h
  feedFish1M = FEEDERsettings.feedFish1m;  
  if (feedFish1M >60) {feedFish1M=0;}		    // 0....60 min
  feedFish2H = FEEDERsettings.feedFish2h;
  if (feedFish2H >24) {feedFish2H=20;}		    // 0....24 h
  feedFish2M = FEEDERsettings.feedFish2m;  
  if (feedFish2M >60) {feedFish2M=0;}		    // 0....60 min
  feedFish3H = FEEDERsettings.feedFish3h;
  if (feedFish3H >24) {feedFish3H=20;}		    // 0....24 h
  feedFish3M = FEEDERsettings.feedFish3m;  
  if (feedFish3M >60) {feedFish3M=0;}		    // 0....60 min
  feedFish4H = FEEDERsettings.feedFish4h;
  if (feedFish4H >24) {feedFish4H=20;}		    // 0....24 h
  feedFish4M = FEEDERsettings.feedFish4m;
  if (feedFish4M >60) {feedFish4M=0;}		    // 0....60 min
  FEEDTime1 = FEEDERsettings.feedTime1;
  if (FEEDTime1 >1) {FEEDTime1=0;} 
  FEEDTime2 = FEEDERsettings.feedTime2;
  if (FEEDTime2 >1) {FEEDTime2=0;} 
  FEEDTime3 = FEEDERsettings.feedTime3;
  if (FEEDTime3 >1) {FEEDTime3=0;} 
  FEEDTime4 = FEEDERsettings.feedTime4;  
  if (FEEDTime4 >1) {FEEDTime4=0;} 
  
   
 EEPROM_readAnything(1900+200, DOSINGsettings);  //680
  dozPump1H = DOSINGsettings.DozTime1h;
  if (dozPump1H >24) {dozPump1H=20;}		    // 0....24 h
  dozPump1M = DOSINGsettings.DozTime1m;  
  if (dozPump1M >60) {dozPump1M=0;}		    // 0....60 min
  dozPump2H = DOSINGsettings.DozTime2h;
  if (dozPump2H >24) {dozPump2H=20;}		    // 0....24 h
  dozPump2M = DOSINGsettings.DozTime2m;  
  if (dozPump2M >60) {dozPump2M=0;}		    // 0....60 min
  dozPump3H = DOSINGsettings.DozTime3h;
  if (dozPump3H >24) {dozPump3H=20;}		    // 0....24 h
  dozPump3M = DOSINGsettings.DozTime3m;  
  if (dozPump3M >60) {dozPump3M=0;}		    // 0....60 min
  dozPump4H = DOSINGsettings.DozTime4h;
  if (dozPump4H >24) {dozPump4H=20;}		    // 0....24 h
  dozPump4M = DOSINGsettings.DozTime4m;
  if (dozPump4M >60) {dozPump4M=0;}		    // 0....60 min
  DOZTime1 = DOSINGsettings.dozTime1;
  if (DOZTime1 >1) {DOZTime1=0;} 
  DOZTime2 = DOSINGsettings.dozTime2;
  if (DOZTime2 >1) {DOZTime2=0;} 
  DOZTime3 = DOSINGsettings.dozTime3;
  if (DOZTime3 >1) {DOZTime3=0;} 
  DOZTime4 = DOSINGsettings.dozTime4;  
  if (DOZTime4 >1) {DOZTime4=0;} 
  numDoz1 = DOSINGsettings.doz1num;
  numDoz2 = DOSINGsettings.doz2num;  
  numDoz3 = DOSINGsettings.doz3num;
  numDoz4 = DOSINGsettings.doz4num;
  intDoz1 = DOSINGsettings.doz1int;
  intDoz2 = DOSINGsettings.doz2int;  
  intDoz3 = DOSINGsettings.doz3int;  
  intDoz4 = DOSINGsettings.doz4int;
  dozVal1 = DOSINGsettings.doz1sec;
  dozVal2 = DOSINGsettings.doz2sec;  
  dozVal3 = DOSINGsettings.doz3sec;
  dozVal4 = DOSINGsettings.doz4sec;
  dozCal1 = DOSINGsettings.doz1cal;  
  dozCal2 = DOSINGsettings.doz2cal;  
  dozCal3 = DOSINGsettings.doz3cal;  
  dozCal4 = DOSINGsettings.doz4cal; 
    
   EEPROM_readAnything(1200, TIMERsettings); // таймеры // 700+200
   on1 = TIMERsettings.on1;    
   on2 = TIMERsettings.on2;    
   on3 = TIMERsettings.on3;    
   on4 = TIMERsettings.on4;
   on5 = TIMERsettings.on5;
   off1 = TIMERsettings.off1;   
   off2 = TIMERsettings.off2;  
   off3 = TIMERsettings.off3;  
   off4 = TIMERsettings.off4;  
   off5 = TIMERsettings.off5; 
 
  EEPROM_readAnything(1500, DIMMsettings);      // Dimm
  DimmL = DIMMsettings.DimmL;
  setLEDsDimPercentL = DIMMsettings.setLEDsDimPercentL; }
      
// чтение из памяти адресов датчиков температуры  
void ReadDallasAddress (){
 for (byte i = 0; i < 8; i++){
  Heatsink1Thermometer [i] = EEPROM.read(900+i);    // sensor address
  Heatsink2Thermometer [i] = EEPROM.read(900+i+9);
  waterThermometer [i] = EEPROM.read(900+i+18); }
  counterB1 = EEPROM.read(900+8);	           // config byte
  counterB2 = EEPROM.read(900+17);
  counterB3 = EEPROM.read(900+26); }
  
 void ReadLCDbright(){  // чтение из памяти настроек яркости
   EEPROM_readAnything(1900, LCDbright);} 
  
    
// RTC FUNCTIONS 
void TimeDateBar(boolean refreshAll=false){
   RTC.getTime();
   
     myGLCD.setFont(RusFont1); 
     myGLCD.setColor(38, 195, 178);   // цвет бирюзовый
     myGLCD.setBackColor(30, 30, 30); // фон серый
     strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[0]))); 
     myGLCD.print(buffer, 224, 229);  // надпись на русском "Время"  
     myGLCD.setFont(RusFont2);        // большой шрифт
     myGLCD.setColor(255, 255, 0);    // цвет шрифта желтый
     myGLCD.setBackColor(30, 30, 30); // фон серый
                   
// Отображение времени в главном экране
      if (RTC.hour < 10) {
            myGLCD.print(" ",271,227);
            myGLCD.printNumI(RTC.hour,280, 227);  
               } else myGLCD.printNumI(RTC.hour,270, 227);
       
      myGLCD.print(":",285, 227);
       
      if (RTC.minute < 10) {
             myGLCD.print("0",293,227);
             myGLCD.printNumI(RTC.minute,300, 227);
               }else myGLCD.printNumI(RTC.minute,293, 227);
             }
           
           
           
           
           
void titledate(boolean refreshAll=false){ // Отображение даты и дня недели в нижней части экрана
 //             Time t;
 //       RTC.getTime(); 
               
// Отображать день недели   RTC.day - дата, RTC.month  - месяц, RTC.year - год, RTC.dow  - день недели
                myGLCD.setFont(RusFont1);  // Выбор шрифта 
                myGLCD.setColor(190, 190, 190);  // серый шрифт
                myGLCD.setBackColor(30, 30, 30);
           //     RTC.dow=calcDOW(RTC.day, RTC.month, RTC.year);  
                                   
     if (RTC.dow==1){ myGLCD.print(print_text[156], 5, 230);}   // Понедельник / Monday                
     if (RTC.dow==2){ myGLCD.print(print_text[157], 5, 230);}   // Вторник / Tuesday                 
     if (RTC.dow==3){ myGLCD.print(print_text[158], 5, 230);}   // Среда / Wednesday             
     if (RTC.dow==4){ myGLCD.print(print_text[159], 5, 230);}   // Четверг / Thursday              
     if (RTC.dow==5){ myGLCD.print(print_text[160], 5, 230);}   // Пятница / Friday   
                    myGLCD.setColor(255, 0, 0); // шрифт красный для(субботы, воскресенья)
     if (RTC.dow==6){ myGLCD.print(print_text[161], 5, 230);}   // Суббота / Saturday    
     if (RTC.dow==0){ myGLCD.print(print_text[162], 5, 230);}   // Воскресенье / Sunda
            
// Позиция отображения даты, в зависимости от длинны имени дня недели 
      if (RTC.dow==1 || RTC.dow==0 ){xdate=93;} // 92                   // позиция для Понедельник, Воскресенье.
      if (RTC.dow==2 || RTC.dow==4 || RTC.dow==5 || RTC.dow==6 ){xdate=61;} // позиция для Вторник, Четверг, Пятница, Суббота.
      if (RTC.dow==3 ){xdate=45;}                                     // позиция для Среда.
            
// Отображать дату
           setFont(SMALL, 160, 255, 255, 30, 30, 30);
           myGLCD.printNumI(RTC.day, xdate+5, 227);
            
// Отображать месяц   // позиция месяца xdate+24
                myGLCD.setFont(RusFont1);  // Выбор шрифта 
                myGLCD.setColor(255, 255, 255);  // белый шрифт
                myGLCD.setBackColor(30, 30, 30); // фон серый
           //setFont(RUS1, 255, 255, 255, 30, 30, 30); // белый шрифт, фон серый
       if (RTC.month==1){ myGLCD.print(print_text[163], xdate+25, 230);}     // Января / January           
       if (RTC.month==2){ myGLCD.print(print_text[164], xdate+25, 230);}     // Февраля / February                
       if (RTC.month==3){ myGLCD.print(print_text[165], xdate+25, 230);}     // Марта  / March               
       if (RTC.month==4){ myGLCD.print(print_text[166], xdate+25, 230);}     // Апреля / Avril              
       if (RTC.month==5){ myGLCD.print(print_text[167], xdate+25, 230);}     // Мая / May            
       if (RTC.month==6){ myGLCD.print(print_text[168], xdate+25, 230);}     // Июня / June                
       if (RTC.month==7){ myGLCD.print(print_text[169], xdate+25, 230);}     // Июля / July           
       if (RTC.month==8){ myGLCD.print(print_text[170], xdate+25, 230);}     // Августа / August              
       if (RTC.month==9){ myGLCD.print(print_text[171], xdate+25, 230);}     // Сентября / September              
       if (RTC.month==10){ myGLCD.print(print_text[172], xdate+25, 230);}    // Октября / October               
       if (RTC.month==11){ myGLCD.print(print_text[173], xdate+25, 230);}    // Ноября / November                
       if (RTC.month==12){ myGLCD.print(print_text[174], xdate+25, 230);}    // Декабря / December
            
// Позиция отображения года, в зависимости от длинны имени месяца
       if (RTC.month==1 || RTC.month==4 || RTC.month==8 || RTC.month==11){xdate=xdate+90;} // Января, Апрель, Август, Ноябрь         
       if (RTC.month==3 || RTC.month==6 || RTC.month==7){xdate=xdate+70;}              // Март, Июнь, Июль
       if (RTC.month==2 || RTC.month==10 || RTC.month==12){xdate=xdate+86;}            // Февраль, Октябрь, Декабрь
       if (RTC.month==5){xdate=xdate+60;}                                      // Май
       if (RTC.month==9){xdate=xdate+95;}                                      // Сентябрь
            
// Отображать год
          setFont(SMALL, 248, 255, 120, 30, 30, 30);
          myGLCD.printNumI(RTC.year, xdate, 227); } 
                       
byte calcDOW(byte d, byte m, int y){ int dow;
    byte mArr[12] = {6,2,2,5,0,3,5,1,4,6,2,4}; 
    dow = (y % 100);
    dow = dow*1.25;
    dow += d;
    dow += mArr[m-1];
 if (((y % 4)==0) && (m<3))dow -= 1; while (dow>7)dow -= 7; return dow; }

void showDOW(byte dow){ // отображение дней недели в меню настройки времен и даты
 char* str[] = {"OMD","BRN","QPD","WRB","ORM","QAR","BQJ"}; // ПНД ВТО СРД ЧТВ ПТН СБТ ВСК 
   myGLCD.setFont(RusFont2);
   myGLCD.setColor(255, 255, 255);
   myGLCD.setBackColor(0, 0, 0);
   //setFont(RUS2, 255, 255, 255, 0, 0, 0);
   myGLCD.print(F("         "), 17, 178);
   myGLCD.print(str[dow-1], 41, 178); }
  
byte validateDate(byte d, byte m, word y){
    byte mArr[12] = {31,0,31,30,31,30,31,31,30,31,30,31};
    byte od; 
  if (m==2){ if ((y % 4)==0){ if (d==30) od=1; else 
  if (d==0) od=29; else od=d; } else { 
  if (d==29) od=1; else 
  if (d==0) od=28; else od=d; } } else { 
  if (d==0) od=mArr[m-1]; else 
  if (d==(mArr[m-1]+1)) od=1; else od=d; } return od; }

byte validateDateForMonth(byte d, byte m, word y){
    byte mArr[12] = {31,0,31,30,31,30,31,31,30,31,30,31};
    byte od;
    boolean dc=false; 
  if (m==2){ if ((y % 4)==0){ if (d>29){ d=29;} } else { 
  if (d>28){ d=28;} } } else { 
  if (d>mArr[m-1]){ d=mArr[m-1];} } return d; }
 
//-------------------------------------------------------------------------------------------------------
/*void showmonth(){    // Отображать месяц (в меню установки времени и даты)
        myGLCD.print(F("      "), 160, 144);
        myGLCD.setFont(RusFont6);         // Выбор шрифта 
        myGLCD.setColor(255, 255, 255);  // белый шрифт
        myGLCD.setBackColor(0, 0, 0); // фон серый
  if (RTC.month==1){ myGLCD.print(print_text[163], 167, 144);}     // Января           
  if (RTC.month==2){ myGLCD.print(print_text[164], 167, 144);}     // Февраля                
  if (RTC.month==3){ myGLCD.print(print_text[165], 167, 144);}     // Марта                 
  if (RTC.month==4){ myGLCD.print(print_text[166], 167, 144);}     // Апреля              
  if (RTC.month==5){ myGLCD.print(print_text[167], 175, 144);}     // Мая             
  if (RTC.month==6){ myGLCD.print(print_text[168], 177, 144);}     // Июня                
  if (RTC.month==7){ myGLCD.print(print_text[169], 177, 144);}     // Июля            
  if (RTC.month==8){ myGLCD.print(print_text[170], 165, 144);}     // Августа               
  if (RTC.month==9){ myGLCD.print(print_text[171], 160, 144);}     // Сентября              
  if (RTC.month==10){ myGLCD.print(print_text[172], 160, 144);}    // Октября                
  if (RTC.month==11){ myGLCD.print(print_text[173], 168, 144);}    // Ноября                
  if (RTC.month==12){ myGLCD.print(print_text[174], 165, 144);}}   // Декабря 
 //------------------------------------------------------------------------------------------------------- 

/************************************* LED LEVELS *************************************/
void LED_levelo_output(){
  int sector, sstep, t1, t2 ;
 
if (colorLEDtest==true){ 
     wled_out = map(wcol_out, 0, 100, 0, 2000);   // new value 0-2000
     bled_out = map(bcol_out, 0, 100, 0, 2000);
    rbled_out = map(rbcol_out, 0, 100, 0, 2000);  // 1985 = 2000
     rled_out = map(rcol_out, 0, 100, 0, 2000);   
    uvled_out = map(uvcol_out, 0, 100, 0, 2000);
     oLed_out = map(ocol_out, 0, 100, 0, 2000);
     gled_out = map(grcol_out, 0, 100, 0, 2000);	
  moonled_out = map(mooncol_out, 0, 100, 0, 255); 
      } else {
  if (min_cnt>=1440) {min_cnt=1;}      // 24 hours of minutes   
        sector = min_cnt/15;           // divided by gives sector -- 15 minute
        sstep = min_cnt%15;            // remainder gives add on to sector value 
            t1 =sector;
     
  if (t1==95) {t2=0;} else {t2 = t1+1;}
//    if (debugon){ Serial.print(t1);
//                  Serial.print(F("__"));
//                  Serial.print(sstep);}
   if (sstep==0){
         wled_out = map(wled[t1],0, 100, 0, 2000);     // new value 0-2000 
         bled_out = map(bled[t1],0, 100, 0, 2000);     // new value 0-2000	
        rbled_out = map(rbled[t1],0, 100, 0, 2000);    // new value 0-2000
         rled_out = map(rled[t1],0, 100, 0, 2000);     // new value 0-2000
        uvled_out = map(uvled[t1],0, 100, 0, 2000);    // new value 0-2000
         oLed_out = map(oLed[t1],0, 100, 0, 2000);      // new value 0-2000
         gled_out = map(gled[t1],0, 100, 0, 2000);      // new value 0-2000

     } else {
//= Для 11 бит 
	   wled_out = check(wled[t1]*20, wled[t2]*20, sstep);
	   bled_out = check(bled[t1]*20, bled[t2]*20, sstep);
           rbled_out = check(rbled[t1]*20, rbled[t2]*20, sstep);
           rled_out = check(rled[t1]*20, rled[t2]*20, sstep);
           uvled_out = check(uvled[t1]*20, uvled[t2]*20, sstep);
	   oLed_out = check(oLed[t1]*20, oLed[t2]*20, sstep);
           gled_out = check(gled[t1]*20, gled[t2]*20, sstep); } 

       float lunarCycle = moonPhase(RTC.year, RTC.month, RTC.day);             // get a value for the lunar cycle
       moonled_out = map ((MinI *(1 - lunarCycle) + MaxI * lunarCycle + 0.5), 0, 100, 0, 255); }  
  
   if (setDimLEDsOnOff==1) {HOT_LEDs();}  // 
               Soft_Start_Led();          // increase led brigtness after start programm during 50sec
  
// Проверка отключен ли канал LED 
// check channel status ON/OFF
// if reader bit =0 LedShannelFlag_on_off = false, else LedShannelFlag_on_off = true;
// WHITE=1-bit 0, BLUE=2-bit 1, ROYAL=3-bit 2, RED=4-bit 3, ULTRA=5-bit 4, ORANGE=6-bit 5, GREEN=8-bit 6

  if (bitRead(LedShannelStatusByte,0)== false) {wled_out=0;}  
  if (bitRead(LedShannelStatusByte,1)== false) {bled_out=0;}
  if (bitRead(LedShannelStatusByte,2)== false) {rbled_out=0;}  
  if (bitRead(LedShannelStatusByte,3)== false) {rled_out=0;}  
  if (bitRead(LedShannelStatusByte,4)== false) {uvled_out=0;}  
  if (bitRead(LedShannelStatusByte,5)== false) {oLed_out=0;}  
  if (bitRead(LedShannelStatusByte,6)== false) {gled_out=0;}   


// Диммирование вручную (адаптация)----- Dimm
    if (DimmL == 1) {
           PercentDimL = setLEDsDimPercentL*0.01;
           wled_out = PercentDimL*wled_out;
           bled_out = PercentDimL*bled_out;
           rbled_out = PercentDimL*rbled_out;                        
           rled_out = PercentDimL*rled_out;
           uvled_out = PercentDimL*uvled_out;
           oLed_out = PercentDimL*oLed_out;
           gled_out = PercentDimL*gled_out;
             
 if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){
               setFont(SMALL, 0, 224, 134, 0, 0, 0);
               myGLCD.print(F("Dimm:"), 66, 30);           // отображения уровней в главном экране
               myGLCD.printNumI(setLEDsDimPercentL, 106, 30);
               myGLCD.print(print_text[142], 125, 30);} }  // %

  if (RECOM_RCD == true){   // обычное управление 0 - 2000
                w_out = wled_out;
                b_out = bled_out;
               rb_out = rbled_out;
                r_out = rled_out;
               uv_out = uvled_out;
                o_out = oLed_out;
               gr_out = gled_out;
             moon_out = moonled_out;
              } else {     //  инверсное управление 2000 - 0
                   w_out = 2000 - wled_out;   // (11бит) 
                   b_out = 2000 - bled_out;
                  rb_out = 2000 - rbled_out;
                   r_out = 2000 - rled_out;
                  uv_out = 2000 - uvled_out;
                   o_out = 2000 - oLed_out;
                  gr_out = 2000 - gled_out;
                    moon_out = moonled_out; } 

//----------- 11 bit PWM outputs 
  if (w_out)  sbi_mix(TCCR4A, COM4B1); else cbi_mix(TCCR4A, COM4B1); // T4B port 7
  if (b_out)  sbi_mix(TCCR3A, COM3C1); else cbi_mix(TCCR3A, COM3C1); // T3C port 3
  if (rb_out) sbi_mix(TCCR4A, COM4C1); else cbi_mix(TCCR4A, COM4C1); // T4C port 8
  if (r_out)  sbi_mix(TCCR4A, COM4A1); else cbi_mix(TCCR4A, COM4A1); // T4A port 6
  if (uv_out) sbi_mix(TCCR1A, COM1B1); else cbi_mix(TCCR1A, COM1B1); // T1B port 12
  if (o_out)  sbi_mix(TCCR3A, COM3A1); else cbi_mix(TCCR3A, COM3A1); // T3A port 5   
  if (gr_out) sbi_mix(TCCR3A, COM3B1); else cbi_mix(TCCR3A, COM3B1); // T3B port 2      

// reload Output Compare Register 
  OCR4B = w_out;       
  OCR3C = b_out;		
  OCR4C = rb_out;       
  OCR4A = r_out;       
  OCR1B = uv_out;		
  OCR3A = o_out;       
  OCR3B = gr_out;		 
  
  analogWrite(ledPinMoon, moon_out);  // 8 bit MOON PWM

  
     //************************Подсветка экрана*********************************   
 //  byte bright = 40;  //analogRead(sensLight);
 //  byte tmpLCDbright = map(bright, 1, 255, 1, 100);
//   setFont(SMALL, 255, 151, 48, 0, 0 , 0);
//   myGLCD.printNumI(tmpLCDbright, 10, 10);
//  Serial.print ("Sens  "); Serial.println (bright);
  byte bout = map(tmpLCDbright, 1, 100, 1, 255);  // подсветка экрана
//  Serial.print ("   Light  "); Serial.println (bout);
   analogWrite(LCDbrightPin, bout);}
  //******************************************************************************

// pt1 и pt2 от чего и к чему диммируем(границы), lstep сколько минут прошло от начала интервала          
int check( int pt1, int pt2, int lstep){
  int result;
  float fresult;
   
 if (pt1==pt2) {result = pt1;} else        // No change
 if (pt1<pt2){                             // Increasing brightness
     fresult = ((float(pt2-pt1)/900) * float(lstep*60+RTC.second))+float(pt1);
     result = int(fresult); }
                                               // Decreasing brightness
    else {fresult = -((float(pt1-pt2)/900) * float(lstep*60+RTC.second))+float(pt1);
          result = int(fresult); } return result; }   

/******************************** TEMPERATURE FUNCTIONS *******************************/
void checkTempC(){
  
    if (millis() - lastTempRequest >= delayInMillis){     // waited long enough??
                  digitalWrite(A15, LOW);                 // вкл / выкл вентелятора
      
    tempW = (sensors.getTempC(waterThermometer));         // чтение датчика температуры воды 
    tempH1 = (sensors.getTempC(Heatsink1Thermometer));    // read Heatsink1's heatsink temperature
    tempH2 = (sensors.getTempC(Heatsink2Thermometer));    // read Heatsink2's heatsink temperature 
    
    delayInMillis = 750 / (1 << (12 - resolution));
    lastTempRequest = millis(); 
                  digitalWrite(A15, HIGH);                 // вкл / выкл вентелятора
              sensors.requestTemperatures();               // temperature request to all devices on the bus
    }
//--------------- Чтение температуры Воды ------------------
    if (counterB1 ==1 || counterB2 ==1 || counterB3 ==1){  // check Water temperature if B1 or B2 or B3 = 1 (Water) 
    if (tempW == -127 || tempW == -196){                   // sensor disconnected
	    digitalWrite(tempHeatPin, LOW);		   // off heater and chiller
            analogWrite(tempChillPin, LOW);          
		 tempAlarmflag=true;                       // turn on alarm
		 tempCoolflag=false;
                 tempHeatflag=false; }
        
  if (tempW<(setTempC+offTempC+alarmTempC) && tempW>(setTempC-offTempC-alarmTempC)){ // turn off alarm after OverHeating/OverCooling			  
	 tempAlarmflag=false;
	 AlarmflagON=false;
	 digitalWrite(tempAlarmPin, LOW); }        // OFF alarm
		 
  if (tempW<(setTempC+offTempC) && tempW>(setTempC-offTempC)){  // turn off chiller / heater, Water temp is NORMAL T< SET+offset
        tempCoolflag=false;
        tempHeatflag=false;
        digitalWrite(tempHeatPin, LOW);
        analogWrite(tempChillPin, LOW); }

   if (offTempC>0){
       if (tempW >=(setTempC+offTempC)){         // холодильник  
        tempCoolflag=true;
        digitalWrite(tempChillPin, HIGH);        // turn on chiller pin
        digitalWrite(tempHeatPin, LOW); }        // turn off heater pin
          
   if (tempW<=(setTempC-offTempC)){              // нагреватель
        tempHeatflag=true;
        digitalWrite(tempHeatPin, HIGH);         // turn on heater pin
	analogWrite(tempChillPin, LOW); }}      // turn off chiller pin
          
  if (alarmTempC>0){                             // turn on alarm
     if ((tempW>=(setTempC+offTempC+alarmTempC)) || (tempW<=(setTempC-offTempC-alarmTempC))){
          tempAlarmflag = true; } }
       
//----- максимум температуры для датчика в аквариуме -----
  	 if (tempW >= MaxTempW){ MaxTempW = tempW;}                    // store max temp
         if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){ // display MAX TEMP
	      setFont(SMALL, 255, 0, 204, 0, 0, 0);                    // шрифт красный
              myGLCD.printNumF(MaxTempW, 1, 270,133);}
             }  else {digitalWrite(tempHeatPin, LOW);                 // выключить нагреватель 
	               analogWrite(tempChillPin, LOW);                // выключить холодильник 
		           tempCoolflag=false;
		           tempHeatflag=false; }

//------------ Fan Controller for Heatsink1 ---------------
  if (counterB1 ==2 || counterB2 ==2 || counterB3 ==2){               // check Heatsink 1 temperature if B1 or B2 or B3 = 2	
      Heatsink1TempInterval = (tempH1 - setTempToBeginHeatsink1FanC); // Sets the interval to start from 0
  if (Heatsink1TempInterval >=20){Heatsink1TempInterval =20; }        // Set maximal value PWM =255 if current temp >= setTempToBeginHeatsink2FanC+20C
  if (Heatsink1TempInterval <= -1) {Heatsink1PWM = 0;  // 255
	         analogWrite(Heatsink1_FansPWM, Heatsink1PWM); 
		 bitClear(GlobalStatus1Byte,6);                       // GlobalStatus1Byte.6 = StartUPFan1 ;
		 bitClear(GlobalStatus1Byte,7);}                      // GlobalStatus1Byte.6 = StartUPFan1_Timeout;
                          
  if (Heatsink1TempInterval >=1){
      Heatsink1PWM = map (Heatsink1TempInterval, -1 , 20, 5, 200);  // maximum PWM = setTempToBeginHeatsink2FanC+20C, minimum speed 40% of max
  if (bitRead(GlobalStatus1Byte,7) == false) {                        // set flag startup process
            analogWrite(Heatsink1_FansPWM, 255); // 255

            unsigned long cMillis = millis();
            previousMillisAlarm = cMillis;                            // reset 1 sec timer for fan startup process 
	    bitSet(GlobalStatus1Byte,6);}
	else { analogWrite(Heatsink1_FansPWM, Heatsink1PWM); }}}
  
//------------------------ Sound alarm Section ------------------------------  
 if (tempH1 >= setTempToSoundAlarmC || tempH1 == -127) {tempAlarmflagH1 = true;
	                                  analogWrite(Heatsink1_FansPWM, 255);}      // set maximum fan speed
	   else {tempAlarmflagH1 = false; digitalWrite(tempAlarmPin, LOW);}	     // OFF alarm

//----------- Максимум температуры за день для датчика на радиаторе 1 ------
   if (tempH1 >= MaxTempH1){ MaxTempH1 = tempH1;}                                    // store max temp
          if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){              // display MAX Temp      
	      setFont(SMALL, 255, 151, 48, 0, 0 , 0); // красный (морковный)
              myGLCD.printNumF(MaxTempH1, 1, 270 ,145); } 
              
//-------------- Fan Controller for Heatsink 2 ----------------
  if (counterB1 ==3 || counterB2 ==3 || counterB3 ==3){ 	      // check Heatsink 2 temperature if B1 or B2 or B3 = 3      
     Heatsink2TempInterval = (tempH2 - setTempToBeginHeatsink2FanC);  // Sets the interval to start from 0
     if (Heatsink2TempInterval >=20 ){Heatsink2TempInterval =20; } // Set maximal value PWM =255 if current temp >= setTempToBeginHeatsink2FanC+20C
     if (Heatsink2TempInterval <= -1) { Heatsink2PWM =0;  // 
	     analogWrite(Heatsink2_FansPWM, Heatsink2PWM);  
	     bitClear(GlobalStatus1Byte,4);                 // GlobalStatus1Byte.4 = StartUPFan2 ;
	     bitClear(GlobalStatus1Byte,5); }               // GlobalStatus1Byte.5 = StartUPFan2_Timeout;
                                
 if (Heatsink2TempInterval >=1){         //                100  255
       Heatsink2PWM = map (Heatsink2TempInterval, -1 , 20, 5, 200);  // maximum PWM = setTempToBeginHeatsink2FanC+20C
      // Serial.println(Heatsink2PWM);
      
 if (bitRead(GlobalStatus1Byte,5) == false) {			       // set flag startup process 
            analogWrite(Heatsink2_FansPWM, 255);  // 255
            unsigned long cMillis = millis();       
            previousMillisAlarm = cMillis;			       // reset 1 sec timer for fan startup process 
	    bitSet(GlobalStatus1Byte,4); }
    else { analogWrite(Heatsink2_FansPWM, Heatsink2PWM); }}}
  
//---------------------- Sound alarm Section ------------------------------
      if (tempH2 >= setTempToSoundAlarmC || tempH2 == -127) {tempAlarmflagH2 = true;
	                           analogWrite(Heatsink2_FansPWM, 255);}          // set maximum fan speed
	   else {tempAlarmflagH2 = false; digitalWrite(tempAlarmPin, LOW);}	  // OFF alarm

//----------- Максимум температуры за день для датчика на радиаторе 2 ------
    if (tempH2 >= MaxTempH2){ MaxTempH2 = tempH2;}                               // store max temp
      if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){              // display max temp 
	  setFont(SMALL, 255, 151, 48, 0, 0 , 0);    // красный (морковный)                            
          myGLCD.printNumF(MaxTempH2, 1, 270 ,157); } 
          digitalWrite(A15, HIGH); // включение вентелятора 2
          sensors.requestTemperatures();					  // call sensors.requestTemperatures() to issue a global 
        									  // temperature request to all devices on the bus
//-------- Clear Max Temp every Day -- очистка максимумов температур --------
     if (min_cnt/15 == StartTime){ MaxTempW=0; MaxTempH1=0; MaxTempH2=0; } 
     
// ------------ запоминание температуры по времени для ЛОГ и ЛОГ Темп1,2,3 ----------------
//                            V-колличество минут в сутки
            for ( int i=0; i<1440; i+=30 ){
    if (timer==i) {media[(i-30)/30+1]=tempW*10;}}       // температура воды в аквариуме   
            for ( int i=0; i<1440; i+=30 ){             // температура радиатора датч 1, 2
    if (timer==i){tF1[(i-30)/30+1]=tempH1*10; tF2[(i-30)/30+1]=tempH2*10; } } }  

void Alarm(){   // Тревога
          unsigned long cMillis = millis();
     if (cMillis - previousMillisAlarm > 700){
          previousMillisAlarm = cMillis;
          
//----------------- FAN start UP ------------------------
  	if (bitRead(GlobalStatus1Byte,6) == true){ // startup for FAN1
	    bitClear(GlobalStatus1Byte,6);
	    bitSet(GlobalStatus1Byte,7); }  

	if (bitRead(GlobalStatus1Byte,4) == true){ // startup for FAN2
	    bitClear(GlobalStatus1Byte,4);
	    bitSet(GlobalStatus1Byte,5); }  

//------------- Sound Alarm -------------------------- звуковая тревога		
    if (counterB1 + counterB2 + counterB3 !=0){  // no sound alarm if dallas sensor not installed, ie counterB1=counterB2=counterB3=0
    if (tempAlarmflag == true || tempAlarmflagH1 == true || tempAlarmflagH2 == true){       
    if (AlarmflagON == true){ digitalWrite(tempAlarmPin, HIGH);   // ON alarm
        AlarmflagON = false; } else {
    if (AlarmflagON == false){ digitalWrite(tempAlarmPin, LOW);   // OFF alarm
        AlarmflagON = true; }}}}}}
        
void Speed_Fan(){  // Скорость вентеляторов в главном экране   
//------------ Скорость вентилятора на радиаторе датчик 1 ------------- 

if ((dispScreen == 0) && (screenSaverCounter<setScreenSaverTimer)){
          myGLCD.setFont(RusFont1);
          myGLCD.setColor(0, 255, 0);  // green
          myGLCD.setBackColor(0, 0, 0);
          myGLCD.print(F("#"), 270, 18);  // вентелятор
          myGLCD.print(F("#"), 270, 28); // 2
          myGLCD.setColor(255, 255, 255);
          myGLCD.print(F("1"), 283, 18);  // 1
          myGLCD.print(F("2"), 283, 28);  // 2 номер датчика

  if (Heatsink1PWM !=0){myGLCD.setColor(0, 250, 0);
       Heatsink1PWM = map (Heatsink1PWM, 0 , 255, 1, 99);
      if (Heatsink1PWM < 10) {myGLCD.print(" ", 294, 18);myGLCD.printNumI( Heatsink1PWM, 302, 18);}
         else { myGLCD.printNumI( Heatsink1PWM, 294, 18); } // вентелятор 1
           myGLCD.print(print_text[142], 310, 18);      
     } else { myGLCD.setColor(255, 0, 0);    
             strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
         myGLCD.print(buffer, 294, 18); }   // Вык 
         
   if (Heatsink2PWM !=0){myGLCD.setColor(0, 250, 0); 
         Heatsink2PWM = map (Heatsink2PWM, 0 , 255, 1, 99);
         if (Heatsink2PWM < 10) {myGLCD.print(" ", 294, 28);myGLCD.printNumI( Heatsink2PWM, 302, 28);} 
          else { myGLCD.printNumI( Heatsink2PWM, 294, 28);}   // вентелятор 2
           myGLCD.print(print_text[142], 310, 28);
       } else { myGLCD.setColor(255, 0, 0);     
               strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171])));
           myGLCD.print(buffer, 294, 28);}    // Вык 
      
 if (tempH1 == -127 || tempH1 == -196){ // при обрыве или неподключенном датчике
      myGLCD.setFont(SmallFont); 
      myGLCD.setColor(255, 153, 153);
      analogWrite(Heatsink1_FansPWM, 255); 
      myGLCD.print("max", 294, 15);}   // вентелятор 1 максимальная скорость 
      
 if (tempH2 == -127 || tempH2 == -196){
      myGLCD.setFont(SmallFont); 
      myGLCD.setColor(255, 153, 153);
      analogWrite(Heatsink2_FansPWM, 255); 
      myGLCD.print("max", 294, 25);}} // вентелятор 2 максимальная скорость   
  
}
/***************************УРОВЕНЬ ВОДЫ В ГЛАВНОМ ЭКРАНЕ***********(в разработке)****************/     
void waterlevels(){

if ((dispScreen == 0) && (screenSaverCounter<setScreenSaverTimer)){
if (waterlevel == 0) {
 myGLCD.setColor(0, 0, 0);
       myGLCD.fillRoundRect(121, 171, 157, 177);
       myGLCD.fillRoundRect(121, 181, 157, 187);
 myGLCD.setColor(200, 0, 0);      
       myGLCD.fillRoundRect(121, 191, 157, 197);}
 else      
if (waterlevel == 1) {
 myGLCD.setColor(0, 0, 0);
       myGLCD.fillRoundRect(121, 171, 157, 177);
 myGLCD.setColor(0, 150, 0);       
       myGLCD.fillRoundRect(121, 181, 157, 187);      
       myGLCD.fillRoundRect(121, 191, 157, 197);}
  else     
if (waterlevel == 2) {
 myGLCD.setColor(200, 0, 0);
       myGLCD.fillRoundRect(121, 171, 157, 177);
       myGLCD.fillRoundRect(121, 181, 157, 187);      
       myGLCD.fillRoundRect(121, 191, 157, 197);} 
     
}}
    
       
//************ Soft Led Start function **************
void Soft_Start_Led(){  // increase led brigtness after start programm during 50sec

  if (PercentSoftStart < 1.1){
       wled_out = PercentSoftStart*wled_out;
       bled_out = PercentSoftStart*bled_out;   
       rbled_out = PercentSoftStart*rbled_out;
       rled_out = PercentSoftStart*rled_out;
       uvled_out = PercentSoftStart*uvled_out;     
       oLed_out = PercentSoftStart*oLed_out;
       gled_out = PercentSoftStart*gled_out;
	   PercentSoftStart +=0.1; } }        

/*********** DIM LEDs WHEN HOT FUNCTION *************************** Уменьшение яркости каналов при перегреве */ 
void HOT_LEDs(){
//     датчик на радиаторе 1        датчик на радиаторе 2             датчик в аквариуме   
 if (tempH1>=setLEDsDimTempC+1 || tempH2>=setLEDsDimTempC+1){  //|| tempW>=setLEDsDimTempC+1){
         bitSet(GlobalStatus1Byte,1);}
    
 if (tempH1<=setLEDsDimTempC-1 && tempH2<=setLEDsDimTempC-1){  //&& tempW<=setLEDsDimTempC-1){
         bitClear(GlobalStatus1Byte,1);}

 if (bitRead(GlobalStatus1Byte,1) == true){  
       PercentDim = setLEDsDimPercent*0.01;
       wled_out = PercentDim*wled_out;     // белый 
       bled_out = PercentDim*bled_out;    
       rbled_out = PercentDim*rbled_out;
       rled_out = PercentDim*rled_out;
       uvled_out = PercentDim*uvled_out;
       oLed_out = PercentDim*oLed_out; // оранжевый
       gled_out = PercentDim*gled_out; 
       
        screenSaverCounter=0; // обнулить счетчик хранителя экрана
      //setDimLEDsOnOff = 0; // выкл. Dimm leds
      //setDimLEDsOnOff = 1; // вкл.
  if (dispScreen == 0){
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRoundRect(265, 144, 302, 169); // макс температура очистить 
        setFont(LARGE, 255, 76, 0, 0, 0, 0);
        myGLCD.drawRect(265, 144, 302, 169); // красная рамка вокруг макс. темп        
        myGLCD.printNumI(setLEDsDimPercent, 80, 63);  // мощность в процентах
        myGLCD.print(print_text[142], 110, 63);       // %      
             strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[33]))); 
        // myGLCD.print(buffer, 48, 44);       // ALARM!!  (на каналах уровней)   
         myGLCD.print(buffer, 192, 185);      // ALARM!!  (внизу)
         myGLCD.print(F("POWER!!"), 48, 44);  // POWER!!
         myGLCD.setFont(RusFont6);
         myGLCD.setColor(80, 255, 255); // бирюзовый
         myGLCD.print("OEPECPEB", 213, 205); } } } // ПЕРЕГРЕВ РАДИАТОРА "OEPECPEB P@DH@RNP@", 180, 205);
       
/******************************* LUNAR PHASE FUNCTION *********************************/  
float moonPhase(int moonYear, int moonMonth, int moonDay){ 
    float phase; double IP;
  
   long YY, MM, K1, K2, K3, JulianDay; 
     YY = moonYear - floor((12 - moonMonth) / 10); 
     MM = moonMonth + 9;
 if (MM >= 12) { MM = MM - 12; }
     K1 = floor(365.25 * (YY + 4712));
     K2 = floor(30.6 * MM + 0.5);
     K3 = floor(floor((YY / 100) + 49) * 0.75) - 38;
     JulianDay = K1 + K2 + moonDay + 59;
 if (JulianDay > 2299160) { JulianDay = JulianDay - K3; }
     IP = MyNormalize((JulianDay - 2451550.1) / LC);
     AG = IP*LC; phase = 0; 
  
// Determine the Moon Illumination %
 if ((AG >= 0) && (AG <= LC/2)){ phase = (2*AG)/LC; }   // FROM New Moon 0% TO Full Moon 100%
 if ((AG > LC/2) && (AG <= LC)){ phase = 2*(LC-AG)/LC;} // FROM Full Moon 100% TO New Moon 0%
      
// Determine the Lunar Phase
if ((AG >= 0)&&(AG <= 1.85)){LP= print_text[112]; MoonPic= pgm_get_far_address(First_Quarter);}
//MoonPic= pgm_get_far_address(New_Moon);}         // ~0-12.5% Новолуние
if ((AG > 1.85)&&(AG <= 5.54)){LP= print_text[113]; MoonPic= pgm_get_far_address(First_Quarter);}
//MoonPic= pgm_get_far_address(New_Moon);}
//}//MoonPic= pgm_get_far_address(Waxing_Crescent);}// ~12.5-37.5% Растущая
if ((AG > 5.54) && (AG <= 9.23)){LP= print_text[114]; MoonPic= pgm_get_far_address(First_Quarter);}  // ~37.5-62.5% 1 Четверть 
if ((AG > 9.23)&&(AG <= 12.92)){LP= print_text[115]; MoonPic= pgm_get_far_address(First_Quarter);}
//}//MoonPic= pgm_get_far_address(Waxing_Gibbous);}// ~62.5-87.5% Растущая
if ((AG > 12.92)&&(AG <= 16.61)){LP= print_text[116]; MoonPic= pgm_get_far_address(Full_Moon);}      // ~87.5-100-87.5% Полнолуние   
if ((AG > 16.61)&&(AG <= 20.30)){LP= print_text[117]; MoonPic= pgm_get_far_address(Full_Moon);}
//}//MoonPic= pgm_get_far_address(Waning_Gibbous);}// ~87.5-62.5% Убывающая   
if ((AG > 20.30)&&(AG <= 23.99)){LP= print_text[118]; MoonPic= pgm_get_far_address(Full_Moon);}
//}//MoonPic= pgm_get_far_address(Last_Quarter);}  // ~62.5-37.5% 3 Четверть
if ((AG > 23.99)&&(AG <= 27.68)){LP= print_text[117]; MoonPic= pgm_get_far_address(Full_Moon);}
//}//MoonPic= pgm_get_far_address(Waning_Crescent);}// ~37.5-12.5% Убывающая
if ((AG >= 27.68)&&(AG <= LC)){LP= print_text[112]; MoonPic= pgm_get_far_address(First_Quarter);} 
//MoonPic= pgm_get_far_address(New_Moon);}        // ~12.5-0% Новолуние 

  lunar_perc = phase*100; return phase; }

double MyNormalize(double v){ v = v - floor(v);
  if (v < 0) v = v + 1; return v; } 

/********************************* MISC. FUNCTIONS ************************************/
void clearFscreen(){   // очистить полностью весь экран
   myGLCD.setColor(0, 0, 0);
   myGLCD.fillRect(1, 0, 318, 226); }

void clearScreen(){   // очистить экран (кроме верхнего и нижнего бара)
   myGLCD.setColor(0, 0, 0);
   myGLCD.fillRect(1, 13, 318, 226); }

void printButGreen(char* buton){    // рамка вкл в меню таймеров (вверху)
   myGLCD.setFont(RusFont2);  // 3
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(8, 17, 142, 34);  // белая кайма вокруг зелёной ON
   myGLCD.setColor(70, 200, 0);           // цвет зеленый для ON  
   myGLCD.fillRoundRect(9, 18, 141, 33);  // размер кнопки ON
   myGLCD.setColor(0, 0, 0);              // текст черный
   myGLCD.setBackColor(70, 200, 0);
   myGLCD.print(buton, 16, 21); }  // текст  (buton, 16, 23);

void printButRed(char* butoff){    // рамка выкл в меню таймеров (вверху)
   myGLCD.setFont(RusFont2);  // 3
   myGLCD.setColor(255, 255, 255);
   myGLCD.drawRoundRect(173, 17, 312, 34); // белая кайма вокруг красной OFF
   myGLCD.setColor(255, 0, 0);             // цвет красный для OFF   
   myGLCD.fillRoundRect(174, 18, 311, 33); // размер кнопки OFF
   myGLCD.setColor(0, 0, 0);               // текст черный
   myGLCD.setBackColor(255, 0, 0);
   myGLCD.print(butoff, 179, 21); }  // текст  (butoff, 179, 23); 
  
void printPres(char* pres){ // preset п1-п4
    myGLCD.setColor(0, 0, 255);
    myGLCD.drawRect(271, 88, 313, 103);
    myGLCD.setFont(RusFont6);
    myGLCD.setColor(0, 255, 0);
    myGLCD.setBackColor( 0, 0, 0); 
    myGLCD.print(pres, 281, 90); } 

void printPresoff(char* presoff){ // preset off
    myGLCD.setColor(68, 68, 68);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.drawRect(271, 88, 313, 103);
    myGLCD.setFont(RusFont6); 
    myGLCD.print(presoff, 277, 90); } 
    
void printHeader(){      // верхний баннер (желтая рамка)
    myGLCD.setFont(RusFont3);       // шрифт русский
    myGLCD.setColor(255, 255, 0);   // цвет желтый  (255, 255, 0);
    myGLCD.setBackColor(255, 255, 0);
    myGLCD.fillRect (1, 0, 318, 13);
    myGLCD.setColor(255, 0, 0);      // цвет красный 
    myGLCD.drawRect(1, 0, 318, 13);  // рамка     
    myGLCD.setColor(0, 0, 0);           // шрифт текста черный
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Header_Text_table[PrintStringIndex]))); 
    myGLCD.print(buffer, CENTER, 3 ); } // текст 4

void printFramework(){ // timer
     myGLCD.setColor(95, 95, 95);              // цвет серый 
     myGLCD.drawRoundRect(21, 93, 65, 129);    // кайма вокруг часов включения
     myGLCD.drawRoundRect(89, 93, 133, 129);   // кайма вокруг минут
     myGLCD.drawRoundRect(187, 93, 230, 129);  // кайма вокруг часов выключения
     myGLCD.drawRoundRect(256, 93, 300, 129);  // кайма вокруг минут
     myGLCD.drawRoundRect(10, 40, 142, 183);   // белая кайма вокруг времени включения 
     myGLCD.drawRoundRect(8, 38, 144, 185);    // вторая кайма
     myGLCD.drawRoundRect(175, 40, 310, 183);  // кайма вокруг времени выключения
     myGLCD.drawRoundRect(173, 38, 312, 185);  // вторая кайма
     myGLCD.drawRoundRect(149, 38, 168, 66);}  // кайма вокруг номера канала
	  
void printPicture(){ // картики стрелок таймеров
     myGLCD.drawBitmap(20, 44, 48, 31, up, 1);    // cтрелка вверх часы включения
     myGLCD.drawBitmap(20, 149, 48, 31, down, 1); // cтрелка вниз часы включения  
     myGLCD.drawBitmap(89, 44, 48, 31, up, 1);    // cтрелка вверх часы включения
     myGLCD.drawBitmap(89, 149, 48, 31, down, 1); // cтрелка вниз часы включения 
     
     myGLCD.drawBitmap(185, 44, 48, 31, up, 1);     // cтрелка вверх часы выключения
     myGLCD.drawBitmap(185, 149, 48, 31, down, 1);  // cтрелка вниз часы выключения 
     myGLCD.drawBitmap(255, 44, 48, 31, up, 1);     // cтрелка вверх часы выключения
     myGLCD.drawBitmap(255, 149, 48, 31, down, 1);} // cтрелка вниз часы выключения 
/*     
void printPicturedos(){ // картики стрелок дозаторов
     myGLCD.drawBitmap(valUP[0]+1, valUP[1]+1, 48, 31, pgm_get_far_address(up), 1);    // cтрелка вверх 
     myGLCD.drawBitmap(valDOWN[0]+1, valDOWN[1]+1, 48, 31, pgm_get_far_address(down), 1); // cтрелка вниз   
     myGLCD.drawBitmap(calUP[0]+1, calUP[1]+1, 48, 31, pgm_get_far_address(up), 1);    // cтрелка вверх 
     myGLCD.drawBitmap(calDOWN[0]+1, calDOWN[1]+1, 48, 31, pgm_get_far_address(down), 1);} // cтрелка вниз  
*/     
     
void printTimernumber(char* tnumber){ // обозначить номер таймера
     setFont(LARGE, 255, 255, 255, 0, 0, 0);
     myGLCD.print(print_text[56], 70, 103);   // : 
     myGLCD.print(print_text[56], 236, 103);  // : 
     myGLCD.setFont(DotMatrix_M_Num); // шрифт DotMatrix
     myGLCD.setColor(88, 255, 113);   // цвет зелёный
     myGLCD.setBackColor(0, 0, 0);     
     myGLCD.print(tnumber, 151, 41); } 

void setFont(boolean font, byte cr, byte cg, byte cb, byte br, byte bg, byte bb){ 
    myGLCD.setBackColor(br, bg, bb);                // font background black
    myGLCD.setColor(cr, cg, cb);                    // font color white
 if (font==LARGE) myGLCD.setFont(BigFont); else     // font size LARGE
 if (font==SMALL) myGLCD.setFont(SmallFont); }      // маленький фонт

void waitForTouchRelease(){
  while (myTouch.dataAvailable()==true)     // Wait for release
    myTouch.read(); }

 int LedToPercent (int Led_out, int resolution){ // returns LED output in % with rounding to the nearest whole number

  int result;
  float Temp_in = Led_out;
  float Temp = 100*Temp_in/resolution;
  result  = map(Led_out, 0, resolution, 0, 100); 
  if ((Temp - result) >= 0.5) {result +=1;};      // rounding to the nearest whole number  
          return result; }

 void SmallLedBarGraph(int Led_out, int resolution, byte shift_X, byte cR, byte cG, byte cB){
     int bar = map(Led_out, 0, resolution, 83, 33);   // Top end bar - Y=31, Bot end bar Y=131 
  if (bar <33 ){bar=33;}
  if (bar >83){bar=83;}
    myGLCD.setColor(0, 0, 0);                     // V -    
    myGLCD.fillRect(12+shift_X, bar, 12+18+shift_X, 33);   // hide end of last bar (Top end bar)
	myGLCD.setColor(cR, cG, cB); //  ^ - ширина бара           
    myGLCD.drawRect(12+shift_X, 83, 12+18+shift_X, 83);  // %bar place holder
    myGLCD.fillRect(12+shift_X, 83, 12+18+shift_X, bar);} // percentage bar
                      //                 ^ - 

void drawBarGraph(){                        // графика уровней каналов в главном экране  

  setFont(SMALL, 255, 255, 255, 0, 0, 0);     
  myGLCD.drawRect(3, 86, 164, 86);       // x-line  (30, 148, 164, 148);
 for (int i=0; i<5; i++){myGLCD.drawLine(2,(i*11)+36,6,(i*11)+36);} // tick-marks big 
       myGLCD.setColor(190, 190, 190);  
 for (int i=0; i<4; i++){myGLCD.drawLine(2,(i*11)+42,4,(i*11)+42);} // tick-marks small       
// (пунктир) 
myGLCD.setColor(90, 90, 90); //               V - 55
  for(int i=36; i<86; i++){ myGLCD.drawPixel(10,i);i=i+3;} 
//           ^-верх^ - размер пунктира 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(32,i);i=i+3;} 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(54,i);i=i+3;} 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(76,i);i=i+3;} 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(98,i);i=i+3;} 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(120,i);i=i+3;} 
  for(int i=36; i<86; i++){ myGLCD.drawPixel(142,i);i=i+3;}
  for(int i=36; i<86; i++){ myGLCD.drawPixel(164,i);i=i+3;}  

}
//==================== Слайдерное ручное управление ======================
void drawSliderBarGraph(){ 
           
  setFont(SMALL, 255, 255, 255, 0, 0, 0); // шрифт белый, фон черный  
  myGLCD.drawRect(30, 173, 315, 173);     // x-line
  myGLCD.drawRect(30, 173, 30, 44);       // y-line  
  myGLCD.setColor(190, 190, 190);
  
  for (int i=0; i<10; i++){               // tick-marks шкала процентов  
       myGLCD.drawLine(31, (i*13)+44, 37, (i*13)+44); } 
  for (int i=0; i<10; i++){               // вторая шкала
       myGLCD.drawLine(31, (i*13)+51, 34, (i*13)+51); } 

   myGLCD.setFont(RusFont1); // нарисовать шкалу процентов
   myGLCD.print(print_text[46], 5, 41);    // 100
   myGLCD.print(print_text[45], 12, 54);   // 90
   myGLCD.print(print_text[44], 12, 67);   // 80
   myGLCD.print(print_text[43], 12, 80);   // 70
   myGLCD.print(print_text[42], 12, 93);   // 60
   myGLCD.print(print_text[41], 12, 106);  // 50
   myGLCD.print(print_text[40], 12, 119);  // 40
   myGLCD.print(print_text[39], 12, 132);  // 30
   myGLCD.print(print_text[38], 12, 145);  // 20
   myGLCD.print(print_text[37], 12, 158);  // 10
   myGLCD.print(print_text[187], 20, 171); // 0

   myGLCD.setColor(180, 180, 180);                  
   for (int i=0; i<10; i++){                      // горизонтальные пунктирные линии шкалы  
   for(int k=46; k<311; k++){ myGLCD.drawPixel(k,(i*13)+44); k=k+2;} }
//------------ 39    308
    myGLCD.setFont(SmallFont);
    myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);     // цвет белый 
    myGLCD.drawRect(49, 44, 79, 172);   // БЕЛЫЙ %bar place holder 
    myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);     // цвет голубой 
    myGLCD.drawRect(87, 44, 117, 172);  // ГОЛУБОЙ %bar place holder
    myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);       // цвет синий 
    myGLCD.drawRect(125, 44, 155, 172); // РОЯЛЬ %bar place holder 
    myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);         // цвет красный 
    myGLCD.drawRect(163, 44, 193, 172); // КРАСНЫЙ %bar place holder;
    myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);     // цвет фиолетовый
    myGLCD.drawRect(201, 44, 231, 172); // UV %bar place holder     
    myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);      // цвет оранжевый 
    myGLCD.drawRect(239, 44, 269, 172); // ОРАНЖЕВЫЙ %bar place holder
    myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);         // цвет зеленый 
    myGLCD.drawRect(277, 44, 307, 172); // ЗЕЛЕНЫЙ %bar place holder 

   myGLCD.drawBitmap(10, 190, 24, 24, clos, 1); // картинка красный крестик (выход) (7, 187, 24, 24, clos, 1);

}
//===========================================================================
void SliderBars(){ // ========= Слайдер ручной настройки яркости =========
  int TempY;
  TempY = map(y, 172, 44, 0, 100); tSlide=TempY; // 0-255
     
  myGLCD.setColor(sbR, sbG, sbB);                      // Slider Bar Color
  myGLCD.fillRect(sbX1, y, sbX2, 172);                 // draw the bar where you touch  
  myGLCD.setColor(0, 0, 0);  
  myGLCD.fillRect(sbX1+1, y, sbX2-1, 45);              // hide the bar  
  myGLCD.setColor(sbR, sbG, sbB);                      // Slider Bar Color
  myGLCD.drawLine(sbX1, 44, sbX2, 44);                 // fills in the top of bar   
  setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
  
  if (y<=44) {y=44;}   
  if (y>=172) {y=172;} 
  if (TempY>=100) {TempY=100;} 
  
  if (TempY<=0) {TempY=0;} yStore = TempY; 
  if (TempY<10){ myGLCD.print(print_text[188], sbX1+5, 186);  // 00
                 myGLCD.printNumI(TempY, sbX1+21, 186);} 
                 
   if ((TempY>=10)&&(TempY<100)){
      myGLCD.print(print_text[187], sbX1+5, 186);             // 0
      myGLCD.printNumI(TempY, sbX1+13, 186);}  
   if (TempY>=100) {myGLCD.printNumI(TempY, sbX1+5, 186);} 
 
   
  for (int i=0; i<7; i++){
     if ((x>=(i*38)+39) && (x<=(i*38)+69)){ tled[k]=TempY;    // 49 79
      if (i==0) {yWHT=TempY; wcol_out=yWHT; }  
      if (i==1) {yBLU=TempY; bcol_out=yBLU; }
      if (i==2) {yRBL=TempY; rbcol_out=yRBL;}
      if (i==3) {yRED=TempY; rcol_out=yRED; }
      if (i==4) {yUVL=TempY; uvcol_out=yUVL; }
      if (i==5) {ySMP=TempY; ocol_out=ySMP; }
      if (i==6) {yGRN=TempY; grcol_out=yGRN; }}}
               LED_levelo_output(); }
          
void SliderBarsForChange(){ // ===== сам слайдер =======
   int TempY;
  myGLCD.setColor(sbR, sbG, sbB);         // Slider Bar Color
  myGLCD.fillRect(sbX1, y, sbX2, 185);    // draw the bar where you touch
  myGLCD.setColor(0, 0, 0);  
  myGLCD.fillRect(sbX1+1, y, sbX2-1, 72); // hide the bar
  myGLCD.setColor(sbR, sbG, sbB);         // Slider Bar Color
  myGLCD.drawLine(sbX1, 71, sbX2, 71);    // fills in the top of bar
  setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 

  if (y<=71) {y=71; }  // размер слайдера 114 пикс 185-71 
  if (y>=185) {y=185; } TempY = map(y, 185, 71, 0, 2000); tSlide=TempY;  // 255
  
 

  if (TempY<=0) {TempY=0;} 
  if (TempY>=100) {TempY=100;} // 255
  
  if (TempY<10){ myGLCD.print(print_text[188], sbX1+5, 187);   // 00
                 myGLCD.printNumI(TempY, sbX1+21, 187); } 
      
  if ((TempY>=10)&&(TempY<100)){
             myGLCD.print(print_text[187], sbX1+5, 187);       // 0
             myGLCD.printNumI(TempY, sbX1+13, 187); }
    
  if (TempY>=100){myGLCD.printNumI(TempY, sbX1+5, 187); }  
      for (int i=0; i<8; i++){
  if ((x>=(i*36)+34) && (x<=(i*36)+64)){tled[k]=TempY;}}}
  
void UpDnButtonSlide(){ // ==== слайдер ручного управления (кнопки Вверх и Вниз) 
     int yTemp; 
   if (yWHT>=100) {yWHT=100; } if (yWHT<=0) {yWHT=0; } // white  255
   if (yBLU>=100) {yBLU=100; } if (yBLU<=0) {yBLU=0; }
   if (yRBL>=100) {yRBL=100; } if (yRBL<=0) {yRBL=0; }
   if (yRED>=100) {yRED=100; } if (yRED<=0) {yRED=0; }
   if (yUVL>=100) {yUVL=100; } if (yUVL<=0) {yUVL=0; }
   if (ySMP>=100) {ySMP=100; } if (ySMP<=0) {ySMP=0; }
   if (yGRN>=100) {yGRN=100; } if (yGRN<=0) {yGRN=0; } // green
   
         for (int i=0; i<7; i++){ 
    if ((x>=(i*38)+49) && (x<=(i*38)+79)){   
    if (i==0) {yStore=yWHT; wcol_out=yWHT; }   // запомнить текущую яркость
    if (i==1) {yStore=yBLU; bcol_out=yBLU; }
    if (i==2) {yStore=yRBL; rbcol_out=yRBL; }
    if (i==3) {yStore=yRED; rcol_out=yRED; }
    if (i==4) {yStore=yUVL; uvcol_out=yUVL; }
    if (i==5) {yStore=ySMP; ocol_out=ySMP; }
    if (i==6) {yStore=yGRN; grcol_out=yGRN;}}} // green 

  if (yStore>=100){yStore=100; } 
  if (yStore<=0){yStore=0; }      
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
              myGLCD.print(F("    "), sbX1+5, 186);           
  if (yStore<10){ myGLCD.print(print_text[188], sbX1+5, 186);  //  00      
              myGLCD.printNumI(yStore, sbX1+21, 186); }
              
  if ((yStore>=10)&&(yStore<100)){  
              myGLCD.print(print_text[187], sbX1+5, 186);      // 0
              myGLCD.printNumI(yStore, sbX1+13, 186); }
  if (yStore>=100){myGLCD.printNumI(yStore, sbX1+5, 186); }  

      yTemp=map(yStore, 0, 100, 172, 44);       
      myGLCD.setColor(sbR, sbG, sbB);            // Bar Color
      myGLCD.fillRect(sbX1, yTemp, sbX2, 172);   // draw the bar from where it was last touched
      myGLCD.setColor(0, 0, 0);  
      myGLCD.fillRect(sbX1+1, yTemp, sbX2-1, 45);// hide the bar
      myGLCD.setColor(sbR, sbG, sbB);            // Bar Color
      myGLCD.drawLine(sbX1, 44, sbX2, 44);       // fills in the top of bar
         LED_levelo_output(); }  

//========================= Слайдер настройки каналов яркости OLEG ========================
int SliderBarsForChange2(int TopSldY, int BopSldY, int y, int sbR, int sbG, int sbB, int sbX1, int sbX2){
//  Slider bar vertical size: TopSldY/BotSldY, Y - value, sbR/sbG/sbB - colour
          int TempY;					   // sbX1/sbX2 - horisontal size
          
  if (y<=TopSldY) {y=TopSldY;}
  if (y>=BotSldY) {y=BotSldY;} 

  if (SliderSwitch == true) {			     // for slider update after pressing up/down button 
      if (tSlide<=0) {tSlide=0;}		     // and for first time open slider windows
      if (tSlide>=100) {tSlide=100;}
      y = map(tSlide, 0, 100, BotSldY, TopSldY);     // mapping 0-100 counter to slider size and draw
	  TempY = tSlide; }			     // for display value in %	
          
  myGLCD.setColor(0, 0, 0);  
  myGLCD.fillRect(sbX1+1, y, sbX2-1, TopSldY);       // hide the bar (for white bar rectangle)
//myGLCD.fillRect(sbX1+1, y, sbX2-1, TopSldY+1);     // hide the bar (for color bar rectangle)
  myGLCD.setColor(sbR, sbG, sbB);                    // Slider Bar Color
  myGLCD.fillRect(sbX1+1, y, sbX2-1, BotSldY);       // draw the bar where you touch

   if (SliderSwitch == false) {			     // for TOUCH function, convert touch coordinate to 0-100% 	
         TempY = map(y, BotSldY, TopSldY, 0, 100);   // mapping slider size to 0-100% 
         
   if (TempY>=100) {TempY=100;}		             // slider change from 0% to 100%
   if (TempY<=0) {TempY=0;} }

//     setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);       // set printed text colour
       setFont(SMALL, 255, 255, 255, 0, 0, 0);       // set printed text colour 
   if (TempY<10){myGLCD.print("  ", sbX1+0, BotSldY+2); 
        myGLCD.printNumI(TempY, sbX1+16, BotSldY+2); 
        myGLCD.print(print_text[142], sbX1+24, BotSldY+2); } // %
       
   if ((TempY>=10) && (TempY<100)){ myGLCD.print(F("  "), sbX1+0, BotSldY+2);       
        myGLCD.printNumI(TempY, sbX1+8, BotSldY+2); 
        myGLCD.print(print_text[142], sbX1+24, BotSldY+2); } // %
      
   if (TempY>=100){ 
        myGLCD.printNumI(TempY, sbX1+0, BotSldY+2); 
        myGLCD.print(print_text[142], sbX1+24, BotSldY+2); }  // %
	        tSlide = TempY;				      // temporary store previos value 
	        return TempY; }

//------------------------------------------------------------------------------------------------------------
  
void TimeSaver(boolean refreshAll=false){  // хранитель экрана

       myGLCD.setColor(127, 255, 212);                                                                                        //          ЦВЕТ ЧАСОВ НА СКРИНСЕЙВЕРЕ
       myGLCD.setBackColor(0, 0, 0);  
       myGLCD.setFont(SevenSegNumFont); // большой шрифт 
       if (RTC.hour < 10) {
            myGLCD.printNumI(0,80,65);
            myGLCD.printNumI(RTC.hour,115, 65);  
               } else myGLCD.printNumI(RTC.hour,80, 65);
       
      // myGLCD.print(":",CENTER, 65);
       myGLCD.fillRoundRect(156, 75, 164, 83);
       myGLCD.fillRoundRect(156, 93, 164, 101);
       
       
       
       if (RTC.minute < 10) {
         myGLCD.printNumI(0,175,65);
            myGLCD.printNumI(RTC.minute,208, 65);
       }else myGLCD.printNumI(RTC.minute,175, 65);
           
if (setScreensaverDOWonOff==1){             // Date and Date

// RTC.day - дата, RTC.month  - месяц, RTC.year - год, RTC.dow  - день недели       
// Отображать день недели           
                myGLCD.setFont(RusFont3);  // Выбор шрифта 
                myGLCD.setColor(127, 255, 212);  
                myGLCD.setBackColor(0, 0, 0);
              //setFont(RUS1, 0, 0, 255, 0, 0, 0);
         //    RTC.dow=calcDOW(RTC.day, RTC.month, RTC.year);    
                
      if (RTC.dow==1){ myGLCD.print(print_text[156], 80, 135);}   // Понедельник / Monday              
      if (RTC.dow==2){ myGLCD.print(print_text[157], 95, 135);}   // Вторник / Tuesday                
      if (RTC.dow==3){ myGLCD.print(print_text[158], 95, 135);}   // Среда / Wednesday                
      if (RTC.dow==4){ myGLCD.print(print_text[159], 95, 135);}   // Четверг / Thursday               
      if (RTC.dow==5){ myGLCD.print(print_text[160], 95, 135);}   // Пятница / Friday              
      if (RTC.dow==6){ myGLCD.print(print_text[161], 95, 135);}   // Суббота / Saturday    
      if (RTC.dow==0){ myGLCD.setColor(255, 0, 0); myGLCD.print(print_text[162], 80, 135);}   // Воскресенье / Sunday
            
// Позиция отображения даты, в зависимости от длинны имени дня недели 
            if (RTC.dow==1 || RTC.dow==0 ){xdate=83;}                         // позиция для Понедельник, Воскресенье.
            if (RTC.dow==2 || RTC.dow==4 || RTC.dow==5 || RTC.dow==6 ){xdate=73;} // позиция для Вторник, Четверг, Пятница, Суббота.
            if (RTC.dow==3 ){xdate=59;}                                     // позиция для Среда.
            
// Отображать дату
           //setFont(SMALL, 160, 255, 255, 30, 30, 30);
           myGLCD.setColor(127, 255, 212);
           myGLCD.setFont(BigFont);  // Выбор шрифта 
           myGLCD.printNumI(RTC.day, xdate+91, 128);
            
// Отображать месяц   // позиция месяца xdate+24
            myGLCD.setFont(RusFont3);  // Выбор шрифта 
     if (RTC.month==1){ myGLCD.print(print_text[163], xdate+130, 135);}       // Января / January                
     if (RTC.month==2){ myGLCD.print(print_text[164], xdate+130, 135);}       // Февраля / February                 
     if (RTC.month==3){ myGLCD.print(print_text[165], xdate+130, 135);}       // Марта / March                 
     if (RTC.month==4){ myGLCD.print(print_text[166], xdate+130, 135);}       // Апреля / Avril               
     if (RTC.month==5){ myGLCD.print(print_text[167], xdate+130, 135);}       // Мая / May               
     if (RTC.month==6){ myGLCD.print(print_text[168], xdate+130, 135);}       // Июня / June             
     if (RTC.month==7){ myGLCD.print(print_text[169], xdate+130, 135);}       // Июля / July                 
     if (RTC.month==8){ myGLCD.print(print_text[170], xdate+130, 135);}       // Августа / August                
     if (RTC.month==9){ myGLCD.print(print_text[171], xdate+130, 135);}       // Сентября / September                
     if (RTC.month==10){ myGLCD.print(print_text[172], xdate+130, 135);}      // Октября / October                
     if (RTC.month==11){ myGLCD.print(print_text[173], xdate+130, 135);}      // Ноября / November                 
     if (RTC.month==12){ myGLCD.print(print_text[174], xdate+130, 135);} }    // Декабря / December
     
// Отображать температуру
           //setFont(SMALL, 160, 255, 255, 30, 30, 30);
           myGLCD.setFont(RusFont3);  // Выбор шрифта 
           myGLCD.print(print_text[214], 58, 170);
           myGLCD.setFont(BigFont);
           myGLCD.printNumF(tempW, 1, 193, 163);
           
           myGLCD.setFont(SmallFont);
           myGLCD.print("o", 263, 161);
           myGLCD.print("C", 270, 167);
}
      
//******************************************************СКРИНСЕЙВЕР********************************************************
           
           
                       
void screenSaver(){         // Make the Screen Go Blank after so long
     setScreenSaverTimer = setSSmintues * 12;
     
   if ((setScreensaverOnOff==1) && (tempAlarmflag==false)){   // вкл / выкл
       if (myTouch.dataAvailable()){ processMyTouch();} else { screenSaverCounter++;}     
       if (screenSaverCounter==setScreenSaverTimer){ dispScreen=0; myGLCD.clrScr(); }
       
if (setClockOrBlank==0){ // пустой экран / часы 

  if (digital==0){       // цифровые часы 
     if (screenSaverCounter>setScreenSaverTimer){ dispScreen=0; TimeSaver(true); } }       // запустить TimeSaver (циф. часы)
     
  if (analog==0){        // аналоговые часы  analog==0
     if (screenSaverCounter>setScreenSaverTimer){aclock = 1; dispScreen=0; ATimeSaver();}} // запустить ATimeSaver (аналог. часы)
 }}}    

void ScreensaverSelect(){
  if (setClockOrBlank==0){                      // Choose Screensaver Buttons
      myGLCD.setColor(0, 0, 255);
      myGLCD.fillRoundRect(185, 20, 235, 40);
      setFont(SMALL, 255, 255, 255, 0, 0, 255);  
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[4]))); 
      myGLCD.print(buffer, 191, 24);     // BLANK
      myGLCD.setColor(0, 180, 86);       // green
      myGLCD.fillRoundRect(255, 20, 305, 40);      
      setFont(SMALL, 0, 0, 0, 0, 180, 86);   
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[5]))); 
      myGLCD.print(buffer, 261, 24);     // CLOCK    
      myGLCD.setColor(64, 64, 64);
      myGLCD.drawLine(0, 76, 319, 76);   

      myGLCD.setFont(RusFont1);         // русский фонт
      myGLCD.setColor(200, 200, 200); 
      myGLCD.setBackColor(0, 0, 0);
       //setFont(RUS1, 200, 200, 200, 0, 0, 0);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[6]))); 
      myGLCD.print(buffer, 232, 84);   // тип экрана
      myGLCD.setColor(0, 255, 0);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[7]))); 
      myGLCD.print(buffer, 32, 57);    // показывать дату      
  } else { 
        myGLCD.setColor(0, 180, 86); 
        myGLCD.fillRoundRect(185, 20, 235, 40);
        setFont(SMALL, 0, 0, 0, 0, 180, 86);  
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[4]))); 
        myGLCD.print(buffer, 191, 24);     //  чистый экран
        myGLCD.setColor(0, 0, 255);
        myGLCD.fillRoundRect(255, 20, 305, 40);    
        setFont(SMALL, 255, 255, 255, 0, 0, 255);  
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[5]))); 
        myGLCD.print(buffer, 261, 24);     //  часы
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(1, 47, 318, 77); } 
        myGLCD.setColor(255, 255, 255);
        myGLCD.drawRoundRect(185, 20, 235, 40);
        myGLCD.drawRoundRect(255, 20, 305, 40);

 if (setClockOrBlank==0){
     if (setScreensaverDOWonOff==0){         // Show Date on Screensaver
           
     printButton(print_text[36],ClocBlno[0],ClocBlno[1],ClocBlno[2],ClocBlno[3],SMALL,GREEN_BAC); // green  YES 
     printButton(print_text[35], ClocBlyes[0], ClocBlyes[1], ClocBlyes[2], ClocBlyes[3], SMALL);  // blue  NO  
    } else { 
     printButton(print_text[36],ClocBlno[0],ClocBlno[1],ClocBlno[2],ClocBlno[3],SMALL);            // blue  YES 
     printButton(print_text[35], ClocBlyes[0], ClocBlyes[1], ClocBlyes[2], ClocBlyes[3], SMALL,GREEN_BAC);}} // green  NO          

 if (setClockOrBlank==0){ // выбор типа хранителя экрана аналоговые / цифровые часы
    if (setScreensaverTupe==0){ 
      myGLCD.setColor(0, 0, 255);    // синий
      myGLCD.fillRoundRect(245, 132, 301, 152);  // размер кнопки А.ЧАСЫ         
      myGLCD.setFont(RusFont2);                  // русский фонт
      myGLCD.setColor(255, 255, 255);            // белый
      myGLCD.setBackColor(0, 0, 255);            // ФОН СИНИЙ
      //setFont(RUS2, 255, 255, 255, 0, 0, 255);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[11]))); 
      myGLCD.print(buffer, 250, 137);            // аналоговые часы  А.ЧАСЫ 
      myGLCD.setColor(0, 180, 86);               // зелёный
      myGLCD.fillRoundRect(245, 100, 301, 119);  // размер кнопки Ц.ЧАСЫ
      
      myGLCD.setColor(0, 0, 0);
      myGLCD.setBackColor(0, 180, 86);  
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[10]))); 
      myGLCD.print(buffer, 250, 104);             // цифровые часы  Ц.ЧАСЫ 
    } else { 
        myGLCD.setFont(RusFont2);                 // русский фонт
        myGLCD.setColor(0, 180, 86);              // зелёный
        myGLCD.fillRoundRect(245, 132, 301, 152); // размер кнопки А.ЧАСЫ

        myGLCD.setColor(0, 0, 0);
        myGLCD.setBackColor(0, 180, 86);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[11]))); 
        myGLCD.print(buffer, 250, 137);            // аналоговые часы  А.ЧАСЫ 
        myGLCD.setColor(0, 0, 255);                // синий
        myGLCD.fillRoundRect(245, 100, 301, 119);  // размер кнопки Ц.ЧАСЫ  
        
        myGLCD.setColor(255, 255, 255);    // белый шрифт
        myGLCD.setBackColor(0, 0, 255);    // голубой фон
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[10]))); 
        myGLCD.print(buffer, 250, 104); }           // цифровые часы  Ц.ЧАСЫ 
        myGLCD.setColor(255, 255, 255);            
        myGLCD.drawRoundRect(245, 132, 301, 152);   // белая рамка вокруг кнопок А.ЧАСЫ
        myGLCD.drawRoundRect(245, 100, 301, 119);}} // белая рамка вокруг кнопок Ц.ЧАСЫ
        
//----------------------- главные настройки стр. 1 -----------------------------        
void genSetSelect_1(){
        
    myGLCD.setColor(0, 0, 255);             // Change Fan Startup Temps Button
    myGLCD.fillRoundRect(195, 159, 295, 179); // кнопка УСТАН. ТЕМП
    myGLCD.fillRoundRect(205, 123, 285, 143); // кнопка ЯРК. ЭКРАНА подсветка экрана
    myGLCD.setFont(RusFont2);
    myGLCD.setColor(255, 255, 255);
    myGLCD.setBackColor(0, 0, 255);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[18]))); 
    myGLCD.print(buffer, 205, 164);            // УСТАН.ТЕМП.    
    myGLCD.print(print_text[122], 211, 128);   // УСТАНОВ. ( яркость подсветки) 

    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect(195, 159, 295, 179);   // белая рамка вокруг кнопки установ. температуры 
    myGLCD.drawRoundRect(205, 123, 285, 143); } // УСТАНОВ. ( яркость подсветки)
 
//------------------------- главные настройки стр.2 -----------------------------
void genSetSelect_2(){ 
  
 if (setDimLEDsOnOff==1){                // Dim LEDs Temp Buttons

    printButton(print_text[212],DledOn[0],DledOn[1],DledOn[2],DledOn[3],SMALL,GREEN_BAC);   // green  ON
    printButton(print_text[213], DledOff[0], DledOff[1], DledOff[2], DledOff[3], SMALL);    // blue  OFF
   } else { 
    printButton(print_text[212],DledOn[0],DledOn[1],DledOn[2],DledOn[3],SMALL);              // blue  ON
    printButton104(print_text[213], DledOff[0], DledOff[1], DledOff[2], DledOff[3], SMALL);} // red  OFF  

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 55, 305, 75);
  myGLCD.setFont(RusFont2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 255);
 //setFont(RUS2, 255, 255, 255, 0, 0, 255); 
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[127]))); 
  myGLCD.print(buffer, 192, 60);      // УСТАНОВ. ТЕМП.
 
 if (setScreensaverOnOff==1){        // Хранитель экрана Screensaver 
        
     printButton(print_text[212],SsavOn[0],SsavOn[1],SsavOn[2],SsavOn[3],SMALL,GREEN_BAC); // green ON
     printButton(print_text[213], SsavOff[0], SsavOff[1], SsavOff[2], SsavOff[3], SMALL);  // blue OFF
    } else { 
     printButton(print_text[212],SsavOn[0],SsavOn[1],SsavOn[2],SsavOn[3],SMALL);              // blue ON
     printButton104(print_text[213], SsavOff[0], SsavOff[1], SsavOff[2], SsavOff[3], SMALL);} // red OFF        

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 129, 305, 149);
  myGLCD.setFont(RusFont2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 255);
  //setFont(RUS2, 255, 255, 255, 0, 0, 255); 
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[21]))); 
  myGLCD.print(buffer, 210, 134);        // НАСТРОИТЬ
 
 if (DimmL==1){                          // Dimm  ОГРАНИЧЕНИЕ МОЩНОСТИ (РУЧНОЕ ДИММИРОВАНИЕ)        
     printButton(print_text[212],DimmLOn[0],DimmLOn[1],DimmLOn[2],DimmLOn[3],SMALL,GREEN_BAC);    // green  ON
     printButton(print_text[213], DimmLOff[0], DimmLOff[1], DimmLOff[2], DimmLOff[3], SMALL);     // blue OFF
   } else { 
     printButton(print_text[212],DimmLOn[0],DimmLOn[1],DimmLOn[2],DimmLOn[3],SMALL);              // blue  ON
     printButton104(print_text[213], DimmLOff[0], DimmLOff[1], DimmLOff[2], DimmLOff[3], SMALL);} // red OFF       

     myGLCD.setColor(255, 255, 255);    
     myGLCD.drawRoundRect(185, 129, 305, 149); 
     myGLCD.drawRoundRect(185, 55, 305, 75); } 

/*********************** ГЛАВНЫЙ ЭКРАН ****************** MAIN SCREEN *************************** dispScreen = 0 */                           //                                                             ГЛАВНЫЙ ЭКРАН
void mainScreen(boolean refreshAll=false){
  int ledLevel, bar;
  String oldval, deg;
  
char buffer_Led_Out[15];
  TimeDateBar(true);
    titledate(true); // дата в нижней строке

// RTC.dow=calcDOW(RTC.day, RTC.month, RTC.year);
// if ((RTC.dow || refreshAll)){ TimeDateBar(); titledate();} 
   oldval = day;                               // refresh day if different
   day = String(RTC.day);

  if ((oldval!=day) || refreshAll){
     myGLCD.setColor(92, 92, 92);
     myGLCD.drawRect(4, 166, 110, 178);         // таймеры
     myGLCD.drawRect(4, 178, 110, 189); 
     myGLCD.drawRect(4, 189, 110, 200); 
     myGLCD.drawRect(4, 200, 110, 211); 
     myGLCD.drawRect(4, 211, 110, 211);      
     myGLCD.setColor(0, 153, 153);              // Draw Borders & Dividers 
     myGLCD.drawRect(0, 14, 319, 226);          // Outside Border
     myGLCD.drawRect(168, 14, 170, 226);       // Vertical Divider
     myGLCD.drawRect(114, 158, 164, 222);       // вокруг авто-долива
     myGLCD.drawRect(4, 158, 110, 222);
     myGLCD.drawRect(265, 4, 265, 107);       // вертикальная линия справа от луны
     myGLCD.drawRect(265, 39, 319, 39);       // Горизонтальная линия над фильтром       
     myGLCD.drawRect(4, 95, 110, 148);        // рамка вокруг дозаторов
     myGLCD.drawRect(114, 95, 164, 138);      // рамка вокруг PH
     myGLCD.drawRect(265, 73, 319, 110);      // вокруг пресетов 
     myGLCD.drawRect(218, 110, 303, 169);     // рамка вокруг текущей и максимальной температуры        
     myGLCD.drawRect(264, 131, 303, 169);     // рамка вокруг максимальной температуры      
     myGLCD.drawRect(170, 131, 319, 169);     // рамка вокруг всей температуры 
     myGLCD.drawRect(175, 175, 314, 222); // МОНИТОР ТРЕВОГИ
     myGLCD.drawRect(170, 108, 319, 110); // горизонтальная линия (под луной)
     myGLCD.setColor(64, 64, 64);
     myGLCD.fillRect(0, 0, 319, 14);      // Top Bar
     myGLCD.setColor(0, 0, 0);
     myGLCD.drawLine(159, 126, 161, 126);  // Horizontal Divider Separator     
     myGLCD.setFont(RusFont3);
     myGLCD.setColor(255, 255, 0);
     myGLCD.setBackColor(64, 64, 64);
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[22])));                                                                                             
       myGLCD.print(buffer, CENTER, 3);    // УПРАВЛЕНИЕ АКВАРИУМОМ
       
       myGLCD.setFont(RusFont1);           // русский фонт
       myGLCD.setColor(32, 255, 255);      // цвет бирюзовый
       myGLCD.setBackColor(0, 0, 0); 
     
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[23]))); 
       myGLCD.print(buffer, 25, 18);         // ЯРКОСТЬ КАНАЛОВ
       
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[72]))); 
       myGLCD.print(buffer, 24, 92);         //ДОЗАТОРЫ
       myGLCD.setColor(0, 204, 204);      // цвет бирюзовый
       myGLCD.drawRoundRect(10, 102, 55, 122);
       myGLCD.drawRoundRect(60, 102, 105, 122);
       myGLCD.drawRoundRect(10, 125, 55, 145);
       myGLCD.drawRoundRect(60, 125, 105, 145);
       
         if (DOZTime1==1)
    { myGLCD.setColor(0, 100, 0);
      myGLCD.fillRoundRect(11, 103, 54, 121); 
      setFont(SMALL, 255, 255, 0, 0, 100, 0);
     myGLCD.setFont(RusFont1); 
     if (dozPump1H<10){
     myGLCD.printNumI(dozPump1H, 23, 109);}
     else {myGLCD.printNumI(dozPump1H, 15, 109);}
     myGLCD.print(":", 29, 109);
     if (dozPump1M<10){
     myGLCD.printNumI(dozPump1M, 44, 109);
     myGLCD.print("0", 36, 109);}
     else {myGLCD.printNumI(dozPump1M, 36, 109);}
    }
         if (DOZTime1==0)
    { myGLCD.setColor(100, 0, 0);
      myGLCD.fillRoundRect(11, 103, 54, 121); 
      setFont(SMALL, 255, 255, 255, 100, 0, 0);
     myGLCD.setFont(RusFont1); 
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 15, 109);}

         if (DOZTime2==1)
    { myGLCD.setColor(0, 100, 0);
      myGLCD.fillRoundRect(61, 103, 104, 121); 
      setFont(SMALL, 255, 255, 0, 0, 100, 0);
     myGLCD.setFont(RusFont1); 
     if (dozPump2H<10){
     myGLCD.printNumI(dozPump2H, 73, 109);}
     else {myGLCD.printNumI(dozPump2H, 65, 109);}
     myGLCD.print(":", 79, 109);
     if (dozPump2M<10){
     myGLCD.printNumI(dozPump2M, 94, 109);
     myGLCD.print("0", 86, 109);}
     else {myGLCD.printNumI(dozPump2M, 86, 109);}
    }
         if (DOZTime2==0)
    { myGLCD.setColor(100, 0, 0);
      myGLCD.fillRoundRect(61, 103, 104, 121); 
      setFont(SMALL, 255, 255, 255, 100, 0, 0);
    myGLCD.setFont(RusFont1); 
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 65, 109);}

         if (DOZTime3==1)
    { myGLCD.setColor(0, 100, 0);
      myGLCD.fillRoundRect(11, 126, 54, 144); 
      setFont(SMALL, 255, 255, 0, 0, 100, 0);
   myGLCD.setFont(RusFont1); 
     if (dozPump3H<10){
     myGLCD.printNumI(dozPump3H, 23, 131);}
     else {myGLCD.printNumI(dozPump3H, 15, 131);}
     myGLCD.print(":", 29, 131);
     if (dozPump3M<10){
     myGLCD.printNumI(dozPump3M, 44, 131);
     myGLCD.print("0", 36, 131);}
     else {myGLCD.printNumI(dozPump3M, 36, 131);}
    }
         if (DOZTime3==0)
    { myGLCD.setColor(100, 0, 0);
      myGLCD.fillRoundRect(11, 126, 54, 144); 
      setFont(SMALL, 255, 255, 255, 100, 0, 0);
     myGLCD.setFont(RusFont1); 
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 15, 131);}

         if (DOZTime4==1)
    { myGLCD.setColor(0, 100, 0);
      myGLCD.fillRoundRect(61, 126, 104, 144); 
      setFont(SMALL, 255, 255, 0, 0, 100, 0);
   myGLCD.setFont(RusFont1); 
     if (dozPump4H<10){
     myGLCD.printNumI(dozPump4H, 73, 131);}
     else {myGLCD.printNumI(dozPump4H, 65, 131);}
     myGLCD.print(":", 79, 131);
     if (dozPump4M<10){
     myGLCD.printNumI(dozPump4M, 94, 131);
     myGLCD.print("0", 86, 131);}
     else {myGLCD.printNumI(dozPump4M, 86, 131);}
   }
         if (DOZTime4==0)
    { myGLCD.setColor(100, 0, 0);
      myGLCD.fillRoundRect(61, 126, 104, 144); 
      setFont(SMALL, 255, 255, 255, 100, 0, 0);
     myGLCD.setFont(RusFont1); 
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 65, 131);}
           
       
       myGLCD.setColor(32, 255, 255);      // цвет бирюзовый
       myGLCD.setBackColor(0, 0, 0);
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[82]))); 
       myGLCD.print(buffer, 132, 92);         //PH   
          myGLCD.setFont(BigFont);
if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 10){
         myGLCD.setFont(BigFont);
         myGLCD.printNumF(avgMeasuredPH,1, 116, 110);
  }
  else{ if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer)
      myGLCD.drawBitmap(128, 108, 24, 24, clos, 1);    // картинка  крестик 
  }
       myGLCD.setFont(RusFont1);
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[76]))); 
       myGLCD.print(buffer, 112, 145);         //УРОВЕНЬ         
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[73]))); 
       myGLCD.print(buffer, 125, 155);         //ВОДЫ
       myGLCD.setColor(120, 120, 120);      // цвет 
       myGLCD.drawRoundRect(120, 170, 158, 178);
       myGLCD.drawRoundRect(120, 180, 158, 188);
       myGLCD.drawRoundRect(120, 190, 158, 198);
       myGLCD.drawRoundRect(120, 203, 158, 218);
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[77]))); 
       myGLCD.print(buffer, 125, 207);     //СЛИВ
       
       myGLCD.setColor(190, 190, 190);      // цвет серый
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[24])));        
       myGLCD.print(buffer, 182, 18);        // ФАЗA ЛУНЫ
       
       myGLCD.setColor(190, 190, 190);      // темно-зел (0, 176, 114);
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[25]))); 
       myGLCD.print(buffer, 229, 113);      // ТЕМПЕРАТ.  
                 
          myGLCD.setColor(192, 0, 28); // красный (150, 150, 150); - серый
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[13]))); 
       myGLCD.print(buffer, 186, 173);       // МОНИТОР ТРЕВОГИ 
       
       myGLCD.setFont(RusFont6);  // font     
       myGLCD.setColor(210, 210, 210);        // зел. шрифт берюзовый (32, 255, 255); 
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[12]))); 
       myGLCD.print(buffer, 178, 115);       // Датч.  (buffer, 178, 122);
       
        myGLCD.setFont(RusFont1);     // font
        myGLCD.setColor(0, 176, 114); // зел. шрифт берюзовый (32, 255, 255);
       myGLCD.print(print_text[146], 221, 122);    // Текущ       
       myGLCD.setColor(255, 151, 48);  // красн (морковный)
       myGLCD.print(print_text[145], 269, 122);    // Макс
                    
 //    myGLCD.setFont(SmallFont);     
     myGLCD.setColor(0, 216, 255); 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[169]))); 
     myGLCD.print(buffer, 32, 153);    // Timer
     myGLCD.setColor(100, 240, 100);
     myGLCD.setFont(RusFont1);        // русский фонт
//     myGLCD.print(print_text[47], 7, 168);  // 1
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[151]))); 
     myGLCD.print(buffer, 8, 168);     
//     myGLCD.print(print_text[48], 7, 180);  // 2
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[152]))); 
     myGLCD.print(buffer, 8, 180);     
//     myGLCD.print(print_text[49], 7, 191);  // 3
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[153]))); 
     myGLCD.print(buffer, 8, 191);     
 //    myGLCD.print(print_text[50], 7, 202);  // 4
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[154]))); 
     myGLCD.print(buffer, 8, 202);     
 //    myGLCD.print(print_text[51], 7, 213);  // 5
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[155]))); 
     myGLCD.print(buffer, 8, 213);     
           
           
     float lunarCycle = moonPhase(RTC.year, RTC.month, RTC.day); // get a value for the lunar cycle 

     myGLCD.drawBitmap(194, 29, 53, 53, First_Quarter, 1);  // new  картинка луны

     myGLCD.drawBitmap(271, 76, 45, 9, preset, 1);    // картинка  пресеты
  
     myGLCD.setFont(RusFont1); 
     myGLCD.setColor(210, 210, 210);
     myGLCD.setBackColor(0, 0, 0); 
     
 if ((lunarCycle*100) < 1){ myGLCD.print(F(" 0.0"), 216, 96);} // Print % of Full to LCD (94-для шрифта 3)
      else { myGLCD.printNumF(lunarCycle*100, 1, 216, 96);}
      myGLCD.print(print_text[142], 251, 96);  // % знак процентов
     
       myGLCD.setColor(156, 156, 156);
       myGLCD.setBackColor(0, 0, 0);
       myGLCD.print(print_text[89], 178, 96); // Луна
       char bufferLP[16];
       LP.toCharArray(bufferLP, 16);
       myGLCD.print(bufferLP, 179, 86);       // отображение текущей фазы луны    
   
}  
// бары уровней яркости LED 
   if ((whiteLed!=wled_out) || refreshAll){         // refresh red led display  
        whiteLed = wled_out;
        ledLevel = LedToPercent(wled_out, 2000); 
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(wled_out, 2000, 0, 255, 255, 204); // led_out, resoluton, Xshift, bar color
        } // имя канала + уровень

   if ((blueLed!=bled_out) || refreshAll){         // refresh red led display  
        blueLed = bled_out;
        ledLevel = LedToPercent(bled_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(bled_out, 2000, 22, 255, 255, 255); // led_out, resoluton, Xshift, bar color
//                                       ^ - расстояние между барами 22
        } 

    if ((rblueLed!=rbled_out) || refreshAll){         // refresh red led display  
        rblueLed = rbled_out;
        ledLevel = LedToPercent(rbled_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(rbled_out, 2000, 44, 58, 95, 205); // led_out, resoluton, Xshift, bar color
        } 

    if ((redLed!=rled_out) || refreshAll){         // refresh red led display  
        redLed = rled_out;
        ledLevel = LedToPercent(rled_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(rled_out, 2000, 66, 255, 0, 0); // led_out, resoluton, Xshift, bar color
        } 

    if ((uvLed!=uvled_out) || refreshAll){         // refresh red led display  
        uvLed = uvled_out;
        ledLevel = LedToPercent(uvled_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(uvled_out, 2000, 88, 224, 102, 255); // led_out, resoluton, Xshift, bar color
        } 

   if ((orLed!=oLed_out) || refreshAll){         // refresh red led display  
        orLed = oLed_out;
        ledLevel = LedToPercent(oLed_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(oLed_out, 2000, 110, 255, 143, 32); // led_out, resoluton, Xshift, bar color 
        } 

    if ((grLed!=gled_out) || refreshAll){          // refresh red led display  
        grLed = gled_out;	
        ledLevel = LedToPercent(gled_out, 2000);
	oldval.toCharArray(buffer_Led_Out, 15);
        SmallLedBarGraph(gled_out, 2000, 132, 0, 255, 0);   // led_out, resoluton, Xshift, bar color
        } 

 if ((whiteLed=wled_out)||(blueLed=bled_out)||(rblueLed=rbled_out)||(redLed=rled_out) 
   ||(uvLed=uvled_out)||(orLed=oLed_out)||(grLed=gled_out)||refreshAll){drawBarGraph();} // refresh all static led bar 

// ----------------------------------ТЕМПЕРАТУРА В ГЛАВНОМ ЭКРАНЕ -----------------------------------        
  if (refreshAll ){   
    if (counterB1 ==1 || counterB2 ==1 || counterB3 ==1) { 
            myGLCD.setFont(RusFont1); 
            myGLCD.setColor(78, 210, 0);   
            myGLCD.setBackColor(0, 0, 0);
            //setFont(RUS1, 78, 210, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[27]))); 
            myGLCD.print(buffer, 176, 135);      // ВОДА  (Темп.Воды)
            setFont(SMALL, 200, 200, 200, 0, 0, 0);
            myGLCD.drawCircle(307, 135, 1);     //  значек цельсия (темп воды)         
            myGLCD.print(print_text[57], 311, 133); // C темп воды
            myGLCD.printNumF(MaxTempW, 1, 270 ,133);}      // отображение максимальной температуры воды

    if (counterB1 ==2 || counterB2 ==2 || counterB3 ==2){
                myGLCD.setFont(RusFont1);
                myGLCD.setColor(78, 210, 0);  
                myGLCD.setBackColor(0, 0, 0);    
                //setFont(RUS1, 78, 210, 0, 0, 0, 0);
                    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[28]))); 
                myGLCD.print(buffer, 176, 147);     // РАД 1
                setFont(SMALL, 200, 200, 200, 0, 0, 0);
                myGLCD.drawCircle(307, 147, 1);     // значек цельсия (темп рад)            
                myGLCD.print(print_text[57], 311, 145);  // C  рад 1
                myGLCD.printNumF(MaxTempH1, 1, 270 ,145); }    // отображение максимальной температуры радиатор датчик 1
         
     if (counterB1 ==3 || counterB2 ==3 || counterB3 ==3){
                   myGLCD.setFont(RusFont1);
                   myGLCD.setColor(78, 210, 0);  
                   myGLCD.setBackColor(0, 0, 0);  
                  //setFont(RUS1, 78, 210, 0, 0, 0, 0);
                       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[29]))); 
                   myGLCD.print(buffer, 176, 159);      // РАД 2
                   setFont(SMALL, 200, 200, 200, 0, 0, 0);
                    myGLCD.drawCircle(307, 159, 1);     // значек цельсия (темп самп)           
                    myGLCD.print(print_text[57], 311, 157);  // C для рад 2
                    myGLCD.printNumF(MaxTempH2, 1, 270 ,157); }  // отображение максимальной температуры радиатор датчик 2              
	                 calculateStartTime(); }

           myGLCD.setColor(0, 0, 0);       // clear cooler / heater & alarm notices
   if (tempCoolflag==false&&tempHeatflag==false){myGLCD.fillRect(190, 184, 303, 201);} // очистка баннера нагреватель холодильник(196, 184, 295, 201);
   if (tempAlarmflag==false){myGLCD.fillRect(185, 202, 305, 222);}  // очистка баннера alarm 

// ------- Отображение температуры Воды ---------------------
  if (counterB1 ==1 || counterB2 ==1 || counterB3 ==1) {
        if (tempW == -127 || tempW == -196 ){    // range in deg C no matter what
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
	         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30]))); 
             myGLCD.print(buffer, 227, 133);     // Err.  sensor disconnected
		 tempCoolflag=false;
		 tempHeatflag=false; } else { 

     if (tempCoolflag==true){                    // Water temperature too HIGH
            myGLCD.setFont(RusFont3);
            myGLCD.setColor(16, 63, 255);
            myGLCD.setBackColor(0, 0, 0);
            //setFont(RUS3, 16, 63, 255, 0, 0, 0);
            myGLCD.drawRect(198, 186, 293, 200); 
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[31]))); 
            myGLCD.print(buffer, 203, 190); }      // ХОЛОДИЛЬНИК 
               else 
          if (tempHeatflag==true){                 // Water temperature too LOW
                myGLCD.setFont(RusFont3);
                myGLCD.setColor(255, 24, 127);
                myGLCD.setBackColor(0, 0, 0);
                //setFont(RUS3, 255, 24, 127, 0, 0, 0);
                myGLCD.drawRect(198, 186, 293, 200);
                    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[32]))); 
                myGLCD.print(buffer, 203, 190); }    // НАГРЕВАТЕЛЬ

		  setFont(SMALL, 0, 255, 0, 0, 0, 0);
	//          myGLCD.printNumF( tempW, 1, 227, 133+ShiftDrawY); // отображение темп в аквариуме 
                   myGLCD.printNumF( tempW, 1, 227, 133); // отображение темп в аквариуме
                   
	          setFont(SMALL, 200, 200, 200, 0, 0, 0);   
                  myGLCD.drawCircle(307, 135, 1);     // значек цельсия (темп воды)         
                  myGLCD.print(print_text[57], 311, 133);  // C для воды в аквариуме

	  if (tempAlarmflag==true){
		    setFont(LARGE, 255, 0, 0, 0, 0, 0);
                    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[33]))); 
                    myGLCD.print(buffer, 192, 206); }}}   // ALARM!! 
                      
//-------- Отображение температуры радиатора Датчик 1 -----------
  if (counterB1 ==2 || counterB2 ==2 || counterB3 ==2) {
        if (tempH1 == -127 || tempH1 == -196){
	    setFont(SMALL, 255, 0, 0, 0, 0, 0);
                  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30]))); // sensor disconnected
              myGLCD.print(buffer, 227, 145);     // Err.  sensor disconnected
          }else { setFont(SMALL, 0, 255, 0, 0, 0, 0);
	      myGLCD.printNumF( tempH1, 1, 227, 145);  // Heatsink1 temperature (No Flags)
	      setFont(SMALL, 200, 200, 200, 0, 0, 0);
                myGLCD.drawCircle(307, 147, 1);         // значек цельсия  (темп рад)            
                myGLCD.print(print_text[57], 311, 145); }} // C для рад 1
 
//-------- Отображение температуры радиатора Датчик 2 -----------
  if (counterB1 ==3 || counterB2 ==3 || counterB3 ==3) { 
	if (tempH2 == -127 || tempH2 == -196){
	     setFont(SMALL, 255, 0, 0, 0, 0, 0);
                   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30]))); // sensor disconnected
               myGLCD.print(buffer, 227, 157);    // Err.  sensor disconnected
           } else { setFont(SMALL, 0, 255, 0, 0, 0, 0);
               myGLCD.printNumF( tempH2, 1, 227, 157); // Heatsink2 temperature (No Flags)
	       setFont(SMALL, 200, 200, 200, 0, 0, 0); 
                myGLCD.drawCircle(307, 159, 1);       // значек цельсия  (темп самп)           
                myGLCD.print(print_text[57], 311, 157);}}}  // C для рад 2

//************************************ОКОНЧАНИЕ ГЛАВНОГО ЭКРАНА**************************************//

void screenReturn(){   // Авто-возврат в главный экран
     setReturnTimer = setScreenSaverTimer * .75;  // время через которое возврат в главный экран
  if (SCREEN_RETURN==true){                      // 75% от времени хранителя экрана
if (dispScreen!=0){
  if (myTouch.dataAvailable()){ processMyTouch();} else { returnTimer++; }
      if (returnTimer>setReturnTimer){ returnTimer=0; 
        LEDtestTick = false; colorLEDtest = false; ReadFromEEPROM(); 
        dispScreen=0; clearScreen(); mainScreen(true); }}}}
        
//**************************************КОРМУШКА ВКЛ ВЫКЛ*************************************** dispScreen = ? ****//
        
        void feedingTimeOnOff()
{
  if ((feedTime==1) && (FEEDTime1==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
  	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((feedTime==1) && (FEEDTime1==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((feedTime==2) && (FEEDTime2==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((feedTime==2) && (FEEDTime2==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((feedTime==3) && (FEEDTime3==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((feedTime==3) && (FEEDTime3==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((feedTime==4) && (FEEDTime4==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((feedTime==4) && (FEEDTime4==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(70, 150, 250, 170);   
}

        
        void dosingTimeOnOff()
{
  if ((dozTime==1) && (DOZTime1==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
  	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((dozTime==1) && (DOZTime1==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((dozTime==2) && (DOZTime2==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((dozTime==2) && (DOZTime2==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((dozTime==3) && (DOZTime3==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((dozTime==3) && (DOZTime3==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  if ((dozTime==4) && (DOZTime4==1))
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 94, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128]))); 
	  myGLCD.print(buffer, 94+120, 157);}

  if ((dozTime==4) && (DOZTime4==0))
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(70, 150, 250, 170); 
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
   	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer, 90, 157);
	  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
	  myGLCD.print(buffer, 90+120, 157);}

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(70, 150, 250, 170);   
}

        
        
        


//*********************** ЭКРАН ГЛАВНОГО МЕНЮ *********************************************** dispScreen = 1 *// 
void menuScreen(){  

      PrintStringIndex=0; printHeader ();  // ГЛАВНОЕ МЕНЮ
         
   myGLCD.setColor(64, 64, 64);
   myGLCD.drawRect(0, 14, 319, 225);   // вокруг всего экрана
   myGLCD.drawRect(0, 194, 319, 196);  // внизу (под кнопками) 194, 196
   myGLCD.setColor(110, 110, 110); 
   myGLCD.drawRoundRect(9, 201, 193, 215); // ГРАФИК ТЕМПЕРАТУР (закругленные края)
   printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);   // ОТМЕНА
  
  printButton("", tanD[0], tanD[1], tanD[2], tanD[3]);            // УСТАНОВКА ВРЕМЕНИ И ДАТЫ
  printButton("", temC[0], temC[1], temC[2], temC[3]);            // УСТАНОВКА ТЕМПЕРАТУРЫ
  printButton("", feed[0], feed[1], feed[2], feed[3]);            // АВТОКОРМУШКА
  printButton("", Pumpset[0], Pumpset[1], Pumpset[2], Pumpset[3]);// Дозатор УДО
  printButton("", gSet[0], gSet[1], gSet[2], gSet[3]);            // ОСНОВНЫЕ НАСТРОЙКИ 
  printButton("", tesT[0], tesT[1], tesT[2], tesT[3]);            // АВТО ТЕСТ
  printButton("", tesT2[0], tesT2[1], tesT2[2], tesT2[3]);        // Тест 2 (с графиками)  
  printButton("", ledChM[0], ledChM[1], ledChM[2], ledChM[3]);    // Настройка каналов по цветам  
  printButton("", Sector[0], Sector[1], Sector[2], Sector[3]);    // Настройка по секторам времени
  printButton("", Preset[0], Preset[1], Preset[2], Preset[3]);    // Запись присетов 
  printButton("", timday[0], timday[1], timday[2], timday[3]);    // Суточные Таймеры 
  printButton("", PHset[0], PHset[1], PHset[2], PHset[3]);        // PH
  printButton("", logW[0], logW[1], logW[2], logW[3]);            // график темп. для воды 
  printButton("", logH[0], logH[1], logH[2], logH[3]);            // график темп. радиаторы 

          myGLCD.setFont(RusFont1);
          myGLCD.setColor(190, 190, 190);
          myGLCD.setBackColor(0, 0, 0);
        myGLCD.print(print_text[121], 34, 198);   // ГРАФИК ТЕМПЕРАТУР
           
           myGLCD.setColor(255, 255, 255);
           myGLCD.setBackColor(0, 0, 255);
        myGLCD.print(print_text[122], 20, 29);    // УСТАНОВКА
        myGLCD.print(print_text[123], 28, 42);    // ВРЕМЕНИ
        myGLCD.print(print_text[130], 143, 29);   // АВТО
        myGLCD.print(print_text[131], 142, 41);   // ТЕСТ   
        myGLCD.print(print_text[132], 242, 29);   // ГРАФИК
        myGLCD.print(print_text[133], 238, 41);   // КАНАЛОВ 
        myGLCD.print(print_text[122], 20, 73);    // УСТАНОВКА
        myGLCD.print(print_text[124], 12, 87);    // ТЕМПЕРАТУРЫ
        myGLCD.print(print_text[122], 125, 72);   // УСТАНОВКА
        myGLCD.print(print_text[134], 132, 86);   // ЯРКОСТИ
        myGLCD.print(print_text[126], 238, 118);  // ДОЗАТОР
        myGLCD.print(print_text[141], 222, 131);  // УГЛЕКИСЛОТЫ
        myGLCD.print(print_text[136], 36, 118);   // ВРЕМЯ
        myGLCD.print(print_text[216], 19, 131);   // КОРМЛЕНИЯ
        myGLCD.print(print_text[135], 135, 118);  // CЕКТОР  
        myGLCD.print(print_text[123], 131, 131);  // ВРЕМЕНИ     
        myGLCD.print(print_text[137], 240, 72);   // ЗАПИСЬ
        myGLCD.print(print_text[138], 232, 86);   // ПРЕСЕТОВ
        myGLCD.print(print_text[128], 25, 160);   // ОСНОВНЫЕ
        myGLCD.print(print_text[129], 20, 174);   // НАСТРОЙКИ                
        myGLCD.print(print_text[126], 238, 160);  // ДОЗАТОР
        myGLCD.print(print_text[127], 238, 174);  // УДОБРЕНИЙ
        myGLCD.print(print_text[139], 128, 160);  // СУТОЧНЫЕ  
        myGLCD.print(print_text[140], 132, 174);  // ТАЙМЕРЫ
         myGLCD.setFont(RusFont1);
        myGLCD.print(print_text[119], 25, 212);   // АКВАР.  
        myGLCD.print(print_text[120], 115, 212);   // РАДИАТ 
   }
/************** ЭКРАН УСТАНОВКИ ВРЕМЕНИ ************************************************** dispScreen = 2 */
void clockScreen(boolean refreshAll=true) {
  if (refreshAll){
    rtcSetMin=RTC.minute; rtcSetHr=RTC.hour; 
    rtcSetDy=RTC.day; rtcSetMon=RTC.month; rtcSetYr=RTC.year;
                
    PrintStringIndex=1; printHeader ();  // УСТАНОВКА ВРЕМЕНИ И ДАТЫ

    myGLCD.setColor(64, 64, 64);                   // Draw Dividers in Grey
    myGLCD.drawRect(0, 196, 319, 194);             // Bottom Horizontal Divider
    myGLCD.drawLine(0, 105, 319, 105);             // Middle Horizontal Divider
    myGLCD.drawRoundRect(9, 35, 95, 90);        // рамка вокруг текста (ВРЕМЯ)
    
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
    printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА
     
    drawUpButtonSlide(houU[0], houU[1]);           // hour up
    drawUpButtonSlide(minU[0], minU[1]);           // min up
    drawDownButtonSlide(houD[0], houD[1]);         // hour down
    drawDownButtonSlide(minD[0], minD[1]);         // min down       
    drawUpButtonSlide(dayU[0], dayU[1]);           // day up
    drawUpButtonSlide(monU[0], monU[1]);           // month up
    drawUpButtonSlide(yeaU[0], yeaU[1]);           // year up
    drawDownButtonSlide(dayD[0], dayD[1]);         // day down
    drawDownButtonSlide(monD[0], monD[1]);         // month down
    drawDownButtonSlide(yeaD[0], yeaD[1]); }       // year down
     
  timeDispH=rtcSetHr; timeDispM=rtcSetMin; 
  xTimeH=107; yTime=52; xColon=xTimeH+42;
  xTimeM10=xTimeH+70; xTimeM1=xTimeH+86;   
              timeChange();              
 
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(print_text[155], 149, 142);    // | /  
    myGLCD.print(print_text[155], 219, 142);    // | /
    
// установка числа, месяца, года     DD/MM/YYYY Format
     //setFont(SMALL, 0, 0, 255, 0, 0, 0);
     myGLCD.setFont(RusFont6);
     myGLCD.setColor(0, 0, 255); // blue
     myGLCD.setBackColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[34]))); 
     myGLCD.print(buffer, 5, 158);         // (ДД/MM/ГГГГ)  День / Месяц / Год 
//--------------------------------------------------------------------------
    myGLCD.setColor(32, 255, 255);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[0]))); 
     myGLCD.print(buffer, 34, 43); //   ВРЕМЯ:
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[3]))); 
     myGLCD.print(buffer, 17, 55);  // в формате (время)
     myGLCD.print(buffer, 17, 144); // в формате (дата)
             
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[1]))); 
     myGLCD.print(buffer, 36, 132); //   ДАТА:
       setFont(LARGE, 16, 255, 27, 0, 0, 0);   // зел. шрифт      
  if ((rtcSetDy>=0) && (rtcSetDy<=9)){                // Set DAY
            myGLCD.print(print_text[187], 107, 142);  // 0
            myGLCD.printNumI(rtcSetDy, 123, 142);}
     else { myGLCD.printNumI(rtcSetDy, 107, 142);}
          
  if ((rtcSetMon>=0) && (rtcSetMon<=9)){              // Set MONTH
            myGLCD.print(print_text[187], 177, 142);  // 0
            myGLCD.printNumI(rtcSetMon, 193, 142);}
     else { myGLCD.printNumI(rtcSetMon, 177, 142);} 
 
           //setFont(LARGE, 255, 255, 255, 0, 0, 0); 
           myGLCD.printNumI(rtcSetYr, 247, 142); }    // Set YEAR 

void timeChange(){  // установка времени

      setFont(SMALL, 0, 0, 255, 0, 0, 0); 
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[166]))); 
      myGLCD.print(buffer, 32, yTime+17); timeCorrectFormat(); } // (24HR)  
       
void timeCorrectFormat(){ 
     // setFont(LARGE, 255, 255, 255, 0, 0, 0); // белый шрифт  
      setFont(LARGE, 16, 255, 27, 0, 0, 0);   // зел. шрифт
      myGLCD.print(print_text[56], xColon, yTime);        // :
     
       //  setFont(LARGE, 255, 255, 255, 0, 0, 0);  
  if ((timeDispH>=0) && (timeDispH<=9)){                  // Set HOUR
           myGLCD.print(print_text[187], xTimeH, yTime);  // 0
           myGLCD.printNumI(timeDispH, xTimeH+16, yTime);}
    else { myGLCD.printNumI(timeDispH, xTimeH, yTime);}  
                  
  if ((timeDispM>=0) && (timeDispM<=9)) {                  // Set MINUTES
          myGLCD.print(print_text[187], xTimeM10, yTime);  // 0
          myGLCD.printNumI(timeDispM, xTimeM1, yTime);}
   else { myGLCD.printNumI(timeDispM, xTimeM10, yTime);}
               }

    
/************** ЭКРАН УСТАНОВОК ТЕМПЕРАТУРЫ ******************************************************* dispScreen = 3 */
void tempScreen(boolean refreshAll=false){
     if (refreshAll){
       
// Текущая температура (монитор)
      myGLCD.setFont(RusFont1);
      myGLCD.setColor(88, 255, 238);           
      myGLCD.setBackColor(0, 0, 0);
      myGLCD.print(print_text[146], 275, 22);  // Текущ
      myGLCD.print(print_text[145], 280, 57);  // Макс
      myGLCD.setColor(200, 200, 200);
      myGLCD.drawCircle(311, 37, 1);           // значек цельсия (темп воды)
      myGLCD.drawCircle(311, 72, 1);           // значек цельсия (темп воды) max
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.printNumF( tempW, 1, 278, 36);    // отображение темп в аквариуме 
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.printNumF(MaxTempW, 1, 278 ,71);  // отображение максимальной температуры воды
      
  if (setTempC==0) { setTempC = 26.1; }        // change to 26.1 deg C 
  if (offTempC==0) { offTempC = 2.0; }         // change to 1 deg C 
            temp2beS = setTempC;               // отображение заданной температуры
            temp2beO = offTempC;               // гистерезис теипературы
            temp2beA = alarmTempC;             // гистерезис тревоги
     
     PrintStringIndex=2; printHeader ();       // УСТАНОВКА ТЕМПЕРАТУРЫ ВОДЫ
     
     myGLCD.setColor(0, 0, 255);                // синий цвет (royal)
     myGLCD.drawRoundRect(51, 24, 269, 187);    // нарисовать синюю рамку
     myGLCD.setColor(64, 64, 64); 
     myGLCD.drawRoundRect(273, 33, 315, 50);    // рамка вокруг текущ. температуры
     myGLCD.drawRoundRect(273, 68, 315, 85);    // рамка вокруг max
     myGLCD.drawRoundRect(273, 98, 315, 180);   // рамка внизу
     myGLCD.setColor(140, 140, 140);  // серый цвет

//======= лог за сутки 
            myGLCD.setColor(255, 255, 255);
            myGLCD.drawRoundRect(13, 83, 37, 127); // нарисовать рамку log 
            myGLCD.setColor(40, 244, 255);
            myGLCD.setBackColor(0, 0, 0);
                 myGLCD.print(F("K"), 22, 90);           // Л
                 myGLCD.print(F("N"), 22, 102);          // О
                 myGLCD.print(print_text[57], 22, 114);  // Г 
              
            myGLCD.setColor(120, 120, 120);     // серый цвет
            myGLCD.drawLine(24, 78, 24, 30);    // линия от кнопки лог вверх
            myGLCD.drawLine(24, 133, 24, 182);  // линия от кнопки лог вниз 
            myGLCD.drawLine(26, 78, 26, 30);    // вторая линия от кнопки лог вверх
            myGLCD.drawLine(26, 133, 26, 182);  // вторая линия от кнопки лог вниз
            
     myGLCD.setColor(64, 64, 64);                    // Draw Dividers in Grey
     myGLCD.drawRect(0, 196, 319, 194);              // Bottom Horizontal Divider  
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);

     myGLCD.setFont(RusFont1);
     myGLCD.setColor(0, 224, 134);   
     myGLCD.setBackColor(0, 0, 0);
     //setFont(RUS1, 0, 224, 134, 0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[37]))); 
     myGLCD.print(buffer, 71, 36);                           // заданная температура 
     myGLCD.setColor(104, 255, 40);    // зеленый
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[38]))); 
     myGLCD.print(buffer, CENTER, 87);                       // Гистерезис Температуры
     myGLCD.setColor(254, 136, 255);   // красный
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[39]))); 
     myGLCD.print(buffer, CENTER, 137);                      // Гистерезис Тревоги
     setFont(SMALL, 255, 255, 255, 0, 0, 0);
     myGLCD.drawCircle(239, 35, 1);     // знак цельсия          
     myGLCD.print(print_text[57], 244, 33); // C
 if (tempW == -127 || tempW == -196){       // sensor disconnected    
     myGLCD.setColor(0, 0, 0); 
     myGLCD.fillRect(270, 21, 315, 95); }    
     
          temp2beS = PlusMinusCountF (false, false, temM[0], temM[1], 150, 10, 40, 0.1, temp2beS);     // desired temperature
         if (PlsMnsPress == true) { PlsMnsPress = false;}
     temp2beO = PlusMinusCountF (false, false, temM[0], offM[1], 150, 0.2, 5, 0.1, temp2beO);     // temperature accurancy
         if (PlsMnsPress == true) { PlsMnsPress = false;}
     temp2beA = PlusMinusCountF (false, false, temM[0], SoundAlarmTm[1], 150, 1, 10, 0.1, temp2beA);    // alarm temperature
         if (PlsMnsPress == true) { PlsMnsPress = false;}
     
     }} 

/***************************ТЕСТ МАССИВА ОСВЕЩЕНИЯ******************************************* dispScreen = 5 */
void testArrayScreen(boolean refreshAll=false){    
  if (refreshAll){ 

     PrintStringIndex=3; printHeader (); // АВТОМАТИЧЕСКОЕ ТЕСТИРОВАНИЕ КАНАЛОВ
     myGLCD.fillRoundRect (1, 15, 318, 37); // очистка баннера "Test in Progress"
     myGLCD.setColor(64, 64, 64);            // Draw Dividers in Grey
     myGLCD.drawRect(0, 196, 319, 194);      // Bottom Horizontal Divider
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
     
     printButton ("", stsT[0], stsT[1], stsT[2], stsT[3], true);      // start/stop
     printButton (print_text[58], tenM[0], tenM[1], tenM[2], tenM[3], true);  // -10s
     printButton (print_text[59], tenP[0], tenP[1], tenP[2], tenP[3], true);  // +10s     
      myGLCD.print(print_text[96], stsT[0]+6, stsT[1]+15);      // START
      myGLCD.print(print_text[154], stsT[0]+15, stsT[1]+40);    // TEST
                        } else { min_cnt=560;                   // начало теста с 9:20  
     myGLCD.setColor(0, 0, 0);
     myGLCD.fillRect (1, 37, 318, 99);        // clear test results if any
     myGLCD.fillRect (1, 187, 318, 227);      // clear the "Back" and "Cancel" Buttons
     myGLCD.setColor(130, 130, 130);          // цвет серый
     myGLCD.drawRoundRect (6, 41, 98, 105);   // рамка вокруг времени
     myGLCD.setColor(110, 110, 110);
     myGLCD.drawRoundRect (119, 39, 311, 53); // рамка вокруг УРОВНИ ВЫХОДОВ (0-255)
          
       myGLCD.setColor(0, 0, 255); // синяя кнопка тест выкл.
     myGLCD.fillRect(stsT[0]+5, stsT[1]+5, stsT[2]-5, stsT[3]-40);  // clear 'start'
       setFont(LARGE, 255, 0, 0, 0, 0, 255); 
     myGLCD.print(print_text[4], stsT[0]+15, stsT[1]+15);  // STOP

      myGLCD.setFont(RusFont1);
      myGLCD.setColor(200, 200, 200);
      myGLCD.setBackColor(0, 0, 0);
     //setFont(RUS1, 200, 200, 200, 0, 0, 0);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[40]))); 
      myGLCD.print(buffer, 239, 166);     // Время 
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[41]))); 
      myGLCD.print(buffer, 235, 175);     // Вперед
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[40]))); 
      myGLCD.print(buffer, 37, 166);      // Время 
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[42]))); 
      myGLCD.print(buffer, 37, 175);      // Назад
             
     myGLCD.setColor(255, 0, 0);        // красный шрифт
     myGLCD.fillRect (2, 15, 318, 37);  // баннер "Test in Progress"
     myGLCD.drawRoundRect (stsT[0], stsT[1], stsT[2], stsT[3]); // red button during test
     setFont(LARGE, 255, 255, 255, 255, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[43]))); 
     myGLCD.print(buffer, CENTER, 17);   // Test in Progress
     setFont(LARGE, 0, 255, 0, 0, 0, 0); 
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[44]))); 
      myGLCD.print(buffer, 19, 49);      // TIME 
          
     myGLCD.setFont(RusFont2);
     myGLCD.setColor(88, 229, 255);
     myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS2, 88, 229, 255, 0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[45]))); 
     myGLCD.print(buffer, 125, 41);                   // УРОВНИ ВЫХОДОВ
     
   while (LEDtestTick){                               // test LED and speed up time  
      unsigned long currentMillis = millis();
      if (myTouch.dataAvailable()){ processMyTouch();}
      if (currentMillis - previousMillisLED > 800){    // change time every 0.5s 
         previousMillisLED = currentMillis; min_cnt++;
         int hours = min_cnt/60;
         int minut = min_cnt%60;
         
// Test LED 
         setFont(LARGE, 255, 255, 255, 0, 0, 0);  
         myGLCD.print(print_text[56], 44, 78);     // :        
         myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта 
               
     if ((hours>=0)&&(hours<=9)){ myGLCD.printNumI(0, 12, 75);   
                                  myGLCD.printNumI(hours, 28, 75);}       
                           else { myGLCD.printNumI(hours, 12, 75);}       
                      
     if ((minut>=0)&&(minut<=9)){ myGLCD.printNumI(0, 60, 75);    
                                  myGLCD.printNumI(minut, 76, 75);}      
                           else { myGLCD.printNumI(minut, 60, 75);}     

           currentTime = millis(); 
     if(currentTime >=(loopTime + 250)){         // сравниваем текущий таймер с переменной loopTime + 1 секунда
             loopTime = currentTime; }           // в loopTime записываем новое значение
// ----------- 
         myGLCD.setFont(RusFont1);
         myGLCD.setColor(255, 255, 204);
         myGLCD.setBackColor(0, 0, 0);
     String wled = print_text[104] + String(wled_out) + print_text[153] + " ";  // БЕЛЫЙ :
         char bufferW[11]; wled.toCharArray(bufferW, 11);
         myGLCD.print(bufferW, 133, 57);
//------------         
         myGLCD.setColor(255, 255, 255);
         myGLCD.setBackColor(0, 0, 0);
     String bled = print_text[105] + String(bled_out) + print_text[153] + " ";  // ГОЛУБОЙ     
         char bufferB[11]; bled.toCharArray(bufferB, 11);
         myGLCD.print(bufferB, 133, 67);  
//------------ 
         myGLCD.setColor(58, 95, 205);
         myGLCD.setBackColor(0, 0, 0); //                 v -два пробела
     String rbled = print_text[106] + String(rbled_out) + print_text[153] + " "; // СИНИЙ
         char bufferRB[11]; rbled.toCharArray(bufferRB, 11);
         myGLCD.print(bufferRB, 133, 77); 
//------------     
         myGLCD.setColor(255, 0, 0);
         myGLCD.setBackColor(0, 0, 0);
     String rled = print_text[107] + String(rled_out) + print_text[153] + " ";  // КРАСНЫЙ
         char bufferR[11]; rled.toCharArray(bufferR, 11);
         myGLCD.print(bufferR, 225, 57);
//------------      
         myGLCD.setColor(224, 102, 255);
         myGLCD.setBackColor(0, 0, 0);
     String uvled = print_text[108] + String(uvled_out) + print_text[153] + " "; // УЛЬТРА
         char bufferUV[11]; uvled.toCharArray(bufferUV, 11);
         myGLCD.print(bufferUV, 225, 67); 
//------------         
         myGLCD.setColor(255, 143, 32);
         myGLCD.setBackColor(0, 0, 0);
     String oLed = print_text[109] + String(oLed_out) + print_text[153] + " "; // ОРАНЖ.
         char bufferS[11]; oLed.toCharArray(bufferS, 11);
         myGLCD.print(bufferS, 225, 77);
//------------         
         myGLCD.setColor(0, 255, 0);
         myGLCD.setBackColor(0, 0, 0);
     String gled = print_text[110] + String(gled_out) + print_text[153] + " ";  // ЗЕЛЕНЫЙ:
         char bufferGR[11]; gled.toCharArray(bufferGR, 11);
         myGLCD.print(bufferGR, 225, 87);  
         LED_levelo_output(); checkTempC(); } } } }    // TimeDateBar(); 
               
/*********************************************** ТЕСТ ОСВЕЩЕНИЯ ПОКАНАЛЬНО *************************************************** dispScreen = 6 */
void testIndLedScreen(){

  PrintStringIndex=4; printHeader (); // РУЧНОЙ ТЕСТ В ДИАПАЗОНЕ 0-2000
    
  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.setColor(255, 255, 204);
  myGLCD.print(print_text[184], 54, 176);   //  WHT
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 
  
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(print_text[183], 92, 176);    // BLU
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 
  
  myGLCD.setColor(58, 95, 205);
  myGLCD.print(print_text[182], 130, 176);   // RBL
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 

  myGLCD.setColor(255, 0, 0);
  myGLCD.print(print_text[181], 168, 176);   // RED
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 

  myGLCD.setColor(224, 102, 255);  
  myGLCD.print(print_text[180], 206, 176);   // UVL
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));  

  myGLCD.setColor(255, 143, 32);     // initial Prints
  myGLCD.print(print_text[179], 244, 176);   // ORG
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 

  myGLCD.setColor(0, 255, 0);
  myGLCD.print(print_text[178], 282, 176);   // GRN  
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46]))); 

  for (int b=0; b<7; b++){ drawUpButtonSlide((b*38)+49, 17);} 
  for (int b=0; b<7; b++){ drawDownButtonSlide((b*38)+49, 200);}     
            drawSliderBarGraph(); 
  
       for (int i=0; i<7; i++){  
  if (i==0){sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; sbX1=49; sbX2=79; yWHT; x=54; y=25; UpDnButtonSlide();}    // WHITE 
  if (i==1){sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; sbX1=87; sbX2=117; yBLU; x=92; y=25; UpDnButtonSlide();}     // BLUE 
  if (i==2){sbR=20; sbG=60; sbB=255; sbX1=125; sbX2=155; yRBL; x=130; y=25; UpDnButtonSlide();}   // ROYAL BLUE
  if (i==3){sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];sbX1=163; sbX2=193; yRED; x=168; y=25; UpDnButtonSlide();}      // RED 
  if (i==4){sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; sbX1=201; sbX2=231; yUVL; x=206; y=25; UpDnButtonSlide();} // UV     
  if (i==5){sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; sbX1=239; sbX2=269; ySMP; x=244; y=25; UpDnButtonSlide();}  // Orange                                                            
  if (i==6){sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];sbX1=277; sbX2=307; yGRN; x=282; y=25; UpDnButtonSlide();}}}    // Green 
           
/************************************ НАСТРОЙКА КАНАЛОВ ПО ЦВЕТАМ ************************************** dispScreen = 7 */
void ledColorViewScreen(){

  PrintStringIndex=5; printHeader (); // НАСТРОЙКА КАНАЛОВ ПО ЦВЕТАМ

  myGLCD.setColor(64, 64, 64);                  // Draw Dividers in Grey
  myGLCD.drawRect(0, 196, 319, 194);            // Bottom Horizontal Divider
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); 

  myGLCD.setColor(255, 255, 204);
  myGLCD.fillRoundRect(10, 20, 150, 50); // размер кнопки белого канала
  myGLCD.setFont(RusFont6); // RusFont6
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(255, 255, 204);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[47]))); 
  myGLCD.print(buffer, 61, 30);     // БЕЛЫЙ
  
  myGLCD.setColor(58, 95, 205);
  myGLCD.fillRoundRect(10, 60, 150, 90);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(58, 95, 205);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[48]))); 
  myGLCD.print(buffer, 60, 70);    // СИНИЙ

  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRoundRect(10, 100, 150, 130);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[49]))); 
  myGLCD.print(buffer, 52, 110);    // КРАСНЫЙ

  myGLCD.setColor(176, 176, 176);  
  myGLCD.fillRoundRect(170, 140, 310, 170);
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(176, 176, 176);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[50]))); 
  myGLCD.print(buffer, 224, 150);    // ЛУНА
  
  myGLCD.setColor(255, 143, 32);
  myGLCD.fillRoundRect(170, 100, 310, 130);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 143, 32);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[51]))); 
  myGLCD.print(buffer, 205, 110);     // ОРАНЖЕВЫЙ
   
  myGLCD.setColor(224, 102, 255);
  myGLCD.fillRoundRect(170, 60, 310, 90);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(224, 102, 255);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[52]))); 
  myGLCD.print(buffer, 201, 70);      // ФИОЛЕТОВЫЙ
   
  myGLCD.setColor(255, 255, 255);
  myGLCD.fillRoundRect(170, 20, 310, 50);
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(255, 255, 255);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[53]))); 
  myGLCD.print(buffer, 212, 30);      // ГОЛУБОЙ
 
  myGLCD.setColor(0, 180, 86);   // Green
  myGLCD.fillRoundRect(10, 140, 150, 170);
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(0, 180, 86);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[54]))); 
  myGLCD.print(buffer, 51, 150);      // ЗЕЛЕНЫЙ

  myGLCD.setColor(255, 255, 255);    // цвет белый
  for (int x=0; x<2; x++){ 
  for (int y=0; y<4; y++){  
  myGLCD.drawRoundRect((x*160)+10,(y*40)+20,(x*160)+150,(y*40)+50); // белые рамки вокруг кнопок
  //myGLCD.setColor(255, 8, 82);   // красный
  myGLCD.drawRoundRect(10, 20, 150, 50);        // рамка вокруг белого канала 
 // myGLCD.drawBitmap(145, 185, 32, 25, gr, 1); // картинка переход к графикам
   }}} 
   
/*********************************** ЗНАЧЕНИЯ ЯРКОСТИ ********************************* dispScreen = 8 */
void ledValuesScreen(){
    int a;
  
if (COLOR==1) { for (byte i=0; i<96; i++) tled[i] = wled[i];
     setFont(SMALL, 255, 255, 204, 255, 255, 204);
      printFrame();
      myGLCD.setColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[55]))); 
     myGLCD.print(buffer, CENTER, 3);            // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ БЕЛОГО КАНАЛА
      printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==2) { for (byte i=0; i<96; i++) tled[i] = bled[i];
     setFont(SMALL, 255, 255, 255, 255, 255, 255);
      printFrame();
      myGLCD.setColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[55]))); 
     myGLCD.print(buffer, CENTER, 3);            // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ГОЛУБОГО КАНАЛА
      printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==3) { for (byte i=0; i<96; i++) tled[i] = rbled[i];
     setFont(SMALL, 58, 95, 205, 58, 95, 205);
      printFrame();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[58]))); 
     myGLCD.print(buffer, CENTER, 3);           // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ СИНЕГО КАНАЛА
      printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==4) { for (byte i=0; i<96; i++) tled[i] = rled[i];
     setFont(SMALL, 255, 0, 0, 255, 0, 0);
      printFrame();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[59]))); 
     myGLCD.print(buffer, CENTER, 3);           // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ КРАСНОГО КАНАЛА
       printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==5) { for (byte i=0; i<96; i++) tled[i] = uvled[i];
     setFont(SMALL, 224, 102, 255, 224, 102, 255);
      printFrame();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[60]))); 
     myGLCD.print(buffer, CENTER, 3);               // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ФИОЛЕТОВОГО КАНАЛА
       printMes(); }    // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==6) { for (byte i=0; i<96; i++) tled[i] = oLed[i];
     setFont(SMALL, 255, 143, 32, 255, 143, 32);
      printFrame();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[61]))); 
     myGLCD.print(buffer, CENTER, 3);              // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ОРАНЖЕВОГО КАНАЛА
       printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

if (COLOR==7) { for (byte i=0; i<96; i++) tled[i] = gled[i];  // Green
     setFont(SMALL, 0, 255, 0, 0, 255, 0);
      printFrame();
      myGLCD.setColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[62]))); 
     myGLCD.print(buffer, CENTER, 3);               // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ЗЕЛЕНОГО КАНАЛА 
       printMes(); }   // ДЛЯ ПЕРЕХОДА К ГРАФИКАМ-НАЖАТЬ НА ЭКРАН

 // MOON   
 if (COLOR==8) { tMinI=MinI; tMaxI=MaxI;        
     PrintStringIndex=6;  printHeader (); // УСТАНОВКА ЯРКОСТИ ЛУНЫ
    
     myGLCD.setFont(RusFont2);
     myGLCD.setColor(255, 255, 255);
     myGLCD.setBackColor(0, 0, 0);
     //setFont(RUS2, 255, 255, 255, 0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[63]))); 
    myGLCD.print(buffer, 36, 20);     // Минимальная
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[65]))); 
    myGLCD.print(buffer, 52, 32);     //   Яркость    
    myGLCD.drawBitmap(52, 47, 53, 53, First_Quarter, 1);

    myGLCD.print(print_text[176], 46, 122);       // (0--100)(48, 122) 255  0...100 %"
    myGLCD.print(print_text[60], 20, 177);  // -1       
    myGLCD.print(print_text[61], 124, 177); // +5   
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[64]))); 
    myGLCD.print(buffer, 195, 20);      // Максимальная
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[65]))); 
    myGLCD.print(buffer, 214, 32);      //   Яркость 
    myGLCD.drawBitmap(215, 48, 53, 53, Full_Moon, 1);
  
    myGLCD.print(print_text[176], 207, 122);   // (0--100)(209, 122) 255  0...100 %"
    myGLCD.print(print_text[60], 182, 177);    // -1      
    myGLCD.print(print_text[61], 286, 177);    // +5    
    myGLCD.setFont(RusFont1);    
    myGLCD.setColor(176, 176, 176);  
    myGLCD.setBackColor(0, 0, 0);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[66]))); 
    myGLCD.print(buffer, 39, 108);       // НОВАЯ ЛУНА
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[67]))); 
    myGLCD.print(buffer, 196, 108);      // ПОЛНАЯ ЛУНА

    setFont(LARGE, 255, 255, 255, 0, 0, 0);
                   myGLCD.print(print_text[111], 55, 152);  // (три пробела)
    if (tMinI<=9){ myGLCD.printNumI(tMinI, 71, 152);}
    if ((tMinI>=10)&&(tMinI<=99)){ myGLCD.printNumI(tMinI, 63, 152);}
    if (tMinI>=100){ myGLCD.printNumI(tMinI, 55, 152);}
      
                   myGLCD.print(print_text[111], 217, 152); // (три пробела)
    if (tMaxI<=9){ myGLCD.printNumI(tMaxI, 233, 152);}
    if ((tMaxI>=10)&&(tMaxI<=99)){ myGLCD.printNumI(tMaxI, 225, 152);}
    if (tMaxI>=100){ myGLCD.printNumI(tMaxI, 217, 152);}
   
    printButton(print_text[28], MINiM[0], MINiM[1], MINiM[2], MINiM[3], true);  // Minimum Illum. minus
    printButton(print_text[27], MINiP[0], MINiP[1], MINiP[2], MINiP[3], true);  // Minimum Illum. plus
    printButton(print_text[28], MAXiM[0], MAXiM[1], MAXiM[2], MAXiM[3], true);  // Max Illum. minus
    printButton(print_text[27], MAXiP[0], MAXiP[1], MAXiP[2], MAXiP[3], true);  // Max Illum. plus

    myGLCD.setColor(64, 64, 64);      // серый
    myGLCD.drawRect(158, 14, 160, 194);
    myGLCD.drawRect(0, 196, 319, 194);
    myGLCD.drawRect(54, 147, 104, 173);  // Min
    myGLCD.drawRect(216, 147, 266, 173); // Max
    
    myGLCD.setColor(0, 0, 0);
    myGLCD.drawLine(159, 193, 159, 195);
    printButtonRUS(print_text[5], back[0], back[1], back[2], back[3], SMALL);
    printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); }  
    
if (COLOR!=8) {  // переход к настройке уровней (все, кроме color 8)
    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    for (int i=0; i<12; i++){     // показать таблицу уровней
        myGLCD.setColor(0, 255, 255);
        myGLCD.printNumI((i*2), (i*26)+13, 14);
        myGLCD.printNumI(((i*2)+1), (i*26)+13, 24);
     for (int j=0; j<8; j++){ a= (i*8)+j;         
        myGLCD.setColor(255, 255, 255);
        myGLCD.printNumI(tled[a], (i*26)+7, (j*18)+39);
        myGLCD.setColor(100, 100, 100);
        myGLCD.drawRect((i*26)+4, (j*18)+35, (i*26)+30, (j*18)+53); } }
        myGLCD.setColor(64, 64, 64);       
        myGLCD.drawRect(0, 196, 319, 194);  
        
    printButtonRUS(print_text[5], back[0], back[1], back[2], back[3], SMALL);   
    printButtonRUS(print_text[6], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL);
    printButtonRUS(print_text[3], eeprom[0], eeprom[1], eeprom[2], eeprom[3]); } }
    

//======================== выбор представления значений яркости таблица график ===========================

void ledValuegScreen(){ // print TOP NAME bar & horisontal buttons, with timetable 12x8
   
    myGLCD.setColor(0, 0, 0);       // очистить верхнюю область экрана
    myGLCD.fillRect(0, 0, 319, 15); // clear only header
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(170, 170, 170); // серый
    myGLCD.setBackColor(0, 0, 0);
   //setFont(RUS1, 170, 170, 170, 0, 0, 0);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[35]))); 
    myGLCD.print(buffer, 83, 5);    // ГРАФИК СВЕТОВОГО ДНЯ
                 
  if (COLOR==1){for (byte i=0; i<96; i++)tled[i]=wled[i]; sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; } //(white)
  if (COLOR==2){for (byte i=0; i<96; i++)tled[i]=bled[i];  sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2];}
  if (COLOR==3){for (byte i=0; i<96; i++)tled[i]=rbled[i]; sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2];}
  if (COLOR==4){for (byte i=0; i<96; i++)tled[i]=rled[i]; sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];}  
  if (COLOR==5){for (byte i=0; i<96; i++)tled[i]=uvled[i]; sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2];}  
  if (COLOR==6){for (byte i=0; i<96; i++)tled[i]=oLed[i]; sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2];}
  if (COLOR==7){for (byte i=0; i<96; i++)tled[i]=gled[i]; sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];}

    for (byte i=0; i<=95; i++){ byte TEMP=tled[i];  // find START graph Time
	   if (TEMP>0){ StartTime=i-1;
     if (StartTime == 255){StartTime=0;} break; }}  // 255

    for (byte i=95; i>0; i-- ){ byte TEMP=tled[i];  // find STOP graph Time
	   if (TEMP>0){StopTime=i+1; break; }}
//		byte stepScale = (294 - 26)/(StopTime-StartTime);    // 26 and 318 - left / right
		drawLedStaticChartP1 ();		             // calculate draw TOP ON/OFF time and X/Y border
       		draw_one_led_graph(); 
		drawLedStaticChartP2 ();		             // draw BOT ON/OFF time and X/Y scale and tick mark (15min/30min/1hour)

		myGLCD.setColor(64, 64, 64); // цвет серый      
		myGLCD.drawRect(0, 196, 319, 194); 
                printButtonRUS(print_text[5], back[0], back[1], back[2], back[3], SMALL);   
                printButtonRUS(print_text[6], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL);
                printButtonRUS(print_text[3], eeprom[0], eeprom[1], eeprom[2], eeprom[3]); } 

/***************************************ИЗМЕНЕНИЕ ЗНАЧЕНИЙ ЯРКОСТИ ******************************************** dispScreen = 9 */
void ledChangeScreen(){

  if (COLOR==1){ setFont(SMALL, 255, 255, 204, 255, 255, 255);
       printFrame2();
       myGLCD.setColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[55]))); 
     myGLCD.print(buffer, CENTER, 3);            // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ БЕЛОГО КАНАЛА
        sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; } 
        
  if (COLOR==2){ setFont(SMALL, 255, 255, 255, 9, 184, 255);
      printFrame2();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[55]))); 
     myGLCD.print(buffer, CENTER, 3);            // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ГОЛУБОГО КАНАЛА  
        sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; }  
        
  if (COLOR==3){ setFont(SMALL, 58, 95, 205, 58, 95, 205);
      printFrame2();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[58]))); 
     myGLCD.print(buffer, CENTER, 3);           // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ СИНЕГО КАНАЛА
       sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; } 
       
  if (COLOR==4){ setFont(SMALL, 255, 0, 0, 255, 0, 0);
      printFrame2();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[59]))); 
     myGLCD.print(buffer, CENTER, 3);           // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ КРАСНОГО КАНАЛА
       sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; } 
       
  if (COLOR==5){ setFont(SMALL, 224, 102, 255, 224, 102, 255);
      printFrame2();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[60]))); 
     myGLCD.print(buffer, CENTER, 3);               // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ФИОЛЕТОВОГО КАНАЛА
       sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; } 
       
  if (COLOR==6){ setFont(SMALL, 255, 143, 32, 255, 143, 32);
      printFrame2();
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[61]))); 
     myGLCD.print(buffer, CENTER, 3);              // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ОРАНЖЕВОГО КАНАЛА
       sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; }
       
  if (COLOR==7){ setFont(SMALL, 0, 255, 0, 0, 255, 0);
      printFrame2();
      myGLCD.setColor(0, 0, 0);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[62]))); 
     myGLCD.print(buffer, CENTER, 3);               // ЗНАЧЕНИЯ ЯРКОСТИ ДЛЯ ЗЕЛЕНОГО КАНАЛА 
       sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; } 
       
//------------------------------------------------------------------------------------------------      
   // Цвет еще не нажатых цифр 
       setFont(SMALL, 190, 190, 190, 0,0,0); 
  for (int i=0; i<12; i++){  
        myGLCD.setColor(190, 190, 190);
      
  if (i<5){ 
      myGLCD.printNumI((i*2), (i*26)+15, 17);
      myGLCD.printNumI(((i*2)+1), (i*26)+15, 28);}
    else {
      myGLCD.printNumI((i*2), (i*26)+11, 17);
      myGLCD.printNumI(((i*2)+1), (i*26)+11, 28);}
      myGLCD.setColor(100, 100, 100);
      myGLCD.drawRect((i*26)+4, 15 , (i*26)+30, 41); } // нарисовать таблицу интервалов времени (i*26)+4, 16 , (i*26)+30, 41);    

// 2 кнопки 
  myGLCD.setColor(0, 0, 0);	             
  //myGLCD.fillRect(1, 226, 318, 240);       // clear bottom time|date bar
  myGLCD.drawRect(4, 43, 30, 223);           // left menu bar
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(8, 76, 26, 130);      // OK Button 
  myGLCD.fillRoundRect(8, 145, 26, 219);   
  myGLCD.setColor(255, 255, 255);  // Кайма
  myGLCD.drawRoundRect(8, 76, 26, 130);       // OK Button 
  myGLCD.drawRoundRect(8, 145, 26, 219);      // CANCEL Button
  
   myGLCD.setFont(RusFont1);
   myGLCD.setColor(255, 255, 255);
   myGLCD.setBackColor(0, 0, 255);
  //setFont(RUS1, 255, 255, 255, 0, 0, 255);     
   myGLCD.print(print_text[187], 14, 96);  // О  
   myGLCD.print(F("J"), 14, 106);          // К

   myGLCD.print(print_text[187], 14, 153); // О 
   myGLCD.print(print_text[23], 14, 163);  // Т
   myGLCD.print(F("L"), 14, 173);          // М
   myGLCD.print(print_text[85], 14, 183);  // Е  
   myGLCD.print(F("M"), 14, 193);          // Н
   myGLCD.print(F("@"), 14, 203);          // А

   myGLCD.setColor(255, 0, 0);
   myGLCD.setBackColor(0, 0, 0);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[68]))); 
    myGLCD.print(buffer, 56, 110);                             //  В ВЕРХНЕЙ ЧАСТИ ЭКРАНА ВЫБРАТЬ 
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[69]))); 
    myGLCD.print(buffer, 43, 122);                             // ЛЮБОЙ ДВУХЧАСОВОЙ ОТРЕЗОК ВРЕМЕНИ
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[70]))); 
    myGLCD.print(buffer, 60, 134);                             //  И УСТАНОВИТЬ ЗНАЧЕНИЯ ЯРКОСТИ
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[71]))); 
    myGLCD.print(buffer, 94, 146); }                           //      ДЛЯ ВЫБРАННОГО КАНАЛА
                                                             
        

 
/************************************* ГЛАВНЫЕ НАСТРОЙКИ, СТР.1******************************** dispScreen = 14 */ 
void generalSettingsScreen_1(){

   PrintStringIndex=10; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.1

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194); // Bottom Horizontal Divider 
    myGLCD.drawLine(0, 47, 319, 47);   // разделительная линия (поиск датч / бекап)
    myGLCD.drawLine(0, 81, 319, 81);   
    myGLCD.drawLine(0, 115, 319, 115); // разделительная линия (сброс / установ)    
    myGLCD.drawLine(0, 151, 319, 151); // разделительная линия (установ / устан. темп.)
    
// кнопки    
    myGLCD.setColor(0, 0, 255);   // синий цвет                 
    myGLCD.fillRoundRect(195, 20, 295, 40);       // detect
    myGLCD.fillRoundRect(205, 54, 285, 74);       // backup
    myGLCD.fillRoundRect(205, 88, 285, 108);      // сброс
// рамки
    myGLCD.setColor(255, 255, 255);  // шрифт белый                  
    myGLCD.drawRoundRect(195, 20, 295, 40);       // detect
    myGLCD.drawRoundRect(205, 54, 285, 74);       // backup
    myGLCD.drawRoundRect(205, 88, 285, 108);      // сброс
    
  printButtonRUS(print_text[2], backGS[0], backGS[1], backGS[2], backGS[3], SMALL);
  printButtonRUS(print_text[17], nextGS[0], nextGS[1], nextGS[2], nextGS[3], SMALL);
  printButtonRUS(print_text[3], prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3], SMALL);
  printButtonRUS(print_text[1], canCgs[0], canCgs[1], canCgs[2], canCgs[3], SMALL); 
    
         myGLCD.setFont(RusFont6); // 1
         myGLCD.setColor(0, 255, 0);
         myGLCD.setBackColor(0, 0, 0); 
      //setFont(RUS1, 0, 255, 0, 0, 0, 0);
      
     myGLCD.print(print_text[211], 12, 24);  // ПОИСК ДАТЧИКОВ ТЕМПЕР. 27
     myGLCD.print(print_text[210], 16, 58);  // CОХР. НАСТР. НА КАРТУ  60
     myGLCD.print(print_text[209], 31, 92);  // СБРОСИТЬ НАСТРОЙКИ     94
         
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[133]))); 
     myGLCD.print(buffer, 36, 127);          // ПОДСВЕТКА ЭКРАНА  129
 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[134]))); 
     myGLCD.print(buffer, 17, 165);          // ТЕМП. ВКЛ. ВЕНТИЛЯТОРА  
      
           myGLCD.setFont(RusFont2);
           myGLCD.setColor(255, 255, 255); // шрифт белый
           myGLCD.setBackColor(0, 0, 255);
       //setFont(RUS2, 255, 255, 255, 0, 0, 255);
       myGLCD.print(print_text[195], 206, 25);    // ПОИСК ДАТЧ. 
       myGLCD.print(print_text[194], 225, 59);    // БЕКАП
       myGLCD.print(print_text[193], 224, 93);    // СБРОС    
                genSetSelect_1(); 
      	          
	bitClear(GlobalStatus1Byte,0); }    // reset bit for Y/N button
     
/*********************************** ГЛАВНЫЕ НАСТРОЙКИ, СТР.2 ************************ dispScreen = 15 */
void generalSettingsScreen_2(){  

  PrintStringIndex=11; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.2

  myGLCD.setColor(64, 64, 64);     // серый цвет
  myGLCD.drawLine(0, 88, 319, 88);  
  myGLCD.drawLine(0, 162, 319, 162);
  myGLCD.drawRect(0, 196, 319, 194); // двойная линия над кнопками (back next) 
      printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
      printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL); 
      printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);     
    myGLCD.setFont(RusFont6); // 1
    myGLCD.setColor(0, 255, 0);
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 0, 255, 0, 0, 0, 0);  
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[135]))); 
    myGLCD.print(buffer, 15, 38);          // АВТО-УМЕНЬШ. ЯРКОСТИ 40
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[136]))); 
    myGLCD.print(buffer, 43, 53);          //    ПРИ ПЕРЕГРЕВЕ  51
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[137]))); 
    myGLCD.print(buffer, 53, 113);         //    УСТАНОВКИ 115
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[138]))); 
    myGLCD.print(buffer, 27, 128);         // ХРАНИТЕЛЯ ЭКРАНА 126
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[139]))); 
    myGLCD.print(buffer, 18, 173);         //  ОГРАНИЧЕНИЕ МОЩНОСТИ 175
 
    genSetSelect_2(); }
    
/******* GENERAL SETTINGS PG3 SCREEN ************************************************** dispScreen = 14 */ 
/* 
void generalSettingsScreen_3(){

   PrintStringIndex=12; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.3
   
   myGLCD.setColor(64, 64, 64);     // серый цвет
  myGLCD.drawLine(0, 83, 319, 83);  
  myGLCD.drawLine(0, 162, 319, 162);
  myGLCD.drawRect(0, 196, 319, 194); 
      
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
    printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
}
*/
   
/**************************** УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ ************************************* dispScreen = 16 */
void ChangeFanTempsScreen(boolean refreshAll=false){ 
   //  String deg;
     
  if (refreshAll){
  if (setTempToBeginHeatsink1FanC==0){ setTempToBeginHeatsink1FanC = 29.0;}   // change to 29 deg C
  if (setTempToBeginHeatsink2FanC==0) { setTempToBeginHeatsink2FanC = 29.0;}  // change to 29 deg C  
     temp2beHFan=setTempToBeginHeatsink1FanC; temp2beSFan=setTempToBeginHeatsink2FanC; 
               
     PrintStringIndex=14; printHeader (); // УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ
 
     myGLCD.setColor(64, 64, 64);
     myGLCD.drawRect(0, 103, 319, 105); 
     myGLCD.setColor(0, 123, 176);        // синий   
     myGLCD.drawRect(123, 56, 197, 80);   // рамка вокруг вентелятор охлаждения радиатора-1 
     myGLCD.drawRect(123, 147, 197, 171); // рамка вокруг вентелятор охлаждения радиатора-2  
     myGLCD.setColor(110, 110, 110);      // цвет серый
     myGLCD.drawRect(59, 34, 260, 100);   // большая рамка вокруг температуры включения 1
     myGLCD.drawRect(59, 125, 260, 191);  // большая рамка вокруг температуры включения 2

       myGLCD.setFont(RusFont1);
       myGLCD.setColor(0, 255, 0); 
       myGLCD.setBackColor(0, 0, 0); 
       //setFont(RUS1, 0, 255, 0, 0, 0, 0);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[140]))); 
        myGLCD.print(buffer, CENTER, 20);      // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-1
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[141]))); 
        myGLCD.print(buffer, CENTER, 112);     // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-2
                    
     myGLCD.setFont(RusFont1);
     myGLCD.setColor(220, 220, 220);
     myGLCD.setBackColor(0, 0, 0);
     //setFont(RUS1, 220, 220, 220, 0, 0, 0);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[142]))); 
      myGLCD.print(buffer, 66, 39);       // д1 ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[142]))); 
      myGLCD.print(buffer, 66, 130);      // д2 ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ
      
      myGLCD.print(print_text[146], 270, 39);   // Текущ  для датчика радиатора 1
      myGLCD.print(print_text[145], 275, 69);   // Макс  для датчика радиатора 1    
      myGLCD.print(print_text[146], 270, 130);  // Текущ  для датчика радиатора 2
      myGLCD.print(print_text[145], 275, 160);  // Макс  для датчика радиатора 2
      
     setFont(SMALL, 255, 255, 255, 0, 0, 0);
     myGLCD.drawCircle(239, 38, 1);      // радиатор д-1           
     myGLCD.print(print_text[57], 244, 36);
     myGLCD.drawCircle(239, 129, 1);     // радиатор д-2           
     myGLCD.print(print_text[57], 244, 127);
         
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[144]))); 
        myGLCD.print(buffer, CENTER, 86);     // (25-50)
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[144]))); 
        myGLCD.print(buffer, CENTER, 177);    // (25-50)

     printButton(print_text[28], HoodFanTm[0], HoodFanTm[1], HoodFanTm[2], HoodFanTm[3], LARGE);
     printButton(print_text[27], HoodFanTp[0], HoodFanTp[1], HoodFanTp[2], HoodFanTp[3], LARGE);      
     printButton(print_text[28], SumpFanTm[0], SumpFanTm[1], SumpFanTm[2], SumpFanTm[3], LARGE);
     printButton(print_text[27], SumpFanTp[0], SumpFanTp[1], SumpFanTp[2], SumpFanTp[3], LARGE);
     
     printButton(print_text[152], SalaRm[0], SalaRm[1], SalaRm[2], SalaRm[3], SMALL); // sound alarm

     myGLCD.setColor(64, 64, 64);
     myGLCD.drawRect(0, 196, 319, 194);     
     myGLCD.drawRoundRect(268, 49, 311, 65);    // рамка вокруг текущ. температуры радиатора 1
     myGLCD.drawRoundRect(268, 78, 311, 95);    // рамка вокруг max радиатора 1    
     myGLCD.drawRoundRect(268, 140, 311, 156);  // рамка вокруг текущ. температуры радиатора 2
     myGLCD.drawRoundRect(268, 169, 311, 186);  // рамка вокруг max радиатора 2
     
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); }
    
     setFont(LARGE, 255, 255, 255, 0, 0, 0);
     myGLCD.printNumF(temp2beHFan, 1, CENTER, 61);  // Fan 1   
     myGLCD.printNumF(temp2beSFan, 1, CENTER, 152); // Fan 2 
     
     //setFont(SMALL, 0, 255, 0, 0, 0, 0);    // зеленый шрифт для текущ. 
     
 if (tempH1 == -127 || tempH1 == -196){   // sensor disconnected
     myGLCD.setColor(0, 0, 0); myGLCD.fillRect(265, 34, 311, 100);       
     } else { setFont(SMALL, 0, 255, 0, 0, 0, 0);  // зеленый шрифт для текущ.     
     myGLCD.printNumF( tempH1, 1, 275, 51); }      // отображение текущ. температуры радиатора 1
     
 if (tempH2 == -127 || tempH2 == -196){   // sensor disconnected
     myGLCD.setColor(0, 0, 0); myGLCD.fillRect(265, 125, 311, 191);     
      } else { setFont(SMALL, 0, 255, 0, 0, 0, 0);  // зеленый шрифт для текущ. 
     myGLCD.printNumF( tempH2, 1, 275, 142);  }     // отображение текущ. температуры радиатора 2
              
  if (MaxTempH1 == 0.0){   // sensor disconnected
      myGLCD.setColor(0, 0, 0); myGLCD.print(print_text[153], 275, 81); // (два пробела)  
      } else { setFont(SMALL, 255, 0, 0, 0, 0, 0);  // красный шрифт для макс.    
      myGLCD.printNumF( MaxTempH1, 1, 275, 81);  }  // отображение max температуры радиатора 1
           
  if (MaxTempH2 == 0.0){  // sensor disconnected
      myGLCD.setColor(0, 0, 0); myGLCD.print(print_text[153], 275, 172); // (два пробела)
      } else { setFont(SMALL, 255, 0, 0, 0, 0, 0);    // красный шрифт для макс.
      myGLCD.printNumF( MaxTempH2, 1, 275, 172); } }  // отображение max температуры радиатора 2
     
void SoundAlarm(){
    myGLCD.setColor(255, 240, 255);
    myGLCD.fillRect(50, 70, 269, 218);   
    myGLCD.setColor(0, 0, 255);  
    myGLCD.drawRoundRect(50+2, 70+2, 269-2, 218-2); 
    myGLCD.setColor(255, 0, 0); // red
    myGLCD.drawRoundRect(214, 116, 258, 152);
    myGLCD.drawRoundRect(62, 116, 106, 152);
    
    myGLCD.setFont(RusFont1);    
    myGLCD.setColor(0, 0, 0); 
    myGLCD.setBackColor(255, 240, 255);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[167]))); 
    myGLCD.print(buffer, CENTER, 85);         // УСТАНОВКА ЗВУКОВОЙ ТРЕВОГИ при достижении заданной температуры
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRoundRect(117, 104, 203,165);
    myGLCD.setColor(0, 0, 255);
    myGLCD.drawRoundRect(115, 102, 205,167);
    
   printButton(print_text[28], SoundATm[0], SoundATm[1], SoundATm[2], SoundATm[3], LARGE); // звуковая тревога +
   printButton(print_text[27], SoundATp[0], SoundATp[1], SoundATp[2], SoundATp[3], LARGE); // звуковая тревога -
   
        myGLCD.setFont(SmallFont);
        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRoundRect(95, 185, 135, 205);
        setFont(SMALL, 0, 0, 0, 0, 255, 0);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[17]))); 
      myGLCD.print(buffer, 107, 190);     // OK
      myGLCD.setColor(0, 0, 255);
      myGLCD.fillRoundRect(175, 185, 240, 205);
      setFont(SMALL, 255, 255, 255, 0, 0, 255); 
      myGLCD.setFont(RusFont1); 
      myGLCD.print(print_text[1], 175+9, 185+5); // CANCEL 
      
     myGLCD.setColor(0, 0, 0);
     myGLCD.drawRoundRect(95, 185, 135, 205);  
     myGLCD.drawRoundRect(175, 185, 240, 205);  
               setSalarm(); }
     
 void setSalarm(){
          setFont(LARGE, 255, 0, 0, 0, 0, 0);
    if (setTempToSoundAlarmC ==255){   
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[26]))); 
          myGLCD.print(buffer, 122, 125);           //  ( OFF alarm )
        } else { setFont(LARGE, 255, 255, 255, 0, 0, 0);
          myGLCD.printNumF(setTempToSoundAlarmC, 1, 129, 125); } } // display sound alarm temp 
          
/******** УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ ******************************************************** dispScreen = 17 */
void DimLEDsAtTempScreen(){  

  PrintStringIndex=15; printHeader ();  // УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ
      
  myGLCD.setFont(RusFont6);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 0, 255, 0, 0, 0, 0);
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[145]))); 
   myGLCD.print(buffer, 6, 52);       // ПРИ НАГРЕВЕ РАДИАТОРА
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[146]))); 
   myGLCD.print(buffer, 35, 67);      //    ДО ТЕМПЕРАТУРЫ:
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[147]))); 
   myGLCD.print(buffer, 21, 131);     // УМЕНЬШАТЬ ЯРКОСТЬ  
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[114]))); 
   myGLCD.print(buffer, 33, 145);     //  CВЕТОДИОДОВ ДО:
   
  setFont(SMALL, 0, 255, 0, 0, 0, 0);  
  myGLCD.drawCircle(273, 61, 1);              
  myGLCD.print(print_text[57], 278, 59);
  myGLCD.print(print_text[142], 273, 140); // % (проценты)
  myGLCD.setColor(64, 64, 64);  
  myGLCD.drawRoundRect(176, 31, 265, 96);   // рамка вокруг кнопок регулировки 
  myGLCD.drawRoundRect(176, 112, 265, 177);
    drawUpButton(235, 36);  
    drawDownButton(235, 66);
    drawUpButton(235, 117);  
    drawDownButton(235, 147);

        TempLEDsDimTemp=setLEDsDimTempC;
        TempLEDsDimPercent=setLEDsDimPercent; 
       
      setFont(LARGE, 255, 108, 72, 0, 0, 0);
  if (TempLEDsDimTemp>=100){ myGLCD.printNumI(TempLEDsDimTemp, 181, 55);}
  if ((TempLEDsDimTemp<=99)&&(TempLEDsDimTemp>=10)){
             myGLCD.printNumI(TempLEDsDimTemp, 189, 55);}     
  if (TempLEDsDimTemp<=9){ myGLCD.printNumI(TempLEDsDimTemp, 198, 55);} 
  
        setFont(LARGE, 255, 255, 255, 0, 0, 0);
  if (TempLEDsDimPercent>=100){ myGLCD.printNumI(TempLEDsDimPercent, 181, 136);}
  if ((TempLEDsDimPercent<=99)&&(TempLEDsDimPercent>=10)){
                myGLCD.printNumI(TempLEDsDimPercent, 189, 136);}
  if (TempLEDsDimPercent<=9){ myGLCD.printNumI(TempLEDsDimPercent, 198, 136);}              
  
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);  
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);         // Back
  printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); }       // Cancel

/***** УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА *************************************************** dispScreen = 18 */
void ScreensaverSettingsScreen(){  

  PrintStringIndex=16; printHeader (); // УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА 

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawLine(0, 45, 319, 45);   
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 255, 0); 
  myGLCD.setBackColor(0, 0, 0);
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[148]))); 
   myGLCD.print(buffer, 18, 26);      // ЧАСЫ / ПУСТОЙ ЭКРАН
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[149]))); 
   myGLCD.print(buffer, 8, 114);      // активация 
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[150]))); 
   myGLCD.print(buffer, 23, 125);     // после
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[132]))); 
   myGLCD.print(buffer, 174, 119);    // минут  
  myGLCD.setColor(64, 64, 64);  
  myGLCD.drawRoundRect(84, 84, 167, 165); // рамка вокруг установок времени скринсейва

  drawUpButton(135, 92);     // кнопка вверх установки времени хранителя экрана 
  drawDownButton(135, 132);  // кнопка вниз установки времени хранителя экрана 
 
  myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта 
  myGLCD.setColor(80, 255, 246);   // цвет голубой
  myGLCD.setBackColor(0, 0, 0);   // цвет фона черный
      TempSSminutes=setSSmintues;
  if (TempSSminutes>=10) { myGLCD.printNumI(TempSSminutes, 87, 112);} 
                    else { myGLCD.printNumI(TempSSminutes, 102, 112);} 
  
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194); // горизонтальные линии над кнопкой 
  myGLCD.drawRect(0, 176, 319, 174); // горизонтальные линии под A.clock
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); 
     ScreensaverSelect(); }
  
/************** СУТОЧНЫЕ ТАЙМЕРЫ ************************************************************** dispScreen = 19 */
void TimerScreen(){        

      PrintStringIndex=17; printHeader ();   // СУТОЧНЫЕ ТАЙМЕРЫ
               
        myGLCD.setColor(0, 0, 255);     // цвет синий 
   for (int x=0; x<5; x++){myGLCD.fillRoundRect((x*64)+7,22,(x*64)+56,45);} // имена таймеров         
        myGLCD.setColor(255, 255, 255); // цвет белый
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+7,22,(x*64)+56,45);} // белые рамки вокруг таймеров
            
// кнопки ON      
        myGLCD.setColor(70, 200, 0);     // цвет зеленый
   for (int x=0; x<5; x++){myGLCD.fillRoundRect((x*64)+7,51,(x*64)+56,79);} // нарисовать кнопки ON          
        myGLCD.setColor(255, 255, 255);  // цвет белый
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+7,51,(x*64)+56,79);} // белые рамки вокруг ON   

// размер рамки времени OFF       
        myGLCD.setColor(255, 255, 255);  // цвет белый
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+7,81,(x*64)+56,113);} // рамки вокруг времени ON
       
// кнопки OFF         
        myGLCD.setColor(255, 0, 0);
   for (int x=0; x<5; x++){myGLCD.fillRoundRect((x*64)+7,122,(x*64)+56,150);} // нарисовать кнопки OFF    
        myGLCD.setColor(255, 255, 255);   // цвет белый
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+7,122,(x*64)+56,150);} // белые рамки вокруг OFF        
        myGLCD.setColor(255, 255, 255);   // цвет белый
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+7,152,(x*64)+56,184);} // размер рамки таймера OFF
       
// рамка вокруг кнопок таймеров 
         myGLCD.setColor(130, 130, 130);  // цвет серый       
   for (int x=0; x<5; x++){myGLCD.drawRoundRect((x*64)+5,20,(x*64)+58,186);}
         
// показать время включения таймера 1 в окне ON 
     setFont(SMALL, 32, 255, 255, 0, 0 , 0);  // цвет шрифта бирюзовый, фон черный 
 if (on1/60<10){ myGLCD.print(print_text[187], 12, 91);             //  0  добавить 0 к часам
      myGLCD.printNumI(on1/60,21, 91);}else{myGLCD.printNumI(on1/60,12, 91);} // положение часов на экране 
      myGLCD.print(print_text[56], 29, 90);     // :
 if (on1-((on1/60)*60)<10){ myGLCD.print(print_text[187], 37, 91);   //  0  добавить 0 к минутам
      myGLCD.printNumI(on1-((on1/60)*60),46, 91);} else { 
      myGLCD.printNumI(on1-((on1/60)*60),37, 91);} 
      
// показать время включения таймера 2 в окне ON
 if (on2/60<10){ myGLCD.print(print_text[187], 76, 91);            //  0  добавить 0 к часам
      myGLCD.printNumI(on2/60,85, 91);}else{myGLCD.printNumI(on2/60,76, 91);} // положение часов на экране 
      myGLCD.print(print_text[56], 93, 90);     // :
 if (on2-((on2/60)*60)<10){ myGLCD.print(print_text[187], 101, 91); //  0  добавить 0 к минутам
      myGLCD.printNumI(on2-((on2/60)*60),110, 91);} else { 
      myGLCD.printNumI(on2-((on2/60)*60),101, 91);} 
      
// показать время включения таймера 3 в окне ON
  if (on3/60<10){ myGLCD.print(print_text[187], 140, 91);            //  0  добавить 0 к часам
      myGLCD.printNumI(on3/60,149, 91);}else{myGLCD.printNumI(on3/60,140, 91);} // положение часов на экране 
      myGLCD.print(print_text[56], 157, 90);     // :
  if (on3-((on3/60)*60)<10){ myGLCD.print(print_text[187], 165, 91);   //  0  добавить 0 к минутам
      myGLCD.printNumI(on3-((on3/60)*60),174, 91);} else { 
      myGLCD.printNumI(on3-((on3/60)*60),165, 91);} 
      
// показать время включения таймера 4 в окне ON
  if (on4/60<10){ myGLCD.print(print_text[187], 204, 91);            //  0 
     myGLCD.printNumI(on4/60,213, 91);}else{myGLCD.printNumI(on4/60,204, 91);} // положение часов на экране 
      myGLCD.print(print_text[56], 221, 90);     // :
  if (on4-((on4/60)*60)<10){ myGLCD.print(print_text[187], 229, 91);  //  0 
      myGLCD.printNumI(on4-((on4/60)*60),238, 91);} else { 
      myGLCD.printNumI(on4-((on4/60)*60),229, 91);}
      
// показать время включения таймера 5 в окне ON
  if (on5/60<10){ myGLCD.print(print_text[187], 268, 91);           //  0
     myGLCD.printNumI(on5/60,277, 91);}else{myGLCD.printNumI(on5/60,268, 91);} // положение часов на экране 
      myGLCD.print(print_text[56], 285, 90);     // :
  if (on5-((on5/60)*60)<10){ myGLCD.print(print_text[187], 293, 91); //  0
      myGLCD.printNumI(on5-((on5/60)*60),302, 91);} else { 
      myGLCD.printNumI(on5-((on5/60)*60),293, 91);} 

// показать время выключения таймера 1 в окне OFF 
  if (off1/60<10){ myGLCD.print(print_text[187], 12, 162);            //  0
      myGLCD.printNumI(off1/60,21, 162);} else { myGLCD.printNumI(off1/60,12, 162);}
       myGLCD.print(print_text[56], 29, 161);     // :
  if (off1-((off1/60)*60)<10){ myGLCD.print(print_text[187], 37, 162); //  0
       myGLCD.printNumI(off1-((off1/60)*60),46, 162);} else {
       myGLCD.printNumI(off1-((off1/60)*60),37, 162);}
       
// показать время выключения таймера 2 в окне OFF
  if (off2/60<10){ myGLCD.print(print_text[187], 76, 162);              //  0 
       myGLCD.printNumI(off2/60,85, 162);} else { myGLCD.printNumI(off2/60,76, 162);}
       myGLCD.print(print_text[56], 93, 161);      // :
  if (off2-((off2/60)*60)<10){ myGLCD.print(print_text[187], 101, 162);  //  0 
       myGLCD.printNumI(off2-((off2/60)*60),110, 162);} else {
       myGLCD.printNumI(off2-((off2/60)*60),101, 162);}
       
// показать время выключения таймера 3 в окне OFF 
  if (off3/60<10){ myGLCD.print(print_text[187], 140, 162);             //  0 
       myGLCD.printNumI(off3/60,149, 162);} else { myGLCD.printNumI(off3/60,140, 162);}
        myGLCD.print(print_text[56], 157, 161);     // :
  if (off3-((off3/60)*60)<10){ myGLCD.print(print_text[187], 165, 162);  //  0 
       myGLCD.printNumI(off3-((off3/60)*60),174, 162);} else {
       myGLCD.printNumI(off3-((off3/60)*60),165, 162);} 
       
// показать время выключения таймера 4 в окне OFF 
  if (off4/60<10){ myGLCD.print(print_text[187], 204, 162);              //  0 
       myGLCD.printNumI(off4/60,213, 162);} else { myGLCD.printNumI(off4/60,204, 162);}
       myGLCD.print(print_text[56], 221, 161);     // :
  if (off4-((off4/60)*60)<10){ myGLCD.print(print_text[187], 229, 162);  //  0
       myGLCD.printNumI(off4-((off4/60)*60),238, 162);} else {
       myGLCD.printNumI(off4-((off4/60)*60),229, 162);}
       
// показать время выключения таймера 5 в окне OFF
  if (off5/60<10){ myGLCD.print(print_text[187], 268, 162);              //  0
       myGLCD.printNumI(off5/60,277, 162);} else { myGLCD.printNumI(off5/60,268, 162);}
       myGLCD.print(print_text[56], 285, 161);     // :
  if (off5-((off5/60)*60)<10){ myGLCD.print(print_text[187], 293, 162);  //  0 
       myGLCD.printNumI(off5-((off5/60)*60),302, 162);} else {
       myGLCD.printNumI(off5-((off5/60)*60),293, 162);}
      
     setFont(SMALL, 255, 255, 255, 0, 0 , 255); // цвет шрифта белый, цвет вокруг текста названия таймера синий 
     myGLCD.setFont(RusFont1);
 #ifdef freshwater
      myGLCD.print(print_text[62],16,30);    // 1
      myGLCD.print(print_text[63],82,30);    // 2
      myGLCD.print(print_text[64],136,30);   // 3
      myGLCD.print(print_text[65],213,30);   // 4
      myGLCD.print(print_text[66],271 ,30);  // 5
  #endif  
  #ifdef seawater 
        myGLCD.setFont(BigFont);         // выбрать шрифт  
      myGLCD.print(print_text[62],8,26);    // 1
      myGLCD.print(print_text[63],72,26);    // 2
      myGLCD.print(print_text[64],136,26);   // 3
      myGLCD.print(print_text[65],200,26);   // 4
      myGLCD.print(print_text[66],264,26);   // 5
  #endif 
        myGLCD.setFont(BigFont);         // выбрать шрифт
        myGLCD.setBackColor(70, 200, 0); // цвет вокруг текста ON зеленый      
      myGLCD.print(print_text[212], 15, 57);   // ON
      myGLCD.print(print_text[212], 79, 57);   // ON
      myGLCD.print(print_text[212], 143, 57);  // ON
      myGLCD.print(print_text[212], 207, 57);  // ON 
      myGLCD.print(print_text[212], 271, 57);  // ON
      
        myGLCD.setBackColor(255, 0, 0);   // цвет вокруг текста OFF красный
      myGLCD.print(print_text[213], 8, 129);    // OFF
      myGLCD.print(print_text[213], 72, 129);   // OFF
      myGLCD.print(print_text[213], 136, 129);  // OFF
      myGLCD.print(print_text[213], 200, 129);  // OFF 
      myGLCD.print(print_text[213], 264, 129);  // OFF
            
    myGLCD.setColor(64, 64, 64);      // цвет серый
    myGLCD.drawRect(0, 195, 319, 193);
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL); }
          
//********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 1********************************************* Timer 1 dispScreen = 20 */
 void light1set(){
      
          printFramework(); // кайма вокруг часов включения
          printPicture();   // картинки стрелок
          printButGreen(print_text[67]); // время вкл.
          printButRed(print_text[68]);  // время выкл.         
          PrintStringIndex=18; printHeader (); // УСТАНОВКИ ТАЙМЕРА 1
          printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
          printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
          printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
                                               
          printTimernumber(print_text[47]); // номер таймера  (1)
            myGLCD.setColor(0, 255, 255);   // цвет бирюзовый      
                   timer1Change(); }  

 void timer1Change(){ printFont();
      
       if (on1/60<10){               // время включения часы
            myGLCD.print(print_text[187], 27, 100);     //  0 
            myGLCD.printNumI(on1/60,44, 100);} else {
            myGLCD.printNumI(on1/60,27, 100);}
      
       if (on1-((on1/60)*60)<10){    // время включения минуты
            myGLCD.print(print_text[187], 95, 100);     //  0
            myGLCD.printNumI(on1-((on1/60)*60),112, 100);} else {
            myGLCD.printNumI(on1-((on1/60)*60),95, 100);}

       if (off1/60<10){             // время выключения часы
            myGLCD.print(print_text[187], 193, 100);    //  0 
            myGLCD.printNumI(off1/60,210, 100);} else {
            myGLCD.printNumI(off1/60,193, 100);}
      
       if (off1-((off1/60)*60)<10){  // время выключения минуты
            myGLCD.print(print_text[187], 262, 100);    //  0 
            myGLCD.printNumI(off1-((off1/60)*60),279, 100);} else {
            myGLCD.printNumI(off1-((off1/60)*60),262, 100);} }    
            
void light2set(){ /************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 2***************************************** Timer 2 dispScreen = 21 */

          printFramework(); // кайма вокруг часов включения
          printPicture();   // картинки стрелок
          printButGreen(print_text[67]); // время вкл.
          printButRed(print_text[68]);   // время выкл.
          PrintStringIndex=19; printHeader (); // УСТАНОВКИ ТАЙМЕРА 2
          printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
          printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
          printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
                                                            
          printTimernumber(print_text[48]);  // номер таймера (2)
            myGLCD.setColor(0, 255, 255);    // цвет бирюзовый
                  timer2Change(); }
                
 void timer2Change(){ printFont();
            
       if (on2/60<10){    // время включения
            myGLCD.print(print_text[187], 27, 100);     //  0
            myGLCD.printNumI(on2/60,44, 100);} else {
            myGLCD.printNumI(on2/60,27, 100);}
      
       if (on2-((on2/60)*60)<10){   
            myGLCD.print(print_text[187], 95, 100);     //  0
            myGLCD.printNumI(on2-((on2/60)*60),112, 100);} else {
            myGLCD.printNumI(on2-((on2/60)*60),95, 100);}

       if (off2/60<10){   // время выключения
            myGLCD.print(print_text[187], 193, 100);     //  0
            myGLCD.printNumI(off2/60,210, 100);} else {
            myGLCD.printNumI(off2/60,193, 100);}
    
       if (off2-((off2/60)*60)<10){;
            myGLCD.print(print_text[187], 262, 100);     //  0 
            myGLCD.printNumI(off2-((off2/60)*60),279, 100);} else {
            myGLCD.printNumI(off2-((off2/60)*60),262, 100);} }
                                   
void light3set(){ /********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 3***************************** Timer 3 dispScreen = 22 */

          printFramework(); // кайма вокруг часов включения
          printPicture();  // картинки стрелок
          printButGreen(print_text[67]); // время вкл.
          printButRed(print_text[68]);   // время выкл.
          PrintStringIndex=20; printHeader (); // УСТАНОВКИ ТАЙМЕРА 3
          printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
          printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
          printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
                                                   
          printTimernumber(print_text[49]); // номер таймера (3)
            myGLCD.setColor(0, 255, 255);   // цвет бирюзовый
                   timer3Change(); }
            
 void timer3Change(){ printFont(); 
             
      if (on3/60<10){            // время включения
            myGLCD.print(print_text[187], 27, 100);     //  0 
            myGLCD.printNumI(on3/60,44, 100);} else {
            myGLCD.printNumI(on3/60,27, 100);}
      
      if (on3-((on3/60)*60)<10){  
            myGLCD.print(print_text[187], 95, 100);     //  0 
            myGLCD.printNumI(on3-((on3/60)*60),112, 100);} else {
            myGLCD.printNumI(on3-((on3/60)*60),95, 100);}

      if (off3/60<10){     // время выключения
            myGLCD.print(print_text[187], 193, 100);     //  0
            myGLCD.printNumI(off3/60,210, 100);} else {
            myGLCD.printNumI(off3/60,193, 100);}
      
      if (off3-((off3/60)*60)<10){
            myGLCD.print(print_text[187], 262, 100);     //  0
            myGLCD.printNumI(off3-((off3/60)*60),279, 100);} else {
            myGLCD.printNumI(off3-((off3/60)*60),262, 100);} }
                                           
void light4set(){ /*********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 4******************************* Timer 4 dispScreen = 23 */

          printFramework(); // кайма вокруг часов включения
          printPicture();  // картинки стрелок
          printButGreen(print_text[67]); // время вкл.
          printButRed(print_text[68]);   // время выкл.
          PrintStringIndex=21; printHeader (); // УСТАНОВКИ ТАЙМЕРА 4
          printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
          printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
          printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
                                                  
          printTimernumber(print_text[50]);  // номер таймера  (4)
            myGLCD.setColor(0, 255, 255);    // цвет бирюзовый
                    timer4Change(); }
                    
 void timer4Change(){ printFont();
   
      if (on4/60<10){         // время включения
            myGLCD.print(print_text[187], 27, 100);     //  0
            myGLCD.printNumI(on4/60,44, 100);} else {
            myGLCD.printNumI(on4/60,27, 100);}
       
      if (on4-((on4/60)*60)<10){     
            myGLCD.print(print_text[187], 95, 100);     //  0 
            myGLCD.printNumI(on4-((on4/60)*60),112, 100);} else {
            myGLCD.printNumI(on4-((on4/60)*60),95, 100);}

      if (off4/60<10){     // время выключения
            myGLCD.print(print_text[187], 193, 100);     //  0 
            myGLCD.printNumI(off4/60,210, 100);} else {
            myGLCD.printNumI(off4/60,193, 100);}
      
      if (off4-((off4/60)*60)<10){
            myGLCD.print(print_text[187], 262, 100);     //  0 
            myGLCD.printNumI(off4-((off4/60)*60),279, 100);} else {
            myGLCD.printNumI(off4-((off4/60)*60),262, 100);} }
                       
void light5set(){ /*******************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 5*********************************** Timer 5  dispScreen = 24 */

          printFramework(); // кайма вокруг часов включения
          printPicture();  // картинки стрелок
          printButGreen(print_text[67]); // время вкл.
          printButRed(print_text[68]);   // время выкл.
          PrintStringIndex=22; printHeader (); // УСТАНОВКИ ТАЙМЕРА 5
          printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
          printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
          printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
                                          
          printTimernumber(print_text[51]); // номер таймера  (5)
            myGLCD.setColor(0, 255, 255);    // цвет бирюзовый
                  timer5Change(); }
                  
void timer5Change(){ printFont();
   
     if (on5/60<10){ // время включения часы 
            myGLCD.print(print_text[187], 27, 100);     //  0
            myGLCD.printNumI(on5/60,44, 100);} else {
            myGLCD.printNumI(on5/60,27, 100);}
   
     if (on5-((on5/60)*60)<10){       
            myGLCD.print(print_text[187], 95, 100);     //  0 
            myGLCD.printNumI(on5-((on5/60)*60),112, 100);} else {
            myGLCD.printNumI(on5-((on5/60)*60),95, 100);}

     if (off5/60<10){            // времявключения
            myGLCD.print(print_text[187], 193, 100);     //  0 
            myGLCD.printNumI(off5/60,210, 100);} else {
            myGLCD.printNumI(off5/60,193, 100);}
     
     if (off5-((off5/60)*60)<10){
            myGLCD.print(print_text[187], 262, 100);     //  0
            myGLCD.printNumI(off5-((off5/60)*60),279, 100);} else {
            myGLCD.printNumI(off5-((off5/60)*60),262, 100);} }

void light(){ // ------------- Срабатывание таймеров
# ifdef freshwater
// 1 - канал таймера АЭРАЦИИ
     if (timer1Status<2 && prog1==0){    
          if (on1<off1){if (timer>=on1 && timer<off1){timer1Status=1;} else {timer1Status=0;}}
          if (on1>off1){if (timer<on1 && timer>=off1){timer1Status=0;} else {timer1Status=1;}}
        }
// 2 - канал таймера подачи СО2            
     if (timer2Status<2 && prog2==0){ 
          if (on2<off2){
            #ifdef PH_sensor_I2C  // Если подключен датчик PH учитываются его показания
          if ((timer>=on2 && timer<off2) && (avgMeasuredPH > SetvalPH)){timer2Status=1;} else {timer2Status=0;}}         
            #else                // иначе просто таймер
          if (timer>=on2 && timer<off2){timer2Status=1;} else {timer2Status=0;}}
            #endif
          if (on2>off2){if (timer<on2 && timer>=off2){timer2Status=0;} else {timer2Status=1;}}
        }
// 3 - канал таймера ФИЛЬТРА            
     if (timer3Status<2 && prog3==0){ 
          if (on3<off3){if (timer>=on3 && timer<off3){timer3Status=1;} else {timer3Status=0;}}
          if (on3>off3){if (timer<on3 && timer>=off3){timer3Status=0;} else {timer3Status=1;}}
        }
// 4 - канал таймера УФ ЛАМПЫ             
     if (timer4Status<2 && prog4==0){ 
          if (on4<off4){if (timer>=on4 && timer<off4 && timer3Status==1){timer4Status=1;} else {timer4Status=0;}}
          if (on4>off4){if (timer<on4 && timer>=off4){timer4Status=0;} else {timer4Status=1;}}
        }
// 5 - канал таймера ДОЛИВА             
     if (timer5Status<2 && prog5==0){ 
          if (on5<off5){if (timer>=on5 && timer<off5){timer5Status=1;} else {timer5Status=0;}}
          if (on5>off5){if (timer<on5 && timer>=off5){timer5Status=0;} else {timer5Status=1;}}
        }
#endif         
          
# ifdef seawater          
// 1 - канал таймера 
     if (timer1Status<2 && prog1==0){    
          if (on1<off1){if (timer>=on1 && timer<off1){timer1Status=1;} else {timer1Status=0;}}
          if (on1>off1){if (timer<on1 && timer>=off1){timer1Status=0;} else {timer1Status=1;}}}
// 2 - канал таймера              
     if (timer2Status<2 && prog2==0){ 
          if (on2<off2){if (timer>=on2 && timer<off2){timer2Status=1;} else {timer2Status=0;}}
          if (on2>off2){if (timer<on2 && timer>=off2){timer2Status=0;} else {timer2Status=1;}}}
// 3 - канал таймера             
     if (timer3Status<2 && prog3==0){ 
          if (on3<off3){if (timer>=on3 && timer<off3){timer3Status=1;} else {timer3Status=0;}}
          if (on3>off3){if (timer<on3 && timer>=off3){timer3Status=0;} else {timer3Status=1;}}}
// 4 - канал таймера              
     if (timer4Status<2 && prog4==0){ 
          if (on4<off4){if (timer>=on4 && timer<off4){timer4Status=1;} else {timer4Status=0;}}
          if (on4>off4){if (timer<on4 && timer>=off4){timer4Status=0;} else {timer4Status=1;}}}
// 5 - канал таймера             
     if (timer5Status<2 && prog5==0){ 
          if (on5<off5){if (timer>=on5 && timer<off5){timer5Status=1;} else {timer5Status=0;}}
          if (on5>off5){if (timer<on5 && timer>=off5){timer5Status=0;} else {timer5Status=1;}}}
#endif         
      
             
// управление портами  0-auto off, 1-auto on, 2-on, 3-off.           
// Timer 1      
          if (timer1Status==0){digitalWrite(timer1,LOW);}  // Auto OFF
          if (timer1Status==1){digitalWrite(timer1,HIGH);} // Auto ON  
          if (timer1Status==2){digitalWrite(timer1,HIGH);} // ON
          if (timer1Status==3){digitalWrite(timer1,LOW);}  // OFF
// Timer 2
          if (timer2Status==0){digitalWrite(timer2,LOW);}  // Auto OFF
          if (timer2Status==1){digitalWrite(timer2,HIGH);} // Auto ON
          if (timer2Status==2){digitalWrite(timer2,HIGH);} // ON
          if (timer2Status==3){digitalWrite(timer2,LOW);}  // OFF
// Timer 3 фильтр
            if (timer3Status==0){digitalWrite(timer3,LOW);}  // Auto OFF
            if ((timer3Status==1) && (FeedWaveCtrl_1==false) && 
        (FeedWaveCtrl_2==false) && (FeedWaveCtrl_3==false) && 
        (FeedWaveCtrl_4==false)) {digitalWrite(timer3,HIGH);} // Auto ON
            if (timer3Status==2){digitalWrite(timer3,HIGH);} // ON
            if (timer3Status==3){digitalWrite(timer3,LOW);}  // OFF
           
//  таймер 3 для сервопривода          
        //  if (timer3Status==0){digitalWrite(timer3,LOW); servo_1 = false; Servo_out1();}                   // Auto OFF
        //  if (timer3Status==1){digitalWrite(timer3,HIGH); ReadFromEEPROM(); servo_1 = true; Servo_out1();} // Auto ON
        //  if (timer3Status==2){digitalWrite(timer3,HIGH); ReadFromEEPROM();servo_1 = true; Servo_out1();}  // ON
        //  if (timer3Status==3){digitalWrite(timer3,LOW); servo_1 = false; Servo_out1();}                   // OFF           
// Timer 4 
          if (timer4Status==0){digitalWrite(timer4,LOW);}  // Auto OFF
          if (timer4Status==1){digitalWrite(timer4,HIGH);} // Auto ON
          if (timer4Status==2){digitalWrite(timer4,HIGH);} // ON
          if (timer4Status==3){digitalWrite(timer4,LOW);}  // OFF
// Timer 5 
          if (timer5Status==0){digitalWrite(timer5,LOW);}  // Auto OFF
          if (timer5Status==1){digitalWrite(timer5,HIGH);} // Auto ON
          if (timer5Status==2){digitalWrite(timer5,HIGH);} // ON
          if (timer5Status==3){digitalWrite(timer5,LOW);}} // OFF

     //       if (timer5Status==0){digitalWrite(timer5,LOW); timer_dayStatus=0;}  // Auto OFF
     //       if (timer5Status==1){digitalWrite(timer5,HIGH); timer_dayStatus=1; SSTouch=1; } // Auto ON
     //       if (timer5Status==2){digitalWrite(timer5,HIGH); timer_dayStatus=1; SSTouch=1; } // ON
     //       if (timer5Status==3){digitalWrite(timer5,LOW); timer_dayStatus=0;}} // OFF

void graphonoff(){  /*********** Экран ручного управления таймерами ****************************** dispScreen = 25 */

    printButtonRUS(print_text[0], back[0], back[1], back[2], back[3], SMALL); // << MENU
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3]);        // CANCEL

        PrintStringIndex=23; printHeader (); // РУЧНОЕ УПРАВЛЕНИЕ ТАЙМЕРАМИ
    
           myGLCD.setColor(64, 64, 64); // цвет серый
 //--------------------------------------------------------------------------------------------------------------   
            myGLCD.drawRect(11,23,89,43);   // 1 кайма вокруг таймеров
            myGLCD.drawRect(11,59,89,79);   // 2
            myGLCD.drawRect(11,95,89,115);  // 3
            myGLCD.drawRect(11,131,89,151); // 4
            myGLCD.drawRect(11,167,89,187); // 5
            
       myGLCD.drawLine(1,51,100,51);   // 1  линии между кнопками     
       myGLCD.drawLine(1,87,100,87);   // 2      
       myGLCD.drawLine(1,123,100,123); // 3   
       myGLCD.drawLine(1,159,100,159); // 4
 //--------------------------------------------------------------------------------------------------------------------           
         myGLCD.drawRect(100,15,173,51);  // auto
         myGLCD.drawRect(100,51,173,87);
         myGLCD.drawRect(100,87,173,123);
         myGLCD.drawRect(100,123,173,159);
         myGLCD.drawRect(100,159,173,195);

         myGLCD.drawRect(173,15,246,51);  // on
         myGLCD.drawRect(173,51,246,87);
         myGLCD.drawRect(173,87,246,123);
         myGLCD.drawRect(173,123,246,159);
         myGLCD.drawRect(173,159,246,195);

         myGLCD.drawRect(246,15,319,51);  // off
         myGLCD.drawRect(246,51,319,87);
         myGLCD.drawRect(246,87,319,123);
         myGLCD.drawRect(246,123,319,159);
         myGLCD.drawRect(246,159,319,195);
//--------------------------------------------------------------------------------------------------------------------------
         myGLCD.setFont(RusFont3);
        // myGLCD.setColor(152, 229, 255); // бирюзовый
         myGLCD.setColor(0, 0, 255);
         myGLCD.setBackColor(0, 0, 0);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[151]))); 
    myGLCD.print(buffer, 20, 29);     // Таймер 1
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[152]))); 
    myGLCD.print(buffer, 20, 65);     // Таймер 2
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[153]))); 
    myGLCD.print(buffer, 20, 102);    // Таймер 3
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[154]))); 
    myGLCD.print(buffer, 20, 137);    // Таймер 4
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[155]))); 
    myGLCD.print(buffer, 20, 173);    // Таймер 5

       onoff1(); onoff2(); onoff3(); onoff4(); onoff5(); }
                      
void onoff1(){ // графика для Таймера 1
       myGLCD.setFont(BigFont);      // font size LARGE
       myGLCD.setBackColor(0, 0, 0); 
  if (timer1Status==0){ myGLCD.setColor(147, 115, 255); // AUTO OFF
                 OnOffTimer1(); }
     
  if (timer1Status==1){ myGLCD.setColor(0, 184, 9); // AUTO ON
                 OnOffTimer1(); }
     
  if (timer1Status==2){ myGLCD.setColor(0, 255, 0); // ON 
     myGLCD.print(print_text[212], 192, 25);        // ON
       myGLCD.drawRect(175,17,244,49); myGLCD.drawRect(177,19,242,47);
       myGLCD.setColor(80, 80, 80); 
      myGLCD.print(print_text[213], 259, 25);      // OFF 
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
    myGLCD.print(buffer, 105, 25);        // AUTO  
       myGLCD.setColor(190, 190, 190);    // серый
       myGLCD.drawRect(248,17,317,49);    // OFF
       myGLCD.drawRect(250,19,315,47);    // off
       myGLCD.drawRect(102,17,171,49);}   // AUTO 
     
  if (timer1Status==3){ myGLCD.setColor(255, 0, 0); // OFF
     myGLCD.print(print_text[213], 259, 25);        // OFF
       myGLCD.drawRect(248,17,317,49); myGLCD.drawRect(250,19,315,47);
       myGLCD.setColor(80, 80, 80);  
     myGLCD.print(print_text[212], 192, 25);        // ON
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 25);        // AUTO
       myGLCD.setColor(190, 190, 190);     // серый
       myGLCD.drawRect(175,17,244,49);     // ON
       myGLCD.drawRect(177,19,242,47);     // on
       myGLCD.drawRect(102,17,171,49);}}   // AUTO 
                 
void onoff2(){ // графика для Таймера 2 
       myGLCD.setFont(BigFont);      // font size LARGE
       myGLCD.setBackColor(0, 0, 0); 
  if (timer2Status==0){ myGLCD.setColor(147, 115, 255); // AUTO OFF
                  OnOffTimer2(); }
     
  if (timer2Status==1){ myGLCD.setColor(0, 184, 9); // AUTO ON
                  OnOffTimer2(); }
     
  if (timer2Status==2){ myGLCD.setColor(0, 255, 0); // ON
     myGLCD.print(print_text[212], 192, 61);        // ON 
       myGLCD.drawRect(175,53,244,85); myGLCD.drawRect(177,55,242,83);
       myGLCD.setColor(80, 80, 80);  
     myGLCD.print(print_text[213], 259, 61);       // OFF 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 61);       // AUTO 
       myGLCD.setColor(190, 190, 190);    // серый
       myGLCD.drawRect(248,53,317,85);    // OFF
       myGLCD.drawRect(250,55,315,83);    // off
       myGLCD.drawRect(102,53,171,85);}   // AUTO 
     
 if (timer2Status==3){ myGLCD.setColor(255, 0, 0);  // OFF
     myGLCD.print(print_text[213], 259, 61);        // OFF 
       myGLCD.drawRect(248,53,317,85); myGLCD.drawRect(250,55,315,83);
       myGLCD.setColor(80, 80, 80);  
     myGLCD.print(print_text[212], 192, 61);        // ON
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 61);        // AUTO
       myGLCD.setColor(190, 190, 190);     // серый
       myGLCD.drawRect(175,53,244,85);     // ON
       myGLCD.drawRect(177,55,242,83);     // on
       myGLCD.drawRect(102,53,171,85);}}   // AUTO 
             
void onoff3(){ // графика для Таймера 3 
       myGLCD.setFont(BigFont);      // font size LARGE
       myGLCD.setBackColor(0, 0, 0); 
  if (timer3Status==0){ myGLCD.setColor(147, 115, 255); // AUTO OFF
                OnOffTimer3(); }
     
  if (timer3Status==1){ myGLCD.setColor(0, 184, 9); // AUTO ON
                OnOffTimer3(); }
     
  if (timer3Status==2){ myGLCD.setColor(0, 255, 0); // ON
     myGLCD.print(print_text[212], 192, 97);        // ON
       myGLCD.drawRect(175,89,244,121); myGLCD.drawRect(177,91,242,119);
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 97);        // OFF
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 97);        // AUTO
       myGLCD.setColor(190, 190, 190);     // серый
       myGLCD.drawRect(248,89,317,121);    // OFF
       myGLCD.drawRect(250,91,315,119);    // off
       myGLCD.drawRect(102,89,171,121);}   // AUTO 
     
  if (timer3Status==3){ myGLCD.setColor(255, 0, 0); // OFF
     myGLCD.print(print_text[213], 259, 97);        // OFF
       myGLCD.drawRect(248,89,317,121); myGLCD.drawRect(250,91,315,119);
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[212], 192, 97);        // ON 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 97);        // AUTO
       myGLCD.setColor(190, 190, 190);     // серый
       myGLCD.drawRect(175,89,244,121);    // ON
       myGLCD.drawRect(177,91,242,119);    // on
       myGLCD.drawRect(102,89,171,121);}}  // AUTO 
                          
void onoff4(){ // графика для Таймера 4 
       myGLCD.setFont(BigFont);      // font size LARGE
       myGLCD.setBackColor(0, 0, 0); 
  if (timer4Status==0){ myGLCD.setColor(147, 115, 255); // AUTO OFF
                 OnOffTimer4(); }
     
  if (timer4Status==1){ myGLCD.setColor(0, 184, 9); // AUTO ON
                 OnOffTimer4(); }
   
  if (timer4Status==2){ myGLCD.setColor(0, 255, 0); // ON
     myGLCD.print(print_text[212], 192, 134);        // ON
       myGLCD.drawRect(175,125,244,157); myGLCD.drawRect(177,127,242,155);
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 133);       // OFF 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 134);        // AUTO
       myGLCD.setColor(190, 190, 190);      // серый
       myGLCD.drawRect(248,125,317,157);    // OFF
       myGLCD.drawRect(250,127,315,155);    // off
       myGLCD.drawRect(102,125,171,157);}   // AUTO 
     
  if (timer4Status==3){ myGLCD.setColor(255, 0, 0); // OFF
     myGLCD.print(print_text[213], 259, 133);       // OFF
       myGLCD.drawRect(248,125,317,157); myGLCD.drawRect(250,127,315,155);
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[212], 192, 134);        // ON
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 134);        // AUTO
       myGLCD.setColor(190, 190, 190);      // серый
       myGLCD.drawRect(175,125,244,157);    // ON
       myGLCD.drawRect(177,127,242,155);    // on
       myGLCD.drawRect(102,125,171,157);}}  // AUTO 
                        
void onoff5(){ // графика для Таймера 5 
       myGLCD.setFont(BigFont);      // font size LARGE
       myGLCD.setBackColor(0, 0, 0); 
  if (timer5Status==0){ myGLCD.setColor(147, 115, 255); // AUTO OFF
                OnOffTimer5(); }
     
  if (timer5Status==1){ myGLCD.setColor(0, 184, 9); // AUTO ON
                OnOffTimer5(); }
     
  if (timer5Status==2){ myGLCD.setColor(0, 255, 0); // ON
     myGLCD.print(print_text[212], 192, 168);       // ON
       myGLCD.drawRect(175,161,244,193); myGLCD.drawRect(177,163,242,191);
       myGLCD.setColor(80, 80, 80); 
     myGLCD.print(print_text[213], 259, 168);       // OFF 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 168);        // AUTO 
       myGLCD.setColor(190, 190, 190);      // серый
       myGLCD.drawRect(248,161,317,193);    // OFF
       myGLCD.drawRect(250,163,315,191);    // off
       myGLCD.drawRect(102,161,171,193);}   // AUTO 
     
  if (timer5Status==3){ myGLCD.setColor(255, 0, 0); // OFF
     myGLCD.print(print_text[213], 259, 168);       // OFF
       myGLCD.drawRect(248,161,317,193); myGLCD.drawRect(250,163,315,191);
       myGLCD.setColor(80, 80, 80);  
     myGLCD.print(print_text[212], 192, 168);       // ON 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156]))); 
     myGLCD.print(buffer, 105, 168);        // AUTO
       myGLCD.setColor(190, 190, 190);      // серый
       myGLCD.drawRect(177,163,242,191);    // on
       myGLCD.drawRect(175,161,244,193);    // ON
       myGLCD.drawRect(102,161,171,193);}}  // AUTO 
     
void lightdraw(){   // отображение соостояний таймеров в главном экране  
if ((dispScreen == 0) && (screenSaverCounter<setScreenSaverTimer)){
    myGLCD.setFont(RusFont1);
    
// Таймер 1 статус в главном экране    
    if (timer1Status==0){myGLCD.setColor(190, 190, 190); // цвет бара в режиме Auto OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 168);}   // Вык 
    if (timer1Status==1){myGLCD.setColor(104, 232, 0);  // цвет бара в режиме Auto ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 168);}   // Вкл
    if (timer1Status==2){myGLCD.setColor(0, 255, 0);    // цвет бара в режиме Мanual ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 168);}   // Вкл
    if (timer1Status==3){myGLCD.setColor(255, 0, 0);    // цвет бара в режиме Мanual OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 168);}  // Вык
  
// Таймер 2 статус             
    if (timer2Status==0){myGLCD.setColor(190, 190, 190); // цвет бара в режиме Auto OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 180);}   // Вык
    if (timer2Status==1){myGLCD.setColor(104, 232, 0);   // цвет бара в режиме Auto ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 180);}   // Вкл
    if (timer2Status==2){myGLCD.setColor(0, 255, 0);    // цвет бара в режиме Мanual ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 180);}   // Вкл
    if (timer2Status==3){myGLCD.setColor(255, 0, 0);    // цвет бара в режиме Мanual OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 180);}   // Вык
  
// Таймер 3 статус             
    if (timer3Status==0){myGLCD.setColor(190, 190, 190); // цвет бара в режиме Auto OFF
             strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
         myGLCD.print(buffer, 84, 191);}    // Вык
    if (timer3Status==1){myGLCD.setColor(104, 232, 0);   // цвет бара в режиме Auto ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 191);}   // Вкл
    if (timer3Status==2){myGLCD.setColor(0, 255, 0);    // цвет бара в режиме Мanual ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 191);}   // Вкл
    if (timer3Status==3){myGLCD.setColor(255, 0, 0);    // цвет бара в режиме Мanual OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 191);}   // Вык
  
// Таймер 4 статус             
    if (timer4Status==0){myGLCD.setColor(190, 190, 190); // цвет бара в режиме Auto OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 202);}   // Вык
    if (timer4Status==1){myGLCD.setColor(104, 232, 0);  // цвет бара в режиме Auto ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 202);}   // Вкл
    if (timer4Status==2){myGLCD.setColor(0, 255, 0);    // цвет бара в режиме Мanual ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 202);}   // Вкл
    if (timer4Status==3){myGLCD.setColor(255, 0, 0);   // цвет бара в режиме Мanual OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 202);}   // Вык
  
// Таймер 5 статус              
    if (timer5Status==0){myGLCD.setColor(190, 190, 190); // цвет бара в режиме Auto OFF 
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 213);}   // Вык
    if (timer5Status==1){myGLCD.setColor(104, 232, 0);   // цвет бара в режиме Auto ON  
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 213);}   // Вкл
    if (timer5Status==2){myGLCD.setColor(0, 255, 0);    // цвет бара в режиме Мanual ON
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[172]))); 
          myGLCD.print(buffer, 84, 213);}   // Вкл
    if (timer5Status==3){myGLCD.setColor(255, 0, 0);    // цвет бара в режиме Мanual OFF
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[171]))); 
          myGLCD.print(buffer, 84, 213);}}  // Вык 

}
void tempgScreen(){ // ***************** график температуры ****************************************** dispScreen = 26
// большие 3
     // printButton(print_text[2], back[0], back[1], back[2], back[3], SMALL);         // НАЗАД
    //  printButton(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
     // printButton(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);             // ОТМЕНА
// маленькие
  printButtonRUS(print_text[2], backGS[0], backGS[1], backGS[2], backGS[3], SMALL);
//  printButtonRUS(print_text[18], nextGS[0], nextGS[1], nextGS[2], nextGS[3], SMALL);   
//  printButtonRUS(print_text[19], prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3], SMALL); 
  printButtonRUS(print_text[1], canCgs[0], canCgs[1], canCgs[2], canCgs[3], SMALL);
  
  int x, y, z; 
  
 // myGLCD.setFont(SmallFont);
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 255, 255, 255, 0, 0, 0);  
  myGLCD.print(print_text[69], 6, 171); // 22
  myGLCD.print(print_text[70], 6, 156); // 23
  myGLCD.print(print_text[71], 6, 140); // 24
  myGLCD.print(print_text[72], 6, 124); // 25
  myGLCD.print(print_text[73], 6, 108); // 26
  myGLCD.print(print_text[74], 6, 92);  // 27
  myGLCD.print(print_text[75], 6, 76);  // 28
  myGLCD.print(print_text[76], 6, 60);  // 29
  myGLCD.print(print_text[39], 6, 44);  // 30
  myGLCD.print(print_text[78], 6, 28);  // 31

 // myGLCD.setFont(RusFont1);
  myGLCD.setColor(200, 200, 200);
  myGLCD.print(print_text[187], 28, 183);   // 0
  myGLCD.print(print_text[48], 50, 183);    // 2
  myGLCD.print(print_text[50], 72, 183);    // 4 
  myGLCD.print(print_text[79], 94, 183);    // 6
  myGLCD.print(print_text[80], 116, 183);   // 8
  myGLCD.print(print_text[37], 138-5, 183); // 10
  myGLCD.print(print_text[81], 160-4, 183); // 12
  myGLCD.print(print_text[82], 182-5, 183); // 14
  myGLCD.print(print_text[83], 204-5, 183); // 16
  myGLCD.print(print_text[84], 226-5, 183); // 18
  myGLCD.print(print_text[38], 248-5, 183); // 20
  myGLCD.print(print_text[69], 270-5, 183); // 22
  myGLCD.print(print_text[71], 292-5, 183); // 24
  
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255, 104, 255);
  myGLCD.drawCircle(8, 13, 1);    // значек градуса   
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[16]))); 
   myGLCD.print(buffer, 13, 10);  //  C (цельсий)  
  
  myGLCD.setColor(64, 64, 64);    // серый цвет 
  myGLCD.drawRoundRect(0, 0, 319, 225);  // нарисовать рамку  
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(88, 255, 238);  // цвет бирюзовый 
  myGLCD.setBackColor(0, 0, 0);
   //setFont(RUS1, 88, 255, 238, 0, 0, 0); 
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[157]))); 
   myGLCD.print(buffer, 70, 6);   // ТЕМПЕРАТУРА ВОДЫ ЗА СУТКИ
   
  //myGLCD.setColor(200, 200, 200);
  for(int k=31; k<191; k+=16){ myGLCD.drawLine(30, k, 24, k);}   // шкала напротив температуры 
  for(int k=39; k<181; k+=16){ myGLCD.drawLine(30, k, 27, k);}   // доп. шкала напротив температуры
  
  for(int L=30; L<306; L+=22){ myGLCD.drawLine(L, 175, L, 181);} // шкала (горизонт.) x-основная
  for(int L=41; L<306; L+=22){ myGLCD.drawLine(L, 175, L, 178);} // шкала (горизонт.) x-доп.
  
  myGLCD.setColor(200, 200, 200);
  myGLCD.drawLine(29, 27, 29, 176);    // y вертикальная линия шкалы
//                     ^ - по вертикали 
  myGLCD.drawLine(30, 175, 310, 175);  // x горизонтальная линия шкалы                       

   myGLCD.setColor(64, 64, 64);        // цвет линии серый
  for(int k=31; k<175; k+=16){ myGLCD.drawLine(30, k, 310, k);}
  
  // myGLCD.drawRect(64, 3, 265, 16);    // рамка вокруг надписи лог температуры
  
// Y вертикальные линии (сетка) в меню график температуры 
  for(int L=30; L<306; L+=11){ myGLCD.drawLine(L, 31, L, 175);} // вертикальные линии (сетка)

  linhaR =setTempC;           // Линии сравнения (номинальная темппература R)
  linhaG =(setTempC+offTempC);
  linhaB = (setTempC-offTempC);
                                                   
  if ((linhaR>22)&&(linhaR<=31))x=(175-((linhaR-22)*16));   // номинальная температура
                               else if (linhaR > 31) x=15;   
                               else if (linhaR < 22) x=175;  

  if ((linhaG>22)&&(linhaG<=31))y=(175-((linhaG-22)*16));
                               else if (linhaG > 31) y=15;   
                               else if (linhaG < 22) y=175; 
                               
   if ((linhaB>22)&&(linhaB<=31))z=(175-((linhaB-22)*16));
                               else if (linhaB > 31) z=15;   
                               else if (linhaB < 22) z=175;                              

  myGLCD.setColor(255, 0, 0);
  myGLCD.drawLine(31, x, 310, x);  // Желаемая Температура (красная линия)
  myGLCD.setColor(10, 10, 255);    // Цвет синий 
  myGLCD.drawLine(31, y, 310, y);  // верхняя линия 
  myGLCD.drawLine(31, z, 310, z);  // нижняя линия 
  
// лог температуры из памяти
   myGLCD.setColor(255, 255, 0);
       for (int i=0; i<46; i++){ 
         int stl = 5.5;
         int Ste = ((i*5.5)+30);
//         int stl = 11;
//         int Ste = ((i*11)-58);         			
         float tLinS, tLinS1;
         float tLinE, tLinE1;
         tLinS = media[i]; if (tLinS > 310) {tLinS = 310;} if (tLinS < 220) {tLinS = 220;}
         tLinE = media[i+1]; if (tLinE > 310) {tLinE = 310;} if (tLinE < 220) {tLinE = 220;} 
         tLinS = map (tLinS, 220, 310, 175, 30);
         tLinE = map (tLinE, 220, 310, 175, 30); 
         myGLCD.drawLine(Ste, tLinS, Ste+stl, tLinE );

         tLinS1 = media[46]; if (tLinS1 > 310) {tLinS1 = 310;} if (tLinS1 < 220) {tLinS1 = 220;}
         tLinE1 = media[0]; if (tLinE1 > 310) {tLinE1 = 310;} if (tLinE1 < 220) {tLinE1 = 220;} 
         tLinS1 = map (tLinS1, 220, 310, 175, 30);
         tLinE1 = map (tLinE1, 220, 310, 175, 30); 
         myGLCD.drawLine(283, tLinS1, 294, tLinE1 ); } }
         
// Вертикальная шкала времени 
void timedrawScreen(){     
            myGLCD.setFont(SmallFont);
      float x1 = (RTC.hour*11+30.0); //+RTC.minute
            myGLCD.setColor(200, 200, 200);   // вертикальная линия времени
            myGLCD.setBackColor(0, 0, 0);    // цвет фона черный
            myGLCD.drawLine(x1,177,x1,22);  // размер стрелки
            myGLCD.setColor(0, 240, 0);            
   // if (RTC.hour<10){myGLCD.print("0",x1-40,16); myGLCD.printNumI(RTC.hour,x1-32,16);} // часы
  if (RTC.hour<10){myGLCD.print(print_text[187],x1-40,16); myGLCD.printNumI(RTC.hour,x1-32,16);} // часы
                                                  else { myGLCD.printNumI(RTC.hour,x1-40,16);}
         //   myGLCD.print(":00",x1-24,16);
            myGLCD.print(print_text[56],x1-24,16);  // : двоеточие          
   if (RTC.minute <10){myGLCD.print(print_text[187],x1-16,16); myGLCD.printNumI(RTC.minute,x1-8,16);} // минуты
                                                   else { myGLCD.printNumI(RTC.minute,x1-16,16);} //}
            myGLCD.setFont(RusFont6);
            myGLCD.setColor(0, 240, 0);   // Text rotation              
           // myGLCD.print(print_text[153], 33,31,270);     // сегодня   перевернутый текст         
        //    myGLCD.print("'BWEP@%", 305,130,270);   // вчера   290,120,270);  
    }
 void tFANScreen(){ // график температуры радиаторов датчики 1,2 *************************** dispScreen = 27 
 
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
  
     graph_colorFAN(); }

void graph_colorFAN(){
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(1, 1, 318, 193);     // clear
       
    printButton100("", Fg1[0], Fg1[1], Fg1[2], Fg1[3], SMALL);   // ДАТЧ1
    printButton100("", Fg2[0], Fg2[1], Fg2[2], Fg2[3], SMALL);   // ДАТЧ2
    
        myGLCD.setFont(RusFont6);
        myGLCD.setColor(255, 255, 255);
        myGLCD.setBackColor(100, 100, 100); 
        myGLCD.print(print_text[87], 117, 203);  // ДАТЧ1
        myGLCD.setBackColor(100, 100, 100);
        myGLCD.print(print_text[88], 168, 203);  // ДАТЧ2 
        
  myGLCD.setColor(90, 90, 90);        // цвет линии серый (сетка)
  for(int k=31; k<175; k+=8){ myGLCD.drawLine(30, k, 305, k);}  // горизонтальные линии (сетка) 
  for(int L=30; L<306; L+=11){ myGLCD.drawLine(L, 31, L, 175);} // вертикальные линии (сетка)

  myGLCD.setColor(255, 255, 255);        
  myGLCD.drawLine(29, 27, 29, 176);   // y вертикальная линия шкалы
  myGLCD.drawLine(30, 175, 310, 175); // x горизонтальная линия шкалы 
   
 for (byte i=0; i<3; i++) {	
//   int FAN = i; 
 if ((i==0)&&(F1==true)){for (byte i=0; i<47; i++) tFA[i]=tF1[i]; sbR=255; sbG=0; sbB=0; 
      printButton104("", Fg1[0], Fg1[1], Fg1[2], Fg1[3], SMALL); 
           myGLCD.setFont(RusFont6); 
           myGLCD.setColor(255, 255, 255);       
           myGLCD.setBackColor(255, 0, 0);   // серый
           myGLCD.print(print_text[87], 117, 203); } // ДАТЧ1     
                                                
 if ((i==2)&&(F2==true)){for (byte i=0; i<47; i++) tFA[i]=tF2[i]; sbR=180; sbG=180; sbB=0; 
      printButton("", Fg2[0], Fg2[1], Fg2[2], Fg2[3],SMALL, GREEN_BAC); 
           myGLCD.setFont(RusFont6); 
           myGLCD.setColor(255, 255, 255);       
           myGLCD.setBackColor(0, 180, 86);   // серый
           myGLCD.print(print_text[88], 168, 203); } // ДАТЧ2                
                                                                              
    if ((F1==false) && (F2==false)) {for (byte i=0; i<47; i++) tFA[i] = 0;}
   
    myGLCD.setColor(sbR, sbG, sbB);
       for (int i=0; i<46; i++) {               // построение графиков 
         float stl = 5.5;    
         float Ste = ((i*5.5)+30);			
         float tLinS, tLinS1;
         float tLinE, tLinE1;
      tLinS = tFA[i]; if (tLinS > 420) {tLinS = 420;} if (tLinS < 240) {tLinS = 240;}
      tLinE = tFA[i+1]; if (tLinE > 420) {tLinE = 420;} if (tLinE < 240) {tLinE = 240;}
      tLinS = map (tLinS, 240, 420, 175, 30);
      tLinE = map (tLinE, 240, 420, 175, 30);
    myGLCD.drawLine(Ste, tLinS, Ste+stl, tLinE );

   tLinS1=tFA[46]; if (tLinS1>420){tLinS1=420;} if (tLinS1<240){tLinS1=240;} tLinS1=map(tLinS1, 240, 420, 175, 30);
   tLinE1=tFA[0]; if (tLinE1>420){tLinE1=420;} if (tLinE1<240){tLinE1=240;} tLinE1=map(tLinE1, 240, 420, 175, 30);
    myGLCD.drawLine(283, tLinS1, 294, tLinE1 ); }
    
//   шкала температуры
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 255, 255, 255, 0, 0, 0);
  myGLCD.print(F("42"), 7, 28);          // 42
  myGLCD.print(print_text[40], 7, 44);   // 40
  myGLCD.print(F("38"), 7, 60);          // 38
  myGLCD.print(F("36"), 7, 76);          // 36
  myGLCD.print(F("34"), 7, 92);          // 34
  myGLCD.print(F("32"), 7, 108);         // 32
  myGLCD.print(print_text[39], 7, 124);  // 30
  myGLCD.print(print_text[75], 7, 140);  // 28
  myGLCD.print(print_text[73] , 7, 156); // 26
  myGLCD.print(print_text[71] , 7, 171); // 24
  
//   шкала времени     
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(255, 255, 255);
  myGLCD.print(print_text[187], 28, 183);   // 0
  myGLCD.print(print_text[48], 50, 183);    // 2
  myGLCD.print(print_text[50], 72, 183);    // 4
  myGLCD.print(print_text[79], 94, 183);    // 6
  myGLCD.print(print_text[80], 116, 183);   // 8
  myGLCD.print(print_text[37], 138-5, 183); // 10
  myGLCD.print(print_text[81], 160-4, 183); // 12
  myGLCD.print(print_text[82], 182-5, 183); // 14
  myGLCD.print(print_text[83], 204-5, 183); // 16
  myGLCD.print(print_text[84], 226-5, 183); // 18
  myGLCD.print(print_text[38], 248-5, 183); // 20
  myGLCD.print(print_text[69], 270-5, 183); // 22
  myGLCD.print(print_text[71] , 292-5, 183);// 24

  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255, 104, 255);
  myGLCD.drawCircle(8, 13, 1);   // значек градуса  
       strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[16]))); 
   myGLCD.print(buffer, 13,10);  // C значек сельсия
  
  myGLCD.setColor(64, 64, 64);   // серый цвет 
  myGLCD.drawRoundRect(0, 1, 319, 225);  // нарисовать рамку 
  
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(88, 255, 238);   // цвет бирюзовый 
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 88, 255, 238, 0, 0, 0); 
  myGLCD.print(print_text[151], CENTER, 6);   // температура радиаторов за сутки

  myGLCD.setColor(255, 255, 255);
  for(int k=31; k<191; k+=16){ myGLCD.drawLine(30, k, 24, k);}     // шкала (трип) напротив температуры 
  for(int k=39; k<181; k+=16){ myGLCD.drawLine(30, k, 27, k);}     // доп. шкала напротив температуры

  for(int L=30; L<306; L+=22){ myGLCD.drawLine(L, 175, L, 181);}   // шкала (трип)x-основная
  for(int L=41; L<306; L+=22){ myGLCD.drawLine(L, 175, L, 178);}}} // шкала (трип)x-доп.
   
/******************* Графики освещенности ********************************************************** dispScreen = 27 */
void AllColourGraph(){   // used in "rapid test" and "All color graph "

	myGLCD.setColor(64, 64, 64);                          // Draw Dividers in Grey
	myGLCD.drawRect(0, 196, 319, 194);                    // Bottom Horizontal Divider
	printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);  // << BACK
	printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);  // CANCEL

        calculateStartTime();  // calculate SUNRISE time 
	calculateStopTime();   // calculate SUNSET time

 if (dispScreen ==29){printButton(print_text[96], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL);} // button start/stop test 
          graph_color(); }

void graph_color() {     //  repeat 9 time

      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(1, 0, 318, 193);  // Очистить экран     
      printButton100(print_text[20], Wg[0], Wg[1], Wg[2], Wg[3], SMALL);       // white
      printButton100(print_text[21], Bg[0], Bg[1], Bg[2], Bg[3], SMALL);       // blue
      printButton100(print_text[22], RBg[0], RBg[1], RBg[2], RBg[3], SMALL);   // rblue
      printButton100(print_text[23], Rg[0], Rg[1], Rg[2], Rg[3], SMALL);       // R
      printButton100(print_text[24], UVg[0], UVg[1], UVg[2], UVg[3], SMALL);   // UV
      printButton100(print_text[25], ORg[0], ORg[1], ORg[2], ORg[3], SMALL);   // Or
      printButton100(print_text[26], GRg[0], GRg[1], GRg[2], GRg[3], SMALL);   // Gr

 if (LedShannelStatusByte==0){StartTime=0; StopTime=95;}          // draw graph if all led is OFF
             drawLedStaticChartP1();
   
            for (byte i=1; i<=8; i++){ COLOR = i; 
  if ((i==1)&&(W==true)){for(byte i=0; i<96; i++)tled[i]=wled[i];sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; 
              myGLCD.setColor(255, 255, 255);         // белый канал
              myGLCD.fillRoundRect (37, 0, 67,15);
             setFont(SMALL, 0, 0, 0, 255, 255, 255);
              myGLCD.print(print_text[20], 50, 2);   // W
              myGLCD.setColor(255, 0, 0);
              myGLCD.drawRoundRect (37, 0, 67,15); }
               
  if ((i==2)&&(B==true)){for(byte i=0; i<96; i++)tled[i]=rbled[i];sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; 
              myGLCD.setColor(9, 184, 255);         // голубой канал
              myGLCD.fillRoundRect (77, 0, 107,15);
             setFont(SMALL, 255, 255, 255, 9, 184, 255);
              myGLCD.print(print_text[21], 90, 2);   // B
              myGLCD.setColor(255, 255, 255);
              myGLCD.drawRoundRect (77, 0, 107,15); }
                              
  if ((i==3)&&(RB==true)){for(byte i=0; i<96; i++)tled[i]=bled[i];sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; 
               printButton(print_text[22], RBg[0], RBg[1], RBg[2], RBg[3], SMALL); }        // rblue 
               
  if ((i==4)&&(R==true)){for(byte i=0; i<96; i++)tled[i]=rled[i];sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; 
               printButton104(print_text[23], Rg[0], Rg[1], Rg[2], Rg[3], SMALL);}          // red 
               
  if ((i==5)&&(UV==true)){for(byte i=0; i<96; i++)tled[i]=uvled[i];sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; 
              myGLCD.setColor(224, 102, 255);         // фиолетовый канал
              myGLCD.fillRoundRect (197, 0, 227,15);
             setFont(SMALL, 255, 255, 255, 224, 102, 255);
              myGLCD.print(print_text[24], 206, 2);   // UV
              myGLCD.setColor(255, 255, 255);
              myGLCD.drawRoundRect (197, 0, 227,15); }
               
  if ((i==6)&&(OR==true)){for(byte i=0; i<96; i++)tled[i]=oLed[i];sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; 
              myGLCD.setColor(255, 143, 32);         // оранжевый канал
              myGLCD.fillRoundRect (237, 0, 267,15);
             setFont(SMALL, 255, 255, 255, 255, 143, 32);
              myGLCD.print(print_text[25], 247, 2);  // Or
              myGLCD.setColor(255, 255, 255);
              myGLCD.drawRoundRect (237, 0, 267,15); }
                              
  if ((i==7)&&(SU==true)){for(byte i=0; i<96; i++)tled[i]=gled[i];sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; 
               printButton(print_text[26], GRg[0], GRg[1], GRg[2], GRg[3],SMALL, GREEN_BAC);}  // зеленая кнопка
      
  if (i!=8){ ReadOnOffLedStatus(); 
       if (LedShannelFlag_on_off == true){ draw_one_led_graph(); }
                    else { EndScale=290;}                     // for draw tick scale if all led is OFF
	           } } drawLedStaticChartP2(); }

void draw_one_led_graph(){
           byte stepScale = (294 - 26)/(StopTime-StartTime);	  // 26 and 294 - left / right 
	        myGLCD.setColor(sbR, sbG, sbB);
            for (byte i=StartTime; i<StopTime; i++){		  // draw led value graph (curve)
	         int tempLineS;				          // start segment of line
		 int tempLineE;					  // stop segment of line
		 int XStep = map((i-StartTime), 0, (StopTime-StartTime), 26, 294);
	                                tempLineS=tled[i];

  if (tempLineS > 100) {tempLineS =100;} // 255 
		      tempLineE=tled[i+1];
  if (tempLineE > 100) {tempLineE =100;} // 255
		 tempLineS= map (tempLineS, 0, 100, BotSldY, TopSldY);   // mapping 0-100 value to chart size 0, 255
		 tempLineE= map (tempLineE, 0, 100, BotSldY, TopSldY);

		 myGLCD.drawLine(XStep, tempLineS, XStep+stepScale, tempLineE );
		 myGLCD.drawLine(XStep, tempLineS+1, XStep+stepScale, tempLineE+1 );
		 EndScale = XStep+stepScale; } }

void drawLedStaticChartP1(){   // draw TOP ON/OFF time and X/Y border 

      TopSldY=20; BotSldY=170;				// graph vertical size - график - размер по вертикали
	  LightDay= (StopTime-StartTime)/4*10;		// light day in HOUR * 10 - свет дня в часах*10
	  int Temp = 10*(StopTime-StartTime)/4;		// rounding to the nearest whole number if "light day" similar 3.4 or 3.75 etc...округление до целого числа
  if ((Temp - LightDay)>=5) {LightDay= (StopTime-StartTime)/4+1;}
                       else {LightDay= (StopTime-StartTime)/4; }    

	  myGLCD.setColor(255, 255, 255);		// цвет шкалы графиков
	  setFont(SMALL, 255, 255, 255, 0, 0, 0);       // шрифт надписей на шкалах графиков
	  myGLCD.drawRect(26, BotSldY, 27, TopSldY);	// print y-line
	     for (byte i=1; i<11; i++){                           
    myGLCD.drawLine(28, (i*(BotSldY-TopSldY)/10+5), 30, (i*(BotSldY-TopSldY)/10+5));} // Y tick-marks 

    	  myGLCD.drawRect(26, BotSldY, 307, BotSldY+1);	 // print x-line (горизонтальная линия шкалы)
    
        for (byte i=0; i<=10; i++){   // вертикальная шкала процентов 0-100 (10 делений)
           // myGLCD.setFont(RusFont1);
            myGLCD.setColor(192, 236, 255); // голубой
	       if (i==0) {myGLCD.printNumI(0, 18, ((BotSldY-TopSldY)/10*(11-i)));}    
               if (i==10){myGLCD.printNumI(100, 2, (BotSldY-TopSldY)/10*(11-i));}
		    else {myGLCD.printNumI(i*10, 10, (BotSldY-TopSldY)/10*(11-i));}}}
		     
void drawLedStaticChartP2(){         // draw BOT ON/OFF time and X/Y scale and tick mark
            
	         myGLCD.setColor(255, 255, 255);
	         myGLCD.drawRect(26, BotSldY, 307, BotSldY+1);	  // print x-line 

	if (LightDay<=7){					  // print X-tick and dot background	
	       for (byte i=0; i<(StopTime-StartTime)+2; i++){	  // i-horisontal count, X tick-marks with 15min resolution
		     int XStep = map(i, 0, (StopTime-StartTime), 26, EndScale);
		        myGLCD.setColor(255, 255, 255);
                        myGLCD.drawLine(XStep, BotSldY+3, XStep, BotSldY-3);  // k - vertical count	
                        
                  for (byte k=1; k<=10; k++){                                 // k - vertical count   
		        myGLCD.setColor(180, 180, 180);
		        myGLCD.drawPixel(XStep, k*(BotSldY-TopSldY)/10+5);}}}

	 if (LightDay>7 && LightDay<=12){
	       for (byte i=0; i<(StopTime-StartTime)/2+2; i++){	    // X tick-marks with 30min resolution
		    int XStep = map(i, 0, (StopTime-StartTime)/2, 26, EndScale);
			 myGLCD.setColor(255, 255, 255);
			 myGLCD.drawLine(XStep, BotSldY+3, XStep, BotSldY-3);   // X mark on horisontal scale
   
		   for (byte k=1; k<=10; k++){                              // k - vertical count
		   myGLCD.setColor(180, 180, 180);
		   myGLCD.drawPixel(XStep, k*(BotSldY-TopSldY)/10+5);}}}       

	 if (LightDay>12 && LightDay<=24){
	       for (byte i=0; i<(StopTime-StartTime)/4+4; i++){             // X tick-marks with 1hour resolution
		    int XStep = map(i, 0, (StopTime-StartTime)/4, 26, EndScale);
                        myGLCD.setColor(255, 255, 255); 
	                myGLCD.drawLine(XStep, BotSldY+3, XStep, BotSldY-3);
                        myGLCD.setColor(255, 255, 255); 
                        
		    for (byte k=1; k<=10; k++){                             // k - vertical count
		    myGLCD.setColor(180, 180, 180);
		    myGLCD.drawPixel(XStep, k*(BotSldY-TopSldY)/10+5);}}} // пунктир 

                         myGLCD.setFont(RusFont1);
                         myGLCD.setColor(0, 255, 0);
                         myGLCD.setBackColor(0, 0, 0);
	 		 myGLCD.printNumI(StartTime/4, 8, BotSldY+8, 2, '00');  // print start time on scale 
                         myGLCD.print(print_text[56], 24, BotSldY+8);  // :
			 myGLCD.printNumI((StartTime*15)%60, 32, BotSldY+8, 2, '00');  
	
                         myGLCD.setFont(RusFont1);
                         myGLCD.setColor(255, 0, 0);
                         myGLCD.setBackColor(0, 0, 0);
	  	         myGLCD.printNumI(StopTime/4, EndScale-16, BotSldY+8);  // print stop time on scale 
                         myGLCD.print(print_text[56], EndScale-1, BotSldY+7);  // :
		         myGLCD.printNumI((StopTime*15)%60, EndScale+8, BotSldY+8, 2, '00'); 

                         myGLCD.setFont(RusFont1);
                         myGLCD.setColor(190, 190, 190); // серый шрифт
                         myGLCD.setBackColor(0, 0, 0);
                         myGLCD.print(print_text[143], 2, 186);    // рассвет 
                         myGLCD.print(print_text[144], 277, 186);  // закат
                        
                         myGLCD.setColor(255, 255, 255); // белый шрифт
		         myGLCD.print(print_text[81], 86, 183);    // 12
		         myGLCD.print(print_text[82], 131, 183);   // 14 
		         myGLCD.print(print_text[83], 176, 183);   // 16
		         myGLCD.print(print_text[84], 220, 183); } // 18


    
// *************** Авто-поиск датчиков температуры ************************************ dispScreen 
void DetectDallalsSensors(boolean refreshSensor=true){ 

      PrintStringIndex=24; printHeader (); // АВТО-ОПРЕДЕЛЕНИЕ ДАТЧИКОВ  

   if (refreshSensor== true){
	myGLCD.setColor(64, 64, 64);              // Draw Dividers in Grey
	myGLCD.drawLine(0, 93, 319, 93);
	myGLCD.drawRect(0, 196, 319, 194);        // Bottom Horizontal Divider 
        myGLCD.setColor(0, 0, 255);               
        myGLCD.fillRoundRect(165, 19, 295, 41);  // кнопка ПОИСК ДАТЧИКОВ  (185, 20, 275, 40);
        myGLCD.setColor(255, 255, 255);                    
        myGLCD.drawRoundRect(165, 19, 295, 41);
        
          myGLCD.setFont(RusFont2);
          myGLCD.setColor(255, 255, 255);    // шрифт белый
          myGLCD.setBackColor(0, 0, 255); 
        //setFont(RUS2, 255, 255, 255, 0, 0, 255);
        myGLCD.print(print_text[204], 175, 25); // ПОИСК ДАТЧ.   

	printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
	printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
        printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
        
          myGLCD.setFont(RusFont1);
          myGLCD.setColor(0, 255, 0);      // шрифт зеленый
          myGLCD.setBackColor(0, 0, 0); 
        //setFont(RUS1, 0, 255, 0, 0, 0, 0);
        myGLCD.print(print_text[208], 5, 26);     // Найдено:
        myGLCD.print(print_text[207], 90, 26); }  // Датчиков
	
   if (refreshSensor== false) { 
	myGLCD.setColor(0, 0, 0);	       // clear button and text area after pressing refresh
	myGLCD.fillRoundRect(4, 95, 318, 190);
	myGLCD.fillRoundRect(4, 45, 318, 92);}

	sensors.begin();			             // start sensors
	numberOfDevices = sensors.getDeviceCount();          // find devices
	setFont(SMALL, 255, 255, 255, 0, 0, 0);              // шрифт белый

          //myGLCD.setFont(RusFont1);
          //myGLCD.setColor(255, 255, 255);   // шрифт белый
          //myGLCD.setBackColor(0, 0, 0);     // синий фон 
	myGLCD.printNumI(numberOfDevices, 18+56, 24);       // колличество найденных датчиков (найдено)
            
    if (numberOfDevices == 0) {counterB1=0; counterB2=0; counterB3=0;}  // set all counter N.C, 
	  tempAlarmflag = false;    // clear all error flag
	  tempHeatflag = false;
	  tempCoolflag = false;
	 digitalWrite(tempAlarmPin, LOW);

	for (byte i = 0; i < 8; i++){                         // clear previos sensor ID value
              Heatsink1Thermometer[i]=0;
	      waterThermometer[i]=0 ;
	      Heatsink2Thermometer[i]=0; }

	for (byte k=0; k<numberOfDevices; k++){
	myGLCD.setColor(0, 0, 255);      // синий
        myGLCD.fillRoundRect(165, 83+20+30*k, 231, 83+40+30*k); // кнопка     
	myGLCD.setColor(255, 255, 255);  // белый                  
        myGLCD.drawRoundRect(165, 83+20+30*k, 231, 83+40+30*k); // рамка
        
              myGLCD.setFont(RusFont3);         // русский фонт
              myGLCD.setColor(255, 255, 255);   // шрифт белый
              myGLCD.setBackColor(0, 0, 255);   // синий фон
             //setFont(RUS3, 255, 255, 255, 0, 0, 255); // шрифт белый, синий фон
   if (k==0){ strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB1]))); 
	  myGLCD.print(buffer, 173, 85+24+30*k);}   // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2	            
   if (k==1){ strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB2])));   
	   myGLCD.print(buffer, 173, 85+24+30*k);}  // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2	            
   if (k==2) { strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB3]))); 
	   myGLCD.print(buffer, 173, 85+24+30*k);}} // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2

	 setFont(SMALL, 0, 255, 0, 0, 0, 0); // показать колличество найденных датчиков
	 for(byte k=0; k<numberOfDevices; k++){   	

     if (sensors.getAddress(tempDeviceAddress, k)){
	  float tempC = sensors.getTempC(tempDeviceAddress);
              
// connect sensor 1...3 to output Water / Heatsink1 / Heatsink2
	if (k==0){ for (byte i=0; i<8; i++) {
	      if (counterB1 == 1) {waterThermometer[i] = tempDeviceAddress[i];}
              if (counterB1 == 2) {Heatsink1Thermometer[i] = tempDeviceAddress[i];}
              if (counterB1 == 3) {Heatsink2Thermometer[i] = tempDeviceAddress[i];} } }
              
	if (k==1){ for (byte i=0; i<8; i++) {
	      if (counterB2 == 1) {waterThermometer[i] = tempDeviceAddress[i];}
              if (counterB2 == 2) {Heatsink1Thermometer[i] = tempDeviceAddress[i];}
              if (counterB2 == 3) {Heatsink2Thermometer[i] = tempDeviceAddress[i];} } }
              
	if (k==2){ for (byte i=0; i<8; i++) {
	      if (counterB3 == 1) {waterThermometer[i] = tempDeviceAddress[i];}
              if (counterB3 == 2) {Heatsink1Thermometer[i] = tempDeviceAddress[i];}
              if (counterB3 == 3) {Heatsink2Thermometer[i] = tempDeviceAddress[i];} } }             
              
	    myGLCD.printNumF( tempC, 1, 230+40, k*30+83+24);    // показать текущую температуру найденного датчика
	    myGLCD.print(F("T.-"), 241, k*30+83+24); 
            myGLCD.setColor(255, 255, 255);    // шрифт белый
	    myGLCD.print(print_text[57], 230+40+32+6, k*30+83+24);
            myGLCD.drawCircle(230+40+32+2, k*30+83+26, 1);   
            
            myGLCD.setColor(255, 255, 255);    // шрифт белый
            myGLCD.printNumI(k+1, 5+62, k*15+48);    // номер датчика 
            myGLCD.print(print_text[56], 5+62+8, k*15+48);  // :
                                                       
                myGLCD.setFont(RusFont1);      // русский фонт
                myGLCD.setColor(80, 158, 255); // cиний
         //    myGLCD.print(buffer, 5, k*30+85+24);     // Датчик (имя) (перед "подключен") (имя из буфера)      
             myGLCD.print(print_text[206], 5, k*30+85+24);  // Датчик (имя) (перед "подключен")   
             myGLCD.setColor(255, 255, 255);
             myGLCD.printNumI(k+1, 5+55, k*30+85+24); // номер датчика (цифра) (перед "подключен") (64)
                
                myGLCD.setColor(0, 255, 0);    // шрифт зеленый
                myGLCD.setBackColor(0, 0, 0); 
              myGLCD.print(print_text[206], 11, k*15+51);         // Датчик
              myGLCD.print(print_text[205], 9+55+8, k*30+83+26);  // Подключен К
            
           setFont(SMALL, 88, 255, 238, 0, 0, 0);
		for (byte i = 0; i < 8; i++) {             // серийный номер датчика  unic ID
         
    byte result;
	byte temp = tempDeviceAddress[i];
	                       result = temp/16;		     // first char  convert HEX to ASCII for print 
	if (result == 15) {myGLCD.print(F("F"), i*24+88, k*15+48);}         // F
	if (result == 14) {myGLCD.print(print_text[85], i*24+88, k*15+48);} // E
	if (result == 13) {myGLCD.print(print_text[86], i*24+88, k*15+48);} // D
	if (result == 12) {myGLCD.print(print_text[57], i*24+88, k*15+48);} // C
	if (result == 11) {myGLCD.print(print_text[21], i*24+88, k*15+48);} // B
	if (result == 10) {myGLCD.print(F("A"), i*24+88, k*15+48);}         // A
	if (result < 10 ) {myGLCD.printNumI(result, i*24+88, k*15+48);}

	                    result = temp - (temp/16)*16;           // second char  convert HEX to ASCII for print
	if (result == 15) {myGLCD.print(F("F"), i*24+8+88, k*15+48);}         // F
	if (result == 14) {myGLCD.print(print_text[85], i*24+8+88, k*15+48);} // E
	if (result == 13) {myGLCD.print(print_text[86], i*24+8+88, k*15+48);} // D
	if (result == 12) {myGLCD.print(print_text[57], i*24+8+88, k*15+48);} // C
	if (result == 11) {myGLCD.print(print_text[21], i*24+8+88, k*15+48);} // B
	if (result == 10) {myGLCD.print(F("A"), i*24+8+88, k*15+48);}         // A
	if (result < 10 ) {myGLCD.printNumI(result, i*24+8+88, k*15+48);}
        if (i<7) {myGLCD.print(print_text[56], i*24+16+88, k*15+48);} } } } } // двоеточие

void Backup(){  // запись настроек на SD карту
        PrintStringIndex=25; printHeader ();  // СОХРАНИТЬ / ЗАГРУЗИТЬ НАСТРОЙКИ
	printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
	printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
	myGLCD.setColor(0, 0, 255); 
        myGLCD.fillRoundRect(15, 39+34, 115, 75+34);  // нарисовать кнопку ЗАГРУЗИТЬ  
        myGLCD.fillRoundRect(205, 39+34, 305, 75+34); // нарисовать кнопку СОХРАНИТЬ
	myGLCD.setColor(255, 255, 255);
        myGLCD.drawRoundRect(15, 39+34, 115, 75+34);
        myGLCD.drawRoundRect(205, 39+34, 305, 75+34); // рамка вокруг СОХРАНИТЬ

        myGLCD.setFont(RusFont1);        // русский фонт
        myGLCD.setColor(255, 255, 255);  // шрифт зеленый
        myGLCD.setBackColor(0, 0, 255);
        //setFont(RUS1, 255, 255, 255, 0, 0, 255); 
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[130]))); 
        myGLCD.print(buffer, 215+5, 45+36);            // СОХРАНИТЬ      
        myGLCD.print(print_text[150], 220, 94);        // НАСТРОЙКИ     
        myGLCD.print(print_text[192], 11+18, 45+36);   // ЗАГРУЗИТЬ (RESTORE) 
        myGLCD.print(print_text[150], 30, 94);         // НАСТРОЙКИ

            myGLCD.setBackColor(0, 0, 0);
          myGLCD.print(print_text[149], CENTER, 24);    // ВСЕ НАСТРОЙКИ НАХОДЯТСЯ НА ФЛЕШ КАРТЕ
          myGLCD.print(print_text[148], 47, 24+12+5);   // В ТЕКСТОВОМ ФАЙЛЕ
       
            setFont(SMALL, 0, 255, 0, 0, 0, 0); 
           myGLCD.print(print_text[147], 198, 24+10+5);  //  BACKUP.TXT
        
	    myGLCD.setColor(64, 64, 64);            // Draw Dividers in Grey
	    myGLCD.drawLine(0, 60, 319, 60);
            myGLCD.setColor(150, 150, 150);
            myGLCD.drawRoundRect(15, 125, 304, 182); // рамка в центре экрана
        
            myGLCD.setFont(RusFont1);
            myGLCD.setColor(0, 255, 0);  // шрифт зеленый
            myGLCD.setBackColor(0, 0, 0);
           //setFont(RUS1, 0, 255, 0, 0, 0, 0);
           myGLCD.print(print_text[203], CENTER, 138);   // Инициализация Флеш карты...

// initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
// breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(SDchipSelect, SPI_HALF_SPEED)){ sd.initErrorPrint();
        myGLCD.setFont(RusFont1);      // русский фонт
        myGLCD.setColor(255, 0, 0);    // шрифт зеленый
        myGLCD.setBackColor(0, 0, 0);
       //setFont(RUS1, 255, 0, 0, 0, 0, 0); 
        myGLCD.print(print_text[202], CENTER, 140+13+5);      // Ошибка ! инициализации флеш карты 
  } else {
        myGLCD.print(print_text[201], CENTER, 140+13+5); } }  // Инициализация успешно завершена 
               
void drawTestLedArrayScale(){  // авто - тест 
              int i=0;         // start scale is constant
          min_cnt = StartTime*15;

   while (LEDtestTick== true &&  i<= ((StopTime-StartTime)*15)/2){   // LEDtestTick== true - start test	  
             unsigned long currentMillis = millis();
             
	 if (myTouch.dataAvailable()){ processMyTouch();}
         if (currentMillis - previousMillisLED > 1000)                   // change time every 1s
             {previousMillisLED = currentMillis; 

	 int XStep = map(i, 0, ((StopTime-StartTime)*15)/2, 26, EndScale);
	        myGLCD.setColor(110, 110, 110);     // серый  
                myGLCD.drawRect(31, 23, 111, 42);         // белая рамка вокруг отсчета времени
	        setFont(LARGE, 88, 255, 238, 0, 0, 0);    // отсчет времени шрифт бирюзовый, фон черный            
     if (((min_cnt/15)/4)<=9) {myGLCD.printNumI((min_cnt/15)/4, 48, 25);} // 2
		         else {myGLCD.printNumI((min_cnt/15)/4, 32, 25);} // 2 
                               myGLCD.print(print_text[56], 64, 25);      // :
                               myGLCD.printNumI(min_cnt%60, 78, 25, 2, '00'); // 20, 20
                               
        	       myGLCD.setColor(88, 255, 238);   // цвет полосы времени бирюзовый
		       myGLCD.drawLine(XStep, BotSldY+8, XStep, BotSldY+3 );
		            LED_levelo_output();
				 min_cnt+=2 ;             // two minutes increment
			          i +=1; } }
         
      printButton(print_text[96], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL); // buton start/stop test
	   LEDtestTick = false; }                        // end test, enable button BACK/Cancel 

void calculateStartTime(){
	byte i;
     for (i=0; i<=95; i++){       // высчитать время "StartTime"  
	if (wled[i]!=0 && bitRead(LedShannelStatusByte,0)== true){goto ext_Stlbl;}	
	if (bled[i]!=0 && bitRead(LedShannelStatusByte,1)== true){goto ext_Stlbl;}		
	if (rbled[i]!=0 && bitRead(LedShannelStatusByte,2)== true){goto ext_Stlbl;}
	if (rled[i]!=0 && bitRead(LedShannelStatusByte,3)== true){goto ext_Stlbl;}	
	if (uvled[i]!=0 && bitRead(LedShannelStatusByte,4)== true){goto ext_Stlbl;}
	if (oLed[i]!=0 && bitRead(LedShannelStatusByte,5)== true){goto ext_Stlbl;}
	if (gled[i]!=0 && bitRead(LedShannelStatusByte,6)== true){goto ext_Stlbl;} }

ext_Stlbl:
    StartTime=i-1;
   if (StartTime == 255) {StartTime=0; } } // 255

void calculateStopTime(){   
	byte i;
      for (i=95; i>0; i--){        // высчитать время "StopTime"       
	if (wled[i]!=0 && bitRead(LedShannelStatusByte,0)== true){goto ext_SStlbl;}	
	if (bled[i]!=0 && bitRead(LedShannelStatusByte,1)== true){goto ext_SStlbl;}		
	if (rbled[i]!=0 && bitRead(LedShannelStatusByte,2)== true){goto ext_SStlbl;}
	if (rled[i]!=0 && bitRead(LedShannelStatusByte,3)== true){goto ext_SStlbl;}	
	if (uvled[i]!=0 && bitRead(LedShannelStatusByte,4)== true){goto ext_SStlbl;}
	if (oLed[i]!=0 && bitRead(LedShannelStatusByte,5)== true){goto ext_SStlbl;}
	if (gled[i]!=0 && bitRead(LedShannelStatusByte,6)== true){goto ext_SStlbl;} }
	 
ext_SStlbl:
      StopTime=i+1; } 

void testIndLedScreen2(boolean refreshTest=false){ // Sector *********************************************** dispScreen = 4

     TopSldY=53; BotSldY=TopSldY+100;
 if (refreshTest==true) {
   myGLCD.setColor(0, 0, 0);
   myGLCD.fillRect(0, 0, 319, 15);          // clear only header
   setFont(SMALL, 255, 255, 255, 0, 0, 0);
     myGLCD.setColor(255, 255, 204);    // белый
  myGLCD.print(print_text[184], 26, 15);    // name WHT 
     myGLCD.setColor(255, 255, 255);     // голубой
  myGLCD.print(print_text[183], 61, 15);    // name BLU
     myGLCD.setColor(58, 95, 205);     // рояль
  myGLCD.print(print_text[182], 96, 15);    // name RBL
     myGLCD.setColor(255, 0, 0);      // красный
  myGLCD.print(print_text[181], 131, 15);   // name RED
     myGLCD.setColor(224, 102, 255);  // фиолетовый  
  myGLCD.print(print_text[180], 166, 15);   // name UVL
     myGLCD.setColor(255, 143, 32);   // оранжевый
  myGLCD.print(print_text[179], 201, 15);  // name ORG
     myGLCD.setColor(0, 255, 0);      // зеленый  
  myGLCD.print(print_text[178], 235, 15);  // name GRN  
     myGLCD.setColor(176, 176, 176);  // серый
  myGLCD.print(print_text[177], 266, 15);  // name Moon
  
// sector 
//                                                  v - начало по горизонтали
   for (int b=0; b<8; b++){drawUpButtonSlide((b*35)+21, TopSldY-26);}    // UP Buttons (button)
   for (int b=0; b<8; b++){drawDownButtonSlide((b*35)+21, TopSldY+115);} // DOWN Buttons   
       temp_sector= min_cnt/15;        // read current time sector only first time

  	  myGLCD.setColor(64, 64, 64);                                  
	  myGLCD.drawRect(0, 196, 319, 194);          // рамка вокруг нижних кнопок
      printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
      printButtonRUS(print_text[29], Psect[0], Psect[1], Psect[2], Psect[3], SMALL);  // < sector
      printButtonRUS(print_text[30], Nsect[0], Nsect[1], Nsect[2], Nsect[3], SMALL);  // sector >
      printButton("", StopDay[0], StopDay[1], StopDay[2], StopDay[3], SMALL);      // top buttons
      printButton("", StartDay[0], StartDay[1], StartDay[2], StartDay[3], SMALL);  // top buttons

    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 255, 255); 
    myGLCD.setBackColor(0, 0, 255); 
    //setFont(RUS1, 255, 255, 255, 0, 0, 255); 
    myGLCD.print(print_text[143], 13, 3);    // рассвет  
  //myGLCD.setColor(176, 176, 176);  
    myGLCD.print(print_text[144], 261, 3); } // закат

  if (refreshTest==true){
	 myGLCD.setColor(130, 130, 130);       // Cерая рамка вокруг сектора времени 
         myGLCD.drawRect(182, 1, 228, 14);
       //  myGLCD.setColor(0, 255, 0);         // fill sector background in green 
       //  myGLCD.fillRect(89, 1, 226, 14);
         
// display current TIME sector = min_cnt/15, convert to hour min_cnt/15)/4, to min. (min_cnt/15)*15)%60 
              setFont(SMALL, 0, 255, 0, 0, 0, 0);        // print SECTOR TIME on TOP  шрифт зел
                  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[14]))); 
              myGLCD.print(buffer, 88, 2); }            // TIME SECTOR 
	      setFont(SMALL, 88, 255, 238, 0, 0, 0);

   if ((temp_sector/4)<=9) {
		  myGLCD.print(" ",91+96, 2);
		  myGLCD.printNumI(temp_sector/4, 91+104, 2);}               // Start sector is min_cnt/15 
	   else { myGLCD.printNumI(temp_sector/4, 91+96, 2);}               
	          myGLCD.printNumI((temp_sector*15)%60, 91+120, 2, 2, '00'); // min 00/15/30/45 in hour

                w_out = wled[temp_sector];      // read setting from buffer
		b_out = bled[temp_sector];
		rb_out = rbled[temp_sector];
		r_out = rled[temp_sector];
		uv_out = uvled[temp_sector];
		o_out = oLed[temp_sector];
		gr_out = gled[temp_sector];

// check if channel on or off
  if (bitRead(LedShannelStatusByte,0) == false){w_out=0;}  
  if (bitRead(LedShannelStatusByte,1) == false){b_out=0;}  
  if (bitRead(LedShannelStatusByte,2) == false){rb_out=0;}  
  if (bitRead(LedShannelStatusByte,3) == false){r_out=0;}  
  if (bitRead(LedShannelStatusByte,4) == false){uv_out=0;}   
  if (bitRead(LedShannelStatusByte,5) == false){o_out=0;}  
  if (bitRead(LedShannelStatusByte,6) == false){gr_out=0;}  

   for (byte i=0; i<8; i++){sbX1=(i*35)+21; sbX2=(i*35)+51;    // draw white bar outline rectangle (byte i=0; i<8; i++){sbX1=(i*35)+4; sbX2=(i*35)+34;
	   setFont(SMALL, 255, 255, 255, 0, 0, 0); 
	   myGLCD.drawRect(sbX1, TopSldY-1, sbX2, BotSldY);}
		
   for (byte i=0; i<8; i++){sbX1=(i*35)+21; sbX2=(i*35)+51;   // print Slider Bar current values
                  SliderSwitch  = true;
       
  if (i==0){ sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide= w_out;	 // CW colour(белый)
              setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      wcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} 
    
  if (i==1){ sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide= b_out;	 // BL colour (синий)
             setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      bcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
     
  if (i==2){ sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide = rb_out;	  // RBL colour (рояль)
              setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      rbcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} 
      		
  if (i==3){ sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; tSlide= r_out;	  // DR colour (красный)	
               setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      rcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                
  if (i==4){ sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide= uv_out;    // UV colour (фиолет)
               setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
       uvcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} 
           
  if (i==5){ sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide= o_out;	   // OR colour (оранж)
              setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      ocol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} 
      
  if (i==6){ sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide= gr_out;	   // GR colour (зел)
              setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
      grcol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}

  if (i==7){ sbR=rgbCh8[0]; sbG=rgbCh8[1]; sbB=rgbCh8[2]; tSlide= map (moon_out, 0, 255, 0, 100);  // MOON colour 255
             setFont(SMALL, sbR, sbG, sbB, 0, 0, 0); 
       mooncol_out = SliderBarsForChange2(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} } }

//========================= Preset Screen ==============================
void PresetLedScreen(boolean refreshPresetscreen = false){    //  пресеты
   if (refreshPresetscreen == true){
   myGLCD.setColor(0, 0, 0);
   myGLCD.fillRect(0, 0, 319, 15);			    // clear only header
   setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.setColor(255, 255, 204); // белый
  myGLCD.print(print_text[184], 27, 15);   //  WHT
  myGLCD.setColor(255, 255, 255);   // голубой
  myGLCD.print(print_text[183], 67, 15);   // BLU
  myGLCD.setColor(58, 95, 205);   // рояль 
  myGLCD.print(print_text[182], 107, 15);  // RBL
  myGLCD.setColor(255, 0, 0);     // красный
  myGLCD.print(print_text[181], 147, 15);  // RED
  myGLCD.setColor(224, 102, 255); // фиолетовый
  myGLCD.print(print_text[180], 187, 15);  // UVL 
  myGLCD.setColor(255, 143, 32);  // оранжевый
  myGLCD.print(print_text[179], 227, 15);  // ORG
  myGLCD.setColor(0, 255, 0);     // зеленый 
  myGLCD.print(print_text[178], 267, 15);  // GRN

   TopSldY=53; BotSldY=TopSldY+100;

   for (byte b=0; b<7; b++){        // UP Buttons   
     drawUpButtonSlide((b*40)+4+18, TopSldY-26);} 
   for (byte b=0; b<7; b++){        // DOWN Buttons  
     drawDownButtonSlide((b*40)+4+18, TopSldY+115);}
        temp_sector= min_cnt/15;    // read current time sector only first time

    for (byte i=0; i<7; i++){sbX1=(i*40)+4+18; sbX2=(i*40)+34+18;  // draw white bar outline rectangle 
	      setFont(SMALL, 255, 255, 255, 0, 0, 0); 
		  myGLCD.drawRect(sbX1, TopSldY-1, sbX2, BotSldY);} }
                  SliderSwitch  = true;

         if ((GlobalStatus2Byte & 0xF) ==0 ) {   // no preset 
      		      wcol_out =0;	   // CW colour	(белый)
                      bcol_out =0;	   // BL colour (синий)
                      rbcol_out =0;	   // RBL colour (рояль)
		      rcol_out =0;         // DR colour	(красный)      
		      uvcol_out =0;        // UV colour (фиолет)
		      ocol_out =0;         // OR colour (оранж)
		      grcol_out =0;	   // GR colour (зелен)
		   colorLEDtest = false; } 

   for (byte i=0; i<7; i++){sbX1=(i*40)+4+18; sbX2=(i*40)+34+18;   // print Slider Bar current values 
   
    if (i==0 && bitRead(LedShannelStatusByte,0) == true){ 
	      sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide= wcol_out;	   // CW colour	(белый)
	      wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}

    if (i==1 && bitRead(LedShannelStatusByte,1) == true){ 
	      sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide= bcol_out;	   // BL colour (голубой)
              bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
    if (i==2 && bitRead(LedShannelStatusByte,2) == true){ 
	      sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide= rbcol_out;	   // RBL colour (рояль)
              rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
    if (i==3 && bitRead(LedShannelStatusByte,3) == true){ 
	      sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];  tSlide= rcol_out;		   // DR colour	 (красный)
	      rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); }
           
     if (i==4 && bitRead(LedShannelStatusByte,4) == true){  
	      sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide= uvcol_out;        // UV colour (uv)
              uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
     if (i==5 && bitRead(LedShannelStatusByte,5) == true){  
	      sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide= ocol_out;          // OR colour (oLed)
              ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
      if (i==6 && bitRead(LedShannelStatusByte,6) == true){  
	      sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide= grcol_out;	           // GR colour (green)
              grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} }
              
// if (ledON == 0){  // включение управления каналами при настройке пресетов            
                    LED_levelo_output();   //}        // send calculated value to PWM
         
     if (refreshPresetscreen == true){
                myGLCD.setColor(64, 64, 64);                       // Draw Dividers in Grey
		myGLCD.drawRect(0, 196, 319, 194);                 // Bottom Horizontal Divider
	     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
	     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
	     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);  

 if (bitRead(GlobalStatus2Byte,0)== 1){
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL, GREEN_BAC);} // ON preset
        else {printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);}           // OFF preset
 if (bitRead(GlobalStatus2Byte,1)== 1){
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL, GREEN_BAC);} // ON preset
        else {printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);}           // OFF preset
 if (bitRead(GlobalStatus2Byte,2)== 1){
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL, GREEN_BAC);} // ON preset
	else {printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);}           // OFF preset
 if (bitRead(GlobalStatus2Byte,3)== 1){
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL, GREEN_BAC);} // ON preset
        else {printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);} } }        // OFF preset
		
void SaveAndExit (){     // сохранить настройки пресетов

   myGLCD.setColor(255, 255, 255);                       
   myGLCD.drawRoundRect(5, 53, 315, 153);                 
   myGLCD.setColor(100, 100, 100);
   myGLCD.fillRoundRect(6, 54, 314, 152); 

     myGLCD.setFont(RusFont2);
     myGLCD.setColor(255, 255, 255);
     myGLCD.setBackColor(100, 100, 100);
     //setFont(RUS2, 255, 255, 255, 100, 100, 100);
     myGLCD.print(print_text[185], 40, 73);   // ВЫ ХОТИТЕ СОХРАНИТЬ НАСТРОЙКИ,
     myGLCD.print(print_text[186], 69, 95);   //     ПЕРЕД ВЫХОДОМ ИЗ МЕНЮ ?
     
   printButton(print_text[35], Yes[0], Yes[1], Yes[2], Yes[3], SMALL);  
   printButton(print_text[36], No[0], No[1], No[2], No[3], SMALL); 

//	    DisplayTimeSector(temp_sector);
		w_out = wled[temp_sector];  // read setting from buffer
		b_out = bled[temp_sector];
		rb_out = rbled[temp_sector];
		r_out = rled[temp_sector];
		uv_out = uvled[temp_sector];
		o_out = oLed[temp_sector];
		gr_out = gled[temp_sector];

// check if channel on or off
  if (bitRead(LedShannelStatusByte,0) == false) {w_out=0;}   // white 
  if (bitRead(LedShannelStatusByte,1) == false) {b_out=0;}   // blue 
  if (bitRead(LedShannelStatusByte,2) == false) {rb_out=0;}  // rblue 
  if (bitRead(LedShannelStatusByte,3) == false) {r_out=0;}   // red 
  if (bitRead(LedShannelStatusByte,4) == false) {uv_out=0;}  // uv 
  if (bitRead(LedShannelStatusByte,5) == false) {o_out=0;}   // oLed
  if (bitRead(LedShannelStatusByte,6) == false) {gr_out=0;}} // green 

/*************************** Preset Function ******************************************/
void PresetSwitch(){ // переключение пресетов в главном меню
       calculateStopTime();
	switch ((GlobalStatus2Byte & 0x0F)) {
       case 0:
              myGLCD.setColor(0, 0, 0);
              myGLCD.fillRoundRect (272, 89, 312, 102);
              printPres(print_text[52]);  // П-1
	      GlobalStatus2Byte=(GlobalStatus2Byte & 0xF0 | 0x1);
	      AddressShift = 0; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
    break;
       case 1:
               printPres(print_text[53]);  // П-2
	       GlobalStatus2Byte=(GlobalStatus2Byte & 0xF0 | 0x2);  
	       AddressShift = 9; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
    break;
       case 2:
               printPres(print_text[54]);  // П-3
	       GlobalStatus2Byte=(GlobalStatus2Byte & 0xF0 | 0x4);
	       AddressShift = 18; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
    break;
       case 4:
               printPres(print_text[55]);  // П-4
	       GlobalStatus2Byte=(GlobalStatus2Byte & 0xF0 | 0x8);
	       AddressShift = 27; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
    break;
       case 8:
               printPresoff(print_text[95]); // ВЫКЛ
	       GlobalStatus2Byte=(GlobalStatus2Byte & 0xF0); 
	       colorLEDtest = false;
    break; } }
    

 
// ======================  Скринсейв Аналоговые часы 
 void ATimeSaver(){  
 
// Нарисовать Циферблат 
  myGLCD.setColor(0, 0, 255);   // циферблат синий
  myGLCD.setBackColor(0, 0, 0); // фон черный
  for (int i=0; i<5; i++){ // толщина внешнего синего круга 5 пикс 
  myGLCD.drawCircle(clockCenterX, clockCenterY, 119-i);} // диаметр циферблата 119
  
  myGLCD.setColor(64, 64, 64); // серый
  for (int i=0; i<1; i++){ // толщина внешнего синего круга 1 пикс 
  myGLCD.drawCircle(clockCenterX, clockCenterY, 113-i);} // диаметр циферблата 112
  
  myGLCD.setColor(0, 0, 255); 
  for (int i=0; i<5; i++){ // внутренний синий круг
  myGLCD.drawCircle(clockCenterX, clockCenterY, i);}
  
 // myGLCD.setFont(BigFont);
 // myGLCD.setColor(192, 192, 255);   // цвет цифр серый
  setFont(LARGE, 192, 192, 255, 0, 0, 0);
  myGLCD.print(print_text[49], clockCenterX+92, clockCenterY-8);   // 3  шкала 
  myGLCD.print(print_text[79], clockCenterX-8, clockCenterY+95);   // 6
  myGLCD.print(F("9"), clockCenterX-109, clockCenterY-8);          // 9
  myGLCD.print(print_text[81], clockCenterX-16, clockCenterY-109); // 12
  
  for (int i=0; i<12; i++){ if ((i % 3)!=0) drawMark(i); }  
    
  RTC.getTime();
  drawMin(RTC.minute);
  drawHour(RTC.hour, RTC.minute);
  drawSec(RTC.second);
  oldsec=RTC.second; }

void drawMark(int h){  // часовая шкала 
  float x1, y1, x2, y2; h=h*30; h=h+270;  
  x1=110*cos(h*0.0175);
  y1=110*sin(h*0.0175);
  x2=100*cos(h*0.0175);
  y2=100*sin(h*0.0175); 
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY ); }

void drawSec(int s){       // отображение секунд
  float x1, y1, x2, y2;
  int ps = s-1;
  
  myGLCD.setColor(0, 0, 0); // чёрный цвет
if (ps==-1)ps=59; ps=ps*6; ps=ps+270; // гасим след за стрелкой  
  x1=95*cos(ps*0.0175);
  y1=95*sin(ps*0.0175);
  x2=6*cos(ps*0.0175); 
  y2=6*sin(ps*0.0175); 
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY); 
  myGLCD.setColor(255, 0, 0); s=s*6; s=s+270;  
  x1=95*cos(s*0.0175);
  y1=95*sin(s*0.0175);
  x2=6*cos(s*0.0175); 
  y2=6*sin(s*0.0175);  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY);}

void drawMin(int m){ // Отображение минут
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int pm = m-1;
  
  myGLCD.setColor(0, 0, 0);
  if (pm==-1)pm=59; pm=pm*6; pm=pm+270;
  
  x1=80*cos(pm*0.0175); 
  y1=80*sin(pm*0.0175); 
  x2=6*cos(pm*0.0175); 
  y2=6*sin(pm*0.0175);
  x3=30*cos((pm+4)*0.0175); 
  y3=30*sin((pm+4)*0.0175); 
  x4=30*cos((pm-4)*0.0175); 
  y4=30*sin((pm-4)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);
  
  myGLCD.setColor(0, 255, 0); m=m*6; m=m+270;
  
  x1=80*cos(m*0.0175); 
  y1=80*sin(m*0.0175); 
  x2=6*cos(m*0.0175); 
  y2=6*sin(m*0.0175);
  x3=30*cos((m+4)*0.0175); 
  y3=30*sin((m+4)*0.0175); 
  x4=30*cos((m-4)*0.0175); 
  y4=30*sin((m-4)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);}
  
void drawHour(int h, int m){ // Отображение часов
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int ph = h;
  
  myGLCD.setColor(0, 0, 0);
  if (m==0){ ph=((ph-1)*30)+((m+59)/2); } else { ph=(ph*30)+((m-1)/2); }
  
  ph=ph+270;  
  x1=60*cos(ph*0.0175); 
  y1=60*sin(ph*0.0175); 
  x2=6*cos(ph*0.0175); 
  y2=6*sin(ph*0.0175);
  x3=20*cos((ph+5)*0.0175); 
  y3=20*sin((ph+5)*0.0175); 
  x4=20*cos((ph-5)*0.0175); 
  y4=20*sin((ph-5)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);

  myGLCD.setColor(255, 255, 0); h=(h*30)+(m/2); h=h+270; // часовая стрелка (желтый цвет) 
  x1=60*cos(h*0.0175);
  y1=60*sin(h*0.0175);
  x2=6*cos(h*0.0175);
  y2=6*sin(h*0.0175);
  x3=20*cos((h+5)*0.0175);
  y3=20*sin((h+5)*0.0175);
  x4=20*cos((h-5)*0.0175);
  y4=20*sin((h-5)*0.0175);
  
  myGLCD.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY);
  myGLCD.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY);
  myGLCD.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY);
  myGLCD.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY);}
    
void analogClock(){   // (главный цикл для аналоговых часов) 
    int x, y;  
RTC.getTime();
 
 if (oldsec!=RTC.second){      
 if (RTC.second==0){ drawMin(RTC.minute); drawHour(RTC.hour, RTC.minute); }      
      drawSec(RTC.second); oldsec=RTC.second; } }
      
      
  
/******** AUTOMATIC FEEDER SCREEN ************* dispScreen = 38 **********************/

void autoFeederScreen()
{
    PrintStringIndex=27; 
    printHeader ();
  
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
//  printButtonRUS(print_text[215], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);   // УСТАНОВ.
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА    
  myGLCD.setColor(0, 192, 192);
  myGLCD.drawRect(159, 194, 161, 121);
  myGLCD.drawRoundRect(78, 87, 242, 121);    
  myGLCD.drawRoundRect(80, 89, 240, 119);  
  myGLCD.drawRect(0, 103, 78, 105);
  myGLCD.drawRect(242, 103, 319, 105); 
  myGLCD.drawLine(159, 87, 159, 14);  
  myGLCD.drawLine(161, 87, 161, 14);  
  myGLCD.setColor(0, 0, 0);
  myGLCD.drawLine(160, 195, 160, 193);
  myGLCD.drawLine(160, 122, 160, 120);
  myGLCD.drawLine(77, 104, 79, 104);
  myGLCD.drawLine(241, 104, 243, 104);  
  myGLCD.drawLine(160, 88, 160, 86);
  myGLCD.setColor(153, 0, 102);
  myGLCD.fillRoundRect(85, 94, 235, 114);           //Feed Fish Now Button
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(85, 94, 235, 114); 
  setFont(SMALL, 255, 255, 255, 153, 0, 102);
  myGLCD.setFont(RusFont3);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[180]))); 
  myGLCD.print(buffer, 97, 101);

  if (FEEDTime1==0)                                 //Feeding Time 1 Button 
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(5, 20, 155, 40);
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("1-E", 30, 27);      
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 60, 27);
      setFont(SMALL, 255, 0, 0, 0, 0, 0);     
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182]))); 
      myGLCD.print(buffer, 62, 52);
      myGLCD.setFont(RusFont2); 
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183]))); 
      myGLCD.print(buffer, 24, 65);
}
  else
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(5, 20, 155, 40);
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("1-E", 30, 27);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 60, 27);
      timeDispH=feedFish1H; timeDispM=feedFish1M; 
      if (setTimeFormat==0) { xTimeH=40;}
      if (setTimeFormat==1) { xTimeH=16;}
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=56; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;
      timeCorrectFormat();}
      
      
  if (FEEDTime2==0)                                 //Feeding Time 2 Button
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(165, 20, 315, 40);
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("2-E", 194, 27);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 224, 27);
      setFont(SMALL, 255, 0, 0, 0, 0, 0);     
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182]))); 
      myGLCD.print(buffer, 224, 52);
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183]))); 
      myGLCD.print(buffer, 184, 65);}

  else
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(165, 20, 315, 40);
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("2-E", 194, 27);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 224, 27);
      timeDispH=feedFish2H; timeDispM=feedFish2M; 
      if (setTimeFormat==0) { xTimeH=200;}
      if (setTimeFormat==1) { xTimeH=176;}      
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=56; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}   
      
  if (FEEDTime3==0)                                 //Feeding Time 3 Button
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(5, 168, 155, 188);
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("3-E", 30, 175);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 60, 175);
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182]))); 
      myGLCD.print(buffer, 62, 133);      
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183]))); 
      myGLCD.print(buffer, 24, 146);}
  else
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(5, 168, 155, 188);
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("3-E", 30, 175);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 60, 175);
      timeDispH=feedFish3H; timeDispM=feedFish3M; 
      if (setTimeFormat==0) { xTimeH=40;}
      if (setTimeFormat==1) { xTimeH=16;}
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=137; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}  

      
  if (FEEDTime4==0)                                 //Feeding Time 4 Button
    { myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(165, 168, 315, 188);
      setFont(SMALL, 255, 255, 255, 255, 0, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("4-E", 194, 175);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 224, 175);
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.setFont(RusFont2);     
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182]))); 
      myGLCD.print(buffer, 224, 133);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183]))); 
      myGLCD.print(buffer, 184, 146);}
  else
    { myGLCD.setColor(0, 255, 0);
      myGLCD.fillRoundRect(165, 168, 315, 188);
      setFont(SMALL, 0, 0, 0, 0, 255, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("4-E", 194, 175);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
      myGLCD.print(buffer, 224, 175);
      timeDispH=feedFish4H; timeDispM=feedFish4M;
      if (setTimeFormat==0) { xTimeH=200;}
      if (setTimeFormat==1) { xTimeH=176;}      
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=137; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}      

  myGLCD.setColor(255, 255, 255);
  for (int x=0; x<2; x++)
    { for (int y=0; y<2; y++)
        { myGLCD.drawRoundRect((x*160)+5, (y*148)+20, (x*160)+155, (y*148)+40); }
    }    
}

/******************************************* ВЫХОД АВТОКОРМУШКИ ***************************************************/
void feedingTimeOutput(){
  if ((FEEDTime1==1) && (feedFish1H==RTC.hour) && (feedFish1M==RTC.minute) && (RTC.second>=0 && RTC.second<5)){      
       fiveTillBackOn1=0; FeedWaveCtrl_1=true;
       digitalWrite(autoFeeder, HIGH);}     
       else {
              if ((FEEDTime2==1) && (feedFish2H==RTC.hour) && (feedFish2M==RTC.minute) && (RTC.second>=0 && RTC.second<5)){     
              fiveTillBackOn2=0; FeedWaveCtrl_2=true;      
              digitalWrite(autoFeeder, HIGH);}
              else {
                    if ((FEEDTime3==1) && (feedFish3H==RTC.hour) && (feedFish3M==RTC.minute) && (RTC.second>=0 && RTC.second<5)){     
                    fiveTillBackOn3=0; FeedWaveCtrl_3=true;     
                    digitalWrite(autoFeeder, HIGH);}
                    else {                
                          if ((FEEDTime4==1) && (feedFish4H==RTC.hour) && (feedFish4M==RTC.minute) && (RTC.second>=0 && RTC.second<5)){     
                          fiveTillBackOn4=0; FeedWaveCtrl_4=true;     
                          digitalWrite(autoFeeder, HIGH);}
                          else {digitalWrite(autoFeeder, LOW);}}}}
        
        
       if (FeedWaveCtrl_1==true){
     fiveTillBackOn1++;
      if (fiveTillBackOn1>120)                       //120 is 10 minutes (120/12=10) 
        { FeedWaveCtrl_1=false;}}
        
       if (FeedWaveCtrl_2==true){
     fiveTillBackOn2++;
      if (fiveTillBackOn2>120)                       //120 is 10 minutes (120/12=10) 
        { FeedWaveCtrl_2=false;}}    
      
       if (FeedWaveCtrl_3==true){
     fiveTillBackOn3++;
      if (fiveTillBackOn3>120)                       //120 is 10 minutes (120/12=10) 
        { FeedWaveCtrl_3=false;}}
        
       if (FeedWaveCtrl_4==true){
     fiveTillBackOn4++;
      if (fiveTillBackOn4>120)                       //120 is 10 minutes (120/12=10) 
        { FeedWaveCtrl_4=false;}}
        
}

/*********************** END OF AUTOMATIC FEEDER SETTINGS SCREEN **********************/  


/***** SET AUTOMATIC FEEDER TIMES SCREEN ********** dispScreen = 39 *******************/

void setFeederTimesScreen(boolean refreshAll=true) 
{
  if (feedTime==1)
    { //printHeader("Set Feeding Time 1"); }
      PrintStringIndex=27; 
	  printHeader ();}
  if (feedTime==2)
    { //printHeader("Set Feeding Time 2");}
      PrintStringIndex=27; 
	  printHeader ();}
  if (feedTime==3)
    { //printHeader("Set Feeding Time 3");}
      PrintStringIndex=27; 
	  printHeader ();}
  if (feedTime==4)
    { //printHeader("Set Feeding Time 4");}
      PrintStringIndex=27; 
	  printHeader ();}

  if (refreshAll)
  { 
//   rtcSetMin=RTC.minute; rtcSetHr=RTC.hour; 
   
     if (feedTime==1)
     {rtcSetMin=feedFish1M; rtcSetHr=feedFish1H;}
    if (feedTime==2)
     {rtcSetMin=feedFish2M; rtcSetHr=feedFish2H;}
    if (feedTime==3)
     {rtcSetMin=feedFish3M; rtcSetHr=feedFish3H;}
    if (feedTime==4)
     {rtcSetMin=feedFish4M; rtcSetHr=feedFish4H;}
    
   myGLCD.setColor(64, 64, 64);
   myGLCD.drawRect(0, 196, 319, 194);
   
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА    


   feedingTimeOnOff();

   drawUpButton(houP[0], houP[1]);                //hour up
   drawUpButton(minP[0], minP[1]);                //min up
   drawDownButton(houM[0], houM[1]);              //hour down
   drawDownButton(minM[0], minM[1]);              //min down
   if (setTimeFormat==1)
     { drawUpButton(ampmP[0], ampmP[1]);          //AM/PM up   
       drawDownButton(ampmM[0], ampmM[1]);}       //AM/PM down  
  }    
  
  timeDispH=rtcSetHr; timeDispM=rtcSetMin;
  xTimeH=107; yTime=68;  xColon=xTimeH+42;
  xTimeM10=xTimeH+70; xTimeM1=xTimeH+86; xTimeAMPM=xTimeH+155;
  timeChange();
  
  
}

/********************** END OF SET AUTOMATIC FEEDER TIMES SCREEN **********************/


/***********************УСТАНРВКА УРОВНЯ PH********************************************/
void SetPH(){
  
    PrintStringIndex=9; 
    printHeader ();
    
    
    
    setFont(SMALL, 0, 255, 255, 0, 0, 0);
    myGLCD.setFont(RusFont3);  
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[78]))); 
    myGLCD.print(buffer, 23, 24);    
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122]))); 
    myGLCD.print(buffer, 80, 90);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[79]))); 
    myGLCD.print(buffer, 170, 90);
    
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[80]))); 
    myGLCD.print(buffer, 230, 23);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[81]))); 
    myGLCD.print(buffer, 227, 35);
    
 
    
      myGLCD.drawRoundRect(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
      myGLCD.drawRoundRect(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);
       myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(dos3b[0], dos3b[1], dos3b[2]+50, dos3b[3]);
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawRoundRect(dos3b[0], dos3b[1], dos3b[2]+50, dos3b[3]);  
       myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(dos4b[0], dos4b[1], dos4b[2]+50, dos4b[3]);
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawRoundRect(dos4b[0], dos4b[1], dos4b[2]+50, dos4b[3]);
    setFont(SMALL, 255, 255, 0, 255, 0, 0);
    myGLCD.setFont(RusFont3); 
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122]))); 
    myGLCD.print(buffer, dos3b[0]+6, dos3b[1]+12);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[82]))); 
    myGLCD.print(buffer, dos3b[0]+96, dos3b[1]+12);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[84]))); 
    myGLCD.print(buffer, dos3b[0]+116, dos3b[1]+12);
    
    
    
    
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122]))); 
    myGLCD.print(buffer, dos4b[0]+6, dos4b[1]+12);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[82]))); 
    myGLCD.print(buffer, dos4b[0]+96, dos4b[1]+12);
          strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[85]))); 
    myGLCD.print(buffer, dos4b[0]+116, dos4b[1]+12);  
      
      

    
         SetvalPH = PlusMinusCountF (false, false, dos1b[0], dos1b[1]+20, 150, 5, 9, 0.1, SetvalPH);     
         if (PlsMnsPress == true) { PlsMnsPress = false;}
    
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);
  
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА      
  
  
}
/*******************************ИЗМЕРЕНИЕ PH ****************************************/

unsigned sum(unsigned *buf, unsigned count)
{
  unsigned r = 0;
  for(int i=0; i<count; ++i)
    r += buf[i];
  return r;
}

float avg(unsigned *buf, unsigned count)
{
  return (float)sum(buf, count) / count;
}

float pH(float voltage) {
  return 7 + ((2.5 - voltage) / 0.18);
  //return 3.5*voltage;
}

#define PH_BUF_SIZE 10

float getPHVoltage()
{ 
  static unsigned buf[PH_BUF_SIZE];
  static unsigned char index = 0;
  static unsigned char count = 0;

  buf[index] = analogRead(PH_sensor_ADC);
  if (++index >= PH_BUF_SIZE)
    index = 0;
  if (count < PH_BUF_SIZE)
    ++count;

  return avg(buf, count) *5./1024.; //convert the analog into millivolt
}

void CheckPH(){

#ifndef PH_sensor_ADC
  int x;
  for(x=0;x< sampleSize;x++)
  {
  
  phVolt = getPHVolts();    
  tempAdjusted10 = adjustPHBasedOnTemp(10,calibrationTempC);
  voltsPerPH = abs((volt10-volt7) / (tempAdjusted10-7));
  
  realPHVolt = (volt7 - phVolt);
  phUnits = realPHVolt / voltsPerPH;
  measuredPH = 7 + phUnits;
 
  roomTempC =  getRoomTemperatureC(); 
  roomTempCompensatedMeasuredPH = adjustPHBasedOnTemp(measuredPH,roomTempC);
  
  avgMeasuredPH+=measuredPH;
  avgRoomTemperatureCompensatedMeasuredPH+=roomTempCompensatedMeasuredPH;
  avgRoomTempC+=roomTempC;
  avgPHVolts += phVolt;
    
  
  }
  
  avgMeasuredPH/=sampleSize;
  avgRoomTemperatureCompensatedMeasuredPH/=sampleSize;
  avgRoomTempC/=sampleSize;
  avgPHVolts/=sampleSize;

#else
  avgPHVolts = getPHVoltage() - 0.03;
  avgMeasuredPH = pH(avgPHVolts);
#endif

  Serial.print("avgPHVolts= ");
  Serial.print(avgPHVolts);
  Serial.print(", pH= ");
  Serial.println(avgMeasuredPH);
  
  setFont(SMALL, 0, 255, 255, 0, 0, 0);
  if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 10){
         myGLCD.setFont(BigFont);
         myGLCD.printNumF(avgMeasuredPH,1, 116, 110);
  }
  else{  if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer)
      myGLCD.drawBitmap(128, 108, 24, 24, clos, 1);    // картинка  крестик 
  }
   if (dispScreen==11 && screenSaverCounter<setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 10){
         myGLCD.setFont(BigFont);
         myGLCD.printNumF(avgMeasuredPH,1, 232, 45); 
          myGLCD.printNumF(avgPHVolts,4, 215, 60); 
       myGLCD.printNumF(volt7,4, dosval3[0]+2, dosval3[1]+6);
       myGLCD.printNumF(volt10,4, dosval4[0]+2, dosval4[1]+6);
     
   
 }
  
}


/******** AUTOMATIC DOSER SCREEN / ЭКРАН ДОЗАТОРОВ ************* dispScreen = 36 **********************/                                                                                

void autoDoserScreen()
{
    PrintStringIndex=8; 
    printHeader ();
  
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
  //  printButtonRUS(print_text[215], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);   // УСТАНОВ.
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА 

      myGLCD.drawRoundRect(dosval1[0], dosval1[1], dosval1[2], dosval1[3]);
      myGLCD.drawRoundRect(dosval2[0], dosval2[1], dosval2[2], dosval2[3]);
      myGLCD.drawRoundRect(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
      myGLCD.drawRoundRect(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);
      
      setFont(SMALL, 255, 255, 255, 224, 0, 224);
      printFont();
      myGLCD.printNumI(numDoz1,  dosval1[0]-110, dosval1[1]+5);   
      if ((dozVal1/numDoz1)>99){myGLCD.printNumI(dozVal1/numDoz1,  dosval1[0]-75, dosval1[1]+5);} else
       if ((dozVal1/numDoz1)>9){myGLCD.printNumI(dozVal1/numDoz1,  dosval1[0]-65, dosval1[1]+5);}
                          else {myGLCD.printNumI(dozVal1/numDoz1,  dosval1[0]-55, dosval1[1]+5);}
      myGLCD.printNumI(numDoz2,  dosval2[0]-110, dosval2[1]+5);
      if ((dozVal2/numDoz2)>99){myGLCD.printNumI(dozVal2/numDoz2,  dosval2[0]-75, dosval2[1]+5);} else
       if ((dozVal2/numDoz2)>9){myGLCD.printNumI(dozVal2/numDoz2,  dosval2[0]-65, dosval2[1]+5);}
                          else {myGLCD.printNumI(dozVal2/numDoz2,  dosval2[0]-55, dosval2[1]+5);}
      myGLCD.printNumI(numDoz3,  dosval3[0]-110, dosval3[1]+5);
      if ((dozVal3/numDoz3)>99){myGLCD.printNumI(dozVal3/numDoz3,  dosval3[0]-75, dosval3[1]+5);} else
       if ((dozVal3/numDoz3)>9){myGLCD.printNumI(dozVal3/numDoz3,  dosval3[0]-65, dosval3[1]+5);}
                          else {myGLCD.printNumI(dozVal3/numDoz3,  dosval3[0]-55, dosval3[1]+5);}
       myGLCD.printNumI(numDoz4,  dosval4[0]-110, dosval4[1]+5);
      if ((dozVal4/numDoz4)>99){myGLCD.printNumI(dozVal4/numDoz4,  dosval4[0]-75, dosval4[1]+5);} else
       if ((dozVal4/numDoz4)>9){myGLCD.printNumI(dozVal4/numDoz4,  dosval4[0]-65, dosval4[1]+5);}
                          else {myGLCD.printNumI(dozVal4/numDoz4,  dosval4[0]-55, dosval4[1]+5);}                         
 
            if (dozVal1>99){myGLCD.printNumI(dozVal1,  dosval1[0]+8, dosval1[1]+5);}
                     else if (dozVal1>9&&dozVal1<100){myGLCD.printNumI(dozVal1,  dosval1[0]+24, dosval1[1]+5);}
                     else {myGLCD.printNumI(dozVal1,  dosval1[0]+40, dosval1[1]+5);}
                     
            if (dozVal2>99){myGLCD.printNumI(dozVal2,  dosval2[0]+8, dosval2[1]+5);}
                     else if (dozVal2>9&&dozVal2<100){myGLCD.printNumI(dozVal2,  dosval2[0]+24, dosval2[1]+5);}           
                     else {myGLCD.printNumI(dozVal2,  dosval2[0]+40, dosval2[1]+5);}
                     
            if (dozVal3>99){myGLCD.printNumI(dozVal3,  dosval3[0]+8, dosval3[1]+5);}
                     else if (dozVal3>9&&dozVal3<100){myGLCD.printNumI(dozVal3,  dosval3[0]+24, dosval3[1]+5);}
                     else {myGLCD.printNumI(dozVal3,  dosval3[0]+40, dosval3[1]+5);}
                     
            if (dozVal4>99){myGLCD.printNumI(dozVal4,  dosval4[0]+8, dosval4[1]+5);}
                     else if (dozVal4>9&&dozVal4<100){myGLCD.printNumI(dozVal4,  dosval4[0]+24, dosval4[1]+5);}
                     else {myGLCD.printNumI(dozVal4,  dosval4[0]+40, dosval4[1]+5);}    
      
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[121])));      // ТЕКСТ НА КНОПКАХ (Мл.)
      myGLCD.print(buffer,  dosval1[0]+60, dosval1[1]+16);
      myGLCD.print(buffer,  dosval1[0]-25, dosval1[1]+16);
      myGLCD.print(buffer,  dosval2[0]+60, dosval2[1]+16);
      myGLCD.print(buffer,  dosval2[0]-25, dosval2[1]+16);
      myGLCD.print(buffer,  dosval3[0]+60, dosval3[1]+16);
      myGLCD.print(buffer,  dosval3[0]-25, dosval3[1]+16);
      myGLCD.print(buffer,  dosval4[0]+60, dosval4[1]+16);
      myGLCD.print(buffer,  dosval4[0]-25, dosval4[1]+16);
      myGLCD.setFont(BigFont);
      myGLCD.print("X", dosval1[0]-90, dosval1[1]+8);
      myGLCD.print("X", dosval2[0]-90, dosval2[1]+8);
      myGLCD.print("X", dosval3[0]-90, dosval3[1]+8);
      myGLCD.print("X", dosval4[0]-90, dosval4[1]+8);
      
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawRoundRect(dos1b[0], dos1b[1], dos1b[2], dos1b[3]);
      myGLCD.drawRoundRect(dos2b[0], dos2b[1], dos2b[2], dos2b[3]);
      myGLCD.drawRoundRect(dos3b[0], dos3b[1], dos3b[2], dos3b[3]);
      myGLCD.drawRoundRect(dos4b[0], dos4b[1], dos4b[2], dos4b[3]);
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);
      myGLCD.print("-1", dos1b[0]+68, dos1b[1]-10);      //-1
      myGLCD.print("-2", dos2b[0]+68, dos2b[1]-10);
      myGLCD.print("-3", dos3b[0]+68, dos3b[1]-10); 
      myGLCD.print("-4", dos4b[0]+68, dos4b[1]-10);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123]))); 
      myGLCD.print(buffer,  dos1b[0]+13, dos1b[1]-10);    // дозатор
      myGLCD.print(buffer,  dos2b[0]+13, dos2b[1]-10);
      myGLCD.print(buffer,  dos3b[0]+13, dos3b[1]-10);
      myGLCD.print(buffer,  dos4b[0]+13, dos4b[1]-10);
 
  
  if (DOZTime1==0)                                 //Время первой дозы дозатора 1 
    { setFont(SMALL, 255, 0, 0, 0, 0, 0);     
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
      myGLCD.print(buffer, dos1b[0]+33, dos1b[1]+9);    // выкл
    }
  else
    { timeDispH=dozPump1H; timeDispM=dozPump1M; 
      if (setTimeFormat==0) { xTimeH=dosT1[0]+3;}
      if (setTimeFormat==1) { xTimeH=16;}
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=dos1b[1]+5; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;
      timeCorrectFormat();}
      
      
  if (DOZTime2==0)                                 //Время первой дозы дозатора 2
    { setFont(SMALL, 255, 0, 0, 0, 0, 0);     
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
      myGLCD.print(buffer, dos2b[0]+33, dos2b[1]+9);}

  else
    { 
      timeDispH=dozPump2H; timeDispM=dozPump2M; 
      if (setTimeFormat==0) { xTimeH=dosT2[0]+3;}
      if (setTimeFormat==1) { xTimeH=176;}      
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=dos2b[1]+5; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}   
      
  if (DOZTime3==0)                                 //Время первой дозы дозатора 3
    { setFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.setFont(RusFont2);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
      myGLCD.print(buffer, dos3b[0]+33, dos3b[1]+9);}

  else
    { timeDispH=dozPump3H; timeDispM=dozPump3M; 
      if (setTimeFormat==0) { xTimeH=dosT3[0]+3;}
      if (setTimeFormat==1) { xTimeH=16;}
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=dos3b[1]+5; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}  

      
  if (DOZTime4==0)                                 //Время первой дозы дозатора 4
    { setFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.setFont(RusFont2);     
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184]))); 
      myGLCD.print(buffer, dos4b[0]+33, dos4b[1]+9);}

  else
    { timeDispH=dozPump4H; timeDispM=dozPump4M;
      if (setTimeFormat==0) { xTimeH=dosT4[0]+3;}
      if (setTimeFormat==1) { xTimeH=176;}      
      if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
      else { AM_PM=2;}          
      yTime=dos4b[1]+5; xColon=xTimeH+32;
      xTimeM10=xTimeH+48; xTimeM1=xTimeH+64; xTimeAMPM=xTimeH+96;      
      timeCorrectFormat();}   
      

}
/***********************************КАЛИБРОВКА ДОЗАТОРОВ **************************************************/
void doscalibrateScreen(){
      PrintStringIndex=8; 
      printHeader ();
      myGLCD.setColor(64, 64, 64);
      myGLCD.drawRect(1, 196, 319, 194);
      myGLCD.drawRect(1, 15, 319, 193);
      
      myGLCD.setColor(255, 255, 0);
      myGLCD.drawRoundRect(5,40,315,150);
      myGLCD.drawRoundRect(dos4b[0]+5, dos4b[1]-6, dos4b[2]+52, dos4b[3]-4); //КНОПКА КАЛИБРОВКИ   
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);  
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[120]))); 
      myGLCD.print(buffer,  dos1b[0]+10, dos1b[1]-3);      //время работы дозатора
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[118]))); 
      myGLCD.print(buffer,  dos1b[0]+248, dos1b[1]-3);      //Сек 
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[119]))); 
      myGLCD.print(buffer,  valUP[0]-155, valUP[1]+12);      //объем дозы (мл.)
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[115]))); 
      myGLCD.print(buffer,  numUP[0]-155, numUP[1]+12);      //Количество доз
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[116]))); 
      myGLCD.print(buffer,  intUP[0]-155, intUP[1]+12);      //Интервал (Часы)      
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      
      
      
      
      
      
      myGLCD.setFont(RusFont3);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122]))); 
      myGLCD.print(buffer,  calUP[0]-130, calUP[1]+13);      //КАЛИБРОВКА 





      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);
    //  myGLCD.setFont(RusFont3); 

            if (CalMode==1){myGLCD.print("X", dosval1[0]-105, dosval1[1]-5);
          DosVal1();dosSec1();}      
            if (CalMode==2){myGLCD.print("X", dosval1[0]-105, dosval1[1]-5);
          DosVal2();dosSec2();}
            if (CalMode==3){myGLCD.print("X", dosval1[0]-105, dosval1[1]-5);
          DosVal3();dosSec3();}     
            if (CalMode==4){myGLCD.print("X", dosval1[0]-105, dosval1[1]-5);
          DosVal4();dosSec4();}
      
  
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
    printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);   // СОХРАНИТЬ
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА   
}
      
 /********************************* Экраны калибровки дозаторов*******************************************/ 
 
void dosSec1(){DosSec1=(dozVal1*10/dozCal1);       
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);
      myGLCD.printNumI(numDoz1,  dosval1[0]-120, dosval1[1]-5);      
      if (DosSec1/numDoz1>99){myGLCD.printNumI(DosSec1/numDoz1,  dosval1[0]-89, dosval1[1]-5);}
            else {if (DosSec1/numDoz1>9 && DosSec1/numDoz1<100){myGLCD.printNumI(DosSec1/numDoz1,  dosval1[0]-73, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-90, dosval1[1]-5, dosval1[0]-74, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec1/numDoz1,  dosval1[0]-57, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-73, dosval1[1]-5, dosval1[0]-60, dosval1[1]+12);}}
     
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);         
            if (DosSec1>99){myGLCD.printNumI(DosSec1,  dosval1[0]-29, dosval1[1]-5);}
              else {if (DosSec1>9 && DosSec1<100){myGLCD.printNumI(DosSec1,  dosval1[0]-13, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-30, dosval1[1]-5, dosval1[0]-14, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec1,  dosval1[0]+3, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-13, dosval1[1]-5, dosval1[0], dosval1[1]+12);}}
                      setFont(SMALL, 255, 255, 255, 224, 0, 224);}
                      
void dosSec2(){DosSec2=(dozVal2*10/dozCal2);
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);
      myGLCD.printNumI(numDoz2,  dosval1[0]-120, dosval1[1]-5);      
      if (DosSec2/numDoz2>99){myGLCD.printNumI(DosSec2/numDoz2,  dosval1[0]-89, dosval1[1]-5);}
            else {if (DosSec2/numDoz2>9 && DosSec2/numDoz2<100){myGLCD.printNumI(DosSec2/numDoz2,  dosval1[0]-73, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-90, dosval1[1]-5, dosval1[0]-74, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec2/numDoz2,  dosval1[0]-57, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-73, dosval1[1]-5, dosval1[0]-60, dosval1[1]+12);}}
                      
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont); 
            if (DosSec2>99){myGLCD.printNumI(DosSec2,  dosval1[0]-29, dosval1[1]-5);}
              else {if (DosSec2>9 && DosSec2<100){myGLCD.printNumI(DosSec2,  dosval1[0]-13, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-30, dosval1[1]-5, dosval1[0]-14, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec2,  dosval1[0]+3, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-13, dosval1[1]-5, dosval1[0], dosval1[1]+12);}}
                      setFont(SMALL, 255, 255, 255, 224, 0, 224);}

 void dosSec3(){DosSec3=(dozVal3*10/dozCal3);
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);
      myGLCD.printNumI(numDoz3,  dosval1[0]-120, dosval1[1]-5);      
      if (DosSec3/numDoz3>99){myGLCD.printNumI(DosSec3/numDoz3,  dosval1[0]-89, dosval1[1]-5);}
            else {if (DosSec3/numDoz3>9 && DosSec3/numDoz3<100){myGLCD.printNumI(DosSec3/numDoz3,  dosval1[0]-73, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-90, dosval1[1]-5, dosval1[0]-74, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec3/numDoz3,  dosval1[0]-57, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-73, dosval1[1]-5, dosval1[0]-60, dosval1[1]+12);}}
      
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont); 
            if (DosSec3>99){myGLCD.printNumI(DosSec3,  dosval1[0]-29, dosval1[1]-5);}
              else {if (DosSec3>9 && DosSec3<100){myGLCD.printNumI(DosSec3,  dosval1[0]-13, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-30, dosval1[1]-5, dosval1[0]-14, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec3,  dosval1[0]+3, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-13, dosval1[1]-5, dosval1[0], dosval1[1]+12);}}
                      setFont(SMALL, 255, 255, 255, 224, 0, 224);}

 void dosSec4(){DosSec4=(dozVal4*10/dozCal4);
       setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont);
      myGLCD.printNumI(numDoz4,  dosval1[0]-120, dosval1[1]-5);      
      if (DosSec4/numDoz4>99){myGLCD.printNumI(DosSec4/numDoz4,  dosval1[0]-89, dosval1[1]-5);}
            else {if (DosSec4/numDoz4>9 && DosSec4/numDoz4<100){myGLCD.printNumI(DosSec4/numDoz4,  dosval1[0]-73, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-90, dosval1[1]-5, dosval1[0]-74, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec4/numDoz4,  dosval1[0]-57, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-73, dosval1[1]-5, dosval1[0]-60, dosval1[1]+12);}}
     
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(BigFont); 
            if (DosSec4>99){myGLCD.printNumI(DosSec4,  dosval1[0]-29, dosval1[1]-5);}
              else {if (DosSec4>9 && DosSec4<100){myGLCD.printNumI(DosSec4,  dosval1[0]-13, dosval1[1]-5);              
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect(dosval1[0]-30, dosval1[1]-5, dosval1[0]-14, dosval1[1]+12);}                  
                  else {myGLCD.printNumI(DosSec4,  dosval1[0]+3, dosval1[1]-5);              
                      myGLCD.setColor(0, 0, 0);
                      myGLCD.fillRect(dosval1[0]-13, dosval1[1]-5, dosval1[0], dosval1[1]+12);}}
                      setFont(SMALL, 255, 255, 255, 224, 0, 224);}                      
                    
                      
void DosVal1(){dozVal1 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz1, 999, numDoz1, dozVal1);     // объем дозы 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
         numDoz1 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz1);     // Количество доз 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz1 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz1);     // Интервал между дозами (часы) 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}        
         
          dozCal1 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal1);     // количество за 10 секунд
         if (PlsMnsPress == true) { PlsMnsPress = false;}}
         
                     
void DosVal2() {         
          dozVal2 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz2, 999, numDoz2, dozVal2);     // объем дозы 2 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
                  
         numDoz2 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz2);     // Количество доз 2 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz2 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz2);     // Интервал между дозами (часы) 2 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}  
         
          dozCal2 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal2);     // количество за 10 секунд
         if (PlsMnsPress == true) { PlsMnsPress = false;}} 
         
                    
void DosVal3(){          
          dozVal3 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz3, 999, numDoz3, dozVal3);     // объем дозы 3 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
                  
         numDoz3 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz3);     // Количество доз 3 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz3 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz3);     // Интервал между дозами (часы) 3 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}  
         
          dozCal3 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal3);     // количество за 10 секунд
         if (PlsMnsPress == true) { PlsMnsPress = false;}}
         
                     
void DosVal4(){          
        dozVal4 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz4, 999, numDoz4, dozVal4);     // объем дозы 4 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
                  
         numDoz4 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz4);     // Количество доз 4 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz4 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz4);     // Интервал между дозами (часы) 4 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}  
         
          dozCal4 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal4);     // количество за 10 секунд
         if (PlsMnsPress == true) { PlsMnsPress = false;}} 

/***********************ВЫХОД КАЛИБРОВКИ ПРОИЗВОДИТЕЛЬНОСТИ ДОЗАТОРОВ************************/
void doscalibrate(){
            
            if (CalMode==1){RTC.getTime(); sec1 = ((RTC.minute*60)+RTC.second+10);Doscalibrate1=1;
            digitalWrite(pump1, HIGH);}
         
            if (CalMode==2){RTC.getTime(); sec2 = ((RTC.minute*60)+RTC.second+10);Doscalibrate2=1;
            digitalWrite(pump2, HIGH);}
            
            if (CalMode==3){RTC.getTime(); sec3 = ((RTC.minute*60)+RTC.second+10);Doscalibrate3=1;
            digitalWrite(pump3, HIGH);}
         
            if (CalMode==4){RTC.getTime(); sec4 = ((RTC.minute*60)+RTC.second+10);Doscalibrate4=1;
            digitalWrite(pump4, HIGH);}

}
/*******************************************РАСЧЕТ ВРЕМЕНИ ДОЗ*************************************************/
void caldosetime(){
if (numDoz1 == 4) {shiftH12=(dozPump1H+intDoz1);if (shiftH12>=24) {shiftH12=shiftH12-24;}
                   shiftH13=(dozPump1H+(intDoz1*2));if (shiftH13>=24) {shiftH13=shiftH13-24;}
                   shiftH14=(dozPump1H+(intDoz1*3));if (shiftH14>=24) {shiftH14=shiftH14-24;}}                   
if (numDoz1 == 3) {shiftH12=(dozPump1H+intDoz1);if (shiftH12>=24) {shiftH12=shiftH12-24;}
                   shiftH13=(dozPump1H+(intDoz1*2));if (shiftH13>=24) {shiftH13=shiftH13-24;}
                   shiftH14=25;}
if (numDoz1 == 2) {shiftH12=(dozPump1H+intDoz1);if (shiftH12>=24) {shiftH12=shiftH12-24;}
                   shiftH13=25; shiftH14=25;}
if (numDoz1 == 1) {shiftH12=25; shiftH13=25; shiftH14=25;}

if (numDoz2 == 4) {shiftH22=(dozPump2H+intDoz2);if (shiftH22>=24) {shiftH22=shiftH22-24;}
                   shiftH23=(dozPump2H+(intDoz2*2)); if (shiftH23>=24) {shiftH23=shiftH23-24;}
                   shiftH24=(dozPump2H+(intDoz2*3));if (shiftH24>=24) {shiftH24=shiftH24-24;}}
if (numDoz2 == 3) {shiftH22=(dozPump2H+intDoz2);if (shiftH22>=24) {shiftH22=shiftH22-24;} 
                   shiftH23=(dozPump2H+(intDoz2*2));if (shiftH23>=24) {shiftH23=shiftH23-24;}
                   shiftH24=25;}
if (numDoz2 == 2) {shiftH22=(dozPump2H+intDoz2);if (shiftH22>=24) {shiftH22=shiftH22-24;}
                   shiftH23=25; shiftH14=25;}
if (numDoz2 == 1) {shiftH22=25; shiftH23=25; shiftH24=25;}

if (numDoz3 == 4) {shiftH32=(dozPump3H+intDoz3);  if (shiftH32>=24) {shiftH32=shiftH32-24;}
                   shiftH33=(dozPump3H+(intDoz3*2));if (shiftH33>=24) {shiftH33=shiftH33-24;}
                   shiftH34=(dozPump3H+(intDoz3*3));if (shiftH34>=24) {shiftH34=shiftH34-24;}}
if (numDoz3 == 3) {shiftH32=(dozPump3H+intDoz3);if (shiftH32>=24) {shiftH32=shiftH32-24;}
                   shiftH33=(dozPump3H+(intDoz3*2));if (shiftH33>=24) {shiftH33=shiftH33-24;}
                   shiftH34=25;}
if (numDoz3 == 2) {shiftH32=(dozPump3H+intDoz3);if (shiftH32>=24) {shiftH32=shiftH32-24;}
                   shiftH33=25; shiftH34=25;}
if (numDoz3 == 1) {shiftH32=25; shiftH33=25; shiftH34=25;}

if (numDoz4 == 4) {shiftH42=(dozPump4H+intDoz4);if (shiftH42>=24) {shiftH42=shiftH42-24;}
                   shiftH43=(dozPump4H+(intDoz4*2));if (shiftH43>=24) {shiftH43=shiftH43-24;}
                   shiftH44=(dozPump4H+(intDoz4*3));if (shiftH44>=24) {shiftH44=shiftH44-24;}}
if (numDoz4 == 3) {shiftH42=(dozPump4H+intDoz4);if (shiftH42>=24) {shiftH42=shiftH42-24;}
                   shiftH43=(dozPump4H+(intDoz4*2));if (shiftH43>=24) {shiftH43=shiftH43-24;}
                   shiftH44=25;}
if (numDoz4 == 2) {shiftH42=(dozPump4H+intDoz4);if (shiftH42>=24) {shiftH42=shiftH42-24;}
                   shiftH43=25; shiftH44=25;}
if (numDoz4 == 1) {shiftH42=25; shiftH43=25; shiftH44=25;}
/*
Serial.print (shiftH12); Serial.print ("       ");Serial.print (shiftH13); Serial.print ("       ");Serial.print (shiftH14);
Serial.println ("       ");
Serial.print (shiftH22); Serial.print ("       ");Serial.print (shiftH23);Serial.print ("       "); Serial.print (shiftH24);
Serial.println ("       ");
Serial.print (shiftH32); Serial.print ("       ");Serial.print (shiftH33); Serial.print ("       ");Serial.print (shiftH34);
Serial.println ("       ");
Serial.print (shiftH42); Serial.print ("       ");Serial.print (shiftH43); Serial.print ("       ");Serial.print (shiftH44);
Serial.println ("       ");
*/
}

/******************************************* ВЫХОД ДОЗАТОРА ***************************************************/
void dosingTimeOutput(){
     RTC.getTime();
     sec=(RTC.minute*60)+RTC.second;
     
     
   if ((sec>=sec1)&&(Doscalibrate1==1)){
     Doscalibrate1=0; digitalWrite(pump1, LOW);
     
     if (dispScreen == 10) {
     setFont(SMALL, 255, 255, 0, 0, 0, 0);  
     myGLCD.setFont(RusFont3);  
     strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
     myGLCD.print(buffer,  calUP[0]-150, calUP[1]+13);}}
     
   if ((sec>=sec2)&&(Doscalibrate2==1)){
     Doscalibrate2=0; digitalWrite(pump2, LOW);
     
     if (dispScreen == 10) {
     setFont(SMALL, 255, 255, 0, 0, 0, 0);  
     myGLCD.setFont(RusFont3);  
     strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
     myGLCD.print(buffer,  calUP[0]-150, calUP[1]+13);}}
     
   if ((sec>=sec3)&&(Doscalibrate3==1)){
     Doscalibrate3=0; digitalWrite(pump3, LOW);
     
     if (dispScreen == 10) {
     setFont(SMALL, 255, 255, 0, 0, 0, 0);  
     myGLCD.setFont(RusFont3);  
     strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
     myGLCD.print(buffer,  calUP[0]-150, calUP[1]+13);}}
     
   if ((sec>=sec4)&&(Doscalibrate4==1)){
     Doscalibrate4=0; digitalWrite(pump4, LOW);
     
     if (dispScreen == 10) {
     setFont(SMALL, 255, 255, 0, 0, 0, 0);  
     myGLCD.setFont(RusFont3);  
     strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
     myGLCD.print(buffer,  calUP[0]-150, calUP[1]+13);}}    
  
  if ((DOZTime1==1) && ((dozPump1H==RTC.hour)||(shiftH12==RTC.hour)||( shiftH13==RTC.hour)||(shiftH14==RTC.hour))
  && (dozPump1M==RTC.minute) && (RTC.second>=0 && RTC.second<1))
     {RTC.getTime(); sec1 = ((RTC.minute*60)+RTC.second+(DosSec1/numDoz1));Doscalibrate1=1; // Длительность дозы
            digitalWrite(pump1, HIGH);}   

  if ((DOZTime2==1) && ((dozPump2H==RTC.hour)||(shiftH22==RTC.hour)||( shiftH23==RTC.hour)||(shiftH24==RTC.hour))
  && (dozPump2M==RTC.minute) && (RTC.second>=0 && RTC.second<1))
     {RTC.getTime(); sec2 = ((RTC.minute*60)+RTC.second+(DosSec2/numDoz2));Doscalibrate2=1;
            digitalWrite(pump2, HIGH);}
    
  if ((DOZTime3==1) && ((dozPump3H==RTC.hour)||(shiftH32==RTC.hour)||( shiftH33==RTC.hour)||(shiftH34==RTC.hour))
  && (dozPump3M==RTC.minute) && (RTC.second>=0 && RTC.second<1))
   {RTC.getTime(); sec3 = ((RTC.minute*60)+RTC.second+(DosSec3/numDoz3));Doscalibrate3=1;
            digitalWrite(pump3, HIGH);}

  if ((DOZTime4==1) && ((dozPump4H==RTC.hour)||(shiftH42==RTC.hour)||( shiftH43==RTC.hour)||(shiftH44==RTC.hour))
  && (dozPump4M==RTC.minute) && (RTC.second>=0 && RTC.second<1))
   {RTC.getTime(); sec4 = ((RTC.minute*60)+RTC.second+(DosSec4/numDoz4));Doscalibrate4=1;
            digitalWrite(pump4, HIGH);}   
}

/*********************** END OF AUTOMATIC DOSER SETTINGS SCREEN **********************/  


/***** УСТАНОВКА ВРЕМЕНИ ДОЗАТОРОВ****** SET AUTOMATIC DOSER TIMES SCREEN *****dispScreen = 37 */

void setDoserTimesScreen(boolean refreshAll=true) 
{
  if (dozTime==1)
    { //printHeader("Set Feeding Time 1"); }
      PrintStringIndex=26; 
	  printHeader ();}
  if (dozTime==2)
    { //printHeader("Set Feeding Time 2");}
      PrintStringIndex=26; 
	  printHeader ();}
  if (dozTime==3)
    { //printHeader("Set Feeding Time 3");}
      PrintStringIndex=26; 
	  printHeader ();}
  if (dozTime==4)
    { //printHeader("Set Feeding Time 4");}
      PrintStringIndex=26; 
	  printHeader ();}

  if (refreshAll)
  {    
     if (dozTime==1)
     {rtcSetMin=dozPump1M; rtcSetHr=dozPump1H;}
    if (dozTime==2)
     {rtcSetMin=dozPump2M; rtcSetHr=dozPump2H;}
    if (dozTime==3)
     {rtcSetMin=dozPump3M; rtcSetHr=dozPump3H;}
    if (dozTime==4)
     {rtcSetMin=dozPump4M; rtcSetHr=dozPump4H;}
    
   myGLCD.setColor(64, 64, 64);
   myGLCD.drawRect(0, 196, 319, 194);
   
     printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
     printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
     printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА    


   dosingTimeOnOff();

   drawUpButton(houP[0], houP[1]);                //hour up
   drawUpButton(minP[0], minP[1]);                //min up
   drawDownButton(houM[0], houM[1]);              //hour down
   drawDownButton(minM[0], minM[1]);              //min down
   if (setTimeFormat==1)
     { drawUpButton(ampmP[0], ampmP[1]);          //AM/PM up   
       drawDownButton(ampmM[0], ampmM[1]);}       //AM/PM down  
  }    
  
  timeDispH=rtcSetHr; timeDispM=rtcSetMin;
  xTimeH=107; yTime=68;  xColon=xTimeH+42;
  xTimeM10=xTimeH+70; xTimeM1=xTimeH+86; xTimeAMPM=xTimeH+155;
  timeChange();
  
  
}

/********************** END OF SET AUTOMATIC DOSER TIMES SCREEN **********************/

    
      
      

 /***** ОГРАНИЧЕНИЕ МОЩНОСТИ (РУЧНОЕ ДИММИРОВАНИЕ) **************************** dispScreen = 31 */
void SetDimm() {
  myGLCD.setColor(255, 240, 255);
  myGLCD.fillRect(35, 50, 284, 218);   
    myGLCD.setColor(0, 0, 255);  
    myGLCD.drawRoundRect(35+2, 50+2, 284-2, 218-2); 

    myGLCD.setFont(RusFont1);    
    myGLCD.setColor(0, 0, 0); 
    myGLCD.setBackColor(255, 240, 255);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[143]))); 
    myGLCD.print(buffer, CENTER, 74);         //  величина сигнала всех каналов от 100 %
    myGLCD.print(print_text[142], 225, 130);  // %

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRoundRect(115, 102, 205,167);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRoundRect(115, 102, 205,167);  
  drawUpButton(175, 107);  
  drawDownButton(175, 137);
  
  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  TempsetLEDsDimPercentL=setLEDsDimPercentL;
  if (TempsetLEDsDimPercentL>=10){ myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);}
                            else { myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);}

        myGLCD.setColor(0, 255, 0);
        myGLCD.fillRoundRect(95, 185, 135, 205);
        setFont(SMALL, 0, 0, 0, 0, 255, 0);  
           strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[17]))); 
      myGLCD.print(buffer, 107, 190);       // OK
      myGLCD.setColor(0, 0, 255);
      myGLCD.fillRoundRect(185, 185, 250, 205);    
      setFont(SMALL, 255, 255, 255, 0, 0, 255);
      myGLCD.setFont(RusFont6);  
      myGLCD.print(print_text[1], 185+9, 185+5); // CANCEL 
      
     myGLCD.setColor(0, 0, 0);
     myGLCD.drawRoundRect(95, 185, 135, 205);  
     myGLCD.drawRoundRect(185, 185, 250, 205); } 
  
void LCDbrigh (boolean refreshAll=false){  // Яркость подсветки экрана   
       byte drgb[]={0,0,125};
       
   if (refreshAll) {
    
     tmpLCDbright = LCDbright; 
 
  PrintStringIndex=13; printHeader ();   // НАСТРОЙКА ЯРКОСТИ ЭКРАНА
          
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawLine(157, 19, 157, 221); // верт линия
  myGLCD.drawLine(159, 19, 159, 221); // верт линия
  myGLCD.drawLine(163, 59, 313, 59);
  myGLCD.drawLine(163, 91, 313, 91);
  myGLCD.drawLine(163, 149, 313, 149);
  myGLCD.drawLine(163, 181, 313, 181);
  
  myGLCD.drawRect(165, 61, 311, 89);   // Save
  myGLCD.drawRect(165, 151, 311, 179); // Load
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[130])));     
  myGLCD.print(buffer, 205, 70);              // СОХРАНИТЬ
  myGLCD.print ( print_text[192], 205, 160);  // ЗАГРУЗИТЬ

 
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(200, 200, 200);
  myGLCD.setBackColor(0, 0, 0); 
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[19]))); 
     myGLCD.print(buffer, 165, 28);                // Сохранить Установки
             strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[20]))); 
     myGLCD.print(buffer, 165, 120);               // Загрузить Установки    
 
  printButtonRUS(print_text[1],canC[0],canC[1],canC[2],canC[3]); }  
  printVBar(tmpLCDbright, gseB[0], gseB[1], drgb); }    
      
  
//============================= Очистка памяти ================================== 
void resetScreen(){ // EEPROM Clear Sets all of the bytes of the EEPROM to 0.   
   int i = 0; EEPROM.write(i, 0); 
    EraseAllEEPROM_SaveDefault(); 
    clearScreen(); dispScreen=0; mainScreen(true); }
//===============================================================================

/************************************ TOUCH SCREEN ОБРАБОТКА КАСАНИЙ ЭКРАНА ************************************/
void processMyTouch(){
     myTouch.read(); x=myTouch.getX(); y=myTouch.getY();  
     returnTimer=0; screenSaverCounter=0;    //timegraph=0; // линия графиков
    
  if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])         // press cancel 
     && (dispScreen!=0) && (dispScreen!=4) && (dispScreen!=5) && (dispScreen!=6) 
     && (dispScreen!=8) && (dispScreen!=9) && (dispScreen!=11) && (dispScreen!=13) 
     && (dispScreen!=14) && (dispScreen!=15) && (dispScreen!=28) && (dispScreen!=29)
     && (dispScreen!=32) && (dispScreen!=35)){ waitForIt(canC[0], canC[1], canC[2], canC[3]); 
            
       LEDtestTick = false;     
       ReadFromEEPROM(); dispScreen=0; clearScreen(); mainScreen(true); 
       myGLCD.setColor(64, 64, 64);
       myGLCD.drawRect(0, 226, 319, 239);   // рамка 
       myGLCD.setColor(30, 30, 30);
       myGLCD.fillRect(0, 226, 319, 239);  // Bottom 
       titledate(); } else
    
  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])          // press back    
     && (dispScreen!=0) && (dispScreen!=1) && (dispScreen!=4) && (dispScreen!=5) 
     && (dispScreen!=6) && (dispScreen!=8) && (dispScreen!=9) && (dispScreen!=10)     
     && (dispScreen!=11) && (dispScreen!=13) && (dispScreen!=14) && (dispScreen!=15) 
     && (dispScreen!=16) && (dispScreen!=17) && (dispScreen!=18) && (dispScreen!=20) 
     && (dispScreen!=21) && (dispScreen!=22) && (dispScreen!=23) && (dispScreen!=24) 
     && (dispScreen!=26) && (dispScreen!=27) && (dispScreen!=28) && (dispScreen!=29) 
     && (dispScreen!=30) && (dispScreen!=31) && (dispScreen!=32) && (dispScreen!=33) 
     && (dispScreen!=34) && (dispScreen!=35) && (dispScreen!=37) && (dispScreen!=39)){
          waitForIt(back[0], back[1], back[2], back[3]);
          LEDtestTick = false;         
          dispScreen=1; clearScreen(); menuScreen();      
       myGLCD.setColor(30, 30, 30);
       myGLCD.fillRect(0, 226, 319, 239);   // нижняя область  
                         titledate();       // обновить дату 
        } else { switch (dispScreen) {
                                                                                                              // ************************************************ГЛАВНЫЙ ЭКРАН
  case 0: //------------- MAIN SCREEN (Press Any Key) ---------------
     if (x>1 && x<100 && y>166 && y<222){ dispScreen=25; clearScreen(); graphonoff(); }   // ручное управление таймерами 
     if (x>1 && x<164 && y>92 && y<160){ dispScreen=1; clearScreen(); menuScreen(); }  // главное меню (имена каналов)   
     if (x>170 && x<318 && y>125 && y<225){ dispScreen=1; clearScreen(); menuScreen(); } // главное меню (температура)           
     if (x>274 && x<317 && y>90 && y<105){ PresetSwitch(); _delay_ms(200); }              // присеты    
    // if (x>170 && x<318 && y>125 && y<225){ dispScreen=1; clearScreen(); menuScreen(); } // меню настройки яркости луны
     
     if (x>1 && x<150 && y>5 && y<90){ dispScreen=6; clearScreen(); myGLCD.clrScr();   // cлайдерное управление

         
     wled_out = map(wled_out, 0, 2000, 0, 100);   // new value 0-100%
     bled_out = map(bled_out, 0, 2000, 0, 100);
     rbled_out = map(rbled_out, 0, 2000, 0, 100); 
     rled_out = map(rled_out, 0, 2000, 0, 100);   
     uvled_out = map(uvled_out, 0, 2000, 0, 100);
     oLed_out = map(oLed_out, 0, 2000, 0, 100);  
     gled_out = map(gled_out, 0, 2000, 0, 100);
         yWHT=wled_out; yBLU=bled_out; yRBL=rbled_out; yRED=rled_out ; 
         yUVL=uvled_out; ySMP=oLed_out; yGRN=gled_out;         
     testIndLedScreen(); colorLEDtest = true;  
     myGLCD.setColor(30, 30, 30);
     myGLCD.fillRect(0, 226, 319, 239);  titledate(); }  // Нижний Bar (часы и дата) 
        
  break;     
  case 1: //--------------------- Главное Меню -------------------------

    if ((x>=tanD[0]) && (x<=tanD[2])) {            // Первый столбец   
    if ((y>=tanD[1]) && (y<=tanD[3])) {          // Нажатие утановки времени
        waitForIt(tanD[0], tanD[1], tanD[2], tanD[3]);        
    if ((timeDispH>=0) && (timeDispH<=11))        // Время и дата 
             _delay_ms(80);  clearScreen();
  //        t_temp = rtc.getTime(); t_temp.dow=calcDOW
  //     (rtcSetDy, rtcSetMon, rtcSetYr);  _delay_ms(50); showDOW(t_temp.dow); 
       dispScreen=2;  clockScreen(); } 
    if ((y>=temC[1]) && (y<=temC[3])){            // press Temp Control
          waitForIt(temC[0], temC[1], temC[2], temC[3]);
         dispScreen=3; clearScreen(); tempScreen(true); }          
    if ((y>=feed[1]) && (y<=feed[3])){            // press Feeder
          waitForIt(feed[0], feed[1], feed[2], feed[3]);
         dispScreen=38; clearFscreen(); autoFeederScreen(); }                    
    if ((y>=gSet[1]) && (y<=gSet[3])){             // press General Settings
          waitForIt(gSet[0], gSet[1], gSet[2], gSet[3]);
          dispScreen=14; clearScreen(); generalSettingsScreen_1(); }}
          
          
     if ((x>=tesT[0]) && (x<=tesT[2])) {            // Второй столбец   
    if ((y>=tesT[1]) && (y<=tesT[3])) {          // Автотест 
    waitForIt(tesT[0], tesT[1], tesT[2], tesT[3]);
     dispScreen=5; clearScreen(); testArrayScreen(true); } 
    if ((y>=ledChM[1]) && (y<=ledChM[3])){            // Настройка яркости каналов по цветам
          waitForIt(ledChM[0], ledChM[1], ledChM[2], ledChM[3]);
         dispScreen=7; clearScreen(); ledColorViewScreen(); }          
    if ((y>=Sector[1]) && (y<=Sector[3])){            // Настройка яркости каналов по секторам времени
          waitForIt(Sector[0], Sector[1], Sector[2], Sector[3]);
        dispScreen=4; clearScreen();                   
         testIndLedScreen2(true);          // test and control individual led
           colorLEDtest = true;
       bitClear(GlobalStatus1Byte,3);      // clear bit for " Save changes before exit "
       bitClear(GlobalStatus1Byte,2); }                    
    if ((y>=timday[1]) && (y<=timday[3])){             // Суточные таймеры
          waitForIt(timday[0], timday[1], timday[2], timday[3]);
          dispScreen=19; clearScreen(); TimerScreen(); }} 
                    
      
        if ((x>=tesT2[0]) && (x<=tesT2[2])) {            // Третий столбец   
    if ((y>=tesT2[1]) && (y<=tesT2[3])) {          // Графики каналов 
    waitForIt(tesT2[0], tesT2[1], tesT2[2], tesT2[3]);
    dispScreen=29; clearFscreen(); LEDtestTick= false; AllColourGraph(); } 
    if ((y>=PHset[1]) && (y<=PHset[3])){            // Настройка PH
          waitForIt(PHset[0], PHset[1], PHset[2], PHset[3]);
            dispScreen=11; clearScreen(); SetPH(); }      
    if ((y>=Preset[1]) && (y<=Preset[3])){            // Настройка яркости каналов по цветам
          waitForIt(Preset[0], Preset[1], Preset[2], Preset[3]);
         dispScreen=13; clearScreen();
       PresetLedScreen(true);             // preset led windows
       colorLEDtest = true;  }           
    if ((y>=Pumpset[1]) && (y<=Pumpset[3])){             // Дозатор УДО
          waitForIt(Pumpset[0], Pumpset[1], Pumpset[2], Pumpset[3]);
           dispScreen=36; clearScreen(); autoDoserScreen();}
         } 
      
           if (x>=logW[0] && x<=logW[2] && y>logW[1] && y<logW[3]){ waitForIt(logW[0], logW[1], logW[2], logW[3]);     // график температуры воды за сутки
          dispScreen=28; clearFscreen(); tempgScreen(); timedrawScreen(); }
           
     if (x>=logH[0] && x<=logH[2] && y>logH[1] && y<logH[3]){ waitForIt(logH[0], logH[1], logH[2], logH[3]);   // график температуры радиатора за сутки
          dispScreen=27; clearFscreen(); tFANScreen(); timedrawScreen(); }    
          
        

  
  break;     
  case 2:  //--------------- CLOCK & DATE SETUP SCREEN -----------------
    if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
          waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          RTC.stopClock();// останавливаем часы
           RTC.fillByHMS(rtcSetHr,rtcSetMin,0); // "подкручиваем стрелки 
           RTC.fillByYMD(rtcSetYr,rtcSetMon,rtcSetDy);
           RTC.setTime();// отправляем "подкрученное время" самому модулю
           RTC.startClock(); // и опять запускаем часы
          
        //  rtc.setTime(rtcSetHr, rtcSetMin, 0);
        //  rtc.setDate(rtcSetDy, rtcSetMon, rtcSetYr);
          myGLCD.setColor(30, 30, 30);
          myGLCD.fillRect(0, 226, 319, 239);  // Bottom 
          dispScreen=0; clearScreen(); mainScreen(true); } else {
        
    if ((y>=houU[1]) && (y<=houU[3])){                        // FIRST ROW (TIME UP)   
     if ((x>=houU[0]) && (x<=houU[2])){                       // press hour up
            waitForIt(houU[0], houU[1], houU[2], houU[3]); rtcSetHr++;
          if (rtcSetHr>=24){rtcSetHr=0; }
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
            myGLCD.print(buffer, 23, 21); }    // 28 Changed!
            
      if ((x>=minU[0]) && (x<=minU[2])){                       // press min up
            waitForIt(minU[0], minU[1], minU[2], minU[3]); rtcSetMin++;
          if (rtcSetMin>=60){rtcSetMin = 0; }
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
            myGLCD.print(buffer, 23, 21); } } // Changed!
                        
     if ((y>=houD[1]) && (y<=houD[3])) {                        // SECOND ROW (TIME DOWN) 
        if ((x>=houD[0]) && (x<=houD[2])){                      // press hour down
            waitForIt(houD[0], houD[1], houD[2], houD[3]); rtcSetHr--;
            if (rtcSetHr<0){ rtcSetHr=23; }
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
            myGLCD.print(buffer, 23, 21); }    // Changed!
            
      if ((x>=minD[0]) && (x<=minD[2])){                         // press min down
            waitForIt(minD[0], minD[1], minD[2], minD[3]); rtcSetMin--;
            if (rtcSetMin<0) {rtcSetMin = 59; } 
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
            myGLCD.print(buffer, 23, 21); } }   // Changed!
            
     if ((y>=dayU[1]) && (y<=dayU[3])) {          // THIRD ROW (DATE UP)  DD/MM/YYYY Format               
      if ((x>=dayU[0]) && (x<=dayU[2])){          // press day up
               waitForIt(dayU[0], dayU[1], dayU[2], dayU[3]); rtcSetDy++;
               rtcSetDy=validateDate(rtcSetDy, rtcSetMon, rtcSetYr);   //calendar(); 
               setFont(SMALL, 255, 0, 0, 0, 0, 0);
                   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
               myGLCD.print(buffer, 23, 114); }    // 118 Changed!
               
      if ((x>=monU[0]) && (x<=monU[2])){                // press месяц up  
            waitForIt(monU[0], monU[1], monU[2], monU[3]); rtcSetMon++; // RTC.month++;
           if (rtcSetMon>12) {rtcSetMon=1;}            // месяцы от 1 до 12
           //if (RTC.month>12) {RTC.month=1;}            
               rtcSetDy=validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar(); 
               setFont(SMALL, 255, 0, 0, 0, 0, 0);
                   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
               myGLCD.print(buffer, 23, 114); }     // Changed!
                        
      if ((x>=yeaU[0]) && (x<=yeaU[2])){                // press year up
            waitForIt(yeaU[0], yeaU[1], yeaU[2], yeaU[3]); rtcSetYr++;
            rtcSetDy=validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar(); 
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                 strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
             myGLCD.print(buffer, 23, 114); }      // Changed!           
           
     //       t_temp = rtc.getTime();
      //      t_temp.dow=calcDOW(rtcSetDy, rtcSetMon, rtcSetYr);  // день, месяц, год
       //         showDOW(t_temp.dow);   // показать день недели (из 3-х букв)
     }
    if ((y>=dayD[1]) && (y<=dayD[3])) {          // FOURTH ROW (DATE DOWN)   DD/MM/YYYY Format        
      if ((x>=dayD[0]) && (x<=dayD[2])){         // press day down
               waitForIt(dayD[0], dayD[1], dayD[2], dayD[3]); rtcSetDy--;
               rtcSetDy=validateDate(rtcSetDy, rtcSetMon, rtcSetYr);  //calendar(); 
               setFont(SMALL, 255, 0, 0, 0, 0, 0);
                   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
               myGLCD.print(buffer, 23, 114); }   // Changed!
               
      if ((x>=monD[0]) && (x<=monD[2])){                 // press месяц down
            waitForIt(monD[0], monD[1], monD[2], monD[3]); rtcSetMon--; // RTC.month--;
           if (rtcSetMon<1) {rtcSetMon=12;}              // месяцы от 1 до 12 
            //if (RTC.month<1) {RTC.month=12;} 
            
               rtcSetDy=validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar(); 
               setFont(SMALL, 255, 0, 0, 0, 0, 0);
                   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
               myGLCD.print(buffer, 23, 114); }    // Changed! 
                                    
      if ((x>=yeaD[0]) && (x<=yeaD[2])){                      // press year down
            waitForIt(yeaD[0], yeaD[1], yeaD[2], yeaD[3]); rtcSetYr--;
            rtcSetDy=validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar(); 
            setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170]))); 
            myGLCD.print(buffer, 23, 114); }      // Changed!           
           
      //      t_temp = rtc.getTime();
      //      t_temp.dow=calcDOW(rtcSetDy, rtcSetMon, rtcSetYr); 
      //         showDOW(t_temp.dow); 
    } 
    clockScreen(false); }      
          
  break;
  case 3:  //------------------ TEMPERATURE CONTROL (Вода) ----------------------
  
// Переход в экран лог файла 
    if (x>=13 && x<=37 && y>83 && y<127){ waitForIt(13, 83, 37, 127);  // график температуры воды за сутки
        dispScreen=28; clearFscreen(); tempgScreen(); timedrawScreen(); } //timedraw();}
  
    if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
         
         setTempC = temp2beS;
         offTempC = temp2beO;
         alarmTempC = temp2beA;             
         dispScreen=0; SaveTempToEEPROM(); clearScreen(); mainScreen(true);
    } else {
         
         
     temp2beS = PlusMinusCountF (true, true, temM[0], temM[1], 150, 10, 40, 0.1, temp2beS);     // desired temperature
         if (PlsMnsPress == true) { PlsMnsPress = false;}
     temp2beO = PlusMinusCountF (true, true, temM[0], offM[1], 150, 0.2, 5, 0.1, temp2beO);     // temperature accurancy
         if (PlsMnsPress == true) { PlsMnsPress = false;}
     temp2beA = PlusMinusCountF (true, true, temM[0], SoundAlarmTm[1], 150, 1, 10, 0.1, temp2beA);    // alarm temperature
         if (PlsMnsPress == true) { PlsMnsPress = false;}
    }
         
  break;
  case 4:  //----------------- TEST INDIVIDUAL LED COLOR SCREEN (sector) ------------------------ 
           TopRows == true;   

if ((x>=back[0])&&(x<=back[2])&&(y>=back[1])&&(y<=back[3])&&(bitRead(GlobalStatus1Byte,2)==false)){// press back, exit to "Test option screen"     
       waitForIt(back[0], back[1], back[2], back[3]);
       
   if (bitRead(GlobalStatus1Byte,3) == false){      // bit3 =0, no changes settings      
	dispScreen=1; clearFscreen(); menuScreen(); colorLEDtest = false;
	  LED_levelo_output(); }                    // return to current values led before exit to main      
	  else {colorLEDtest = true;
	 bitSet(GlobalStatus1Byte,2);               // for detect button Yes/No in to "save and exit" windows
	       SaveAndExit(); } } 

 if ((x>=No[0])&&(x<=No[2])&&(y>=No[1])&&(y<=No[3])&&(bitRead(GlobalStatus1Byte,2)==true)){ //press "NO", exit to menu without save,  
       waitForIt(No[0], No[1], No[2], No[3]); LEDtestTick = false; dispScreen=1; clearFscreen(); menuScreen(); 
	    COLOR = 0; ReadLedFromEEPROM();
	     bitClear(GlobalStatus1Byte,3);              // bit 3 - changes setting (0 - no change, 1 -change )
	     bitClear(GlobalStatus1Byte,2);              // clear bit for " Save changes before exit "
	     colorLEDtest = false; LED_levelo_output();} // return to current values led before exit to main
	   
 if ((x>=Yes[0])&&(x<=Yes[2])&&(y>=Yes[1])&&(y<=Yes[3])&&(bitRead(GlobalStatus1Byte,2)==true)){ //press "Yes" for save all changes and exit 
       waitForIt(Yes[0], Yes[1], Yes[2], Yes[3]); LEDtestTick = false; clearFscreen();

	setFont(LARGE, 255, 0, 0, 0, 0, 0);
         myGLCD.print(print_text[175], 72, 80);   // PLEASE WAIT
		COLOR=0 ; SaveLEDToEEPROM();      // save all colour
		bitClear(GlobalStatus1Byte,3);    // clear bit for " Save changes before exit "
		bitClear(GlobalStatus1Byte,2);    // clear bit for " Save changes before exit "
	    dispScreen=4; colorLEDtest = false;
            ReadLedFromEEPROM(); clearFscreen(); LED_levelo_output();       // return to current values led before exit to main
             dispScreen=1; menuScreen(); } 
 
   if ((x>=StopDay[0]) && (x<=StopDay[2]) && (y>=StopDay[1]) && (y<=StopDay[3] && colorLEDtest==true) 
  	 && (bitRead(GlobalStatus1Byte,2) == false)){             // press SUNSET, jump to end "light day"
         waitForIt(StopDay[0], StopDay[1], StopDay[2], StopDay[3]);
	  calculateStopTime();           // calculate SUNSET Time
	   temp_sector = StopTime;
	   testIndLedScreen2(false); }

   if ((x>=StartDay[0]) && (x<=StartDay[2]) && (y>=StartDay[1]) && (y<=StartDay[3] && colorLEDtest==true)
	 && (bitRead(GlobalStatus1Byte,2) == false)){               // press  SUNRISE, jump to start "light day"
        waitForIt(StartDay[0], StartDay[1], StartDay[2], StartDay[3]);
	 calculateStartTime();           // calculate SUNRISE Time
	  temp_sector = StartTime;
	  testIndLedScreen2(false); }  	   

    if ((x>=Nsect[0]) && (x<=Nsect[2]) && (y>=Nsect[1]) && (y<=Nsect[3] && colorLEDtest==true)
	   && (bitRead(GlobalStatus1Byte,2) == false)){          // press NEXT SECTOR, without exit from screen
                waitForIt(Nsect[0], Nsect[1], Nsect[2], Nsect[3]);
	              temp_sector+=1;                            // insrease time sector 

    if (temp_sector>95) {temp_sector=95;} testIndLedScreen2(false); }
     if ((x>=Psect[0]) && (x<=Psect[2]) && (y>=Psect[1]) && (y<=Psect[3] && colorLEDtest==true)
             && (bitRead(GlobalStatus1Byte,2) == false)){          // press  PREV SECTOR, without exit from screen
                 waitForIt(Psect[0], Psect[1], Psect[2], Psect[3]);
	                  temp_sector-=1;                          // desrease time sector

     if (temp_sector<=0) {temp_sector=0;} testIndLedScreen2(false); }  	   

 if ((y>=TopSldY)&&(y<=BotSldY)&&(colorLEDtest==true)&&(bitRead(GlobalStatus1Byte,2)==false)){ // change value with Slider Bars touch   

       for (byte i=0; i<8; i++){    
            if ((x>=(i*35)+21+5) && (x<=(i*35)+51-5)){    // desrease slider touchable area (5pix)
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7;  
//                                            V - ширина слайдера                  
		  sbX1=(i*35)+21; sbX2=(i*35)+51;       // slider width 30 pix, clearens between sliders 5pix 
		  TopSldY=53, BotSldY=TopSldY+100; 
			SliderSwitch  = false;

      if (i==0 && bitRead(LedShannelStatusByte,0) == true){ sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2];    // CW colour	(белый)
	       wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}

       if (i==1 && bitRead(LedShannelStatusByte,1) == true){ sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; 	  // BL colour (синий)
               bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
               
       if (i==2 && bitRead(LedShannelStatusByte,2) == true){ sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; 	  // RBL colour (рояль)
               rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);} 

       if (i==3 && bitRead(LedShannelStatusByte,3) == true){ sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];       // DR colour (красный)	
	        rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); }              
              
       if (i==4 && bitRead(LedShannelStatusByte,4) == true){ sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2];	  // UV colour
                uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                    
       if (i==5 && bitRead(LedShannelStatusByte,5) == true){ sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2];	  // OR colour (оранж)
               ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
       if (i==6 && bitRead(LedShannelStatusByte,6) == true){ sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];	  // GR colour (зелен)
               grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
              
       if (i==7) { sbR=rgbCh8[0]; sbG=rgbCh8[1]; sbB=rgbCh8[2];		    // MOON colour
               mooncol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}      
                        wled[temp_sector] = wcol_out;
			bled[temp_sector] = bcol_out;    // save to buffer before store to eeprom
			rbled[temp_sector] = rbcol_out;
			rled[temp_sector] = rcol_out;
			uvled[temp_sector] = uvcol_out;
			oLed[temp_sector] = ocol_out;
			gled[temp_sector] = grcol_out;   
			  bitSet(GlobalStatus1Byte,3);      // set bit3=1 if setting changed, for " Save changes before exit "
		          LED_levelo_output(); } } } else   // send calculated value to PWM		  
                   
 if ((y>=29+2)&&(y<=44-2)&&(colorLEDtest==true)&&(bitRead(GlobalStatus1Byte,2)==false)){  // UP Buttons were touched, desrease button touchable area (2pix)
      
          for (byte i=0; i<8; i++){   
            if ((x>=(i*35)+21) && (x<=(i*35)+51)){       // slider width 30 pix, clearens 5pix
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7; , 

			sbX1=(i*35)+21; sbX2=(i*35)+51;
			TopSldY=53, BotSldY=TopSldY+100; 
			    SliderSwitch  = true;

    if (i==0 && bitRead(LedShannelStatusByte,0) == true){ 
	     sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide= wcol_out; tSlide+=1; 	 // CW colour (белый)
	     wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		         tSlide= wcol_out; }

    if (i==1 && bitRead(LedShannelStatusByte,1) == true){ 
	     sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide= bcol_out; tSlide+=1;         // BL colour (голубой)
             bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		         tSlide= bcol_out;}

    if (i==2 && bitRead(LedShannelStatusByte,2) == true){ 
	     sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide= rbcol_out; tSlide+=1; 	  // RBL colour (рояль)
             rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		         tSlide= rbcol_out;}

     if (i==3 && bitRead(LedShannelStatusByte,3) == true){ 
	     sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; tSlide= rcol_out; tSlide+=1;          // DR colour (красный)	
	     rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
			   tSlide= rcol_out; } 

     if (i==4 && bitRead(LedShannelStatusByte,4) == true){  
	      sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide= uvcol_out; tSlide+=1;   // UV colour (фиолет)
              uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		         tSlide= uvcol_out;}

     if (i==5 && bitRead(LedShannelStatusByte,5) == true){  
	     sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide= ocol_out; tSlide+=1;  	 // OR colour (оранж)
             ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		          tSlide= ocol_out; }

     if (i==6 && bitRead(LedShannelStatusByte,6) == true){  
	     sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide= grcol_out; tSlide+=1;         // GR colour (зелен)
             grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		         tSlide= grcol_out; }

     if (i==7){ sbR=rgbCh8[0]; sbG=rgbCh8[1]; sbB=rgbCh8[2]; tSlide= mooncol_out; tSlide+=1;  // MOON colour
            mooncol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		          tSlide= mooncol_out; }

                        wled[temp_sector] = wcol_out;   // белый)
			bled[temp_sector] = bcol_out;   // save to buffer before store to eeprom
			rbled[temp_sector] = rbcol_out; // рояль
			rled[temp_sector] = rcol_out;   // красный
			uvled[temp_sector] = uvcol_out;
			oLed[temp_sector] = ocol_out;
			gled[temp_sector] = grcol_out;  // (зелен)
		        LED_levelo_output();             // send calculated value to PWM
			 bitSet(GlobalStatus1Byte,3);    // set bit3=1  if setting changed, for " Save changes before exit "
                        _delay_ms(50); } }               // delay 50msec after touch UP/DOWN button
                         SliderSwitch  = false; } else
                    
if ((y>=174+2)&&(y<=187-2)&&(colorLEDtest==true)&&(bitRead(GlobalStatus1Byte,2)==false)){// DOWN Buttons were touched,desrease button touchable area (2pix)
        
          for (byte i=0; i<8; i++){ if ((x>=(i*35)+21) && (x<=(i*35)+51)){      // slider width 30 pix, clearens 5pix
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7;  

		sbX1=(i*35)+21; sbX2=(i*35)+51;
		TopSldY=53, BotSldY=TopSldY+100; 
		     SliderSwitch  = true;

     if (i==0 && bitRead(LedShannelStatusByte,0) == true){ 
	    sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide = wcol_out; tSlide-=1;  // CW colour (белый)	
	    wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		             tSlide= wcol_out; }

     if (i==1 && bitRead(LedShannelStatusByte,1) == true){ 
	    sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide = bcol_out; tSlide-=1;	// BL colour (синий)
             bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		             tSlide= bcol_out; }

     if (i==2 && bitRead(LedShannelStatusByte,2) == true){ 
	    sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide = rbcol_out; tSlide-=1;    // RBL colour (рояль)
            rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		            tSlide= rbcol_out; } 

     if (i==3 && bitRead(LedShannelStatusByte,3) == true){ 
	    sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; tSlide = rcol_out; tSlide-=1;      // DR colour (красный)	
	     rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		            tSlide = rcol_out ; }

     if (i==4 && bitRead(LedShannelStatusByte,4) == true){
	    sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide = uvcol_out; tSlide-=1;   // UV colour (фиолет)
             uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		             tSlide= uvcol_out; } 

     if (i==5 && bitRead(LedShannelStatusByte,5) == true){
	    sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide = ocol_out; tSlide-=1;	 // OR colour (оранж)
             ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		              tSlide= ocol_out; } 

      if (i==6 && bitRead(LedShannelStatusByte,6) == true){
	      sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide = grcol_out; tSlide-=1;	 // GR colour (зелен)
              grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		              tSlide= grcol_out; }

      if (i==7){ sbR=rgbCh8[0]; sbG=rgbCh8[1]; sbB=rgbCh8[2]; tSlide = mooncol_out; tSlide-=1; // MOON colour
	    mooncol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
			    tSlide= mooncol_out; }
	
			wled[temp_sector] = wcol_out;
                        bled[temp_sector] = bcol_out;    // save to buffer before store to eeprom
			rbled[temp_sector] = rbcol_out;
			rled[temp_sector] = rcol_out;
			uvled[temp_sector] = uvcol_out;
			oLed[temp_sector] = ocol_out;
			gled[temp_sector] = grcol_out; 
		            LED_levelo_output();          // send calculated value to PWM
			  bitSet(GlobalStatus1Byte,3);    // set bit3=1 if setting changed for, " Save changes before exit "
                           _delay_ms(50);                 // delay 50msec after touch UP/DOWN button
                          } } } SliderSwitch  = false;   
  break;        
  case 5:  //---------------- TEST LED ARRAY SCREEN ------------------
    if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])   // Press back    
       && (LEDtestTick==false)){ waitForIt(back[0], back[1], back[2], back[3]);
       LEDtestTick = false; ReadFromEEPROM(); dispScreen=1; clearScreen(); menuScreen(); }
            
   if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])   // Press CANCEL
       && (LEDtestTick==false)){ waitForIt(canC[0], canC[1], canC[2], canC[3]);
       LEDtestTick = false; ReadFromEEPROM(); dispScreen=0; clearScreen(); mainScreen(true); }
        
   if ((x>=stsT[0]) && (x<=stsT[2]) && (y>=stsT[1]) && (y<=stsT[3])){  // Press start / stop test
         waitForIt(stsT[0], stsT[1], stsT[2], stsT[3]);  
           
    if (LEDtestTick) { LEDtestTick = false; testArrayScreen(true);
              } else { LEDtestTick = true; testArrayScreen(); } } else {
           
    if ((x>=tenM[0]) && (x<=tenM[2]) && (y>=tenM[1]) && (y<=tenM[3])){ min_cnt -= 10; // Press -10s              
    if (min_cnt<0) { min_cnt= 0; } _delay_ms(10); } 
    if ((x>=tenP[0]) && (x<=tenP[2]) && (y>=tenP[1]) && (y<=tenP[2])){ min_cnt += 10; // Press +10s                  
    if (min_cnt>1440) { min_cnt = 1440; } _delay_ms(10); } } 
     
  break;        
  case 6: // TEST INDIVIDUAL LED SCREEN -------------------------- Слайдерное ручное управление   
    int CL_check, CL_check2;
           
    if ((x>=8) && (x<=33) && (y>=190) && (y<=214)){     // press back    
         waitForIt(8, 190, 33, 214);                    // возврат в главный экран (крестик)
         wled_out = map(wcol_out, 0, 2000, 0, 100);      // new value 0-100%
         bled_out = map(bcol_out, 0, 2000, 0, 100);
         rbled_out = map(rbcol_out, 0, 2000, 0, 100); 
         rled_out = map(rcol_out, 0, 2000, 0, 100);   
         uvled_out = map(uvcol_out, 0, 2000, 0, 100);
         oLed_out = map(oLed_out, 0, 2000, 0, 100);  
         gled_out = map(gled_out, 0, 2000, 0, 100);	
         clearScreen();
         LEDtestTick = false; dispScreen=0; mainScreen(true); colorLEDtest = false;
         myGLCD.setColor(30, 30, 30);
         myGLCD.fillRect(0, 226, 319, 239); }    // Нижний Bar (часы и дата) 

    if ((y>=44)&&(y<=172)){ for (int i=0; i<7; i++) {          // Slider Bars    
         if ((x>=(i*38)+49) && (x<=(i*38)+79)) {                                           
    if (i==0){sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; sbX1=49; sbX2=79; SliderBars();}   // WHITE LED Slider  
    if (i==1){sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; sbX1=87; sbX2=117; SliderBars();}    // BLUE LED Slider   
    if (i==2){sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; sbX1=125; sbX2=155; SliderBars();}   // R. BLUE LED Slider
    if (i==3){sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; sbX1=163; sbX2=193; SliderBars();}     // RED LED Slider    
    if (i==4){sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; sbX1=201; sbX2=231; SliderBars();} // UV LED Slider     
    if (i==5){sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; sbX1=239; sbX2=269; SliderBars();}  // ORANGE LED Slider 
    if (i==6){sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; sbX1=277; sbX2=307; SliderBars();}}}}  // Green LED Slider    

    if ((y>=17) && (y<=39)){ for (int i=0; i<7; i++) {        //Up buttons were touched  
         if ((x>=(i*38)+49) && (x<=(i*38)+79)){      
    if (i==0){sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; sbX1=49; sbX2=79; yWHT+=1; UpDnButtonSlide();}   // WHITE + 
    if (i==1){sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; sbX1=87; sbX2=117; yBLU+=1; UpDnButtonSlide();}    // BLUE + 
    if (i==2){sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; sbX1=125; sbX2=155; yRBL+=1; UpDnButtonSlide();}   // ROYAL BLUE +
    if (i==3){sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];sbX1=163; sbX2=193; yRED+=1; UpDnButtonSlide();}      // RED +
    if (i==4){sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; sbX1=201; sbX2=231; yUVL+=1; UpDnButtonSlide();} // UV +
    if (i==5){sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; sbX1=239; sbX2=269; ySMP+=1; UpDnButtonSlide();}  // ORANGE +
    if (i==6){sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];sbX1=277; sbX2=307; yGRN+=1; UpDnButtonSlide();}}}}   // Green + 
     
     if ((y>=200) && (y<=222)){ for (int i=0; i<7; i++) {      //Down buttons were touched 
          if ((x>=(i*38)+49) && (x<=(i*38)+79)) {    
     if (i==0){sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2];sbX1=49; sbX2=79; yWHT-=1; UpDnButtonSlide();}   // WHITE -  
     if (i==1){sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2];sbX1=87; sbX2=117; yBLU-=1; UpDnButtonSlide();}    // BLUE - 
     if (i==2){sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2];sbX1=125; sbX2=155; yRBL-=1; UpDnButtonSlide();}   // ROYAL BLUE -
     if (i==3){sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];sbX1=163; sbX2=193; yRED-=1; UpDnButtonSlide();}     // RED -
     if (i==4){sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2];sbX1=201; sbX2=231; yUVL-=1; UpDnButtonSlide();} // UV - 
     if (i==5){sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2];sbX1=239; sbX2=269; ySMP-=1; UpDnButtonSlide();}  // ORANGE -    
     if (i==6){sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];sbX1=277; sbX2=307; yGRN-=1; UpDnButtonSlide();}}}}  // GREEN -        
               
  break;   
  case 7:  // ----------- VIEW INDIVIDUAL LED COLORS SCREEN -----------    
    if ((x>=10) && (x<=150)) {         // first column   
       if ((y>=20) && (y<=50)){ waitForIt(10, 20, 150, 50);               // View White LEDs Array
           dispScreen=8; COLOR = WHITE; clearScreen(); ledValuesScreen(); } 
           
       if ((y>=60) && (y<=90)){ waitForIt(10, 60, 150, 90);               // View Royal Blue LEDs Array  
           dispScreen=8; COLOR = ROYAL; clearScreen(); ledValuesScreen(); }  
           
       if ((y>=100) && (y<=130)){ waitForIt(10, 100, 150, 130);           // View Red LEDs Array 
           dispScreen=8; COLOR = RED; clearScreen(); ledValuesScreen(); } 
           
       if ((x>=10) && (x<=150) && (y>=140) && (y<=170)){ waitForIt(10, 140, 150, 170); // View GREEN LEDs Array   
           dispScreen=8; COLOR = GREEN; clearScreen(); ledValuesScreen(); } }
      
    if ((x>=170) && (x<=310)) {      // second column
       if ((y>=20) && (y<=50)){ waitForIt(170, 20, 310, 50);               // View Blue LEDs Array
           dispScreen=8; COLOR = BLUE; clearScreen(); ledValuesScreen(); } 
           
       if ((y>=60) && (y<=90)){ waitForIt(170, 60, 310, 90);               // View Ultra LEDs Array
           dispScreen=8; COLOR = ULTRA; clearScreen(); ledValuesScreen(); } 
           
       if ((y>=100) && (y<=130)){ waitForIt(170, 100, 310, 130);           // View ORANGE LEDs Array   
           dispScreen=8; COLOR = ORANGE; clearScreen(); ledValuesScreen(); } }  
                       
    if ((x>=170) && (x<=310) && (y>=140) && (y<=170)){ waitForIt(170, 140, 310, 170); // View Moon LEDs MaxI   
         ReadLedFromEEPROM(); dispScreen=26; COLOR = MOON; clearScreen(); ledValuesScreen(); }
                          
  break;       
  case 8:  // SHOW LED VALUES TABLE SCREEN // переключение между экранами график / таблица
 
   if ((x>=5) && (x<=315) && (y>=50) && (y<=180)){      // переход к представлению в виде графиков  
        dispScreen=34; clearScreen(); ledValuegScreen(); } else
 
  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   // press MORE COLORS
        waitForIt(back[0], back[1], back[2], back[3]);
        ReadLedFromEEPROM(); dispScreen=7; clearScreen(); ledColorViewScreen(); } else
     
  if ((x>=ledChV[0]) && (x<=ledChV[2]) && (y>=ledChV[1]) && (y<=ledChV[3])){  // press CHANGE 
        waitForIt(ledChV[0], ledChV[1], ledChV[2], ledChV[3]);
        ReadLedFromEEPROM(); dispScreen=9; clearScreen(); ledChangeScreen(); } else
      
  if ((x>=eeprom[0]) && (x<=eeprom[2]) && (y>=eeprom[1]) && (y<=eeprom[3])){  // press SAVE 
        waitForIt(eeprom[0], eeprom[1], eeprom[2], eeprom[3]);
        SaveLEDToEEPROM(); dispScreen=7; clearScreen(); ledColorViewScreen(); } else 
             
   if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){         // press CANCEL
        waitForIt(canC[0], canC[1], canC[2], canC[3]);
        ReadFromEEPROM(); LEDtestTick = false; dispScreen=0; clearScreen(); mainScreen(true); }  
    
  break;
  case 9:  // CHANGE LED VALUES SCREEN ---- Идивидуальныая настройка яркости LED 0 - 100 %
 
    if ((x>=8) && (x<=26) && (y>=76) && (y<=130)){ // press OK, exit from "chande led output value" to "LED output table" menu 
         waitForIt(8, 76, 26, 130); dispScreen=8;

       if (COLOR==1) { for (int i; i<96; i++) { wled[i]=tled[i]; } } // чтение установок яркости белых LED
       if (COLOR==2) { for (int i; i<96; i++) { bled[i]=tled[i]; } }  
       if (COLOR==3) { for (int i; i<96; i++) { rbled[i]=tled[i]; } } 
       if (COLOR==4) { for (int i; i<96; i++) { rled[i]=tled[i]; } }  
       if (COLOR==5) { for (int i; i<96; i++) { uvled[i]=tled[i]; } } 
       if (COLOR==6) { for (int i; i<96; i++) { oLed[i]=tled[i]; } } 
       if (COLOR==7) { for (int i; i<96; i++) { gled[i]=tled[i]; } } // green
         
         DrawStaticElement = false;
		 oldLCT = 0;
	 myGLCD.setColor(64, 64, 64);		       // fill with background bottom time|date bar
         myGLCD.fillRect(1, 226, 318, 240);
	  clearScreen(); ledValuesScreen();	       // print TOP NAME bar & horisontal buttons, with LED GRAPH
		// TimeDateBar(true); 
                   } else
      
   if ((x>=8) && (x<=26) && (y>=145) && (y<=219)){    // press CANCEL, exit to "main display" 
       waitForIt(8, 145, 26, 219);
      
       DrawStaticElement = false;
       oldLCT = 0; COLOR = 0; ReadLedFromEEPROM(); dispScreen=0;
        myGLCD.setColor(64, 64, 64);		      // fill with background bottom time|date bar
        myGLCD.fillRect(1, 226, 318, 240);
        clearScreen(); TimeDateBar(true); mainScreen(true); } else
     
    if ((y>=15) && (y<=41)){                         // top row with times was touched
        if ((x>=4) && (x<=316)){
             if (DrawStaticElement==false){
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(50, 110, 318, 146);
                for (byte b=0; b<8; b++){ drawUpButtonSlide((b*36)+34, 45);}    // draw UP Buttons
                for (byte b=0; b<8; b++){ drawDownButtonSlide((b*36)+34, 200);} // draw DOWN Buttons
		     TopSldY=71; BotSldY=185;

		 for (byte i=0; i<8; i++){                      // draw white bar outline rectangle
                sbX1=(i*36)+34; sbX2=(i*36)+64;
	        setFont(SMALL, 255, 255, 255, 0, 0, 0); 
	        myGLCD.drawRect(sbX1, TopSldY-1, sbX2, BotSldY);}
	        myGLCD.setColor(140, 140, 140);                  // print vertical divider 
	        myGLCD.fillRect(174, 42, 176, 240); }

// # oleg change function 00 time ( LedChangTime shifted +1 )
		      DrawStaticElement=true;
	              LedChangTime = map(x, 3, 320, 1, 13);

          if ( oldLCT != LedChangTime){	     // highlight and refresh touched time only first touch current time  
              if (oldLCT !=0){
                  myGLCD.setColor(0, 0, 0);
                  myGLCD.fillRect((oldLCT-1)*26+5, 17, (oldLCT-1)*26+29, 41);
                  myGLCD.setColor(100, 100, 100);
                  myGLCD.drawRect((oldLCT-1)*26+4, 16 , (oldLCT-1)*26+30, 41);
                  setFont(SMALL, 0, 255, 255, 0, 0, 0);
		      
          if ((oldLCT-1)<5){
                  myGLCD.printNumI(((oldLCT-1)*2), ((oldLCT-1)*26)+15, 17);
                  myGLCD.printNumI((((oldLCT-1)*2)+1), ((oldLCT-1)*26)+15, 28);}
                else {
                  myGLCD.printNumI(((oldLCT-1)*2), ((oldLCT-1)*26)+11, 17);
                  myGLCD.printNumI((((oldLCT-1)*2)+1), ((oldLCT-1)*26)+11, 28);} }

                  myGLCD.setColor(255, 0, 0);   // red
          if ((LedChangTime-1)<5) { // выбор двухчасового промежутка
                  myGLCD.fillRect(((LedChangTime-1)*26)+5, 17, ((LedChangTime-1)*26)+29, 41);
                  setFont(SMALL, 255, 255, 255, 255, 0, 0); // белый шрифт на красном фоне
		  myGLCD.printNumI(((LedChangTime-1)*2), ((LedChangTime-1)*26)+15, 17);
                  myGLCD.printNumI((((LedChangTime-1)*2)+1), ((LedChangTime-1)*26)+15, 28);}
                else {
                  myGLCD.fillRect(((LedChangTime-1)*26)+5, 17, ((LedChangTime-1)*26)+29, 41);
                  setFont(SMALL, 255, 255, 255, 255, 0, 0);
		  myGLCD.printNumI(((LedChangTime-1)*2), ((LedChangTime-1)*26)+11, 17);
                  myGLCD.printNumI((((LedChangTime-1)*2)+1), ((LedChangTime-1)*26)+11, 28);}
                  oldLCT= LedChangTime; 				

              for (int i=0; i<8; i++){		    // print led values & slider value bars for highlighted time        
                   int k=((LedChangTime-1)*8)+i;
//                  waitForTouchRelease();	    // #oleg this line prevent "parasitic" slider jump
	   sbX1=(i*36)+34; sbX2=(i*36)+64; TopSldY=71; BotSldY=185;
		    SliderSwitch  = true;
		    tSlide=tled[k];
	 SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); } } } } else
      
   if ((y>=TopSldY)&&(y<=BotSldY) && (DrawStaticElement==true)){    // Slider Bars touch
//       waitForTouchRelease();		 //#oleg this line prevent "parasitic" slider jump  

       for (int i=0; i<8; i++){
          if ((x>=(i*36)+34+5) && (x<=(i*36)+64-5)){      // decrease on 5pix slider touch area
             int k=((LedChangTime-1)*8)+i;
            sbX1=(i*36)+34; sbX2=(i*36)+64; TopSldY=71; BotSldY=185;
		  SliderSwitch  = false;   
  	 tled[k]= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); } } } else
       
    if ((y>=45) && (y<=67) && (DrawStaticElement==true)){           // UP Buttons were touched
        for (int i=0; i<8; i++){ if ((x>=(i*36)+34) && (x<=(i*36)+64)){
              int k= ((LedChangTime-1)*8)+i;
	 TopSldY=71; BotSldY=185; sbX1=(i*36)+34; sbX2=(i*36)+64;
	     SliderSwitch  = true; 
     	  tSlide = tled[k]; tSlide+=1; 
	  tled[k] = SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
		 _delay_ms(100); } } } else // delay after touch UP/DOWN button
      
    if ((y>=200) && (y<=222) && (DrawStaticElement==true)){          // DOWN Buttons were touched 
        for (int i=0; i<8; i++){ if ((x>=(i*36)+34) && (x<=(i*36)+64)){
              int k= ((LedChangTime-1)*8)+i;
	 TopSldY=71; BotSldY=185; sbX1=(i*36)+34; sbX2=(i*36)+64;
	     SliderSwitch  = true; 
     	 tSlide = tled[k]; tSlide-=1; 
	 tled[k] = SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
	       _delay_ms(100); } } } // delay after touch UP/DOWN button
                       
    break;   
    case 10://-----------------КАЛИБРОВКА УДО-----------------------   
         
    if (CalMode==1){     
          dozVal1 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz1, 999, numDoz1, dozVal1);     // доза
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          numDoz1 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz1);     // Количество доз 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz1 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz1);     // Интервал между дозами (часы) 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}        

          dozCal1 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal1);     // калибровка 10 сек
         if (PlsMnsPress == true) { PlsMnsPress = false;}dosSec1();}
          
    if (CalMode==2){     
          dozVal2 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz2, 999, numDoz2, dozVal2);     // доза
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          numDoz2 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz2);     // Количество доз 2 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz2 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz2);     // Интервал между дозами (часы) 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}         
                  
          dozCal2 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal2);     // калибровка 10 сек
         if (PlsMnsPress == true) { PlsMnsPress = false;}dosSec2();}     
          
    if (CalMode==3){     
          dozVal3 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz3, 999, numDoz3, dozVal3);     // доза
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          numDoz3 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz3);     // Количество доз 3 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz3 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz3);     // Интервал между дозами (часы) 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}         
         
          dozCal3 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal3);     // калибровка 10 сек
         if (PlsMnsPress == true) { PlsMnsPress = false;}dosSec3();}
          
    if (CalMode==4){     
          dozVal4 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz4, 999, numDoz4, dozVal4);     // доза
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          numDoz4 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz4);     // Количество доз 4 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}
         
          intDoz4 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz4);     // Интервал между дозами (часы) 1 дозатора
         if (PlsMnsPress == true) { PlsMnsPress = false;}         
         
          dozCal4 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal4);     // калибровка 10 сек
         if (PlsMnsPress == true) { PlsMnsPress = false;}dosSec4();}     

         if ((x>=dos4b[0]+5)&&(x<=dos4b[2]+52)&&(y>=dos4b[1]-6)&&(y<=dos4b[3]-4)){
          waitForIt(dos4b[0]+5, dos4b[1]-6, dos4b[2]+52, dos4b[3]-4);
          doscalibrate();
}
      
    
    if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   // Press back      
          waitForIt(back[0], back[1], back[2], back[3]);
          dispScreen=36; clearScreen(); autoDoserScreen(); } 

    if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          SaveDoseTimesToEEPROM();
          dispScreen=36; clearScreen(); autoDoserScreen(); }
                 
    if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){   // Press CANCEL 
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
       dispScreen=0; clearScreen(); mainScreen(true); } 
                                     
    break;
    case 11: //-----------------ДОЗАТОР СО2 / УСТАНОВКА И КАЛИБРОВКА РН-----------------------
    
        
          SetvalPH = PlusMinusCountF (true, true, dos1b[0], dos1b[1]+20, 150, 5, 9, 0.1, SetvalPH);     // 
            if (PlsMnsPress == true) { PlsMnsPress = false;}
         
            if ((x>=dos3b[0]) && (x<=dos3b[2]+50) && (y>=dos3b[1]) && (y<=dos3b[3])){   // калибровка РН 7     
          waitForIt(dos3b[0], dos3b[1], dos3b[2]+50, dos3b[3]);
          volt7 = avgPHVolts;                      // Напряжения калибровки датчика PH 7
          dispScreen=11; clearScreen(); SetPH(); }
          
            if ((x>=dos4b[0]) && (x<=dos4b[2]+50) && (y>=dos4b[1]) && (y<=dos4b[3])){   // калибровка РН 10    
          waitForIt(dos4b[0], dos4b[1], dos4b[2]+50, dos4b[3]);
          volt10 = avgPHVolts;                      // Напряжения калибровки датчика PH 10
          dispScreen=11; clearScreen(); SetPH(); }
          

    
    
    
    if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   // Press back      
          waitForIt(back[0], back[1], back[2], back[3]);
          dispScreen=1; clearScreen(); menuScreen(); } 

    if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          SavePHsetToEEPROM();
          dispScreen=0; clearScreen(); mainScreen(true); }
                 
    if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){   // Press CANCEL 
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
       dispScreen=0; clearScreen(); mainScreen(true); } 
    
    break;
    case 12:  //------------------ (РЕЗЕРВ)------------------

    break;
    case 13:  // LED Preset (запись пресетов) ==================================================================

 //  if ((x>=298) && (x<=317) && (y>=51) && (y<=110)){ waitForIt(298, 51, 317, 110);        
   //          ledON = 1; } else { ledON = 0; } 

   if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])    // press back    
        && (LEDtestTick==false)){ waitForIt(back[0], back[1], back[2], back[3]);
         LEDtestTick = false; dispScreen=1; clearFscreen(); menuScreen(); 
         GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0); colorLEDtest = false; } // отключить присеты
                  
    if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])   // press CANCEL
         && (LEDtestTick==false)){
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
         LEDtestTick = false; dispScreen=0; clearFscreen(); mainScreen(true); 
         GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0); colorLEDtest = false; } // отключить присеты
            
   if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
        waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
	   if (bitRead(GlobalStatus2Byte,0)== 1) {AddressShift=0;}
	   if (bitRead(GlobalStatus2Byte,1)== 1) {AddressShift=9;}
	   if (bitRead(GlobalStatus2Byte,2)== 1) {AddressShift=18;}
	   if (bitRead(GlobalStatus2Byte,3)== 1) {AddressShift=27;}
       if ((GlobalStatus2Byte & 0xF) !=0 ) {SaveLEDPresetToEEPROM();} }

    if ((x>=LedPres1[0]) && (x<=LedPres1[2]) && (y>=LedPres1[1]) && (y<=LedPres1[3])  // press Preset N1
       && (LEDtestTick==false)){ waitForIt(LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3]);
       if (bitRead(GlobalStatus2Byte,0)== 0){
		  printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL, GREEN_BAC); // ON preset 1 
		  printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);  
		  printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);  
		  printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
		  GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x1); // set flag Preset1, clear 2..4  
		  colorLEDtest = true; }

	    else {printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL); // OFF preset
		  colorLEDtest = false;
		  bitClear(GlobalStatus2Byte,0);} // clear flag Preset1
		  AddressShift = 0;
		  ReadLEDPresetFromEEPROM();
		  PresetLedScreen(false); }       // preset led windows
	  	  
     if ((x>=LedPres2[0]) && (x<=LedPres2[2]) && (y>=LedPres2[1]) && (y<=LedPres2[3])   // press Preset N2
          && (LEDtestTick==false)){ waitForIt(LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3]);
		  if (bitRead(GlobalStatus2Byte,1)== 0){
		  printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
		  printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL, GREEN_BAC); // ON preset 2   
		  printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);  
		  printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
  		  GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x2); // set flag Preset2, clear 1,3,4  
		  colorLEDtest = true; }

	      else {printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL); // OFF preset
		  colorLEDtest = false;
		  bitClear(GlobalStatus2Byte,1);} // clear flag Preset2
		  AddressShift = 9;
		  ReadLEDPresetFromEEPROM(); 
		  PresetLedScreen(false); }        // preset led windows
	  
     if ((x>=LedPres3[0]) && (x<=LedPres3[2]) && (y>=LedPres3[1]) && (y<=LedPres3[3])   // press Preset N3
         && (LEDtestTick==false)){ waitForIt(LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3]);
		  if (bitRead(GlobalStatus2Byte,2)== 0){
		  printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
		  printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);  
		  printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL, GREEN_BAC); // ON preset 3   
		  printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
		  GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0| 0x4); // set flag Preset3, clear 1,2,4  
		  colorLEDtest = true; }

	    else {printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL); // OFF preset
		  colorLEDtest = false;
		  bitClear(GlobalStatus2Byte,2);} // clear flag Preset3
		  AddressShift = 18;
		  ReadLEDPresetFromEEPROM(); 
		  PresetLedScreen(false); }       // preset led windows
	  
     if ((x>=LedPres4[0]) && (x<=LedPres4[2]) && (y>=LedPres4[1]) && (y<=LedPres4[3])   // press Preset N4
          && (LEDtestTick==false)){ waitForIt(LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3]);
		  if (bitRead(GlobalStatus2Byte,3)== 0){
		  printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
		  printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);  
		  printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);  
		  printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL, GREEN_BAC); // ON preset 4 
		  GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x8);   // set flag Preset4, clear 1..3  
		  colorLEDtest = true; }

	    else {printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL); // OFF preset
		  colorLEDtest = false;
		  bitClear(GlobalStatus2Byte,3);}  // clear flag Preset4
		  AddressShift = 27;
		  ReadLEDPresetFromEEPROM(); 
		  PresetLedScreen(false); }        // preset led windows
	  
     if ((y>=TopSldY)&&(y<=BotSldY) && (colorLEDtest==true)){  // change value with Slider Bars touch
	     TopSldY=53, BotSldY=TopSldY+100; 
	     SliderSwitch  = false;

      for (byte i=0; i<7; i++){ sbX1=(i*40)+4+18; sbX2=(i*40)+34+18;
           if (x>=sbX1+5 && x<=sbX2-5){    // desrease slider touchable area (-5pix)
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6 
// slider width 30 pix, clearens between sliders 5pix

	 if (i==0 && bitRead(LedShannelStatusByte,0) == true){ sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2];        // CW colour (белый)	
	        wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}

	 if (i==1 && bitRead(LedShannelStatusByte,1) == true){ sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; 	 // BL colour (голубой)
                bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                   
	 if (i==2 && bitRead(LedShannelStatusByte,2) == true){ sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2];           // RBL colour (рояль)
                rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                  
         if (i==3 && bitRead(LedShannelStatusByte,3) == true){ sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2];      	  // DR colour (красный)	
		 rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); }       
                  
	 if (i==4 && bitRead(LedShannelStatusByte,4) == true){ sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2];         // UV colour (фиолет)
                   uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                   
	 if (i==5 && bitRead(LedShannelStatusByte,5) == true){ sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2];	   // OR colour (оранж)
                  ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                  
	 if (i==6 && bitRead(LedShannelStatusByte,6) == true){ sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2];		   // GR colour (зелен)
                  grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);}
                  
  //  if (ledON==0){ // включение управления каналами при настройке пресетов             
		  LED_levelo_output();  //} 
      } } } else

    if ((y>=29+2) && (y<=44-2) && (colorLEDtest==true)){ // UP Buttons were touched, desrease button touchable area (2pix)
 	 TopSldY=53, BotSldY=TopSldY+100; 
	     SliderSwitch  = true;

       for (byte i=0; i<7; i++){ sbX1=(i*40)+4+18; sbX2=(i*40)+34+18;
          if (x>=sbX1 && x<=sbX2){  // slider width 30 pix, clearens 5pix
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6
            
 if (i==0 && bitRead(LedShannelStatusByte,0) == true){ 
        sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide= wcol_out; tSlide+=1; 		     // CW colour (белый)	
	wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= wcol_out; }

 if (i==1 && bitRead(LedShannelStatusByte,1) == true){ 
	sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide= bcol_out; tSlide+=1; 			     // BL colour (голубой)
        bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= bcol_out;}

 if (i==2 && bitRead(LedShannelStatusByte,2) == true){ 
        sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide= rbcol_out; tSlide+=1; 		            // RBL colour (рояль)
        rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= rbcol_out;}
        
  if (i==3 && bitRead(LedShannelStatusByte,3) == true){ 
        sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; tSlide= rcol_out; tSlide+=1;                         // DR colour (красный)	
	rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= rcol_out; }       

  if (i==4 && bitRead(LedShannelStatusByte,4) == true){  
         sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide= uvcol_out; tSlide+=1; 	             // UV colour (фиолет)
         uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= uvcol_out;}

  if (i==5 && bitRead(LedShannelStatusByte,5) == true){  
        sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide= ocol_out; tSlide+=1;  		     // OR colour (оранж)
        ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= ocol_out; }

 if (i==6 && bitRead(LedShannelStatusByte,6) == true){  
        sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide= grcol_out; tSlide+=1; 		             // GR colour (зелен)
        grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= grcol_out; }
        
//if (ledON==0){ // включение управления каналами при настройке пресетов 
        _delay_ms(100);                // delay after touch UP/DOWN button       
       LED_levelo_output(); } }  
     
        SliderSwitch  = false; } else
      
    if ((y>=174+2) && (y<=187-2) && (colorLEDtest==true)){ // DOWN Buttons were touched,desrease button touchable area (2pix)  
	  TopSldY=53, BotSldY=TopSldY+100; 
	   SliderSwitch  = true;
       for (byte i=0; i<7; i++){ sbX1=(i*40)+4+18; sbX2=(i*40)+34+18;
          if (x>=sbX1 && x<=sbX2){                      // slider width 30 pix, clearens 5pix
          		
// WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6
 if (i==0 && bitRead(LedShannelStatusByte,0) == true){ 
          sbR=rgbCh0[0]; sbG=rgbCh0[1]; sbB=rgbCh0[2]; tSlide = wcol_out; tSlide-=1;		  // CW colour (белый)	
	  wcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= wcol_out; }

 if (i==1 && bitRead(LedShannelStatusByte,1) == true){ 
	  sbR=rgbCh1[0]; sbG=rgbCh1[1]; sbB=rgbCh1[2]; tSlide = bcol_out; tSlide-=1;		  // BL colour (голубой)
           bcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= bcol_out; }

 if (i==2 && bitRead(LedShannelStatusByte,2) == true){ 
          sbR=rgbCh2[0]; sbG=rgbCh2[1]; sbB=rgbCh2[2]; tSlide = rbcol_out; tSlide-=1; 	           // RBL colour (рояль)
          rbcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= rbcol_out; } 
          
 if (i==3 && bitRead(LedShannelStatusByte,3) == true){ 
          sbR=rgbCh3[0]; sbG=rgbCh3[1]; sbB=rgbCh3[2]; tSlide = rcol_out;	tSlide-=1;                  // DR colour (красный)	
	  rcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = rcol_out ; }        

 if (i==4 && bitRead(LedShannelStatusByte,4) == true){
          sbR=rgbCh4[0]; sbG=rgbCh4[1]; sbB=rgbCh4[2]; tSlide = uvcol_out; tSlide-=1;		   // UV colour (фиолет)
          uvcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= uvcol_out; } 

 if (i==5 && bitRead(LedShannelStatusByte,5) == true){
          sbR=rgbCh5[0]; sbG=rgbCh5[1]; sbB=rgbCh5[2]; tSlide = ocol_out; tSlide-=1;	            // OR colour (оранж)
          ocol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= ocol_out; }

 if (i==6 && bitRead(LedShannelStatusByte,6) == true){
          sbR=rgbCh6[0]; sbG=rgbCh6[1]; sbB=rgbCh6[2]; tSlide = grcol_out; tSlide-=1;		            // GR colour (зелен)
          grcol_out= SliderBarsForChange2( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide= grcol_out; }
          
 //if (ledON==0){ // включение управления каналами при настройке пресетов         
             _delay_ms(100);                    // delay 50msec after touch UP/DOWN button
	     LED_levelo_output(); } } }
	     SliderSwitch  = false;   

    break;   
    case 14:  //------------- GENERAL SETTINGS (PAGE 1) -----------------
     if ((x>=backGS[0]) && (x<=backGS[2]) && (y>=backGS[1]) && (y<=backGS[3])){  // press back    
           waitForIt(backGS[0], backGS[1], backGS[2], backGS[3]);
           LEDtestTick = false;        
           ReadFromEEPROM(); dispScreen=1; clearScreen(); menuScreen(); } else
        
     if ((x>=nextGS[0]) && (x<=nextGS[2]) && (y>=nextGS[1]) && (y<=nextGS[3])){  // press next
          waitForIt(nextGS[0], nextGS[1], nextGS[2], nextGS[3]);
          dispScreen=15; clearScreen(); generalSettingsScreen_2(); } else 
        
     if ((x>=prSAVEgs[0]) && (x<=prSAVEgs[2]) && (y>=prSAVEgs[1]) && (y<=prSAVEgs[3])){ // press SAVE
           waitForIt(prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3]);
           SaveGenSetsToEEPROM();
           SaveLEDsFailsafeToEEPROM(); dispScreen=1; clearScreen(); menuScreen(); }
             
     if ((x>=canCgs[0]) && (x<=canCgs[2]) && (y>=canCgs[1]) && (y<=canCgs[3])){      // press cancel 
           waitForIt(canCgs[0], canCgs[1], canCgs[2], canCgs[3]);
           LEDtestTick = false;    
           ReadFromEEPROM(); dispScreen=0; clearScreen(); mainScreen(true); } else        

      if (x>=195 && x<=295 && y>=20 && y<=40){      // press DETECT button 
           waitForIt(195, 20, 295, 40);
	    clearScreen(); dispScreen=30; DetectDallalsSensors(true); }

      if (x>=205 && x<=285 && y>=54 && y<=74){      // press BACKUP button 
           waitForIt(205, 54, 285, 74);
	    clearScreen(); dispScreen=31; Backup();}

      if ((x>=205) && (x<=285) && (y>=123) && (y<=143)){    // brignt
            waitForIt(205, 123, 285, 143);
            dispScreen=33; clearScreen(); LCDbrigh(true); } 

      if (x>=205 && x<=285 && y>=88 && y<=108 && bitRead(GlobalStatus1Byte,0) == false){  // press RESET button 
	     waitForIt(205, 88, 285, 108);
	      myGLCD.setColor(0, 0, 0);                    
              myGLCD.fillRoundRect(205, 88, 305, 108);    // clear button position
	      bitSet(GlobalStatus1Byte,0);                // set bit for check Y/N button

		myGLCD.setColor(255, 0, 0); 
		myGLCD.fillRoundRect(195, 88, 235, 108);       // YES
                myGLCD.setColor(0, 0, 255); 
	        myGLCD.fillRoundRect(255, 88, 295, 108);       // NO
		myGLCD.setColor(255, 255, 255);  
                myGLCD.setBackColor(255, 0, 0);        // фон красный
		myGLCD.drawRoundRect(195, 88, 235, 108);       // YES
                myGLCD.setBackColor(0, 0, 255);        // фон синий
		myGLCD.drawRoundRect(255, 88, 295, 108);       // NO

		setFont(SMALL, 255, 255, 255, 255, 0, 0);
                    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[8]))); 
                myGLCD.print(buffer, 195+8, 92);    // YES
                setFont(SMALL, 255, 255, 255, 0, 0, 255);
                    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[9]))); 
                myGLCD.print(buffer, 255+16, 92);   // NO
                myGLCD.setFont(RusFont2);
                myGLCD.setColor(255, 0, 0);
                myGLCD.setBackColor(0, 0, 0);
                myGLCD.print(print_text[200], 20, 92);     // СБРОСИТЬ НАСТРОЙКИ ? 
               } else
               
    if (x>=195 && x<=235 && y>=88 && y<=108 && bitRead(GlobalStatus1Byte,0) == true){  // press Yes button 
	 waitForIt(195, 88, 235, 108);  clearFscreen();
         myGLCD.setFont(RusFont1);
         myGLCD.setColor(255, 0, 0);
         myGLCD.setBackColor(0, 0, 0);
         myGLCD.print(print_text[191], CENTER, 110);     // ПОДОЖДИТЕ, идет очистка памяти
            resetScreen (); } else

    if (x>=255 && x<=295 && y>=88 && y<=108 && bitRead(GlobalStatus1Byte,0) == true){  // press NO button 
         waitForIt(255, 88, 295, 108); 
	 dispScreen=14; clearScreen(); generalSettingsScreen_1();
          bitClear(GlobalStatus1Byte,0); }                   // clear bit for check Y/N button
                       
      if ((x>=195) && (x<=295) && (y>=159) && (y<=179)){     // press CHANGE TEMPS
           waitForIt(195, 159, 295, 179);
           ReadFromEEPROM(); dispScreen=16; clearScreen(); ChangeFanTempsScreen(true);}
        
    break;  
    case 15:  //------------- GENERAL SETTINGS (PAGE 2) -----------------
//========================== для 3х кнопок ================================    
    if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   // Press back      
          waitForIt(back[0], back[1], back[2], back[3]);
          dispScreen=14; clearScreen(); generalSettingsScreen_1(); } else

    if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
           SaveGenSetsToEEPROM(); SaveDimmLEDToEEPROM();
          dispScreen=0; clearScreen(); mainScreen(true); }
                 
    if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){   // Press CANCEL 
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
       dispScreen=0; clearScreen(); mainScreen(true); } else 


    if ((x>=185) && (x<=305) && (y>=55) && (y<=75)){        // press CHANGE TEMP (Dim LEDs)
           waitForIt(185, 55, 305, 75);
          // ReadLedFromEEPROM();
           dispScreen=17; clearScreen(); DimLEDsAtTempScreen(); }

    if ((x>=185) && (x<=305) && (y>=129) && (y<=149)){      // press SETTINGS (Screensaver)
           waitForIt(185, 129, 305, 149);
           dispScreen=18; clearScreen(); ScreensaverSettingsScreen(); }

    if ((x>=195) && (x<=235)){                                // first column  
        if ((y>=27) && (y<=47)){ waitForIt(195, 27, 235, 47);   // press ON (Dim LEDs)
              setDimLEDsOnOff = 1; genSetSelect_2(); }
            
    if ((y>=101) && (y<=121)){ waitForIt(195, 101, 235, 121);  // press ON (Screensaver) 
           setScreensaverOnOff = 1; genSetSelect_2(); } 
              
      if ((y>=169) && (y<=189)){ waitForIt(195, 169, 235, 189);     // press ON (Dimm)
          DimmL=1; genSetSelect_2(); dispScreen=32; SetDimm(); } }    
                                
    if ((x>=255) && (x<=295)){                                  // second column     
        if ((y>=27) && (y<=47)){ waitForIt(255, 27, 295, 47);       // press OFF (Dim LEDs) 
             setDimLEDsOnOff = 0; genSetSelect_2(); }
            
    if ((y>=101) && (y<=121)){ waitForIt(255, 101, 295, 121);    // press OFF (Screensaver)
           setScreensaverOnOff = 0; genSetSelect_2(); } 
            
      if ((y>=169) && (y<=189)){ waitForIt(255, 169, 295, 189);    // press OFF (Dimm)
             DimmL=0; genSetSelect_2(); } } 
              
    break;    
    case 16: //------------------ CHANGE Heatsink FAN TEMP ---------------------
  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){    // press back    
         waitForIt(back[0], back[1], back[2], back[3]);
         dispScreen=14; clearScreen(); generalSettingsScreen_1(); } else
        
  if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
         setTempToBeginHeatsink1FanC=temp2beHFan;
         setTempToBeginHeatsink2FanC=temp2beSFan;       
         dispScreen=14; SaveGenSetsToEEPROM(); SaveTempToEEPROM(); clearScreen();
                generalSettingsScreen_1(); } else
        
     if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){  // press cancel 
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
         LEDtestTick = false;  
         dispScreen=0; clearScreen(); mainScreen(true); } else 

     if ((x>=SalaRm[0]) && (x<=SalaRm[2]) && (y>=SalaRm[1]) && (y<=SalaRm[3])){  // press Sound alarm 
         waitForIt(SalaRm[0], SalaRm[1], SalaRm[2], SalaRm[3]);    
         dispScreen=35; SoundAlarm(); } //else 
         
           setFont(LARGE, 255, 255, 255, 0, 0, 0);
      if ((x>=HoodFanTm[0]) && (x<=HoodFanTm[2])){           // first column    
      if ((y>=HoodFanTm[1]) && (y<=HoodFanTm[3])){           // press Heatsink1 Fan Temp -0.1
                  temp2beHFan -= 0.1;
            
        if (temp2beHFan <= 25.0) { temp2beHFan = 25.0; } // минимум температуры
            ChangeFanTempsScreen(); _delay_ms(140); }
           
       if ((y>=SumpFanTm[1]) && (y<=SumpFanTm[3])){ temp2beSFan -= 0.1;  // Радиатор датчик2 Fan Temp -0.1            
       if (temp2beSFan <= 25.0){ temp2beSFan = 25.0; }
            ChangeFanTempsScreen(); _delay_ms(140); } } 
             
      if ((x>=HoodFanTp[0]) && (x<=HoodFanTp[2])){              // second column 
         if ((y>=HoodFanTp[1]) && (y<=HoodFanTp[3])){ temp2beHFan += 0.1; // Радиатор датчик1 Fan Temp +0.1
        if (temp2beHFan >= 50.0) { temp2beHFan = 50.0; }
              ChangeFanTempsScreen(); _delay_ms(140); }
           
         if ((y>=SumpFanTp[1]) && (y<=SumpFanTp[3])){ temp2beSFan += 0.1;  // press Heatsink2 Fan Temp +0.1           
        if (temp2beSFan >= 50.0){ temp2beSFan = 50.0; }
             ChangeFanTempsScreen(); _delay_ms(140); } }
             
    break;   
    case 17:  // Температура авто-уменьшения яркости светильника
   if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]);
         dispScreen=15; clearScreen(); generalSettingsScreen_2(); } else
          
   if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE 
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
         setLEDsDimTempC=TempLEDsDimTemp;  //tempLED=255;    
         TempLEDsDimTemp=setDimLEDsOnOff;     
         setLEDsDimPercent=TempLEDsDimPercent;
         TempLEDsDimPercent=setLEDsDimPercent;    
         SaveLEDsFailsafeToEEPROM();         
         dispScreen=15; clearScreen(); generalSettingsScreen_2(); } else
          
   if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){  // press cancel 
         waitForIt(canC[0], canC[1], canC[2], canC[3]);
         LEDtestTick = false;
         ReadFromEEPROM();
         dispScreen=0; clearScreen(); mainScreen(true); } else

   if ((x>=235) && (x<=260) && (y>=36) && (y<=61)){    // press TEMP UP button
         waitForIt(235, 36, 260, 61); TempLEDsDimTemp++; 
         
         setFont(LARGE, 255, 108, 72, 0, 0, 0); // красный шрифт
      if (TempLEDsDimTemp>=255){ TempLEDsDimTemp=255;} // 255
      if (TempLEDsDimTemp>=100){ myGLCD.printNumI(TempLEDsDimTemp, 181, 55);}
      if ((TempLEDsDimTemp<=99)&&(TempLEDsDimTemp>=10)){ myGLCD.printNumI(TempLEDsDimTemp, 189, 55);}  
           
   if (TempLEDsDimTemp<=9){ myGLCD.printNumI(TempLEDsDimTemp, 198, 55);} } else
        
    if ((x>=235) && (x<=260) && (y>=66) && (y<=91)){    // press TEMP DOWN button 
         waitForIt(235, 66, 260, 91); TempLEDsDimTemp--; 
         setFont(LARGE, 255, 108, 72, 0, 0, 0);
        if (TempLEDsDimTemp<=0){ TempLEDsDimTemp=0;}
        if (TempLEDsDimTemp>=100){ myGLCD.printNumI(TempLEDsDimTemp, 181, 55);}
        if ((TempLEDsDimTemp<=99)&&(TempLEDsDimTemp>=10)){ 
             myGLCD.setColor(0, 0, 0);
             myGLCD.fillRect(181, 55, 188, 71);
             myGLCD.fillRect(221, 55, 229, 71);             
             setFont(LARGE, 0, 255, 0, 0, 0, 0);
             myGLCD.printNumI(TempLEDsDimTemp, 189, 55);}
             
   if (TempLEDsDimTemp<=9){ 
             myGLCD.setColor(0, 0, 0);
             myGLCD.fillRect(181, 55, 197, 71);
             myGLCD.fillRect(214, 55, 229, 71);             
             setFont(LARGE, 0, 255, 0, 0, 0, 0);
             myGLCD.printNumI(TempLEDsDimTemp, 198, 55); } } else        

   if ((x>=235) && (x<=260) && (y>=117) && (y<=142)){          // press % UP button
         waitForIt(235, 117, 260, 142); TempLEDsDimPercent++; 
         setFont(LARGE, 255, 255, 255, 0, 0, 0);
       if (TempLEDsDimPercent>=100){ TempLEDsDimPercent=100;}
       if (TempLEDsDimPercent>=100){ myGLCD.printNumI(TempLEDsDimPercent, 181, 136);}         
       if ((TempLEDsDimPercent<=99)&&(TempLEDsDimPercent>=10)){
             myGLCD.printNumI(TempLEDsDimPercent, 189, 136);}
       if (TempLEDsDimPercent<=9){ myGLCD.printNumI(TempLEDsDimPercent, 198, 136);} } else 
            
    if ((x>=235) && (x<=260) && (y>=147) && (y<=172)){          // press % DOWN button
         waitForIt(235, 147, 260, 172); TempLEDsDimPercent--; 
         setFont(LARGE, 255, 255, 255, 0, 0, 0);
         if (TempLEDsDimPercent<=0){ TempLEDsDimPercent=0;}
         if ((TempLEDsDimPercent<=99)&&(TempLEDsDimPercent>=10)){ 
             myGLCD.setColor(0, 0, 0);
             myGLCD.fillRect(181, 136, 188, 152);
             myGLCD.fillRect(221, 136, 229, 152);             
             setFont(LARGE, 0, 255, 0, 0, 0, 0);
             myGLCD.printNumI(TempLEDsDimPercent, 189, 136);}
         if (TempLEDsDimPercent<=9){ 
             myGLCD.setColor(0, 0, 0);
             myGLCD.fillRect(181, 136, 197, 152);
             myGLCD.fillRect(214, 136, 229, 152);             
             setFont(LARGE, 0, 255, 0, 0, 0, 0);
             myGLCD.printNumI(TempLEDsDimPercent, 198, 136);} }        
    break;      
    case 18:  //-------------- SET SCREENSAVER WAIT TIME ----------------
      if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   // press back    
           waitForIt(back[0], back[1], back[2], back[3]);
           dispScreen=15; clearScreen(); generalSettingsScreen_2(); } else
        
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
           waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
           setSSmintues=TempSSminutes; SaveGenSetsToEEPROM();
            dispScreen=0; clearScreen(); mainScreen(true); } else
        
      if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){  // press cancel 
           waitForIt(canC[0], canC[1], canC[2], canC[3]);
           LEDtestTick = false;  
           ReadFromEEPROM(); dispScreen=0; clearScreen(); mainScreen(true); } else  

    if ((x>=185) && (x<=235)){                 // first column
        if ((y>=20) && (y<=40)){               // press BLANK screensaver
           waitForIt(185, 20, 235, 40); setClockOrBlank = 1; ScreensaverSelect();}
           
    if ((y>=51) && (y<=71)){                   // press YES, show Date on Screensaver
          waitForIt(185, 51, 235, 71); setScreensaverDOWonOff = 1; ScreensaverSelect(); } } 
          
  if ((x>=255) && (x<=305)){                   // second column      
    if ((y>=20) && (y<=40)){                   // press CLOCK screensaver
          waitForIt(255, 20, 305, 40); setClockOrBlank = 0; ScreensaverSelect();}            
           
    if ((y>=51) && (y<=71)){                   // press NO, show Date on Screensaver
          waitForIt(255, 51, 305, 71); setScreensaverDOWonOff = 0; ScreensaverSelect(); } }         

    if ((x>=245) && (x<=301) && (y>=100) && (y<=119)){      // Выбрать A.Clock скринсейв 
          waitForIt(245, 100, 301, 119); 
           digital = 1; analog = 0; setScreensaverTupe = 0; ScreensaverSelect();}             
           
    if ((x>=245) && (x<=301) && (y>=132) && (y<=152)){      // Выбрать D.Clock скринсейв
          waitForIt(245, 132, 301, 152); 
           analog = 1; digital = 0; setScreensaverTupe = 1; ScreensaverSelect(); }
      
    if ((x>=135) && (x<=160) && (y>=92) && (y<=117)){  // Кнопка минуты плюс 
          waitForIt(135, 92, 160, 117); TempSSminutes++; _delay_ms(10);
           myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта 
           myGLCD.setColor(80, 255, 246);    // цвет голубой
           myGLCD.setBackColor(0, 0, 0);     // цвет фона черный
           
    if (TempSSminutes>=99){ TempSSminutes=99;}
    if (TempSSminutes>=10){
          myGLCD.printNumI(TempSSminutes, 87, 112);} else {   // Время включения скринсейва
          myGLCD.printNumI(TempSSminutes, 102, 112);}} else  
        
     if ((x>=135) && (x<=160) && (y>=132) && (y<=157)){       // Кнопка минуты минус 
         waitForIt(135, 132, 160, 157); TempSSminutes--; _delay_ms(10);
          myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта 
          myGLCD.setColor(80, 255, 246);    // цвет голубой
          myGLCD.setBackColor(0, 0, 0);     // цвет фона черный
          
     if (TempSSminutes<=1){ TempSSminutes=1;}
     if (TempSSminutes>=10){ myGLCD.printNumI(TempSSminutes, 87, 112);}
                      else { myGLCD.printNumI(TempSSminutes, 102, 112);
              myGLCD.setColor(0, 0, 0);
              myGLCD.fillRect(87, 112, 98, 135); }}  // очистка при смене цифр
                                 
    break; 
    case 19:  // Timer
// Таймер 1      
         if ((x>=5) && (x<=58) && (y>=20) && (y<=186)){waitForIt(5, 20, 58, 186);     // координаты кнопки
              dispScreen=20; clearScreen();light1set();}                                        
// Таймер 2                          
         if ((x>=69) && (x<=122) && (y>=20) && (y<=186)){waitForIt(69, 20, 122, 186); // координаты кнопки
              dispScreen=21; clearScreen();light2set();}                                  
// Таймер 3        
         if ((x>=133) && (x<=186) && (y>=20) && (y<=186)){waitForIt(133, 20, 186, 186);// координаты кнопки
              dispScreen=22; clearScreen();light3set();}                                            
// Таймер 4       
         if ((x>=197) && (x<=250) && (y>=20) && (y<=186)){waitForIt(197, 20, 250, 186);// координаты кнопки
              dispScreen=23; clearScreen();light4set();}                                                 
// Таймер 5       
         if ((x>=261) && (x<=314) && (y>=20) && (y<=186)){waitForIt(261, 20, 314, 186);// координаты кнопки
              dispScreen=24; clearScreen();light5set();} 
              
         if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
              waitForIt(back[0], back[1], back[2], back[3]);
               dispScreen=1; clearScreen(); menuScreen(); } 
    break;      
   case 20:  // Timer 1    
// Часы включения (плюс)
            if (x>btonhup[0] && x<btonhup[2] && y>btonhup[1] && y<btonhup[3]){on1=on1+60;}   // Часы включения +
            if (x>btonhdn[0] && x<btonhdn[2] && y>btonhdn[1] && y<btonhdn[3]){on1=on1-60;} // Часы включения -           
// Минуты включения (плюс)
            if (x>btonmup[0] && x<btonmup[2] && y>btonmup[1] && y<btonmup[3]){on1++;}       // Минуты включения +
            if (x>btonmdn[0] && x<btonmdn[2] && y>btonmdn[1] && y<btonmdn[3]){on1--;}     // Минуты включения -           
// Часы выключения
            if (x>btofhup[0] && x<btofhup[2] && y>btofhup[1] && y<btofhup[3]){off1=off1+60;}  // Часы выключения +
            if (x>btofhdn[0] && x<btofhdn[2] && y>btofhdn[1] && y<btofhdn[3]){off1=off1-60;}// Часы выключения -           
// Минуты выключения
            if (x>btofmup[0] && x<btofmup[2] && y>btofmup[1] && y<btofmup[3]){off1++;}     // Минуты выключения +
            if (x>btofmdn[0] && x<btofmdn[2] && y>btofmdn[1] && y<btofmdn[3]){off1--;}   // Минуты выключения - 
                                 
            if (on1<0){on1=1439;}                 // кнопка (плюс) ON
            if (on1>1439){on1=0;}                 // кнопка (минус) ON 
            if (off1<0){off1=1439;}               
            if (off1>1439){off1=0;}               // установки таймера освещения канала 1
               timer1Change(); _delay_ms(100); 

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]); 
         dispScreen=19; clearScreen(); TimerScreen();  } else  // Суточные Таймеры 
                
// возврат в меню cуточных таймеров
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
           waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
           SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen=19; TimerScreen(); } 
    break;  
   case 21:   // Timer 2 
 // Часы включения (плюс)
            if (x>btonhup[0] && x<btonhup[2] && y>btonhup[1] && y<btonhup[3]){on2=on2+60;}   // Часы включения +
            if (x>btonhdn[0] && x<btonhdn[2] && y>btonhdn[1] && y<btonhdn[3]){on2=on2-60;} // Часы включения -           
// Минуты включения (плюс)
            if (x>btonmup[0] && x<btonmup[2] && y>btonmup[1] && y<btonmup[3]){on2++;}   // Минуты включения +
            if (x>btonmdn[0] && x<btonmdn[2] && y>btonmdn[1] && y<btonmdn[3]){on2--;} // Минуты включения -           
// Часы выключения
            if (x>btofhup[0] && x<btofhup[2] && y>btofhup[1] && y<btofhup[3]){off2=off2+60;}  // Часы выключения +
            if (x>btofhdn[0] && x<btofhdn[2] && y>btofhdn[1] && y<btofhdn[3]){off2=off2-60;}// Часы выключения -           
// Минуты выключения
            if (x>btofmup[0] && x<btofmup[2] && y>btofmup[1] && y<btofmup[3]){off2++;}   // Минуты выключения +
            if (x>btofmdn[0] && x<btofmdn[2] && y>btofmdn[1] && y<btofmdn[3]){off2--;} // Минуты выключения - 
                                 
            if (on2<0){on2=1439;}                 // кнопка (плюс) ON
            if (on2>1439){on2=0;}                 // кнопка (минус) ON 
            if (off2<0){off2=1439;}               
            if (off2>1439){off2=0;}               // установки таймера освещения канала 1
               timer2Change(); _delay_ms(100); 	

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]); 
         dispScreen=19; clearScreen(); TimerScreen();  } else  // Суточные Таймеры 

// запись настроек
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
           waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
           SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen=19; TimerScreen(); }
     break;  
    case 22:  // Timer 3      
// Часы включения (плюс)
            if (x>btonhup[0] && x<btonhup[2] && y>btonhup[1] && y<btonhup[3]){on3=on3+60;}   // Часы включения +
            if (x>btonhdn[0] && x<btonhdn[2] && y>btonhdn[1] && y<btonhdn[3]){on3=on3-60;} // Часы включения -            
// Минуты включения (плюс)
            if (x>btonmup[0] && x<btonmup[2] && y>btonmup[1] && y<btonmup[3]){on3++;}   // Минуты включения +
            if (x>btonmdn[0] && x<btonmdn[2] && y>btonmdn[1] && y<btonmdn[3]){on3--;} // Минуты включения -            
// Часы выключения
            if (x>btofhup[0] && x<btofhup[2] && y>btofhup[1] && y<btofhup[3]){off3=off3+60;}  // Часы выключения +
            if (x>btofhdn[0] && x<btofhdn[2] && y>btofhdn[1] && y<btofhdn[3]){off3=off3-60;}// Часы выключения -            
// Минуты выключения
            if (x>btofmup[0] && x<btofmup[2] && y>btofmup[1] && y<btofmup[3]){off3++;}   // Минуты выключения +
            if (x>btofmdn[0] && x<btofmdn[2] && y>btofmdn[1] && y<btofmdn[3]){off3--;} // Минуты выключения - 
                                 
            if (on3<0){on3=1439;}                 // кнопка (плюс) ON
            if (on3>1439){on3=0;}
            if (off3<0){off3=1439;}               // кнопка (минус)
            if (off3>1439){off3=0;}               // установки таймера освещения канала 3
             timer3Change(); _delay_ms(100); 

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]); 
         dispScreen=19; clearScreen(); TimerScreen();  } else  // Суточные Таймеры 
                
// запись настроек
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
          waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen=19; TimerScreen(); } 
    break;  
   case 23:   // Timer 4      
// Часы включения (плюс)
            if (x>btonhup[0] && x<btonhup[2] && y>btonhup[1] && y<btonhup[3]){on4=on4+60;}   // Часы включения +
            if (x>btonhdn[0] && x<btonhdn[2] && y>btonhdn[1] && y<btonhdn[3]){on4=on4-60;} // Часы включения -          
// Минуты включения (плюс)
            if (x>btonmup[0] && x<btonmup[2] && y>btonmup[1] && y<btonmup[3]){on4++;}   // Минуты включения +
            if (x>btonmdn[0] && x<btonmdn[2] && y>btonmdn[1] && y<btonmdn[3]){on4--;} // Минуты включения -           
// Часы выключения
            if (x>btofhup[0] && x<btofhup[2] && y>btofhup[1] && y<btofhup[3]){off4=off4+60;}  // Часы выключения +
            if (x>btofhdn[0] && x<btofhdn[2] && y>btofhdn[1] && y<btofhdn[3]){off4=off4-60;}// Часы выключения -         
// Минуты выключения
            if (x>btofmup[0] && x<btofmup[2] && y>btofmup[1] && y<btofmup[3]){off4++;}   // Минуты выключения +
            if (x>btofmdn[0] && x<btofmdn[2] && y>btofmdn[1] && y<btofmdn[3]){off4--;} // Минуты выключения - 
                                 
            if (on4<0){on4=1439;}                 // кнопка (плюс) ON
            if (on4>1439){on4=0;}
            if (off4<0){off4=1439;}               // кнопка (минус)
            if (off4>1439){off4=0;}               // установки таймера освещения канала 4
              timer4Change(); _delay_ms(100); 

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]); 
         dispScreen=19; clearScreen(); TimerScreen();  } else  // Суточные Таймеры 
                
// запись настроек
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
          waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen=19; TimerScreen(); } 
    break;  
   case 24:   // Timer 5      
// Часы включения (плюс)
            if (x>btonhup[0] && x<btonhup[2] && y>btonhup[1] && y<btonhup[3]){on5=on5+60;}   // Часы включения +
            if (x>btonhdn[0] && x<btonhdn[2] && y>btonhdn[1] && y<btonhdn[3]){on5=on5-60;} // Часы включения -            
// Минуты включения (плюс)
            if (x>btonmup[0] && x<btonmup[2] && y>btonmup[1] && y<btonmup[3]){on5++;}   // Минуты включения +
            if (x>btonmdn[0] && x<btonmdn[2] && y>btonmdn[1] && y<btonmdn[3]){on5--;} // Минуты включения -            
// Часы выключения
            if (x>btofhup[0] && x<btofhup[2] && y>btofhup[1] && y<btofhup[3]){off5=off5+60;}  // Часы выключения +
            if (x>btofhdn[0] && x<btofhdn[2] && y>btofhdn[1] && y<btofhdn[3]){off5=off5-60;}// Часы выключения -            
// Минуты выключения
            if (x>btofmup[0] && x<btofmup[2] && y>btofmup[1] && y<btofmup[3]){off5++;}   // Минуты выключения +
            if (x>btofmdn[0] && x<btofmdn[2] && y>btofmdn[1] && y<btofmdn[3]){off5--;} // Минуты выключения - 
                                 
            if (on5<0){on5=1439;}                 // кнопка (плюс) ON
            if (on5>1439){on5=0;}
            if (off5<0){off5=1439;}               // кнопка (минус)
            if (off5>1439){off5=0;}               // установки таймера освещения канала 5
              timer5Change(); _delay_ms(100); 

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){ // press back    
         waitForIt(back[0], back[1], back[2], back[3]); 
         dispScreen=19; clearScreen(); TimerScreen();  } else  // Суточные Таймеры 
                
// запись настроек
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){ // press SAVE
          waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
          SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen=19; TimerScreen(); } 
    break;         
    case 25: // ручное управление таймерами      
// Таймер 1
            if (x>107 && x<166 && y>18 && y<48){_delay_ms(200); timer1Status=0;onoff1();}   // auto
            if (x>175 && x<240 && y>18 && y<48){_delay_ms(200); timer1Status=2;onoff1();}   // on
            if (x>245 && x<313 && y>18 && y<48){_delay_ms(200); timer1Status=3;onoff1();}   // off            
// Таймер 2
            if (x>107 && x<166 && y>54 && y<84){_delay_ms(200); timer2Status=0;onoff2();}   
            if (x>175 && x<240 && y>54 && y<84){_delay_ms(200); timer2Status=2;onoff2();}
            if (x>245 && x<313 && y>54 && y<84){_delay_ms(200); timer2Status=3;onoff2();}               
// Таймер 3
            if (x>107 && x<166 && y>90 && y<120){_delay_ms(200); timer3Status=0;onoff3();}   
            if (x>175 && x<240 && y>90 && y<120){_delay_ms(200); timer3Status=2;onoff3();}
            if (x>245 && x<313 && y>90 && y<120){_delay_ms(200); timer3Status=3;onoff3();}            
// Таймер 4
            if (x>107 && x<166 && y>126 && y<156){_delay_ms(200); timer4Status=0;onoff4();}   
            if (x>175 && x<240 && y>126 && y<156){_delay_ms(200); timer4Status=2;onoff4();}
            if (x>245 && x<313 && y>126 && y<156){_delay_ms(200); timer4Status=3;onoff4();}            
// Таймер 5
            if (x>107 && x<166 && y>162 && y<192){_delay_ms(200); timer5Status=0;onoff5();}  
            if (x>175 && x<240 && y>162 && y<192){_delay_ms(200); timer5Status=2;onoff5();}
            if (x>245 && x<313 && y>162 && y<192){_delay_ms(200); timer5Status=3;onoff5();}
   break;   
   case 26: // экран настройки яркости луны
  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){   
        waitForIt(back[0], back[1], back[2], back[3]);
        dispScreen=7; clearScreen(); ledColorViewScreen(); } else 
                  
  if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])&& (COLOR==8)){ // press SAVE 
        waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
        SaveMoonLEDToEEPROM(); dispScreen=7; clearScreen(); ledColorViewScreen(); } else  
      
  if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3]) && (COLOR==8)){    // press CANCEL
       waitForIt(canC[0], canC[1], canC[2], canC[3]);
       COLOR =8; ReadFromEEPROM(); LEDtestTick = false;
       dispScreen=0; clearScreen(); mainScreen(true); } else 
                
// настройки ЛУНЫ 
  if ((x>=MINiM[0]) && (x<=MINiM[2]) && (y>=MINiM[1]) && (y<=MINiM[3])){    // press MinI minus
       waitForIt(MINiM[0], MINiM[1], MINiM[2], MINiM[3]); tMinI -= 1;
     if (tMinI <= 0) { tMinI = 0; } MinI = tMinI;         
       setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 55, 152);  
       if (tMinI<=9){ myGLCD.printNumI(tMinI, 71, 152);}
       if ((tMinI>=10)&&(tMinI<=99)){ myGLCD.printNumI(tMinI, 63, 152);}
       if (tMinI>=100){ myGLCD.printNumI(tMinI, 55, 152);} } else 
      
  if ((x>=MINiP[0]) && (x<=MINiP[2]) && (y>=MINiP[1]) && (y<=MINiP[3])){   // press MinI plus
       waitForIt(MINiP[0], MINiP[1], MINiP[2], MINiP[3]); tMinI += 5;
    if (tMinI >100) { tMinI = 100; } MinI = tMinI;    // 255     
       setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 55, 152);
       if (tMinI<=9){ myGLCD.printNumI(tMinI, 71, 152);}
       if ((tMinI>=10)&&(tMinI<=99)){ myGLCD.printNumI(tMinI, 63, 152);}
       if (tMinI>=100){ myGLCD.printNumI(tMinI, 55, 152);} } else

  if ((x>=MAXiM[0]) && (x<=MAXiM[2]) && (y>=MAXiM[1]) && (y<=MAXiM[3])){   // press MaxI minus
       waitForIt(MAXiM[0], MAXiM[1], MAXiM[2], MAXiM[3]); tMaxI -= 1;
     if (tMaxI <= 0) { tMaxI = 0; } MaxI = tMaxI;         
       setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 217, 152);
       if (tMaxI<=9){ myGLCD.printNumI(tMaxI, 233, 152);}
       if ((tMaxI>=10)&&(tMaxI<=99)){ myGLCD.printNumI(tMaxI, 225, 152);}
       if (tMaxI>=100){ myGLCD.printNumI(tMaxI, 217, 152);} } else 
      
  if ((x>=MAXiP[0]) && (x<=MAXiP[2]) && (y>=MAXiP[1]) && (y<=MAXiP[3])){   // press MaxI plus
       waitForIt(MAXiP[0], MAXiP[1], MAXiP[2], MAXiP[3]); tMaxI += 5;
     if (tMaxI >100) { tMaxI = 100; } MaxI = tMaxI;   // 255      
       setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 217, 152);
       if (tMaxI<=9){ myGLCD.printNumI(tMaxI, 233, 152);}
       if ((tMaxI>=10)&&(tMaxI<=99)){ myGLCD.printNumI(tMaxI, 225, 152);}
       if (tMaxI>=100){ myGLCD.printNumI(tMaxI, 217, 152);}}

   break;
   case 27:  // График ЛОГ температуры радиаторов F1, F2 
  
     if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){      // press back    
          waitForIt(back[0], back[1], back[2], back[3]);                                   
           dispScreen=1; clearFscreen(); menuScreen(); }          

     if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){      // press CANCEL
          waitForIt(canC[0], canC[1], canC[2], canC[3]); 
          LEDtestTick = false; dispScreen=0; clearScreen(); mainScreen(true); }
       
     if ((x>=Fg1[0]) && (x<=Fg1[2]) && (y>=Fg1[1]) && (y<=Fg1[3])){      
          waitForIt(Fg1[0], Fg1[1], Fg1[2], Fg1[3]);      
     if (F1 == true) { F1 = false; graph_colorFAN(); timedrawScreen(); }
                else { F1 = true; graph_colorFAN(); timedrawScreen(); }}
                  
     if ((x>=Fg2[0]) && (x<=Fg2[2]) && (y>=Fg2[1]) && (y<=Fg2[3])){   
          waitForIt(Fg2[0], Fg2[1], Fg2[2], Fg2[3]);
     if (F2 == true) { F2 = false; graph_colorFAN(); timedrawScreen();}
                else { F2 = true; graph_colorFAN(); timedrawScreen(); }}      
		
  break;
  case 28:  // Лог температуры Воды в аквариуме 
  
     if ((x>=backGS[0]) && (x<=backGS[2]) && (y>=backGS[1]) && (y<=backGS[3])){   // press << back    
           waitForIt(backGS[0], backGS[1], backGS[2], backGS[3]);
           LEDtestTick = false;      
           dispScreen=1; clearFscreen(); menuScreen(); } else     // выход в главное меню
             
     if ((x>=canCgs[0]) && (x<=canCgs[2]) && (y>=canCgs[1]) && (y<=canCgs[3])){      // press cancel 
           waitForIt(canCgs[0], canCgs[1], canCgs[2], canCgs[3]);    
           ReadFromEEPROM(); dispScreen=0; clearFscreen(); mainScreen(true); } 

  break;
  case 29: // авто-тест каналов с графиками 

    if ((x>=Wg[0]) && (x<=Wg[2]) && (y>=Wg[1]) && (y<=Wg[3])){             // W
                       waitForIt(Wg[0], Wg[1], Wg[2], Wg[3]);
    if (W == true) { W = false; graph_color(); } else {W = true; graph_color(); }}       
    if ((x>=RBg[0]) && (x<=RBg[2]) && (y>=RBg[1]) && (y<=RBg[3])){         // RB
                        waitForIt(RBg[0], RBg[1], RBg[2], RBg[3]);
    if (RB == true) { RB = false; graph_color(); } else {RB = true; graph_color(); }}       
    if ((x>=Bg[0]) && (x<=Bg[2]) && (y>=Bg[1]) && (y<=Bg[3])){             // B
                        waitForIt(Bg[0], Bg[1], Bg[2], Bg[3]);
    if (B == true) { B = false; graph_color(); } else {B = true; graph_color(); }}     
    if ((x>=Rg[0]) && (x<=Rg[2]) && (y>=Rg[1]) && (y<=Rg[3])){             // R
                         waitForIt(Rg[0], Rg[1], Rg[2], Rg[3]);
    if (R == true) { R = false; graph_color(); } else {R = true; graph_color(); }}     
    if ((x>=UVg[0]) && (x<=UVg[2]) && (y>=UVg[1]) && (y<=UVg[3])){         // UV
                         waitForIt(UVg[0], UVg[1], UVg[2], UVg[3]);                     
    if (UV == true) { UV = false; graph_color(); } else {UV = true; graph_color(); }}      
    if ((x>=GRg[0]) && (x<=GRg[2]) && (y>=GRg[1]) && (y<=GRg[3])){         // SU
                          waitForIt(GRg[0], GRg[1], GRg[2], GRg[3]);     
    if (SU == true) { SU = false; graph_color(); } else {SU = true; graph_color(); }}     
    if ((x>=ORg[0]) && (x<=ORg[2]) && (y>=ORg[1]) && (y<=ORg[3])){         // OR
                          waitForIt(ORg[0], ORg[1], ORg[2], ORg[3]);
    if (OR == true) { OR = false; graph_color(); } else {OR = true; graph_color(); }} 

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])    // press back    
       && (LEDtestTick==false)){ waitForIt(back[0], back[1], back[2], back[3]);
       LEDtestTick = false; ReadFromEEPROM(); ReadLedFromEEPROM();
       dispScreen=1; clearFscreen(); menuScreen(); }   // display buttons "Rapid test" and "Control Individual Leds"
           
  if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])   // press CANCEL
       && (LEDtestTick==false)){ waitForIt(canC[0], canC[1], canC[2], canC[3]);
       LEDtestTick = false; ReadLedFromEEPROM(); dispScreen=0; clearFscreen(); mainScreen(true); } 
       
// Кнопка Старт / Стоп
    if ((x>=ledChV[0]) && (x<=ledChV[2]) && (y>=ledChV[1]) && (y<=ledChV[3]) && dispScreen==29){  // press start/stop test
        waitForIt(ledChV[0], ledChV[1], ledChV[2], ledChV[3]); 
        
   //printButton(print_text[4], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL, GREEN_BAC); // STOP buton start/stop test
    myGLCD.setColor(255, 255, 255);
    printButton104(print_text[4], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL);          // красная кнопка

     if (LEDtestTick == true){ LEDtestTick = false;    // stop test
                      } else { LEDtestTick = true;     // start test         
	 myGLCD.setColor(0, 0, 0);
	 myGLCD.fillRect(26, BotSldY+8, 318, BotSldY+3);    // clear test bar
                 drawTestLedArrayScale(); } }

   break;      
   case 30:  // ============== меню авто-определения датчиков температуры ==================		
  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3]) && dispScreen==30 ){ // press back    
       waitForIt(back[0], back[1], back[2], back[3]);
       dispScreen=14; clearScreen(); generalSettingsScreen_1(); } else
     
  if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])){    	// press SAVE
       waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
        DetectDallalsSensors(false);
        SaveDallasAddress(); dispScreen=0; clearScreen(); mainScreen(true); } else

   if (x>=165 && x<=295 && y>=19 && y<=41){  // поиск датчиков
         waitForIt(165, 19, 295, 41);        // re-read all sensor data
	 DetectDallalsSensors(false); }
	 
    if ((x>=165) && (x<=231) && (y>=83+20) && (y<=83+40) && numberOfDevices >=1){  // sensor N1 button 
	   waitForIt(165, 83+20, 231, 83+40); printCoun(); counterB1 +=1;

    if (counterB1  > 3) {counterB1 = 0;}
	   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB1]))); 
        myGLCD.print(buffer, 173, 85+24); } else  // counterBX = 0/1/2/3 -> Отключ / Д.Воды / Д.Рад:1 / Д.Рад:2 

     if ((x>=165) && (x<=231) && (y>=83+50) && (y<=83+70) && numberOfDevices >=2){  // sensor N2 button 
	   waitForIt(165, 83+50, 231, 83+70); printCoun(); counterB2 +=1;

     if (counterB2  > 3) {counterB2 = 0;}
             strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB2]))); 
         myGLCD.print(buffer, 173, 85+54); } else // counterBX = 0/1/2/3 -> Отключ./ Д.Воды / Д.Рад:1 / Д.Рад:2

     if ((x>=165) && (x<=231) && (y>=83+80) && (y<=83+100) && numberOfDevices ==3){  // sensor N3 button 
	   waitForIt(165, 83+80, 231, 83+100); printCoun(); counterB3 +=1;

     if (counterB3  > 3) {counterB3 = 0;}
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176+counterB3]))); 
	  myGLCD.print(buffer, 173, 85+84); }    // counterBX = 0/1/2/3 -> Отключ / Д.Воды / Д.Рад:1 / Д.Рад:2
  
   break;
   case 31:  // ----------- BACKUP ALL EEPROM SETTING -----------------
	 if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3]) && dispScreen==31 ){ // press back    
               waitForIt(back[0], back[1], back[2], back[3]);
		dispScreen=14; clearScreen(); generalSettingsScreen_1(); } else 

	 if ((x>=205 && x<=305 && y>=73 && y<=109) && dispScreen==31){  // BACKUP  сохранить настройки на флеш карту
	       waitForIt(205, 73, 305, 109);
               myGLCD.setColor(0, 0, 0);			        // clear text area
	       myGLCD.fillRoundRect(16, 126, 303, 181);
                sd.remove("Backup.txt");                                // remove old file from card
	      myFile.open("Backup.txt", O_CREAT | O_EXCL | O_WRITE) ;

 	 if (myFile.isOpen()){ 
                   myGLCD.setFont(RusFont1); 
                   myGLCD.setColor(38, 195, 178);  // цвет бирюзовый
                   myGLCD.setBackColor(0, 0, 0);
		   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[174]))); 
	       myGLCD.print(buffer, CENTER, 139);       // Запись настроек на флеш карту

		 for (int i=0; i<=4095; i++){ byte TEMP = EEPROM.read(i);  // readed data in DEC format 
                   myFile.print(TEMP, DEC);                                // store to file in DEC   
		   myFile.print(','); }         // store separator "," between bytes
   		    myFile.close();
   
                   myGLCD.setFont(RusFont1); 
                   myGLCD.setColor(0, 255, 0);  // цвет бирюзовый
                   myGLCD.setBackColor(0, 0, 0);
		    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[175]))); 
	        myGLCD.print(buffer, CENTER, 158);       // Настройки успешно сохранены
                        } else {	
                    myGLCD.setColor(255, 0, 0);   // красный цвет
                    myGLCD.setBackColor(0, 0, 0);
                    myGLCD.print(print_text[199], CENTER, 158); }  // Ошибка открытия файла настроек
         	      } else

        if ((x>=15 && x<=115 && y>=73 && y<=109) && dispScreen==31){  // press RESTORE button ВОССТАНОВИТЬ
	       waitForIt(15, 73, 115, 109);
               myGLCD.setColor(0, 0, 0);			      // clear text area
	       myGLCD.fillRoundRect(16, 126, 303, 181);

// re-open the file for reading:
	if (myFile.open("Backup.txt", O_READ)){
                 myGLCD.setFont(RusFont1); 
                 myGLCD.setColor(0, 255, 0);  // цвет бирюзовый
                 myGLCD.setBackColor(0, 0, 0);
                 myGLCD.print(print_text[198], CENTER, 139);   // Чтение файла настроек на флеш карте 
                 myGLCD.print(print_text[197], CENTER, 158);   // Пожалуйста подождите.... 

	 int TEMP; int i=0; byte SD_buff[3]; byte k=0;

// read from the file until there's nothing else in it:
	  while ((TEMP = myFile.read()) > 0){    // readed data in ASCII only, need convert ASCII to DEC
					
//	  Serial.write((char)TEMP);		        // output to serial port 			
	if (TEMP != 44 && k<=2) { TEMP= TEMP -'0';      // convert ASCII to DEC
	      SD_buff[k]= TEMP; k++; } else {
  
    int Result = SD_buff[0]*100 + SD_buff[1]*10 + SD_buff[2];    // convert three consecutive bytes to one decimal
      if (k ==2) {Result = Result/10;}                           // before save to eeprom    
      if (k ==1) {Result = Result/100;} EEPROM.write(i, Result); // store data to EEPROM
		i +=1; k=0;
		SD_buff[0]=0;                            // clean buffer before new read
		SD_buff[1]=0;
		SD_buff[2]=0; } }
		myFile.close();
		ReadDallasAddress ();	                  // read temp sensor address from eeprom
		ReadLedFromEEPROM();			  // read led setting from EEPROM
		ReadFromEEPROM();			  // read other setting from eeprom
					
             myGLCD.setFont(RusFont1); 
             myGLCD.setColor(0, 255, 0);  
             myGLCD.setBackColor(0, 0, 0); 
             myGLCD.print(print_text[196], CENTER, 158);     // Восстановление настроек завершено 
	 } else {
             myGLCD.setColor(255, 0, 0);  
             myGLCD.setBackColor(0, 0, 0); 
             myGLCD.print(print_text[199], CENTER, 158); }} // Ошибка открытия файла настроек 
         
   break;   
   case 32:   //------------ Ограничение Мощности  ------------

      if ((x>=95) && (x<=135) && (y>=185) && (y<=205)){   // ok
            waitForIt (95, 185, 135, 205);
          setLEDsDimPercentL=TempsetLEDsDimPercentL;
          DimmL=1; SaveDimmLEDToEEPROM();
          dispScreen=15; clearScreen(); generalSettingsScreen_2();}  

      if ((x>=185) && (x<=250) && (y>=185) && (y<=205)){  // cancel
           waitForIt (185, 185, 250, 205); 
          dispScreen=15; clearScreen(); generalSettingsScreen_2(); }

      if ((x>=175) && (x<=200) && (y>=107) && (y<=132)){  // press Minute UP button
          TempsetLEDsDimPercentL++; _delay_ms(60);
         setFont(LARGE, 255, 255, 255, 0, 0, 0);
         if (TempsetLEDsDimPercentL>=99){ TempsetLEDsDimPercentL=99;}
         if (TempsetLEDsDimPercentL>=10){ 
                myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);}
         else { myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);} } else 
        
      if ((x>=175) && (x<=200) && (y>=137) && (y<=162)){  // press Minute DOWN button
          TempsetLEDsDimPercentL--; _delay_ms(60);
         setFont(LARGE, 255, 255, 255, 0, 0, 0);
         if (TempsetLEDsDimPercentL<=1){ TempsetLEDsDimPercentL=1;}
         if (TempsetLEDsDimPercentL>=10){ myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);}
                                   else { myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(129, 126, 136, 142);
                myGLCD.fillRect(153, 126, 161, 142);} } 
      break;  
      case 33:  //=============================================================== подсветка экрана  ====================================================================
      
    if ((x>=165) && (x<=311) && (y>=151) && (y<=179)){     // загрузить настройки
          waitForIt (165, 151, 311, 179);
        ReadLCDbright();
      LCDbrigh(); } else  
      
    if ((x>=165) && (x<=311) && (y>=61) && (y<=89)){   // сохранить настройки
          waitForIt (165, 61, 311, 89); LCDbright = tmpLCDbright;
           byte bout = map(LCDbright, 0, 100, 2, 255); // 255
           analogWrite(LCDbrightPin, bout);
	  SaveLCDbrightToEEPROM();
           dispScreen=0; clearScreen(); mainScreen(true); } else 

     if ((x>=gseB[0]) && (x<=gseB[0]+29) && (y>=gseB[1]) && (y<=gseB[1]+145)){          
       if ((y>=gseB[1]) && (y<=gseB[1]-15)){ drawUpButtonSlide(gseB[0], gseB[1]-15);         // plus   gseB[1]-10     
        if (tmpLCDbright <100) tmpLCDbright++; } else 

     if ((y>=gseB[1]+23) && (y<=gseB[1]+122)) tmpLCDbright = gseB[1] + 127 - y; else 
       if ((y>=gseB[1]) && (y<=gseB[1]+140)){ drawDownButtonSlide(gseB[0], gseB[1]+140); // minus   gseB[1]+134 
        if (tmpLCDbright>0) tmpLCDbright--; }       
           byte bout = map(tmpLCDbright, 0, 100, 2, 255); // 255
           analogWrite(LCDbrightPin, bout); LCDbrigh(); }         
     break;


   
   case 34: // переключения между экранами график / таблица   
  if ((x>=5) && (x<=315) && (y>=50) && (y<=180)){      // возврат к таблице     
        dispScreen=8; clearScreen(); ledValuesScreen(); } else

  if ((x>=back[0]) && (x<=back[2]) && (y>=back[1]) && (y<=back[3])){      // press MORE COLORS
        waitForIt(back[0], back[1], back[2], back[3]);
        ReadLedFromEEPROM();  
        dispScreen=7; clearScreen(); ledColorViewScreen(); } else
     
  if ((x>=ledChV[0]) && (x<=ledChV[2]) && (y>=ledChV[1]) && (y<=ledChV[3])){  // press CHANGE 
        waitForIt(ledChV[0], ledChV[1], ledChV[2], ledChV[3]);
        ReadLedFromEEPROM();  
        dispScreen=9; clearScreen(); ledChangeScreen(); } else
      
  if ((x>=eeprom[0]) && (x<=eeprom[2]) && (y>=eeprom[1]) && (y<=eeprom[3])){  // press SAVE 
        waitForIt(eeprom[0], eeprom[1], eeprom[2], eeprom[3]);
        SaveLEDToEEPROM(); 
        dispScreen=7; clearScreen(); ledColorViewScreen(); } else 
             
   if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3])){         // press CANCEL
        waitForIt(canC[0], canC[1], canC[2], canC[3]);
        ReadFromEEPROM(); LEDtestTick = false;
        dispScreen=0; clearScreen(); mainScreen(true); } 
        
   break;
   case 35: // настройка звуковой тревоги при перегреве радиатора (case 16:)

   if ((x>=95) && (x<=135) && (y>=185) && (y<=205)){    // ok
        waitForIt (95, 185, 135, 205);
        SaveTempToEEPROM(); 
        dispScreen=16; clearScreen(); ChangeFanTempsScreen(true);}  

   if ((x>=175) && (x<=240) && (y>=185) && (y<=205)){   // cancel
        waitForIt (175, 185, 240, 205); 
        dispScreen=16; clearScreen(); ChangeFanTempsScreen(true); }

//------ звуковая тревога при перегреве радиатора 
 if ((x>=SoundATm[0]) && (x<=SoundATm[2])) {
    if ((y>=SoundATm[1]) && (y<=SoundATm[3])){                  // press Sound Alarm -
	  setTempToSoundAlarmC -= 1; _delay_ms(150);    
      if (setTempToSoundAlarmC ==254) {setTempToSoundAlarmC = 99;}
      if (setTempToSoundAlarmC <= 39.0) {setTempToSoundAlarmC = 255;} 
                     setSalarm(); } } else
                              
 if ((x>=SoundATp[0]) && (x<=SoundATp[2])) {        
     if ((y>=SoundATp[1]) && (y<=SoundATp[3])){                 // press Sound Alarm + 
	  setTempToSoundAlarmC += 1; _delay_ms(150);   
      if (setTempToSoundAlarmC ==256) {setTempToSoundAlarmC = 40;}
      if (setTempToSoundAlarmC > 99.0) {setTempToSoundAlarmC = 255;}  // OFF alarm
                     setSalarm(); } } 
                   
      break;
      case 36: // ДОЗАТОР УДО    
 

		//--------------- AUTOMATIC DOSER PAGE --------------                                                                                          ДОЗАТОР
      if ((x>=dos1b[0]) && (x<=dos1b[2]) && (y>=dos1b[1]) && (y<=dos1b[3]))      //press Feeding Time 1
        {
         waitForIt(dos1b[0], dos1b[1], dos1b[2], dos1b[3]);
         dozTime=1;
         dispScreen=37;
         clearScreen();
         setDoserTimesScreen();
        } else
      if ((x>=dos2b[0]) && (x<=dos2b[2]) && (y>=dos2b[1]) && (y<=dos2b[3]))    //press Feeding Time 2
        {
         waitForIt(dos2b[0], dos2b[1], dos2b[2], dos2b[3]);
         dozTime=2;
         dispScreen=37;
         clearScreen();
         setDoserTimesScreen();
        } else
      if ((x>=dos3b[0]) && (x<=dos3b[2]) && (y>=dos3b[1]) && (y<=dos3b[3]))    //press Feeding Time 3
        {
         waitForIt(dos3b[0], dos3b[1], dos3b[2], dos3b[3]);
         dozTime=3;
         dispScreen=37;
         clearScreen();
         setDoserTimesScreen();
        } else
      if ((x>=dos4b[0]) && (x<=dos4b[2]) && (y>=dos4b[1]) && (y<=dos4b[3]))  //press Feeding Time 4
        {
         waitForIt(dos4b[0], dos4b[1], dos4b[2], dos4b[3]);
         dozTime=4;
         dispScreen=37;
         clearScreen();
         setDoserTimesScreen();
        } else
      if ((x>=dosval1[0]) && (x<=dosval1[2]) && (y>=dosval1[1]) && (y<=dosval1[3]))    //КАЛИБРОВКА 1
        {
          waitForIt(dosval1[0], dosval1[1], dosval1[2], dosval1[3]);
          dispScreen=10;
          CalMode=1;
          clearScreen();
          doscalibrateScreen();
        } 
      if ((x>=dosval2[0]) && (x<=dosval2[2]) && (y>=dosval2[1]) && (y<=dosval2[3]))    //КАЛИБРОВКА 2
        {
          waitForIt(dosval2[0], dosval2[1], dosval2[2], dosval2[3]);
          dispScreen=10;
          CalMode=2;
          clearScreen();
          doscalibrateScreen();
        }        
      if ((x>=dosval3[0]) && (x<=dosval3[2]) && (y>=dosval3[1]) && (y<=dosval3[3]))    //КАЛИБРОВКА 3
        {
          waitForIt(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
          dispScreen=10;
          CalMode=3;
          clearScreen();
          doscalibrateScreen();
        }
      if ((x>=dosval4[0]) && (x<=dosval4[2]) && (y>=dosval4[1]) && (y<=dosval4[3]))    //КАЛИБРОВКА 4
        {
          waitForIt(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);
          dispScreen=10;
          CalMode=4;
          clearScreen();
          doscalibrateScreen();
        }


       break;
       case 37:  // УСТАНОВКА ВРЕМЕНИ ДОЗИРОВАНИЯ

		//------------ SET AUTOMATIC DOSER TIMES ------------
      if ((x>=back[0]) && (x<=back[2]) && (y>back[1]) && (y<=back[3]))          //press back    
        {
         waitForIt(back[0], back[1], back[2], back[3]);
         if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
         else { AM_PM=2;}
         dispScreen=36;
         clearScreen();
         autoDoserScreen();
        } else
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])) //press SAVE
        {
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
         if (setTimeFormat==1)
           {
            if ((rtcSetHr==0) && (AM_PM==2))
              { rtcSetHr+=12;}
            if (((rtcSetHr>=1) && (rtcSetHr<=11)) && (AM_PM==2))
              { rtcSetHr+=12;}  
            if (((rtcSetHr>=12) && (rtcSetHr<=23)) && (AM_PM==1))
              { rtcSetHr-=12;}           
           }                  
         if (dozTime==1)
           { dozPump1H=rtcSetHr; dozPump1M=rtcSetMin;}
         if (dozTime==2)
           { dozPump2H=rtcSetHr; dozPump2M=rtcSetMin;}
         if (dozTime==3)
           { dozPump3H=rtcSetHr; dozPump3M=rtcSetMin;}
         if (dozTime==4)
           { dozPump4H=rtcSetHr; dozPump4M=rtcSetMin;}        
         SaveDoseTimesToEEPROM();
         dispScreen=36;
         clearScreen();
         autoDoserScreen();
        } else
      if ((x>=70) && (x<=250) && (y>=150) && (y<=170))   //Вкл выкл дозатора ON/OFF
        {
         waitForIt(70, 150, 250, 170);
         if (dozTime==1) 
           { if (DOZTime1==1) { DOZTime1=0;} 
             else { DOZTime1=1;}}
         if (dozTime==2) 
           { if (DOZTime2==1) { DOZTime2=0;} 
             else { DOZTime2=1;}}
         if (dozTime==3) 
           { if (DOZTime3==1) { DOZTime3=0;} 
             else { DOZTime3=1;}}
         if (dozTime==4) 
           { if (DOZTime4==1) { DOZTime4=0;} 
             else { DOZTime4=1;}}
         dosingTimeOnOff();
        } 
      else      
        {
         if ((y>=houP[1]) && (y<=houP[3]))               //FIRST ROW
           {
            if ((x>=houP[0]) && (x<=houP[2]))            //press hour up
              {
               waitForIt(houP[0], houP[1], houP[2], houP[3]);
               rtcSetHr++;
               if (rtcSetHr>=24) 
                 { rtcSetHr=0; }
              }
            if ((x>=minP[0]) && (x<=minP[2]))            //press min up
              {
               waitForIt(minP[0], minP[1], minP[2], minP[3]);
               rtcSetMin=rtcSetMin+30;
               if (rtcSetMin>30) {rtcSetMin=0; }
              }
            if ((x>=ampmP[0]) && (x<=ampmP[2])           //press AMPM up
               && (setTimeFormat==1))         
              {
               waitForIt(ampmP[0], ampmP[1], ampmP[2], ampmP[3]);
               if (AM_PM==1) {AM_PM=2;}
               else {AM_PM=1;}
              }
           }
         if ((y>=houM[1]) && (y<=houM[3]))               //SECOND ROW
           {
            if ((x>=houM[0]) && (x<=houM[2]))            //press hour down
              {
               waitForIt(houM[0], houM[1], houM[2], houM[3]);             
               rtcSetHr--;
               if (rtcSetHr<0) 
                 { rtcSetHr=23; }
              }
            if ((x>=minM[0]) && (x<=minM[2]))            //press min down
              {
               waitForIt(minM[0], minM[1], minM[2], minM[3]);
               rtcSetMin=rtcSetMin-30;
               if (rtcSetMin<0) {rtcSetMin=30; }
               if (rtcSetMin<30) {rtcSetMin=0; }
              }
            if ((x>=ampmM[0]) && (x<=ampmM[2])           //press AMPM down
                && (setTimeFormat==1))        
              {
               waitForIt(ampmM[0], ampmM[1], ampmM[2], ampmM[3]);
               if (AM_PM==1) {AM_PM=2;}
               else {AM_PM=1;}
              }  
           } 
         setDoserTimesScreen(false);
        }

       
       break;
    
    
   
    case 38:   

		//--------------- AUTOMATIC FISH FEEDER PAGE --------------                                                                                          КОРМУШКА
      if ((x>=5) && (x<=155) && (y>=20) && (y<=40))      //press Feeding Time 1
        {
         waitForIt(5, 20, 155, 40);
         feedTime=1;
         dispScreen=39;
         clearScreen();
         setFeederTimesScreen();
        } else
      if ((x>=165) && (x<=315) && (y>=20) && (y<=40))    //press Feeding Time 2
        {
         waitForIt(165, 20, 315, 40);
         feedTime=2;
         dispScreen=39;
         clearScreen();
         setFeederTimesScreen();
        } else
      if ((x>=5) && (x<=155) && (y>=168) && (y<=188))    //press Feeding Time 3
        {
         waitForIt(5, 168, 155, 188);
         feedTime=3;
         dispScreen=39;
         clearScreen();
         setFeederTimesScreen();
        } else
      if ((x>=165) && (x<=315) && (y>=168) && (y<=188))  //press Feeding Time 4
        {
         waitForIt(165, 168, 315, 188);
         feedTime=4;
         dispScreen=39;
         clearScreen();
         setFeederTimesScreen();
        } else
      if ((x>=85) && (x<=235) && (y>=94) && (y<=114))    //press Feeding Fish Now!
        {
         waitForIt(85, 94, 235, 114);
         myGLCD.setColor(0, 255, 0);
         myGLCD.fillRoundRect(85, 94, 235, 114);
         myGLCD.setColor(255, 255, 255);
         myGLCD.drawRoundRect(85, 94, 235, 114); 
         setFont(SMALL, 0, 0, 0, 0, 255, 0);
         myGLCD.setFont(RusFont3);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181]))); 
         myGLCD.print(buffer, 126, 101);
         digitalWrite(autoFeeder, HIGH);
         delay(5000);
         myGLCD.setColor(153, 0, 102);
         myGLCD.fillRoundRect(85, 94, 235, 114);
         myGLCD.setColor(255, 255, 255);
         myGLCD.drawRoundRect(85, 94, 235, 114); 
         setFont(SMALL, 255, 255, 255, 153, 0, 102);
         myGLCD.setFont(RusFont3);
         strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[180]))); 
         myGLCD.print(buffer, 97, 101);
         digitalWrite(autoFeeder, LOW);
        } 

    break; 
    
   case 39:  

		//------------ SET AUTOMATIC FISH FEEDER TIMES ------------
      if ((x>=back[0]) && (x<=back[2]) && (y>back[1]) && (y<=back[3]))          //press back    
        {
         waitForIt(back[0], back[1], back[2], back[3]);
         if ((timeDispH>=0) && (timeDispH<=11)) { AM_PM=1;}
         else { AM_PM=2;}
         dispScreen=38;
         clearScreen();
         autoFeederScreen();
        } else
      if ((x>=prSAVE[0]) && (x<=prSAVE[2]) && (y>=prSAVE[1]) && (y<=prSAVE[3])) //press SAVE
        {
         waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
         if (setTimeFormat==1)
           {
            if ((rtcSetHr==0) && (AM_PM==2))
              { rtcSetHr+=12;}
            if (((rtcSetHr>=1) && (rtcSetHr<=11)) && (AM_PM==2))
              { rtcSetHr+=12;}  
            if (((rtcSetHr>=12) && (rtcSetHr<=23)) && (AM_PM==1))
              { rtcSetHr-=12;}           
           }                  
         if (feedTime==1)
           { feedFish1H=rtcSetHr; feedFish1M=rtcSetMin;}
         if (feedTime==2)
           { feedFish2H=rtcSetHr; feedFish2M=rtcSetMin;}
         if (feedTime==3)
           { feedFish3H=rtcSetHr; feedFish3M=rtcSetMin;}
         if (feedTime==4)
           { feedFish4H=rtcSetHr; feedFish4M=rtcSetMin;}        
         SaveFeedTimesToEEPROM();
         dispScreen=38;
         clearScreen();
         autoFeederScreen();
        } else
      if ((x>=70) && (x<=250) && (y>=150) && (y<=170))   //Feeding ON/OFF
        {
         waitForIt(70, 150, 250, 170);
         if (feedTime==1) 
           { if (FEEDTime1==1) { FEEDTime1=0;} 
             else { FEEDTime1=1;}}
         if (feedTime==2) 
           { if (FEEDTime2==1) { FEEDTime2=0;} 
             else { FEEDTime2=1;}}
         if (feedTime==3) 
           { if (FEEDTime3==1) { FEEDTime3=0;} 
             else { FEEDTime3=1;}}
         if (feedTime==4) 
           { if (FEEDTime4==1) { FEEDTime4=0;} 
             else { FEEDTime4=1;}}
         feedingTimeOnOff();
        } 
      else      
        {
         if ((y>=houP[1]) && (y<=houP[3]))               //FIRST ROW
           {
            if ((x>=houP[0]) && (x<=houP[2]))            //press hour up
              {
               waitForIt(houP[0], houP[1], houP[2], houP[3]);
               rtcSetHr++;
               if (rtcSetHr>=24) 
                 { rtcSetHr=0; }
              }
            if ((x>=minP[0]) && (x<=minP[2]))            //press min up
              {
               waitForIt(minP[0], minP[1], minP[2], minP[3]);
               rtcSetMin++;
               if (rtcSetMin>=60) {rtcSetMin=0; }
              }
            if ((x>=ampmP[0]) && (x<=ampmP[2])           //press AMPM up
               && (setTimeFormat==1))         
              {
               waitForIt(ampmP[0], ampmP[1], ampmP[2], ampmP[3]);
               if (AM_PM==1) {AM_PM=2;}
               else {AM_PM=1;}
              }
           }
         if ((y>=houM[1]) && (y<=houM[3]))               //SECOND ROW
           {
            if ((x>=houM[0]) && (x<=houM[2]))            //press hour down
              {
               waitForIt(houM[0], houM[1], houM[2], houM[3]);             
               rtcSetHr--;
               if (rtcSetHr<0) 
                 { rtcSetHr=23; }
              }
            if ((x>=minM[0]) && (x<=minM[2]))            //press min down
              {
               waitForIt(minM[0], minM[1], minM[2], minM[3]);
               rtcSetMin--;
               if (rtcSetMin<0) {rtcSetMin=59; } 
              }
            if ((x>=ampmM[0]) && (x<=ampmM[2])           //press AMPM down
                && (setTimeFormat==1))        
              {
               waitForIt(ampmM[0], ampmM[1], ampmM[2], ampmM[3]);
               if (AM_PM==1) {AM_PM=2;}
               else {AM_PM=1;}
              }  
           } 
         setFeederTimesScreen(false);
        }

    break;



 } } }   
   
void setup(void){  // ============ SETUP
  Serial.begin(9600); // Setup usb serial connection to computer
  delay(100); Serial.println("Starting...");

  TCCR5B = (TCCR5B & 0xF8) | PWM_FRQ_Value_Fan; // 30hz  pin 44, 45, 46 for FANs

// set timer mode 14 - fast PWM
  TCCR4A = B00000010;		// mode 14Fast PWM timer4   
  TCCR3A = B00000010;		// mode 14Fast PWM timer3   
  TCCR1A = B00000010;		// mode 14Fast PWM timer1
  TCCR4B = B00011000;
  TCCR3B = B00011000;
  TCCR1B = B00011000;
// set prescaler value
  TCCR1B = TCCR1B | PWM_FRQ_Value;  // pin 11, 12
  TCCR3B = TCCR3B | PWM_FRQ_Value;  // pin 2, 3, 
  TCCR4B = TCCR4B | PWM_FRQ_Value;  // pin 6, 7, 8
  
  OCR1A = 0;  // 0 vary this value between 0 and 1024 for 10-bit precision
  OCR1B = 0;
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  ICR1 = 2005; // колличество шагов
  ICR3 = 2005; 
  ICR4 = 2005;

  cbi_mix( PORTB, 5 ); // Timer1, port 11 
  sbi_mix( DDRB , 5 ); 
  cbi_mix( PORTB, 6 ); // Timer1, port 12
  sbi_mix( DDRB , 6 ); 
  cbi_mix( PORTE, 3 ); // Timer3, port 5
  sbi_mix( DDRE , 3 ); 
  cbi_mix( PORTE, 4 ); // Timer3, port 2
  sbi_mix( DDRE , 4 ); 
  cbi_mix( PORTE, 5 ); // Timer3, port 3
  sbi_mix( DDRE , 5 ); 
  cbi_mix( PORTH, 3 ); // Timer4, port 6
  sbi_mix( DDRH , 3 ); 
  cbi_mix( PORTH, 4 ); // Timer4, port 7 
  sbi_mix( DDRH , 4 ); 
  cbi_mix( PORTH, 5 ); // Timer4, port 8
  sbi_mix( DDRH , 5 ); 


  pinMode(ledPinWhite, OUTPUT);   // white
  pinMode(ledPinBlue, OUTPUT);    // blue
  pinMode(ledPinRoyBlue, OUTPUT); // royal
  pinMode(ledPinRed, OUTPUT);     // red
  pinMode(ledPinUV, OUTPUT);      // uv
  pinMode(ledPinOrange, OUTPUT);  // oLed 
  pinMode(ledPinGr, OUTPUT);      // green
  
  pinMode(ledPinMoon, OUTPUT);    // Пин луны
  
  pinMode(LCDbrightPin, OUTPUT);  // подсветка экрана
 
  pinMode(tempHeatPin, OUTPUT);     // нагреватель
  pinMode(tempChillPin, OUTPUT);    // холодильник
  pinMode(tempAlarmPin, OUTPUT);    // тревога
  pinMode(autoFeeder, OUTPUT);
  pinMode(Heatsink1_FansPWM, OUTPUT); // вентилятор 1 (шим)
  pinMode(Heatsink2_FansPWM, OUTPUT); // вентилятор 2 (шим)   
  
  // Таймеры 
  pinMode(timer1, OUTPUT);
  pinMode(timer2, OUTPUT);  
  pinMode(timer3, OUTPUT);
  pinMode(timer4, OUTPUT); 
  pinMode(timer5, OUTPUT); 
  // Дозаторы
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);
  
  pinMode(vacpump, OUTPUT);


         
  ReadDallasAddress ();   
  sensors.begin();         // start up temperature library
  sensors.setResolution(waterThermometer, resolution);     // set the resolution to 11 bit
  sensors.setResolution(Heatsink1Thermometer, resolution);
  sensors.setResolution(Heatsink2Thermometer, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();  
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis();
 
  SetRoomTemperataureResolutionBits(12);//12 bits room temp resolution in celcius
 
  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.clrScr(); 
  
  EraseAllEEPROM_SaveDefault();  // check and erase eeprom for my format, save all default
  myGLCD.clrScr();
  
  myTouch.InitTouch(LANDSCAPE);
  myTouch.setPrecision(PREC_MEDIUM); // LOW / MEDIUM / HIGH 
  myTouch.read();                    // dummy read, enale IRQ 
  
  myGLCD.setColor(30, 30, 30);
  myGLCD.fillRect(0, 226, 319, 239);  // нижний Bar

  RTC.startClock();                    // Set the clock to run-mode
  RTC.getTime();  
  min_cnt= (RTC.hour*60)+RTC.minute;
  timer= (RTC.hour*60)+RTC.minute;
  
  sec_cnt=(RTC.minute*60)+RTC.second;

  COLOR=0;               // read all colors
  ReadLedFromEEPROM();   // read led setting from EEPROM 
  ReadFromEEPROM();      // считать из памяти все настройки
  LED_levelo_output(); 
  checkTempC();     
 // feed_output();   
  mainScreen(true);
   titledate();         // отображение даты в нижнем баре
   
 calculateStartTime();  // calculate SUNRISE time 
 calculateStopTime();   // calculate SUNSET time

 periode = 50000;        // time in ms from 0-255 or 0-255 
 starttime = millis(); 

 dataSerial.begin(9600); 
} // change nothing below here


void UART_receiver()
{
  if (dataSerial.available())
  {
    DataContainer data;
    data.setPH(avgMeasuredPH);
    data.setLight(100);
    data.setLevel(100);
    data.setTemperature(tempW);
    data.setCRC();

    uint8_t command[AQUARIUM_REQUEST_LENGTH + 1];
    int iCount = dataSerial.readBytes(command, AQUARIUM_REQUEST_LENGTH + 1);

    if (command[0] == 0xff)
    {
      if (command[AQUARIUM_REQUEST_LENGTH] == AQUARIUM_checksum(command, AQUARIUM_REQUEST_LENGTH))
      {
        dataSerial.write((uint8_t *)&data, AQUARIUM_RESPONSE_LENGTH);
        dataSerial.flush();
      }
      else
        Serial.println("AQUARIUM Checksum doesn't match");

      //serialPrintArray("COMMAND: ", (char *)command, iCount);
      //serialPrintArray("DATA: ", (char *)&data, AQUARIUM_RESPONSE_LENGTH);
    }
  }  
}
 
void loop(void){ // --------------------------------- loop
 
  if (aclock == 1){ analogClock(); }      // cкринсейв аналоговые часы (секундная стрелка) 
  
 
// Выход из хранителя экрана   
  if ((myTouch.dataAvailable()) && (screenSaverCounter>=setScreenSaverTimer)){  
        LEDtestTick = false;        
        screenSaverCounter=0; myGLCD.clrScr(); // выход из хранителя экрана
        myGLCD.setColor(30, 30, 30);
        myGLCD.fillRect(0, 226, 319, 239);    // заполнение Bar
        myGLCD.setColor(64, 64, 64);
        myGLCD.drawRect(0, 226, 319, 239);    // рамка
        
   mainScreen(true); dispScreen=0; aclock = 0;}    // счетчик возврата в главный экран
    else { if (myTouch.dataAvailable()) { processMyTouch();}}

//----------check LED levels every 1s for 8/11 & 11 bit version ---------------------------
  unsigned long currentMillis = millis();
	if (currentMillis - previousMillisOne > 1000){   // check LED levels каждую секунду
	      previousMillisOne = currentMillis;  
	        min_cnt= (RTC.hour*60)+RTC.minute; 
		   LED_levelo_output();  
                   dosingTimeOutput();
          //pumpPWM(); // цикл для помп в режиме шим

     RTC.getTime();
if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){      
  if (RTC.hour==0 && RTC.minute==0 && RTC.second==0){   // в полночь очистить область даты 
     setFont(SMALL, 30, 30, 30, 30, 30, 30);  
      myGLCD.fillRect(0,227,215,238);
      myGLCD.print(F("                          "), 2, 227); } } }

  if (currentMillis - previousMillisFive > 5000){ // проверка времени, температуры и уровней каналов, каждые 5 сек  
                      previousMillisFive = currentMillis;  
                            RTC.getTime();

//------------------------                  
  if (screenSaverCounter<setScreenSaverTimer){ TimeDateBar(); titledate();}
                 

      checkTempC();                   // проверка температуры
      timer=(RTC.hour*60)+RTC.minute;        // проверка таймеров
      light();                        // срабатывание таймеров (проверка каждые 5сек)
      lightdraw();                    // чтение состояния таймеров и отображение состояний 
      Speed_Fan();                    // скорость вентеляторов на радиатое (в главном экране) 
      feedingTimeOutput();
      waterlevels();
      caldosetime();
      #ifdef PH_sensor_I2C
     CheckPH();
      #endif
      UART_receiver();
      
//------------ Отключить пресеты за 45 минут до завершения светового дня -------------------
     if (((GlobalStatus2Byte & 0x0F) !=0) && (min_cnt/15 == StopTime-3 )){   // !=0 if preset is ON     
	   GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0);                   // clear flags Preset1..Preset4  
	  colorLEDtest = false; } screenReturn(); screenSaver(); checkTempC();}
                
  if ((dispScreen == 0) && (screenSaverCounter<setScreenSaverTimer)){aclock = 0; mainScreen(); }}  // в основном цикле

/********************************** END OF MAIN LOOP *********************************/
