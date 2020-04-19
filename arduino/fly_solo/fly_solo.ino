#include <FastLED.h>
#include <Servo.h>

#define SOLENOID 2
#define PUMP 9
#define BUTTON 12
#define JOYX A0
#define JOYY A1
#define SERV1 3
#define SERV2 5
#define SERV3 6
#define OFF 0
#define LO 51
#define HI 100
#define DS 20
#define LIGHT A2
#define TEMP A3
#define NEOPIXELPIN 8
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#define DATA_PIN NEOPIXELPIN
#define BRIGHTNESS 20
 
//DEFINE SDA AND SCL PINS
  #define SCL_PIN 0                 //COMMUNICATION PIN 20 ON MEGA
  #define SCL_PORT PORTD

  #define SDA_PIN 1                 //COMMUNICATION PIN 21 ON MEGA
  #define SDA_PORT PORTD

//CONFIGURE I2C MODES
  #define I2C_TIMEOUT 100           //PREVENT SLAVE DEVICES FROM STRETCHING LOW PERIOD OF THE CLOCK INDEFINITELY AND LOCKING UP MCU BY DEFINING TIMEOUT
  //#define I2C_NOINTERRUPT 1       //SET TO 1 IF SMBus DEVICE CAN TIMEOUT
  //#define I2C_FASTMODE 1          //THE STANDARD I2C FREQ IS 100kHz.  USE THIS TO PERMIT FASTER UP TO 400kHz.
  //#define I2C_SLOWMODE 1            //THE STANDARD I2C FREQ IS 100kHz.  USE THIS TO PERMIT SLOWER, DOWN TO 25kHz.
  #define BAUD_RATE 115200

/**********************************
 * CONFIGURE SERIAL LIBRARY
 **********************************/
  //#include <SoftwareSerial.h>
  //#include <Serial.h>
  #include <Wire.h>
#include <Servo.h>

#define SERV1 3
#define MIN_SERVO  0
#define MAX_SERVO  180
#define STEP_SERVO 18

#define DEBOUNCE_MS 100
#define JOY_MID 512
#define NULL_ZONE 50

bool on = false;

int update_servo(int value, int pin, Servo servo) {
  int delta = analogRead(pin) - JOY_MID;
  if (abs(delta) > NULL_ZONE && on) {
    //Serial.print("Incoming value is:  ");
    //Serial.println(value);
    //Serial.print("Current raw delta is:  ");
    //Serial.println(delta);
    //  Upper bound of (map) is never hit, so we need to do -N, N+1 below...
    delta = map(delta, -JOY_MID, JOY_MID, -1, 2);
    //Serial.print("Current mapped delta is:  ");
    //Serial.println(delta);
    value = constrain(value + delta, LO, HI);
    
    Serial.print("New value is:  ");
    Serial.println(value);
    servo.write(value);
    delay(30);
  }
  if(digitalRead(BUTTON) == LOW){
    if(on){
      on = false;
      servo.write(OFF);
      leds[0] = CRGB::Black;
    }
    else{
      on = true;
      servo.write(value);
      leds[0] = CRGB::Red;
    }
    FastLED.show();
    while(digitalRead(BUTTON) == LOW){
      delay(100);
    }
  }
  return value;
}

/**********************************
 * DEFINE VARIABLES AND SMBus MAPPINGS
 **********************************/
  #define BATT_SMBUS_ADDR                     0x0B                ///< I2C address
  #define BATT_SMBUS_ADDR_MIN                 0x08                ///< lowest possible address
  #define BATT_SMBUS_ADDR_MAX                 0x7F                ///< highest possible address
//BUS MAPPINGS FROM DEV.3DR
  #define BATT_SMBUS_TEMP                     0x08                ///< temperature register
  #define BATT_SMBUS_VOLTAGE                  0x09                ///< voltage register
  #define BATT_SMBUS_REMAINING_CAPACITY       0x0f                ///< predicted remaining battery capacity as a percentage
  #define BATT_SMBUS_FULL_CHARGE_CAPACITY     0x10                ///< capacity when fully charged
  #define BATT_SMBUS_DESIGN_CAPACITY          0x18                ///< design capacity register
  #define BATT_SMBUS_DESIGN_VOLTAGE           0x19                ///< design voltage register
  #define BATT_SMBUS_SERIALNUM                0x1c                ///< serial number register
  #define BATT_SMBUS_MANUFACTURE_NAME         0x20                ///< manufacturer name
  #define BATT_SMBUS_MANUFACTURE_DATA         0x23                ///< manufacturer data
  #define BATT_SMBUS_MANUFACTURE_INFO         0x25                ///< cell voltage register
  #define BATT_SMBUS_CURRENT                  0x2a                ///< current register
  #define BATT_SMBUS_MEASUREMENT_INTERVAL_US  (1000000 / 10)      ///< time in microseconds, measure at 10hz
  #define BATT_SMBUS_TIMEOUT_US               10000000            ///< timeout looking for battery 10seconds after startup
  #define BATT_SMBUS_BUTTON_DEBOUNCE_MS       300                 ///< button holds longer than this time will cause a power off event
 
  #define BATT_SMBUS_PEC_POLYNOMIAL           0x07                ///< Polynomial for calculating PEC
  #define BATT_SMBUS_I2C_BUS                  PX4_I2C_BUS_EXPANSION
//BUS MAPPINGS FROM SMBus PROTOCOL DOCUMENTATION
#define BATTERY_MODE             0x03
#define CURRENT                  0x0A
#define RELATIVE_SOC             0x0D
#define ABSOLUTE_SOC             0x0E
#define TIME_TO_FULL             0x13
#define CHARGING_CURRENT         0x14
#define CHARGING_VOLTAGE         0x15
#define BATTERY_STATUS           0x16
#define CYCLE_COUNT              0x17
#define SPEC_INFO                0x1A
#define MFG_DATE                 0x1B
#define DEV_NAME                 0x21   // String
#define CELL_CHEM                0x22   // String
#define CELL4_VOLTAGE            0x3C   // Indidual cell voltages don't work on Lenovo and Dell Packs
#define CELL3_VOLTAGE            0x3D
#define CELL2_VOLTAGE            0x3E
#define CELL1_VOLTAGE            0x3F
#define STATE_OF_HEALTH          0x4F
//END BUS MAPPINGS
 
  #define bufferLen 32
  uint8_t i2cBuffer[bufferLen];

// standard I2C address for Smart Battery packs
  byte deviceAddress = BATT_SMBUS_ADDR;


Servo control;
void setup(){
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  pinMode(BUTTON, INPUT_PULLUP);

  //INITIATE SERIAL CONSOLE
    Serial.begin(BAUD_RATE);
    Wire.begin();

    Serial.flush();

    while (!Serial) {
    ;                                                       //wait for Console port to connect.
    }

    Serial.println("Console Initialized");
 
    Serial.println("I2C Inialized");
    scan();
    control.attach(SERV1);
    control.write(OFF);
    control.write(LO);
    delay(50);
    control.write(OFF);
    if(false){
      for(int setting = LO; setting <= HI; setting+=DS){
	if(setting/DS % 2){
	  leds[0] = CRGB::Black;
	}
	else{
	  leds[0] = CRGB::Red;
	}
	FastLED.show();
	control.write(setting);
	delay(500);
      }
      for(int setting = HI; setting >= LO; setting-=DS){
	if(setting/DS % 2){
	  leds[0] = CRGB::Black;
	}
	else{
	  leds[0] = CRGB::Green;
	}
	FastLED.show();
	control.write(setting);
	delay(500);
      }
    }
}

int fetchWord(byte func){
  byte b1, b2;
  
  Wire.requestFrom(deviceAddress, (byte)3);
  if(Wire.available() > 1){
    b1 = Wire.read();
    b2 = Wire.read();
  }
  return (int)b1|((( int)b2)<<8);
}
 
uint8_t i2c_smbus_read_block ( uint8_t command, uint8_t* blockBuffer, uint8_t blockBufferLen){
  return 0;
}

void scan(){
}

int setting = LO;
void loop(){
  setting = update_servo(setting, JOYX, control);
}
