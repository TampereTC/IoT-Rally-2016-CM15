// IOT RalliDroid  
// Copyright Jukka Raivio and Jussi Salonen 2016
// MIT License
// 
// Example code is based on Arduino Mega2560 m
//
// This arduino demo setup is connected to esp8266 Wifi via serial. 
// The http services are managed by esp-link firmware in ESP. The esp unit is working as Wifi-Serial bridge.
// No code changes are required for the esp-link
// Github repository for esp-link https://github.com/jeelabs/esp-link

// Temperature & Humidity sensor DHT11 lib & setup
// Adafruit version, https://github.com/adafruit/DHT-sensor-library
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
#include "DHT.h"
#define DHTPIN  2       // 2 digital pin 
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);


// I2C library
#include <Wire.h>
// RTC library by adafruit https://github.com/adafruit/RTClib
#include "RTClib.h"
RTC_DS1307 RTC;
String TimeStr ="";

// Multitasking functions by Arduino millis
// Initialization to enable multithreading features by using millis() functions  
// Sensor update interval
unsigned long previousMillis = 0;       // will store last time sensor updated
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
const long interval = 10000;            // Sensor data sending interval

// Json parser lib Copyright Benoit Blanchon 2014-2016, MIT License
// https://github.com/bblanchon/ArduinoJson
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
#include <ArduinoJson.h>
const char* command;
// Serial data handling, global variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char inChar; // a char to handle incoming data

// RFID tag reader (NFC) MFRC522
/* ------------------------------------
  * RFID Card reader MFRC522 library by Miguel Balboa https://github.com/miguelbalboa/rfid
  * Signal      MFRC522      Mega 2560
  *             Pin          Pin       ´
  * -------------------------------------
  * RST/Reset   RST          49         
  * SPI SS      SDA(SS)      53       
  * SPI MOSI    MOSI         51        
  * SPI MISO    MISO         50        
  * SPI SCK     SCK          52        
*/
#include <SPI.h> // Arduino default SPI library
#include <MFRC522.h> 
#define RST_PIN   49     // Configurable, see typical pin layout above (valk)
#define SS_PIN    53    // Configurable, see typical pin layout above (vihr)
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[3];

/*
 * Gyroscope + accelerometer MPU-6050 library for Arduino by Jeff Rowberg
 * https://github.com/jrowberg/i2cdevlib
 * 
 */
#include <I2Cdev.h>
#include <MPU6050.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// We use I2C address 0x69 as the default (0x68) is used by RTC
// Due to this AD0 pin must be pulled high
MPU6050 accelgyro(0x69);

// Global variables to store the values
int16_t ax, ay, az;
int16_t gx, gy, gz;


// Ultrasonic HC-SR04 library for Arduino by J.Rodrigo https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
// Max distance 51 cm
#include <Ultrasonic.h>
#define TRIG_PIN 10     // Configurable Arduino pins for HC-SR04 Trig PIN
#define ECHO_PIN 9  // Configurable Arduino pins for HC-SR04 Echo pins
Ultrasonic ultrasonic(TRIG_PIN,ECHO_PIN); 

// Edge sensor pinouts
#define right_edge 23 // Right sensor
#define left_edge 22 // Left sensor



// Motor driver Setup
// Pins L298N -> Mega board
int enA = 8;
int in1 = 4;
int in2 = 5;
int in3 = 6;
int in4 = 7;
int enB = 3;
boolean motor_active = true;
boolean auto_drive = true;
// for motor delay
unsigned long mStartMillis = 0;      // will store last time motor delay update
int course; // direction
int mspeed; // motor speed
int mdelay; // motor delay
// millis for sensor reading interval while moving, delay 500 ms
boolean sensor_moving = false;
int sensor_moving_reports = 50;
unsigned long sensor_moving_previousMillis = 0;      // will store last time light delay update

// light blinking 
boolean pin13_blinking = false;
boolean pin12_blinking = false;
boolean pin11_blinking = false;
int pin13_delay;
int pin12_delay;
int pin11_delay;
unsigned long pin13_previousMillis = 0;      // will store last time light delay update
unsigned long pin12_previousMillis = 0;      // will store last time light delay update
unsigned long pin11_previousMillis = 0;      // will store last time light delay update

#define  C0 16.35
#define Db0 17.32
#define D0  18.35
#define Eb0 19.45
#define E0  20.60
#define F0  21.83
#define Gb0 23.12
#define G0  24.50
#define Ab0 25.96
#define LA0 27.50
#define Bb0 29.14
#define B0  30.87
#define C1  32.70
#define Db1 34.65
#define D1  36.71
#define Eb1 38.89
#define E1  41.20
#define F1  43.65
#define Gb1 46.25
#define G1  49.00
#define Ab1 51.91
#define LA1 55.00
#define Bb1 58.27
#define B1  61.74
#define C2  65.41
#define Db2 69.30
#define D2  73.42
#define Eb2 77.78
#define E2  82.41
#define F2  87.31
#define Gb2 92.50
#define G2  98.00
#define Ab2 103.83
#define LA2 110.00
#define Bb2 116.54
#define B2  123.47
#define C3  130.81
#define Db3 138.59
#define D3  146.83
#define Eb3 155.56
#define E3  164.81
#define F3  174.61
#define Gb3 185.00
#define G3  196.00
#define Ab3 207.65
#define LA3 220.00
#define Bb3 233.08
#define B3  246.94
#define C4  261.63
#define Db4 277.18
#define D4  293.66
#define Eb4 311.13
#define E4  329.63
#define F4  349.23
#define Gb4 369.99
#define G4  392.00
#define Ab4 415.30
#define LA4 440.00
#define Bb4 466.16
#define B4  493.88
#define C5  523.25
#define Db5 554.37
#define D5  587.33
#define Eb5 622.25
#define E5  659.26
#define F5  698.46
#define Gb5 739.99
#define G5  783.99
#define Ab5 830.61
#define LA5 880.00
#define Bb5 932.33
#define B5  987.77
#define C6  1046.50
#define Db6 1108.73
#define D6  1174.66
#define Eb6 1244.51
#define E6  1318.51
#define F6  1396.91
#define Gb6 1479.98
#define G6  1567.98
#define Ab6 1661.22
#define LA6 1760.00
#define Bb6 1864.66
#define B6  1975.53
#define C7  2093.00
#define Db7 2217.46
#define D7  2349.32
#define Eb7 2489.02
#define E7  2637.02
#define F7  2793.83
#define Gb7 2959.96
#define G7  3135.96
#define Ab7 3322.44
#define LA7 3520.01
#define Bb7 3729.31
#define B7  3951.07
#define C8  4186.01
#define Db8 4434.92
#define D8  4698.64
#define Eb8 4978.03
// DURATION OF THE NOTES 
#define BPM 120    //  you can change this value changing all the others
#define H 2*Q //half 2/4
#define Q 60000/BPM //quarter 1/4 
#define E Q/2   //eighth 1/8
#define S Q/4 // sixteenth 1/16
#define W 4*Q // whole 4/4



void JsonReportSensorDHT() {
 
  // DHT functions
  // Reading temperature or humidity takes about 250 milliseconds!
  float h = dht.readHumidity();  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);
    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
        // Debugging
        //Serial.println("Failed to read from DHT sensor!");
        return;
    }
    // Create Json for sensor print out
    StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
    String rootJson = ""; String arrayJson = "";
    JsonObject& root = jsonOutBuffer.createObject();
    root["sensor"] = "temp_hum"; root["time"] =  TimeStr; JsonArray& array = jsonOutBuffer.createArray();
    array.add(t); array.add(h);
    // Print to Serial
    root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
    Serial1.println(JointJson);
    return;
}

void JsonReportSensorDistance(){

  StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  root["sensor"] = "distance"; root["time"] =  TimeStr; JsonArray& array = jsonOutBuffer.createArray();
  array.add(ultrasonic.Ranging(CM)); // CM or INC
    // Debugging
    //Serial.print(ultrasonic.Ranging(CM)); // CM or INC
    //Serial.println(" cm" ); 
  root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
    // Debugging
    //Serial.println("json string for edge:" + JointJson);
  Serial1.println(JointJson);
  return;
}  
/*
 *   Read accelerometer and gyroscope raw values and send them
 *   to ESP
 * 
 */
void JsonReportSensorAccAndGyro(){
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Construct json
  StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; 
  String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  
  root["sensor"] = "acc_gyro"; 
  root["time"] =  TimeStr;
  JsonArray& array = jsonOutBuffer.createArray();
  array.add( ax );
  array.add( ay );
  array.add( az );
  array.add( gx );
  array.add( gy );
  array.add( gz );
  
  root.printTo(rootJson); 
  array.printTo(arrayJson); 
  String JointJson = rootJson + ":" + arrayJson + "}";
  
  // Debuggung
  //Serial.println("json string for edge:" + JointJson);
  
  // Send to ESP
  Serial1.println(JointJson);
}

void JsonReportSensorEdge() {
   StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  root["sensor"] = "edge"; 
  root["time"] =  TimeStr;
  JsonArray& array = jsonOutBuffer.createArray();
  if(digitalRead(left_edge) != digitalRead(right_edge))
  {
    stopMotors();
  }
  array.add( digitalRead(left_edge) );
  array.add( digitalRead(right_edge) );
  
  root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
  // Debuggung
  //Serial.println("json string for edge:" + JointJson);
  Serial1.println(JointJson);
  return;
}  
void JsonReportSensorRFID() {
  if ( ! rfid.PICC_ReadCardSerial()) 
    return;
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));
  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }
  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));
    // Define Json for sensor print out
    StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
    String rootJson = "";
    String arrayJson = "";
    JsonObject& root = jsonOutBuffer.createObject();
    root["sensor"] = "rfid";
    JsonArray& array = jsonOutBuffer.createArray();
    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
      }   
    //Serial.println(F("The NUID tag is:"));
    //Serial.print(F("In dec: "));
    byte *buffer = rfid.uid.uidByte; 
    byte bufferSize = rfid.uid.size;
    String NUID = "";
    for (byte i = 0; i < bufferSize; i++) {
      //Serial.print(buffer[i] < 0x10 ? " 0" : " ");
      //Serial.print(buffer[i], DEC);
      NUID = String(NUID + String(buffer[i], DEC)); 
      }
    array.add(NUID);
    // Print json string to Serial1
    root.printTo(rootJson);
    array.printTo(arrayJson);
    String JointJson = rootJson + ":" + arrayJson + "}";
    //Serial.println("json string for rfid:" + JointJson);
        play();
    Serial1.println(JointJson);
    }
  else Serial.println(F("Card read previously."));
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  return;  
}

void driveMotors()
{
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
    // this function will run the motors across the range of possible speeds
    // note that maximum speed is determined by the motor itself and the operating voltage
    // the PWM values sent by analogWrite() are fractions of the maximum speed possible by your hardware
      // Drive Forward  
      if (course == 1) {
        digitalWrite(13, HIGH);
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed * 1.002);
      }
      // Drive Backward
      else if (course == 2 ) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH); digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Turn Right, direction forward
      else if (course == 3 ) {
        digitalWrite(11, HIGH);
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Turn Left, direction forward
      else if (course == 4 ) {
        digitalWrite(12, HIGH);
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Stop
      else if (course == 0) {
        // now turn off motors
        digitalWrite(in1, LOW); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, LOW);  
      }  
}

void stopMotors() {
      // now turn off motors
    // clean up & return
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    course = NULL;
    mspeed = NULL;
    mdelay = NULL;
    motor_active = false;
    // return;
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent1() {
  while (Serial1.available()> 0) {
    inChar = (char)Serial1.read(); // get the new byte: 
    inputString += inChar; // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
        stringComplete = true;
    }
  }
}
void HandleIncomingJson() {
    StaticJsonBuffer<512> jsonInBuffer;                 
    const char *JsonChar = inputString.c_str(); // 1 KB
    JsonObject& root = jsonInBuffer.parseObject(JsonChar);
    int pin;
    int value;
    int ldelay;
    
    // Verify Json 
    if (JsonChar!=NULL && !root.success()) {
      Serial.println("parseObject() failed: ");
      Serial.println(JsonChar);
    }
    else {
        // Led pins 13-11
        command = root["command"];
        pin = root["data"][0];
        value = root["data"][1];
        ldelay = root["data"][2];
        course = root["mdata"][0];
        mspeed = root["mdata"][1];
        mdelay = root["mdata"][2];
        if(course != 5)
        {
         auto_drive = false;  
        }
        else if(course == 5)
        {
          auto_drive = true;
        }
        
    if (command="lights") {
    // Lights on, not blinking
      if (value == 1 && (ldelay == 0 || ldelay == NULL)){
        digitalWrite(pin, HIGH);
        if (pin == 13) { pin13_blinking = false; }
        if (pin == 12) { pin12_blinking = false; }   
        if (pin == 11) { pin11_blinking = false; }                
      }
      // blinking lights
        else if (value == 1 && (!ldelay == 0 || !ldelay == NULL)){
          if (pin == 13) { pin13_blinking = true; pin13_delay = ldelay;}
          if (pin == 12) { pin12_blinking = true; pin12_delay = ldelay;}   
          if (pin == 11) { pin11_blinking = true; pin11_delay = ldelay;}              
        }
        // Lights off
        else if (value == 0 ){
          digitalWrite(pin, LOW);
          if (pin == 13) { pin13_blinking = false; }
            if (pin == 12) { pin12_blinking = false; }   
            if (pin == 11) { pin11_blinking = false; }  
          }
        }
    if (command ="drive") {motor_active = true; mStartMillis = millis();}
    }
  // returning the default state of serialEvent1()
  stringComplete = false;
  // clean json incoming data buffers
  pin = NULL; value = NULL; ldelay = NULL;
  inputString="";
  inChar = NULL; JsonChar = NULL;
  return;      
}

void readTime() {

    DateTime now = RTC.now();
    String Year = String(now.year(), DEC);
    String Month = String(now.month(), DEC);
    String Day = String(now.day(), DEC);
    String Hour = String(now.hour(), DEC);
    String Minutes = String(now.minute(), DEC);
    String Seconds = String(now.second(), DEC);
    TimeStr = Year + "/" + Month + "/" + Day + ":" + Hour + ":" + Minutes + ":" + Seconds;
    // Debugging
    // Serial.print(TimeStr);
    // Serial.println();

}

void followLine()
{
  mdelay = 50;
  mspeed = 70;
  if(digitalRead(right_edge) == HIGH && digitalRead(left_edge) == HIGH)
  {
    course = 1;
  }
   else if(digitalRead(right_edge) == LOW && digitalRead(left_edge) == LOW)
  {
    
    mspeed = 90;
    course = 4;
  }
  else if(digitalRead(right_edge) == LOW)
  {
    mspeed = 90;
    course = 3;    
  }
  else if(digitalRead(left_edge) == LOW)
  {
    mspeed = 90;
    course = 4;
  }
  driveMotors();  
}


void play()
{
  course = 3;
  int pin = 24;
//tone(pin, note, duration)
    tone(pin,LA3,Q); 
    delay(1+Q); //delay duration should always be 1 ms more than the note in order to separate them.
    tone(pin,LA3,Q);
    delay(1+Q);
    tone(pin,LA3,Q);
    delay(1+Q);
    tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    
    tone(pin,LA3,Q);
    delay(1+Q);
    tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,LA3,H);
    delay(1+H);
    
    tone(pin,E4,Q); 
    delay(1+Q); 
    tone(pin,E4,Q);
    delay(1+Q);
    tone(pin,E4,Q);
    delay(1+Q);
    tone(pin,F4,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    
    tone(pin,Ab3,Q);
    delay(1+Q);
    tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,LA3,H);
    delay(1+H);
    
    tone(pin,LA4,Q);
    delay(1+Q);
    tone(pin,LA3,E+S);
    delay(1+E+S);
    tone(pin,LA3,S);
    delay(1+S);
    tone(pin,LA4,Q);
    delay(1+Q);
    tone(pin,Ab4,E+S);
    delay(1+E+S);
    tone(pin,G4,S);
    delay(1+S);
    
    tone(pin,Gb4,S);
    delay(1+S);
    tone(pin,E4,S);
    delay(1+S);
    tone(pin,F4,E);
    delay(1+E);
    delay(1+E);//PAUSE
    tone(pin,Bb3,E);
    delay(1+E);
    tone(pin,Eb4,Q);
    delay(1+Q);
    tone(pin,D4,E+S);
    delay(1+E+S);
    tone(pin,Db4,S);
    delay(1+S);
    
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,B3,S);
    delay(1+S);
    tone(pin,C4,E);
    delay(1+E);
    delay(1+E);//PAUSE QUASI FINE RIGA
    tone(pin,F3,E);
    delay(1+E);
    tone(pin,Ab3,Q);
    delay(1+Q);
    tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,LA3,S);
    delay(1+S);
    
    tone(pin,C4,Q);
    delay(1+Q);
     tone(pin,LA3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,E4,H);
    delay(1+H);
    
     tone(pin,LA4,Q);
    delay(1+Q);
    tone(pin,LA3,E+S);
    delay(1+E+S);
    tone(pin,LA3,S);
    delay(1+S);
    tone(pin,LA4,Q);
    delay(1+Q);
    tone(pin,Ab4,E+S);
    delay(1+E+S);
    tone(pin,G4,S);
    delay(1+S);
    
    tone(pin,Gb4,S);
    delay(1+S);
    tone(pin,E4,S);
    delay(1+S);
    tone(pin,F4,E);
    delay(1+E);
    delay(1+E);//PAUSE
    tone(pin,Bb3,E);
    delay(1+E);
    tone(pin,Eb4,Q);
    delay(1+Q);
    tone(pin,D4,E+S);
    delay(1+E+S);
    tone(pin,Db4,S);
    delay(1+S);
    
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,B3,S);
    delay(1+S);
    tone(pin,C4,E);
    delay(1+E);
    delay(1+E);//PAUSE QUASI FINE RIGA
    tone(pin,F3,E);
    delay(1+E);
    tone(pin,Ab3,Q);
    delay(1+Q);
    tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    
    tone(pin,LA3,Q);
    delay(1+Q);
     tone(pin,F3,E+S);
    delay(1+E+S);
    tone(pin,C4,S);
    delay(1+S);
    tone(pin,LA3,H);
    delay(1+H);
    
    delay(2*H);  
}

void setup() {
  pinMode(24, OUTPUT);   
  //pinMode(9, OUTPUT);       
  //digitalWrite(9,LOW);

    // initialize both serial ports:
    Serial.begin(9600); // Debugging
    delay(1000);
    Serial1.begin(115200); // Serial for esp8266
    delay(1000);
    Wire.begin(); // start I2C & RTC
    RTC.begin();
    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      //RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    // initialize digital pin 13-11 as an output (for LEDs)
    pinMode(13, OUTPUT); // Main lights leds
    pinMode(12, OUTPUT); // Direction lights
    pinMode(11, OUTPUT);
    // Egde sensor input pins
    pinMode(right_edge,INPUT); // Right sensor
    pinMode(left_edge,INPUT); // Left sensor
    inputString.reserve(256); // reserve 256 bytes for the inputString:
    dht.begin(); // Init DHT
    // RFID setup
    SPI.begin(); // Init SPI bus
    rfid.PCD_Init(); // Init MFRC522 
    for (byte i = 0; i < 6; i++) { key.keyByte[i] = 0xFF; } // RFID byte handling
    // HC-SR04 distance sensor setup (TBD, currently fixed VCC/GND)
    //pinMode(4, OUTPUT); // VCC pin
    //pinMode(7, OUTPUT); // GND ping
    //digitalWrite(4, HIGH); // VCC +5V mode  
    //digitalWrite(7, LOW);  // GND mode
}
void loop() {
    // update interval

    unsigned long currentMillis = millis();
    // DHT Sensor reporting
    if (currentMillis - previousMillis >= interval) {
      // save the last time of sensor reporting
      readTime();
      previousMillis = currentMillis;    
      JsonReportSensorDHT();
      if (motor_active == false) {
       JsonReportSensorDistance(); 
       JsonReportSensorEdge();
       JsonReportSensorAccAndGyro();
      } // slow down reporting frequency in static cases
      //JsonReportSensorACC();
    }
    if (motor_active == true) { // speed up reporting frequency in case of moving
      currentMillis = millis();
      if (currentMillis - mStartMillis >= sensor_moving_reports ) {
        readTime();
        JsonReportSensorDistance();
        JsonReportSensorEdge();
        JsonReportSensorAccAndGyro();
      }
    }
    // Look for new RFID cards
    if ( rfid.PICC_IsNewCardPresent()) { JsonReportSensorRFID(); }
    // Handling incoming Json from serial port 
    // JSon serial read, print the string when a newline arrives:
    serialEvent1(); if (stringComplete == true) { HandleIncomingJson();}
    
    // Light blinking
    if ( pin13_blinking == true){
      currentMillis = millis();
      if ( currentMillis - pin13_previousMillis >= pin13_delay ) {
          pin13_previousMillis = currentMillis; 
          if (digitalRead(13) == LOW) { digitalWrite(13, HIGH);
          } else { digitalWrite(13, LOW); }
        }
    }                       
    if ( pin12_blinking == true){
      currentMillis = millis();
      if (currentMillis - pin12_previousMillis >= pin12_delay) {
        pin12_previousMillis = currentMillis; 
        if (digitalRead(12) == LOW) { 
          digitalWrite(12, HIGH);
        } else { 
          digitalWrite(12, LOW); }
        }             
    }
    if ( pin11_blinking == true){
      currentMillis = millis();
      if ( currentMillis - pin11_previousMillis >= pin11_delay) {
        pin11_previousMillis = currentMillis; 
        if (digitalRead(11) == LOW) { 
          digitalWrite(11, HIGH);
        } else { 
          digitalWrite(11, LOW); }
        }             
    }     
    // Drive motors

   if (motor_active == true || auto_drive) {
       
       currentMillis = millis();
       if(auto_drive)
       {
          followLine();
       }
       else
       {
          driveMotors();
        if (currentMillis - mStartMillis >= mdelay) {       
        stopMotors(); }     
       }
      
    }  
 }