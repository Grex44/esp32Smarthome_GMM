#include <PCF8574.h>

#include <Wire.h>
#include "PCF8574.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX
#include "RTClib.h"

#define I2C_SDA 4
#define I2C_SCL 15

TwoWire I2CVLX = TwoWire(0);

// Set i2c address
PCF8574 pcf8574(0x24, 4, 15);
PCF8574 pcf8574b(0x22, 4, 15);

//BME280sensor
Adafruit_BME280 bme;  // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

// Variables will change:
int ledState = HIGH;        // the current state of the output pin
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int relay1State = HIGH;     // Ventilation Fan;
int relay2State = HIGH;     // Aircon
int relay3State = HIGH;     // Surrouding lights
int relay4State = HIGH;     // UV light;the current state of the output pin ; normal  lights
int relay5State = HIGH;     // Occupancy indicator    (RED)
int relay6State = HIGH;     // Occupancy indicator    (GREEN)

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
//unsigned long debounceDelay = 1500;    // the debounce time; increase if the output flickers
unsigned long debounceDelay = 30;  // the debounce time; increase if the output flickers  Ori
int state = 0;


// Presence of human sensor:
// int human1 = 33;    //1st human sensor pin
int human1 = 23;  //1st human sensor pin
int human2 = 19;  //2nd human sensor pin
int human3 = 18;  //3rd human sensor pin
int humanState = LOW;
//might need to use more pins for multiple sensors. 1 or 2 more additional

// Door state (open or close)
uint8_t doorState = 0;  // 0 =close, 1 = open

int sessionState = 0;
unsigned long presenceDelay = 3000;  // the debounce time; increase if the output flickers  Ori
unsigned long timeStamp = 0;

//UV disinfection period timing
bool timerActive;
// unsigned long timerLength;
// float timerLength = 0.75;  //current is testing for 21s at 0.35
float timerLength = 2.5;  //10 mins
unsigned long startTime;
float duration = timerLength * 60 * 1000;
// unsigned long currMillis;
unsigned long timeStampCheck = 0;
unsigned long disinfectionPeriod = 20000;  //check for 1.5 minutes and remain indicator light to maintain as RED
uint32_t nextTime;
uint32_t lastTime;
uint32_t deltaTime;
bool checktimebool = true;

/*
// door trigger, occupancy state 

// State machine; 1 = normal lighting on; 2 = disinfection transition; 3 = after disinfection trigger. **maybe 4 = off 

0 = UV light off but normal light on; aircon off and surrounding lights off; 
2 = UV light trigger; aircon off and surrouding lights off;
3 = UV light on; aircon off and surrounding lights off; 
4 = all off

*/

int stateMachine = 0;
// int prevStateMachine = 0;

//RTC DS1307
RTC_DS1307 rtc;

/*
//ToF sensor parameters
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

SparkFun_VL53L5CX myImager1;
int sensorAddress1 = 0x29; //New address of unit without a wire. Valid: 0x08 <= address <= 0x77
int sensorReset1 = 19; //GPIO that is connected to the Reset pin on sensor 1
VL53L5CX_ResultsData measurementData1;

SparkFun_VL53L5CX myImager2;
int sensorAddress2 = 0x28; //Default VL53L5CX - this is the unit we'll hold in reset (has the wire soldered)
int sensorReset2 = 18; //GPIO that is connected to the Reset pin on sensor 2
VL53L5CX_ResultsData measurementData2;
*/

void setup() {
  Wire.begin(4, 15);   // Wire communication begin
  Serial.begin(9600);  //rasp pi is 9600.
  I2CVLX.begin(I2C_SDA, I2C_SCL);
  // bme.begin(0x76, &I2CVLX);

  pinMode(32, OUTPUT);

  Serial.println("PCF8574 initializing");

  //Initialize the pcf8574 multiplexer
  if (pcf8574.begin()) {
    digitalWrite(32, HIGH);
    Serial.println("OK");
  } else {
    digitalWrite(32, LOW);
    Serial.println("KO");
  }

  if (pcf8574b.begin()) {
    digitalWrite(32, HIGH);
    Serial.println("OK");
  } else {
    digitalWrite(32, LOW);
    Serial.println("KO");
  }

  Serial.println("BME280 initializing");
  //Initializing BME280
  // if (!bme.begin(0x76, &I2CVLX)) {
  if (!bme.begin(0x76)) {  //correct one; don't need the &i2cvlx but need to change the hardware pin config from nodeMCU-32S
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  //Initializing rtc
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  // Set pinMode to INPUT
  pcf8574b.pinMode(P0, INPUT);  // Door sensor;
  pcf8574b.pinMode(P1, INPUT);
  pcf8574b.pinMode(P2, INPUT);
  pcf8574b.pinMode(P3, INPUT);
  pcf8574b.pinMode(P4, INPUT);
  pcf8574b.pinMode(P5, INPUT);

  // Set pinMode to OUTPUT
  pcf8574.pinMode(P0, OUTPUT);  // Ventilation Fan;
  pcf8574.pinMode(P1, OUTPUT);  // fan or aircorn
  pcf8574.pinMode(P2, OUTPUT);  // normal surrounding lights
  pcf8574.pinMode(P3, OUTPUT);  // uv light
  pcf8574.pinMode(P4, OUTPUT);  // Occupancy indicator    (RED)
  pcf8574.pinMode(P5, OUTPUT);  // Occupancy indicator    (GREEN)

  // digitalWrite(32, HIGH);
  //Testing purpose on relays
  pcf8574.digitalWrite(P0, LOW);
  delay(200);
  pcf8574.digitalWrite(P1, LOW);
  delay(200);
  pcf8574.digitalWrite(P2, LOW);
  delay(200);
  pcf8574.digitalWrite(P3, LOW);
  delay(200);
  pcf8574.digitalWrite(P4, LOW);
  delay(200);
  pcf8574.digitalWrite(P5, LOW);
  delay(200);
  pcf8574.digitalWrite(P0, HIGH);
  delay(100);
  pcf8574.digitalWrite(P1, HIGH);
  delay(100);
  pcf8574.digitalWrite(P2, HIGH);
  delay(100);
  pcf8574.digitalWrite(P3, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, HIGH);
  delay(100);
  pcf8574.digitalWrite(P5, HIGH);
  delay(100);

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  if (!rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  //UV timer is false at start
  timerActive = false;  // Timer in inactive state

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  Serial.println("StartLoop");

  /*
  //setup of ToF sensors
  Serial.println("start2");
  pinMode(sensorReset2, OUTPUT);
  Serial.println("start2.1");
  digitalWrite(sensorReset2, HIGH); //Hold sensor 2 in reset while we configure sensor 1
  Serial.println("sensor reset 2");
  
  pinMode(sensorReset1, OUTPUT);
  digitalWrite(sensorReset1, HIGH); //Reset sensor 1
  delay(100);
  digitalWrite(sensorReset1, LOW); //Sensor 1 should now be available at default address 0x29

  Serial.println(F("Initializing sensor 1. This can take up to 10s. Please wait."));
  if (myImager1.begin() == false)
  {
    Serial.println(F("Sensor 1 not found. Check wiring. Freezing..."));
    while (1) ;
  }

  Serial.print(F("Setting sensor 1 address to: 0x"));
  Serial.println(sensorAddress1, HEX);

  if (myImager1.setAddress(sensorAddress1) == false)
  {
    Serial.println(F("Sensor 1 failed to set new address. Please try again. Freezing..."));
    while (1);
  }

  int newAddress = myImager1.getAddress();

  Serial.print(F("New address of sensor 1 is: 0x"));
  Serial.println(newAddress, HEX);

  digitalWrite(sensorReset2, LOW); //Release sensor 2 from reset

  Serial.println(F("Initializing sensor 2. This can take up to 10s. Please wait."));
  if (myImager2.begin() == false)
  {
    Serial.println(F("Sensor 2 not found. Check wiring. Freezing..."));
    while (1) ;
  }

  //Configure both sensors the same just to keep things clean
  myImager1.setResolution(8 * 8); //Enable all 64 pads
  myImager2.setResolution(8 * 8); //Enable all 64 pads

  imageResolution = myImager1.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager1.setRangingFrequency(15);
  myImager2.setRangingFrequency(15);

  myImager1.startRanging();
  myImager2.startRanging();
*/
}

void loop() {
  uint8_t val1 = pcf8574b.digitalRead(P0);
  bool humanSense1 = digitalRead(human1);  //microwave sensor pin to sense human
  bool humanSense2 = digitalRead(human2);  //microwave sensor pin to sense human
  bool humanSense3 = digitalRead(human3);  //microwave sensor pin to sense human

  sensors_event_t temp_event, pressure_event, humidity_event;

  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  //Print bme sensor reading
  // Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  // Serial.print("*C; ");
  Serial.print("; ");
  // Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  // Serial.print("%; ");
  Serial.print("; ");
  Serial.print(doorState);
  Serial.print(" ;");         //the door status
  Serial.print(relay1State);  //ventilation fan
  Serial.print(" ;");
  Serial.print(relay2State);  //fan / aircon
  Serial.print(" ;");
  Serial.print(relay3State);  //Surrouding lights
  Serial.print(" ;");
  Serial.print(relay4State);  //UV light
  Serial.print(" ;");
  Serial.print(relay5State);  //Occupancy LED
  Serial.print(" ;");
  Serial.print(relay6State);  //Occupancy LED   GREEN
  // Serial.print(" ;a");
  Serial.print(" ;");
  Serial.print(humanSense1);
  // Serial.print(" ;b");
  Serial.print(" ;");
  Serial.print(humanSense2);
  // Serial.print(" ;c");
  Serial.print(" ;");
  Serial.print(humanSense3);
  Serial.print(" ;");
  Serial.print(humanState);
  Serial.print(" ;");
  Serial.print(stateMachine);
  Serial.println(" ");
  // Serial.print(" ;");
  // Serial.println(timerActive);
  // Serial.print(" ;");
  // Serial.println("00000");
  // Serial.println((millis() - startTime));  //can remove it in the actual finalcode.

  // temp, humidity, doorsensor, ventilationFan, Aircon, surrouding lights, uv light, occupancy LED, humansense1, humansense2,humansense3, humanstate, statemachine, currMillis, timestampcheck;

  //   Serial.print(lastDebounceTime); Serial.print(" ;");
  //   Serial.print(millis()); Serial.print(" ;");

  //only door state
  // /*
  if (state == 0) {
    lastDebounceTime = millis();
    state = 1;
    buttonState = val1;
  } else {
    if ((millis() - lastDebounceTime) < debounceDelay) {
      if (val1 == HIGH) {
        buttonState = val1;
      }
    } else {
      if (buttonState == HIGH) {
        doorState = HIGH;  //door is open
        ledState = LOW;

        delay(500);  //delay for 1.5s  ** might need to know how to just delay for awhile then breakfrom delaying anymore
        //        Serial.println("Door is OPEN");
      } else {
        doorState = LOW;  //door is close
        ledState = HIGH;
      }
      state = 0;
    }
  }
  // */

  //StateMachine check coding

  if (stateMachine == 1) {
    relay1State = LOW;   //Ventilation Fan             ON
    relay2State = LOW;   //Air con                     ON
    relay3State = LOW;   //Surrouding lights           ON
    relay4State = HIGH;  //UV light to normal light    OFF
    relay5State = LOW;   //Occupancy                   ON  (RED)
    relay6State = HIGH;  //Occupancy                   OFF (GREEN)

    pcf8574.digitalWrite(P0, relay1State);
    pcf8574.digitalWrite(P1, relay2State);
    pcf8574.digitalWrite(P2, relay3State);
    pcf8574.digitalWrite(P3, relay4State);
    pcf8574.digitalWrite(P4, relay5State);
    pcf8574.digitalWrite(P5, relay6State);

  } else if (stateMachine == 2) {
    relay1State = HIGH;  //Ventilation Fan             OFF
    relay2State = HIGH;  //Air con                     OFF
    relay3State = HIGH;  //Surrouding lights           OFF
    // relay4State = LOW;  //UV light to normal light     ON
    relay5State = LOW;   //Occupancy                   ON  (RED)
    relay6State = HIGH;  //Occupancy                   OFF (GREEN)

    //only door state
    // /*
    if (state == 0) {
      lastDebounceTime = millis();
      state = 1;
      buttonState = val1;
    } else {
      if ((millis() - lastDebounceTime) < debounceDelay) {
        if (val1 == HIGH) {
          buttonState = val1;
        }
      } else {
        if (buttonState == HIGH) {
          doorState = HIGH;  //door is open
          ledState = LOW;

          delay(50);
          // delay(500);  //delay for 1.5s  ** might need to know how to just delay for awhile then breakfrom delaying anymore
          //        Serial.println("Door is OPEN");
        } else {
          doorState = LOW;  //door is close
          ledState = HIGH;
          delay(10);
        }
        state = 0;
      }
    }

    delay(10);

    //add the check of door status then only proceed with disinfection procedure
    // if (doorState != 1) {
    //   desinfection();
    //   delay(1);
    // } else {
    //   relay4State = HIGH;  //UV LIGHTS OFF
    // }

    desinfection();
    delay(10);

    //After disinfection sequence triggered, maintain UV light relay to OFF
    // relay4State = HIGH;  //UV light to normal light   OFF

    pcf8574.digitalWrite(P0, relay1State);
    pcf8574.digitalWrite(P1, relay2State);
    pcf8574.digitalWrite(P2, relay3State);
    pcf8574.digitalWrite(P3, relay4State);
    pcf8574.digitalWrite(P4, relay5State);
    pcf8574.digitalWrite(P5, relay6State);

    // If timer is not active then wait for the start button

    // Start button has been pressed
    // Timer now active
    timerActive = true;
    // Record the start time in milliseconds
    startTime = millis();
    // Calculate duration to activate timer for in milliseconds
    // Calculation will be using unsigned long aritmatic
    // as first argument is an unsigned long

    // Serial.println("Before timerActive loop start");

    // The following code will only be reached when the timer is active
    // Check every second if the selected time has elapsed or stop button pressed

    while (timerActive) {
      if ((millis() - startTime) >= duration) {
        // Duration reached so
        // set to not active, turn off active light and relay
        //maybe need to add counter for the UV light thingy to prevent it from switching on and off multiple times
        startTime = millis();
        stateMachine = 3;
        timerActive = false;
        relay4State = HIGH;  //UV light               OFF

        pcf8574.digitalWrite(P3, relay4State);  //UV light off
        pcf8574.digitalWrite(P4, relay5State);  //Indicator lights RED off
        pcf8574.digitalWrite(P5, relay6State);  //Indicator lights change to GREEN
        break;
      }

      if ((digitalRead(human1) == true) || (digitalRead(human3) == true)) { 
      // if (digitalRead(human1) == true) {  //need to remove this and the detect human function
                                                                             // if (digitalRead(human2) == true) {  //for testing purposes; using only 1 sensor
        timerActive = false;                                                 //**might be an error somewhere here need to look into this temeractive thingy
        break;
      }

      // sensors_event_t temp_event, pressure_event, humidity_event;

      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);

      //Print bme sensor reading
      // Serial.print(F("Temperature = "));
      Serial.print(temp_event.temperature);
      // Serial.print("*C; ");
      Serial.print("; ");

      // Serial.print(F("Humidity = "));
      Serial.print(humidity_event.relative_humidity);
      // Serial.print("%; ");
      Serial.print("; ");

      // Serial.print(val1);
      // Serial.print(" ;");
      Serial.print(doorState);
      Serial.print(" ;");         //the door status
      Serial.print(relay1State);  //ventilation fan
      Serial.print(" ;");
      Serial.print(relay2State);  //fan / aircon
      Serial.print(" ;");
      Serial.print(relay3State);  //Surrouding lights
      Serial.print(" ;");
      Serial.print(relay4State);  //UV light
      Serial.print(" ;");
      Serial.print(relay5State);  //Occupancy LED   RED
      Serial.print(" ;");
      Serial.print(relay6State);  //Occupancy LED   GREEN
      // Serial.print(" ;a");
      Serial.print(" ;");
      Serial.print(humanSense1);
      // Serial.print(" ;b");
      Serial.print(" ;");
      Serial.print(humanSense2);
      // Serial.print(" ;c");
      Serial.print(" ;");
      Serial.print(humanSense3);
      Serial.print(" ;");
      Serial.print(humanState);
      Serial.print(" ;");
      Serial.print(stateMachine);
      Serial.println(" ");
      // Serial.print(" ;");
      // Serial.println(timerActive);
      // Serial.print(" ;");
      // Serial.println((millis() - startTime));

      // if((humanSense1 == true) || (humanSense3 == true)){
      // if ((digitalRead(human1) == true) || (digitalRead(human2) == true)) {  //need to remove this and the detect human function
      // if ((digitalRead(human1) == true) || (digitalRead(human3) == true)) { 
      // // if (digitalRead(human1) == true) {  //need to remove this and the detect human function
      //                                                                        // if (digitalRead(human2) == true) {  //for testing purposes; using only 1 sensor
      //   timerActive = false;                                                 //**might be an error somewhere here need to look into this temeractive thingy
      //   break;
      // }
      if (timerActive == false){
        break;
      }
      delay(50);
    }
    // Serial.println("OUT of whileloop; ");

  } else if (stateMachine == 3) {
    relay1State = HIGH;  //Ventilation Fan            OFF
    relay2State = HIGH;  //Air con                    OFF
    relay3State = HIGH;  //Surrouding lights          OFF
    relay4State = HIGH;  //UV light to normal light    OFF
    relay5State = HIGH;  //Occupancy                  ON  (RED) ; current is green
    relay6State = LOW;   //Occupancy                   OFF (GREEN)

    /*
    //Print bme sensor reading
    // Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.print("*C; ");

    // Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.print("%; ");

    Serial.print(val1);
    Serial.print(" ;");
    Serial.print(doorState);
    Serial.print(" ;");         //the door status
    Serial.print(relay1State);  //ventilation fan
    Serial.print(" ;");
    Serial.print(relay2State);  //fan / aircon
    Serial.print(" ;");
    Serial.print(relay3State);  //Surrouding lights
    Serial.print(" ;");
    Serial.print(relay4State);  //UV light
    Serial.print(" ;");
    Serial.print(relay5State);  //Occupancy LED   RED
    Serial.print(" ;");
    Serial.print(relay6State);  //Occupancy LED   GREEN
    Serial.print(" ;");
    Serial.print(humanSense1);
    Serial.print(" ;");
    Serial.print(humanSense2);
    Serial.print(" ;");
    Serial.print(humanSense3);
    Serial.print(" ;");
    Serial.print(humanState);
    Serial.print(" ;");
    Serial.print(stateMachine);
    Serial.print(" ;");
    Serial.print(timerActive);
    Serial.print(" ;");
    Serial.println((millis() - startTime));
    */

    pcf8574.digitalWrite(P0, relay1State);
    pcf8574.digitalWrite(P1, relay2State);
    pcf8574.digitalWrite(P2, relay3State);
    pcf8574.digitalWrite(P3, relay4State);
    pcf8574.digitalWrite(P4, relay5State);
    pcf8574.digitalWrite(P5, relay6State);
  }

  //Test microwave with UV light
  checkHuman2(humanSense1, humanSense2, humanSense3);

  // set the LED, lights, fan or aircon , UV light:
  digitalWrite(32, ledState);
}

void checkHuman2(bool a, bool b, bool c) {
  // if (a == 1 || b == 1 || c == 1) {
  // if (a == true || b == true) {  //Current pod uses 2 sensors
  if (a == true || c == true) {  //Current pod uses 2 sensors
//  if (a == true) {  //Testing with one sensor; for testing purposes
 // if (b == true) {  //Testing with one sensor; for testing purposes
    stateMachine = 1;
    humanState = HIGH;
    relay1State = LOW;   //Ventilation Fan             ON
    relay2State = LOW;   //Air con                     ON
    relay3State = LOW;   //Surrouding lights           ON
    relay4State = HIGH;  //UV light to normal light    OFF
    relay5State = LOW;   //Occupancy                   ON  (RED)
    relay6State = HIGH;  //Occupancy                   OFF (GREEN)

  } else {
    if (stateMachine != 3) {
      // if (stateMachine != 3 || stateMachine != 4) {      //bad idea don't know why it will loop to statemachine 2
      stateMachine = 2;
    }
    humanState = LOW;
  }
}

void desinfection() {
  relay4State = HIGH;
  pcf8574.digitalWrite(P3, relay4State);  //Shut off UV light to off
  delay(1500);                            //pause 3s to full reset as ZERO ; 2000ms

  //disinfestion pattern sequence
  relay4State = LOW;                      //ON
  pcf8574.digitalWrite(P3, relay4State);  //trigger UV light sequence

  delay(250);
  relay4State = HIGH;  //OFF
  pcf8574.digitalWrite(P3, relay4State);

  delay(250);
  relay4State = LOW;  //ON
  pcf8574.digitalWrite(P3, relay4State);
  delay(250);
}

void sessionStarted() {
  if (doorState == HIGH) {
    timeStamp = millis();
    if ((millis() - timeStamp) < presenceDelay) {
      sessionState = 1;
      relay1State = HIGH;
    } else {
      sessionState = 0;
    }
  } else {
    //doorState = LOW
    sessionState = 0;
  }
}

/*
  if (myImager1.isDataReady() == true)
  {
    if (myImager1.getRangingData(&measurementData1)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          Serial.print("1:");
          Serial.print(measurementData1.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  if (myImager2.isDataReady() == true)
  {
    if (myImager2.getRangingData(&measurementData2)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          Serial.print("2:");
          Serial.print(measurementData2.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println();
      
    }
  }
  */

// add filter into the human presence ;maybe around 10 - 15 seconds filter

/*
void checkHuman() {
  int humanState = digitalRead(human1);
  if (humanState == HIGH) {
    Serial.println("Human is present in the pod");
  }
  else {
    Serial.println("No Humans detected");
  }
}
*/