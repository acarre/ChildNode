//Setup devduino's as arduino Pro or Pro-mini
//Always use Serial 115200 (slower causes problems)
//Select custom board Sensor328p 1Mhz Atmega 328p 1.8V
//Select programmer voltage to 3.3V
//Select progammer USBASP

//Devduino connections:
// 3.3V         VCC              Power
// GND          GND              Ground
// A4           SDA              I2C Data
// A5           SCL              I2C Clock


#include <SPI.h>
#include <RF24.h>
#include <SHT2x.h>
//#include <SparkFunHTU21D.h>
#include <Wire.h>
#include <LowPower.h>
#include <APDS9930.h>
#include <avr/wdt.h> //watchdog
//#include <Adafruit_SleepyDog.h>
//#include <EEPROM.h> // EEPROM read/write libary. Simplifies variable types.

typedef struct {
    byte SID;
    byte dataID;
    float value;
}

Message;

// ********** MUST SET! ***********
#define nodeID 2 //node ID 
// ********************************

#define LED 9 // PIN 9 led pin on devduino
#define BUTTON 4 //button at side of devduino


Message sensor;
Message command;

int sleepDur = 0;  // sleep 
bool startSleep = true; // starting state is deep sleep until button pressed
unsigned long burst = 0; //send data at maxium rate
 
//RF24 radio(CE,CSN);
RF24 radio(8,7); //radio CE to pin 8, CSN to pin 7
 
// Radio pipes
const uint64_t writingPipe[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t readingPipe[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

 
///////
 
//HTU21D devTempHumSens; //set name for onboard sensor
APDS9930 apds = APDS9930(); //set name for APDS9930 sensor on I2C bus
uint16_t proximity_data = 0;
uint16_t ch0Light = 0;
uint16_t ch1Light = 0;
float luxLight = 0;
float temp = 0;
float hum = 0;
 
void setup() {
    //Serial.begin(115200); // Serial needs to use 115200. Anything else results in radio failures.
    pinMode(LED, OUTPUT); // Devduino led used to signal state.
    pinMode(BUTTON, INPUT_PULLUP);

    //Watchdog.enable(8000); // turn on the watchdog for 8s

    digitalWrite(LED, HIGH); // LED on means high power mode

    //devTempHumSens.begin(); // Start devuino built in Temp and Humidity sensor
    Wire.begin();

    apds.init();
    //apds.clearProximityInt();
    //apds.setMode(WAIT,1); //enable wait state between sensing cycles. Default is no wait.
    //apds.setProximityGain(PGAIN_2X); //set proximity sensor sensitivity
    //dumpAPDS(); //print out APDS registers

    radio.begin();
    //radio.setPALevel(RF24_PA_HIGH);   // set radio power
    radio.setPALevel(RF24_PA_MAX);   // set radio power
    radio.setDataRate(RF24_250KBPS);  // set radio baud rate
    radio.setChannel(100);  // radio channel
    radio.setRetries(15,15);
    //radio.setPayloadSize(sizeof(sensor));
    radio.setPayloadSize(8);//TEST

    radio.openWritingPipe(writingPipe[nodeID-1]);
    radio.openReadingPipe(1,readingPipe[nodeID-1]);
    radio.stopListening();
    watchdogSetup(); //watchdog
}
 
void loop() {
    
    if (burst < millis()) goToSleep (1); // sleep for 1 seconds if no command received. Master must still be asleep.
    //check button
    //if (digitalRead(BUTTON) == LOW) startSleep = false;
    //if (startSleep == true) flashNodeId(); // flick the node ID pattern
    //else {
      //flashNodeId();
      //send and receive sequence
      apds.enableProximitySensor(false);
      apds.setProximityGain(0); //set gain to 1x
      apds.setProximityDiode(1); //set diode to 25mA
      apds.readProximity(proximity_data);
      delay(10);
      apds.enableLightSensor(false);
      apds.readCh0Light(ch0Light);
      apds.readCh1Light(ch1Light);
      apds.readAmbientLightLux(luxLight);

      //temp = devTempHumSens.readTemperature();
      temp =SHT2x.GetTemperature();
      hum = SHT2x.GetHumidity();

      if (sendSensorMessage(1, temp)) burst = millis()+10000; // transmit data for 10 seconds
      
      //Node data
      /*
      0 Humidity (%RH)
      1 Proximity
      3 Ch0 light
      4 Ch1 light
      5 Lux light
      6 Battery volt
      */

      if (burst>millis()) {
        sendSensorMessage(2, hum);
        sendSensorMessage(3, float(proximity_data)); // proximity measure
        sendSensorMessage(4, float(ch0Light)); 
        sendSensorMessage(5, float(ch1Light)); 
        sendSensorMessage(6, luxLight);
        sendSensorMessage(7, ((float) readVcc())/1000.0); // battery voltage
      }
    //}
    //Watchdog.reset();
    wdt_reset();
}
 
// send data
bool sendSensorMessage(byte dID, float V) {

    sensor.SID = nodeID;
    sensor.dataID = dID;
    sensor.value = V;
    bool commandRec = false;

    //Serial.println("Sending...");
    //Serial.print("Sensor ID: ");
    //Serial.println(sensor.SID); 
    //Serial.print("Data ID: ");
    //Serial.println(sensor.dataID);
    //Serial.print("Value: ");
    //Serial.println(sensor.value);
    //Serial.print("Data Size: ");
    //Serial.println(sizeof(sensor));

    bool ok = radio.write( &sensor, sizeof(sensor) ); 

    //if (ok) Serial.println("ok..."); //removal causes routine failure
    //else Serial.println("failed.");  //removal causes routine to fail.
    //delay(20);
    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout ) {
      if (millis() - started_waiting_at > 500 ) timeout = true; //was 50
    }

    // Describe the results
    if ( timeout ) ;//Serial.println("Failed, response timed out.");
    else {
      // Grab the response, compare, and send to debugging spew
        radio.read( &command, sizeof(command) );
        // Spew it
        //Serial.println("Receiving...");
        //Serial.print("Sensor ID: ");
        //Serial.println(command.SID); 
        //Serial.print("Data ID: ");
      //Serial.println(command.dataID);
      //Serial.print("Value: ");
      //Serial.println(command.value);
      //Serial.print("Data Size: ");
      //Serial.println(sizeof(sensor));
      if (command.SID == nodeID) {
        if (command.dataID == 0) {
          //Serial.print("They got it. Keep reporting."); //removal causes routine to fail
          delay(20);
          commandRec = true;
        }
        if (command.dataID == 1) { // ID =1 signals sleep command
          
          //Serial.print("They got it. Go to sleep and wake up in: ");
          delay(20);
          //Serial.println(command.value);
          sleepDur = (int)command.value; // seconds to sleep.
          //apds.clearProximityInt(); //reset proximity interrupts because data has been received
          goToSleep (sleepDur);
        }
      }
    }
    // needs a retry in here...
    radio.stopListening();
    return commandRec;
}
 
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(75); // Wait for Vref to settle                                                                                                                                                                                                                                                                                               
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void goToSleep (int t) {
  if (t>3600) t=0; //sleep cannot be larger than 1 hour.
    digitalWrite(LED, LOW);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    radio.powerDown();
    apds.disablePower(); //APDS will only check proximity when awake.
    int i  = 0;
    while (i < t) {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      //Watchdog.sleep(1000);
      //Watchdog.reset();
      wdt_reset();
      i++;
    }
    radio.powerUp();
    apds.enablePower(); //apds may have powered down due to a prox interupt.
    digitalWrite(LED, HIGH);  
    
    return;
}

void dumpAPDS () {
/* Register dump */
  uint8_t reg;
  uint8_t val;

  for(reg = 0x00; reg <= 0x19; reg++) {
    if( (reg != 0x10) && \
        (reg != 0x11) )
    {
      apds.wireReadDataByte(reg, val);
      //Serial.print(reg, HEX);
      //Serial.print(": 0x");
      //Serial.println(val, HEX);
    }
  }
  apds.wireReadDataByte(0x1E, val);
  //Serial.print(0x1E, HEX);
  //Serial.print(": 0x");
  //Serial.println(val, HEX);
}

int proxIntStatus() {
  uint8_t val;
  int status = 0;
  apds.wireReadDataByte(APDS9930_STATUS, val);
  status = bitRead(val, 5);
  return status;
}

/*
void flashNodeId() {
  digitalWrite(LED, LOW);
  radio.powerDown();
  apds.disablePower();
  for (int i=0;i<nodeID;i++) {
    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
    if (digitalRead(BUTTON) == LOW) startSleep = false;
  }
}
*/

void watchdogSetup(void)
{
cli();
wdt_reset();
/*
WDTCSR configuration:
WDIE = 0: Interrupt disable
WDE = 1 :Reset Enable
See table for time-out variations:
WDP3 = 1 :For 8000ms Time-out
WDP2 = 0 :
WDP1 = 0 :
WDP0 = 0 :
*/
// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR = (0<<WDIE) | (1<<WDE) |
(1<<WDP3) | (0<<WDP2) | (0<<WDP1) |
(1<<WDP0);
sei();
}