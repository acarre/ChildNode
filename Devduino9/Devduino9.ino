//Setup devduino's as arduino Pro or Pro-mini
//Always use Serial 115200 (slower causes problems)
//Select 5V, 16MHz, otherwise timing is incorrect
//Select programmer voltage to 3.3V
//Select progammer USBASP

//Devduino connections:
// 3.3V         VCC              Power
// GND          GND              Ground
// A4           SDA              I2C Data
// A5           SCL              I2C Clock


#include <SPI.h>
#include <RF24.h>
#include <SparkFunHTU21D.h>
#include <Wire.h>
#include <LowPower.h>
#include <APDS9930.h>

#define nodeID 1 // this node

typedef struct {
  	int SID;
  	int dataID;
  	float value;
}

Message;
 
#define LED 9 //led pin on devduino
#define BUTTON 4 //button at side of devduino

Message sensor;
Message command;

int sleepDur = 0;  // sleep 
bool startSleep = true; // starting state is deep sleep until button pressed
 
//RF24 radio(CE,CSN);
RF24 radio(8,7); //radio CE to pin 8, CSN to pin 7
 
// WritePipe, ReadPipe
const uint64_t pipes[2] = {0xF0F0F0F0E2LL, 0xF0F0F0F0E1LL};
 
///////
 
HTU21D devTempHumSens; //set name for onboard sensor
APDS9930 apds = APDS9930(); //set name for APDS9930 sensor on I2C bus
uint16_t proximity_data = 0;
float ambient_light = 0;
 
void setup() {
  	Serial.begin(115200); // Serial needs to use 115200. Anything else results in radio failures.
  	pinMode(LED, OUTPUT); // Devduino led used to signal state.
    pinMode(BUTTON, INPUT_PULLUP);

    digitalWrite(LED, HIGH); // LED on means high power mode
  	devTempHumSens.begin(); // Start devuino built in Temp and Humidity sensor

    apds.init();
    //apds.clearProximityInt();
    apds.setMode(WAIT,1); //enable wait state between sensing cycles. Default is no wait.
    //apds.setMode(SLEEP_AFTER_INT,1); //enable sleep after interupt.
    //apds.wireWriteDataByte(APDS9930_WTIME, 0xFF); //set wait time to 2.73ms. Default 0xFF.
    //apds.wireWriteDataByte(APDS9930_CONFIG, 0); //set long wait to off. 2= Multiplies wait time by 12x. Default 0 no multiply.
    apds.setProximityGain(PGAIN_2X); //set proximity sensor sensitivity
    //apds.setProximityIntLowThreshold(0); //set low threshold (far)
    //apds.setProximityIntHighThreshold(600); //set high threshold (near)
    //apds.enableProximitySensor(false); //activate proximity sensing interupt
    //apds.enableProximitySensor(false);
    dumpAPDS(); //print out APDS registers

  	radio.begin();
  	//radio.setPALevel(RF24_PA_HIGH);   // set radio power
  	//radio.setDataRate(RF24_250KBPS);  // set radio baud rate
  	radio.setRetries(15,15);
  	//radio.setChannel(100);  // radio channel
  	radio.setPayloadSize(sizeof(sensor));
  	radio.openWritingPipe(pipes[0]);
  	radio.openReadingPipe(1,pipes[1]);
  	radio.stopListening();


}
 
void loop() {
    goToSleep (1);
    //check button
    if (digitalRead(BUTTON) == LOW) startSleep = false;
    if (startSleep == true) delay(2); //flick the led
    else {
      //send and receive sequence
      apds.enableProximitySensor(false);
      apds.readProximity(proximity_data);
      apds.enableLightSensor(false);
      apds.readAmbientLightLux(ambient_light);

      radio.powerUp(); 
      delay(20);
      sendSensorMessage(1, devTempHumSens.readTemperature());
      delay(20);
      sendSensorMessage(2, devTempHumSens.readHumidity());
      delay(20);
      sendSensorMessage(3, float(proximity_data)); // proximity measure
      delay(20);
      sendSensorMessage(4, ambient_light); // Unused
      delay(20);
      sendSensorMessage(5, 333); // unused
      delay(20);
      sendSensorMessage(6, sleepDur); // last sleep time
      delay(20);
      sendSensorMessage(7, ((float) readVcc())/1000.0); // battery voltage
      delay(20);
      radio.powerDown(); 
      delay(20);
    }
}
 
// send data
void sendSensorMessage(int dID, float V) {

  	sensor.SID = nodeID;
  	sensor.dataID = dID;
  	sensor.value = V;

  	Serial.println("Sending...");
  	Serial.print("Sensor ID: ");
  	Serial.println(sensor.SID); 
  	Serial.print("Data ID: ");
  	Serial.println(sensor.dataID);
  	Serial.print("Value: ");
  	Serial.println(sensor.value);
  	Serial.print("Data Size: ");
  	Serial.println(sizeof(sensor));

  	bool ok = radio.write( &sensor, sizeof(sensor) ); 

    if (ok) Serial.println("ok...");
    else Serial.println("failed.");

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout ) {
    	if (millis() - started_waiting_at > 1000 ) timeout = true;
    }

    // Describe the results
    if ( timeout ) Serial.println("Failed, response timed out.");
    else {
    	// Grab the response, compare, and send to debugging spew
      	radio.read( &command, sizeof(command) );
      	// Spew it
        Serial.println("Receiving...");
        Serial.print("Sensor ID: ");
        Serial.println(command.SID); 
        Serial.print("Data ID: ");
  		Serial.println(command.dataID);
  		Serial.print("Value: ");
  		Serial.println(command.value);
  		Serial.print("Data Size: ");
  		Serial.println(sizeof(sensor));
  		if (command.SID == nodeID) {
    		if (command.dataID == 0) Serial.print("They got it. Keep reporting.");
    		if (command.dataID == 1) { // ID =1 signals sleep command
    			Serial.print("They got it. Go to sleep and wake up in: ");
    			Serial.println(command.value);
    			sleepDur = command.value; // seconds to sleep.
          apds.clearProximityInt(); //reset proximity interrupts because data has been received
    			goToSleep (sleepDur);
    		}
    	}
    }
  	// needs a retry in here...
  	radio.stopListening();
  	return;
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
  	digitalWrite(LED, LOW);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
  	radio.powerDown();
    apds.disablePower(); //APDS will only check proximity when awake.
  	int i  = 0;
  	while (i < t) {
    	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
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
      Serial.print(reg, HEX);
      Serial.print(": 0x");
      Serial.println(val, HEX);
    }
  }
  apds.wireReadDataByte(0x1E, val);
  Serial.print(0x1E, HEX);
  Serial.print(": 0x");
  Serial.println(val, HEX);
}

int proxIntStatus() {
  uint8_t val;
  int status = 0;
  apds.wireReadDataByte(APDS9930_STATUS, val);
  status = bitRead(val, 5);
  return status;
}
