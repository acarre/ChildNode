// Program as Arduino Pro mini 16MHz (5V)



#include <SPI.h>
#include <RF24.h>
#include <SparkFunHTU21D.h>
#include <Wire.h>
#include <LowPower.h>
#include <printf.h>

#define nodeID 500 // this node
#define masterID 100 // master node controller

typedef struct {
  	int SID;
  	int dataID;
  	float value;
}

Message;
 
#define LED 9 //led pin on devduino
#define BUTTON 4 //button at side of devduino

Message nodeData;
Message command;

int sleepDur = 0;  // sleep 
 
//RF24 radio(CE,CSN);
RF24 radio(8,7); //radio CE to pin 8, CSN to pin 7
 
// WritePipe, ReadPipe
const uint64_t talking_pipe = 0xF0F0F0F0E1LL;
const uint64_t listening_pipe = 0xF0F0F0F0E2LL;
 
///////
 
HTU21D myHumidity; //set name for onboard sensor
 
void setup() {
  	Serial.begin(9600);
    printf_begin();
  	pinMode(LED, OUTPUT);
  	myHumidity.begin();
  	radio.begin();
  	//radio.setPALevel(RF24_PA_HIGH);   // set radio power
  	//radio.setDataRate(RF24_250KBPS);  // set radio baud rate
  	radio.setRetries(15,15);
  	//radio.setChannel(100);  // radio channel
  	radio.setPayloadSize(sizeof(nodeData));
  	radio.openWritingPipe(talking_pipe);
  	radio.openReadingPipe(1,listening_pipe);
    radio.printDetails();

  	radio.stopListening();
}
 
void loop() {
    digitalWrite(LED, HIGH);
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    digitalWrite(LED, LOW);
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);   

    //send and receive sequence

    radio.powerUp(); 
    delay(20);
    sendSensorMessage(1, myHumidity.readTemperature());
    sendSensorMessage(2, myHumidity.readHumidity());
    sendSensorMessage(3, 111); // unused
    sendSensorMessage(4, 222); // Unused
    sendSensorMessage(5, 333); // unused
    sendSensorMessage(6, sleepDur); // last sleep time
    sendSensorMessage(7, ((float) readVcc())/1000.0); // battery voltage
    radio.powerDown(); 
    delay(20);
}
 
// send data
void sendSensorMessage(int dID, float V) {

  	nodeData.SID = nodeID;
  	nodeData.dataID = dID;
  	nodeData.value = V;

  	Serial.println("Sending...");
  	Serial.print("Sensor ID: ");
  	Serial.println(nodeData.SID); 
  	Serial.print("Data ID: ");
  	Serial.println(nodeData.dataID);
  	Serial.print("Value: ");
  	Serial.println(nodeData.value);
  	Serial.print("Data Size: ");
  	Serial.println(sizeof(nodeData));

  	bool ok = radio.write( &nodeData, sizeof(nodeData) ); 

    if (ok) Serial.println("ok...");
    else Serial.println("failed.");

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout ) {
    	if (millis() - started_waiting_at > 200 ) timeout = true;
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
  		Serial.println(sizeof(nodeData));
  		if (command.SID == masterID) {
    		if (command.dataID == 0) Serial.print("They got it. Keep reporting.");
    		if (command.dataID == 1) { // ID =1 signals sleep command
    			Serial.print("They got it. Go to sleep and wake up in: ");
    			Serial.println(command.value);
    			sleepDur = command.value; // seconds to sleep.
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
  	int i  = 0;
  	while (i < t) {
    	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); // appears to run a little more slowly. 1800 cycles = 1870 seconds
    	i++;
  	}
  	radio.powerUp();
  	return;
}

