#include <VirtualWire.h>

// LM35 connected to analog 1 to measure air temperature
const int AirTemperaturePin = A1;

// Inidicates how many times VCC has not been read, -1 forces an initial read
int vccReadSkipped = 0;
long lastVcc = 0;


static char dtostrfbuffer[5];
static char itostrbuffer[12];
  
// global variables for RF data
const int noOfSlots = 9;
int data[noOfSlots];
const int dataBytes = noOfSlots * sizeof(int);

byte msgLength = dataBytes;

void setup() {
  Serial.begin(9600);
  Serial.println("FanToggleRx v1.1");
  Serial.println("Listening...");
 
  vw_setup(2000);
  vw_rx_start();
}


void loop() {
  int vcc = getVcc(); // Be careful of this method it does something to the subsequent analog reads
  
  float airTemperature  = readLm35(AirTemperaturePin, vcc);
  
  msgLength = dataBytes;
  
  if (vw_get_message((byte*) data, &msgLength)) {
    Serial.print("OK:");
    if (msgLength == dataBytes) {
      for (int i = 0; i < noOfSlots; i++) {
        Serial.print(getLabel(i));
        Serial.print("=");
        Serial.print(normaliseValue(i, data[i]));
        Serial.print(",");
      }
                
      Serial.print("StudyTemp=");
      Serial.println(airTemperature);
    } else {
      Serial.print("ERROR:unexepected msg length of ");
      Serial.println(msgLength);
    }
    
  }
}


char* getLabel(int slotNo) {
  if (slotNo == 0) return "GH_Vcc";
  if (slotNo == 1) return "GH_AirTemp";
  if (slotNo == 2) return "GH_GroundTemp";    
  if (slotNo == 3) return "GH_Rh";      
  if (slotNo == 4) return "GH_TrueRh";        
  if (slotNo == 5) return "GH_GroundMoisture";
  if (slotNo == 6) return "GH_Fan1Rpm";  
  if (slotNo == 7) return "GH_Fan2Rpm";  
  if (slotNo == 8) return "GH_FanState";  
    
   return "Unknown pin";
}

char* normaliseValue(int slotNo, int value) {
  if (slotNo == 1
    || slotNo == 2
    || slotNo == 3
    || slotNo == 4) {
      float correctedValue = value / 100.0;
      char* result = dtostrf(correctedValue, 5, 2, dtostrfbuffer);
      return result;
  }

  itoa(value, itostrbuffer, 10);
  return itostrbuffer;
}

float readLm35(int pin, long vcc) {
//  delay(50);
  analogRead(pin); //dummy read to compensate for getVcc that throws of the subsequent analog reads
//  delay(50);
    
  int sensorValue = analogRead(pin);
  
  float temperatureInDegrees = sensorValue / 1023.0 * vcc / 10;

  return temperatureInDegrees;  
}


long getVcc() {
  vccReadSkipped++;
  
  if (lastVcc == 0 || vccReadSkipped > 50) {
    vccReadSkipped = 0;
    long vcc = _readVcc();
    lastVcc = vcc;
    return vcc;
  } else {
    return lastVcc;
  }
}


long _readVcc() {
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
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
  
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  
  return result; // Vcc in millivolts
}
