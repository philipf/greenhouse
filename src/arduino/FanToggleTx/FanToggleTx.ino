// Author: Philip Fourie
// Date  : 2013/10/13

// RH sensor more info:
// - http://sensing.honeywell.com/product%20page?pr_id=53944   (003)
// - http://sensing.honeywell.com/product%20page?pr_id=53944   (001 - original documentation)

#include <VirtualWire.h>

const int HotTemp       = 32;
const int RecoveredTemp = 27;



// Controls the base on the BC517 darlington transistor
// Arduino digital pin 8
const int FanPin = 8;

// Measures the revolutions per minute off the 2 fans (yellow wire)
int isFanOn = -1;
/*
const int Fan1RpmInterrupt = 0;
const int Fan2RpmInterrupt = 1;
const int Fan1RpmPin = 2;
const int Fan2RpmPin = 3;
volatile int rpm1Counter = -1;
volatile int rpm2Counter = -1;
*/

// LM35 connected to analog0 to measure air temperature
const int AirTemperaturePin = A0;

// LM35 connected to analog 1 to measure ground temperature
const int GroundTemperaturePin = A1;

// DFRobot moisture sensor
const int MoistureSensorPin = A2;

// Reads the humidity from the Honeywell HIH-4000-003 analog sensor
const int HumditySensorPin = A3;


const int LedPin = 9;

const int RfTxPin = 10;

// Greenhouse states
const int StateInit     = 0;
const int StateNormal   = 1;
const int StateHot      = 2;
const int StateCooling  = 3;

int currentState = StateInit;

// Inidicates how many times VCC has not been read, -1 forces an initial read
int vccReadSkipped = 0;
long lastVcc = 0;

// global variables for RF data
const int noOfSlots = 9;
int values[noOfSlots];
const int dataBytes = noOfSlots * sizeof(int);

void setup() {
  currentState = StateInit;
  
  vw_set_tx_pin(RfTxPin);
  vw_setup(2000);   // Bits per second
  
  // Pin setup
  pinMode(FanPin, OUTPUT);
 
//  digitalWrite(Fan1RpmPin, HIGH);
//  digitalWrite(Fan2RpmPin, HIGH);
 
//  attachInterrupts();
  
  pinMode(LedPin, OUTPUT);
}

int getFan1Rpm() {
  return isFanOn;
}

int getFan2Rpm() {
  return isFanOn;
}

void loop() {
  /*
  sei();		//Enables interrupts
  delay (1000);	        //Wait 1 second
  cli();		//Disable interrupts
  */
     
  int vcc = getVcc(); // Be careful of this method it does something to the subsequent analog reads
  
  float airTemperature     = readLm35(AirTemperaturePin, vcc);
  float groundTemperature  = readLm35(GroundTemperaturePin, vcc);
  float humidity           = readHih4000(HumditySensorPin, vcc);
  float trueRh             = calcTrueRh(humidity, airTemperature);
  int groundMoisture       = readGroundMoisture(MoistureSensorPin);
  int fan1Rpm              = getFan1Rpm();
  int fan2Rpm              = getFan2Rpm();
  
  controlFans(airTemperature);
   
  digitalWrite(LedPin, HIGH);
  
  transmitValues(vcc,
                 airTemperature, 
                 groundTemperature,
                 humidity,
                 trueRh,
                 groundMoisture,
                 fan1Rpm,
                 fan2Rpm);
                 
  delay(200);
  digitalWrite(LedPin, LOW);                

  delay(2000);
}

void transmitValues(int vcc,
                    float airTemperature,
                    float groundTemperature,
                    float humidity,
                    float trueRh,
                    int groundMoisture,
                    int fan1Rpm,
                    int fan2Rpm) {
                      
                       
  int iAirTemp     = airTemperature * 100;
  int iGroundTemp  = groundTemperature * 100;
  int iHumidity    = humidity * 100;
  int iTrueRh      = trueRh * 100;
    
  values[0] = vcc;
  values[1] = iAirTemp;
  values[2] = iGroundTemp;
  values[3] = iHumidity;
  values[4] = iTrueRh;
  values[5] = groundMoisture;
  values[6] = fan1Rpm;
  values[7] = fan2Rpm;
  values[8] = currentState;
  
  vw_send((byte*) values, dataBytes);
}

void blinkTempValue(int tempValue) {
  byte blinks = 0;
  if (tempValue <= 10*100) {
    blinks = 1;
  } else if (tempValue <= 20*100) {
    blinks = 2;
  } else if (tempValue <= 30*100) {
    blinks = 3;
  } else if (tempValue <= 40*100) {
    blinks = 4;
  }
   
  for (byte i = 1; i <= blinks; i++) {
    digitalWrite(LedPin, HIGH);
    delay(500);
    digitalWrite(LedPin, LOW);
    delay(500);    
  }
}

void controlFans(float airTemperature) {
  // State normal
  if (currentState == StateNormal || currentState == StateInit) {
    if (airTemperature < HotTemp) {
        fanOff();
        return;
    } else {
      currentState = StateHot;
    }
  }
  
  // State hot
  if (currentState == StateHot) {
    fanOn();
    currentState = StateCooling;
  }
  
  // Cooling down
  if (currentState == StateCooling) {
    if (airTemperature < RecoveredTemp) {
      fanOff();
      currentState = StateNormal;
      return;
    } else {
      // still too hot, keep the fans on
      return;
    }
  }
}

void fanOn() {
  _enableFan(true);
  isFanOn = 1;
}

void fanOff() {
  _enableFan(false);
  isFanOn = 0;
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


void _enableFan(bool enabled) {
  int pinValue;
  
  if (enabled) {
    pinValue = HIGH;
  } else {
    pinValue = LOW;
  }
 
  digitalWrite(FanPin, pinValue);
}


float readHih4000(int pin, long vcc) {
  int analogValue = analogRead(pin);
 
  // http://arduino.cc/en/Reference/map
  float vOut = map(analogValue, 0, 1023, 0, vcc) / 1000.0;
  
    
  // http://sccatalog.honeywell.com/pdbdownload/images/hih-4000.series.chart.1.pdf
  // Vout = VSupply(0.0062*SensorRh + 0.16)
  
  // Solve for SensorRh
  // vOut / VSupply        = 0.0062 * SensorRh + 0.16
  // vOut / VSupply - 0.16 = 0.0062 * SensorRh 
  // SensorRh              = (Vout / VSupply - 0.16) / 0.0062
  
//  float sensorRh = ((vOut / (vSupply / 1000.0)) - 0.16) / 0.0062;

  float sensorRh = (vOut - 0.861) / 0.031;
  
  return sensorRh;
  
  // Temperature compensation
  // True RH = SensorRh / (1.0546 - 0.00216 * TempInCelsius)
}


float calcTrueRh(float sensorRh, float temp) {
  float trueRh = sensorRh / (1.0546 - 0.00216 * temp);
  return trueRh;
}


int readGroundMoisture(int pin) {
  return analogRead(pin);
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

/*
void attachInterrupts() {
  attachInterrupt(Fan1RpmInterrupt, rpm1, RISING); 
  attachInterrupt(Fan2RpmInterrupt, rpm2, FALLING); 
}


void rpm1() {
  rpm1Counter++;
}

void rpm2() {
  rpm2Counter++;
}
*/
