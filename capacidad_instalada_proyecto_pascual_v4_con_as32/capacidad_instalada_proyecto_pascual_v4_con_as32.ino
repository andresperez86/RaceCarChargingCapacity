/*
 * LoRa E32-TTL-100
 * Send fixed broadcast transmission message to a specified channel.
 * https://www.mischianti.org/2019/11/10/lora-e32-device-for-arduino-esp32-or-esp8266-fixed-transmission-part-4/
 *
 * E32-TTL-100----- Arduino UNO or esp8266
 * M0         ----- 3.3v (To config) GND (To send) 21 (To dinamically manage)
 * M1         ----- 3.3v (To config) GND (To send) 19 (To dinamically manage)
 * TX         ----- RX PIN 2 (PullUP)
 * RX         ----- TX PIN 3 (PullUP & Voltage divider)
 * AUX        ----- Not connected (5 if you connect)
 * VCC        ----- 3.3v/5v
 * GND        ----- GND
 *
 */
#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#define ADC_VREF_mV 3300.0 //en milivoltios
#define ADC_RESOLUCION 4096.0 //Resolucion de los pines de entrada son 12 bits
const int sensorPin_C = 4; // Analog pin connected to the current sensor output
const int sensorPin_V = 12; // Analog pin connected to the voltage sensor output
const int sensorPin_T1=25;
const int sensorPin_T2=26;
float milliVolt;
float TempC;

#define SensorLED   2
//Connect the sensor to digital Pin 3 which is Interrupts 1.
#define SensorINPUT  32



static const int RXPin = 15, TXPin = 4;
static const uint32_t GPSBaud = 9600;
char letra = 's';
char final = 'f';
float voltaje = 1.01;
float corriente = 1.1423;
float potencia = 1.1434324;
float temperatura_a = 1.14345;
float temperatura_b = 2.1443432;
float distancia = 2.1434234;
float lat = 21.1423442;
float lng = 32.14423432;
float speed = 2.14543;
int vibracion = 1;
int state =1;

//SoftwareSerial mySerial(D2, D3); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
//LoRa_E32 e32ttl(&mySerial, D5, D7, D6);
LoRa_E32 e32ttl(&Serial2, 15, 21, 19); //  RX AUX M0 M1

// The TinyGPSPlus object
TinyGPSPlus gps;


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Crear un array de caracteres para almacenar el resultado
char resultado[50]; // Ajusta el tamaño según tus necesidades

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
//The setup function is called once at startup of the sketch

//Interrupts function
void blink() {
    state++;
}

void setup()
{
	Serial.begin(115200);  
  Serial.println(F("Proyecto de Capacidad instalada vehiculo eléctrico.ino"));
  //Serial.println(F("A simple demonstration with an attached GPS module"));
  //Serial.print(F("Testing TinyGPSPlus library v. "));
  //Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Andrés Pérez"));
  Serial.println();
  ss.begin(GPSBaud);
	while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB
    }
	delay(100);

	e32ttl.begin(); 
  pinMode(SensorLED, OUTPUT);
  pinMode(SensorINPUT, INPUT);
  pinMode(sensorPin_T1,INPUT);
  //Trigger the blink function when the falling edge is detected
  attachInterrupt(SensorINPUT, blink, FALLING);

	// After set configuration comment set M0 and M1 to low
	// and reboot if you directly set HIGH M0 and M1 to program
//	ResponseStructContainer c;
//	c = e32ttl.getConfiguration();
//	Configuration configuration = *(Configuration*) c.data;
//	configuration.ADDL = 0x01;
//	configuration.ADDH = 0x00;
//	configuration.CHAN = 0x02;
//	configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
//	e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
//	printParameters(configuration);
//	c.close();
	// ---------------------------
}

// The loop function is called in an endless loop
void loop()
{
	delay(2000);
  // if ( state != 0){
  //   state = 0;
  //   VibrationFlag = 1;
  //   digitalWrite(SensorLED,HIGH);
  //   delay(50);
  // }
  // else {
  //   digitalWrite(SensorLED,LOW);
  //   delay(50);    
  //   VibrationFlag = 0;
 // }
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
      Serial.print(F("Location: "));
      //Serial.print(lati, 6);
      Serial.print(F(","));
      //Serial.print(longi, 6);
  // if (true)//(gps.location.isValid())
  // {
      Serial.print(gps.location.lat(), 6);
      lat = gps.location.lat();
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      lng = gps.location.lng();  
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }
  // }
     
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    //while(true);
  }

     
    //}


  //}
  //uint16_t val_t1 = analogRead(sensorPin_T1); 
  leerSensoresTemperatura(temperatura_a,temperatura_b);
  //temperatura_a = tempe_reading(sensorPin_T1); //(float) val_t1*(3.3/10.24);
  //temperatura_a = (float)(3300/4096.0*sensorValue_T1)/10.24;  
  //Serial2.println(resultado);
  Serial.println("Send message to 00 03 04");
  sprintf(resultado, "%c \n%.2f \n%.2f \n%.2f \n%.2f \n%.2f \n%.3f \n%.3f \n%c", letra, voltaje, corriente, temperatura_a,temperatura_b,distancia,lat,lng, final);
  ResponseStatus rs = e32ttl.sendFixedMessage(0, 3, 0x04, resultado);
  Serial.println(rs.getResponseDescription());
  Serial.println(resultado);
}

void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, BIN);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, BIN);
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

}
void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD BIN: "));  Serial.print(moduleInformation.HEAD, BIN);Serial.print(" ");Serial.print(moduleInformation.HEAD, DEC);Serial.print(" ");Serial.println(moduleInformation.HEAD, HEX);

	Serial.print(F("Freq.: "));  Serial.println(moduleInformation.frequency, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");

}

float displayInfo()
{
  float lati,longi;
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    lati = gps.location.lat();
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    longi = gps.location.lng();
  }
  else
  {
    Serial.print(F("INVALID"));
  }




  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }




  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }


  Serial.println();
 
  Serial.print(F("  Velocidad: "));
  if (gps.speed.isValid())
  {
    Serial.print(gps.speed.kmph());
  }
  else
  {
  Serial.print(F("INVALID"));
  }


  Serial.print(F("  Altitud: "));
  if (gps.altitude.isValid())
  {
    Serial.println(gps.altitude.meters());
  }
  else
  {
  Serial.println(F("INVALID"));
  }
  return lati,longi;
}

float voltage_reading(int sensorPin){
  uint16_t val_t1 = analogRead(sensorPin); 
  float milliVolt = val_t1*(ADC_VREF_mV/ADC_RESOLUCION);
  float temperatura_Celsius = milliVolt /10;
  return temperatura_Celsius;

}

float tempe_reading(int sensorPin){
  uint16_t val_t1 = analogRead(sensorPin); 
  float milliVolt = val_t1*(ADC_VREF_mV/ADC_RESOLUCION);
  float temperatura_Celsius = milliVolt /10;
  return temperatura_Celsius;

}

void leerSensoresTemperatura(float &temperaturaSensor1, float &temperaturaSensor2) {
  // Leer sensor 1
  temperaturaSensor1 = tempe_reading(sensorPin_T1);

  // Leer sensor 2
  temperaturaSensor2 = tempe_reading(sensorPin_T2);
}


