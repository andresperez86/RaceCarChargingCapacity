kjhgajhgfdsaEWQ 4321|/*
Author: Andres Perez
Docente: Institución universitaria pascual Bravo
Date: 09/07/2024
Versión: v1.0
 */
#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#define ADC_VREF_mV 3300.0 //en milivoltios
#define ADC_RESOLUCION 4096.0 //Resolucion de los pines de entrada son 12 bits
#define SensorLED   2
#define SensorINPUT  32 //Connect the sensor to digital Pin 3 which is Interrupts 1.
#define SensorPin_C 4
#define SensorPin_V 12

// Prueba Multitarea(Hilos)RTOS en Diferente Nucleo ESP32 by: elprofegarcia.com
TaskHandle_t Tarea0;    // Tarea0 parpadeo LED 0,3 segundos
TaskHandle_t Tarea1;    // Tarea1 parpadeo LED 1 Segundo
TaskHandle_t Tarea2;    // Tarea2 Giro Motor Lento

//const int sensorPin_C = 4; // Analog pin connected to the current sensor output
//const int sensorPin_V = 12; // Analog pin connected to the voltage sensor output
const int sensorPin_T1=25;
const int sensorPin_T2=26;
const int sensorPin = 13; // Pin donde está conectado el sensor de velocidad de la rueda
float voltageRef = 3.3;   // Reference voltage used by ESP32 (3.3V) ACS09
float milliVolt;
float TempC;

// Pines para la conexión serial con el GPS
static const int RXPin = 15, TXPin = 2;
static const uint32_t GPSBaud = 9600;

//Variables globales
char letra = 's';
char final = 'f';
float voltaje = 52.8;
float corriente = 10.1423;
float potencia = 200.1434324;
float temperatura_a = 25.14345;
float temperatura_b = 37.1443432;
float distancia = 2.1434234;
float lat = 21.1423442;
float lng = 32.14423432;
float speed = 2.14543;
int vibracion = 1;
int state =1;

volatile unsigned long tiempoInicio = 0;
volatile unsigned long tiempoFin = 0;
volatile bool pulsoRecibido = false;
volatile unsigned int contadorPulsos = 0;
const float circunferenciaRueda = 2.0; // Circunferencia de la rueda en metros
//float distanciaRecorrida = 1.0;
unsigned long tiempoEntrePulsos = 0; 
float velocidad = 0.0;
float distanciaRecorrida = 0.0;

// Configuración de LoRa y GPS
LoRa_E32 e32ttl(&Serial2, 15, 21, 19); //  RX AUX M0 M1
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


// Crear un array de caracteres para almacenar el resultado
char resultado[60]; // Ajusta el tamaño según tus necesidades

// Declaración de funciones
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
void leerSensoresTemperatura(float &temperaturaSensor1, float &temperaturaSensor2);
float tempe_reading(int sensorPin);
float displayInfo();
void leerCorrienteVoltaje();
void detectarPulso(); // Declaración de la función de interrupción para el sensor de velocidad


//Interrupts function
void blink() {
    state++;
}

void setup() {
 Serial.begin(115200);
 xTaskCreatePinnedToCore(loop0,"Tarea_0",4000,NULL,10,&Tarea0,0); // Core 0
 xTaskCreatePinnedToCore(loop1,"Tarea_1",4000,NULL,10,&Tarea1,0); // Core 0
 xTaskCreatePinnedToCore(loop2,"Tarea_2",4000,NULL,1,&Tarea2,1); // Core 1
 while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB
    }
 delay(100);
 e32ttl.begin(); 
 pinMode(SensorLED, OUTPUT);
 pinMode(SensorINPUT, INPUT);
 pinMode(sensorPin_T1,INPUT);
 attachInterrupt(digitalPinToInterrupt(sensorPin), detectarPulso, RISING);
}

void loop() { //Tarea que se ejecuta en el Core 1
   // Mantener el loop principal vacío para que no interfiera con las tareas
    vTaskDelay(10); // Ceder el control al sistema operativo     
} 
void loop0(void *parameter){  // Tarea0 que se ejecuta en el Core 0
 for(;;){
   bool gps_ready = 0;
   while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
      //Serial.print(F("Location: "));
      //Serial.print(gps.location.lat(), 6);
      lat = gps.location.lat();
      //Serial.print(F(","));
      //Serial.print(gps.location.lng(), 6);
      lng = gps.location.lng();
      gps_ready =1;  
  if (millis() > 10000 && gps.charsProcessed() < 10)
  {  
     delay(200);
     digitalWrite(SensorLED,HIGH);
     delay(200);
     digitalWrite(SensorLED,LOW);

    //Serial.println(F("No GPS detected: check wiring."));
  }
  if (gps_ready){
     leerCorrienteVoltaje();
     leerSensoresTemperatura(temperatura_a, temperatura_b);
        // Simular el envío de datos a través de LoRa
      //sprintf(resultado, "%c \n%.1f \n%.1f \n%.1f \n%.1f \n%.2f \n%.3f \n%.3f \n%c", letra, voltaje, corriente, temperatura_a, temperatura_b, distanciaRecorrida, lat, lng, final);
      ResponseStatus rs = e32ttl.sendFixedMessage(0, 3, 0x04, resultado);
      Serial.println(rs.getResponseDescription());
      Serial.println(resultado);
  }
  vTaskDelay(10); // Retardo de 10ms
 } 
}
void loop1(void *parameter){  // Tarea1 que se ejecuta en el Core 0
 for(;;){
   /// Lectura de corriente con el ACS709  
    //int sensorValue = analogRead(sensorPin_C); // Read the analog value from the sensor
    //float voltaje = sensorValue * voltageRef / 4095.0; // Convert the analog value to voltage
    //float current = (voltaje - voltageRef / 2.0) / 0.185; // Calculate the current using ACS709 specifications
    leerSensoresTemperatura(temperatura_a,temperatura_b);
    vTaskDelay(10); // Retardo de 10ms
  
 } 
}
void loop2(void *parameter){  // Tarea2 que se ejecuta en el Core 1
 for(;;){
    //Serial.println("");
    //delay(1000);
    //Serial.println("WiFi conectado");
    if (pulsoRecibido) {
        //noInterrupts(); // Deshabilitar interrupciones
        unsigned long tiempoEntrePulsos = tiempoFin - tiempoInicio;
        float velocidad = (circunferenciaRueda / tiempoEntrePulsos) * 1000.0; // 1000 para convertir a segundos
        distanciaRecorrida = contadorPulsos * circunferenciaRueda;
        Serial.print("Tiempo entre pulsos: ");
        Serial.print(tiempoEntrePulsos);
        Serial.println(" ms");
        Serial.print("Velocidad de la rueda: ");
        Serial.print(velocidad);
        Serial.println(" m/s");
        Serial.print("Distancia recorrida: ");
        Serial.print(distanciaRecorrida);
        Serial.println(" m");
        pulsoRecibido = false;
        //interrupts();
        vTaskDelay(50); // Retardo de 10ms
        
    }
 } 
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
    //lati = gps.location.lat();
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    //longi = gps.location.lng();
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

void detectarPulso() {
  tiempoInicio = tiempoFin;
  tiempoFin = micros();
  contadorPulsos++;
  pulsoRecibido = true;
}

void leerCorrienteVoltaje() {
    // Lectura de corriente y voltaje con el ACS709
    int adc = analogRead(SensorPin_C);
    int adc1 = analogRead(SensorPin_V);
    corriente = map(adc, 2816, 3040, 0 , 6600) * 5 / 5000.0; // Escalado de corriente
    voltaje = map(adc1, 2816, 3040, 0 , 6600) * 5 / 5000.0;  // Escalado de voltaje

    Serial.print("Corriente: ");
    Serial.println(corriente, 4);
    Serial.print("Voltaje: ");
    Serial.println(voltaje, 4);
}
