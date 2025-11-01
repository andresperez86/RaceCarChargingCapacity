/*
Author: Andres Perez
Docente: Institución Universitaria Pascual Bravo
Date: 01/11/2025
Versión: v1.3
Descripción: Sistema IoT con LoRa E32 (modo normal de transmisión),
GPS, sensores de corriente y temperatura. M0=21, M1=14.
*/

#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// ---------------- CONFIGURACIÓN DE PINES ----------------
#define ADC_VREF_mV 3300.0       // en milivoltios
#define ADC_RESOLUCION 4096.0    // Resolución de 12 bits
#define SensorINPUT   32         // Sensor de vibración
#define SensorPin_C   4          // Sensor de corriente
#define SensorPin_V   12         // Sensor de voltaje
#define sensorPin_T1  25         // Sensor de temperatura 1
#define sensorPin_T2  26         // Sensor de temperatura 2

// Pines LoRa E32
#define LORA_M0 21
#define LORA_M1 14

// Pines GPS
static const int RXPin = 15;   // RX del ESP32 (recibe del GPS)
static const int TXPin = 2;    // TX del ESP32 (envía al GPS)
static const uint32_t GPSBaud = 9600;

// ---------------- VARIABLES GLOBALES ----------------
char letra = 's';
char final = 'f';
float voltaje = 0.0;
float corriente = 0.0;
float temperatura_a = 0.0;
float temperatura_b = 0.0;
float lat = 0.0;
float lng = 0.0;
float speed = 0.0;

// ---------------- OBJETOS ----------------
LoRa_E32 e32ttl(&Serial2, 15, LORA_M0, LORA_M1);  // RX AUX M0 M1
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// ---------------- FUNCIONES ----------------
void leerSensoresTemperatura(float &t1, float &t2);
float tempe_reading(int pin);
void leerCorrienteVoltaje();
void configurarLoRaNormal();

// ---------------- RTOS ----------------
TaskHandle_t Tarea0;
TaskHandle_t Tarea2;

// ==========================================================
//                      SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Inicializando sistema IoT con LoRa E32 ===");

  // Configuración de pines de sensores
  pinMode(SensorINPUT, INPUT);
  pinMode(sensorPin_T1, INPUT);
  pinMode(sensorPin_T2, INPUT);

  // Pines LoRa como salida
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);

  // Configurar modo NORMAL (M0=LOW, M1=LOW)
  configurarLoRaNormal();

  // Inicializar LoRa
  e32ttl.begin();
  Serial.println("📡 LoRa E32 configurado en modo NORMAL (M0=21=0, M1=14=0).");

  // Inicializar GPS
  ss.begin(GPSBaud);
  Serial.println("📍 GPS inicializado en pines RX=15, TX=2");

  // Crear tareas (RTOS)
  xTaskCreatePinnedToCore(loop0, "Tarea_0", 4000, NULL, 10, &Tarea0, 0); // Core 0
  xTaskCreatePinnedToCore(loop2, "Tarea_2", 4000, NULL, 1, &Tarea2, 1);  // Core 1

  while (!Serial) { ; } // Esperar conexión USB si aplica
}

// ==========================================================
//                        LOOP PRINCIPAL
// ==========================================================
void loop() {
  vTaskDelay(10);
}

// ==========================================================
//              CONFIGURAR LORA EN MODO NORMAL
// ==========================================================
void configurarLoRaNormal() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(100);
}

// ==========================================================
//                      TAREAS RTOS
// ==========================================================
void loop0(void *parameter) {
  for (;;) {
    leerCorrienteVoltaje();
    leerSensoresTemperatura(temperatura_a, temperatura_b);

    // Leer datos del GPS
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        speed = gps.speed.kmph();
      }
    }

    // Construir mensaje a enviar
    char resultado[120];
    sprintf(resultado, "%c,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%c",
            letra, voltaje, corriente, temperatura_a, temperatura_b,
            lat, lng, speed, final);

    // Enviar por LoRa
    ResponseStatus rs = e32ttl.sendFixedMessage(0, 3, 0x04, resultado);
    Serial.println(rs.getResponseDescription());
    Serial.println(resultado);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop2(void *parameter) {
  for (;;) {
    // Espacio reservado para vibración o futuras funciones
    vTaskDelay(100);
  }
}

// ==========================================================
//                    FUNCIONES DE SENSORES
// ==========================================================
void leerCorrienteVoltaje() {
  int adcC = analogRead(SensorPin_C);
  int adcV = analogRead(SensorPin_V);

  corriente = map(adcC, 2816, 3040, 0 , 6600) * 5 / 5000.0;
  voltaje   = map(adcV, 2816, 3040, 0 , 6600) * 5 / 5000.0;

  Serial.printf("Corriente: %.3f A | Voltaje: %.3f V\n", corriente, voltaje);
}

void leerSensoresTemperatura(float &t1, float &t2) {
  t1 = tempe_reading(sensorPin_T1);
  t2 = tempe_reading(sensorPin_T2);
  Serial.printf("Temp1: %.2f °C | Temp2: %.2f °C\n", t1, t2);
}

float tempe_reading(int pin) {
  uint16_t val = analogRead(pin); 
  float milliVolt = val * (ADC_VREF_mV / ADC_RESOLUCION);
  return milliVolt / 10.0;  // Sensor tipo LM35: 10mV/°C
}




