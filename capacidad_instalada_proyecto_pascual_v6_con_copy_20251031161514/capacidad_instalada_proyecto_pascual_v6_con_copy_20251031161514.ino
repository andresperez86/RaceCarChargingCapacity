/*
Author: Andres Perez
Docente: Institución Universitaria Pascual Bravo
Date: 09/07/2024
Versión: v2.1
Descripción: Sistema de adquisición de datos con LoRa, GPS y SD Card.
             Pines: 
             - SD: CS=33, SCK=18, MISO=19, MOSI=23
             - GPS: RX=15, TX=2
*/

#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <SPI.h>

// ---------------- CONFIGURACIÓN DE PINES ----------------
#define ADC_VREF_mV 3300.0
#define ADC_RESOLUCION 4096.0
#define SensorINPUT 32
#define SensorPin_C 4
#define SensorPin_V 12
#define sensorPin_T1 25
#define sensorPin_T2 26
#define pinSensorVelocidad 22
#define circunferenciaRueda 2.0

// Pines del módulo SD
#define SD_CS   33
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// Pines del módulo GPS
static const int RXPin = 15;  // RX del ESP32 (recibe del GPS)
static const int TXPin = 2;   // TX del ESP32 (hacia GPS si aplica)
static const uint32_t GPSBaud = 9600;

// ---------------- VARIABLES GLOBALES ----------------
char letra = 's';
char final = 'f';
float voltaje = 0.0, corriente = 0.0;
float temperatura_a = 0.0, temperatura_b = 0.0;
float lat = 0.0, lng = 0.0, speed = 0.0;
volatile unsigned long tiempoInicio = 0, tiempoFin = 0;
volatile bool pulsoRecibido = false;
volatile unsigned int contadorPulsos = 0;
float velocidad = 0.0, distanciaRecorrida = 0.0;

// ---------------- OBJETOS ----------------
LoRa_E32 e32ttl(&Serial2, 15, 21, 19);  // RX AUX M0 M1
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
File archivo;

// ---------------- FUNCIONES ----------------
void detectarPulso();
void leerCorrienteVoltaje();
void leerSensoresTemperatura(float &t1, float &t2);
float tempe_reading(int pin);
void procesarPulsoRueda();
void guardarEnSD(String data);

// ---------------- RTOS ----------------
TaskHandle_t Tarea0;
TaskHandle_t Tarea2;

// ==========================================================
//                      SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SensorINPUT, INPUT);
  pinMode(sensorPin_T1, INPUT);
  pinMode(sensorPin_T2, INPUT);
  pinMode(pinSensorVelocidad, INPUT);

  // Inicializar LoRa
  e32ttl.begin();

  // Inicializar GPS
  ss.begin(GPSBaud);
  Serial.println("Inicializando GPS...");

  // Inicializar bus SPI y tarjeta SD
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  Serial.print("Inicializando tarjeta SD... ");
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("❌ Error al iniciar la tarjeta SD");
  } else {
    Serial.println("✅ Tarjeta SD lista");
    archivo = SD.open("/registro.txt", FILE_APPEND);
    if (archivo) {
      archivo.println("==== Nuevo registro iniciado ====");
      archivo.close();
    }
  }

  // Interrupción sensor de velocidad
  attachInterrupt(digitalPinToInterrupt(pinSensorVelocidad), detectarPulso, RISING);

  // Crear tareas
  xTaskCreatePinnedToCore(loop0, "Tarea_0", 6000, NULL, 10, &Tarea0, 0);
  xTaskCreatePinnedToCore(loop2, "Tarea_2", 4000, NULL, 1, &Tarea2, 1);
}

void loop() {
  vTaskDelay(10);
}

// ==========================================================
//                  TAREAS (RTOS)
// ==========================================================
void loop0(void *parameter) {
  for (;;) {
    leerCorrienteVoltaje();
    leerSensoresTemperatura(temperatura_a, temperatura_b);

    // Lectura del GPS
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        speed = gps.speed.kmph();
      }
    }

    // Construir cadena de datos
    char resultado[160];
    sprintf(resultado,
            "%c,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%c",
            letra, voltaje, corriente, temperatura_a, temperatura_b,
            distanciaRecorrida, lat, lng, speed, final);

    // Enviar por LoRa
    ResponseStatus rs = e32ttl.sendFixedMessage(0, 3, 0x04, resultado);
    Serial.println(rs.getResponseDescription());
    Serial.println(resultado);

    // Guardar en SD
    guardarEnSD(String(resultado));

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop2(void *parameter) {
  for (;;) {
    if (pulsoRecibido) {
      procesarPulsoRueda();
      pulsoRecibido = false;
    }
    vTaskDelay(10);
  }
}

// ==========================================================
//                  FUNCIONES DE SENSORES
// ==========================================================
void leerCorrienteVoltaje() {
  int adcC = analogRead(SensorPin_C);
  int adcV = analogRead(SensorPin_V);
  corriente = map(adcC, 2816, 3040, 0, 6600) * 5 / 5000.0;
  voltaje = map(adcV, 2816, 3040, 0, 6600) * 5 / 5000.0;
  Serial.printf("Corriente: %.3f A | Voltaje: %.3f V\n", corriente, voltaje);
}

void leerSensoresTemperatura(float &t1, float &t2) {
  t1 = tempe_reading(sensorPin_T1);
  t2 = tempe_reading(sensorPin_T2);
}

float tempe_reading(int pin) {
  uint16_t val = analogRead(pin);
  float milliVolt = val * (ADC_VREF_mV / ADC_RESOLUCION);
  return milliVolt / 10.0;
}

// ==========================================================
//                  FUNCIONES DE VELOCIDAD
// ==========================================================
void detectarPulso() {
  tiempoInicio = tiempoFin;
  tiempoFin = micros();
  contadorPulsos++;
  pulsoRecibido = true;
}

void procesarPulsoRueda() {
  noInterrupts();
  unsigned long tiempoEntrePulsos = tiempoFin - tiempoInicio;
  velocidad = (circunferenciaRueda / tiempoEntrePulsos) * 1000.0;
  distanciaRecorrida = contadorPulsos * circunferenciaRueda;
  interrupts();
  Serial.printf("Velocidad: %.3f m/s | Distancia: %.3f m\n", velocidad, distanciaRecorrida);
}

// ==========================================================
//                  FUNCIONES DE ALMACENAMIENTO
// ==========================================================
void guardarEnSD(String data) {
  archivo = SD.open("/registro.txt", FILE_APPEND);
  if (archivo) {
    archivo.println(data);
    archivo.close();
    Serial.println("✅ Datos guardados en SD");
  } else {
    Serial.println("⚠️ Error al escribir en SD");
  }
}



