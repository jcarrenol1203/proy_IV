#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

/*
=============================================================================
        PRUEBA: SENSOR DE COLOR TCS3200 + SENSOR DE DISTANCIA VL53L0X
                          (ESP32 DevKitC V4)
=============================================================================
Adaptación de sensorcolorMain.c (STM32 HAL) a Arduino/ESP32, más lectura de
distancia con el sensor ToF GY-VL53L0XV2 por I2C. Reporta por Serial ambas
lecturas. No incluye motores, servo, electroimán ni sensor IR del proyecto XY
completo (ver main_xy_completo.cpp.bak en esta misma carpeta para
restaurarlo).

Conexiones TCS3200 -> ESP32 DevKitC V4:
  OUT -> GPIO23
  S0  -> GPIO17   S1 -> GPIO16   (escalado de frecuencia: 20%)
  S2  -> GPIO18   S3 -> GPIO19   (selección de filtro: rojo/verde/azul/claro)
  LED -> VCC (3.3V, siempre encendido, ver conversación previa)
  VCC -> 3.3V     GND -> GND

Conexiones GY-VL53L0XV2 -> ESP32 DevKitC V4 (bus I2C, dirección 0x29):
  VDD -> 3.3V     GND -> GND
  SDA -> GPIO21   SCL -> GPIO22
=============================================================================
*/

// --- Pines del sensor de color TCS3200 ---
#define TCS_S0   17
#define TCS_S1   16
#define TCS_S2   18
#define TCS_S3   19
#define TCS_OUT  23

// --- Pines del sensor de distancia VL53L0X (I2C) ---
#define VL53_SDA 21
#define VL53_SCL 22

Adafruit_VL53L0X sensorDistancia = Adafruit_VL53L0X();
bool vl53Disponible = false;

// --- Calibración (portada de sensorcolorMain.c) ---
// Frecuencia (Hz) leída con una muestra BLANCA (max reflexión, mayor
// frecuencia) y una NEGRA (min reflexión, menor frecuencia) frente al
// sensor, con el filtro correspondiente activo. Para calibrar: descomenta
// los prints de frecuencia cruda, mide con tarjeta blanca y luego negra,
// y reemplaza estos valores. Los de abajo son solo un punto de partida.
#define TCS_RED_MIN     630UL
#define TCS_RED_MAX    4000UL
#define TCS_GREEN_MIN   630UL
#define TCS_GREEN_MAX  4000UL
#define TCS_BLUE_MIN    750UL
#define TCS_BLUE_MAX   4700UL

/**
 * @brief Mide la frecuencia (Hz) de la señal OUT del TCS3200 sumando la
 * duración del semiciclo alto y bajo (equivale a un periodo completo entre
 * dos flancos de subida). Equivalente en Arduino/ESP32 (pulseIn) de
 * measure_frequency() en sensorcolorMain.c, que usaba Input Capture por
 * hardware (TIM2) en el STM32.
 * @param timeoutUs Tiempo máximo de espera por flanco, en microsegundos.
 * @return Frecuencia en Hz, o 0 si no hubo pulso (timeout).
 */
uint32_t medirFrecuenciaTCS3200(uint32_t timeoutUs) {
  unsigned long alto = pulseIn(TCS_OUT, HIGH, timeoutUs);
  if (alto == 0) return 0;
  unsigned long bajo = pulseIn(TCS_OUT, LOW, timeoutUs);
  if (bajo == 0) return 0;

  unsigned long periodo = alto + bajo; // en microsegundos
  return (uint32_t)(1000000UL / periodo);
}

/**
 * @brief Mapea una frecuencia cruda al rango 0-255 usando los límites de
 * calibración (freqMin = muestra negra, freqMax = muestra blanca), saturando
 * en los extremos si la lectura se sale del rango calibrado. Portado de
 * normalize() en sensorcolorMain.c.
 */
uint8_t normalizarTCS3200(uint32_t freq, uint32_t freqMin, uint32_t freqMax) {
  if (freq <= freqMin) return 0;
  if (freq >= freqMax) return 255;
  return (uint8_t)(((freq - freqMin) * 255UL) / (freqMax - freqMin));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SENSOR DE COLOR TCS3200 + SENSOR DE DISTANCIA VL53L0X ===");

  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // Escalar la frecuencia de salida al 20% (estándar recomendado)
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  // --- Inicialización del sensor de distancia VL53L0X ---
  Wire.begin(VL53_SDA, VL53_SCL);
  if (sensorDistancia.begin()) {
    vl53Disponible = true;
    Serial.println("VL53L0X detectado correctamente.");
  } else {
    vl53Disponible = false;
    Serial.println("ERROR: no se detectó el VL53L0X. Revisa el cableado (SDA/SCL/GND/VDD).");
  }
}

void loop() {
  const uint32_t TIMEOUT_US = 100000; // 100ms

  // Rojo: S2=LOW, S3=LOW
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, LOW);
  delay(15);
  uint32_t freqRojo = medirFrecuenciaTCS3200(TIMEOUT_US);

  // Verde: S2=HIGH, S3=HIGH
  digitalWrite(TCS_S2, HIGH);
  digitalWrite(TCS_S3, HIGH);
  delay(15);
  uint32_t freqVerde = medirFrecuenciaTCS3200(TIMEOUT_US);

  // Azul: S2=LOW, S3=HIGH
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, HIGH);
  delay(15);
  uint32_t freqAzul = medirFrecuenciaTCS3200(TIMEOUT_US);

  uint8_t r = normalizarTCS3200(freqRojo,  TCS_RED_MIN,   TCS_RED_MAX);
  uint8_t g = normalizarTCS3200(freqVerde, TCS_GREEN_MIN, TCS_GREEN_MAX);
  uint8_t b = normalizarTCS3200(freqAzul,  TCS_BLUE_MIN,  TCS_BLUE_MAX);

  // Color predominante = canal normalizado con mayor valor (a mayor
  // frecuencia, mayor intensidad de reflexión en ese filtro).
  const char *colorPredominante;
  if (freqRojo == 0 && freqVerde == 0 && freqAzul == 0) {
    colorPredominante = "SIN LECTURA";
  } else if (r >= g && r >= b) {
    colorPredominante = "ROJO";
  } else if (g >= r && g >= b) {
    colorPredominante = "VERDE";
  } else {
    colorPredominante = "AZUL";
  }

  Serial.printf("Color: %s | R:%u G:%u B:%u (raw %lu,%lu,%lu Hz)\n",
                colorPredominante, r, g, b,
                (unsigned long)freqRojo, (unsigned long)freqVerde, (unsigned long)freqAzul);

  // --- Lectura de distancia (VL53L0X) ---
  if (vl53Disponible) {
    VL53L0X_RangingMeasurementData_t medida;
    sensorDistancia.rangingTest(&medida, false);

    if (medida.RangeStatus != 4) { // 4 = fuera de rango / lectura inválida
      Serial.printf("Distancia: %u mm\n", medida.RangeMilliMeter);
    } else {
      Serial.println("Distancia: fuera de rango");
    }
  } else {
    Serial.println("Distancia: sensor VL53L0X no disponible");
  }

  delay(2000);
}
