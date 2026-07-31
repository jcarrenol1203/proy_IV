#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ==========================================
// MODO DE OPERACIÓN / PRUEBA
// ==========================================
// 5 = Prueba de motores X/Y SIN finales de carrera
// 4 = Solo prueba los motores X/Y con finales de carrera
// 3 = Solo prueba el electroimán (MOSFET)
// 2 = Solo prueba el servomotor MG90S
// 1 = Solo lee e imprime color (TCS3200) y distancia (VL53L0X)
// 0 = Máquina de Estados Completa con Control MQTT y Telemetría Remota
#define MODO_OPERACION 0

// ==========================================
// CONFIGURACIÓN DE RED WIFI & BROKER MQTT
// ==========================================
// Reemplaza con el nombre y clave de tu red WiFi local (2.4 GHz)
const char* WIFI_SSID = "TU_RED_WIFI";
const char* WIFI_PASS = "TU_CONTRASEÑA_WIFI";

// Broker MQTT Gratuito (ejemplo: broker.emqx.io, broker.hivemq.com, test.mosquitto.org)
const char* MQTT_BROKER        = "broker.emqx.io";
const int   MQTT_PORT          = 1883; // Puerto TCP para microcontroladores
const char* MQTT_TOPIC_CONTROL  = "clasificador/control";
const char* MQTT_TOPIC_TELEMETRY= "clasificador/telemetry";
const char* MQTT_TOPIC_STATUS   = "clasificador/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long ultimoEnvioTelemetria = 0;

/*
=============================================================================
                  PROYECTO FINAL: CLASIFICADOR DE OBJETOS XY
=============================================================================
Control de:
- 2 Motores Paso a Paso (NEMA 17) para los ejes X e Y.
- 1 Servomotor MG90S para subir/bajar el cabezal de agarre.
- 1 Electroimán gobernado por un transistor MOSFET.
- 1 Sensor de Distancia ToF VL53L0X (I2C) para detección en Y.
- 1 Sensor de Color RGB TCS3200.
- 1 Final de carrera (Endstop) en cada eje (X e Y) para Homing.
- Conexión WiFi + MQTT para Control Manual, Visualización y Automatización Global.
=============================================================================
*/

// ==========================================
// 1. DEFINICIÓN DE PINES
// ==========================================
#define PIN_X_STEP     25
#define PIN_X_DIR      14
#define PIN_Y_STEP     26
#define PIN_Y_DIR      27

#define PIN_MOTOR_RESET 32

#define DIR_X_ADELANTE HIGH  // Aleja del final de carrera (posicionActualX aumenta)
#define DIR_X_ATRAS    LOW   // Hacia el final de carrera (posicionActualX baja a 0)
#define DIR_Y_ADELANTE HIGH  // Aleja del final de carrera (posicionActualY aumenta)
#define DIR_Y_ATRAS    LOW   // Hacia el final de carrera (posicionActualY baja a 0)

// --- Actuadores auxiliares ---
#define PIN_SERVO      13   // Servomotor MG90S
#define PIN_MOSFET_MAG  4   // Control de Electroimán

// --- Sensores ---
#define PIN_LIMIT_X    15   // Final de carrera X (Pull-up)
#define PIN_LIMIT_Y    33   // Final de carrera Y (Pull-up)

// --- Sensor de Distancia VL53L0X (I2C, dirección 0x29) ---
#define VL53_SDA       21
#define VL53_SCL       22

// --- Sensor de Color TCS3200 ---
#define TCS_S0         17
#define TCS_S1         16
#define TCS_S2         18
#define TCS_S3         19
#define TCS_OUT        23   // Entrada de pulsos

// ==========================================
// 2. CONSTANTES DE CALIBRACIÓN Y PARÁMETROS
// ==========================================
const int SERVO_ANGULO_ARRIBA = 90;   // Cabezal levantado
const int SERVO_ANGULO_ABAJO  = 10;   // Cabezal abajo (agarre)

const float PASOS_POR_MM = 40.0;     // Polea GT2 40t, 1/16 microstepping

const int DELAY_PASO_X_US        = 1000;
const int DELAY_PASO_Y_US        = 1000;
const int DELAY_HOMING_X_US      = 3500;
const int DELAY_HOMING_Y_US      = 3500;

const float LIMITE_MM_X = 380.0;
const float LIMITE_MM_Y = 180.0;

const float POS_X_INICIO_ESCANEO_MM = 50.0;

// Cajas de clasificación (mm)
const float POS_X_ROJO_MM  = 25.0;   const float POS_Y_ROJO_MM  = 30.0;
const float POS_X_VERDE_MM = 25.0;   const float POS_Y_VERDE_MM = 90.0;
const float POS_X_AZUL_MM  = 25.0;   const float POS_Y_AZUL_MM  = 150.0;

const long LIMITE_PASOS_X = (long)(LIMITE_MM_X * PASOS_POR_MM);
const long LIMITE_PASOS_Y = (long)(LIMITE_MM_Y * PASOS_POR_MM);
const long POS_X_INICIO_ESCANEO = (long)(POS_X_INICIO_ESCANEO_MM * PASOS_POR_MM);

const long POS_X_ROJO  = (long)(POS_X_ROJO_MM  * PASOS_POR_MM);
const long POS_Y_ROJO  = (long)(POS_Y_ROJO_MM  * PASOS_POR_MM);
const long POS_X_VERDE = (long)(POS_X_VERDE_MM * PASOS_POR_MM);
const long POS_Y_VERDE = (long)(POS_Y_VERDE_MM * PASOS_POR_MM);
const long POS_X_AZUL  = (long)(POS_X_AZUL_MM  * PASOS_POR_MM);
const long POS_Y_AZUL  = (long)(POS_Y_AZUL_MM  * PASOS_POR_MM);

const float POS_Y_LECTURA_COLOR_MM = 0.0;
const long POS_Y_LECTURA_COLOR = (long)(POS_Y_LECTURA_COLOR_MM * PASOS_POR_MM);

const float VL53_RANGO_MAX_MM = 150.0;
const int PASOS_ENTRE_LECTURAS_ESCANEO = 40;

#define TCS_RED_MIN     630UL
#define TCS_RED_MAX    4000UL
#define TCS_GREEN_MIN   630UL
#define TCS_GREEN_MAX  4000UL
#define TCS_BLUE_MIN    750UL
#define TCS_BLUE_MAX   4700UL

// ==========================================
// 3. VARIABLES DE ESTADO Y OBJETOS
// ==========================================
Servo miServo;
Adafruit_VL53L0X sensorDistancia = Adafruit_VL53L0X();
bool vl53Disponible = false;

enum EstadoSistema {
  ESTADO_MANUAL,
  ESTADO_HOMING,
  ESTADO_ESCANEO,
  ESTADO_GOTO_OBJETO,
  ESTADO_AGARRE,
  ESTADO_LORE_COLOR,
  ESTADO_DEPOSITAR,
  ESTADO_RETORNO
};

EstadoSistema estadoActual = ESTADO_MANUAL;

long posicionActualX = 0;
long posicionActualY = 0;

long objetoDetectadoX = 0;
long objetoDetectadoY = 0;

enum TipoColor { ROJO, VERDE, AZUL, DESCONOCIDO };
TipoColor colorDetectado = DESCONOCIDO;

bool electroimanEstado = false;
bool servoEstadoAbajo = false;

int contadorRojo = 0;
int contadorVerde = 0;
int contadorAzul = 0;
uint16_t ultimaDistanciaMm = 0;

// Declaraciones previas de funciones
void publicarTelemetria();
void moverACoordenadas(long xDestino, long yDestino);
void resetControladoresMotor();
void homingXY();

// Helper nombres de estado y color
const char* getEstadoNombre(EstadoSistema e) {
  switch (e) {
    case ESTADO_MANUAL: return "MANUAL";
    case ESTADO_HOMING: return "HOMING";
    case ESTADO_ESCANEO: return "ESCANEO";
    case ESTADO_GOTO_OBJETO: return "GOTO_OBJETO";
    case ESTADO_AGARRE: return "AGARRE";
    case ESTADO_LORE_COLOR: return "LECTURA_COLOR";
    case ESTADO_DEPOSITAR: return "DEPOSITAR";
    case ESTADO_RETORNO: return "RETORNO";
    default: return "DESCONOCIDO";
  }
}

const char* getColorNombre(TipoColor c) {
  switch (c) {
    case ROJO: return "ROJO";
    case VERDE: return "VERDE";
    case AZUL: return "AZUL";
    default: return "DESCONOCIDO";
  }
}

// ==========================================
// 4. FUNCIONES DE TELEMETRÍA Y MQTT
// ==========================================
void publicarTelemetria() {
  if (!mqttClient.connected()) return;

  JsonDocument doc;
  doc["state"] = getEstadoNombre(estadoActual);
  doc["x_mm"] = posicionActualX / PASOS_POR_MM;
  doc["y_mm"] = posicionActualY / PASOS_POR_MM;
  doc["x_pasos"] = posicionActualX;
  doc["y_pasos"] = posicionActualY;
  doc["magnet"] = electroimanEstado;
  doc["servo"] = servoEstadoAbajo ? "ABAJO" : "ARRIBA";
  doc["dist_mm"] = ultimaDistanciaMm;
  doc["color"] = getColorNombre(colorDetectado);

  doc["counts"]["red"] = contadorRojo;
  doc["counts"]["green"] = contadorVerde;
  doc["counts"]["blue"] = contadorAzul;

  char buffer[384];
  serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC_TELEMETRY, buffer);
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.println("[MQTT Error] Error al parsear JSON");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  Serial.printf("[MQTT RX] Comando: %s\n", cmd);

  if (strcmp(cmd, "START_SCAN") == 0) {
    estadoActual = ESTADO_ESCANEO;
    publicarTelemetria();
  } else if (strcmp(cmd, "STOP") == 0) {
    estadoActual = ESTADO_MANUAL;
    publicarTelemetria();
  } else if (strcmp(cmd, "HOMING") == 0) {
    estadoActual = ESTADO_HOMING;
    publicarTelemetria();
  } else if (strcmp(cmd, "MAGNET") == 0) {
    bool state = doc["state"];
    electroimanEstado = state;
    digitalWrite(PIN_MOSFET_MAG, state ? HIGH : LOW);
    Serial.printf("[ELECTROIMAN] Control manual: %s\n", state ? "ENCENDIDO" : "APAGADO");
    publicarTelemetria();
  } else if (strcmp(cmd, "SERVO") == 0) {
    const char* sState = doc["state"];
    if (sState && (strcmp(sState, "DOWN") == 0 || strcmp(sState, "ABAJO") == 0)) {
      miServo.write(SERVO_ANGULO_ABAJO);
      servoEstadoAbajo = true;
      Serial.println("[SERVO] Control manual: ABAJO");
    } else {
      miServo.write(SERVO_ANGULO_ARRIBA);
      servoEstadoAbajo = false;
      Serial.println("[SERVO] Control manual: ARRIBA");
    }
    publicarTelemetria();
  } else if (strcmp(cmd, "MOVE") == 0) {
    if (doc["x"].is<float>() && doc["y"].is<float>()) {
      float xMm = doc["x"];
      float yMm = doc["y"];
      long targetX = (long)(xMm * PASOS_POR_MM);
      long targetY = (long)(yMm * PASOS_POR_MM);
      moverACoordenadas(targetX, targetY);
      publicarTelemetria();
    }
  } else if (strcmp(cmd, "MOVE_REL") == 0) {
    float dx = doc["dx"] | 0.0;
    float dy = doc["dy"] | 0.0;
    long targetX = posicionActualX + (long)(dx * PASOS_POR_MM);
    long targetY = posicionActualY + (long)(dy * PASOS_POR_MM);
    moverACoordenadas(targetX, targetY);
    publicarTelemetria();
  } else if (strcmp(cmd, "RESET_DRIVERS") == 0) {
    resetControladoresMotor();
    publicarTelemetria();
  }
}

void reconectarMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] Conectando a broker...");
    String clientId = "ESP32Clasificador-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" ¡Conectado con éxito!");
      mqttClient.subscribe(MQTT_TOPIC_CONTROL);
      mqttClient.publish(MQTT_TOPIC_STATUS, "{\"online\":true}");
      publicarTelemetria();
    } else {
      Serial.printf(" Falló con estado %d. Reintentando en 3 segundos...\n", mqttClient.state());
      delay(3000);
    }
  }
}

void conectarWiFi() {
  Serial.printf("[WIFI] Conectando a %s...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] ¡Conectado! IP asignada: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] No se pudo conectar. Operando en modo local (Serial/MQTT diferido).");
  }
}

// ==========================================
// 5. FUNCIONES DE CONTROL DE MOTORES
// ==========================================
void darPaso(int stepPin, int dirPin, int sentido, int delayMicrosegundos) {
  digitalWrite(dirPin, sentido);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayMicrosegundos);
}

void resetControladoresMotor() {
  Serial.println("[RESET] Reseteando controladores de motor...");
  digitalWrite(PIN_MOTOR_RESET, LOW);
  delay(10);
  digitalWrite(PIN_MOTOR_RESET, HIGH);
  delay(10);
  Serial.println("[RESET] Listo.");
}

bool finalDeCarreraPresionado(int pin) {
  const int LECTURAS = 5;
  const int DELAY_ENTRE_LECTURAS_MS = 2;
  for (int i = 0; i < LECTURAS; i++) {
    if (digitalRead(pin) == LOW) return false;
    delay(DELAY_ENTRE_LECTURAS_MS);
  }
  return true;
}

void moverACoordenadas(long xDestino, long yDestino) {
  xDestino = constrain(xDestino, 0, LIMITE_PASOS_X);
  yDestino = constrain(yDestino, 0, LIMITE_PASOS_Y);

  Serial.printf("[MOVE] Moviendo a X:%ld, Y:%ld (Actual X:%ld, Y:%ld)\n", xDestino, yDestino, posicionActualX, posicionActualY);

  // --- EJE X ---
  int dirX = (xDestino >= posicionActualX) ? DIR_X_ADELANTE : DIR_X_ATRAS;
  long pasosX = abs(xDestino - posicionActualX);
  for (long i = 0; i < pasosX; i++) {
    if (dirX == DIR_X_ATRAS && finalDeCarreraPresionado(PIN_LIMIT_X)) {
      posicionActualX = 0;
      break;
    }
    darPaso(PIN_X_STEP, PIN_X_DIR, dirX, DELAY_PASO_X_US);
    posicionActualX += (dirX == DIR_X_ADELANTE) ? 1 : -1;

    if (i % 150 == 0) mqttClient.loop();
  }

  // --- EJE Y ---
  int dirY = (yDestino >= posicionActualY) ? DIR_Y_ADELANTE : DIR_Y_ATRAS;
  long pasosY = abs(yDestino - posicionActualY);
  for (long i = 0; i < pasosY; i++) {
    if (dirY == DIR_Y_ATRAS && finalDeCarreraPresionado(PIN_LIMIT_Y)) {
      posicionActualY = 0;
      break;
    }
    darPaso(PIN_Y_STEP, PIN_Y_DIR, dirY, DELAY_PASO_Y_US);
    posicionActualY += (dirY == DIR_Y_ADELANTE) ? 1 : -1;

    if (i % 150 == 0) mqttClient.loop();
  }

  Serial.printf("[POSICION] X: %.1f mm | Y: %.1f mm\n", posicionActualX / PASOS_POR_MM, posicionActualY / PASOS_POR_MM);
}

void homingXY() {
  Serial.println("[HOMING] Eje Y...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_Y)) {
    darPaso(PIN_Y_STEP, PIN_Y_DIR, DIR_Y_ATRAS, DELAY_HOMING_Y_US);
    mqttClient.loop();
  }
  posicionActualY = 0;

  Serial.println("[HOMING] Eje X...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_X)) {
    darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ATRAS, DELAY_HOMING_X_US);
    mqttClient.loop();
  }
  posicionActualX = 0;

  Serial.println("[HOMING] Completado.");
}

// ==========================================
// 6. LECTURA DE SENSORES
// ==========================================
bool leerSensorDistancia(long &pasosYCalculados) {
  if (!vl53Disponible) return false;

  VL53L0X_RangingMeasurementData_t medida;
  sensorDistancia.rangingTest(&medida, false);

  if (medida.RangeStatus == 4) return false;

  float distanciaMm = medida.RangeMilliMeter;
  ultimaDistanciaMm = (uint16_t)distanciaMm;

  if (distanciaMm > VL53_RANGO_MAX_MM || distanciaMm > LIMITE_MM_Y) return false;

  long pasosCalculados = (long)(distanciaMm * PASOS_POR_MM);
  pasosYCalculados = constrain(pasosCalculados, 0, LIMITE_PASOS_Y);
  return true;
}

uint32_t medirFrecuenciaTCS3200(uint32_t timeoutUs) {
  unsigned long alto = pulseIn(TCS_OUT, HIGH, timeoutUs);
  if (alto == 0) return 0;
  unsigned long bajo = pulseIn(TCS_OUT, LOW, timeoutUs);
  if (bajo == 0) return 0;

  unsigned long periodo = alto + bajo;
  return (uint32_t)(1000000UL / periodo);
}

uint8_t normalizarTCS3200(uint32_t freq, uint32_t freqMin, uint32_t freqMax) {
  if (freq <= freqMin) return 0;
  if (freq >= freqMax) return 255;
  return (uint8_t)(((freq - freqMin) * 255UL) / (freqMax - freqMin));
}

TipoColor obtenerColorTCS3200() {
  const uint32_t TIMEOUT_US = 20000;

  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, LOW);
  uint32_t freqRojo = medirFrecuenciaTCS3200(TIMEOUT_US);

  digitalWrite(TCS_S2, HIGH);
  digitalWrite(TCS_S3, HIGH);
  uint32_t freqVerde = medirFrecuenciaTCS3200(TIMEOUT_US);

  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, HIGH);
  uint32_t freqAzul = medirFrecuenciaTCS3200(TIMEOUT_US);

  if (freqRojo == 0 && freqVerde == 0 && freqAzul == 0) {
    Serial.println("[COLOR] Sin lectura.");
    return DESCONOCIDO;
  }

  uint8_t r = normalizarTCS3200(freqRojo,  TCS_RED_MIN,   TCS_RED_MAX);
  uint8_t g = normalizarTCS3200(freqVerde, TCS_GREEN_MIN, TCS_GREEN_MAX);
  uint8_t b = normalizarTCS3200(freqAzul,  TCS_BLUE_MIN,  TCS_BLUE_MAX);

  Serial.printf("[COLOR] R:%u G:%u B:%u\n", r, g, b);

  if (r >= g && r >= b) return ROJO;
  if (g >= r && g >= b) return VERDE;
  return AZUL;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

#if MODO_OPERACION == 0
  Serial.println("=== INICIALIZANDO CLASIFICADOR XY CON CONTROL MQTT ===");

  pinMode(PIN_MOTOR_RESET, OUTPUT);
  digitalWrite(PIN_MOTOR_RESET, HIGH);
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Y_DIR, OUTPUT);

  pinMode(PIN_MOSFET_MAG, OUTPUT);
  digitalWrite(PIN_MOSFET_MAG, LOW);

  miServo.attach(PIN_SERVO);
  miServo.write(SERVO_ANGULO_ARRIBA);

  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);

  Wire.begin(VL53_SDA, VL53_SCL);
  if (sensorDistancia.begin()) {
    vl53Disponible = true;
    Serial.println("VL53L0X inicializado.");
  } else {
    vl53Disponible = false;
    Serial.println("ERROR: VL53L0X no detectado.");
  }

  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  // Conexión WiFi & MQTT
  conectarWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(callbackMQTT);
  mqttClient.setBufferSize(512);

  if (WiFi.status() == WL_CONNECTED) {
    reconectarMQTT();
  }
#endif
}

// ==========================================
// BUCLE PRINCIPAL
// ==========================================
#if MODO_OPERACION == 0
void loop() {
  // Mantenimiento de conexión WiFi & MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconectarMQTT();
    }
    mqttClient.loop();
  }

  // Envío periódico de telemetría (cada 300 ms)
  if (millis() - ultimoEnvioTelemetria > 300) {
    ultimoEnvioTelemetria = millis();
    publicarTelemetria();
  }

  // Máquina de estados
  switch (estadoActual) {

    case ESTADO_MANUAL:
      // Espera de comandos manuales recibidos por MQTT o Serial
      delay(10);
      break;

    case ESTADO_HOMING:
      Serial.println("[MAQUINA] Estado: HOMING...");
      electroimanEstado = false;
      digitalWrite(PIN_MOSFET_MAG, LOW);
      miServo.write(SERVO_ANGULO_ARRIBA);
      servoEstadoAbajo = false;
      delay(300);

      homingXY();
      estadoActual = ESTADO_MANUAL;
      publicarTelemetria();
      break;

    case ESTADO_ESCANEO:
      Serial.println("[MAQUINA] Estado: ESCANEO...");
      if (posicionActualX < POS_X_INICIO_ESCANEO) {
        moverACoordenadas(POS_X_INICIO_ESCANEO, posicionActualY);
      }

      while (posicionActualX < LIMITE_PASOS_X && estadoActual == ESTADO_ESCANEO) {
        for (int i = 0; i < PASOS_ENTRE_LECTURAS_ESCANEO && posicionActualX < LIMITE_PASOS_X; i++) {
          darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ADELANTE, 1500);
          posicionActualX++;
        }

        mqttClient.loop();

        long yCalculado = 0;
        if (leerSensorDistancia(yCalculado)) {
          objetoDetectadoX = posicionActualX;
          objetoDetectadoY = yCalculado;
          Serial.printf("[DETECCION] Objeto en X:%ld, Y:%ld\n", objetoDetectadoX, objetoDetectadoY);
          estadoActual = ESTADO_GOTO_OBJETO;
          break;
        }
      }

      if (estadoActual == ESTADO_ESCANEO) {
        Serial.println("[MAQUINA] Fin de escaneo. Retornando...");
        estadoActual = ESTADO_HOMING;
      }
      break;

    case ESTADO_GOTO_OBJETO:
      Serial.println("[MAQUINA] Ir a Posición de Objeto...");
      moverACoordenadas(objetoDetectadoX, objetoDetectadoY);
      delay(400);
      estadoActual = ESTADO_AGARRE;
      break;

    case ESTADO_AGARRE:
      Serial.println("[MAQUINA] Agarre con Electroimán...");
      miServo.write(SERVO_ANGULO_ABAJO);
      servoEstadoAbajo = true;
      publicarTelemetria();
      delay(800);

      digitalWrite(PIN_MOSFET_MAG, HIGH);
      electroimanEstado = true;
      publicarTelemetria();
      delay(800);

      miServo.write(SERVO_ANGULO_ARRIBA);
      servoEstadoAbajo = false;
      publicarTelemetria();
      delay(800);

      moverACoordenadas(posicionActualX, POS_Y_LECTURA_COLOR);
      estadoActual = ESTADO_LORE_COLOR;
      break;

    case ESTADO_LORE_COLOR:
      Serial.println("[MAQUINA] Lectura de Color...");
      colorDetectado = obtenerColorTCS3200();
      publicarTelemetria();
      estadoActual = ESTADO_DEPOSITAR;
      break;

    case ESTADO_DEPOSITAR:
      Serial.println("[MAQUINA] Clasificando y Depositando...");
      long destinoX, destinoY;
      if (colorDetectado == ROJO) {
        destinoX = POS_X_ROJO; destinoY = POS_Y_ROJO;
        contadorRojo++;
      } else if (colorDetectado == VERDE) {
        destinoX = POS_X_VERDE; destinoY = POS_Y_VERDE;
        contadorVerde++;
      } else {
        destinoX = POS_X_AZUL; destinoY = POS_Y_AZUL;
        contadorAzul++;
      }

      moverACoordenadas(destinoX, destinoY);
      delay(400);

      miServo.write(SERVO_ANGULO_ABAJO);
      servoEstadoAbajo = true;
      publicarTelemetria();
      delay(600);

      digitalWrite(PIN_MOSFET_MAG, LOW);
      electroimanEstado = false;
      publicarTelemetria();
      delay(800);

      miServo.write(SERVO_ANGULO_ARRIBA);
      servoEstadoAbajo = false;
      publicarTelemetria();
      delay(400);

      estadoActual = ESTADO_RETORNO;
      break;

    case ESTADO_RETORNO:
      Serial.println("[MAQUINA] Retornando a Inicio de Escaneo...");
      estadoActual = ESTADO_ESCANEO;
      break;
  }
}
#endif
