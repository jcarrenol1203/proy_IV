#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ==========================================
// MODO DE OPERACIÓN / PRUEBA
// ==========================================
// 5 = Prueba de motores X/Y SIN finales de carrera: movimiento manual por
//     pasos relativos, sin Homing ni límites (ni de software ni físicos).
//     Comandos por Serial: "X<pasos>" o "Y<pasos>" (ej: "X200", "Y-100";
//     positivo=adelante, negativo=atrás). Útil para confirmar que los
//     motores/drivers no quedaron dañados, aislado del problema de los
//     finales de carrera. No inicializa ni usa finales de carrera, servo,
//     electroimán, VL53L0X ni TCS3200. Mueve pocos pasos primero.
// 4 = Solo prueba los motores X/Y (Homing con finales de carrera + límites
//     por software de la zona de trabajo). Al iniciar hace Homing en ambos
//     ejes, imprime las zonas configuradas, y luego acepta comandos por
//     Serial: "X<mm>" o "Y<mm>" (ej: "X200", "Y-50") para moverse ese
//     tanto relativo a la posición actual (positivo=adelante,
//     negativo=atrás), "H" para rehacer Homing, "R" para resetear los
//     drivers, o "Z" para volver a imprimir la zona de operación y las
//     zonas de almacenamiento (en pasos y mm). No inicializa ni usa servo,
//     electroimán, VL53L0X ni TCS3200.
// 3 = Solo prueba el electroimán (MOSFET): se activa/desactiva por comandos
//     Serial ('1' = encender, '0' = apagar). No inicializa ni usa ningún
//     otro pin (motores, servo, finales de carrera, VL53L0X, TCS3200), así
//     que sirve con el ESP32 y el electroimán solos, sin nada más conectado.
// 2 = Solo prueba el servomotor MG90S: lo mueve entre SERVO_ANGULO_ARRIBA y
//     SERVO_ANGULO_ABAJO en bucle. No inicializa ni usa ningún otro pin
//     (motores, electroimán, finales de carrera, VL53L0X, TCS3200), así que
//     sirve con el ESP32 y el servo solos, sin nada más conectado.
// 1 = Solo lee e imprime color (TCS3200) y distancia (VL53L0X) por Serial
//     cada 2s, para verificar que ambos sensores funcionan. No usa ni
//     configura los pines de motores/servo/electroimán/finales de carrera,
//     así que sirve aunque solo tengas los sensores conectados.
// 0 = Corre la máquina de estados completa del clasificador XY (requiere
//     todo el hardware físicamente armado).
#define MODO_OPERACION 0

/*
=============================================================================
                  PROYECTO FINAL: CLASIFICADOR DE OBJETOS XY
=============================================================================
Este proyecto implementa el control de:
- 2 Motores Paso a Paso (NEMA 17) para los ejes X e Y.
- 1 Servomotor MG90S para subir/bajar el cabezal de agarre.
- 1 Electroimán gobernado por un transistor MOSFET.
- 1 Sensor de Distancia ToF VL53L0X (GY-VL53L0XV2, I2C) para detección en el
  eje Y.
- 1 Sensor de Color RGB TCS3200 (salida de frecuencia cuadrada).
- 1 Final de carrera (Endstop) en cada eje (X e Y) para el proceso de Homing.

FLUJO DEL SISTEMA:
1. Homing Ejes X e Y: Se mueven los carros hacia atrás hasta chocar con los
   finales de carrera.
2. Escaneo: El carro X avanza paso a paso mientras el sensor VL53L0X mide la
   distancia (posición Y) del objeto.
3. Detección: Si el VL53L0X detecta un objeto dentro del rango útil, se
   calcula su posición (X, Y) a partir de la distancia medida en mm.
4. Agarre: El carro se posiciona sobre el objeto, baja el servo, enciende el
   electroimán y sube el servo.
5. Lectura de Color: El sensor TCS3200 detecta si el objeto es Rojo, Verde o
   Azul.
6. Clasificación: Los motores llevan el objeto al contenedor correspondiente
   a su color, apagan el electroimán para soltarlo y el sistema vuelve a
   iniciar el escaneo.
=============================================================================
*/

// ==========================================
// 1. DEFINICIÓN DE PINES
// ==========================================
// --- Motores Paso a Paso (Drivers A4988 / DRV8825) ---
// PIN_X_STEP iba en GPIO12, pero es un pin de arranque (strapping pin, MTDI)
// que define el voltaje del regulador de flash del ESP32: si el driver lo
// mantiene en un nivel incorrecto durante el reset/boot, puede interferir
// con la carga de firmware (fallos tipo "Packet content transfer stopped").
// Se mueve a GPIO25 (libre, sin uso especial, soporta salida digital normal).
#define PIN_X_STEP     25
#define PIN_X_DIR      14
#define PIN_Y_STEP     26
#define PIN_Y_DIR      27

// RESET (unido a SLEEP en la placa del driver, activo en LOW) de ambos
// drivers X e Y, conectados juntos a este mismo pin. Permite limpiar por
// software una posible protección por sobrecorriente/térmica enclavada, sin
// tener que desconectar físicamente la alimentación de potencia (VMOT).
#define PIN_MOTOR_RESET 32

// Sentido físico de cada eje (confirmado por prueba física con el motor y
// driver reales). Si algún eje se mueve hacia el lado contrario al esperado,
// basta con intercambiar sus dos valores (HIGH <-> LOW) en vez de tocar la
// lógica de Homing/movimiento.
#define DIR_X_ADELANTE HIGH  // Aleja del final de carrera (posicionActualX aumenta)
#define DIR_X_ATRAS    LOW   // Hacia el final de carrera (posicionActualX baja hasta 0)
#define DIR_Y_ADELANTE HIGH  // Aleja del final de carrera (posicionActualY aumenta)
#define DIR_Y_ATRAS    LOW   // Hacia el final de carrera (posicionActualY baja hasta 0)

// --- Actuadores auxiliares ---
#define PIN_SERVO      13   // Servomotor MG90S
#define PIN_MOSFET_MAG  4   // Control de Electroimán

// --- Sensores ---
#define PIN_LIMIT_X    15   // Final de carrera de X (Normalmente Abierto, Pull-up)
// PIN_LIMIT_Y iba en GPIO2, pero es un pin de arranque (strapping pin) del
// ESP32: si el final de carrera lo jala a un nivel incorrecto durante el
// reset/boot, el ESP32 puede fallar al arrancar o comportarse de forma
// errática. Se mueve a GPIO33 (libre, sin uso especial, soporta INPUT_PULLUP).
#define PIN_LIMIT_Y    33   // Final de carrera de Y (Normalmente Abierto, Pull-up)

// --- Sensor de Distancia VL53L0X (I2C, dirección 0x29) ---
#define VL53_SDA       21
#define VL53_SCL       22

// --- Sensor de Color TCS3200 ---
#define TCS_S0         17
#define TCS_S1         16
#define TCS_S2         18
#define TCS_S3         19
#define TCS_OUT        23   // Entrada de pulsos de frecuencia

// ==========================================
// 2. CONSTANTES DE CALIBRACIÓN Y PARÁMETROS
// ==========================================
// --- Posiciones de Servomotor ---
const int SERVO_ANGULO_ARRIBA = 90;   // Cabezal levantado (seguro)
const int SERVO_ANGULO_ABAJO  = 10;  // Cabezal abajo (agarre)

// --- Relación de transmisión (Pasos por milímetro) ---
// PASOS_POR_MM = (pasos_motor_por_vuelta x microstepping) / (dientes_polea x paso_correa_mm)
// Motor de 1.8°/paso (200 pasos/rev), correa GT2 (paso de 2mm), polea de
// ~40 dientes (ESTIMADA a partir del diámetro medido ~24.8mm; confirmar
// contando los dientes físicamente), con drivers DRV8825 en 1/16 de
// microstepping (M0=GND, M1=GND, M2=3.3V):
//   (200 x 16) / (40 x 2) = 40 pasos/mm
// Si se confirma un número de dientes distinto, recalcular con la fórmula.
// Si se vuelve a full step (M0/M1/M2 en GND), dividir este valor entre 16.
const float PASOS_POR_MM = 40.0;     // Polea de 40 dientes (estimada), 1/16 de microstepping

// --- Velocidad de motores (delay entre pasos en microsegundos: menor = más rápido) ---
// Valores confirmados con pruebas físicas de X e Y moviéndose juntos.
const int DELAY_PASO_X_US        = 1000;  // Movimiento normal eje X
const int DELAY_PASO_Y_US        = 1000;  // Movimiento normal eje Y
const int DELAY_HOMING_X_US      = 3500;  // Homing eje X (más lento = toque más suave contra el final de carrera)
const int DELAY_HOMING_Y_US      = 3500;  // Homing eje Y (más lento = toque más suave contra el final de carrera)

// --- Límites Físicos del Área de Trabajo (en milímetros) ---
// Medidos por prueba física: tope mecánico de X en 15200 pasos, de Y en
// 7200 pasos (con PASOS_POR_MM=40 -> 380mm y 180mm).
const float LIMITE_MM_X = 380.0;
const float LIMITE_MM_Y = 180.0;

// --- Zona de operación (escaneo) en X ---
// El escaneo (ESTADO_ESCANEO) no arranca desde X=0, sino desde aquí. La
// franja de X entre 0 y este valor queda fuera del área de escaneo y se usa
// para las zonas de almacenamiento (ver más abajo).
const float POS_X_INICIO_ESCANEO_MM = 50.0;

// --- Posición Física de las Cajas de Clasificación (en milímetros) ---
// Ancladas en la franja de X reservada (fuera de la zona de escaneo, ver
// POS_X_INICIO_ESCANEO_MM), diferenciadas por Y en 3 zonas iguales dentro
// de LIMITE_MM_Y (180mm / 3 = 60mm por zona, tomando el punto medio de cada
// una). Ajustar POS_X_*_MM si el almacenamiento no queda a esa X exacta.
const float POS_X_ROJO_MM  = 25.0;   const float POS_Y_ROJO_MM  = 30.0;   // Zona 1: 0-60mm
const float POS_X_VERDE_MM = 25.0;   const float POS_Y_VERDE_MM = 90.0;   // Zona 2: 60-120mm
const float POS_X_AZUL_MM  = 25.0;   const float POS_Y_AZUL_MM  = 150.0;  // Zona 3: 120-180mm

// --- CONVERSIÓN AUTOMÁTICA A PASOS (Calculado en tiempo de compilación) ---
const long LIMITE_PASOS_X = (long)(LIMITE_MM_X * PASOS_POR_MM);
const long LIMITE_PASOS_Y = (long)(LIMITE_MM_Y * PASOS_POR_MM);
const long POS_X_INICIO_ESCANEO = (long)(POS_X_INICIO_ESCANEO_MM * PASOS_POR_MM);

const long POS_X_ROJO  = (long)(POS_X_ROJO_MM  * PASOS_POR_MM);
const long POS_Y_ROJO  = (long)(POS_Y_ROJO_MM  * PASOS_POR_MM);
const long POS_X_VERDE = (long)(POS_X_VERDE_MM * PASOS_POR_MM);
const long POS_Y_VERDE = (long)(POS_Y_VERDE_MM * PASOS_POR_MM);
const long POS_X_AZUL  = (long)(POS_X_AZUL_MM  * PASOS_POR_MM);
const long POS_Y_AZUL  = (long)(POS_Y_AZUL_MM  * PASOS_POR_MM);

// --- Posición Y de lectura de color ---
// El sensor TCS3200 está anclado al chasis directamente en el home de Y (no
// viaja en el cabezal móvil), así que tras agarrar el objeto hay que
// acercarlo a Y=0 (manteniendo la X donde se agarró) antes de leer el color.
const float POS_Y_LECTURA_COLOR_MM = 0.0;
const long POS_Y_LECTURA_COLOR = (long)(POS_Y_LECTURA_COLOR_MM * PASOS_POR_MM);

// --- Rango útil del VL53L0X para considerar que hay un objeto en frente ---
// Si la distancia medida es mayor a este valor (o la lectura es inválida),
// se asume que no hay objeto presente en esa posición X del escaneo.
const float VL53_RANGO_MAX_MM = 150.0;

// --- Pasos de X entre cada lectura del VL53L0X durante el escaneo ---
// El VL53L0X tarda ~30-50ms por medición; leerlo en cada paso individual
// frena el escaneo. Con 40 pasos/mm, 40 pasos = 1mm de resolución espacial
// del escaneo entre lecturas.
const int PASOS_ENTRE_LECTURAS_ESCANEO = 40;

// --- Calibración del Sensor de Color TCS3200 (portado de sensorcolorMain.c) ---
// Frecuencia (Hz) leída con una muestra BLANCA (max reflexión, mayor
// frecuencia) y una NEGRA (min reflexión, menor frecuencia) frente al sensor,
// con el filtro correspondiente activo. Para calibrar: imprime las
// frecuencias crudas (freqRojo/freqVerde/freqAzul) con una tarjeta blanca y
// luego una negra frente al sensor, y reemplaza estos valores. Los de abajo
// son solo un punto de partida.
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

// Estados de la Máquina de Estados
enum EstadoSistema {
  ESTADO_HOMING,
  ESTADO_ESCANEO,
  ESTADO_GOTO_OBJETO,
  ESTADO_AGARRE,
  ESTADO_LORE_COLOR,
  ESTADO_DEPOSITAR,
  ESTADO_RETORNO
};

EstadoSistema estadoActual = ESTADO_HOMING;

long posicionActualX = 0;
long posicionActualY = 0;

// Variables donde se guardará la posición detectada del objeto
long objetoDetectadoX = 0;
long objetoDetectadoY = 0;

// Colores posibles
enum TipoColor { ROJO, VERDE, AZUL, DESCONOCIDO };
TipoColor colorDetectado = DESCONOCIDO;

// ==========================================
// 4. FUNCIONES DE CONTROL DE MOTORES
// ==========================================

/**
 * @brief Genera un paso físico en un motor determinado con la dirección indicada.
 * @param stepPin Pin de STEP
 * @param dirPin Pin de DIR
 * @param sentido Nivel a escribir en dirPin (usar DIR_X_ADELANTE/ATRAS o DIR_Y_ADELANTE/ATRAS)
 * @param delayMicrosegundos Tiempo entre pulsos (controla la velocidad, menor = más rápido)
 */
void darPaso(int stepPin, int dirPin, int sentido, int delayMicrosegundos) {
  digitalWrite(dirPin, sentido);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10); // Duración mínima del pulso
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayMicrosegundos);
}

/**
 * @brief Pulsa RESET (LOW breve y luego HIGH) en ambos drivers de motor para
 * limpiar su estado interno, incluyendo una posible protección por
 * sobrecorriente/térmica que haya quedado enclavada tras un uso anterior.
 * Requiere PIN_MOTOR_RESET ya configurado como OUTPUT.
 */
void resetControladoresMotor() {
  Serial.println("[RESET] Reseteando controladores de motor...");
  digitalWrite(PIN_MOTOR_RESET, LOW);
  delay(10);
  digitalWrite(PIN_MOTOR_RESET, HIGH);
  delay(10);
  Serial.println("[RESET] Listo.");
}

/**
 * @brief Debounce anti-ruido para los finales de carrera: solo los considera
 * presionados si N lecturas seguidas dan HIGH (los switches físicos están
 * cableados en reposo=LOW, presionado=HIGH). Sale de inmediato apenas
 * encuentra una lectura LOW, así que en operación normal (switch libre) no
 * penaliza la velocidad del movimiento/Homing.
 */
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
  // Límites por Software (Soft Limits) para prevenir que la máquina choque en los extremos opuestos
  xDestino = constrain(xDestino, 0, LIMITE_PASOS_X);
  yDestino = constrain(yDestino, 0, LIMITE_PASOS_Y);

  Serial.printf("[MOVE] Moviendo a X:%ld, Y:%ld (Actual X:%ld, Y:%ld)\n", xDestino, yDestino, posicionActualX, posicionActualY);

  // --- EJE X ---
  int dirX = (xDestino >= posicionActualX) ? DIR_X_ADELANTE : DIR_X_ATRAS;
  long pasosX = abs(xDestino - posicionActualX);
  for (long i = 0; i < pasosX; i++) {
    // Si vamos hacia atrás (GND/Home) y tocamos el final de carrera, nos detenemos por seguridad.
    if (dirX == DIR_X_ATRAS && finalDeCarreraPresionado(PIN_LIMIT_X)) {
      posicionActualX = 0;
      break;
    }
    darPaso(PIN_X_STEP, PIN_X_DIR, dirX, DELAY_PASO_X_US);
    posicionActualX += (dirX == DIR_X_ADELANTE) ? 1 : -1;
  }

  // --- EJE Y ---
  int dirY = (yDestino >= posicionActualY) ? DIR_Y_ADELANTE : DIR_Y_ATRAS;
  long pasosY = abs(yDestino - posicionActualY);
  for (long i = 0; i < pasosY; i++) {
    // Si vamos hacia atrás (GND/Home) y tocamos el final de carrera Y, nos detenemos.
    if (dirY == DIR_Y_ATRAS && finalDeCarreraPresionado(PIN_LIMIT_Y)) {
      posicionActualY = 0;
      break;
    }
    darPaso(PIN_Y_STEP, PIN_Y_DIR, dirY, DELAY_PASO_Y_US);
    posicionActualY += (dirY == DIR_Y_ADELANTE) ? 1 : -1;
  }

  Serial.printf("[POSICION] Desde home -> X:%ld pasos (%.1f mm) | Y:%ld pasos (%.1f mm)\n",
                posicionActualX, posicionActualX / PASOS_POR_MM,
                posicionActualY, posicionActualY / PASOS_POR_MM);
}

/**
 * @brief Lleva ambos ejes hacia atrás hasta presionar sus finales de carrera
 * y calibra posicionActualX/Y en 0. Requiere PIN_LIMIT_X/Y en INPUT_PULLUP y
 * los pines de los motores ya configurados como OUTPUT.
 */
void homingXY() {
  Serial.println("[HOMING] Eje Y...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_Y)) {
    darPaso(PIN_Y_STEP, PIN_Y_DIR, DIR_Y_ATRAS, DELAY_HOMING_Y_US);
  }
  posicionActualY = 0; // Origen Y calibrado

  Serial.println("[HOMING] Eje X...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_X)) {
    darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ATRAS, DELAY_HOMING_X_US);
  }
  posicionActualX = 0; // Origen X calibrado

  Serial.println("[HOMING] Completado con éxito.");
}

/**
 * @brief Imprime por Serial, en pasos y en mm, la zona máxima de operación
 * (LIMITE_PASOS_X/Y) y las zonas de almacenamiento (posiciones de las cajas
 * ROJO/VERDE/AZUL). Útil para verificar la calibración después de hacer
 * Homing, antes de correr el sistema completo.
 */
void imprimirZonas() {
  Serial.println("=== ZONA DE OPERACION (limites maximos) ===");
  Serial.printf("  X: %ld pasos (%.1f mm)\n", LIMITE_PASOS_X, LIMITE_MM_X);
  Serial.printf("  Y: %ld pasos (%.1f mm)\n", LIMITE_PASOS_Y, LIMITE_MM_Y);
  Serial.printf("  Inicio de escaneo en X: %ld pasos (%.1f mm)\n",
                POS_X_INICIO_ESCANEO, POS_X_INICIO_ESCANEO_MM);
  Serial.printf("  Rango util sensor de distancia: %ld pasos (%.1f mm)\n",
                (long)(VL53_RANGO_MAX_MM * PASOS_POR_MM), VL53_RANGO_MAX_MM);

  Serial.println("=== ZONAS DE ALMACENAMIENTO ===");
  Serial.printf("  ROJO  -> X:%ld pasos (%.1f mm) | Y:%ld pasos (%.1f mm)\n",
                POS_X_ROJO, POS_X_ROJO_MM, POS_Y_ROJO, POS_Y_ROJO_MM);
  Serial.printf("  VERDE -> X:%ld pasos (%.1f mm) | Y:%ld pasos (%.1f mm)\n",
                POS_X_VERDE, POS_X_VERDE_MM, POS_Y_VERDE, POS_Y_VERDE_MM);
  Serial.printf("  AZUL  -> X:%ld pasos (%.1f mm) | Y:%ld pasos (%.1f mm)\n",
                POS_X_AZUL, POS_X_AZUL_MM, POS_Y_AZUL, POS_Y_AZUL_MM);

  Serial.printf("  Lectura de color -> Y:%ld pasos (%.1f mm)\n",
                POS_Y_LECTURA_COLOR, POS_Y_LECTURA_COLOR_MM);
}

// ==========================================
// 5. FUNCIONES DE LECTURA DE SENSORES
// ==========================================

/**
 * @brief Lee el sensor de distancia VL53L0X y calcula la posición en pasos
 * del eje Y a partir de la distancia real medida (en mm).
 * @param pasosYCalculados Retorna la conversión de distancia medida a pasos
 * @return true si hay un objeto dentro del rango útil (VL53_RANGO_MAX_MM) Y
 * dentro de la zona de operación (LIMITE_MM_Y). Si el objeto está más lejos
 * que la zona de operación, se ignora (no hace nada), aunque el sensor lo
 * haya detectado.
 */
bool leerSensorDistancia(long &pasosYCalculados) {
  if (!vl53Disponible) return false;

  VL53L0X_RangingMeasurementData_t medida;
  sensorDistancia.rangingTest(&medida, false);

  if (medida.RangeStatus == 4) return false; // 4 = fuera de rango / lectura inválida

  float distanciaMm = medida.RangeMilliMeter;
  if (distanciaMm > VL53_RANGO_MAX_MM || distanciaMm > LIMITE_MM_Y) return false; // fuera del area util o de la zona de operacion

  long pasosCalculados = (long)(distanciaMm * PASOS_POR_MM);
  pasosYCalculados = constrain(pasosCalculados, 0, LIMITE_PASOS_Y);
  return true;
}

/**
 * @brief Mide la frecuencia (Hz) de la señal OUT del TCS3200 sumando la
 * duración del semiciclo alto y bajo (equivale a un periodo completo entre
 * dos flancos de subida). Es el equivalente en Arduino/ESP32 (pulseIn) de
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

/**
 * @brief Lee el sensor TCS3200 (R, G, B) y determina el color predominante.
 * @return TipoColor (ROJO, VERDE o AZUL)
 */
TipoColor obtenerColorTCS3200() {
  // Configuración de filtros del TCS3200 (S2, S3):
  // Rojo: S2 = LOW, S3 = LOW
  // Verde: S2 = HIGH, S3 = HIGH
  // Azul: S2 = LOW, S3 = HIGH
  const uint32_t TIMEOUT_US = 20000; // 20ms

  // 1. Leer Componente Roja
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, LOW);
  uint32_t freqRojo = medirFrecuenciaTCS3200(TIMEOUT_US);

  // 2. Leer Componente Verde
  digitalWrite(TCS_S2, HIGH);
  digitalWrite(TCS_S3, HIGH);
  uint32_t freqVerde = medirFrecuenciaTCS3200(TIMEOUT_US);

  // 3. Leer Componente Azul
  digitalWrite(TCS_S2, LOW);
  digitalWrite(TCS_S3, HIGH);
  uint32_t freqAzul = medirFrecuenciaTCS3200(TIMEOUT_US);

  // Si no hubo pulso en ningún canal, el sensor no está viendo nada válido
  if (freqRojo == 0 && freqVerde == 0 && freqAzul == 0) {
    Serial.println("[COLOR] Sin lectura (timeout en los 3 canales).");
    return DESCONOCIDO;
  }

  uint8_t r = normalizarTCS3200(freqRojo,  TCS_RED_MIN,   TCS_RED_MAX);
  uint8_t g = normalizarTCS3200(freqVerde, TCS_GREEN_MIN, TCS_GREEN_MAX);
  uint8_t b = normalizarTCS3200(freqAzul,  TCS_BLUE_MIN,  TCS_BLUE_MAX);

  Serial.printf("[COLOR] R:%u G:%u B:%u (raw %lu,%lu,%lu Hz)\n",
                r, g, b,
                (unsigned long)freqRojo, (unsigned long)freqVerde, (unsigned long)freqAzul);

  // El color predominante es el canal normalizado con mayor valor
  // (a mayor frecuencia, mayor intensidad de reflexión en ese filtro).
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

#if MODO_OPERACION == 5
  Serial.println("=== MODO PRUEBA: MOTORES X/Y SIN FINALES DE CARRERA ===");
  Serial.println("Comandos: 'X<pasos>' o 'Y<pasos>' (ej: X200, Y-100; positivo=adelante, negativo=atras). 'R' resetea los drivers.");
  Serial.println("CUIDADO: sin finales de carrera no hay limites de software ni fisicos. Mueve pocos pasos primero.");
  pinMode(PIN_MOTOR_RESET, OUTPUT);
  digitalWrite(PIN_MOTOR_RESET, HIGH); // Fuera de reset: drivers habilitados a operar
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Y_DIR, OUTPUT);
  return; // No se inicializa nada más: este modo solo prueba los motores, sin finales de carrera.
#endif

#if MODO_OPERACION == 4
  Serial.println("=== MODO PRUEBA: MOTORES X/Y + FINALES DE CARRERA ===");
  Serial.printf("Limites de trabajo (soft limits): X:[0,%d] Y:[0,%d] mm\n", (int)LIMITE_MM_X, (int)LIMITE_MM_Y);
  pinMode(PIN_MOTOR_RESET, OUTPUT);
  digitalWrite(PIN_MOTOR_RESET, HIGH); // Fuera de reset: drivers habilitados a operar
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Y_DIR, OUTPUT);
  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);

  Serial.printf("[DEBUG] Estado inicial de finales de carrera (con debounce) -> LIMIT_X:%d LIMIT_Y:%d (1=presionado, 0=libre)\n",
                finalDeCarreraPresionado(PIN_LIMIT_X), finalDeCarreraPresionado(PIN_LIMIT_Y));

  Serial.println("Haciendo Homing inicial...");
  homingXY();

  Serial.println("Comandos: 'X<mm>' o 'Y<mm>' para mover relativo a la posicion actual (ej: X200, Y-50), 'H' para rehacer Homing, 'R' para resetear los drivers, 'Z' para ver zonas.");
  imprimirZonas();
  return; // No se inicializa nada más: este modo solo prueba motores + finales de carrera.
#endif

#if MODO_OPERACION == 3
  Serial.println("=== MODO PRUEBA: ELECTROIMAN (MOSFET) ===");
  Serial.println("Envia '1' para ENCENDER el electroiman, '0' para APAGARLO.");
  pinMode(PIN_MOSFET_MAG, OUTPUT);
  digitalWrite(PIN_MOSFET_MAG, LOW); // Electroimán apagado por seguridad al iniciar
  return; // No se inicializa nada más: este modo solo prueba el electroimán.
#endif

#if MODO_OPERACION == 2
  Serial.println("=== MODO PRUEBA: SERVOMOTOR MG90S ===");
  miServo.attach(PIN_SERVO);
  miServo.write(SERVO_ANGULO_ARRIBA); // Empezar con el cabezal arriba
  return; // No se inicializa nada más: este modo solo prueba el servo.
#endif

#if MODO_OPERACION == 1
  Serial.println("=== MODO PRUEBA: SENSOR DE COLOR TCS3200 + SENSOR DE DISTANCIA VL53L0X ===");
  // Este modo no usa los motores, pero sin esto los pines de STEP/RESET
  // quedan flotando (sin pinMode) mientras los sensores sí están activos,
  // y el ruido/acople de sus señales puede hacer que los drivers muevan
  // los motores solos. Se dejan en un estado seguro y fijo: drivers en
  // RESET (deshabilitados) y STEP en LOW firme.
  pinMode(PIN_MOTOR_RESET, OUTPUT);
  digitalWrite(PIN_MOTOR_RESET, LOW);
  pinMode(PIN_X_STEP, OUTPUT);
  digitalWrite(PIN_X_STEP, LOW);
  pinMode(PIN_Y_STEP, OUTPUT);
  digitalWrite(PIN_Y_STEP, LOW);
#else
  Serial.println("=== INICIALIZANDO CLASIFICADOR XY ===");

  // --- Configuración de Motores ---
  pinMode(PIN_MOTOR_RESET, OUTPUT);
  digitalWrite(PIN_MOTOR_RESET, HIGH); // Fuera de reset: drivers habilitados a operar
  pinMode(PIN_X_STEP, OUTPUT);
  pinMode(PIN_X_DIR, OUTPUT);
  pinMode(PIN_Y_STEP, OUTPUT);
  pinMode(PIN_Y_DIR, OUTPUT);

  // --- Configuración de Actuadores ---
  pinMode(PIN_MOSFET_MAG, OUTPUT);
  digitalWrite(PIN_MOSFET_MAG, LOW); // Electroimán apagado por seguridad

  miServo.attach(PIN_SERVO);
  miServo.write(SERVO_ANGULO_ARRIBA); // Empezar con el cabezal arriba

  // --- Configuración de Sensores ---
  pinMode(PIN_LIMIT_X, INPUT_PULLUP);
  pinMode(PIN_LIMIT_Y, INPUT_PULLUP);
#endif

  // --- Configuración del Sensor de Distancia VL53L0X ---
  Wire.begin(VL53_SDA, VL53_SCL);
  if (sensorDistancia.begin()) {
    vl53Disponible = true;
    Serial.println("VL53L0X detectado correctamente.");
  } else {
    vl53Disponible = false;
    Serial.println("ERROR: no se detectó el VL53L0X. Revisa el cableado (SDA/SCL/GND/VDD).");
  }

  // --- Configuración del Sensor de Color ---
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);

  // Escalar la frecuencia de salida del TCS3200 al 20% (Estándar recomendado)
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);
}

// ==========================================
// BUCLE PRINCIPAL
// ==========================================
#if MODO_OPERACION == 5
// ==========================================
// PRUEBA DE MOTORES X/Y SIN FINALES DE CARRERA (PASOS RELATIVOS)
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() < 2) return;

    if (linea.equalsIgnoreCase("R")) {
      resetControladoresMotor();
      return;
    }

    char eje = toupper(linea.charAt(0));
    long pasos = linea.substring(1).toInt();

    if (eje == 'X') {
      int dir = (pasos >= 0) ? DIR_X_ADELANTE : DIR_X_ATRAS;
      Serial.printf("[TEST] Moviendo X: %ld pasos (%s)\n", labs(pasos), (pasos >= 0) ? "adelante" : "atras");
      for (long i = 0; i < labs(pasos); i++) {
        darPaso(PIN_X_STEP, PIN_X_DIR, dir, DELAY_PASO_X_US);
      }
    } else if (eje == 'Y') {
      int dir = (pasos >= 0) ? DIR_Y_ADELANTE : DIR_Y_ATRAS;
      Serial.printf("[TEST] Moviendo Y: %ld pasos (%s)\n", labs(pasos), (pasos >= 0) ? "adelante" : "atras");
      for (long i = 0; i < labs(pasos); i++) {
        darPaso(PIN_Y_STEP, PIN_Y_DIR, dir, DELAY_PASO_Y_US);
      }
    } else {
      Serial.println("[ERROR] Formato invalido. Usa: X<pasos> o Y<pasos> (ej: X200, Y-100)");
    }
  }
}

#elif MODO_OPERACION == 4
// ==========================================
// PRUEBA DE MOTORES X/Y + FINALES DE CARRERA POR SERIAL
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() == 0) return;

    if (linea.equalsIgnoreCase("H")) {
      homingXY();
      return;
    }

    if (linea.equalsIgnoreCase("R")) {
      resetControladoresMotor();
      return;
    }

    if (linea.equalsIgnoreCase("Z")) {
      imprimirZonas();
      return;
    }

    char eje = toupper(linea.charAt(0));
    float mmDelta = linea.substring(1).toFloat();
    long pasosDelta = (long)(mmDelta * PASOS_POR_MM);

    if (eje == 'X') {
      moverACoordenadas(posicionActualX + pasosDelta, posicionActualY);
    } else if (eje == 'Y') {
      moverACoordenadas(posicionActualX, posicionActualY + pasosDelta);
    } else {
      Serial.println("[ERROR] Formato invalido. Usa: X<mm> o Y<mm> (ej: X200, Y-50; relativo a la posicion actual) o 'H' para Homing.");
    }
  }
}

#elif MODO_OPERACION == 3
// ==========================================
// PRUEBA DEL ELECTROIMAN (MOSFET) POR SERIAL
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    char comando = Serial.read();

    if (comando == '1') {
      digitalWrite(PIN_MOSFET_MAG, HIGH);
      Serial.println("[ELECTROIMAN] ENCENDIDO");
    } else if (comando == '0') {
      digitalWrite(PIN_MOSFET_MAG, LOW);
      Serial.println("[ELECTROIMAN] APAGADO");
    }
    // Cualquier otro carácter (ej. '\n', '\r') se ignora.
  }
}

#elif MODO_OPERACION == 2
// ==========================================
// PRUEBA DEL SERVOMOTOR MG90S
// ==========================================
void loop() {
  Serial.println("[SERVO] -> ARRIBA (10 grados)");
  miServo.write(SERVO_ANGULO_ARRIBA);
  delay(1000);

  Serial.println("[SERVO] -> ABAJO (90 grados)");
  miServo.write(SERVO_ANGULO_ABAJO);
  delay(1000);
}

#elif MODO_OPERACION == 1
void loop() {
  // Color: obtenerColorTCS3200() ya imprime R/G/B y frecuencias crudas.
  TipoColor color = obtenerColorTCS3200();
  const char *nombreColor = (color == ROJO) ? "ROJO"
                          : (color == VERDE) ? "VERDE"
                          : (color == AZUL) ? "AZUL"
                          : "SIN LECTURA";

  // Distancia: se imprime el valor crudo en mm, sin filtrar por rango,
  // para poder verificar el sensor a cualquier distancia.
  if (vl53Disponible) {
    VL53L0X_RangingMeasurementData_t medida;
    sensorDistancia.rangingTest(&medida, false);
    if (medida.RangeStatus != 4) {
      Serial.printf("[PRUEBA] Color: %s | Distancia: %u mm\n", nombreColor, medida.RangeMilliMeter);
    } else {
      Serial.printf("[PRUEBA] Color: %s | Distancia: fuera de rango\n", nombreColor);
    }
  } else {
    Serial.printf("[PRUEBA] Color: %s | Distancia: sensor VL53L0X no disponible\n", nombreColor);
  }

  delay(2000);
}

#else
// ==========================================
// MÁQUINA DE ESTADOS DEL CLASIFICADOR XY
// ==========================================
void loop() {
  switch (estadoActual) {

    // -------------------------------------------------------------
    // ESTADO 1: HOMING (Retorno a origen de los ejes X e Y)
    // -------------------------------------------------------------
    case ESTADO_HOMING:
      Serial.println("[MAQUINA] Estado: HOMING Ejes X e Y...");

      // Aseguramos que el electroimán esté apagado y el servo arriba antes del home
      digitalWrite(PIN_MOSFET_MAG, LOW);
      miServo.write(SERVO_ANGULO_ARRIBA);
      delay(500);

      homingXY();
      delay(500);
      estadoActual = ESTADO_ESCANEO;
      break;

    // -------------------------------------------------------------
    // ESTADO 2: ESCANEO (Avanzar en X leyendo el sensor de distancia)
    // -------------------------------------------------------------
    case ESTADO_ESCANEO:
      Serial.println("[MAQUINA] Estado: ESCANEO...");

      // La zona de escaneo empieza en POS_X_INICIO_ESCANEO (50mm), no en
      // X=0: esa franja inicial está reservada para las zonas de almacenamiento.
      if (posicionActualX < POS_X_INICIO_ESCANEO) {
        moverACoordenadas(POS_X_INICIO_ESCANEO, posicionActualY);
      }

      while (posicionActualX < LIMITE_PASOS_X) {
        // Dar varios pasos seguidos en X antes de volver a leer el sensor:
        // el VL53L0X tarda ~30-50ms por medición, así que leerlo en CADA
        // paso individual frena el escaneo a paso de tortuga. Se agrupan
        // PASOS_ENTRE_LECTURAS_ESCANEO pasos (rápido) entre cada lectura.
        for (int i = 0; i < PASOS_ENTRE_LECTURAS_ESCANEO && posicionActualX < LIMITE_PASOS_X; i++) {
          darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ADELANTE, 1500);
          posicionActualX++;
        }

        // Leer sensor de distancia VL53L0X para estimar posición Y
        long yCalculado = 0;
        if (leerSensorDistancia(yCalculado)) {
          objetoDetectadoX = posicionActualX;
          objetoDetectadoY = yCalculado;
          Serial.printf("[DETECCION] Objeto detectado en X: %ld pasos, Y estimado: %ld pasos!\n",
                        objetoDetectadoX, objetoDetectadoY);

          estadoActual = ESTADO_GOTO_OBJETO;
          break;
        }
      }

      // Si terminamos de recorrer X sin detectar nada, hacemos Home y volvemos a escanear
      if (estadoActual == ESTADO_ESCANEO) {
        Serial.println("[MAQUINA] Límite de escaneo X alcanzado sin detecciones. Recomenzando...");
        estadoActual = ESTADO_HOMING;
      }
      break;

    // -------------------------------------------------------------
    // ESTADO 3: GOTO OBJETO (Moverse a la posición del objeto)
    // -------------------------------------------------------------
    case ESTADO_GOTO_OBJETO:
      Serial.println("[MAQUINA] Estado: IR A LA POSICION DEL OBJETO...");
      // Nos movemos al punto exacto donde se detectó el objeto
      moverACoordenadas(objetoDetectadoX, objetoDetectadoY);
      delay(500);
      estadoActual = ESTADO_AGARRE;
      break;

    // -------------------------------------------------------------
    // ESTADO 4: AGARRE (Bajar servo, activar imán y levantar)
    // -------------------------------------------------------------
    case ESTADO_AGARRE:
      Serial.println("[MAQUINA] Estado: AGARRE Y ACOPLE...");

      // 1. Baja el servomotor para acercar el electroimán al objeto
      miServo.write(SERVO_ANGULO_ABAJO);
      delay(1000); // Esperar a que el servo llegue abajo

      // 2. Activa el Electroimán
      digitalWrite(PIN_MOSFET_MAG, HIGH);
      Serial.println("[ACTUADOR] Electroimán ENCENDIDO.");
      delay(1000); // Dar un segundo para asegurar el acople magnético

      // 3. Levanta el servomotor con el objeto agarrado
      miServo.write(SERVO_ANGULO_ARRIBA);
      delay(1000); // Esperar a que el servo levante el objeto

      // 4. El sensor TCS3200 está anclado cerca del home de Y (no viaja con
      // el cabezal), así que hay que acercar el objeto hasta ahí para leerlo.
      moverACoordenadas(posicionActualX, POS_Y_LECTURA_COLOR);
      delay(300);

      estadoActual = ESTADO_LORE_COLOR;
      break;

    // -------------------------------------------------------------
    // ESTADO 5: LECTURA DE COLOR (Escanear espectro de color)
    // -------------------------------------------------------------
    case ESTADO_LORE_COLOR:
      Serial.println("[MAQUINA] Estado: LECTURA DE COLOR...");
      colorDetectado = obtenerColorTCS3200();

      switch (colorDetectado) {
        case ROJO:
          Serial.println("[COLOR] Detectado color: ROJO.");
          break;
        case VERDE:
          Serial.println("[COLOR] Detectado color: VERDE.");
          break;
        case AZUL:
          Serial.println("[COLOR] Detectado color: AZUL.");
          break;
        default:
          Serial.println("[COLOR] Error: Color no detectado o desconocido. Por defecto: ROJO.");
          colorDetectado = ROJO; // Acción segura en caso de error
          break;
      }
      estadoActual = ESTADO_DEPOSITAR;
      break;

    // -------------------------------------------------------------
    // ESTADO 6: DEPOSITAR (Moverse a la caja y soltar)
    // -------------------------------------------------------------
    case ESTADO_DEPOSITAR:
      Serial.println("[MAQUINA] Estado: CLASIFICAR Y DEPOSITAR...");

      // Seleccionar coordenadas de caja de destino según color
      long destinoX, destinoY;
      if (colorDetectado == ROJO) {
        destinoX = POS_X_ROJO;
        destinoY = POS_Y_ROJO;
      } else if (colorDetectado == VERDE) {
        destinoX = POS_X_VERDE;
        destinoY = POS_Y_VERDE;
      } else { // AZUL
        destinoX = POS_X_AZUL;
        destinoY = POS_Y_AZUL;
      }

      // Mover los ejes al contenedor correspondiente
      moverACoordenadas(destinoX, destinoY);
      delay(500);

      // Opcional: Bajar un poco el servo para no tirar el objeto desde muy alto
      miServo.write(SERVO_ANGULO_ABAJO);
      delay(800);

      // Desactivar el Electroimán para soltar el objeto
      digitalWrite(PIN_MOSFET_MAG, LOW);
      Serial.println("[ACTUADOR] Electroimán APAGADO. Objeto liberado.");
      delay(1000); // Esperar a que caiga

      // Volver a subir el cabezal vacío
      miServo.write(SERVO_ANGULO_ARRIBA);
      delay(500);

      estadoActual = ESTADO_RETORNO;
      break;

    // -------------------------------------------------------------
    // ESTADO 7: RETORNO (Prepararse para el siguiente ciclo)
    // -------------------------------------------------------------
    case ESTADO_RETORNO:
      Serial.println("[MAQUINA] Estado: REGRESANDO A ORIGEN...");
      // Volver al punto inicial (Homing) para recalibrar eje X y continuar escaneando
      estadoActual = ESTADO_HOMING;
      delay(500);
      break;
  }
}
#endif // MODO_OPERACION
