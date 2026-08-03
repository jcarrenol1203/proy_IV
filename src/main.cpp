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

// WiFi/MQTT (control remoto + app web) solo se compilan/inicializan en
// MODO_OPERACION 0. Los modos de prueba 1-5 quedan exactamente igual que
// antes, sin este stack cargado.
#if MODO_OPERACION == 0
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#endif

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
const int SERVO_ANGULO_ABAJO  = 8;   // Cabezal abajo (agarre)
const int SERVO_ANGULO_LECTURA_COLOR = 120;  // Altura a la que se tomaron las firmas de color (FIRMA_ROJO/VERDE/AZUL); si se cambia, hay que recalibrar

// --- Rampa del servo al levantar con el objeto agarrado ---
// Subir de golpe (write directo) puede lanzar/botar el objeto por la
// aceleración repentina del cabezal. Moverlo en pasos pequeños con una
// pausa entre cada uno sube el objeto de forma suave.
const int SERVO_PASO_RAMPA_GRADOS = 5;
const int SERVO_DELAY_PASO_RAMPA_MS = 25;

// --- Paso de ajuste fino del servo en MODO_OPERACION 2 (grados por '+'/'-') ---
const int SERVO_PASO_AJUSTE_GRADOS = 2;

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
const int DELAY_HOMING_X_US      = 1750;  // Homing eje X (más lento = toque más suave contra el final de carrera)
const int DELAY_HOMING_Y_US      = 1750;  // Homing eje Y (más lento = toque más suave contra el final de carrera)

// --- Límites Físicos del Área de Trabajo (en milímetros) ---
// Medidos por prueba física: tope mecánico de X en 15200 pasos, de Y en
// 7200 pasos (con PASOS_POR_MM=40 -> 380mm y 180mm).
const float LIMITE_MM_X = 380.0;
const float LIMITE_MM_Y = 180.0;

// --- Zona de operación (escaneo) en X ---
// Ya no hay franja de X reservada para almacenamiento (las cajas se movieron
// a la franja de Y, ver más abajo), así que el escaneo cubre todo el eje X
// desde el origen.
const float POS_X_INICIO_ESCANEO_MM = 0.0;

// --- Posición Física de las Cajas de Clasificación (en milímetros) ---
// Ancladas en la franja de Y reservada [150, 180]mm (el final del eje Y,
// fuera del rango que el escaneo interpreta como "objeto", ver
// VL53_RANGO_MAX_MM más abajo), diferenciadas por X en 3 zonas iguales
// dentro de LIMITE_MM_X (380mm / 3 ≈ 126.7mm por zona, tomando el punto
// medio de cada una). Ajustar POS_Y_*_MM si el almacenamiento no queda a esa
// Y exacta dentro de la franja.
const float POS_X_ROJO_MM  = 63.3;   const float POS_Y_ROJO_MM  = 165.0;  // Zona 1: 0-126.7mm
const float POS_X_VERDE_MM = 190.0;  const float POS_Y_VERDE_MM = 165.0;  // Zona 2: 126.7-253.3mm
const float POS_X_AZUL_MM  = 316.7;  const float POS_Y_AZUL_MM  = 165.0;  // Zona 3: 253.3-380mm

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
// Limitado a 150mm (no a LIMITE_MM_Y=180mm) a propósito: las cajas de
// almacenamiento ahora están físicamente ancladas en la franja de Y
// [150,180]mm (ver POS_Y_*_MM), y quedan dentro del corredor que recorre el
// cabezal. Si el escaneo llegara a interpretar esa franja como zona de
// objetos, confundiría las cajas mismas con un objeto a recoger.
const float VL53_RANGO_MAX_MM = 150.0;

// --- Offset de calibración del VL53L0X (en mm) ---
// Offset TOTAL, calibrado empíricamente moviendo el cabezal al valor
// calculado y comparando contra dónde está realmente el objeto (no una
// separación física medida con regla): incluye tanto cualquier sesgo propio
// del sensor como la separación entre el lente y el punto de agarre. El
// motor Y mueve el agarre, no el sensor, así que su objetivo real es la
// distancia cruda MENOS este offset.
//
// CÓMO RECALIBRAR:
//   1. Sube el firmware, coloca un objeto y deja que el sistema lo detecte
//      y se mueva a la posición calculada (ESTADO_GOTO_OBJETO).
//   2. Mide con regla/cinta métrica dónde está REALMENTE el objeto respecto
//      al home de Y, y compara contra el "Y estimado" que imprime
//      [DETECCION] (que ya incluye el offset actual).
//   3. Si el cabezal se pasó de largo por Nmm, hay que restar N mm más al
//      offset (más negativo). Si se quedó corto por Nmm, sumar N mm.
//   4. Repite hasta que el "Y estimado" impreso coincida con la posición
//      real medida del objeto.
//
// Calibrado: con lectura cruda de 145mm, el offset de -30 dejaba el target
// en 115mm, pero el objeto real estaba en 85mm (30mm antes) -> offset total
// correcto: -60mm (145 - 60 = 85).
const float VL53_OFFSET_MM = -60.0;

// --- Radio de la tapa/objeto a recoger (en mm) ---
// Las tapas son circulares: el escaneo encuentra el centro real en X y el
// borde más cercano en Y buscando el punto de distancia MÍNIMA mientras el
// carro recorre el objeto (ver ESTADO_ESCANEO), y a esa distancia mínima se
// le suma este radio para llegar al centro real en Y.
const float RADIO_TAPA_MM = 15.0;

// --- Confirmación de mínimo local durante el escaneo de una tapa ---
// Cuántas lecturas SEGUIDAS que no mejoran (no bajan) el mínimo actual hacen
// falta para asumir que ya pasamos el punto más cercano de la tapa actual y
// cortar el escaneo ahí (en vez de seguir de largo hacia una posible tapa
// siguiente pegada). Muy bajo = puede cortar antes de tiempo por ruido de un
// solo dato; muy alto = puede alcanzar a mezclarse con la siguiente tapa.
const int UMBRAL_CONFIRMAR_MINIMO_ESCANEO = 3;

// --- Tolerancia a lecturas fallidas seguidas durante el escaneo de una tapa ---
// Mientras se busca el mínimo, no toda lectura sale válida (ej. tapas
// azules/verdes reflejan peor el IR y dan más RangeStatus distinto de 0, ver
// leerSensorDistancia). Se toleran hasta esta cantidad de lecturas fallidas
// SEGUIDAS (sin ninguna detección) antes de asumir que la tapa ya se acabó
// por quedar fuera de rango, no por ruido puntual.
const int TOLERANCIA_PERDIDAS_ESCANEO = 4;

// --- Pasos de X entre cada lectura del VL53L0X durante el escaneo ---
// El VL53L0X tarda ~30-50ms por medición; leerlo en cada paso individual
// frena el escaneo. Con 40 pasos/mm, 40 pasos = 1mm de resolución espacial
// del escaneo entre lecturas.
const int PASOS_ENTRE_LECTURAS_ESCANEO = 40;

// --- Calibración del Sensor de Color TCS3200 (por vecino más cercano) ---
// El método clásico (normalizar cada canal contra un blanco/negro de
// referencia) asume que el blanco refleja limpiamente en las 3 bandas.
// En este montaje eso no se cumplió: una muestra blanca dio MENOS señal
// cruda que las tapas de color (probablemente reflexión especular de la
// superficie blanca vs. difusa de las tapas de plástico mate), así que esa
// calibración no discriminaba bien entre verde/azul/rojo.
//
// En vez de eso, se guarda la "huella" (R,G,B crudos) medida directamente
// de cada tapa real, y se clasifica por la firma más cercana (distancia
// euclidiana al cuadrado) a la lectura actual. No requiere blanco/negro de
// referencia, solo que cada tapa dé una lectura repetible parecida a sí
// misma.
//
// CÓMO RECALIBRAR: sube el firmware en MODO_OPERACION=2, envía "130" para
// llevar el servo a SERVO_ANGULO_LECTURA_COLOR, pon cada tapa a la misma
// distancia del sensor (con el electroimán encendido, como en el agarre
// real) y anota la línea "[COLOR] ... (raw R,G,B Hz)" para cada una.
struct FirmaColorTCS3200 { uint32_t r, g, b; };
const FirmaColorTCS3200 FIRMA_ROJO  = {5319, 3546, 3717};
const FirmaColorTCS3200 FIRMA_VERDE = {4629, 3225, 3355};
const FirmaColorTCS3200 FIRMA_AZUL  = {4444, 3144, 3236};

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
  ESTADO_RETORNO,
#if MODO_OPERACION == 0

  // Estado de pausa: solo se entra aquí si llega un comando MQTT "STOP".
  // El ciclo automático original (HOMING->ESCANEO->...->RETORNO->ESCANEO)
  // nunca lo visita por sí solo, así que sin WiFi/MQTT el comportamiento
  // es idéntico al de siempre.
  ESTADO_MANUAL,
#endif
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

#if MODO_OPERACION == 0
// ==========================================
// 3.1 WIFI + MQTT (control remoto / app web, opcional)
// ==========================================
// Reemplaza con el SSID/clave de tu red WiFi local (2.4 GHz). Si se dejan
// los valores de ejemplo, conectarWiFi() simplemente fallará tras ~10s y el
// sistema sigue funcionando 100% autónomo por Serial, igual que siempre.
const char* WIFI_SSID = "Comunidad_UNMED";
const char* WIFI_PASS = "wifi_med_213";

// Broker MQTT público de ejemplo (broker.emqx.io, broker.hivemq.com,
// test.mosquitto.org). Para uso real, monta tu propio broker.
const char* MQTT_BROKER          = "broker.emqx.io";
const int   MQTT_PORT            = 1883;
const char* MQTT_TOPIC_CONTROL   = "clasificador/control";
const char* MQTT_TOPIC_TELEMETRY = "clasificador/telemetry";
const char* MQTT_TOPIC_STATUS    = "clasificador/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long ultimoEnvioTelemetria = 0;
const unsigned long INTERVALO_TELEMETRIA_MS = 300;

// Espejos de estado solo para telemetría (la lógica real de agarre/color
// sigue gobernada por digitalWrite(PIN_MOSFET_MAG,...) / miServo.write(...)
// exactamente como antes; estas variables solo reflejan esas llamadas para
// poder publicarlas por MQTT).
bool electroimanEstadoMQTT = false;
bool servoAbajoMQTT = false;
int contadorRojo = 0;
int contadorVerde = 0;
int contadorAzul = 0;
uint16_t ultimaDistanciaMmMQTT = 0;

// --- Ajuste fino de Y, calibrable en caliente por MQTT (sin recompilar) ---
// Se suma (en mm) a la posición de agarre calculada por el escaneo (que ya
// incluye VL53_OFFSET_MM y RADIO_TAPA_MM), justo antes de mover el cabezal
// en ESTADO_GOTO_OBJETO. Empieza en 0 (sin corrección extra); la app web lo
// ajusta con el comando "SET_Y_OFFSET" para afinar el agarre sin flashear.
float ajusteFinoYMm = 0.0;

// --- Caja destino manual (override del color detectado) ---
// DESCONOCIDO = automático (usa colorDetectado, el color leído por el
// TCS3200, como siempre). Si la app web fija ROJO/VERDE/AZUL con el comando
// "SET_DESTINO", ESTADO_DEPOSITAR ignora el color detectado y lleva el
// objeto a la caja elegida manualmente, hasta que se vuelva a poner "AUTO".
TipoColor colorDestinoManual = DESCONOCIDO;

// Declarada aquí (definida más abajo) para poder llamarla desde
// moverACoordenadas()/homingXY(), que aparecen antes en el archivo.
void publicarTelemetria();
#endif

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

#if MODO_OPERACION == 0
  // Igual que en homingXY(): un solo movimiento puede recorrer hasta 380mm
  // (varios segundos bloqueado), así que hay que seguir atendiendo MQTT o
  // el broker corta la conexión por keepalive y la app web se ve "offline".
  // También se publica telemetría periódicamente (no solo mqttClient.loop())
  // para que la posición en la app web se vea moverse en vivo, en vez de
  // quedar congelada hasta que termine el movimiento.
  unsigned long ultimoServicioMQTT = 0;
  unsigned long ultimaTelemetriaMovimiento = 0;
#endif

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
#if MODO_OPERACION == 0
    if (millis() - ultimoServicioMQTT > 50) {
      ultimoServicioMQTT = millis();
      if (WiFi.status() == WL_CONNECTED) mqttClient.loop();
    }
    if (millis() - ultimaTelemetriaMovimiento > INTERVALO_TELEMETRIA_MS) {
      ultimaTelemetriaMovimiento = millis();
      publicarTelemetria();
    }
#endif
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
#if MODO_OPERACION == 0
    if (millis() - ultimoServicioMQTT > 50) {
      ultimoServicioMQTT = millis();
      if (WiFi.status() == WL_CONNECTED) mqttClient.loop();
    }
    if (millis() - ultimaTelemetriaMovimiento > INTERVALO_TELEMETRIA_MS) {
      ultimaTelemetriaMovimiento = millis();
      publicarTelemetria();
    }
#endif
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
#if MODO_OPERACION == 0
  // homingXY() puede tardar varios segundos en recorrer todo el eje (peor
  // caso: desde el extremo opuesto al home). Como es bloqueante, sin esto
  // mqttClient.loop() nunca se llama mientras dura, el broker no recibe
  // PING y termina cortando la conexión por keepalive -> la app web ve el
  // ESP32 "offline" aunque en realidad esté funcionando bien. También se
  // publica telemetría periódicamente para que el estado "HOMING" se vea
  // reflejado en la app en vivo, no solo al terminar.
  unsigned long ultimoServicioMQTT = 0;
  unsigned long ultimaTelemetriaMovimiento = 0;
#endif

  Serial.println("[HOMING] Eje Y...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_Y)) {
    darPaso(PIN_Y_STEP, PIN_Y_DIR, DIR_Y_ATRAS, DELAY_HOMING_Y_US);
#if MODO_OPERACION == 0
    if (millis() - ultimoServicioMQTT > 50) {
      ultimoServicioMQTT = millis();
      if (WiFi.status() == WL_CONNECTED) mqttClient.loop();
    }
    if (millis() - ultimaTelemetriaMovimiento > INTERVALO_TELEMETRIA_MS) {
      ultimaTelemetriaMovimiento = millis();
      publicarTelemetria();
    }
#endif
  }
  posicionActualY = 0; // Origen Y calibrado

  Serial.println("[HOMING] Eje X...");
  while (!finalDeCarreraPresionado(PIN_LIMIT_X)) {
    darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ATRAS, DELAY_HOMING_X_US);
#if MODO_OPERACION == 0
    if (millis() - ultimoServicioMQTT > 50) {
      ultimoServicioMQTT = millis();
      if (WiFi.status() == WL_CONNECTED) mqttClient.loop();
    }
    if (millis() - ultimaTelemetriaMovimiento > INTERVALO_TELEMETRIA_MS) {
      ultimaTelemetriaMovimiento = millis();
      publicarTelemetria();
    }
#endif
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

/**
 * @brief Mueve el servo en pasos pequeños (SERVO_PASO_RAMPA_GRADOS) con una
 * pausa entre cada uno, en vez de saltar de golpe al ángulo destino. Se usa
 * al levantar con el objeto ya agarrado: el frenazo/arranque brusco de un
 * write() directo puede lanzarlo fuera del electroimán.
 */
void moverServoGradual(int anguloDestino) {
  int anguloActual = miServo.read(); // último ángulo escrito (sin sensor de posición real)
  int paso = (anguloDestino >= anguloActual) ? SERVO_PASO_RAMPA_GRADOS : -SERVO_PASO_RAMPA_GRADOS;

  for (int a = anguloActual; (paso > 0) ? (a < anguloDestino) : (a > anguloDestino); a += paso) {
    miServo.write(a);
    delay(SERVO_DELAY_PASO_RAMPA_MS);
  }
  miServo.write(anguloDestino);
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

  // RangeStatus: 0=Range Valid, 1=Sigma Fail, 2=Signal Fail, 3=Min Range
  // Fail, 4=Phase Fail, 5=Hardware Fail. Solo 0 es una medida confiable: los
  // demás (sobre todo 1/2, señal débil o ruidosa) son justo lo que se ve
  // cuando NO hay nada al frente, y aceptarlos como válidos es lo que
  // causaba lecturas "fantasma" cambiantes sin objeto presente.
  if (medida.RangeStatus != 0) return false;

  float distanciaMm = medida.RangeMilliMeter;
  Serial.printf("[VL53 RAW] %.0f mm (RangeStatus:%d)\n", distanciaMm, medida.RangeStatus);
#if MODO_OPERACION == 0
  ultimaDistanciaMmMQTT = (uint16_t)distanciaMm;
#endif

  // Traduce la distancia medida (desde el lente, ya real, sin sesgo propio)
  // a la posición que debe alcanzar el AGARRE, restando la separación física
  // fija entre el lente y el agarre (ver VL53_OFFSET_MM).
  float distanciaObjetivoGarraMm = distanciaMm + VL53_OFFSET_MM;
  Serial.printf("[VL53 OBJETIVO GARRA] %.0f mm (offset:%.1f mm)\n", distanciaObjetivoGarraMm, VL53_OFFSET_MM);

  if (distanciaObjetivoGarraMm > VL53_RANGO_MAX_MM || distanciaObjetivoGarraMm > LIMITE_MM_Y) return false; // fuera del area util o de la zona de operacion

  long pasosCalculados = (long)(distanciaObjetivoGarraMm * PASOS_POR_MM);
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
 * @brief Distancia euclidiana al cuadrado entre una lectura cruda (R,G,B) y
 * una firma de color calibrada. Se usa al cuadrado (sin raíz) porque solo
 * importa cuál firma da la distancia menor, no su valor exacto.
 */
long distanciaCuadradaColor(uint32_t r, uint32_t g, uint32_t b, const FirmaColorTCS3200 &firma) {
  long dr = (long)r - (long)firma.r;
  long dg = (long)g - (long)firma.g;
  long db = (long)b - (long)firma.b;
  return dr * dr + dg * dg + db * db;
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

  long dRojo  = distanciaCuadradaColor(freqRojo, freqVerde, freqAzul, FIRMA_ROJO);
  long dVerde = distanciaCuadradaColor(freqRojo, freqVerde, freqAzul, FIRMA_VERDE);
  long dAzul  = distanciaCuadradaColor(freqRojo, freqVerde, freqAzul, FIRMA_AZUL);

  Serial.printf("[COLOR] raw R,G,B: %lu,%lu,%lu Hz | dist2 ROJO:%ld VERDE:%ld AZUL:%ld\n",
                (unsigned long)freqRojo, (unsigned long)freqVerde, (unsigned long)freqAzul,
                dRojo, dVerde, dAzul);

  // El color es el de la firma más cercana (menor distancia) a la lectura actual.
  if (dRojo <= dVerde && dRojo <= dAzul) return ROJO;
  if (dVerde <= dRojo && dVerde <= dAzul) return VERDE;
  return AZUL;
}

#if MODO_OPERACION == 0
// ==========================================
// 6. WIFI + MQTT (control remoto / app web, opcional)
// ==========================================
const char* getEstadoNombre(EstadoSistema e) {
  switch (e) {
    case ESTADO_HOMING: return "HOMING";
    case ESTADO_ESCANEO: return "ESCANEO";
    case ESTADO_GOTO_OBJETO: return "GOTO_OBJETO";
    case ESTADO_AGARRE: return "AGARRE";
    case ESTADO_LORE_COLOR: return "LECTURA_COLOR";
    case ESTADO_DEPOSITAR: return "DEPOSITAR";
    case ESTADO_RETORNO: return "RETORNO";
    case ESTADO_MANUAL: return "MANUAL";
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

/**
 * @brief Publica el estado actual del sistema por MQTT (topic de
 * telemetría). Es puramente informativo: no cambia nada de la máquina de
 * estados, solo reporta lo que ya está pasando.
 */
void publicarTelemetria() {
  if (!mqttClient.connected()) return;

  JsonDocument doc;
  doc["state"] = getEstadoNombre(estadoActual);
  doc["x_mm"] = posicionActualX / PASOS_POR_MM;
  doc["y_mm"] = posicionActualY / PASOS_POR_MM;
  doc["magnet"] = electroimanEstadoMQTT;
  doc["servo"] = servoAbajoMQTT ? "ABAJO" : "ARRIBA";
  doc["dist_mm"] = ultimaDistanciaMmMQTT;
  doc["y_fine_offset_mm"] = ajusteFinoYMm;
  doc["color"] = getColorNombre(colorDetectado);
  doc["destino_manual"] = (colorDestinoManual == DESCONOCIDO) ? "AUTO" : getColorNombre(colorDestinoManual);
  doc["counts"]["red"] = contadorRojo;
  doc["counts"]["green"] = contadorVerde;
  doc["counts"]["blue"] = contadorAzul;

  char buffer[384];
  serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC_TELEMETRY, buffer);
}

/**
 * @brief Comandos remotos aceptados por MQTT (topic de control). Solo
 * START_SCAN/STOP/HOMING tocan la máquina de estados; MAGNET/SERVO/MOVE son
 * overrides manuales pensados para usarse con el sistema detenido
 * (ESTADO_MANUAL), no durante el ciclo automático.
 */
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.println("[MQTT] Error al parsear JSON de comando.");
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  Serial.printf("[MQTT RX] Comando: %s\n", cmd);

  if (strcmp(cmd, "START_SCAN") == 0) {
    estadoActual = ESTADO_ESCANEO;
  } else if (strcmp(cmd, "STOP") == 0) {
    estadoActual = ESTADO_MANUAL;
  } else if (strcmp(cmd, "HOMING") == 0) {
    estadoActual = ESTADO_HOMING;
  } else if (strcmp(cmd, "MAGNET") == 0) {
    bool state = doc["state"];
    electroimanEstadoMQTT = state;
    digitalWrite(PIN_MOSFET_MAG, state ? HIGH : LOW);
    Serial.printf("[ELECTROIMAN] Control manual (MQTT): %s\n", state ? "ENCENDIDO" : "APAGADO");
  } else if (strcmp(cmd, "SERVO") == 0) {
    const char* sState = doc["state"];
    if (sState && (strcmp(sState, "DOWN") == 0 || strcmp(sState, "ABAJO") == 0)) {
      miServo.write(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
    } else {
      miServo.write(SERVO_ANGULO_ARRIBA);
      servoAbajoMQTT = false;
    }
  } else if (strcmp(cmd, "MOVE") == 0 && doc["x"].is<float>() && doc["y"].is<float>()) {
    float xMm = doc["x"];
    float yMm = doc["y"];
    moverACoordenadas((long)(xMm * PASOS_POR_MM), (long)(yMm * PASOS_POR_MM));
  } else if (strcmp(cmd, "MOVE_REL") == 0) {
    float dx = doc["dx"] | 0.0;
    float dy = doc["dy"] | 0.0;
    moverACoordenadas(posicionActualX + (long)(dx * PASOS_POR_MM),
                       posicionActualY + (long)(dy * PASOS_POR_MM));
  } else if (strcmp(cmd, "RESET_DRIVERS") == 0) {
    resetControladoresMotor();
  } else if (strcmp(cmd, "SET_Y_OFFSET") == 0 && doc["value"].is<float>()) {
    ajusteFinoYMm = doc["value"];
    Serial.printf("[CALIBRACION] Ajuste fino de Y (MQTT): %.1f mm\n", ajusteFinoYMm);
  } else if (strcmp(cmd, "RETRY_GRAB") == 0) {
    // Reintenta el agarre en la ÚLTIMA posición detectada (objetoDetectadoX/Y
    // no cambian hasta el próximo escaneo), aplicando el ajuste fino de Y
    // actual. Pensado para calibrar en vivo: ajustar SET_Y_OFFSET y volver a
    // intentar sin tener que re-escanear desde cero.
    Serial.println("[MQTT] Reintentando agarre en la última posición detectada...");
    estadoActual = ESTADO_GOTO_OBJETO;
  } else if (strcmp(cmd, "SET_DESTINO") == 0) {
    const char* colorStr = doc["color"];
    if (!colorStr || strcmp(colorStr, "AUTO") == 0) {
      colorDestinoManual = DESCONOCIDO;
      Serial.println("[CALIBRACION] Caja destino: AUTO (color detectado por sensor)");
    } else if (strcmp(colorStr, "ROJO") == 0) {
      colorDestinoManual = ROJO;
      Serial.println("[CALIBRACION] Caja destino forzada (MQTT): ROJO");
    } else if (strcmp(colorStr, "VERDE") == 0) {
      colorDestinoManual = VERDE;
      Serial.println("[CALIBRACION] Caja destino forzada (MQTT): VERDE");
    } else if (strcmp(colorStr, "AZUL") == 0) {
      colorDestinoManual = AZUL;
      Serial.println("[CALIBRACION] Caja destino forzada (MQTT): AZUL");
    }
  }

  publicarTelemetria();
}

void reconectarMQTT() {
  if (mqttClient.connected()) return;
  Serial.print("[MQTT] Conectando a broker...");
  String clientId = "ESP32Clasificador-" + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println(" conectado.");
    mqttClient.subscribe(MQTT_TOPIC_CONTROL);
    mqttClient.publish(MQTT_TOPIC_STATUS, "{\"online\":true}");
  } else {
    Serial.printf(" falló (estado %d), se reintenta en el próximo ciclo.\n", mqttClient.state());
  }
}

/**
 * @brief Intenta conectar a la red WiFi configurada arriba (WIFI_SSID /
 * WIFI_PASS). Si falla (ej. quedaron los valores de ejemplo), se rinde tras
 * ~10s y el sistema sigue funcionando en modo local, igual que si esta
 * función no existiera.
 */
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
    Serial.printf("\n[WIFI] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] No se pudo conectar. Operando en modo local (sin app/MQTT).");
  }
}
#endif // MODO_OPERACION == 0

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
  Serial.println("=== MODO PRUEBA: SERVOMOTOR MG90S (AJUSTE INTERACTIVO) + LECTURA DE COLOR ===");
  Serial.printf("Comandos: '+' sube %d grados, '-' baja %d grados, un numero (0-180) va directo a ese angulo, 'A' = ARRIBA (%d), 'B' = ABAJO (%d), '1' enciende el electroiman, '0' lo apaga.\n",
                SERVO_PASO_AJUSTE_GRADOS, SERVO_PASO_AJUSTE_GRADOS, SERVO_ANGULO_ARRIBA, SERVO_ANGULO_ABAJO);
  Serial.println("El color leido por el TCS3200 se imprime solo cada cierto tiempo, para calibrar a que altura del servo lee bien.");
  miServo.attach(PIN_SERVO);
  miServo.write(SERVO_ANGULO_ARRIBA); // Empezar con el cabezal arriba
  Serial.printf("[SERVO] -> %d grados\n", SERVO_ANGULO_ARRIBA);

  // --- Configuración del Sensor de Color (para calibrar altura junto al servo) ---
  pinMode(TCS_S0, OUTPUT);
  pinMode(TCS_S1, OUTPUT);
  pinMode(TCS_S2, OUTPUT);
  pinMode(TCS_S3, OUTPUT);
  pinMode(TCS_OUT, INPUT);
  // Escalar la frecuencia de salida del TCS3200 al 20% (Estándar recomendado)
  digitalWrite(TCS_S0, HIGH);
  digitalWrite(TCS_S1, LOW);

  // --- Electroimán activado para la prueba (simula tener el objeto agarrado) ---
  pinMode(PIN_MOSFET_MAG, OUTPUT);
  digitalWrite(PIN_MOSFET_MAG, HIGH);
  Serial.println("[ELECTROIMAN] ENCENDIDO (para probar con el objeto agarrado).");

  return; // No se inicializa nada más: este modo solo prueba servo + color + electroimán.
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

#if MODO_OPERACION == 0
  // --- WiFi + MQTT (opcional): se intenta al final, después de que todo el
  // hardware físico ya quedó exactamente igual que en la versión sin app.
  // Si no hay red configurada o el broker no responde, esto no bloquea ni
  // altera el arranque de la máquina de estados (que sigue empezando en
  // ESTADO_HOMING como siempre).
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
// AJUSTE INTERACTIVO DEL SERVOMOTOR MG90S POR SERIAL (+ LECTURA DE COLOR)
// ==========================================
int anguloServoActual = SERVO_ANGULO_ARRIBA;
unsigned long ultimaLecturaColorMs = 0;
const unsigned long INTERVALO_LECTURA_COLOR_MS = 500;

void loop() {
  if (Serial.available() > 0) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() > 0) {
      if (linea == "1") {
        digitalWrite(PIN_MOSFET_MAG, HIGH);
        Serial.println("[ELECTROIMAN] ENCENDIDO");
      } else if (linea == "0") {
        digitalWrite(PIN_MOSFET_MAG, LOW);
        Serial.println("[ELECTROIMAN] APAGADO");
      } else if (linea == "+") {
        anguloServoActual += SERVO_PASO_AJUSTE_GRADOS;
        anguloServoActual = constrain(anguloServoActual, 0, 180);
        miServo.write(anguloServoActual);
        Serial.printf("[SERVO] -> %d grados\n", anguloServoActual);
      } else if (linea == "-") {
        anguloServoActual -= SERVO_PASO_AJUSTE_GRADOS;
        anguloServoActual = constrain(anguloServoActual, 0, 180);
        miServo.write(anguloServoActual);
        Serial.printf("[SERVO] -> %d grados\n", anguloServoActual);
      } else if (linea.equalsIgnoreCase("A")) {
        anguloServoActual = SERVO_ANGULO_ARRIBA;
        miServo.write(anguloServoActual);
        Serial.printf("[SERVO] -> %d grados\n", anguloServoActual);
      } else if (linea.equalsIgnoreCase("B")) {
        anguloServoActual = SERVO_ANGULO_ABAJO;
        miServo.write(anguloServoActual);
        Serial.printf("[SERVO] -> %d grados\n", anguloServoActual);
      } else {
        bool esNumero = linea.length() > 0;
        for (unsigned int i = 0; i < linea.length() && esNumero; i++) {
          char c = linea.charAt(i);
          if (!isDigit(c) && !(i == 0 && c == '-')) esNumero = false;
        }
        if (!esNumero) {
          Serial.println("[ERROR] Comando invalido. Usa '+', '-', un angulo (0-180), 'A' (arriba), 'B' (abajo), '1'/'0' (electroiman).");
        } else {
          anguloServoActual = constrain(linea.toInt(), 0, 180);
          miServo.write(anguloServoActual);
          Serial.printf("[SERVO] -> %d grados\n", anguloServoActual);
        }
      }
    }
  }

  // Lectura de color periódica (no bloqueada por los comandos del servo),
  // para ver en vivo cómo cambia el color leído según la altura del servo.
  if (millis() - ultimaLecturaColorMs >= INTERVALO_LECTURA_COLOR_MS) {
    ultimaLecturaColorMs = millis();
    TipoColor color = obtenerColorTCS3200();
    const char *nombreColor = (color == ROJO) ? "ROJO"
                            : (color == VERDE) ? "VERDE"
                            : (color == AZUL) ? "AZUL"
                            : "SIN LECTURA";
    Serial.printf("[SERVO %d grados] Color: %s\n", anguloServoActual, nombreColor);
  }
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
    // RangeStatus: 0=Valid, 1=Sigma Fail, 2=Signal Fail, 3=Min Range Fail,
    // 4=Phase Fail, 5=Hardware Fail. Se imprime siempre el status crudo para
    // ver en qué estado cae el sensor cuando no hay nada al frente (suele
    // ser 1 o 2, no solo 4).
    if (medida.RangeStatus == 0) {
      Serial.printf("[PRUEBA] Color: %s | Distancia: %u mm (status:%d VALIDA)\n",
                    nombreColor, medida.RangeMilliMeter, medida.RangeStatus);
    } else {
      Serial.printf("[PRUEBA] Color: %s | Distancia: %u mm (status:%d NO CONFIABLE, descartar)\n",
                    nombreColor, medida.RangeMilliMeter, medida.RangeStatus);
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
  // --- Mantenimiento WiFi/MQTT (no altera la máquina de estados) ---
  if (WiFi.status() == WL_CONNECTED) {
    reconectarMQTT();
    mqttClient.loop(); // procesa mensajes entrantes -> puede invocar callbackMQTT()
  }
  if (millis() - ultimoEnvioTelemetria > INTERVALO_TELEMETRIA_MS) {
    ultimoEnvioTelemetria = millis();
    publicarTelemetria();
  }

  switch (estadoActual) {

    // -------------------------------------------------------------
    // ESTADO MANUAL: espera comandos MQTT/Serial. Solo se llega aquí si
    // llegó un "STOP" por MQTT; el ciclo automático nunca lo visita solo.
    // -------------------------------------------------------------
    case ESTADO_MANUAL:
      delay(20);
      break;

    // -------------------------------------------------------------
    // ESTADO 1: HOMING (Retorno a origen de los ejes X e Y)
    // -------------------------------------------------------------
    case ESTADO_HOMING:
      Serial.println("[MAQUINA] Estado: HOMING Ejes X e Y...");

      // Aseguramos que el electroimán esté apagado y el servo arriba antes del home
      digitalWrite(PIN_MOSFET_MAG, LOW);
      electroimanEstadoMQTT = false;
      miServo.write(SERVO_ANGULO_ARRIBA);
      servoAbajoMQTT = false;
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

      // Busca el mínimo real de distancia (más preciso que "borde + radio"),
      // pero por OBJETO, no global: mientras la distancia sigue bajando (o
      // igualando) el mínimo visto, sigue siendo la misma tapa. En cuanto
      // hay varias lecturas seguidas que YA NO mejoran el mínimo (empezó a
      // subir = pasamos el punto más cercano de ESTA tapa), se corta ahí
      // mismo -- así, si hay una segunda tapa pegada justo después (sin un
      // hueco real de "sin detección"), no se sigue de largo mezclándolas:
      // se corta por la subida antes de llegar a su propio mínimo.
      {
        bool objetoEnVista = false;
        long xEnMinimo = 0;
        long yMinimoPasos = 0;
        int lecturasFallidasSeguidas = 0;
        int lecturasSinMejorarSeguidas = 0;

        while (posicionActualX < LIMITE_PASOS_X && estadoActual == ESTADO_ESCANEO) {
          // Dar varios pasos seguidos en X antes de volver a leer el sensor:
          // el VL53L0X tarda ~30-50ms por medición, así que leerlo en CADA
          // paso individual frena el escaneo a paso de tortuga. Se agrupan
          // PASOS_ENTRE_LECTURAS_ESCANEO pasos (rápido) entre cada lectura.
          for (int i = 0; i < PASOS_ENTRE_LECTURAS_ESCANEO && posicionActualX < LIMITE_PASOS_X; i++) {
            darPaso(PIN_X_STEP, PIN_X_DIR, DIR_X_ADELANTE, 1500);
            posicionActualX++;
          }

          // Deja que MQTT procese un posible "STOP" sin frenar el escaneo
          // (el escaneo completo puede tardar varios segundos en un solo
          // paso por loop(), y sin esto el broker no vería señales de vida).
          if (WiFi.status() == WL_CONNECTED) mqttClient.loop();

          long yCalculado = 0;
          bool huboDeteccion = leerSensorDistancia(yCalculado);

          if (huboDeteccion) {
            lecturasFallidasSeguidas = 0;
            if (!objetoEnVista || yCalculado < yMinimoPasos) {
              yMinimoPasos = yCalculado;
              xEnMinimo = posicionActualX;
              lecturasSinMejorarSeguidas = 0;
            } else {
              lecturasSinMejorarSeguidas++;
            }
            objetoEnVista = true;

            // Ya pasamos el punto más cercano de esta tapa: cortar aquí
            // antes de que el escaneo siga hacia una posible tapa siguiente.
            if (lecturasSinMejorarSeguidas >= UMBRAL_CONFIRMAR_MINIMO_ESCANEO) break;
          } else if (objetoEnVista) {
            // Sin detección, pero ya veníamos viendo el objeto: tolera
            // ruido (ej. tapas azules/verdes reflejan peor el IR) antes de
            // asumir que se acabó por falta de rango, no por subida.
            lecturasFallidasSeguidas++;
            if (lecturasFallidasSeguidas >= TOLERANCIA_PERDIDAS_ESCANEO) break;
          }
        }

        if (objetoEnVista) {
          long pasosRadioTapa = (long)(RADIO_TAPA_MM * PASOS_POR_MM);
          objetoDetectadoX = xEnMinimo;
          objetoDetectadoY = constrain(yMinimoPasos + pasosRadioTapa, 0, LIMITE_PASOS_Y);
          Serial.printf("[DETECCION] Minimo de esta tapa en X: %ld pasos (%.1f mm) | Y minimo: %.1f mm + radio %.1f mm -> objetivo agarre Y: %ld pasos (%.1f mm)!\n",
                        xEnMinimo, xEnMinimo / PASOS_POR_MM,
                        yMinimoPasos / PASOS_POR_MM, RADIO_TAPA_MM,
                        objetoDetectadoY, objetoDetectadoY / PASOS_POR_MM);
          estadoActual = ESTADO_GOTO_OBJETO;
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
    case ESTADO_GOTO_OBJETO: {
      Serial.println("[MAQUINA] Estado: IR A LA POSICION DEL OBJETO...");
      // Nos movemos al punto detectado + el ajuste fino de Y (calibrable
      // por MQTT sin recompilar, ver ajusteFinoYMm).
      long yObjetivoConAjuste = constrain(
          objetoDetectadoY + (long)(ajusteFinoYMm * PASOS_POR_MM), 0, LIMITE_PASOS_Y);
      Serial.printf("[GOTO] Y detectado: %ld pasos (%.1f mm) + ajuste fino: %.1f mm -> objetivo: %ld pasos (%.1f mm)\n",
                    objetoDetectadoY, objetoDetectadoY / PASOS_POR_MM, ajusteFinoYMm,
                    yObjetivoConAjuste, yObjetivoConAjuste / PASOS_POR_MM);
      moverACoordenadas(objetoDetectadoX, yObjetivoConAjuste);
      delay(500);
      estadoActual = ESTADO_AGARRE;
      break;
    }

    // -------------------------------------------------------------
    // ESTADO 4: AGARRE (Bajar servo, activar imán y levantar)
    // -------------------------------------------------------------
    case ESTADO_AGARRE:
      Serial.println("[MAQUINA] Estado: AGARRE Y ACOPLE...");

      // 1. Baja el servomotor para acercar el electroimán al objeto
      miServo.write(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
      delay(1000); // Esperar a que el servo llegue abajo

      // 2. Activa el Electroimán
      digitalWrite(PIN_MOSFET_MAG, HIGH);
      electroimanEstadoMQTT = true;
      Serial.println("[ACTUADOR] Electroimán ENCENDIDO.");
      delay(1000); // Dar un segundo para asegurar el acople magnético

      // 3. Levanta el servomotor con el objeto agarrado, directo a la altura
      // de lectura de color (SERVO_ANGULO_LECTURA_COLOR), no a
      // SERVO_ANGULO_ARRIBA: esa parada intermedia era innecesaria (120° es
      // igual de seguro para moverse que 90°) y solo agregaba una segunda
      // rampa de servo más adelante, con otra oportunidad de que el objeto
      // se resbale. Se mantiene ahí hasta ESTADO_DEPOSITAR.
      //
      // Los drivers de los NEMA17 mantienen corriente de sostenimiento todo
      // el tiempo que están habilitados, incluso quietos, compitiendo por la
      // misma alimentación que el servo (comparten riel). Justo aquí es
      // donde el servo necesita más corriente para levantar el objeto, así
      // que se duermen los drivers (sin soltar la posición de los ejes,
      // solo cortan su corriente de sostenimiento) mientras dura el
      // levantamiento, y se reactivan antes de volver a mover algún eje.
      // Además se sube en rampa (no de golpe) para no lanzar el objeto por
      // el arranque brusco del servo.
      digitalWrite(PIN_MOTOR_RESET, LOW);
      moverServoGradual(SERVO_ANGULO_LECTURA_COLOR);
      servoAbajoMQTT = false;
      delay(500); // Margen extra por si el servo quedó forcejeando al final de la rampa
      digitalWrite(PIN_MOTOR_RESET, HIGH);
      delay(10); // Pequeño margen para que los drivers se reactiven antes de dar pasos

      // 4. El sensor TCS3200 está anclado cerca del home de Y (no viaja con
      // el cabezal), así que hay que acercar el objeto hasta ahí para leerlo.
      // El servo ya quedó a la altura correcta, así que al llegar puede leer
      // de inmediato, sin un segundo movimiento de servo.
      moverACoordenadas(posicionActualX, POS_Y_LECTURA_COLOR);
      delay(300);

      estadoActual = ESTADO_LORE_COLOR;
      break;

    // -------------------------------------------------------------
    // ESTADO 5: LECTURA DE COLOR (Escanear espectro de color)
    // -------------------------------------------------------------
    case ESTADO_LORE_COLOR:
      Serial.println("[MAQUINA] Estado: LECTURA DE COLOR...");
      // El servo ya está en SERVO_ANGULO_LECTURA_COLOR desde ESTADO_AGARRE;
      // no hace falta moverlo de nuevo aquí (el settle ya se hizo con el
      // delay(300) tras el movimiento a POS_Y_LECTURA_COLOR en ESTADO_AGARRE).
      // Espera extra antes de leer: le da tiempo al objeto/cabezal de
      // asentarse del todo y a la iluminación de estabilizarse.
      delay(2500);

      colorDetectado = obtenerColorTCS3200();
      publicarTelemetria();

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
    case ESTADO_DEPOSITAR: {
      Serial.println("[MAQUINA] Estado: CLASIFICAR Y DEPOSITAR...");

      // Si hay una caja destino forzada desde la web (colorDestinoManual !=
      // DESCONOCIDO), se usa esa en vez del color leído por el TCS3200. Se
      // sobreescribe colorDetectado para que la telemetría/UI reflejen a
      // dónde va realmente el objeto.
      if (colorDestinoManual != DESCONOCIDO) {
        Serial.printf("[CALIBRACION] Caja destino forzada activa: se ignora color detectado (%s), va a %s\n",
                      getColorNombre(colorDetectado), getColorNombre(colorDestinoManual));
        colorDetectado = colorDestinoManual;
      }

      // Seleccionar coordenadas de caja de destino según color
      long destinoX, destinoY;
      if (colorDetectado == ROJO) {
        destinoX = POS_X_ROJO;
        destinoY = POS_Y_ROJO;
        contadorRojo++;
      } else if (colorDetectado == VERDE) {
        destinoX = POS_X_VERDE;
        destinoY = POS_Y_VERDE;
        contadorVerde++;
      } else { // AZUL
        destinoX = POS_X_AZUL;
        destinoY = POS_Y_AZUL;
        contadorAzul++;
      }

      // Mover los ejes al contenedor correspondiente
      moverACoordenadas(destinoX, destinoY);
      delay(500);

      // Baja el servo para depositar. El objeto sigue agarrado (electroimán
      // aún encendido) y viene de SERVO_ANGULO_LECTURA_COLOR (120°), un salto
      // más grande que antes, así que se baja en rampa para no botarlo.
      moverServoGradual(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
      delay(500);

      // Desactivar el Electroimán para soltar el objeto
      digitalWrite(PIN_MOSFET_MAG, LOW);
      electroimanEstadoMQTT = false;
      Serial.println("[ACTUADOR] Electroimán APAGADO. Objeto liberado.");
      delay(1000); // Esperar a que caiga

      // Volver a subir el cabezal vacío
      miServo.write(SERVO_ANGULO_ARRIBA);
      servoAbajoMQTT = false;
      delay(500);

      publicarTelemetria();
      estadoActual = ESTADO_RETORNO;
      break;
    }

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
