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
// 2 = MODO CALIBRACIÓN SERVO + SENSOR DE COLOR. Es el modo pensado para
//     caracterizar el TCS3200: mueve el servo SIEMPRE en rampa (nunca de
//     golpe) a cualquier ángulo que se le pida por Serial, y en paralelo
//     imprime en vivo la lectura del sensor de color (crudo, normalizado y
//     el veredicto de clasificación) a esa altura. Así se busca a mano la
//     altura a la que el sensor discrimina bien y, ahí mismo, se capturan
//     las firmas de cada tapa con 'CR'/'CV'/'CA' y se imprimen listas para
//     pegar en el código con 'P'. Usa servo + TCS3200 + electroimán; no
//     inicializa motores, finales de carrera ni VL53L0X.
// 1 = Solo lee e imprime color (TCS3200) y distancia (VL53L0X) por Serial
//     cada 2s, para verificar que ambos sensores funcionan. No usa ni
//     configura los pines de motores/servo/electroimán/finales de carrera,
//     así que sirve aunque solo tengas los sensores conectados.
// 0 = Corre la máquina de estados completa del clasificador XY (requiere
//     todo el hardware físicamente armado).
#define MODO_OPERACION 2

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
const int SERVO_ANGULO_LECTURA_COLOR = 120;  // Altura a la que se tomaron las firmas de color (firmaRojo/firmaVerde/firmaAzul); si se cambia, hay que recalibrar

// --- Rampa del servo (usada en TODOS los movimientos del cabezal) ---
// Antes solo se subía en rampa y se bajaba con un write() directo. Ese salto
// de golpe (ej. de 120° a 8°) hace que el MG90S pida su corriente de arranque
// máxima de una sola vez, compitiendo con la corriente de sostenimiento de
// los drivers NEMA17 en el mismo riel: la tensión se hunde y el servo se
// queda a medio camino o directamente no baja. Moviéndolo en pasos pequeños
// con una pausa entre cada uno el pico de corriente desaparece, el
// movimiento es repetible y, con el objeto agarrado, no se lanza por la
// aceleración repentina.
const int SERVO_PASO_RAMPA_GRADOS = 5;
const int SERVO_DELAY_PASO_RAMPA_MS = 25;

// --- Paso de ajuste fino del servo en MODO_OPERACION 2 (grados por '+'/'-') ---
const int SERVO_PASO_AJUSTE_GRADOS = 2;

// --- Límites de recorrido permitidos al servo (grados) ---
// Evita mandarlo contra su tope mecánico mientras se calibra a mano en
// MODO_OPERACION 2 (forzarlo contra el tope lo hace consumir corriente
// indefinidamente y calentarse).
const int SERVO_ANGULO_MIN = 0;
const int SERVO_ANGULO_MAX = 180;

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

// --- Muestreo del TCS3200 ---
// Tres detalles del muestreo que, mal puestos, arruinan la caracterización:
//
// 1. TCS_DELAY_CAMBIO_FILTRO_MS: al cambiar S2/S3 se conmuta a otro grupo de
//    fotodiodos y la salida tarda un momento en estabilizarse en la nueva
//    frecuencia. Sin esta pausa, el primer periodo que se mide es el de la
//    transición (una mezcla del canal anterior y el nuevo), que es
//    justamente lo que hace que los tres canales se parezcan entre sí.
// 2. TCS_PERIODOS_POR_MUESTRA: medir UN solo periodo con pulseIn da una
//    lectura muy ruidosa (±10% entre lecturas seguidas de la misma tapa).
//    Se promedian varios periodos seguidos.
// 3. TCS_MUESTRAS_POR_LECTURA: además, cada lectura RGB completa se repite y
//    se promedia, para que la firma capturada sea repetible.
const int TCS_DELAY_CAMBIO_FILTRO_MS = 3;
const int TCS_PERIODOS_POR_MUESTRA   = 8;
const int TCS_MUESTRAS_POR_LECTURA   = 5;

// Muestras promediadas al CAPTURAR una firma en MODO_OPERACION 2 (comandos
// 'CR'/'CV'/'CA'). Más alto que en operación normal: la firma se mide una
// sola vez y de ella dependen todas las clasificaciones posteriores, así que
// vale la pena gastar un par de segundos en promediarla bien.
const int TCS_MUESTRAS_CAPTURA_FIRMA = 20;

// --- Calibración del Sensor de Color TCS3200 (por cromaticidad normalizada) ---
// El método clásico (normalizar cada canal contra un blanco/negro de
// referencia) asume que el blanco refleja limpiamente en las 3 bandas.
// En este montaje eso no se cumplió: una muestra blanca dio MENOS señal
// cruda que las tapas de color, así que esa calibración no discriminaba.
//
// Se pasó entonces a guardar la "huella" (R,G,B crudos) de cada tapa real y
// clasificar por la firma más cercana. Eso tampoco bastó, y por esto SIEMPRE
// daba AZUL: comparando valores CRUDOS, la distancia está dominada por el
// brillo total (cuánta luz vuelve al sensor), no por el color. El brillo
// depende de la altura del cabezal, de la luz del cuarto y de lo sucia que
// esté la tapa; entre las tres firmas antiguas la diferencia era casi
// puramente de escala (R>B>G en las tres, ~12600 / ~11200 / ~10800 de suma),
// así que cualquier lectura un poco más oscura de lo normal caía
// automáticamente en la firma de menor suma -> AZUL, sin importar el color.
//
// La solución es clasificar por CROMATICIDAD: se divide cada canal por la
// suma R+G+B, quedándose con las PROPORCIONES entre canales y descartando el
// brillo total. Una tapa roja da r alto y b bajo tanto de cerca como de
// lejos; lo que cambia con la distancia es la suma, que aquí se cancela.
//
// PERO la cromaticidad sola no basta, y las firmas de abajo lo demuestran:
// en las TRES sale R > B > G, y en proporciones casi idénticas
// (0.42/0.28/0.30 la roja, 0.41/0.29/0.30 la azul). Un objeto rojo y uno
// azul NO pueden dar el mismo perfil: lo que se está midiendo ahí no es el
// color del objeto sino la GANANCIA DE CADA CANAL DEL SENSOR. Los fotodiodos
// rojos del TCS3200 responden bastante más que los verdes, y el LED blanco
// del módulo tampoco emite parejo en las tres bandas, así que el canal R sale
// alto SIEMPRE, mire lo que mire. Ese sesgo fijo se come la señal de color,
// que es mucho más pequeña.
//
// Eso es exactamente lo que corrige la CALIBRACIÓN BLANCO/NEGRO (ver
// calBlanco/calNegro abajo): midiendo una referencia blanca se aprende
// cuánto da cada canal cuando "debería" dar lo mismo en los tres, y midiendo
// el negro se aprende el piso de cada uno. Con eso, cada canal se reescala a
// 0.0-1.0 con su propia regla:
//
//     balanceado = (crudo - negro) / (blanco - negro)
//
// Tras ese reescalado el blanco da (1,1,1), el negro (0,0,0) y una tapa roja
// por fin da R alto con G/B bajos. La cromaticidad se calcula ENCIMA del
// valor balanceado: primero se quita el sesgo de canal (blanco/negro),
// después el brillo total (cromaticidad). Con las dos correcciones juntas la
// separación entre colores pasa de ~0.00001 a valores utilizables.
//
// El pipeline completo, entonces:
//   crudo (Hz) -> balance blanco/negro -> cromaticidad -> firma más cercana
//
// Las firmas se siguen guardando en CRUDO (es lo que imprime el sensor y lo
// que se anota al calibrar); cromaticidadDeFirma() les aplica exactamente el
// mismo pipeline que a la lectura antes de comparar, así que ambos lados
// quedan siempre en la misma escala.
struct FirmaColorTCS3200 { uint32_t r, g, b; };

// --- Referencias de blanco y negro (balance de canales) ---
// {0,0,0} en ambas = SIN CALIBRAR: en ese caso se salta el balance y se
// clasifica solo por cromaticidad cruda (lo que hay hoy, y que no alcanza).
// Se llenan con los comandos 'CB' (blanco) y 'CN' (negro) del
// MODO_OPERACION 2, que los imprimen listos para pegar aquí.
//
// CÓMO MEDIRLAS BIEN (es donde falla el intento clásico):
//   - BLANCO: papel blanco MATE o una tapa blanca, puesto en el electroimán,
//     a la MISMA altura exacta a la que se van a leer las tapas. Si se usa
//     algo brillante/satinado, la luz se refleja en espejo y se va del
//     sensor: da MENOS señal que una tapa de color y la calibración queda al
//     revés. Eso fue lo que pasó la vez pasada; no es que el método no
//     sirva, es que la muestra blanca era especular o estaba más lejos.
//   - NEGRO: tapa negra mate a la misma altura, o simplemente tapando el
//     sensor con la mano para que no le llegue luz. Mide el piso de cada
//     canal (el TCS3200 saca una frecuencia pequeña incluso a oscuras).
//
// No son const a propósito: en MODO_OPERACION 2 los comandos 'CB'/'CN' las
// sobreescriben EN VIVO, así que apenas capturas el blanco y el negro la
// impresión periódica ya sale balanceada y puedes comprobar el efecto en el
// momento, sin recompilar ni volver a flashear. Lo que se pega abajo es solo
// para que queden fijas en el firmware final.
FirmaColorTCS3200 calBlanco = {0, 0, 0};
FirmaColorTCS3200 calNegro  = {0, 0, 0};

// !!! LOS VALORES DE ABAJO SON LOS ANTIGUOS Y HAY QUE REEMPLAZARLOS !!!
// Con estos, el clasificador solo puede dar dos respuestas: AZUL (gana por
// un pelo entre tres firmas prácticamente iguales) o DESCONOCIDO (cuando la
// lectura queda fuera del umbral de las tres). Que es justo lo que se ve.
//
// CÓMO RECALIBRAR (MODO_OPERACION 2, ya activo):
//   1. Sube el firmware y abre el monitor serial. El servo arranca arriba y
//      el sensor imprime en vivo cada 500ms.
//   2. Verifica que los LEDs blancos del módulo TCS3200 estén ENCENDIDOS. Si
//      están apagados, el sensor solo ve la luz del cuarto y ningún ajuste
//      de firmware lo va a arreglar.
//   3. Pon una tapa en el electroimán (arranca encendido; '0' lo apaga).
//   4. Busca la altura: 'L' lleva el servo al ángulo de lectura actual, y
//      '+'/'-' lo ajustan de a pocos grados (siempre en rampa). También
//      puedes escribir un ángulo directo, ej: "115".
//      La altura BUENA es aquella en la que, al cambiar de tapa, los valores
//      NORMALIZADOS (norm r,g,b) cambian claramente. Si al cambiar de tapa
//      solo cambia la suma y no las proporciones, el sensor está demasiado
//      lejos: baja más. Como referencia, el TCS3200 quiere ~10-20mm.
//   5. Con la altura ya fija, calibra el balance: blanco al frente -> 'CB';
//      negro (o sensor tapado) -> 'CN'. A partir de ahí la impresión en vivo
//      ya sale balanceada y se nota muchísimo más la diferencia entre tapas.
//   6. Captura cada tapa: la ROJA -> 'CR', la VERDE -> 'CV', la AZUL -> 'CA'.
//   7. Envía 'P': imprime calBlanco, calNegro y las tres firmas listas
//      para copiar y pegar aquí, con el veredicto de separación.
//   8. Pega los valores, y ajusta SERVO_ANGULO_LECTURA_COLOR al ángulo que
//      elegiste (el mismo que reporta 'P').
//
// Igual que calBlanco/calNegro, no son const: 'CR'/'CV'/'CA' las reemplazan
// en vivo, así que después de capturar las tres puedes ir cambiando de tapa y
// ver si el veredicto sale correcto ANTES de recompilar con los valores.
FirmaColorTCS3200 firmaRojo  = {5319, 3546, 3717};
FirmaColorTCS3200 firmaVerde = {4629, 3225, 3355};
FirmaColorTCS3200 firmaAzul  = {4444, 3144, 3236};

// --- Rechazo por lectura demasiado lejos de toda firma ---
// Distancia cromática al cuadrado máxima para aceptar la clasificación. Las
// coordenadas de cromaticidad suman 1, así que 0.010 equivale a ~0.10 de
// distancia euclidiana repartida entre los 3 canales: una lectura que no se
// parece a ninguna tapa calibrada (sin objeto, tapa de otro color, sensor
// tapado) cae fuera y se reporta DESCONOCIDO en vez de inventar un color.
// Súbelo si tras recalibrar salen DESCONOCIDO lecturas que sí son válidas.
const float UMBRAL_MAX_DIST_CROMATICA = 0.010;

// ==========================================
// 3. VARIABLES DE ESTADO Y OBJETOS
// ==========================================
Servo miServo;
Adafruit_VL53L0X sensorDistancia = Adafruit_VL53L0X();
bool vl53Disponible = false;

// Último ángulo ordenado al servo. Se lleva aparte en vez de usar
// miServo.read() (que reconstruye el ángulo desde el ancho de pulso y puede
// devolver 1-2 grados de diferencia) para que la rampa arranque siempre
// exactamente donde terminó la anterior.
int anguloServoActual = SERVO_ANGULO_ARRIBA;

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
 * @brief Mueve el servo hasta anguloDestino en pasos de
 * SERVO_PASO_RAMPA_GRADOS con una pausa entre cada uno, en vez de saltar de
 * golpe. Se usa en TODOS los movimientos del cabezal, subiendo y bajando:
 * bajando evita el pico de corriente que dejaba al servo sin llegar abajo, y
 * subiendo con el objeto agarrado evita que el arranque brusco lo lance
 * fuera del electroimán. Ver comentario de SERVO_PASO_RAMPA_GRADOS.
 */
void moverServoGradual(int anguloDestino) {
  anguloDestino = constrain(anguloDestino, SERVO_ANGULO_MIN, SERVO_ANGULO_MAX);
  int paso = (anguloDestino >= anguloServoActual) ? SERVO_PASO_RAMPA_GRADOS : -SERVO_PASO_RAMPA_GRADOS;

  for (int a = anguloServoActual; (paso > 0) ? (a < anguloDestino) : (a > anguloDestino); a += paso) {
    miServo.write(a);
    delay(SERVO_DELAY_PASO_RAMPA_MS);
  }
  miServo.write(anguloDestino);
  anguloServoActual = anguloDestino;
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
 * @brief Mide la frecuencia (Hz) de la señal OUT del TCS3200 promediando
 * TCS_PERIODOS_POR_MUESTRA periodos seguidos. Cada periodo se arma sumando la
 * duración del semiciclo alto y el bajo (equivale al tiempo entre dos flancos
 * de subida). Es el equivalente en Arduino/ESP32 (pulseIn) de
 * measure_frequency() en sensorcolorMain.c, que usaba Input Capture por
 * hardware (TIM2) en el STM32.
 *
 * Promediar varios periodos (en vez de quedarse con uno solo, como antes) es
 * lo que hace que dos lecturas seguidas de la misma tapa den prácticamente el
 * mismo número: con un solo periodo el ruido era comparable a la diferencia
 * entre colores.
 *
 * @param timeoutUs Tiempo máximo de espera por flanco, en microsegundos.
 * @return Frecuencia en Hz, o 0 si no hubo ningún pulso (timeout).
 */
uint32_t medirFrecuenciaTCS3200(uint32_t timeoutUs) {
  unsigned long sumaPeriodos = 0;
  int periodosValidos = 0;

  for (int i = 0; i < TCS_PERIODOS_POR_MUESTRA; i++) {
    unsigned long alto = pulseIn(TCS_OUT, HIGH, timeoutUs);
    if (alto == 0) continue;
    unsigned long bajo = pulseIn(TCS_OUT, LOW, timeoutUs);
    if (bajo == 0) continue;
    sumaPeriodos += alto + bajo; // en microsegundos
    periodosValidos++;
  }

  if (periodosValidos == 0) return 0;
  unsigned long periodoPromedio = sumaPeriodos / periodosValidos;
  if (periodoPromedio == 0) return 0;
  return (uint32_t)(1000000UL / periodoPromedio);
}

/**
 * @brief Selecciona un canal del TCS3200 con S2/S3, espera a que la salida se
 * estabilice en el nuevo fotodiodo y mide su frecuencia.
 *
 * Filtros (S2, S3): Rojo LOW/LOW | Verde HIGH/HIGH | Azul LOW/HIGH |
 * Clear (sin filtro, luz total) HIGH/LOW.
 */
uint32_t medirCanalTCS3200(int s2, int s3) {
  const uint32_t TIMEOUT_US = 20000; // 20ms
  digitalWrite(TCS_S2, s2);
  digitalWrite(TCS_S3, s3);
  delay(TCS_DELAY_CAMBIO_FILTRO_MS); // sin esto se mide la transición entre canales
  return medirFrecuenciaTCS3200(TIMEOUT_US);
}

// Lectura completa del sensor: los 3 canales de color + el canal "clear"
// (luz total sin filtro). Clear no se usa para clasificar, pero al calibrar
// dice de un vistazo si está llegando luz suficiente al sensor: si es bajo,
// el cabezal está demasiado lejos de la tapa.
struct LecturaColorTCS3200 { uint32_t r, g, b, c; };

/**
 * @brief Lee los 4 canales del TCS3200 promediando muestras completas.
 * @param muestras Cuántas lecturas RGBC completas promediar.
 */
LecturaColorTCS3200 leerCanalesTCS3200(int muestras) {
  uint32_t sumaR = 0, sumaG = 0, sumaB = 0, sumaC = 0;

  for (int i = 0; i < muestras; i++) {
    sumaR += medirCanalTCS3200(LOW, LOW);
    sumaG += medirCanalTCS3200(HIGH, HIGH);
    sumaB += medirCanalTCS3200(LOW, HIGH);
    sumaC += medirCanalTCS3200(HIGH, LOW);
  }

  LecturaColorTCS3200 lectura;
  lectura.r = sumaR / muestras;
  lectura.g = sumaG / muestras;
  lectura.b = sumaB / muestras;
  lectura.c = sumaC / muestras;
  return lectura;
}

// --- Paso 1 del pipeline: balance blanco/negro (quita el sesgo de canal) ---
// Cada canal se reescala con SU PROPIA referencia: el blanco pasa a valer 1.0
// y el negro 0.0 en los tres. Sin esto, el canal R sale alto siempre (los
// fotodiodos rojos responden más y el LED no emite parejo) y ese sesgo fijo
// tapa la señal de color, que es mucho más chica. Ver calBlanco/calNegro.
struct ColorBalanceado { float r, g, b; };

/**
 * @brief ¿Hay una calibración blanco/negro utilizable? Exige que el blanco dé
 * más que el negro en los TRES canales; con las referencias en {0,0,0} (sin
 * calibrar) o con un blanco mal medido (especular, más lejos que las tapas,
 * y por eso más oscuro que ellas) devuelve false y se sigue sin balance.
 */
bool balanceBlancoDisponible() {
  return calBlanco.r > calNegro.r &&
         calBlanco.g > calNegro.g &&
         calBlanco.b > calNegro.b;
}

/**
 * @brief Reescala un canal a 0.0 (negro de referencia) - 1.0 (blanco de
 * referencia). Se recorta por abajo en 0 (una lectura más oscura que el negro
 * es ruido, no un valor negativo) y por arriba en 4.0 solo como tope de
 * seguridad: se permite pasar de 1.0 porque una tapa saturada en su canal
 * puede reflejar más que el blanco de referencia, y recortarla justo en 1.0
 * borraría precisamente la diferencia que interesa.
 */
float balancearCanal(uint32_t crudo, uint32_t negro, uint32_t blanco) {
  float rango = (float)blanco - (float)negro;
  if (rango <= 0.0) return 0.0;
  return constrain(((float)crudo - (float)negro) / rango, 0.0f, 4.0f);
}

ColorBalanceado aplicarBalanceBlanco(uint32_t r, uint32_t g, uint32_t b) {
  ColorBalanceado bal;
  if (!balanceBlancoDisponible()) {
    // Sin calibrar: se pasan los valores crudos tal cual. La cromaticidad de
    // más abajo funciona igual (solo mira proporciones), simplemente sin la
    // corrección del sesgo de canal.
    bal.r = (float)r;
    bal.g = (float)g;
    bal.b = (float)b;
    return bal;
  }
  bal.r = balancearCanal(r, calNegro.r, calBlanco.r);
  bal.g = balancearCanal(g, calNegro.g, calBlanco.g);
  bal.b = balancearCanal(b, calNegro.b, calBlanco.b);
  return bal;
}

// --- Paso 2 del pipeline: cromaticidad (quita el brillo total) ---
// Cada canal dividido por la suma de los tres, así que siempre suman 1.0 y
// describen la PROPORCIÓN entre canales sin el brillo total. Es lo que hace
// que la clasificación no dependa de la altura del cabezal ni de la luz del
// cuarto (ver comentario de las FIRMA_*).
struct CromaticidadColor { float r, g, b; };

CromaticidadColor normalizarColor(float r, float g, float b) {
  CromaticidadColor crom = {0.0, 0.0, 0.0};
  float suma = r + g + b;
  if (suma <= 0.0) return crom; // sin señal: se deja en 0 y la clasificación lo descarta
  crom.r = r / suma;
  crom.g = g / suma;
  crom.b = b / suma;
  return crom;
}

// --- Pipeline completo: crudo -> balance blanco/negro -> cromaticidad ---
// Lo usan por igual la lectura del sensor y las firmas calibradas, así que
// ambos lados se comparan siempre en la misma escala.
CromaticidadColor cromaticidadDeCrudo(uint32_t r, uint32_t g, uint32_t b) {
  ColorBalanceado bal = aplicarBalanceBlanco(r, g, b);
  return normalizarColor(bal.r, bal.g, bal.b);
}

CromaticidadColor cromaticidadDeFirma(const FirmaColorTCS3200 &firma) {
  return cromaticidadDeCrudo(firma.r, firma.g, firma.b);
}

/**
 * @brief Distancia euclidiana al cuadrado entre dos puntos de cromaticidad.
 * Se usa al cuadrado (sin raíz) porque solo importa cuál firma da la
 * distancia menor, no su valor exacto.
 */
float distanciaCuadradaCromatica(const CromaticidadColor &a, const CromaticidadColor &b) {
  float dr = a.r - b.r;
  float dg = a.g - b.g;
  float db = a.b - b.b;
  return dr * dr + dg * dg + db * db;
}

/**
 * @brief Clasifica una lectura ya tomada contra las firmas calibradas,
 * comparando cromaticidad (no valores crudos), e imprime el detalle.
 * @return ROJO/VERDE/AZUL de la firma más cercana, o DESCONOCIDO si no hubo
 * señal o si ninguna firma queda dentro de UMBRAL_MAX_DIST_CROMATICA.
 */
TipoColor clasificarLecturaColor(const LecturaColorTCS3200 &lectura) {
  if (lectura.r == 0 && lectura.g == 0 && lectura.b == 0) {
    Serial.println("[COLOR] Sin lectura (timeout en los 3 canales).");
    return DESCONOCIDO;
  }

  ColorBalanceado bal = aplicarBalanceBlanco(lectura.r, lectura.g, lectura.b);
  CromaticidadColor crom = normalizarColor(bal.r, bal.g, bal.b);

  float dRojo  = distanciaCuadradaCromatica(crom, cromaticidadDeFirma(firmaRojo));
  float dVerde = distanciaCuadradaCromatica(crom, cromaticidadDeFirma(firmaVerde));
  float dAzul  = distanciaCuadradaCromatica(crom, cromaticidadDeFirma(firmaAzul));

  // Canal dominante tras el balance: diagnóstico independiente de las firmas.
  // Con un balance blanco/negro bien hecho, una tapa roja DEBE dar 'R' aquí.
  // Si el dominante no coincide con el color real de la tapa, el problema
  // está en el balance o en la altura, no en las firmas.
  char canalDominante = (crom.r >= crom.g && crom.r >= crom.b) ? 'R'
                      : (crom.g >= crom.b) ? 'G' : 'B';

  Serial.printf("[COLOR] raw R,G,B,C: %lu,%lu,%lu,%lu Hz | bal: %.3f,%.3f,%.3f%s | norm r,g,b: %.3f,%.3f,%.3f (dom:%c) | dist2 ROJO:%.5f VERDE:%.5f AZUL:%.5f\n",
                (unsigned long)lectura.r, (unsigned long)lectura.g,
                (unsigned long)lectura.b, (unsigned long)lectura.c,
                bal.r, bal.g, bal.b, balanceBlancoDisponible() ? "" : " (SIN CAL. BLANCO/NEGRO)",
                crom.r, crom.g, crom.b, canalDominante, dRojo, dVerde, dAzul);

  float mejorDistancia = min(dRojo, min(dVerde, dAzul));
  if (mejorDistancia > UMBRAL_MAX_DIST_CROMATICA) {
    Serial.printf("[COLOR] Lectura lejos de toda firma (mejor dist2:%.5f > umbral:%.5f) -> DESCONOCIDO\n",
                  mejorDistancia, UMBRAL_MAX_DIST_CROMATICA);
    return DESCONOCIDO;
  }

  // El color es el de la firma más cercana (menor distancia) a la lectura actual.
  if (mejorDistancia == dRojo) return ROJO;
  if (mejorDistancia == dVerde) return VERDE;
  return AZUL;
}

/**
 * @brief Lee el sensor TCS3200 y determina el color predominante.
 * @return TipoColor (ROJO, VERDE, AZUL o DESCONOCIDO)
 */
TipoColor obtenerColorTCS3200() {
  return clasificarLecturaColor(leerCanalesTCS3200(TCS_MUESTRAS_POR_LECTURA));
}

#if MODO_OPERACION == 2
// ==========================================
// 5.1 CARACTERIZACIÓN DEL SENSOR DE COLOR (solo MODO_OPERACION 2)
// ==========================================
// Las capturas escriben DIRECTAMENTE sobre las variables que usa el
// clasificador (calBlanco/calNegro y firmaRojo/firmaVerde/firmaAzul), no
// sobre una copia aparte. Así, apenas capturas algo, la impresión periódica
// ya refleja el efecto: puedes capturar las tres tapas e ir cambiándolas para
// comprobar que el veredicto sale bien ANTES de recompilar. 'P' imprime el
// estado actual como código para dejarlo fijo en el firmware.
FirmaColorTCS3200* const FIRMAS[3] = {&firmaRojo, &firmaVerde, &firmaAzul};
bool firmaCapturadaValida[3] = {false, false, false};
int anguloCapturaFirma[3] = {0, 0, 0};     // altura del servo a la que se tomó cada una

const char* NOMBRE_FIRMA[3] = {"ROJO", "VERDE", "AZUL"};
const char* COMANDO_FIRMA[3] = {"CR", "CV", "CA"};

bool calBlancoValido = false;
bool calNegroValido = false;
int anguloCapturaBlanco = 0;

void imprimirAyudaCalibracion() {
  Serial.println("--- Comandos ---");
  Serial.printf("  +           : sube %d grados (en rampa)\n", SERVO_PASO_AJUSTE_GRADOS);
  Serial.printf("  -           : baja %d grados (en rampa)\n", SERVO_PASO_AJUSTE_GRADOS);
  Serial.println("  <numero>    : va directo a ese angulo (0-180), tambien en rampa. Ej: 115");
  Serial.printf("  A / U       : ARRIBA (%d grados)\n", SERVO_ANGULO_ARRIBA);
  Serial.printf("  B / D       : ABAJO (%d grados)\n", SERVO_ANGULO_ABAJO);
  Serial.printf("  L           : angulo de LECTURA DE COLOR actual (%d grados)\n", SERVO_ANGULO_LECTURA_COLOR);
  Serial.println("  1 / 0       : electroiman ENCENDIDO / APAGADO");
  Serial.println("  CB / CN     : captura la referencia de BLANCO / NEGRO (balance de canales)");
  Serial.println("  CR / CV / CA: captura la firma de la tapa ROJA / VERDE / AZUL");
  Serial.println("  P           : imprime blanco, negro y firmas como codigo listo para pegar");
  Serial.println("  T           : pausa/reanuda la impresion continua del color");
  Serial.println("  ?           : vuelve a mostrar esta ayuda");
  Serial.println("--- Como calibrar ---");
  Serial.println("  0) Revisa que los LEDs blancos del modulo TCS3200 esten ENCENDIDOS. Si estan");
  Serial.println("     apagados el sensor solo ve la luz del cuarto y nada de esto va a funcionar.");
  Serial.println("  1) Pon una tapa en el electroiman y busca con '+'/'-' la altura donde los");
  Serial.println("     valores NORMALIZADOS (norm r,g,b) cambien claramente al cambiar de tapa.");
  Serial.println("     Si al cambiar de tapa solo cambia la suma cruda y no las proporciones,");
  Serial.println("     el sensor esta muy lejos: baja mas (el TCS3200 quiere ~10-20mm).");
  Serial.println("  2) Con la altura ya fija, calibra el balance de canales: papel BLANCO MATE");
  Serial.println("     al frente -> 'CB'; superficie NEGRA o sensor tapado -> 'CN'. Desde ese");
  Serial.println("     momento la impresion sale balanceada y la diferencia entre tapas se nota");
  Serial.println("     mucho mas: el blanco pasa a valer 1,1,1 y se va el sesgo del canal rojo.");
  Serial.println("  3) Captura cada tapa: CR (roja), CV (verde), CA (azul).");
  Serial.println("  4) Cambia de tapa y verifica en vivo que 'Veredicto' acierte en las tres.");
  Serial.println("  5) Envia 'P' y pega el resultado en main.cpp.");
}

/**
 * @brief Captura la referencia de BLANCO (comando 'CB'): la lectura de una
 * superficie blanca mate puesta a la altura de trabajo. De aquí sale cuánto
 * responde cada canal cuando los tres "deberían" dar lo mismo, que es el
 * sesgo de canal a corregir.
 */
void capturarReferenciaBlanco() {
  Serial.printf("[CAL BLANCO] Midiendo a %d grados (%d muestras). Pon una superficie BLANCA MATE\n",
                anguloServoActual, TCS_MUESTRAS_CAPTURA_FIRMA);
  Serial.println("[CAL BLANCO] a la misma altura a la que vas a leer las tapas (no brillante: si es");
  Serial.println("[CAL BLANCO] satinada la luz se refleja en espejo y da menos senal que una tapa).");

  LecturaColorTCS3200 lectura = leerCanalesTCS3200(TCS_MUESTRAS_CAPTURA_FIRMA);
  if (lectura.r == 0 && lectura.g == 0 && lectura.b == 0) {
    Serial.println("[CAL BLANCO] FALLO: sin senal en los 3 canales. Revisa el cableado del TCS3200.");
    return;
  }

  calBlanco = {lectura.r, lectura.g, lectura.b};
  calBlancoValido = true;
  anguloCapturaBlanco = anguloServoActual;
  Serial.printf("[CAL BLANCO] Guardado -> %lu,%lu,%lu Hz (clear:%lu)\n",
                (unsigned long)lectura.r, (unsigned long)lectura.g,
                (unsigned long)lectura.b, (unsigned long)lectura.c);
  if (!calNegroValido) {
    Serial.println("[CAL BLANCO] Falta el negro ('CN') para que el balance se active.");
  }
}

/**
 * @brief Captura la referencia de NEGRO (comando 'CN'): el piso de cada canal
 * sin luz reflejada. El TCS3200 saca una frecuencia pequeña incluso a
 * oscuras, y restarla es lo que hace que el 0 sea un 0 real en los tres.
 */
void capturarReferenciaNegro() {
  Serial.printf("[CAL NEGRO] Midiendo a %d grados (%d muestras). Pon una superficie NEGRA MATE a la\n",
                anguloServoActual, TCS_MUESTRAS_CAPTURA_FIRMA);
  Serial.println("[CAL NEGRO] misma altura, o simplemente tapa el sensor con la mano.");

  LecturaColorTCS3200 lectura = leerCanalesTCS3200(TCS_MUESTRAS_CAPTURA_FIRMA);
  calNegro = {lectura.r, lectura.g, lectura.b};
  calNegroValido = true;
  Serial.printf("[CAL NEGRO] Guardado -> %lu,%lu,%lu Hz (clear:%lu)\n",
                (unsigned long)lectura.r, (unsigned long)lectura.g,
                (unsigned long)lectura.b, (unsigned long)lectura.c);

  if (!calBlancoValido) {
    Serial.println("[CAL NEGRO] Falta el blanco ('CB') para que el balance se active.");
    return;
  }

  // balanceBlancoDisponible() ya mira exactamente esta condición sobre las
  // variables vivas, así que aquí solo se explica el porqué cuando falla.
  if (!balanceBlancoDisponible()) {
    Serial.println("[CAL] OJO: el blanco NO da mas que el negro en los 3 canales, asi que el balance");
    Serial.println("[CAL] queda invalido y se sigue ignorando. Suele ser blanco brillante (la luz se");
    Serial.println("[CAL] refleja en espejo y se va del sensor) o medido mas lejos que el negro:");
    Serial.println("[CAL] repite 'CB' con papel blanco MATE a la misma altura que las tapas.");
    return;
  }

  Serial.printf("[CAL] Balance ACTIVO. Rango util por canal (blanco-negro): R:%ld G:%ld B:%ld\n",
                (long)calBlanco.r - (long)calNegro.r,
                (long)calBlanco.g - (long)calNegro.g,
                (long)calBlanco.b - (long)calNegro.b);
  Serial.println("[CAL] Ahora vuelve a poner cada tapa y captura sus firmas (CR/CV/CA).");
}

/**
 * @brief Toma una lectura larga y promediada de la tapa que esté puesta y la
 * guarda como firma del color indicado (índice 0=ROJO, 1=VERDE, 2=AZUL).
 */
void capturarFirmaColor(int indice) {
  Serial.printf("[CAPTURA] Midiendo firma de %s a %d grados (%d muestras, no muevas la tapa)...\n",
                NOMBRE_FIRMA[indice], anguloServoActual, TCS_MUESTRAS_CAPTURA_FIRMA);

  LecturaColorTCS3200 lectura = leerCanalesTCS3200(TCS_MUESTRAS_CAPTURA_FIRMA);
  if (lectura.r == 0 && lectura.g == 0 && lectura.b == 0) {
    Serial.println("[CAPTURA] FALLO: sin senal en los 3 canales. Revisa el cableado del TCS3200.");
    return;
  }

  *FIRMAS[indice] = {lectura.r, lectura.g, lectura.b};
  firmaCapturadaValida[indice] = true;
  anguloCapturaFirma[indice] = anguloServoActual;

  CromaticidadColor crom = cromaticidadDeCrudo(lectura.r, lectura.g, lectura.b);
  Serial.printf("[CAPTURA] %s guardado -> crudo %lu,%lu,%lu Hz (clear:%lu) | norm %.3f,%.3f,%.3f%s\n",
                NOMBRE_FIRMA[indice],
                (unsigned long)lectura.r, (unsigned long)lectura.g, (unsigned long)lectura.b,
                (unsigned long)lectura.c, crom.r, crom.g, crom.b,
                balanceBlancoDisponible() ? "" : " (SIN BALANCE: captura antes CB y CN)");
}

/**
 * @brief Imprime las firmas capturadas como código C listo para pegar, más un
 * informe de qué tan separadas quedaron entre sí. Esa separación es la que
 * decide si la calibración va a servir: si dos firmas quedan más cerca entre
 * ellas que UMBRAL_MAX_DIST_CROMATICA, el sensor no las va a poder distinguir
 * de forma confiable y hay que volver a buscar altura.
 */
void imprimirFirmasCapturadas() {
  bool faltaAlguna = false;
  for (int i = 0; i < 3; i++) {
    if (!firmaCapturadaValida[i]) {
      Serial.printf("[FIRMAS] Falta capturar %s (comando '%s').\n", NOMBRE_FIRMA[i], COMANDO_FIRMA[i]);
      faltaAlguna = true;
    }
  }
  if (faltaAlguna) return;

  bool mismaAltura = (anguloCapturaFirma[0] == anguloCapturaFirma[1] &&
                      anguloCapturaFirma[1] == anguloCapturaFirma[2]);
  if (!mismaAltura) {
    Serial.printf("[FIRMAS] OJO: se capturaron a alturas distintas (%d, %d, %d grados). Las firmas solo\n",
                  anguloCapturaFirma[0], anguloCapturaFirma[1], anguloCapturaFirma[2]);
    Serial.println("[FIRMAS] son comparables si las 3 se toman a la MISMA altura. Recomendado: repetirlas.");
  }

  Serial.println("=== PEGA ESTO EN main.cpp (reemplaza calBlanco/calNegro y las firmas) ===");
  if (calBlancoValido && calNegroValido) {
    Serial.printf("FirmaColorTCS3200 calBlanco = {%lu, %lu, %lu};\n",
                  (unsigned long)calBlanco.r, (unsigned long)calBlanco.g, (unsigned long)calBlanco.b);
    Serial.printf("FirmaColorTCS3200 calNegro  = {%lu, %lu, %lu};\n",
                  (unsigned long)calNegro.r, (unsigned long)calNegro.g, (unsigned long)calNegro.b);
  } else {
    Serial.println("// (sin balance blanco/negro: captura 'CB' y 'CN' y vuelve a enviar 'P')");
  }
  Serial.printf("FirmaColorTCS3200 firmaRojo  = {%lu, %lu, %lu};\n",
                (unsigned long)firmaRojo.r, (unsigned long)firmaRojo.g, (unsigned long)firmaRojo.b);
  Serial.printf("FirmaColorTCS3200 firmaVerde = {%lu, %lu, %lu};\n",
                (unsigned long)firmaVerde.r, (unsigned long)firmaVerde.g, (unsigned long)firmaVerde.b);
  Serial.printf("FirmaColorTCS3200 firmaAzul  = {%lu, %lu, %lu};\n",
                (unsigned long)firmaAzul.r, (unsigned long)firmaAzul.g, (unsigned long)firmaAzul.b);
  if (mismaAltura) {
    Serial.printf("const int SERVO_ANGULO_LECTURA_COLOR = %d;  // altura usada en esta calibracion\n",
                  anguloCapturaFirma[0]);
  }
  Serial.println("========================================================================");

  if (!balanceBlancoDisponible()) {
    Serial.println("[FIRMAS] AVISO: estas firmas se tomaron SIN balance blanco/negro activo. Es la");
    Serial.println("[FIRMAS] causa mas probable de que los tres colores queden pegados: sin balance,");
    Serial.println("[FIRMAS] el canal R domina siempre y tapa la diferencia real de color.");
  }

  Serial.println("--- Separacion entre firmas (distancia cromatica al cuadrado) ---");
  float peorSeparacion = -1.0;
  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 3; j++) {
      float d = distanciaCuadradaCromatica(cromaticidadDeFirma(*FIRMAS[i]),
                                           cromaticidadDeFirma(*FIRMAS[j]));
      Serial.printf("  %s vs %s: %.5f\n", NOMBRE_FIRMA[i], NOMBRE_FIRMA[j], d);
      if (peorSeparacion < 0.0 || d < peorSeparacion) peorSeparacion = d;
    }
  }
  Serial.printf("  Peor separacion: %.5f | umbral de decision: %.5f\n",
                peorSeparacion, UMBRAL_MAX_DIST_CROMATICA);
  if (peorSeparacion < UMBRAL_MAX_DIST_CROMATICA) {
    Serial.println("  VEREDICTO: MALA. Hay dos firmas mas cerca entre si que el umbral: asi no se");
    Serial.println("  distinguen esos dos colores. Si aun no calibraste blanco/negro, hazlo (CB/CN)");
    Serial.println("  y recaptura; si ya esta, baja mas el servo y vuelve a intentar.");
  } else if (peorSeparacion < UMBRAL_MAX_DIST_CROMATICA * 4) {
    Serial.println("  VEREDICTO: JUSTA. Deberia funcionar, pero con poco margen ante cambios de luz.");
    Serial.println("  Si puedes, prueba otra altura a ver si separa mas.");
  } else {
    Serial.println("  VEREDICTO: BUENA. Los tres colores quedan bien separados a esta altura.");
  }
}
#endif // MODO_OPERACION == 2

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
  doc["servo_deg"] = anguloServoActual;
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
    // La app web ya no expone este control (solo muestra la posición), pero
    // se deja disponible para cualquier otro cliente MQTT.
    const char* sState = doc["state"];
    if (sState && (strcmp(sState, "DOWN") == 0 || strcmp(sState, "ABAJO") == 0)) {
      moverServoGradual(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
    } else {
      moverServoGradual(SERVO_ANGULO_ARRIBA);
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
  Serial.println("=== MODO CALIBRACION: SERVO (RAMPA) + SENSOR DE COLOR TCS3200 ===");
  imprimirAyudaCalibracion();
  miServo.attach(PIN_SERVO);
  miServo.write(SERVO_ANGULO_ARRIBA); // Empezar con el cabezal arriba
  anguloServoActual = SERVO_ANGULO_ARRIBA;
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
  anguloServoActual = SERVO_ANGULO_ARRIBA;

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
// CALIBRACIÓN INTERACTIVA: SERVO EN RAMPA + CARACTERIZACIÓN DEL TCS3200
// ==========================================
// La idea del modo: el servo se mueve SIEMPRE con moverServoGradual() (misma
// rampa que usa la máquina de estados real, así que la altura que se calibre
// aquí es exactamente la que va a alcanzar en operación), y el sensor imprime
// en vivo lo que ve a esa altura. Cuando la altura discrimina bien, se
// capturan las tres firmas y se imprimen listas para pegar en el código.
unsigned long ultimaLecturaColorMs = 0;
const unsigned long INTERVALO_LECTURA_COLOR_MS = 500;
bool impresionColorActiva = true;

void moverServoCalibracion(int anguloDestino) {
  int anguloAnterior = anguloServoActual;
  moverServoGradual(anguloDestino);
  Serial.printf("[SERVO] %d -> %d grados (rampa de %d en %d grados)\n",
                anguloAnterior, anguloServoActual, SERVO_DELAY_PASO_RAMPA_MS, SERVO_PASO_RAMPA_GRADOS);
}

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
        moverServoCalibracion(anguloServoActual + SERVO_PASO_AJUSTE_GRADOS);
      } else if (linea == "-") {
        moverServoCalibracion(anguloServoActual - SERVO_PASO_AJUSTE_GRADOS);
      } else if (linea.equalsIgnoreCase("A") || linea.equalsIgnoreCase("U")) {
        moverServoCalibracion(SERVO_ANGULO_ARRIBA);
      } else if (linea.equalsIgnoreCase("B") || linea.equalsIgnoreCase("D")) {
        moverServoCalibracion(SERVO_ANGULO_ABAJO);
      } else if (linea.equalsIgnoreCase("L")) {
        moverServoCalibracion(SERVO_ANGULO_LECTURA_COLOR);
      } else if (linea.equalsIgnoreCase("CB")) {
        capturarReferenciaBlanco();
      } else if (linea.equalsIgnoreCase("CN")) {
        capturarReferenciaNegro();
      } else if (linea.equalsIgnoreCase("CR")) {
        capturarFirmaColor(0);
      } else if (linea.equalsIgnoreCase("CV")) {
        capturarFirmaColor(1);
      } else if (linea.equalsIgnoreCase("CA")) {
        capturarFirmaColor(2);
      } else if (linea.equalsIgnoreCase("P")) {
        imprimirFirmasCapturadas();
      } else if (linea.equalsIgnoreCase("T")) {
        impresionColorActiva = !impresionColorActiva;
        Serial.printf("[COLOR] Impresion continua %s.\n", impresionColorActiva ? "REANUDADA" : "PAUSADA");
      } else if (linea == "?") {
        imprimirAyudaCalibracion();
      } else {
        bool esNumero = true;
        for (unsigned int i = 0; i < linea.length() && esNumero; i++) {
          char c = linea.charAt(i);
          if (!isDigit(c) && !(i == 0 && c == '-')) esNumero = false;
        }
        if (!esNumero) {
          Serial.println("[ERROR] Comando invalido. Envia '?' para ver la lista de comandos.");
        } else {
          moverServoCalibracion(linea.toInt());
        }
      }
    }
  }

  // Lectura de color periódica (no bloqueada por los comandos del servo),
  // para ver en vivo cómo cambia lo que ve el sensor según la altura.
  // clasificarLecturaColor() ya imprime crudo + normalizado + distancias.
  if (impresionColorActiva && millis() - ultimaLecturaColorMs >= INTERVALO_LECTURA_COLOR_MS) {
    ultimaLecturaColorMs = millis();
    TipoColor color = clasificarLecturaColor(leerCanalesTCS3200(TCS_MUESTRAS_POR_LECTURA));
    const char *nombreColor = (color == ROJO) ? "ROJO"
                            : (color == VERDE) ? "VERDE"
                            : (color == AZUL) ? "AZUL"
                            : "SIN LECTURA / DESCONOCIDO";
    Serial.printf("[SERVO %d grados] Veredicto: %s\n", anguloServoActual, nombreColor);
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
      moverServoGradual(SERVO_ANGULO_ARRIBA);
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

      // 1. Baja el servomotor para acercar el electroimán al objeto.
      // En rampa y con los drivers de los motores dormidos, por la misma
      // razón que al subir (ver punto 3): un write() directo desde 120° hasta
      // 8° pedía toda la corriente de arranque del servo de una vez mientras
      // los NEMA17 seguían consumiendo su corriente de sostenimiento en el
      // mismo riel, y el servo se quedaba a medio bajar o no bajaba. Los ejes
      // ya están en su sitio y quietos, así que soltarles la corriente
      // mientras baja el cabezal no mueve nada.
      digitalWrite(PIN_MOTOR_RESET, LOW);
      moverServoGradual(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
      delay(500); // Margen para que el servo asiente al final de la rampa
      digitalWrite(PIN_MOTOR_RESET, HIGH);
      delay(10); // Pequeño margen para que los drivers se reactiven

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
      // grande, así que se baja en rampa y con los drivers dormidos para no
      // botarlo ni hundir la alimentación (igual que en ESTADO_AGARRE).
      digitalWrite(PIN_MOTOR_RESET, LOW);
      moverServoGradual(SERVO_ANGULO_ABAJO);
      servoAbajoMQTT = true;
      delay(500);
      digitalWrite(PIN_MOTOR_RESET, HIGH);
      delay(10);

      // Desactivar el Electroimán para soltar el objeto
      digitalWrite(PIN_MOSFET_MAG, LOW);
      electroimanEstadoMQTT = false;
      Serial.println("[ACTUADOR] Electroimán APAGADO. Objeto liberado.");
      delay(1000); // Esperar a que caiga

      // Volver a subir el cabezal vacío (también en rampa: aunque ya no lleva
      // objeto, el salto de golpe sigue siendo el pico de corriente que
      // conviene evitar)
      moverServoGradual(SERVO_ANGULO_ARRIBA);
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
