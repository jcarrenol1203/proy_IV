# Guía de Uso: Interfaz Web y Control MQTT para Clasificador de Objetos XY

Esta guía explica cómo utilizar la **Interfaz de Usuario (Web App)** y el protocolo **MQTT** para controlar el clasificador de objetos por color desde cualquier parte del mundo.

---

## 🚀 1. Configuración del ESP32 (Firmware)

1. Abre el archivo [`src/main.cpp`](file:///C:/Users/dani0/AppData/Local/Programs/Microsoft%20VS%20Code/proy_IV/src/main.cpp).
2. En las líneas 25-26, ingresa el nombre de tu red WiFi y su contraseña:
   ```cpp
   const char* WIFI_SSID = "NOMBRE_DE_TU_RED_WIFI";
   const char* WIFI_PASS = "CONTRASEÑA_DE_TU_RED_WIFI";
   ```
3. (Opcional) El broker MQTT está preconfigurado con el broker público gratuito **`broker.emqx.io`** (puerto TCP `1883`). No requiere contraseña ni registro previo.
4. Conecta tu ESP32 por USB y sube el firmware utilizando PlatformIO:
   ```bash
   pio run -e esp32dev -t upload
   ```
5. Abre el Monitor Serial (115200 baudios) en VS Code para confirmar la conexión WiFi y MQTT.

---

## 🌐 2. Acceso a la Interfaz Web desde cualquier lugar

La interfaz web está construida con **HTML5, Vanilla CSS y JS con MQTT.js sobre WebSockets (WSS)**. 

### ¿Cómo abrirla?
- **Localmente:** Simplemente haz doble clic o abre el archivo [`web_ui/index.html`](file:///C:/Users/dani0/AppData/Local/Programs/Microsoft%20VS%20Code/proy_IV/web_ui/index.html) en Chrome, Edge, Firefox, Safari o en tu navegador móvil.
- **En la nube / Celular / Remoto:** Puedes subir la carpeta `web_ui` a **GitHub Pages, Vercel o Netlify** (100% gratuito) para tener un enlace `.web.app` o `.github.io` accesible desde cualquier smartphone o computadora conectada a Internet en cualquier parte del mundo.

---

## 🎮 3. Funcionalidades de la Interfaz

### A. Visualización 2D en Tiempo Real (Canvas)
- **Área de trabajo:** Representa la cama de la máquina de **380mm x 180mm**.
- **Cabezal con Electroimán:** Muestra la posición exacta $(X, Y)$ del carro en milímetros con un cursor dinámico y un aura verde brillante cuando el electroimán está encendido.
- **Efecto de Escaneo:** Muestra un barrido láser animado en cian cuando la máquina está en modo **ESCANEO**.
- **Cajas de Almacenamiento:** Muestra las 3 zonas (Rojo, Verde, Azul) en la franja $X = 0 \text{ a } 50\text{mm}$ con contador de objetos depositados.

### B. Control Manual del Electroimán y Cabezal
- **Interruptor Electroimán:** Permite encender o apagar el imán MOSFET en tiempo real.
- **Interruptor Altura Cabezal (Servo):** Permite subir (90°) o bajar (10°) el servomotor para agarre manual.
- **D-Pad de Movimiento:** Botones **X+**, **X-**, **Y+**, **Y-** para desplazar el cabezal.
- **Selector de Paso:** Elige la distancia por clic: `1mm`, `5mm`, `10mm` o `50mm`.
- **Ir a Coordenadas (X, Y):** Escribe posiciones numéricas exactas en milímetros y presiona **Mover a Coordenada**.

### C. Proceso Automático y Escaneo
- **Iniciar Escaneo:** Cambia la máquina a modo `ESCANEO` automático para detectar objetos con el VL53L0X, tomar el objeto, clasificar su color con el TCS3200 y llevarlo a su caja correspondiente.
- **Detener / Pausa:** Detiene el recorrido automático y pasa a modo `MANUAL`.
- **Homing X/Y:** Ejecuta la secuencia de origen (retorno a finales de carrera).

---

## 📡 4. Tópicos MQTT Utilizados

- **Tópico de Control (Publicado por la Web, recibido por ESP32):** `clasificador/control`
- **Tópico de Telemetría (Publicado por ESP32 cada 300ms, recibido por la Web):** `clasificador/telemetry`
