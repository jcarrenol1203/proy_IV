/**
 * Remote Control & Telemetry JavaScript for Object Classifier Machine
 * Connects via MQTT WebSocket (WSS) to any free public or private broker.
 */

// Machine Dimensions in millimeters
const MACHINE_MAX_X_MM = 380;
const MACHINE_MAX_Y_MM = 180;

// Default MQTT Config
let config = {
  host: 'broker.emqx.io',
  port: 8084,
  path: '/mqtt',
  protocol: 'wss',
  topicControl: 'clasificador/control',
  topicTelemetry: 'clasificador/telemetry',
  topicStatus: 'clasificador/status'
};

// Global State Object
let systemState = {
  connected: false,
  espOnline: false,
  state: 'DESCONECTADO',
  mode: 'MANUAL',
  x: 0,
  y: 0,
  magnet: false,
  servo: 'ARRIBA',
  color: 'DESCONOCIDO',
  distanceMm: 0,
  counts: { red: 0, green: 0, blue: 0 }
};

let mqttClient = null;
let canvas, ctx;

// Initialize App on DOM Load
document.addEventListener('DOMContentLoaded', () => {
  initCanvas();
  initUIEventListeners();
  connectMQTT();

  // Redraw canvas on window resize
  window.addEventListener('resize', drawMachineCanvas);
});

// ==========================================
// 1. MQTT CONNECTION HANDLING
// ==========================================
function connectMQTT() {
  const url = `${config.protocol}://${config.host}:${config.port}${config.path}`;
  const clientId = 'web_client_' + Math.random().toString(16).substring(2, 10);

  logEvent(`Conectando a broker MQTT en ${url}...`, 'info');
  updateBrokerStatus(false, 'Conectando...');

  if (mqttClient) {
    try { mqttClient.end(true); } catch (e) {}
  }

  mqttClient = mqtt.connect(url, {
    clientId: clientId,
    clean: true,
    connectTimeout: 8000,
    keepalive: 30
  });

  mqttClient.on('connect', () => {
    systemState.connected = true;
    updateBrokerStatus(true, `Conectado: ${config.host}`);
    logEvent(`¡Conectado exitosamente al broker MQTT!`, 'tx');

    // Subscribe to telemetry and status topics
    mqttClient.subscribe(config.topicTelemetry, (err) => {
      if (!err) logEvent(`Suscrito a tópico: ${config.topicTelemetry}`, 'info');
    });
    mqttClient.subscribe(config.topicStatus, (err) => {
      if (!err) logEvent(`Suscrito a tópico: ${config.topicStatus}`, 'info');
    });
  });

  mqttClient.on('message', (topic, payload) => {
    try {
      const data = JSON.parse(payload.toString());
      if (topic === config.topicTelemetry) {
        handleTelemetryMessage(data);
      } else if (topic === config.topicStatus) {
        handleStatusMessage(data);
      }
    } catch (e) {
      logEvent(`Mensaje recibido (${topic}): ${payload.toString()}`, 'rx');
    }
  });

  mqttClient.on('error', (err) => {
    logEvent(`Error MQTT: ${err.message}`, 'error');
    updateBrokerStatus(false, 'Error de Conexión');
  });

  mqttClient.on('offline', () => {
    systemState.connected = false;
    updateBrokerStatus(false, 'Broker Offline');
    updateESPStatus(false);
  });
}

function sendCommand(cmdObject) {
  if (!mqttClient || !systemState.connected) {
    logEvent('No se puede enviar comando: Broker MQTT desconectado.', 'error');
    alert('Primero debes estar conectado al broker MQTT.');
    return;
  }

  const payload = JSON.stringify(cmdObject);
  mqttClient.publish(config.topicControl, payload, { qos: 0 }, (err) => {
    if (err) {
      logEvent(`Error al publicar comando: ${err.message}`, 'error');
    } else {
      logEvent(`Comando enviado -> ${payload}`, 'tx');
    }
  });
}

// ==========================================
// 2. TELEMETRY & UI UPDATES
// ==========================================
function handleTelemetryMessage(data) {
  systemState.espOnline = true;
  updateESPStatus(true);

  if (data.state !== undefined) systemState.state = data.state;
  if (data.mode !== undefined) systemState.mode = data.mode;
  if (data.x_mm !== undefined) systemState.x = parseFloat(data.x_mm);
  if (data.y_mm !== undefined) systemState.y = parseFloat(data.y_mm);
  if (data.magnet !== undefined) systemState.magnet = Boolean(data.magnet);
  if (data.servo !== undefined) systemState.servo = data.servo;
  if (data.color !== undefined) systemState.color = data.color;
  if (data.dist_mm !== undefined) systemState.distanceMm = data.dist_mm;
  if (data.counts) {
    systemState.counts.red = data.counts.red || 0;
    systemState.counts.green = data.counts.green || 0;
    systemState.counts.blue = data.counts.blue || 0;
  }

  updateDashboardUI();
}

function handleStatusMessage(data) {
  if (data.online !== undefined) {
    systemState.espOnline = data.online;
    updateESPStatus(data.online);
  }
}

function updateDashboardUI() {
  // Machine State Tag
  const stateTag = document.getElementById('machine-state-tag');
  stateTag.textContent = `ESTADO: ${systemState.state}`;

  // Telemetry Cards
  const magnetCard = document.getElementById('card-magnet');
  magnetCard.textContent = systemState.magnet ? 'ENCENDIDO' : 'APAGADO';
  magnetCard.style.color = systemState.magnet ? '#00e676' : '#ffffff';

  const servoCard = document.getElementById('card-servo');
  servoCard.textContent = systemState.servo;

  const distanceCard = document.getElementById('card-distance');
  distanceCard.textContent = `${systemState.distanceMm} mm`;

  // Color Badge
  const colorCard = document.getElementById('card-color');
  colorCard.textContent = systemState.color;
  colorCard.className = 'card-value color-badge ' + systemState.color.toLowerCase();

  // Classified counts
  document.getElementById('count-red').textContent = systemState.counts.red;
  document.getElementById('count-green').textContent = systemState.counts.green;
  document.getElementById('count-blue').textContent = systemState.counts.blue;

  // Sync Switches UI state if not actively dragging
  const magnetSwitch = document.getElementById('switch-magnet');
  magnetSwitch.checked = systemState.magnet;
  document.getElementById('magnet-switch-text').textContent = systemState.magnet ? 'Activado (Imán ON)' : 'Desactivado (Imán OFF)';

  const servoSwitch = document.getElementById('switch-servo');
  servoSwitch.checked = (systemState.servo === 'ABAJO' || systemState.servo === 'DOWN');
  document.getElementById('servo-switch-text').textContent = (systemState.servo === 'ABAJO' || systemState.servo === 'DOWN') ? 'Abajo (Contacto)' : 'Arriba (Elevado)';

  // Canvas positions
  document.getElementById('canvas-x').textContent = systemState.x.toFixed(1);
  document.getElementById('canvas-y').textContent = systemState.y.toFixed(1);

  // Redraw Canvas Visualizer
  drawMachineCanvas();
}

function updateBrokerStatus(online, text) {
  const dot = document.getElementById('broker-dot');
  const label = document.getElementById('broker-status-text');
  dot.className = 'dot ' + (online ? 'online' : 'offline');
  label.textContent = text;
}

function updateESPStatus(online) {
  const dot = document.getElementById('esp-dot');
  const label = document.getElementById('esp-status-text');
  dot.className = 'dot ' + (online ? 'online' : 'offline');
  label.textContent = online ? 'ESP32: Conectado' : 'ESP32: Offline';
}

// ==========================================
// 3. CANVAS 2D MACHINE VISUALIZATION
// ==========================================
function initCanvas() {
  canvas = document.getElementById('machineCanvas');
  ctx = canvas.getContext('2d');
  drawMachineCanvas();
}

function drawMachineCanvas() {
  if (!canvas || !ctx) return;

  const w = canvas.width;
  const h = canvas.height;

  // Clear Canvas
  ctx.clearRect(0, 0, w, h);

  // Margins & Scaled Drawing Area
  const margin = 30;
  const drawWidth = w - margin * 2;
  const drawHeight = h - margin * 2;

  // Background Grid Box
  ctx.fillStyle = '#060a12';
  ctx.fillRect(margin, margin, drawWidth, drawHeight);

  ctx.strokeStyle = 'rgba(0, 242, 254, 0.15)';
  ctx.lineWidth = 1;

  // Grid Lines (every 50mm)
  for (let x = 0; x <= MACHINE_MAX_X_MM; x += 50) {
    const px = margin + (x / MACHINE_MAX_X_MM) * drawWidth;
    ctx.beginPath();
    ctx.moveTo(px, margin);
    ctx.lineTo(px, margin + drawHeight);
    ctx.stroke();

    // X Axis Labels
    ctx.fillStyle = '#5c6b80';
    ctx.font = '10px JetBrains Mono';
    ctx.fillText(`${x}`, px - 8, margin - 8);
  }

  for (let y = 0; y <= MACHINE_MAX_Y_MM; y += 30) {
    const py = margin + drawHeight - (y / MACHINE_MAX_Y_MM) * drawHeight;
    ctx.beginPath();
    ctx.moveTo(margin, py);
    ctx.lineTo(margin + drawWidth, py);
    ctx.stroke();

    // Y Axis Labels
    ctx.fillStyle = '#5c6b80';
    ctx.font = '10px JetBrains Mono';
    ctx.fillText(`${y}`, margin - 24, py + 3);
  }

  // Draw Storage Bins Area (Y: 150 to 180mm, banda superior en todo el ancho de X)
  const binBandHeight = ((180 - 150) / MACHINE_MAX_Y_MM) * drawHeight;
  const binBandTop = margin; // Y=180 (tope del eje) queda en el borde superior del canvas
  ctx.fillStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.fillRect(margin, binBandTop, drawWidth, binBandHeight);
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
  ctx.strokeRect(margin, binBandTop, drawWidth, binBandHeight);

  // Bins: ROJO (X 0-126.7mm), VERDE (X 126.7-253.3mm), AZUL (X 253.3-380mm)
  const binW = drawWidth / 3;

  // Red Bin
  ctx.fillStyle = 'rgba(255, 23, 68, 0.15)';
  ctx.fillRect(margin + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.strokeStyle = '#ff1744';
  ctx.strokeRect(margin + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.fillStyle = '#ff1744';
  ctx.font = 'bold 11px Outfit';
  ctx.fillText('ROJO', margin + 10, binBandTop + binBandHeight / 2);

  // Green Bin
  ctx.fillStyle = 'rgba(0, 230, 118, 0.15)';
  ctx.fillRect(margin + binW + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.strokeStyle = '#00e676';
  ctx.strokeRect(margin + binW + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.fillStyle = '#00e676';
  ctx.fillText('VERDE', margin + binW + 10, binBandTop + binBandHeight / 2);

  // Blue Bin
  ctx.fillStyle = 'rgba(79, 172, 254, 0.15)';
  ctx.fillRect(margin + binW * 2 + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.strokeStyle = '#4facfe';
  ctx.strokeRect(margin + binW * 2 + 4, binBandTop + 4, binW - 8, binBandHeight - 8);
  ctx.fillStyle = '#4facfe';
  ctx.fillText('AZUL', margin + binW * 2 + 10, binBandTop + binBandHeight / 2);

  // Scanning Sweep Beam Effect (if in ESCANEO state)
  if (systemState.state === 'ESCANEO') {
    const headPx = margin + (systemState.x / MACHINE_MAX_X_MM) * drawWidth;
    const gradient = ctx.createLinearGradient(headPx - 25, 0, headPx, 0);
    gradient.addColorStop(0, 'rgba(0, 242, 254, 0)');
    gradient.addColorStop(1, 'rgba(0, 242, 254, 0.3)');

    ctx.fillStyle = gradient;
    ctx.fillRect(headPx - 25, margin, 25, drawHeight);

    ctx.strokeStyle = '#00f2fe';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(headPx, margin);
    ctx.lineTo(headPx, margin + drawHeight);
    ctx.stroke();
  }

  // Draw Gantry Rail X
  const headX = Math.min(Math.max(systemState.x, 0), MACHINE_MAX_X_MM);
  const headY = Math.min(Math.max(systemState.y, 0), MACHINE_MAX_Y_MM);

  const headPxX = margin + (headX / MACHINE_MAX_X_MM) * drawWidth;
  const headPxY = margin + drawHeight - (headY / MACHINE_MAX_Y_MM) * drawHeight;

  // Rail X line
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(headPxX, margin);
  ctx.lineTo(headPxX, margin + drawHeight);
  ctx.stroke();

  // Head Crosshair Target
  ctx.strokeStyle = systemState.magnet ? '#00e676' : '#00f2fe';
  ctx.lineWidth = 2;

  // Outer Magnet Halo Glow if active
  if (systemState.magnet) {
    ctx.beginPath();
    ctx.arc(headPxX, headPxY, 18, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 230, 118, 0.25)';
    ctx.fill();
    ctx.strokeStyle = '#00e676';
    ctx.stroke();
  }

  // Head Circle Icon
  ctx.beginPath();
  ctx.arc(headPxX, headPxY, 10, 0, Math.PI * 2);
  ctx.fillStyle = systemState.magnet ? '#00e676' : '#00f2fe';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Magnet symbol inside head
  ctx.fillStyle = '#040914';
  ctx.font = 'bold 10px FontAwesome';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('🧲', headPxX, headPxY);
}

// ==========================================
// 4. UI EVENT LISTENERS
// ==========================================
function initUIEventListeners() {
  // Config Modal Toggle
  document.getElementById('btn-toggle-config').addEventListener('click', () => {
    document.getElementById('config-panel').classList.remove('hidden');
  });

  document.getElementById('btn-close-config').addEventListener('click', () => {
    document.getElementById('config-panel').classList.add('hidden');
  });

  document.getElementById('btn-reconnect-mqtt').addEventListener('click', () => {
    config.host = document.getElementById('mqtt-host').value.trim();
    config.port = parseInt(document.getElementById('mqtt-port').value);
    config.path = document.getElementById('mqtt-path').value.trim();
    config.protocol = document.getElementById('mqtt-proto').value;
    config.topicControl = document.getElementById('mqtt-topic-ctrl').value.trim();
    config.topicTelemetry = document.getElementById('mqtt-topic-tele').value.trim();

    document.getElementById('config-panel').classList.add('hidden');
    connectMQTT();
  });

  // Start & Stop Scan Buttons
  document.getElementById('btn-start-scan').addEventListener('click', () => {
    sendCommand({ cmd: 'START_SCAN' });
  });

  document.getElementById('btn-stop-scan').addEventListener('click', () => {
    sendCommand({ cmd: 'STOP' });
  });

  document.getElementById('btn-homing').addEventListener('click', () => {
    sendCommand({ cmd: 'HOMING' });
  });

  // Electromagnet Toggle Switch
  document.getElementById('switch-magnet').addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    sendCommand({ cmd: 'MAGNET', state: isChecked });
  });

  // Servo Height Toggle Switch
  document.getElementById('switch-servo').addEventListener('change', (e) => {
    const isChecked = e.target.checked;
    sendCommand({ cmd: 'SERVO', state: isChecked ? 'DOWN' : 'UP' });
  });

  // Helper for Selected Step Size
  function getSelectedStepSize() {
    const selected = document.querySelector('input[name="step_size"]:checked');
    return selected ? parseFloat(selected.value) : 10;
  }

  // D-Pad Navigation Buttons
  document.getElementById('btn-up-y').addEventListener('click', () => {
    const step = getSelectedStepSize();
    sendCommand({ cmd: 'MOVE_REL', dx: 0, dy: step });
  });

  document.getElementById('btn-down-y').addEventListener('click', () => {
    const step = getSelectedStepSize();
    sendCommand({ cmd: 'MOVE_REL', dx: 0, dy: -step });
  });

  document.getElementById('btn-right-x').addEventListener('click', () => {
    const step = getSelectedStepSize();
    sendCommand({ cmd: 'MOVE_REL', dx: step, dy: 0 });
  });

  document.getElementById('btn-left-x').addEventListener('click', () => {
    const step = getSelectedStepSize();
    sendCommand({ cmd: 'MOVE_REL', dx: -step, dy: 0 });
  });

  document.getElementById('btn-center-home').addEventListener('click', () => {
    sendCommand({ cmd: 'MOVE', x: 0, y: 0 });
  });

  // Custom Coordinate Move
  document.getElementById('btn-go-to-coord').addEventListener('click', () => {
    const targetX = parseFloat(document.getElementById('input-target-x').value);
    const targetY = parseFloat(document.getElementById('input-target-y').value);

    if (isNaN(targetX) || isNaN(targetY)) {
      alert('Ingresa coordenadas válidas');
      return;
    }

    sendCommand({ cmd: 'MOVE', x: targetX, y: targetY });
  });

  // Reset Motor Drivers Button
  document.getElementById('btn-reset-drivers').addEventListener('click', () => {
    sendCommand({ cmd: 'RESET_DRIVERS' });
  });

  // Clear Log Console
  document.getElementById('btn-clear-log').addEventListener('click', () => {
    document.getElementById('logConsole').innerHTML = '';
  });
}

// Log Terminal Helper
function logEvent(msg, type = 'info') {
  const consoleEl = document.getElementById('logConsole');
  if (!consoleEl) return;

  const now = new Date().toLocaleTimeString();
  const line = document.createElement('div');
  line.className = `log-entry ${type}`;
  line.textContent = `[${now}] ${msg}`;

  consoleEl.appendChild(line);
  consoleEl.scrollTop = consoleEl.scrollHeight;
}
