#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ============================================================
// MICROSTEP CONFIGURATION  ← change here if you change DIPs
// ============================================================
// Match MICROSTEPS to your TB6600 SW1-SW3 DIP switches:
//
//   MICROSTEPS   SW1   SW2   SW3   steps/rev   speed notes (at STEP_INTERVAL_MIN_US=70)
//   ---------------------------------------------------------------------------------
//      1         ON    ON    ON      200        ~5x faster than shown RPMs
//      2/A       ON    OFF   ON      400
//      4         ON    ON    OFF     800
//      8         ON    OFF   OFF    1600
//      16        OFF   ON    OFF    3200        ~2x RPM of the 1/32 table below
//      32        OFF   OFF   ON     6400        <-- current
//      64        OFF   OFF   OFF   12800        ~0.5x RPM
//
// All 3 TB6600 drivers must use the SAME microstep setting.
// If you change MICROSTEPS, the RPMs printed in the Serial
// monitor will update automatically.
// ============================================================
const int MICROSTEPS = 32;
const int STEPS_PER_REV = 200 * MICROSTEPS;

// ============================================================
// SPEED TUNING
// ============================================================
// At 1/32 microstepping (6400 steps/rev):
//   800 µs  →  31 RPM  (startup)
//    70 µs  → 134 RPM  (current cruise)
//    40 µs  → 234 RPM
//    31 µs  → 300 RPM  (software limit — beyond this needs HW timer)
//
// Lower STEP_INTERVAL_MIN_US for more speed.
// Raise it if motors skip/stall at high speed.
// ============================================================
const unsigned long STEP_INTERVAL_START_US = 800;
const unsigned long STEP_INTERVAL_MIN_US   = 70;
const int PULSE_WIDTH_US = 2;

// Auto sequence timing
const unsigned long AUTO_PAUSE_MS = 500;   // "solder" dwell time per point
const int MAX_POINTS = 50;
const int NUM_MOTORS = 3;

// Derived speed constants (steps per second)
const float VMAX   = 1e6f / STEP_INTERVAL_MIN_US;
const float VSTART = 1e6f / STEP_INTERVAL_START_US;
const float ACCEL  = 50000.0f;   // steps/sec²

// Wi-Fi
const char* ssid     = "SolderMachine_AP";
const char* password = "password123";

AsyncWebServer server(80);

// ---------- Motor state ----------
struct Motor {
  uint8_t pulPin;
  uint8_t dirPin;
  volatile long steps;       // current position (absolute)
  volatile long target;      // target position (auto mode)
  volatile uint8_t mode;     // 0=stop 1=fwd 2=rev 3=to_target
  unsigned long lastStepTime;
  float currentSpeed;        // steps/sec
};

Motor motors[NUM_MOTORS] = {
  {14, 27, 0, 0, 0, 0, 0.0f},   // X
  {25, 26, 0, 0, 0, 0, 0.0f},   // Y
  {32, 33, 0, 0, 0, 0, 0.0f},   // Z
};

// ---------- Auto sequence state ----------
long pointData[MAX_POINTS][3];
volatile int numPoints = 0;
volatile int currentPoint = -1;
volatile uint8_t autoState = 0;   // 0=idle 1=moving 2=pausing 3=done
unsigned long pauseStartTime = 0;

// ---------- HTML / JS UI ----------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Solder Machine Controller</title>
<style>
*{box-sizing:border-box}
body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#f4f4f4}
h1{text-align:center}
.tabs{text-align:center;margin:20px 0}
.tab{padding:12px 30px;font-size:18px;border:none;cursor:pointer;
     background:#ccc;color:#333;border-radius:8px 8px 0 0;margin:0 2px}
.tab.active{background:#3498db;color:#fff}
.panel{display:none;background:#fff;padding:20px;border-radius:0 8px 8px 8px;
       max-width:850px;margin:0 auto;box-shadow:0 2px 8px rgba(0,0,0,.1)}
.panel.active{display:block}
.motor-row{display:flex;align-items:center;justify-content:center;gap:15px;
           margin:12px 0;padding:10px;border-radius:8px;background:#f9f9f9}
.motor-row .label{font-size:20px;font-weight:bold;width:40px}
.motor-row .pos{font-family:monospace;font-size:16px;width:110px;
                background:#222;color:#0f0;padding:5px;border-radius:4px;text-align:center}
.btn{padding:15px 25px;font-size:18px;font-weight:bold;color:#fff;border:none;
     border-radius:8px;cursor:pointer;user-select:none;touch-action:manipulation;min-width:70px}
.btn-fwd{background:#2ecc71}
.btn-rev{background:#e67e22}
.btn:active{opacity:.6}
input[type=number]{width:90px;padding:8px;font-size:16px;border:1px solid #ccc;border-radius:4px}
table{width:100%;border-collapse:collapse;margin-top:15px}
th,td{padding:8px;border:1px solid #ddd;text-align:center}
th{background:#3498db;color:#fff}
td input{width:100px;padding:6px;font-size:14px;border:1px solid #ccc;border-radius:4px}
.controls{margin-top:15px;text-align:center}
.controls button{margin:5px}
#auto-status{margin-top:15px;padding:10px;background:#eef;border-radius:8px;font-family:monospace}
</style>
</head>
<body>
<h1>Solder Machine Controller</h1>

<div class="tabs">
  <button class="tab active" id="tab-manual" onclick="setTab('manual')">Manual</button>
  <button class="tab" id="tab-auto" onclick="setTab('auto')">Automatic</button>
</div>

<!-- MANUAL -->
<div class="panel active" id="panel-manual">
  <h3 style="text-align:center">Manual Jog (all 3 axes independent)</h3>
  <div class="motor-row">
    <span class="label">X</span>
    <button class="btn btn-rev"
      onmousedown="cmd(0,'rev')" onmouseup="cmd(0,'stop')" onmouseleave="cmd(0,'stop')"
      ontouchstart="cmd(0,'rev')" ontouchend="cmd(0,'stop')" ontouchcancel="cmd(0,'stop')">−</button>
    <span class="pos" id="pos-x">0</span>
    <button class="btn btn-fwd"
      onmousedown="cmd(0,'fwd')" onmouseup="cmd(0,'stop')" onmouseleave="cmd(0,'stop')"
      ontouchstart="cmd(0,'fwd')" ontouchend="cmd(0,'stop')" ontouchcancel="cmd(0,'stop')">+</button>
  </div>
  <div class="motor-row">
    <span class="label">Y</span>
    <button class="btn btn-rev"
      onmousedown="cmd(1,'rev')" onmouseup="cmd(1,'stop')" onmouseleave="cmd(1,'stop')"
      ontouchstart="cmd(1,'rev')" ontouchend="cmd(1,'stop')" ontouchcancel="cmd(1,'stop')">−</button>
    <span class="pos" id="pos-y">0</span>
    <button class="btn btn-fwd"
      onmousedown="cmd(1,'fwd')" onmouseup="cmd(1,'stop')" onmouseleave="cmd(1,'stop')"
      ontouchstart="cmd(1,'fwd')" ontouchend="cmd(1,'stop')" ontouchcancel="cmd(1,'stop')">+</button>
  </div>
  <div class="motor-row">
    <span class="label">Z</span>
    <button class="btn btn-rev"
      onmousedown="cmd(2,'rev')" onmouseup="cmd(2,'stop')" onmouseleave="cmd(2,'stop')"
      ontouchstart="cmd(2,'rev')" ontouchend="cmd(2,'stop')" ontouchcancel="cmd(2,'stop')">−</button>
    <span class="pos" id="pos-z">0</span>
    <button class="btn btn-fwd"
      onmousedown="cmd(2,'fwd')" onmouseup="cmd(2,'stop')" onmouseleave="cmd(2,'stop')"
      ontouchstart="cmd(2,'fwd')" ontouchend="cmd(2,'stop')" ontouchcancel="cmd(2,'stop')">+</button>
  </div>
  <div class="controls">
    <button class="btn" style="background:#e74c3c" onclick="resetAll()">Reset Positions to 0</button>
  </div>
</div>

<!-- AUTO -->
<div class="panel" id="panel-auto">
  <h3 style="text-align:center">Automatic Soldering Sequence</h3>
  <div style="text-align:center;margin-bottom:15px">
    <label>Number of points: <input type="number" id="npoints" min="1" max="50" value="3" onchange="genRows()"></label>
  </div>
  <table id="points-table">
    <thead><tr><th>Point</th><th>X (steps)</th><th>Y (steps)</th><th>Z (steps)</th></tr></thead>
    <tbody></tbody>
  </table>
  <div class="controls">
    <button class="btn" style="background:#9b59b6" onclick="capturePos()">+ Capture Current Position</button>
    <button class="btn" style="background:#95a5a6" onclick="clearTable()">Clear Table</button>
  </div>
  <div class="controls">
    <button class="btn" style="background:#2ecc71" onclick="runAuto()">▶ Start Sequence</button>
    <button class="btn" style="background:#e74c3c" onclick="stopAll()">■ Stop</button>
  </div>
  <div id="auto-status">Ready.</div>
</div>

<script>
let autoTimer = null;

function setTab(name) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.getElementById('tab-'+name).classList.add('active');
  document.getElementById('panel-'+name).classList.add('active');
}

function cmd(id, dir){ fetch('/motor?id='+id+'&dir='+dir); }
function resetAll(){ if(confirm('Reset all positions to 0?')) fetch('/reset'); }
function stopAll(){
  fetch('/stop').then(()=>{
    document.getElementById('auto-status').textContent='Stopped.';
    if(autoTimer){clearInterval(autoTimer); autoTimer=null;}
  });
}

function genRows() {
  const n = parseInt(document.getElementById('npoints').value)||0;
  const tbody = document.querySelector('#points-table tbody');
  tbody.innerHTML='';
  for(let i=0;i<n;i++){
    const tr=document.createElement('tr');
    tr.innerHTML = `<td>P${i+1}</td>
      <td><input type="number" class="px" value="0"></td>
      <td><input type="number" class="py" value="0"></td>
      <td><input type="number" class="pz" value="0"></td>`;
    tbody.appendChild(tr);
  }
}

function capturePos() {
  fetch('/status').then(r=>r.json()).then(s=>{
    const tbody = document.querySelector('#points-table tbody');
    const i = tbody.children.length + 1;
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>P${i}</td>
      <td><input type="number" class="px" value="${s.x}"></td>
      <td><input type="number" class="py" value="${s.y}"></td>
      <td><input type="number" class="pz" value="${s.z}"></td>`;
    tbody.appendChild(tr);
    document.getElementById('npoints').value = i;
  });
}

function clearTable() {
  document.querySelector('#points-table tbody').innerHTML='';
  document.getElementById('npoints').value = 1;
  genRows();
}

function runAuto() {
  const rows = document.querySelectorAll('#points-table tbody tr');
  if(rows.length===0){ alert('Add at least one point.'); return; }
  let pts = [];
  for(const row of rows){
    pts.push([
      parseInt(row.querySelector('.px').value)||0,
      parseInt(row.querySelector('.py').value)||0,
      parseInt(row.querySelector('.pz').value)||0
    ]);
  }
  const ptsStr = pts.map(p=>p.join(',')).join('_');
  fetch('/auto?pts=' + encodeURIComponent(ptsStr))
    .then(r=>r.text()).then(t=>{
      document.getElementById('auto-status').textContent = t;
      if(autoTimer) clearInterval(autoTimer);
      autoTimer = setInterval(pollAuto, 300);
    });
}

function pollAuto() {
  fetch('/status').then(r=>r.json()).then(s=>{
    let m='';
    if(s.autoState===1) m='Moving to point '+(s.currentPoint+1)+' / '+s.numPoints;
    else if(s.autoState===2) m='Soldering at point '+(s.currentPoint+1)+' / '+s.numPoints;
    else if(s.autoState===3){ m='Sequence complete.'; if(autoTimer){clearInterval(autoTimer); autoTimer=null;} }
    else m='Idle.';
    document.getElementById('auto-status').textContent = m;
  }).catch(()=>{});
}

// continuous position poll
setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(s=>{
    document.getElementById('pos-x').textContent = s.x;
    document.getElementById('pos-y').textContent = s.y;
    document.getElementById('pos-z').textContent = s.z;
  }).catch(()=>{});
}, 500);

genRows();
</script>
</body>
</html>
)rawliteral";

// ============================================================
//  MOTOR CONTROL TASK (core 1)
// ============================================================
void MotorControlTask(void * pvParameters) {
  unsigned long lastYield = 0;

  for (;;) {
    unsigned long now = micros();
    bool anyMoving = false;

    for (int i = 0; i < NUM_MOTORS; i++) {
      Motor &m = motors[i];

      if (m.mode == 0) {
        m.currentSpeed = VSTART;
        m.lastStepTime = now;
        continue;
      }
      anyMoving = true;

      // ---- Determine direction & remaining ----
      int dir = 0;
      long remaining = 0;
      if (m.mode == 1) dir = 1;
      else if (m.mode == 2) dir = -1;
      else if (m.mode == 3) {
        if (m.steps == m.target) { m.mode = 0; continue; }
        dir = (m.target > m.steps) ? 1 : -1;
        remaining = labs(m.target - m.steps);
      }
      if (dir == 0) continue;

      // DIR setup (set every iteration so it's stable by pulse time)
      digitalWrite(m.dirPin, dir > 0 ? HIGH : LOW);

      // ---- Target speed ----
      float vtarget;
      if (m.mode == 3) {
        float vdecel = sqrtf(2.0f * ACCEL * (float)remaining);
        vtarget = (vdecel < VMAX) ? vdecel : VMAX;
        if (vtarget < VSTART) vtarget = VSTART;   // floor
      } else {
        vtarget = VMAX;
      }

      // ---- Speed update ----
      unsigned long elapsed = now - m.lastStepTime;
      float dt = elapsed * 1e-6f;
      if (dt > 0.05f) dt = 0.05f;
      if (m.currentSpeed < vtarget) {
        m.currentSpeed += ACCEL * dt;
        if (m.currentSpeed > vtarget) m.currentSpeed = vtarget;
      } else if (m.currentSpeed > vtarget) {
        m.currentSpeed -= ACCEL * dt;
        if (m.currentSpeed < vtarget) m.currentSpeed = vtarget;
      }

      unsigned long interval = (unsigned long)(1e6f / m.currentSpeed);
      if (interval > STEP_INTERVAL_START_US) interval = STEP_INTERVAL_START_US;

      // ---- Pulse ----
      if (elapsed >= interval) {
        digitalWrite(m.pulPin, HIGH);
        delayMicroseconds(PULSE_WIDTH_US);
        digitalWrite(m.pulPin, LOW);
        m.steps += dir;
        m.lastStepTime = now;

        // Reached target immediately?
        if (m.mode == 3 && m.steps == m.target) m.mode = 0;
      }
    }

    // ---- Auto sequence state machine ----
    if (autoState == 1) {
      bool allDone = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].mode != 0) { allDone = false; break; }
      }
      if (allDone) { autoState = 2; pauseStartTime = millis(); }
    }
    else if (autoState == 2) {
      if (millis() - pauseStartTime >= AUTO_PAUSE_MS) {
        int next = currentPoint + 1;
        if (next >= numPoints) {
          autoState = 3;
        } else {
          currentPoint = next;
          for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].target = pointData[currentPoint][i];
            motors[i].mode = (motors[i].steps != motors[i].target) ? 3 : 0;
          }
          autoState = 1;
        }
      }
    }

    // ---- Yield to prevent watchdog reset ----
    if (!anyMoving && autoState == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      lastYield = millis();
    } else if (millis() - lastYield > 500) {
      vTaskDelay(pdMS_TO_TICKS(1));
      lastYield = millis();
    }
  }
}

// ============================================================
//  STATUS JSON
// ============================================================
String buildStatusJson() {
  String s = "{\"x\":";     s += String(motors[0].steps);
  s += ",\"y\":";           s += String(motors[1].steps);
  s += ",\"z\":";           s += String(motors[2].steps);
  s += ",\"autoState\":";   s += String(autoState);
  s += ",\"currentPoint\":";s += String(currentPoint);
  s += ",\"numPoints\":";   s += String(numPoints);
  s += "}";
  return s;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("ESP32 Arduino core: %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Serial.printf("Microsteps: %d  Steps/rev: %d\n", MICROSTEPS, STEPS_PER_REV);
  Serial.printf("Cruise speed: %.0f steps/s  (~%.0f RPM)\n",
                VMAX, VMAX * 60.0f / STEPS_PER_REV);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].pulPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    digitalWrite(motors[i].pulPin, LOW);
    digitalWrite(motors[i].dirPin, LOW);
  }

  xTaskCreatePinnedToCore(MotorControlTask, "MotorTask", 6000, NULL, 2, NULL, 1);

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // Serve UI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send_P(200, "text/html", index_html);
  });

  // Manual jog
  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("id") && r->hasParam("dir")) {
      int id = r->getParam("id")->value().toInt();
      String d = r->getParam("dir")->value();
      if (id >= 0 && id < NUM_MOTORS && (autoState == 0 || autoState == 3)) {
        if (d == "fwd") motors[id].mode = 1;
        else if (d == "rev") motors[id].mode = 2;
        else motors[id].mode = 0;
      }
    }
    r->send(200, "text/plain", "OK");
  });

  // Position/status (JSON)
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send(200, "application/json", buildStatusJson());
  });

  // Reset positions to 0
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *r){
    if (autoState == 0 || autoState == 3) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].steps = 0;
        motors[i].target = 0;
        motors[i].mode = 0;
      }
      autoState = 0;
      currentPoint = -1;
      numPoints = 0;
    }
    r->send(200, "text/plain", "OK");
  });

  // Stop everything
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *r){
    autoState = 0;
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].mode = 0;
    r->send(200, "text/plain", "OK");
  });

  // Start auto sequence:  /auto?pts=100,200,300_400,500,600_...
  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *r){
    if (!r->hasParam("pts")) { r->send(400, "text/plain", "Missing pts"); return; }

    // Stop any running sequence first
    autoState = 0;
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].mode = 0;

    String ptsStr = r->getParam("pts")->value();
    int start = 0;
    numPoints = 0;
    while (start < (int)ptsStr.length() && numPoints < MAX_POINTS) {
      int sep = ptsStr.indexOf('_', start);
      if (sep < 0) sep = ptsStr.length();
      String trip = ptsStr.substring(start, sep);
      int c1 = trip.indexOf(',');
      int c2 = (c1 >= 0) ? trip.indexOf(',', c1 + 1) : -1;
      if (c1 > 0 && c2 > c1) {
        pointData[numPoints][0] = trip.substring(0, c1).toInt();
        pointData[numPoints][1] = trip.substring(c1 + 1, c2).toInt();
        pointData[numPoints][2] = trip.substring(c2 + 1).toInt();
        numPoints++;
      }
      start = sep + 1;
    }

    if (numPoints == 0) { r->send(400, "text/plain", "No valid points"); return; }

    currentPoint = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].target = pointData[0][i];
      motors[i].mode = (motors[i].steps != motors[i].target) ? 3 : 0;
    }
    autoState = 1;
    String resp = "Started sequence: " + String(numPoints) + " points";
    r->send(200, "text/plain", resp);
  });

  server.begin();
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.printf("X:%ld Y:%ld Z:%ld | auto:%d pt:%d/%d\n",
                  motors[0].steps, motors[1].steps, motors[2].steps,
                  autoState, currentPoint + 1, numPoints);
  }
}