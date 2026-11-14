#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// -----------------------------------------------------------------------------
// Pin definitions (X, Y, Z)
// -----------------------------------------------------------------------------
const int PUL_PINS[3] = {14, 26, 33};
const int DIR_PINS[3] = {27, 25, 32};

// -----------------------------------------------------------------------------
// Speed and pulse tuning
// -----------------------------------------------------------------------------
const unsigned long STEP_INTERVAL_START_US = 800;
const unsigned long STEP_INTERVAL_MIN_US = 100;  // Safe speed for 3 simultaneous axes
const unsigned long STEP_INTERVAL_ACCEL_US = 2;
const int PULSE_WIDTH_US = 2;
const unsigned long SOLDER_DWELL_MS = 1500;      // 1.5-second pause at each point

const char* ssid = "SolderMachine_AP";
const char* password = "password123";

AsyncWebServer server(80);
TaskHandle_t MotorTaskHandle = NULL;

// -----------------------------------------------------------------------------
// Shared state variables
// -----------------------------------------------------------------------------
volatile int manualDir[3] = {0, 0, 0};      // 0 = Stop, 1 = Forward, -1 = Reverse
volatile long currentSteps[3] = {0, 0, 0};  // Tracks live X, Y, and Z steps

struct Point {
  long x;
  long y;
  long z;
};

const int MAX_POINTS = 50;
Point autoPoints[MAX_POINTS];
volatile int totalAutoPoints = 0;
volatile int currentPointIdx = 0;

enum AutoState {
  AUTO_IDLE,
  AUTO_MOVE_XY,
  AUTO_MOVE_Z_DOWN,
  AUTO_DWELL,
  AUTO_MOVE_Z_UP
};

volatile AutoState autoState = AUTO_IDLE;

// -----------------------------------------------------------------------------
// Web interface (Manual and Auto tabs)
// -----------------------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>3-Axis Solder Controller</title>

  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 15px;
      background: #f4f4f4;
    }

    .tabs {
      display: flex;
      justify-content: center;
      margin-bottom: 20px;
    }

    .tab-btn {
      padding: 14px 28px;
      font-size: 18px;
      font-weight: bold;
      border: none;
      cursor: pointer;
      background: #ddd;
      color: #333;
    }

    .tab-btn.active {
      background: #2c3e50;
      color: #fff;
    }

    .tab-content {
      display: none;
      max-width: 500px;
      margin: auto;
      background: #fff;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.15);
    }

    .tab-content.active {
      display: block;
    }

    .axis-box {
      border: 1px solid #e0e0e0;
      border-radius: 8px;
      padding: 10px;
      margin-bottom: 12px;
      background: #fafafa;
    }

    .axis-title {
      font-weight: bold;
      font-size: 16px;
      margin-bottom: 8px;
      color: #2c3e50;
    }

    .btn {
      padding: 14px 16px;
      font-size: 14px;
      font-weight: bold;
      color: #fff;
      border: none;
      border-radius: 6px;
      margin: 4px;
      cursor: pointer;
      user-select: none;
    }

    .fwd {
      background: #2ecc71;
    }

    .rev {
      background: #3498db;
    }

    .zero {
      background: #e67e22;
    }

    .action {
      background: #8e44ad;
      width: 100%;
      font-size: 16px;
      margin-top: 10px;
    }

    .stop {
      background: #e74c3c;
      width: 100%;
      font-size: 16px;
      margin-top: 10px;
    }

    .btn:active {
      opacity: 0.7;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 15px;
    }

    th,
    td {
      border: 1px solid #ddd;
      padding: 8px;
      text-align: center;
    }

    th {
      background: #2c3e50;
      color: white;
    }

    input[type="number"] {
      width: 70px;
      padding: 6px;
      text-align: center;
      font-size: 14px;
    }
  </style>
</head>

<body>
  <div class="tabs">
    <button id="tabManualBtn" class="tab-btn active" onclick="openTab('manual')">
      MANUAL
    </button>
    <button id="tabAutoBtn" class="tab-btn" onclick="openTab('auto')">
      AUTO
    </button>
  </div>

  <!-- Manual tab -->
  <div id="manual" class="tab-content active">
    <div class="axis-box">
      <div class="axis-title">X-AXIS MOTOR</div>
      <button class="btn fwd"
              onmousedown="jog(0, 1)"
              onmouseup="jog(0, 0)"
              ontouchstart="jog(0, 1)"
              ontouchend="jog(0, 0)">
        FORWARD
      </button>
      <button class="btn rev"
              onmousedown="jog(0, -1)"
              onmouseup="jog(0, 0)"
              ontouchstart="jog(0, -1)"
              ontouchend="jog(0, 0)">
        REVERSE
      </button>
      <button class="btn zero" onclick="jog(0, 2)">ZERO X</button>
    </div>

    <div class="axis-box">
      <div class="axis-title">Y-AXIS MOTOR</div>
      <button class="btn fwd"
              onmousedown="jog(1, 1)"
              onmouseup="jog(1, 0)"
              ontouchstart="jog(1, 1)"
              ontouchend="jog(1, 0)">
        FORWARD
      </button>
      <button class="btn rev"
              onmousedown="jog(1, -1)"
              onmouseup="jog(1, 0)"
              ontouchstart="jog(1, -1)"
              ontouchend="jog(1, 0)">
        REVERSE
      </button>
      <button class="btn zero" onclick="jog(1, 2)">ZERO Y</button>
    </div>

    <div class="axis-box">
      <div class="axis-title">Z-AXIS MOTOR</div>
      <button class="btn fwd"
              onmousedown="jog(2, 1)"
              onmouseup="jog(2, 0)"
              ontouchstart="jog(2, 1)"
              ontouchend="jog(2, 0)">
        FORWARD
      </button>
      <button class="btn rev"
              onmousedown="jog(2, -1)"
              onmouseup="jog(2, 0)"
              ontouchstart="jog(2, -1)"
              ontouchend="jog(2, 0)">
        REVERSE
      </button>
      <button class="btn zero" onclick="jog(2, 2)">ZERO Z</button>
    </div>
  </div>

  <!-- Auto tab -->
  <div id="auto" class="tab-content">
    <label><b>Number of Solder Points (N):</b></label>
    <br><br>

    <input type="number" id="numPoints" min="1" max="50" value="3">
    <button class="btn rev" onclick="generateTable()">Set Points</button>

    <div id="tableContainer"></div>

    <button class="btn action" onclick="startAuto()">
      START AUTO SEQUENCE
    </button>
    <button class="btn stop" onclick="stopAll()">
      EMERGENCY STOP
    </button>
  </div>

  <script>
    function openTab(tabName) {
      document.querySelectorAll('.tab-content').forEach((el) => {
        el.classList.remove('active');
      });

      document.querySelectorAll('.tab-btn').forEach((el) => {
        el.classList.remove('active');
      });

      document.getElementById(tabName).classList.add('active');

      const buttonId = tabName === 'manual' ? 'tabManualBtn' : 'tabAutoBtn';
      document.getElementById(buttonId).classList.add('active');
    }

    function jog(axis, dir) {
      fetch(`/manual?axis=${axis}&dir=${dir}`);
    }

    function generateTable() {
      const n = parseInt(document.getElementById('numPoints').value);

      if (isNaN(n) || n < 1 || n > 50) {
        return alert('Enter a number between 1 and 50');
      }

      let html = '<table><tr><th>Point</th><th>X (Steps)</th>' +
                 '<th>Y (Steps)</th><th>Z (Steps)</th></tr>';

      for (let i = 1; i <= n; i++) {
        html += `<tr>
          <td><b>P${i}</b></td>
          <td><input type="number" id="x_${i}" value="0"></td>
          <td><input type="number" id="y_${i}" value="0"></td>
          <td><input type="number" id="z_${i}" value="0"></td>
        </tr>`;
      }

      html += '</table>';
      document.getElementById('tableContainer').innerHTML = html;
    }

    function startAuto() {
      const n = parseInt(document.getElementById('numPoints').value);
      let payload = '';

      for (let i = 1; i <= n; i++) {
        const xElement = document.getElementById(`x_${i}`);

        if (!xElement) {
          return alert("Click 'Set Points' first!");
        }

        const x = xElement.value || 0;
        const y = document.getElementById(`y_${i}`).value || 0;
        const z = document.getElementById(`z_${i}`).value || 0;

        payload += `${x},${y},${z};`;
      }

      fetch(`/auto?data=${encodeURIComponent(payload)}`);
    }

    function stopAll() {
      fetch('/stop');
    }

    // Generate the default 3 rows when the page loads.
    window.onload = generateTable;
  </script>
</body>
</html>
)rawliteral";

// -----------------------------------------------------------------------------
// Helper: generate a single step
// -----------------------------------------------------------------------------
void stepAxis(int axis, int dir) {
  digitalWrite(DIR_PINS[axis], (dir > 0) ? HIGH : LOW);
  digitalWrite(PUL_PINS[axis], HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(PUL_PINS[axis], LOW);

  currentSteps[axis] += dir;
}

// -----------------------------------------------------------------------------
// Core 1: real-time motion controller
// -----------------------------------------------------------------------------
void MotorControlTask(void* pvParameters) {
  unsigned long lastStepTime[3] = {0, 0, 0};
  unsigned long stepInterval[3] = {
    STEP_INTERVAL_START_US,
    STEP_INTERVAL_START_US,
    STEP_INTERVAL_START_US
  };
  unsigned long dwellStartTime = 0;

  for (;;) {
    const unsigned long now = micros();

    // 1. Automatic mode execution
    if (autoState != AUTO_IDLE) {
      if (currentPointIdx >= totalAutoPoints) {
        autoState = AUTO_IDLE;  // All points completed.
      } else {
        const Point target = autoPoints[currentPointIdx];

        if (autoState == AUTO_MOVE_XY) {
          const bool xDone = (currentSteps[0] == target.x);
          const bool yDone = (currentSteps[1] == target.y);

          if (!xDone && (now - lastStepTime[0] >= STEP_INTERVAL_MIN_US)) {
            stepAxis(0, (target.x > currentSteps[0]) ? 1 : -1);
            lastStepTime[0] = now;
          }

          if (!yDone && (now - lastStepTime[1] >= STEP_INTERVAL_MIN_US)) {
            stepAxis(1, (target.y > currentSteps[1]) ? 1 : -1);
            lastStepTime[1] = now;
          }

          if (xDone && yDone) {
            autoState = AUTO_MOVE_Z_DOWN;
          }
        } else if (autoState == AUTO_MOVE_Z_DOWN) {
          if (currentSteps[2] != target.z) {
            if (now - lastStepTime[2] >= STEP_INTERVAL_MIN_US) {
              stepAxis(2, (target.z > currentSteps[2]) ? 1 : -1);
              lastStepTime[2] = now;
            }
          } else {
            dwellStartTime = millis();
            autoState = AUTO_DWELL;
          }
        } else if (autoState == AUTO_DWELL) {
          if (millis() - dwellStartTime >= SOLDER_DWELL_MS) {
            autoState = AUTO_MOVE_Z_UP;
          }
        } else if (autoState == AUTO_MOVE_Z_UP) {
          if (currentSteps[2] != 0) {
            // Retract Z to the 0-step clearance position.
            if (now - lastStepTime[2] >= STEP_INTERVAL_MIN_US) {
              stepAxis(2, (0 > currentSteps[2]) ? 1 : -1);
              lastStepTime[2] = now;
            }
          } else {
            currentPointIdx++;
            autoState = AUTO_MOVE_XY;  // Move to the next point.
          }
        }
      }
    }
    // 2. Manual mode execution
    else {
      bool anyMotorMoving = false;

      for (int i = 0; i < 3; i++) {
        if (manualDir[i] != 0) {
          anyMotorMoving = true;

          if (now - lastStepTime[i] >= stepInterval[i]) {
            stepAxis(i, manualDir[i]);
            lastStepTime[i] = now;

            if (stepInterval[i] > STEP_INTERVAL_MIN_US) {
              const unsigned long remaining =
                  stepInterval[i] - STEP_INTERVAL_MIN_US;

              stepInterval[i] -=
                  (remaining > STEP_INTERVAL_ACCEL_US)
                      ? STEP_INTERVAL_ACCEL_US
                      : remaining;
            }
          }
        } else {
          stepInterval[i] = STEP_INTERVAL_START_US;
          digitalWrite(PUL_PINS[i], LOW);
        }
      }

      if (!anyMotorMoving) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Feed the RTOS watchdog while idle.
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Core 0: Wi-Fi and web server
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(PUL_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(PUL_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }

  xTaskCreatePinnedToCore(
      MotorControlTask,
      "MotorTask",
      6000,
      NULL,
      1,
      &MotorTaskHandle,
      1
  );

  WiFi.softAP(ssid, password);
  Serial.print("AP Ready. IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  // Manual jog and zero endpoint
  server.on("/manual", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("axis") && request->hasParam("dir")) {
      const int axis = request->getParam("axis")->value().toInt();
      const int dir = request->getParam("dir")->value().toInt();

      if (axis >= 0 && axis < 3) {
        autoState = AUTO_IDLE;  // Manual input overrides automatic mode.

        if (dir == 2) {
          currentSteps[axis] = 0;
          Serial.printf("Axis %d Zeroed.\n", axis);
        } else {
          manualDir[axis] = dir;
        }
      }
    }

    request->send(200, "text/plain", "OK");
  });

  // Automatic sequence parser endpoint
  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("data")) {
      const String data = request->getParam("data")->value();
      int count = 0;
      int startIdx = 0;

      while (startIdx < data.length() && count < MAX_POINTS) {
        const int semiIdx = data.indexOf(';', startIdx);
        if (semiIdx == -1) {
          break;
        }

        const String row = data.substring(startIdx, semiIdx);
        const int comma1 = row.indexOf(',');
        const int comma2 = row.indexOf(',', comma1 + 1);

        if (comma1 != -1 && comma2 != -1) {
          autoPoints[count].x = row.substring(0, comma1).toInt();
          autoPoints[count].y = row.substring(comma1 + 1, comma2).toInt();
          autoPoints[count].z = row.substring(comma2 + 1).toInt();
          count++;
        }

        startIdx = semiIdx + 1;
      }

      if (count > 0) {
        totalAutoPoints = count;
        currentPointIdx = 0;
        autoState = AUTO_MOVE_XY;

        Serial.printf(
            "Started Auto Sequence with %d points.\n",
            totalAutoPoints
        );
      }
    }

    request->send(200, "text/plain", "OK");
  });

  // Emergency stop endpoint
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* request) {
    autoState = AUTO_IDLE;

    for (int i = 0; i < 3; i++) {
      manualDir[i] = 0;
    }

    Serial.println("EMERGENCY STOP TRIGGERED.");
    request->send(200, "text/plain", "STOPPED");
  });

  server.begin();
}

void loop() {
  static long lastX = 0;
  static long lastY = 0;
  static long lastZ = 0;
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 250) {
    if (currentSteps[0] != lastX ||
        currentSteps[1] != lastY ||
        currentSteps[2] != lastZ) {
      Serial.printf(
          "Pos -> X: %ld | Y: %ld | Z: %ld\n",
          currentSteps[0],
          currentSteps[1],
          currentSteps[2]
      );

      lastX = currentSteps[0];
      lastY = currentSteps[1];
      lastZ = currentSteps[2];
    }

    lastPrint = millis();
  }
}
