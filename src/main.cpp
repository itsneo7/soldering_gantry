#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const int PUL_PIN = 14;
const int DIR_PIN = 27;

// 1/32 microstepping = 6400 steps/rev
const int MICROSTEPS = 32;
const int STEPS_PER_REV = 200 * MICROSTEPS;

// ---- SPEED TUNING (microseconds between rising edges) ----
// At 6400 steps/rev:
//   500 µs →  31 RPM  (startup)
//    62 µs → 151 RPM
//    40 µs → 234 RPM
//    31 µs → 300 RPM   <-- practical software limit
// Slower, higher torque acceleration limits
// ---- OPTIMIZED HIGH SPEED TUNING ----
const unsigned long STEP_INTERVAL_START_US = 800; // Safe startup frequency
const unsigned long STEP_INTERVAL_MIN_US   = 70;  // Increased speed (Ramped up from 200 µs to 70 µs)
const unsigned long STEP_INTERVAL_ACCEL_US = 1;   // Smoother, gentler acceleration increment
const int PULSE_WIDTH_US = 2;                     // Kept at 2 µs for optocoupler safety


const char* ssid = "SolderMachine_AP";
const char* password = "password123";

AsyncWebServer server(80);

enum MotorState { STOPPED, MOVING_FORWARD, MOVING_REVERSE };
volatile MotorState currentState = STOPPED;
TaskHandle_t MotorTaskHandle = NULL;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Solder Machine Manual Control</title>
<style>
body { font-family: Arial; text-align: center; margin-top: 50px; background:#f4f4f4; }
.btn { display:inline-block; width:200px; padding:20px; font-size:20px; font-weight:bold;
       color:#fff; border:none; border-radius:10px; margin:15px; cursor:pointer; user-select:none; }
.forward { background:#2ecc71; } .reverse { background:#3498db; }
.btn:active { opacity:0.7; }
</style></head><body>
<h2>Manual Axis Control</h2>
<button class="btn forward"
  onmousedown="sendCmd('forward')" onmouseup="sendCmd('stop')"
  ontouchstart="sendCmd('forward')" ontouchend="sendCmd('stop')">FORWARD</button><br>
<button class="btn reverse"
  onmousedown="sendCmd('reverse')" onmouseup="sendCmd('stop')"
  ontouchstart="sendCmd('reverse')" ontouchend="sendCmd('stop')">REVERSE</button>
<script>function sendCmd(a){fetch('/motor?dir='+a);}</script>
</body></html>
)rawliteral";

void MotorControlTask(void * pvParameters) {
  unsigned long lastStepTime = 0;
  unsigned long stepInterval = STEP_INTERVAL_START_US;

  for (;;) {
    if (currentState != STOPPED) {
      unsigned long now = micros();
      if (now - lastStepTime >= stepInterval) {
        // 1. Trigger the step edge
        digitalWrite(PUL_PIN, HIGH);
        lastStepTime = now;
        
        // 2. Keep pulse HIGH just long enough for the TB6600 optocoupler to see it (2 microseconds)
        delayMicroseconds(2); 
        digitalWrite(PUL_PIN, LOW); 
        
        // 3. Smooth Acceleration Profile
        if (stepInterval > STEP_INTERVAL_MIN_US) {
          unsigned long remaining = stepInterval - STEP_INTERVAL_MIN_US;
          stepInterval -= (remaining > STEP_INTERVAL_ACCEL_US) ? STEP_INTERVAL_ACCEL_US : remaining;
        }
      }
    } else {
      stepInterval = STEP_INTERVAL_START_US;
      digitalWrite(PUL_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(1)); // Feed the RTOS watchdog
    }
  }
}



void setup() {
  Serial.begin(115200);
  Serial.printf("ESP32 Arduino core: %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  xTaskCreatePinnedToCore(MotorControlTask, "MotorTask", 4000, NULL, 1, &MotorTaskHandle, 1);

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("dir")) {
      String d = r->getParam("dir")->value();
      if      (d == "forward") { digitalWrite(DIR_PIN, HIGH); currentState = MOVING_FORWARD; }
      else if (d == "reverse") { digitalWrite(DIR_PIN, LOW);  currentState = MOVING_REVERSE; }
      else                     { currentState = STOPPED; }
    }
    r->send(200, "text/plain", "OK");
  });
  server.begin();
}

void loop() {}