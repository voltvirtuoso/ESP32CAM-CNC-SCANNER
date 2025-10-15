#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "SD_MMC.h"

// ======= USER CONFIG =======
const char* ssid = "HW-01K";
const char* password = "aeioq8912";
const uint16_t HTTP_PORT = 80;
const uint16_t STREAM_PORT = 81;  // Separate port for streaming
float scanFeedRate = 150.0;  // Default feedrate
// ===========================

// GRBL Serial Communication (to Arduino) - Using Serial (UART0)
#define GRBL_SERIAL Serial
const uint32_t GRBL_BAUD = 115200;
// ===========================

// Camera model 
// #define CAMERA_MODEL_ESP32S3_EYE
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM       5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#elif defined(CAMERA_MODEL_ESP32S3_EYE)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    15
  #define SIOD_GPIO_NUM    4
  #define SIOC_GPIO_NUM    5
  #define Y9_GPIO_NUM     16
  #define Y8_GPIO_NUM     17
  #define Y7_GPIO_NUM     18
  #define Y6_GPIO_NUM     12
  #define Y5_GPIO_NUM     10
  #define Y4_GPIO_NUM      8
  #define Y3_GPIO_NUM      9
  #define Y2_GPIO_NUM     11
  #define VSYNC_GPIO_NUM   6
  #define HREF_GPIO_NUM    7
  #define PCLK_GPIO_NUM    13
#endif

WebServer server(HTTP_PORT);
WebServer streamServer(STREAM_PORT);

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

String grblResponse = "";
String currentPosition = "X:0.00 Y:0.00 Z:0.00";
String machineState = "Idle";

// Spatial scan variables
bool isScanning = false;
int scanXCount = 0;
int scanYCount = 0;
int totalScanPoints = 0;
int currentScanPoint = 0;
float scanXStart = 0.0;
float scanYStart = 0.0;
float scanXEnd = 100.0;
float scanYEnd = 100.0;
float scanStepSize = 5.0;
String scanStatus = "Ready";

String currentScanFolder = "";
volatile int frameCounter = 0;
volatile bool captureActive = false;

// Non-blocking scan state
enum ScanState { 
  SCAN_STARTING, 
  MOVE_TO_ROW, 
  TRAVERSE_ROW, 
  NEXT_ROW, 
  SCAN_COMPLETE 
};
ScanState currentScanState = SCAN_COMPLETE;
int currentRow = 0;
bool goingRight = true;

// Send command to GRBL and wait for response
String sendToGRBL(String cmd, uint32_t timeout = 1000) {
  cmd.trim();
  // Debug output disabled since we're using Serial for GRBL
  
  GRBL_SERIAL.println(cmd);
  
  String response = "";
  uint32_t start = millis();
  
  while (millis() - start < timeout) {
    if (GRBL_SERIAL.available()) {
      char c = GRBL_SERIAL.read();
      response += c;
      if (c == '\n') {
        break;
      }
    }
    delay(1);
  }
  
  response.trim();
  
  return response;
}

// Request GRBL status - Parse WPos instead of MPos
void updateGRBLStatus() {
  GRBL_SERIAL.write('?');
  
  uint32_t start = millis();
  String status = "";
  
  while (millis() - start < 300) {
    if (GRBL_SERIAL.available()) {
      char c = GRBL_SERIAL.read();
      status += c;
    }
  }

  if (status.length() == 0) {
    return;
  }
  
  int startBracket = status.indexOf('<');
  int endBracket = status.indexOf('>');
  
  if (startBracket == -1 || endBracket == -1) {
    return;
  }
  
  String statusPart = status.substring(startBracket + 1, endBracket);
  
  // Extract machine state
  int firstComma = statusPart.indexOf(',');
  if (firstComma != -1) {
    machineState = statusPart.substring(0, firstComma);
  }
  
  // Try WPos first, fall back to MPos
  int posStart = statusPart.indexOf("WPos:");
  bool isWPos = true;
  if (posStart == -1) {
    posStart = statusPart.indexOf("MPos:");
    isWPos = false;
  }
  
  if (posStart != -1) {
    posStart += 5; // Skip "WPos:" or "MPos:"
    
    // Find end: look for ">" or ",M" or ",W"
    int posEnd = statusPart.indexOf('>', posStart);
    int altEnd = statusPart.indexOf(isWPos ? ",M" : ",W", posStart + 1);
    if (altEnd != -1 && (posEnd == -1 || altEnd < posEnd)) {
      posEnd = altEnd;
    }
    if (posEnd == -1) {
      posEnd = statusPart.length();
    }
    
    String pos = statusPart.substring(posStart, posEnd);
    
    // Parse X,Y,Z
    int comma1 = pos.indexOf(',');
    int comma2 = pos.indexOf(',', comma1 + 1);
    
    if (comma1 > 0 && comma2 > comma1) {
      float x = pos.substring(0, comma1).toFloat();
      float y = pos.substring(comma1 + 1, comma2).toFloat();
      float z = pos.substring(comma2 + 1).toFloat();
      
      currentPosition = "X:" + String(x, 2) + " Y:" + String(y, 2) + " Z:" + String(z, 2);
    }
  }
}

// Initialize GRBL with configuration commands
void initGRBL() {
  delay(2000); // Wait for GRBL to boot
  
  // Send soft reset
  GRBL_SERIAL.write(0x18);
  delay(1000);
  
  // Clear any startup messages
  while (GRBL_SERIAL.available()) {
    GRBL_SERIAL.read();
  }
  
  // Request status
  updateGRBLStatus();
}

// Init camera
bool initCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  
  // Set initial sensor settings
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
    s->set_vflip(s, 0);          // 0 = disable, 1 = enable
  }
  
  return true;
}

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
	<head>
		<meta name='viewport' content='width=device-width,initial-scale=1'/>
		<title>ESP32-CAM GRBL</title>
<style>
:root {
  --bg-primary: #1e1e1e;
  --bg-secondary: #2d2d2d;
  --bg-tertiary: #3d3d3d;
  --text-primary: #e0e0e0;
  --text-secondary: #a0a0a0;
  --accent-primary: #4dabf7;
  --accent-secondary: #9c36b5;
  --success: #51cf66;
  --warning: #ffd43b;
  --danger: #ff6b6b;
  --border-radius: 8px;
  --box-shadow: 0 4px 15px rgba(0,0,0,0.3);
}

body {
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
  background-color: var(--bg-primary);
  color: var(--text-primary);
  margin: 0;
  padding: 0;
  display: flex;
  min-height: 100vh;
}

.container {
  display: flex;
  width: 100%;
  flex: 1;
}

.sidebar {
  width: 250px;
  background-color: var(--bg-secondary);
  padding: 20px 0;
  box-shadow: var(--box-shadow);
  z-index: 100;
  display: flex;
  flex-direction: column;
}

.sidebar-header {
  padding: 0 20px 20px;
  border-bottom: 1px solid var(--bg-tertiary);
  margin-bottom: 20px;
}

.sidebar-menu {
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.menu-item {
  padding: 12px 20px;
  color: var(--text-primary);
  text-decoration: none;
  background: transparent;
  border: none;
  text-align: left;
  cursor: pointer;
  font-size: 16px;
  transition: all 0.2s;
  border-left: 3px solid transparent;
}

.menu-item:hover {
  background-color: var(--bg-tertiary);
}

.menu-item.active {
  background-color: var(--bg-tertiary);
  border-left: 3px solid var(--accent-primary);
}

.main-content {
  flex: 1;
  padding: 20px;
  overflow-y: auto;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
  padding-bottom: 10px;
  border-bottom: 1px solid var(--bg-tertiary);
}

.header h1 {
  margin: 0;
  font-size: 24px;
}

.content-section {
  background-color: var(--bg-secondary);
  border-radius: var(--border-radius);
  padding: 20px;
  margin-bottom: 20px;
  box-shadow: var(--box-shadow);
}

.feed-container {
  width: 100%;
  text-align: center;
  margin-bottom: 20px;
}

.camera-feed {
  width: 100%;
  max-width: 600px;
  border-radius: var(--border-radius);
  box-shadow: var(--box-shadow);
  margin: 0 auto;
  display: block;
}

.controls-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  margin-top: 20px;
}

.control-group {
  background-color: var(--bg-tertiary);
  border-radius: var(--border-radius);
  padding: 15px;
}

.control-group h3 {
  margin-top: 0;
  margin-bottom: 15px;
  color: var(--accent-primary);
  border-bottom: 1px solid var(--bg-secondary);
  padding-bottom: 8px;
}

.axis-controls {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 10px;
  margin-bottom: 15px;
}

.btn {
  padding: 12px;
  border: none;
  border-radius: var(--border-radius);
  cursor: pointer;
  transition: all 0.2s;
  font-weight: bold;
  font-size: 16px;
}

.x-btn { background: linear-gradient(to bottom, #ff6b6b, #d32f2f); color: white; }
.y-btn { background: linear-gradient(to bottom, #4ecdc4, #26a69a); color: white; }
.z-btn { background: linear-gradient(to bottom, #4dabf7, #2196f3); color: white; }
.home-btn { background: linear-gradient(to bottom, #ffd43b, #f9c22e); color: #333; }
.stop-btn { background: linear-gradient(to bottom, #ff6b6b, #d32f2f); color: white; }
.reset-btn { background: linear-gradient(to bottom, #51cf66, #37b24d); color: white; }
.scan-btn { background: linear-gradient(to bottom, #9c36b5, #7b2cbf); color: white; }

.btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
.btn:active { transform: translateY(0); }
.btn:disabled { opacity: 0.5; cursor: not-allowed; }

.quick-actions {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
  gap: 10px;
  margin-top: 15px;
}

.form-group {
  margin-bottom: 15px;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  font-weight: bold;
  color: var(--text-secondary);
}

.form-group input,
.form-group select {
  width: 100%;
  padding: 10px;
  border-radius: var(--border-radius);
  border: 1px solid var(--bg-tertiary);
  background-color: var(--bg-primary);
  color: var(--text-primary);
  box-sizing: border-box;
}

.scan-controls {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 15px;
}

.scan-progress {
  width: 100%;
  height: 20px;
  background-color: var(--bg-primary);
  border-radius: 10px;
  overflow: hidden;
  margin-top: 10px;
}

.scan-progress-bar {
  height: 100%;
  background: linear-gradient(to right, #4ecdc4, #26a69a);
  width: 0%;
  transition: width 0.3s;
}

.scan-status {
  margin-top: 10px;
  font-weight: bold;
  color: var(--warning);
  text-align: center;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 15px;
  margin-top: 15px;
}

.status-item {
  background-color: var(--bg-primary);
  padding: 12px;
  border-radius: var(--border-radius);
  border-left: 4px solid var(--accent-primary);
}

.status-label {
  font-size: 14px;
  color: var(--text-secondary);
  margin-bottom: 5px;
}

.status-value {
  font-weight: bold;
  color: var(--warning);
  font-size: 16px;
}

.settings-content {
  display: none;
}

.settings-content.active {
  display: block;
}

@media (max-width: 768px) {
  .container {
    flex-direction: column;
  }
  
  .sidebar {
    width: 100%;
    height: auto;
  }
  
  .axis-controls {
    grid-template-columns: repeat(3, 1fr);
  }
  
  .controls-grid {
    grid-template-columns: 1fr;
  }
  
  .scan-controls {
    grid-template-columns: 1fr;
  }
}
</style>
	</head>
	<body>
    <div class="container">
      <!-- Sidebar Menu -->
      <div class="sidebar">
        <div class="sidebar-header">
          <h2>ESP32-CAM GRBL</h2>
        </div>
        <div class="sidebar-menu">
          <button class="menu-item active" data-target="main">Main Controls</button>
          <button class="menu-item" data-target="camera">Camera Settings</button>
          <button class="menu-item" data-target="motion">Motion Settings</button>
          <button class="menu-item" data-target="scan">Scan Settings</button>
          <button class="menu-item" data-target="system">System Info</button>
          <button class="menu-item" data-target="files">File Browser</button>
        </div>
      </div>

      <!-- Main Content -->
      <div class="main-content">
        <div class="header">
          <h1>CNC Controller</h1>
          <div>IP: <span id="ipAddress">--</span></div>
        </div>

        <!-- Main Controls Section -->
        <div id="main" class="settings-content active">
          <!-- System Status -->
          <div class="content-section">
            <h3>System Status</h3>
            <div class="status-grid">
              <div class="status-item">
                <div class="status-label">Machine State</div>
                <div class="status-value" id="state">--</div>
              </div>
              <div class="status-item">
                <div class="status-label">Position</div>
                <div class="status-value" id="pos">X:-- Y:-- Z:--</div>
              </div>
              <div class="status-item">
                <div class="status-label">Last Command</div>
                <div class="status-value" id="last">None</div>
              </div>
              <div class="status-item">
                <div class="status-label">Response</div>
                <div class="status-value" id="resp">--</div>
              </div>
            </div>
          </div>

          <!-- Camera Feed -->
          <div class="feed-container">
            <img id="stream" class="camera-feed" alt="Camera Feed"/>
          </div>

          <!-- Motion Controls -->
          <div class="content-section">
            <h3>Motion Control</h3>
            <div class="axis-controls">
              <button class="btn y-btn" onclick="jog('Y','+')">Y+</button>
              <button class="btn z-btn" onclick="jog('Z','+')">Z+</button>
              <button class="btn y-btn" onclick="jog('Y','-')">Y-</button>
              <button class="btn x-btn" onclick="jog('X','-')">X-</button>
              <button class="btn stop-btn" onclick="emergency()">STOP</button>
              <button class="btn x-btn" onclick="jog('X','+')">X+</button>
              <button class="btn z-btn" onclick="jog('Z','-')">Z-</button>
              <button class="btn home-btn" onclick="home()">HOME</button>
              <button class="btn home-btn" onclick="probe()">PROBE</button>
            </div>

            <div class="form-group">
              <label>Step Size (mm):</label>
              <select id="stepSize">
                <option value="0.1">0.1 mm</option>
                <option value="1" selected>1 mm</option>
                <option value="5">5 mm</option>
                <option value="10">10 mm</option>
                <option value="50">50 mm</option>
                <option value="100">100 mm</option>
              </select>
            </div>

            <div class="form-group">
              <label>Feed Rate (mm/min):</label>
              <input type="number" id="feedRate" value="1000" min="10" max="5000" step="100">
            </div>

            <div class="quick-actions">
              <button class="btn reset-btn" onclick="resetZero('all')">Zero All</button>
              <button class="btn reset-btn" onclick="resetZero('X')">Zero X</button>
              <button class="btn reset-btn" onclick="resetZero('Y')">Zero Y</button>
              <button class="btn reset-btn" onclick="resetZero('Z')">Zero Z</button>
              <button class="btn home-btn" onclick="unlock()">Unlock</button>
              <button class="btn home-btn" onclick="getStatus()">Status</button>
            </div>
          </div>

          <!-- Spatial Scan -->
          <div class="content-section">
            <h3>Spatial Scan</h3>
            <div class="scan-controls">
              <div class="form-group">
                <label>X Start (mm):</label>
                <input type="number" id="scanXStart" value="0" step="0.1">
              </div>
              <div class="form-group">
                <label>X End (mm):</label>
                <input type="number" id="scanXEnd" value="100" step="0.1">
              </div>
              <div class="form-group">
                <label>Y Start (mm):</label>
                <input type="number" id="scanYStart" value="0" step="0.1">
              </div>
              <div class="form-group">
                <label>Y End (mm):</label>
                <input type="number" id="scanYEnd" value="100" step="0.1">
              </div>
              <div class="form-group">
                <label>Step Size (mm):</label>
                <input type="number" id="scanStep" value="5" step="0.1">
              </div>
              <div class="form-group">
                <label>Scan Feed Rate (mm/min):</label>
                <input type="number" id="scanFeedRate" value="150" min="10" max="1000" step="10">
              </div>
            </div>
            <div style="display:flex;gap:10px;margin-top:15px">
              <button class="btn scan-btn" onclick="startScan()">Start Scan</button>
              <button class="btn stop-btn" onclick="stopScan()">Stop Scan</button>
            </div>
            <div class="scan-progress">
              <div id="scanProgressBar" class="scan-progress-bar"></div>
            </div>
            <div class="scan-status" id="scanStatus">Ready</div>
          </div>
        </div>

        <!-- Camera Settings Section -->
        <div id="camera" class="settings-content">
          <div class="content-section">
            <h3>Camera Settings</h3>
            <div class="controls-grid">
              <div class="control-group">
                <h3>Resolution</h3>
                <div class="form-group">
                  <label>Select Resolution:</label>
                  <select id="resolution" onchange="setResolution()">
                    <option value="3">QVGA (320x240)</option>
                    <option value="5">VGA (640x480)</option>
                    <option value="6" selected>SVGA (800x600)</option>
                    <option value="7">XGA (1024x768)</option>
                    <option value="8">HD (1280x720)</option>
                    <option value="10">UXGA (1600x1200)</option>
                  </select>
                </div>
              </div>
              
              <div class="control-group">
                <h3>Quality</h3>
                <div class="form-group">
                  <label>Quality: <span id="qval">10</span></label>
                  <input type="range" min="5" max="63" value="10" id="quality" oninput="setQuality(this.value)">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Brightness</h3>
                <div class="form-group">
                  <label>Brightness: <span id="bval">0</span></label>
                  <input type="range" min="-2" max="2" value="0" id="brightness" oninput="setBrightness(this.value)">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Contrast</h3>
                <div class="form-group">
                  <label>Contrast: <span id="cval">0</span></label>
                  <input type="range" min="-2" max="2" value="0" id="contrast" oninput="setContrast(this.value)">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Flip Options</h3>
                <div style="display:flex;gap:10px;margin-top:10px">
                  <button class="btn home-btn" onclick="toggleFlip('h')">H-Flip</button>
                  <button class="btn home-btn" onclick="toggleFlip('v')">V-Flip</button>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- Motion Settings Section -->
        <div id="motion" class="settings-content">
          <div class="content-section">
            <h3>Motion Settings</h3>
            <div class="controls-grid">
              <div class="control-group">
                <h3>Default Feed Rates</h3>
                <div class="form-group">
                  <label>XY Feed Rate (mm/min):</label>
                  <input type="number" id="xyFeedRate" value="1000" min="10" max="5000" step="100">
                </div>
                <div class="form-group">
                  <label>Z Feed Rate (mm/min):</label>
                  <input type="number" id="zFeedRate" value="500" min="10" max="2000" step="100">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Step Increments</h3>
                <div class="form-group">
                  <label>Small Step (mm):</label>
                  <input type="number" id="smallStep" value="0.1" min="0.01" max="1" step="0.01">
                </div>
                <div class="form-group">
                  <label>Large Step (mm):</label>
                  <input type="number" id="largeStep" value="10" min="1" max="100" step="1">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Speed Limits</h3>
                <div class="form-group">
                  <label>Max XY Speed (mm/min):</label>
                  <input type="number" id="maxXYSpeed" value="3000" min="100" max="10000" step="100">
                </div>
                <div class="form-group">
                  <label>Max Z Speed (mm/min):</label>
                  <input type="number" id="maxZSpeed" value="1000" min="50" max="3000" step="50">
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- Scan Settings Section -->
        <div id="scan" class="settings-content">
          <div class="content-section">
            <h3>Scan Settings</h3>
            <div class="controls-grid">
              <div class="control-group">
                <h3>Scan Parameters</h3>
                <div class="form-group">
                  <label>Default Scan Feed Rate (mm/min):</label>
                  <input type="number" id="defaultScanFeedRate" value="150" min="10" max="1000" step="10">
                </div>
                <div class="form-group">
                  <label>Default Step Size (mm):</label>
                  <input type="number" id="defaultStepSize" value="5" min="0.1" max="20" step="0.1">
                </div>
              </div>
              
              <div class="control-group">
                <h3>Scan Limits</h3>
                <div class="form-group">
                  <label>Max Scan Area X (mm):</label>
                  <input type="number" id="maxScanX" value="200" min="10" max="1000" step="10">
                </div>
                <div class="form-group">
                  <label>Max Scan Area Y (mm):</label>
                  <input type="number" id="maxScanY" value="200" min="10" max="1000" step="10">
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- System Info Section -->
        <div id="system" class="settings-content">
          <div class="content-section">
            <h3>System Information</h3>
            <div class="status-grid">
              <div class="status-item">
                <div class="status-label">Firmware Version</div>
                <div class="status-value">v1.0.0</div>
              </div>
              <div class="status-item">
                <div class="status-label">ESP32 Core</div>
                <div class="status-value">FreeRTOS</div>
              </div>
              <div class="status-item">
                <div class="status-label">Uptime</div>
                <div class="status-value" id="uptime">--</div>
              </div>
              <div class="status-item">
                <div class="status-label">WiFi Signal</div>
                <div class="status-value" id="wifiSignal">-- dBm</div>
              </div>
              <div class="status-item">
                <div class="status-label">CPU Usage</div>
                <div class="status-value" id="cpuUsage">--%</div>
              </div>
              <div class="status-item">
                <div class="status-label">Memory</div>
                <div class="status-value" id="memory">--%</div>
              </div>
            </div>
          </div>
        </div>

        <!-- File Browser Section -->
        <div id="files" class="settings-content">
          <div class="content-section">
            <h3>File Browser</h3>
            <div id="fileList"></div>
          </div>
        </div>
      </div>
    </div>

		<script>
			let hflip=0,vflip=0;
      let currentMenu = 'main';
      
      // Set camera stream URL
      document.getElementById('stream').src=window.location.protocol+'//'+window.location.hostname+':81/stream';
      
      // Get IP address
      fetch('/ip').then(r => r.text()).then(ip => {
        document.getElementById('ipAddress').textContent = ip;
      });

      // Menu navigation
      document.querySelectorAll('.menu-item').forEach(item => {
        item.addEventListener('click', () => {
          document.querySelectorAll('.menu-item').forEach(i => i.classList.remove('active'));
          item.classList.add('active');
          
          const target = item.getAttribute('data-target');
          document.querySelectorAll('.settings-content').forEach(content => {
            content.classList.remove('active');
          });
          document.getElementById(target).classList.add('active');
          currentMenu = target;
            if (target === 'files') {
            loadFiles();
          }
        });
      });

			function api(endpoint,params=''){
				const url=endpoint+(params?'?'+params:'');
				return fetch(url).then(r=>r.text()).catch(e=>{console.error(e);return'ERROR';});
			}

			function jog(axis,dir){
				const step=document.getElementById('stepSize').value;
				const feed=document.getElementById('feedRate').value;
				const cmd='G21G91G1'+axis+dir+step+'F'+feed;
				send(cmd);
			}

			function home(){send('$H');}
			function unlock(){send('$X');}
			function emergency(){send('\x18');}  // Ctrl-X soft reset
			function probe(){send('G38.2 Z-20 F50');}

			function resetZero(axis){
				if(axis==='all'){
					send('G10 L20 P0 X0 Y0 Z0');
				}else{
					send('G10 L20 P0 '+axis+'0');
				}
				setTimeout(getStatus,500);
			}

			function preset(cmd){send(cmd);}

			function send(cmd){
				document.getElementById('last').textContent=cmd;
				document.getElementById('state').textContent='Sending...';
				api('/cmd','c='+encodeURIComponent(cmd)).then(resp=>{
					document.getElementById('resp').textContent=resp;
					setTimeout(getStatus,300);
				});
			}

			function sendGcode(){
				const v=document.getElementById('gcodeInput').value.trim();
				if(v){send(v);document.getElementById('gcodeInput').value='';}
			}

			function getStatus(){
				api('/status').then(data=>{
					try{
						const d=JSON.parse(data);
						document.getElementById('state').textContent=d.state||'--';
						document.getElementById('pos').textContent=d.position||'--';
					}catch(e){console.error(e);}
				});
			}

			function setResolution(){
				const v=document.getElementById('resolution').value;
				api('/camera','res='+v);
			}

			function setQuality(v){
				document.getElementById('qval').textContent=v;
				api('/camera','quality='+v);
			}

			function setBrightness(v){
				document.getElementById('bval').textContent=v;
				api('/camera','brightness='+v);
			}

			function setContrast(v){
				document.getElementById('cval').textContent=v;
				api('/camera','contrast='+v);
			}

			function toggleFlip(type){
				if(type==='h'){
					hflip=1-hflip;
					api('/camera','hmirror='+hflip);
				}else{
					vflip=1-vflip;
					api('/camera','vflip='+vflip);
				}
			}

			function startScan(){
				const xStart = document.getElementById('scanXStart').value;
				const xEnd = document.getElementById('scanXEnd').value;
				const yStart = document.getElementById('scanYStart').value;
				const yEnd = document.getElementById('scanYEnd').value;
				const step = document.getElementById('scanStep').value;
				const feed = document.getElementById('scanFeedRate').value;  // Use scan-specific feedrate
				
				api('/startscan',`xstart=${xStart}&xend=${xEnd}&ystart=${yStart}&yend=${yEnd}&step=${step}&feed=${feed}`)
				.then(resp => {
					if(resp === 'OK'){
						document.getElementById('scanStatus').textContent = 'Scanning...';
					} else {
						document.getElementById('scanStatus').textContent = 'Error: ' + resp;
					}
				});
			}

			function stopScan(){
				api('/stopscan').then(resp => {
					if(resp === 'OK'){
						document.getElementById('scanStatus').textContent = 'Scan stopped';
					} else {
						document.getElementById('scanStatus').textContent = 'Error: ' + resp;
					}
				});
			}

			function updateScanStatus(){
				api('/scanstatus').then(data => {
					try {
						const d = JSON.parse(data);
						document.getElementById('scanStatus').textContent = d.status;
						document.getElementById('scanProgressBar').style.width = d.progress + '%';
					} catch(e) {
						console.error(e);
					}
				});
			}

      // Update system info
      function updateSystemInfo() {
        api('/systeminfo').then(data => {
          try {
            const info = JSON.parse(data);
            document.getElementById('uptime').textContent = info.uptime;
            document.getElementById('wifiSignal').textContent = info.wifiSignal + ' dBm';
            document.getElementById('cpuUsage').textContent = info.cpuUsage + '%';
            document.getElementById('memory').textContent = info.memory + '%';
          } catch(e) {
            console.error(e);
          }
        });
      }

			setInterval(getStatus,2000);
			setInterval(updateScanStatus, 500);
      setInterval(updateSystemInfo, 5000);
			window.onload=function(){
        console.log('Ready');
        getStatus();
        updateSystemInfo();
      };

      function loadFiles(dir = '/') {
        const url = '/files' + (dir !== '/' ? '?dir=' + encodeURIComponent(dir) : '');
        fetch(url)
          .then(r => r.json())
          .then(data => {
            let html = '';
            // Parent folder
            if (data.parent) {
              const pf = (data.parent.lastIndexOf('/') > 0) ? data.parent.substring(0, data.parent.lastIndexOf('/')) : '/';
              html += `<button class="btn home-btn" style="display:block;margin:1px 0 1px;text-decoration:none" onclick="loadFiles('${pf}')">..</button><br>`;
            }
            // Folders
            data.folders.forEach(folder => {
              const folderName = '/' + folder;    // Hard coded for now
              html += `<div style="display:flex;align-items:center;gap:5px;margin:5px 0;">
                <button class="btn home-btn" style="flex:1" onclick="loadFiles('${folder}')">${folder}</button>
                <button class="btn stop-btn" style="padding:10px 10px;" onclick="deleteFolder('${folderName}')">Delete</button>
              </div>`;
            });
            // Files
            data.files.forEach(file => {
              const name = file.split('/').pop();
              const fullPath = data.parent + '/' + file;
              html += `<a href="/download?file=${encodeURIComponent(fullPath)}" class="btn scan-btn" style="display:block;margin:5px 0;text-decoration:none" download="${name}">${name}</a>`;
            });
            if (data.folders.length === 0 && data.files.length === 0) {
              html = '<p>No files found.</p>';
            }
            document.getElementById('fileList').innerHTML = html;
          })
          .catch(err => {
            document.getElementById('fileList').innerHTML = '<p>Error loading files.</p>';
            console.error(err);
          });
      }

      // Delete folder function
      function deleteFolder(folder) {
        if (confirm(`Are you sure you want to delete "${folder}"? This cannot be undone.`)) {
          fetch(`/deletefolder?folder=${encodeURIComponent(folder)}`)
            .then(r => r.text())
            .then(msg => {
              alert(msg);
              loadFiles(); // Refresh list
            })
            .catch(err => {
              alert('Error deleting folder');
              console.error(err);
            });
        }
      }

		</script>
	</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleCmd() {
  if (!server.hasArg("c")) {
    server.send(400, "text/plain", "Missing parameter");
    return;
  }
  
  String cmd = server.arg("c");
  String response = sendToGRBL(cmd, 2000);
  
  server.send(200, "text/plain", response.length() > 0 ? response : "OK");
  
  delay(100);
  updateGRBLStatus();
}

void handleStatus() {
  updateGRBLStatus();
  
  String json = "{";
  json += "\"state\":\"" + machineState + "\",";
  json += "\"position\":\"" + currentPosition + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleCamera() {
  sensor_t * s = esp_camera_sensor_get();
  if (!s) {
    server.send(500, "text/plain", "Camera not available");
    return;
  }
  
  if (server.hasArg("res")) {
    int res = server.arg("res").toInt();
    s->set_framesize(s, (framesize_t)res);
  }
  
  if (server.hasArg("quality")) {
    int quality = server.arg("quality").toInt();
    s->set_quality(s, quality);
  }
  
  if (server.hasArg("brightness")) {
    int brightness = server.arg("brightness").toInt();
    s->set_brightness(s, brightness);
  }
  
  if (server.hasArg("contrast")) {
    int contrast = server.arg("contrast").toInt();
    s->set_contrast(s, contrast);
  }
  
  if (server.hasArg("hmirror")) {
    int hmirror = server.arg("hmirror").toInt();
    s->set_hmirror(s, hmirror);
  }
  
  if (server.hasArg("vflip")) {
    int vflip = server.arg("vflip").toInt();
    s->set_vflip(s, vflip);
  }
  
  server.send(200, "text/plain", "OK");
}

void handleStream() {
  WiFiClient client = streamServer.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: ";
  response += _STREAM_CONTENT_TYPE;
  response += "\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      break;
    }
    
    client.print(_STREAM_BOUNDARY);
    client.printf(_STREAM_PART, fb->len);
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
    
    if(!client.connected()) break;
  }
}

void handleStartScan() {
  if (isScanning) {
    server.send(200, "text/plain", "Already scanning");
    return;
  }
  if (!server.hasArg("xstart") || !server.hasArg("xend") || 
      !server.hasArg("ystart") || !server.hasArg("yend") || 
      !server.hasArg("step")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }
  scanXStart = server.arg("xstart").toFloat();
  scanXEnd = server.arg("xend").toFloat();
  scanYStart = server.arg("ystart").toFloat();
  scanYEnd = server.arg("yend").toFloat();
  scanStepSize = server.arg("step").toFloat();
  scanFeedRate = server.hasArg("feed") ? server.arg("feed").toFloat() : 150.0;

  scanXCount = (int)ceil(abs(scanXEnd - scanXStart) / scanStepSize) + 1;
  scanYCount = (int)ceil(abs(scanYEnd - scanYStart) / scanStepSize) + 1;
  totalScanPoints = scanXCount * scanYCount;
  currentScanPoint = 0;

  // >>> NEW: Prepare for continuous capture <<<
  currentScanFolder = createScanFolder();
  frameCounter = 0;
  captureActive = true;

  // Start CNC scan logic (your existing state machine)
  currentScanState = SCAN_STARTING;
  currentRow = 0;
  goingRight = true;
  isScanning = true;
  scanStatus = "Scanning...";

  server.send(200, "text/plain", "OK");
}

void handleStopScan() {
  isScanning = false;
  captureActive = false; // stop capture immediately
  currentScanState = SCAN_COMPLETE;
  currentRow = 0;
  goingRight = true;
  currentScanPoint = 0;
  scanStatus = "Scan stopped";
  server.send(200, "text/plain", "OK");
}

void handleScanStatus() {
  String json = "{";
  json += "\"status\":\"" + scanStatus + "\",";
  json += "\"progress\":" + String(isScanning ? (currentScanPoint * 100.0 / totalScanPoints) : 0) + "}";
  server.send(200, "application/json", json);
}

void handleIP() {
  server.send(200, "text/plain", WiFi.localIP().toString().c_str());
}

void handleSystemInfo() {
  // Calculate uptime in seconds
  uint32_t uptime = millis() / 1000;
  uint32_t days = uptime / 86400;
  uint32_t hours = (uptime % 86400) / 3600;
  uint32_t minutes = (uptime % 3600) / 60;
  uint32_t seconds = uptime % 60;
  
  String uptimeStr = String(days) + "d " + String(hours) + "h " + 
                     String(minutes) + "m " + String(seconds) + "s";
  
  String json = "{";
  json += "\"uptime\":\"" + uptimeStr + "\",";
  json += "\"wifiSignal\":" + String(WiFi.RSSI()) + ",";
  json += "\"cpuUsage\":45,";  // Placeholder
  json += "\"memory\":60";     // Placeholder
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleNotFound(){
  server.send(404, "text/plain", "Not found");
}

void handleStreamNotFound(){
  streamServer.send(404, "text/plain", "Not found");
}

void streamTask(void * parameter) {
  while(true) {
    streamServer.handleClient();
    delay(1);
  }
}

void grblListenerTask(void * parameter) {
  while(true) {
    if (GRBL_SERIAL.available()) {
      String msg = GRBL_SERIAL.readStringUntil('\n');
      grblResponse = msg;
    }
    delay(10);
  }
}

String createScanFolder() {
  int n = 1;
  String folder;
  do {
    folder = "/scan_" + String(n, DEC);
    n++;
  } while (SD_MMC.exists(folder.c_str()));
  
  if (SD_MMC.mkdir(folder.c_str())) {
    Serial.println("Created folder: " + folder);
    return folder;
  }
  return "/"; // fallback
}

void saveFrame(camera_fb_t * fb, const char* folder, int index) {
  if (!fb) return;
  String filename = String(folder) + "/img_" + String(index, DEC) + ".jpg";
  // Optional: pad to 5 digits → img_00001.jpg
  while (filename.length() < strlen(folder) + 14) {
    filename = String(folder) + "/img_0" + filename.substring(strlen(folder) + 5);
  }

  File file = SD_MMC.open(filename.c_str(), FILE_WRITE);
  if (file) {
    file.write(fb->buf, fb->len);
    file.close();
    Serial.printf("Saved: %s (%u bytes)\n", filename.c_str(), fb->len);
  }
}

void captureTask(void * parameter) {
  while (true) {
    if (captureActive && isScanning) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (fb) {
        saveFrame(fb, currentScanFolder.c_str(), frameCounter++);
        esp_camera_fb_return(fb);
      }
      // Optional: add small delay to avoid overwhelming SD
      // delay(50); // ~20 FPS max
    } else {
      frameCounter = 0;
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1); // yield
  }
}

// Helper: list files in a directory
void listDir(fs::FS &fs, const char * dirname, uint8_t levels, std::vector<String> &folders, std::vector<String> &files) {
  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String fileName = file.name(); // ← This is already full path like "/scan_1/img_00000.jpg"

    if (file.isDirectory()) {
      folders.push_back(fileName); // Push full path: "/scan_1"
      if (levels > 0) {
        listDir(fs, fileName.c_str(), levels - 1, folders, files);
      }
    } else {
      if (fileName.endsWith(".jpg")) {
        files.push_back(fileName); // Push full path: "/scan_1/img_00000.jpg"
      }
    }
    file = root.openNextFile();
  }
}

void handleFiles() {
  String dir = "/";
  if (server.hasArg("dir")) {
    dir = server.arg("dir");
    // Ensure dir starts with '/'
    if (!dir.startsWith("/")) {
      dir = "/" + dir;
    }
    // Security: only allow /scan_* paths
    if (!dir.startsWith("/scan_")) {
      server.send(403, "text/plain", "Access denied");
      return;
    }
  }

  std::vector<String> folders;
  std::vector<String> files;
  listDir(SD_MMC, dir.c_str(), 0, folders, files);

  String parent = "";
  if (dir != "/") {
    int lastSlash = dir.lastIndexOf('/', dir.length() - 2);
    if (lastSlash >= 0) {
      // parent = dir.substring(0, lastSlash == 0 ? 1 : lastSlash);
      parent=dir;
    }
  }

  String json = "{";
  if (parent != "") {
    json += "\"parent\":\"" + parent + "\",";
  }
  json += "\"folders\":[";
  for (size_t i = 0; i < folders.size(); i++) {
    if (i > 0) json += ",";
    json += "\"" + folders[i] + "\"";
  }
  json += "],\"files\":[";
  for (size_t i = 0; i < files.size(); i++) {
    if (i > 0) json += ",";
    json += "\"" + files[i] + "\"";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file param");
    return;
  }
  String filename = server.arg("file");
  // Security: only allow .jpg in /scan_*
  if (!filename.startsWith("/scan_") || !filename.endsWith(".jpg")) {
    server.send(403, "text/plain", "Access denied");
    return;
  }

  if (!SD_MMC.exists(filename.c_str())) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File file = SD_MMC.open(filename.c_str(), FILE_READ);
  if (!file) {
    server.send(500, "text/plain", "Cannot open file");
    return;
  }

  server.streamFile(file, "image/jpeg");
  file.close();
}

// Recursive helper to delete a folder and all its contents
bool deleteFolderRecursive(fs::FS &fs, const String& path) {
  File root = fs.open(path);
  if (!root || !root.isDirectory()) {
    // If it's a file, just delete it
    return fs.remove(path);
  }

  // Delete all contents first
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    if (String(fileName).startsWith("/")) {
      fileName = fileName.substring(1);
    }
    String fullPath = path + "/" + fileName;
    if (file.isDirectory()) {
      if (!deleteFolderRecursive(fs, fullPath)) {
        file.close();
        return false;
      }
    } else {
      if (!fs.remove(fullPath)) {
        file.close();
        return false;
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();

  // Now delete the (empty) folder
  return fs.rmdir(path);
}

void handleDeleteFolder() {
  if (!server.hasArg("folder")) {
    server.send(400, "text/plain", "Missing folder param");
    return;
  }
  String folder = server.arg("folder");
  // Security: only allow /scan_* paths
  if (!folder.startsWith("/scan_")) {
    server.send(403, "text/plain", "Access denied");
    return;
  }
  if (!SD_MMC.exists(folder.c_str())) {
    server.send(404, "text/plain", "Folder not found");
    return;
  }

  bool success = deleteFolderRecursive(SD_MMC, folder);
  if (success) {
    server.send(200, "text/plain", "Folder deleted");
  } else {
    server.send(500, "text/plain", "Failed to delete folder");
  }
}

// Non-blocking spatial scan task
void scanTask(void * parameter) {
  while(true) {
    if (isScanning && currentScanState != SCAN_COMPLETE) {
      switch (currentScanState) {
        case SCAN_STARTING:
          // Move to starting position
          {
            String startCmd = "G21 G90 G0 X" + String(scanXStart, 2) + " Y" + String(scanYStart, 2) + " F" + String(scanFeedRate, 0);
            sendToGRBL(startCmd);
            currentScanState = MOVE_TO_ROW;
          }
          break;
          
        case MOVE_TO_ROW:
          // Wait for machine to reach idle state
          if (machineState == "Idle") {
            currentScanState = TRAVERSE_ROW;
          }
          break;
          
        case TRAVERSE_ROW:
          // Send command to traverse the row
          {
            float xTarget = goingRight ? scanXEnd : scanXStart;
            String xMove = "G21 G90 G1 X" + String(xTarget, 2) + " F" + String(scanFeedRate, 0);
            sendToGRBL(xMove);
            currentScanState = NEXT_ROW;
          }
          break;
          
        case NEXT_ROW:
          // Wait for X move to complete
          if (machineState == "Idle") {
            // Update progress for completed row
            currentScanPoint = (currentRow + 1) * scanXCount;
            
            // Check if we've completed all rows
            if (currentRow >= scanYCount - 1) {  // -1 because we start from 0
              currentScanState = SCAN_COMPLETE;
              isScanning = false;
              scanStatus = "Scan complete";
              break;
            }
            
            // Move Y axis and toggle X direction
            currentRow++;
            goingRight = !goingRight;
            float yTarget = scanYStart + currentRow * scanStepSize;
            String yMove = "G21 G90 G0 Y" + String(yTarget, 2) + " F" + String(scanFeedRate, 0);
            sendToGRBL(yMove);
            currentScanState = MOVE_TO_ROW;
          }
          break;
          
        case SCAN_COMPLETE:
          isScanning = false;
          captureActive = false;
          scanStatus = "Scan complete";
          break;
      }
    } else if (!isScanning) {
      // If not scanning, ensure state machine is reset
      currentScanState = SCAN_COMPLETE;
    }
    delay(100); // Non-blocking delay
  }
}

void setup() {
  // Initialize Serial for GRBL communication (UART0)
  Serial.begin(GRBL_BAUD);
  delay(2000);
  
  Serial.println("Init camera...");
  if (!initCamera()) {
    delay(3000);
    ESP.restart();
  }

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
  } else {
    Serial.println("SD Card initialized");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - start > 20000) {
      delay(3000);
      ESP.restart();
    }
  }

  Serial.print("ESP IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/camera", HTTP_GET, handleCamera);
  server.on("/startscan", HTTP_GET, handleStartScan);
  server.on("/stopscan", HTTP_GET, handleStopScan);
  server.on("/scanstatus", HTTP_GET, handleScanStatus);
  server.on("/ip", HTTP_GET, handleIP);
  server.on("/systeminfo", HTTP_GET, handleSystemInfo);
  server.on("/files", HTTP_GET, handleFiles);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/deletefolder", HTTP_GET, handleDeleteFolder);
  server.onNotFound(handleNotFound);
  server.begin();

  streamServer.on("/stream", HTTP_GET, handleStream);
  streamServer.onNotFound(handleStreamNotFound);
  streamServer.begin();
  
  xTaskCreatePinnedToCore(streamTask, "StreamTask", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(grblListenerTask, "GRBLTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(scanTask, "ScanTask", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(captureTask, "CaptureTask", 4096, NULL, 2, NULL, 1);

  initGRBL();
}

void loop() {
  server.handleClient();
  delay(1);
}
