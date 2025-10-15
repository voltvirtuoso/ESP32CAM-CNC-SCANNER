# ESP32-CAM + GRBL CNC Spatial Scan System
**Full Technical Documentation**

---

## 📌 Overview

This system integrates an **ESP32-CAM** module with a **GRBL-controlled CNC machine** to perform **automated spatial scanning**. The ESP32 serves dual roles:

1. **Web-based CNC controller** with real-time camera feed.
2. **Automated image capture engine** synchronized with CNC motion.

A companion **Python utility** enables users to **download captured images** and **generate time-lapse videos** (MP4, AVI, MOV, GIF) from scan sessions.

---

## 📥 Firmware Upload Guide

### 🔧 Required Tools & Prerequisites

#### Hardware Requirements
- **ESP32-CAM Module** (AI-Thinker or compatible)
- **Arduino Uno** (or GRBL-compatible board)
- **FTDI Programmer** (3.3V compatible, 6-pin connection) - *for ESP32-CAM*
- **USB-A to USB-B cable** - *for Arduino Uno*
- **Jumper wires** (male-to-female recommended)
- **Computer** with USB port

#### Software Requirements
**For ESP32-CAM:**
- **Arduino IDE** (v2.0.0 or higher)
- **ESP32 Board Package** (v2.0.0+)
- **Required Libraries** (install via Arduino Library Manager):
  - `ESP32 Camera` by Espressif Systems
  - `SD_MMC` by Espressif Systems
  - `WiFi` (built-in)
  - `WebServer` (built-in)

**For Arduino Uno (GRBL):**
- **Arduino IDE** (v1.8.x or v2.x)
- **GRBL Firmware** (latest stable release)

### 📦 ESP32-CAM Firmware Upload

#### 1. **Arduino IDE Setup for ESP32**
```cpp
// 1. Install ESP32 Board Package
// File → Preferences → Additional Boards Manager URLs:
// https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

// 2. Install via Tools → Board → Boards Manager → Search "ESP32"
// 3. Select: Tools → Board → "AI Thinker ESP32-CAM"
// 4. Configure settings:
//    - Partition Scheme: Huge App (3MB No OTA/1MB SPIFFS)
//    - Flash Frequency: 80MHz
//    - Flash Mode: QIO
//    - Flash Size: 4MB
//    - Core Debug Level: None
```
![[assets/1.png]]
#### 2. **Hardware Wiring for ESP32 Upload**
```
ESP32-CAM → FTDI Programmer
GPIO0    → GND (for upload mode - connect before power on)
VCC      → 5V
GND      → GND
TX       → RX
RX       → TX
```

⚠️ **Important**: 
- Connect GPIO0 to GND before powering on for upload mode
- Remove GPIO0-GND connection after upload for normal operation
- Ensure FTDI outputs 3.3V (not 5V) if possible

#### 3. **ESP32 Firmware Configuration**
Before uploading, configure your WiFi credentials in the firmware:

```cpp
// In your main .ino file, locate these variables:
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

#### 4. **Upload ESP32 Firmware Process**
1. **Connect ESP32-CAM** to FTDI programmer (GPIO0 to GND)
2. **Select Board**: `AI Thinker ESP32-CAM`
3. **Select Port**: Choose the detected COM/USB port
4. **Verify Code**: Click `✓` (Check mark)
5. **Upload**: Click `→` (Right arrow)
6. **Monitor Progress**: Watch serial output for success message

#### 5. **Post-Upload ESP32 Configuration**
After successful upload:
1. **Disconnect power** from ESP32-CAM
2. **Remove GPIO0-GND connection** (for normal operation)
3. **Reconnect power** to ESP32-CAM
4. **Monitor Serial Console** (115200 baud) for:
   - WiFi connection success
   - IP address assignment
   - SD card initialization
   - System ready status

### ⚙️ Arduino Uno (GRBL) Firmware Upload

#### 1. **Download GRBL Firmware**
- **GitHub Repository**: https://github.com/grbl/grbl
- **Latest Release**: Download `.zip` file from Releases
- **Alternative**: Clone repository via Git

#### 2. **Arduino IDE Setup for GRBL**
```cpp
// 1. Extract GRBL zip file
// 2. Open Arduino IDE
// 3. Goto Sketch -> Include Library -> Add .ZIP Library
// 4. Navigate to extratcted gbrl folder and go to grbl inside it and add it to arduino
// 5. Go to File -> Examples -> gbrl
// 6. Select: Tools → Board → "Arduino Uno"
// 7. Select: Tools → Port → Choose Arduino Uno port and upload
```

#### 3. **GRBL Configuration (Optional)**
Before upload, you can modify GRBL settings in `config.h`:
```cpp
// Common modifications:
#define DEFAULT_STEP_PULSE_MICROSECONDS 10
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255
#define DEFAULT_FEED_RATE 1000.0
#define DEFAULT Rapids_FEED_RATE 2000.0
```

#### 4. **Upload GRBL Process**
1. **Open GRBL source** in Arduino IDE
2. **Select Board**: `Arduino Uno`
3. **Select Port**: Choose Arduino Uno COM port
4. **Upload**: Click `→` (Right arrow)
5. **Wait for Success**: "Done uploading" message

#### 5. **Verify GRBL Installation**
1. **Open Serial Monitor** (115200 baud)
2. **Send `$`** command
3. **Expected Response**: Should show GRBL version and help info
4. **Test Connection**: Send `?` for real-time status

### 🖥️ **System Integration Check**
After both firmwares are uploaded:

**ESP32-CAM → Arduino Uno Connection:**
```
ESP32 (RX) → Arduino Uno TX (Pin 1)
ESP32 (TX) → Arduino Uno RX (Pin 0)
ESP32 GND → Arduino GND
```

### 🔄 **GitHub Repository Setup**

#### 1. **Clone ESP32 Firmware Repository**
```bash
# Open terminal/command prompt
git clone https://github.com/voltvirtuoso/ESP32CAM-CNC-SCANNER.git
cd ESP32CAM-CNC-SCANNER

# Or download ZIP from GitHub → Code → Download ZIP
```

#### 2. **Uploading Firmware**
```cpp
// 1. For direct GitHub sketch:
// File → Open → Navigate to cloned repository folder
// Select the .ino file
```

#### 3. **Updating Firmware from GitHub**
```bash
# Navigate to repository
cd ESP32CAM-CNC-SCANNER
git pull origin main  # Fetch latest changes

# Then open in Arduino IDE and upload as usual
```

### 🖥️ **Web Interface Access**
- **ESP32 UI**: `http://[ESP32_IP_ADDRESS]`
- **ESP32 Stream**: `http://[ESP32_IP_ADDRESS]:81/stream`
- **ESP32 API**: `http://[ESP32_IP_ADDRESS]/[endpoint]`
- **GRBL Serial**: Connect via any serial terminal (115200 baud)

### ⚠️ **Troubleshooting Common Upload Issues**

| Component       | Issue                                                              | Solution                                          |
| --------------- | ------------------------------------------------------------------ | ------------------------------------------------- |
| **ESP32-CAM**   | ESP keep looping                                                   | Remove GPIO0-GND connection, reset ESP32          |
| **ESP32-CAM**   | `A fatal error occurred: MD5 of file does not match data in flash` | Retry upload, ensure stable USB connection        |
| **ESP32-CAM**   | `error: espcomm_upload_mem failed`                                 | Check wiring, ensure 3.3V power supply            |
| **ESP32-CAM**   | Configuring flash size... Failed to connect                        | Verify GPIO0 to GND during upload                 |
| **Arduino Uno** | Upload fails with timeout                                          | Disconnect USB-B from Uno, upload, then reconnect |
| **Arduino Uno** | GRBL not responding                                                | Check baud rate (115200), verify wiring           |
| **System**      | ESP32 can't communicate with GRBL                                  | Check TX/RX connections, verify baud rates match  |
> [!Warning] Firmware Uploading
> Disconnect ESP32 and Arduino before flashing firmware


### ✅ **Verification Checklist**
**ESP32-CAM:**
- [ ] WiFi connection established
- [ ] Web UI accessible at assigned IP
- [ ] Camera stream working
- [ ] SD card detected and writable

**Arduino Uno (GRBL):**
- [ ] GRBL firmware installed
- [ ] Serial communication working
- [ ] `$` command responds with GRBL info
- [ ] `?` command shows real-time status

**System Integration:**
- [ ] ESP32 can send G-code to GRBL
- [ ] Status polling working
- [ ] CNC motion control functional
- [ ] Scan functionality operational

---

## 🧠 Architecture

### Hardware Stack
| Component             | Role |
|----------------------|------|
| **ESP32-CAM (AI-Thinker)** | Camera + Web server + GRBL serial bridge |
| **GRBL-compatible CNC** (e.g., Arduino Uno + CNC Shield) | Motion control |
| **MicroSD Card** (mounted via SD_MMC) | Stores captured images per scan |
| **WiFi Network** | Hosts web UI + enables remote access |

### Software Stack
| Layer            | Technology                                    |
| ---------------- | --------------------------------------------- |
| **Firmware**     | CUSTOM ESP32 + Arduino AVR GBRL               |
| **Web UI**       | HTML5 + CSS3 + Vanilla JS                     |
| **Backend APIs** | RESTful endpoints on ESP32                    |
| **Helper Tool**  | Python 3 + Requests + OpenCV + ImageIO + Rich |

---

## 📦 Core Features

### 1. **Web-Based CNC Control**
- Real-time GRBL status polling (`?` command parsing)
- Jogging (X/Y/Z ± with configurable step/feed)
- Homing (`$H`), unlocking (`$X`), probing (`G38.2`)
- Work coordinate zeroing (`G10 L20`)
- Emergency stop (soft reset: `0x18`)

![[assets/2.png]]
### 2. **Live Camera Streaming**
- MJPEG stream over dedicated port (`:81/stream`)
- Configurable:
  - Resolution (QVGA → UXGA)
  - JPEG quality (5–63)
  - Brightness/contrast (-2 → +2)
  - Horizontal/vertical flip

![[assets/3.png]]
### 3. **Automated Spatial Scanning**
- **Raster scan pattern**: Zig-zag motion over user-defined XY grid
- Parameters:
  - X/Y start/end (mm)
  - Step size (mm)
  - Feed rate (mm/min)
- **Non-blocking state machine** ensures smooth motion + capture
- Each scan creates a **dedicated folder** (`/scan_N/`) on SD card
- Images saved as `img_00000.jpg`, `img_00001.jpg`, ...

![[assets/4.png]]
### 4. **File Management**
- Browse/delete scan folders via web UI
- Secure download endpoint (only `.jpg` in `/scan_*` allowed)
- Recursive folder deletion

![[assets/5.png]]
### 5. **Python Scan-to-Video Utility**
- Auto-detects scan folders from ESP32
- Downloads all images in order
- Encodes to multiple video formats:
  - **MP4** (H.264 via OpenCV)
  - **AVI** (Motion JPEG)
  - **MOV** (QuickTime-compatible)
  - **GIF** (via ImageIO)
- Optional: Save raw images to `downloads/`
- Rich CLI with progress bars, tables, and validation

![[assets/6.png]]

---

## 🔧 Firmware: Object & Type Reference

### Global State Variables
| Variable | Type | Description |
|--------|------|-------------|
| `isScanning` | `bool` | Scan in progress flag |
| `currentScanState` | `enum ScanState` | FSM state (`SCAN_STARTING`, `MOVE_TO_ROW`, etc.) |
| `scanXStart`, `scanYEnd`, etc. | `float` | Scan geometry |
| `scanFeedRate` | `float` | Current scan speed (mm/min) |
| `currentScanFolder` | `String` | Active SD folder (e.g., `"/scan_3"`) |
| `captureActive` | `volatile bool` | Enables concurrent image capture |
| `frameCounter` | `volatile int` | Auto-incrementing image index |

### Key Functions
| Function             | Signature                              | Purpose                               |
| -------------------- | -------------------------------------- | ------------------------------------- |
| `sendToGRBL()`       | `String(String cmd, uint32_t timeout)` | Send G-code, return response          |
| `updateGRBLStatus()` | `void()`                               | Parse `<Idle                          |
| `initGRBL()`         | `void()`                               | Reset + flush GRBL on boot            |
| `handleStartScan()`  | `void()`                               | API endpoint: parse params, init scan |
| `scanTask()`         | `void*` (FreeRTOS task)                | Non-blocking scan FSM                 |
| `captureTask()`      | `void*` (FreeRTOS task)                | Concurrent image capture to SD        |
| `saveFrame()`        | `void(camera_fb_t*, const char*, int)` | Write JPEG to SD                      |

### Web Endpoints (`HTTP GET`)
| Path | Params | Action |
|-----|--------|--------|
| `/` | — | Serve HTML UI |
| `/cmd` | `c=GCODE` | Send G-code to GRBL |
| `/status` | — | Return `{state, position}` JSON |
| `/camera` | `res=6&quality=10&...` | Update camera settings |
| `/startscan` | `xstart=0&xend=100&...` | Begin spatial scan |
| `/stopscan` | — | Halt scan immediately |
| `/scanstatus` | — | Return `{status, progress}` |
| `/files` | `dir=/scan_1` | List folders/files (JSON) |
| `/download` | `file=/scan_1/img_00000.jpg` | Stream image |
| `/deletefolder` | `folder=/scan_1` | Recursively delete |

---

## 🐍 Python Utility: Object & Type Reference

### Settings Dictionary
```python
{
  "esp32_ip": str,        # e.g., "192.168.43.133"
  "fps": int,             # Video frame rate
  "save_images": bool,    # Keep raw JPGs?
  "output_format": str    # "mp4", "avi", "mov", "gif"
}
```

### Core Functions
| Function | Returns | Description |
|--------|--------|------------|
| `get_scan_folders()` | `List[str]` | Fetch `/scan_N` folders |
| `fetch_image_list()` | `List[str]` | Get sorted `.jpg` paths |
| `download_images()` | `List[np.ndarray]` | Load frames as OpenCV BGR arrays |
| `save_video()` | `bool` | Encode frames to file |

### Data Flow
1. **User config** → IP, FPS, format
2. **Fetch folders** → Parse `/files` JSON
3. **Select folder** → Validate `scan_N` pattern
4. **List images** → Sort by `img_XXXXX.jpg` number
5. **Download** → Concurrent requests → OpenCV frames
6. **Encode** → OpenCV (video) or ImageIO (GIF)

---

## 🛠️ Build & Deployment

### ESP32 Firmware
1. Install **Arduino IDE** + ESP32 board support
2. Install libraries:
   - `ESP32 Camera`
   - `SD_MMC`
3. Configure:
   ```cpp
   const char* ssid = "YOUR_SSID";
   const char* password = "YOUR_PASS";
   ```
4. Flash to ESP32-CAM

### Python Utility
```bash
pip install requests opencv-python numpy imageio rich
python scan_to_video.py
```

---

## 🔒 Security Considerations
- **Folder access restricted** to `/scan_*` paths
- **File downloads limited** to `.jpg` extension
- No authentication (intended for local/trusted networks)
- **Recommendation**: Run on isolated WiFi (e.g., ESP32 AP mode)

## 📎 Appendix: API Specification

### `/status` Response
```json
{
  "state": "Idle",
  "position": "X:12.34 Y:56.78 Z:0.00"
}
```

### `/scanstatus` Response
```json
{
  "status": "Scanning...",
  "progress": 42.5
}
```

### `/files` Response
```json
{
  "parent": "/scan_1",
  "folders": ["/scan_1/sub"],
  "files": ["/scan_1/img_00000.jpg", ...]
}
```

---

> ✅ **System Status**: Production-ready for lab/industrial prototyping  
> 📸 **Use Cases**: PCB inspection, macro photography, material scanning, CNC verification

--- 

*Documentation generated for firmware v1.0.0 • Python utility v1.2.0*
