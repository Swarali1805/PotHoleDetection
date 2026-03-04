# System Architecture Design

## System Architecture Overview

The Intelligent Pothole Detection System (IPDS) is designed around an event-driven, distributed computing model. It leverages inexpensive, low-power microcontrollers for data acquisition (the hardware layer) and offloads heavyweight computer vision inference to a more powerful, centralized processing hub (the application layer).

```text
[Hardwear Layer]          [Communication]         [Processing Layer]
ESP32-CAM (Vision)   ---- (WiFi / MJPEG) ---->   Python YOLOv8 Logic
ESP32 Dev (Sensor)   <--- (WiFi / HTTP)  ---->   Python Data Fusion
```

## High Level Architecture
The architecture comprises four primary layers:
1. **Hardware Layer:** Physical sensors mapping the real world.
2. **Firmware Layer:** C++ code allowing microcontrollers to operate the hardware.
3. **Processing Layer:** Python AI and logic bridging the raw data into insights.
4. **Application Layer:** The final output generation (logs and video).

## Hardware Layer
The system splits hardware responsibilities into two distinct physical nodes to prevent blocking the video frame rate with slow I2C sensor reads:
- **Vision Node:** An ESP32-CAM module acting purely as a wireless webcam.
- **Sensor Node:** An ESP32 Dev board acting as an I2C and UART hub, pulling in environmental data from an MPU6050 (Accelerometer), NEO-6M (GPS), and DS3231 (RTC).

## Firmware Layer
- **MJPEG Server (`esp_32_cam_final.ino`):** Uses the `esp_http_server` library to continuously push JPEG frames over an open socket.
- **Sensor Server (`esp_32_final.ino`):** Uses the `WebServer` library to host a simple REST API. It idles until queried, drastically reducing power consumption compared to continuous broadcasting.

## Processing Layer
The central brain of the system, currently a Python instance.
- **Computer Vision:** Ultralytics YOLOv8 processes frames natively for object detection.
- **Tracking:** SORT maintains state across frames utilizing Kalman filtering to predict pothole movement even during momentary obscuration.
- **Logic Matrix:** A heuristic engine that evaluates the tracked objects (filtering by Area/Aspect Ratios and persistence) to determine when the vehicle's axle is actively crossing the hazard.

## Application Layer
Where the gathered and processed data becomes actionable.
- **CSV Data Logger:** Generates database-ready tables `(Outputs/logs/)` containing the precise ID, Date, GPS, and calculated Severity.
- **Annotated Video Muxer:** Generates MP4s `(Outputs/videos/)` actively painting bounding boxes and confidence scores over the raw stream for visual verification.

## Data Flow Architecture
1. **Continuous Capture:** ESP32-CAM pushes 320x240 RGB frames over WiFi.
2. **Continuous Inference:** Processing hub receives a frame, passes it to YOLOv8, and updates the SORT tracker.
3. **Event Trigger:** If a recognized pothole crosses the designated reference line (near the bottom of the frame where the bumper is), the system halts tracking for a millisecond to fire an HTTP GET request to the Sensor Node (`http://IP/query?pothole_id=x`).
4. **Acquisition:** Sensor Node fires I2C reads to the MPU6050, grabs the exact GPS coordinates, and returns a JSON payload.
5. **Fusion & Log:** Python receives the JSON, mathematically fuses the YOLO confidence score with the MPU `m/s²` peak jerk, writes the result to the CSV file, and resumes the video loop.

## Module Breakdown
### Vision Node
- Focused entirely on low-latency frame extraction. No processing occurs here to avoid thermal throttling.
### Sensor Node
- Acts as a stateless microservice. When queried, it measures the immediate physical impact, preventing the need to synchronize complex clock times between the video and the sensors.
### Data Processing
- Built entirely independently of the hardware. The Python script doesn't care if the data comes from a local file or live streams; it processes standard formats.
### Detection System
- Designed iteratively. YOLO provides the "What", SORT provides the "Where", Heuristics provide the "Is it fake?", and the MPU provides the "How bad is it?".

## Communication Protocols
- **WiFi:** The overarching transport protocol connecting the nodes and the hub (TCP/IP).
- **HTTP / MJPEG:** The application-level protocol for video streaming.
- **HTTP/REST (GET):** The application-level protocol for sensor querying and JSON delivery.
- **I2C:** Inter-Integrated Circuit protocol (SDA/SCL) utilized heavily on the Sensor Node to quickly pull data from the MPU and RTC.
- **UART:** Serial protocol utilized on the Sensor Node to receive NMEA sentences from the GPS module.

## Scalability
The architecture is inherently scalable horizontally.
- Because the Sensor Node is a standard REST API, multiple cameras (e.g., front, left side, right side) could theoretically stream frames to an edge server, and all individually request severity data from the single central Sensor Node exactly when needed.
- Upgrading the Vision Node to a stereoscopic camera for depth inference solely requires pointing the Python script to a new IP address.

## Design Decisions
- **Why dual ESP32s instead of one?**
  - Attempting to run an I2C accelerometer read (which takes a few milliseconds) interrupts the blocking `esp_camera_fb_get()` function on the ESP32-CAM. This causes extreme frame drops and stuttering in the video. Splitting them solves the concurrency issue perfectly.
- **Why WiFi instead of Bluetooth?**
  - Bluetooth bandwidth is notoriously tricky for reliable high-framerate MJPEG video streaming. WiFi opens up massive bandwidth overhead.
- **Why YOLOv8 over older models?**
  - YOLOv8 provides extreme optimization, allowing 10-15 FPS inference straight out-of-the-box on a CPU, whereas older iterations demanded desktop-class GPUs.
