# Assistive Braille System

An embedded assistive system designed to help visually impaired users by capturing text from the environment and converting it into accessible output using OCR, speech transcription, sensor-based gate logic, and Braille-style LED feedback.

This project combines a microcontroller-driven hardware sensing layer, a camera-enabled ESP32 module, and a Python Flask backend that runs computer vision and speech recognition models.

## Overview

The system monitors the user environment using:

- Light sensor input
- Motion detection
- Distance sensing
- Camera capture for text recognition
- Audio capture for speech-to-text conversion
- Braille-style LED output and buzzer alerts

When a trigger condition is met, the hardware captures either an image or an audio segment, sends it to the backend, and receives interpreted text or an alert response.

## System Architecture

The solution is split into three main components:

1. K64F controller firmware
   - Reads sensor values
   - Handles button triggers and timing logic
   - Controls LEDs, buzzer, LCD, and UART communication
   - Sends captured data to the ESP32 and/or server

2. ESP32-S3 camera node
   - Captures JPEG frames from the camera
   - Sends images to the backend OCR endpoint
   - Receives audio frames and forwards them to the backend audio endpoint
   - Communicates with the K64F through UART

3. Python backend server
   - Runs Flask API endpoints
   - Saves sensor, OCR, and audio logs to SQLite
   - Uses EasyOCR for image text recognition
   - Uses Whisper for audio transcription
   - Serves a live dashboard for monitoring activity

## Hardware Components

- NXP FRDM-K64F board (MK64F12)
- ESP32-S3-EYE / ESP32-S3 camera module
- Light-dependent resistor (LDR)
- Accelerometer (ADXL345)
- VL53L0X or equivalent distance sensor
- Braille LED output array
- Buzzer
- LCD display
- Wi-Fi network for ESP32-to-server communication

## Software Components

### `backend_server.py`

Main Flask application that:

- Accepts OCR image uploads from the ESP32
- Accepts audio payloads from the ESP32
- Accepts stability/sensor event data
- Runs OCR and speech transcription pipelines
- Stores results in SQLite tables
- Exposes dashboard and API endpoints

### `k64f_main.c`

Main firmware for the K64F. It includes:

- ADC readings for light and microphone inputs
- LED and buzzer control
- trigger logic and button handling
- UART handling for communication with ESP32
- sensor gate logic for motion/light/distance checks
- Braille output and LCD management

### `esp32_main.c`

ESP32 firmware responsible for:

- Camera initialization and image capture
- Wi-Fi connectivity
- HTTP uploads to the Flask server
- Audio frame reception and forwarding
- UART communication with the K64F

## Key Features

- OCR-based text recognition from captured images
- Confidence threshold filtering for reliable text extraction
- Alert behavior for low-confidence or unreadable input
- Audio transcription using Whisper
- SQLite logging for OCR, audio, and stability events
- Dashboard UI for live monitoring
- Modular design spanning embedded hardware and backend intelligence

## Data Flow

1. K64F detects a trigger condition via sensors
2. ESP32 captures image/audio data
3. ESP32 sends the data to the Flask backend
4. Backend performs OCR or speech recognition
5. Recognized text is returned to the K64F
6. K64F displays the result and/or activates Braille output and buzzer feedback

## Project Structure

```text
Assistive-Braille-System/
├── README.md
├── backend_server.py
├── k64f_main.c
├── esp32_main.c
├── block diagram.jpeg
├── recordings/
├── photos/
├── sep600.db
└── ...
```

## Setup and Installation

### Python dependencies

Install the required packages:

```bash
pip install flask numpy scipy easyocr openai-whisper
```

### Start the backend server

```bash
python backend_server.py
```

This starts the Flask server and loads the OCR and Whisper models on first run.

### Note on the database and output folders

The backend automatically creates:

- `recordings/` for audio WAV files
- `photos/` for captured image files
- `sep600.db` for SQLite event logging

## API Endpoints

- `GET /` — dashboard UI
- `GET /test` — connection health check
- `GET /status` — server status and configuration
- `POST /stability` — sensor event logging
- `POST /ocr` — image-based text recognition
- `POST /audio` — speech transcription input
- `GET /api/events` — stability data
- `GET /api/ocr` — OCR log history
- `GET /api/audio` — audio log history

## Dashboard

The project includes a live dashboard served by the Flask server. It displays:

- current sensor values
- OCR confidence meter
- latest recognition output
- audio transcript history
- stability/event logs

A diagram of the system is included in: [block diagram.jpeg](./block diagram.jpeg)

## Configuration Notes

The backend contains several tunable parameters such as:

- `CONFIDENCE_THRESHOLD`
- `SAMPLE_RATE`
- `WHISPER_MODEL`
- `OCR_LANGUAGES`

These values can be adjusted depending on the hardware environment and recognition quality.

## Usage Notes

- Wi-Fi credentials in the ESP32 firmware should be updated before deployment.
- Some endpoints expect the ESP32 and K64F to be on the same network or physically linked through UART.
- OCR and Whisper require model downloads on the first run, which may take additional time.

## Project Status

This project is an embedded AI-assisted accessibility prototype combining sensor processing, computer vision, speech recognition, and tactile Braille-style feedback to improve environmental interaction for visually impaired users.

## License

This project does not currently include a formal license file. If you plan to distribute or extend it, add an appropriate license and attribution before publication.

