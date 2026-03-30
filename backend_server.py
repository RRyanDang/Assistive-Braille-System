"""
backend_server.py
-----
SEP600 — Complete Flask server.

Fixed to match esp32_sep600.ino and combined.c exactly:

  ESP32 POSTs JPEG      → POST /ocr       (was /photo)
  ESP32 POSTs binary    → POST /audio     (was /data with JSON)
  ESP32 POSTs stability → POST /stability (unchanged)

Pipelines:
  POST /ocr        → JPEG bytes → EasyOCR → confidence gate → SQLite → plain text
  POST /audio      → raw int16 PCM bytes → WAV → Whisper STT → plain text
  POST /stability  → CSV sensor gate events → SQLite
  GET  /           → live dashboard
  GET  /api/ocr    → OCR log JSON
  GET  /api/events → stability log JSON
  GET  /api/audio  → audio log JSON
  GET  /test       → ping
  GET  /status     → uptime + config

Milestone 5 response (plain text — ESP32 forwards as TRANSCRIPT:text to K64F):
  HIGH confidence (>= threshold) → OCR text   → K64F Braille
  LOW  confidence (<  threshold) → "BUZZER"   → K64F fires buzzer

Run:
    pip install flask numpy scipy easyocr openai-whisper
    python m5.py
"""

import os
import sqlite3
import datetime
import numpy as np
import easyocr
import whisper
from scipy.io.wavfile import write as wav_write
from flask import Flask, request, jsonify

app = Flask(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────

BASE_DIR       = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(BASE_DIR, "recordings")
PHOTOS_DIR     = os.path.join(BASE_DIR, "photos")
DB_FILE        = os.path.join(BASE_DIR, "sep600.db")

# Must match K64F combined.c:
#   MAX_SAMPLES = 19000
#   DECIMATION_FACTOR = 2
#   Elapsed time measured at runtime — we use a fixed value here
#   Actual rate prints in K64F serial: "Sample rate: ~XXXX Hz"
SAMPLE_RATE = 12751   # Hz — update if K64F serial shows a different value

OCR_LANGUAGES = ['en']
WHISPER_MODEL = "base"

# Milestone 5 — confidence threshold
# EasyOCR returns 0.0–1.0 per word. We average all words.
# Below this → LOW confidence → return "BUZZER" → K64F fires buzzer
CONFIDENCE_THRESHOLD = 0.60   # 60% — tune as needed

# ─────────────────────────────────────────────────────────────────────────────
# STARTUP — load models once (slow first time)
# ─────────────────────────────────────────────────────────────────────────────

print(f"[EasyOCR] Loading reader for languages: {OCR_LANGUAGES}...")
ocr_reader = easyocr.Reader(OCR_LANGUAGES, gpu=False)
print(f"[EasyOCR] Reader ready.")

print(f"[Whisper] Loading model '{WHISPER_MODEL}'...")
whisper_model = whisper.load_model(WHISPER_MODEL)
print(f"[Whisper] Model ready.")

os.makedirs(RECORDINGS_DIR, exist_ok=True)
os.makedirs(PHOTOS_DIR,     exist_ok=True)

print(f"[DIR] Recordings : {RECORDINGS_DIR}")
print(f"[DIR] Photos     : {PHOTOS_DIR}")

# ─────────────────────────────────────────────────────────────────────────────
# SQLITE
# ─────────────────────────────────────────────────────────────────────────────

def init_db():
    """Create all tables. Safe to call repeatedly."""
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()

    # Stability/sensor gate events
    c.execute("""
        CREATE TABLE IF NOT EXISTS stability_log (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            status    TEXT,
            light     INTEGER,
            motion    INTEGER,
            distance  INTEGER,
            dist_cm   INTEGER
        )
    """)

    # OCR results with confidence
    c.execute("""
        CREATE TABLE IF NOT EXISTS ocr_log (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp    TEXT,
            filename     TEXT,
            ocr_text     TEXT,
            confidence   REAL,
            alert        TEXT
        )
    """)

    # Audio transcription log
    c.execute("""
        CREATE TABLE IF NOT EXISTS audio_log (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT,
            filename    TEXT,
            transcript  TEXT,
            sample_rate INTEGER
        )
    """)

    conn.commit()
    conn.close()
    print(f"[DB] Initialised at {DB_FILE}")


def db_save_stability(status, light, motion, distance, dist_cm):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "INSERT INTO stability_log "
        "(timestamp, status, light, motion, distance, dist_cm) "
        "VALUES (?, ?, ?, ?, ?, ?)",
        (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
         status, light, motion, distance, dist_cm)
    )
    conn.commit()
    conn.close()


def db_save_ocr(filename, ocr_text, confidence, alert):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "INSERT INTO ocr_log "
        "(timestamp, filename, ocr_text, confidence, alert) "
        "VALUES (?, ?, ?, ?, ?)",
        (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
         filename, ocr_text, round(confidence, 4), alert)
    )
    conn.commit()
    conn.close()


def db_save_audio(filename, transcript):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "INSERT INTO audio_log "
        "(timestamp, filename, transcript, sample_rate) "
        "VALUES (?, ?, ?, ?)",
        (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
         filename, transcript, SAMPLE_RATE)
    )
    conn.commit()
    conn.close()


def db_get_recent_stability(n=50):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "SELECT timestamp, status, light, motion, distance, dist_cm "
        "FROM stability_log ORDER BY id DESC LIMIT ?", (n,)
    )
    rows = c.fetchall()
    conn.close()
    return rows


def db_get_recent_ocr(n=10):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "SELECT timestamp, filename, ocr_text, confidence, alert "
        "FROM ocr_log ORDER BY id DESC LIMIT ?", (n,)
    )
    rows = c.fetchall()
    conn.close()
    return rows


def db_get_recent_audio(n=5):
    conn = sqlite3.connect(DB_FILE)
    c    = conn.cursor()
    c.execute(
        "SELECT timestamp, filename, transcript "
        "FROM audio_log ORDER BY id DESC LIMIT ?", (n,)
    )
    rows = c.fetchall()
    conn.close()
    return rows

# ─────────────────────────────────────────────────────────────────────────────
# OCR HELPER
# ─────────────────────────────────────────────────────────────────────────────

def run_ocr(image_path):
    """
    Run EasyOCR on a saved JPEG.
    Returns (text, confidence) where confidence is 0.0–1.0 average.
    Returns ("", 0.0) if nothing detected.
    """
    try:
        results = ocr_reader.readtext(image_path, detail=1, paragraph=False)

        if not results:
            print("  [EasyOCR] No text detected.")
            return "", 0.0

        words       = []
        confidences = []

        for (bbox, text, conf) in results:
            words.append(text)
            confidences.append(conf)
            print(f"  [EasyOCR]  '{text}'  conf={conf*100:.1f}%")

        full_text      = " ".join(words).strip().lower()
        avg_confidence = sum(confidences) / len(confidences)

        print(f"  [EasyOCR] Full text : '{full_text}'")
        print(f"  [EasyOCR] Avg conf  : {avg_confidence*100:.1f}%")

        return full_text, avg_confidence

    except Exception as e:
        print(f"  [EasyOCR] Error: {e}")
        return "", 0.0

# ─────────────────────────────────────────────────────────────────────────────
# AUDIO HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def binary_pcm_to_float32(raw_bytes):
    """
    Convert raw binary int16 PCM bytes (from ESP32 /audio POST) to
    normalised float32 array for Whisper.

    K64F sends int16 samples (little-endian) via SendAudioFrame().
    ESP32 receives them and POSTs the raw bytes to /audio.
    We interpret them here as int16 little-endian, then normalise.
    """
    # Interpret raw bytes as int16 little-endian array
    samples = np.frombuffer(raw_bytes, dtype='<i2').astype(np.float32)

    # Remove DC offset (ADC has a bias around mid-scale)
    samples -= np.mean(samples)

    # Normalise to -1.0 ... +1.0
    peak = np.max(np.abs(samples))
    if peak > 0:
        samples /= peak

    return samples


def save_wav(float_samples, frame_number):
    """Save normalised float32 array as 16-bit mono WAV."""
    filename = os.path.join(RECORDINGS_DIR, f"frame_{frame_number:03d}.wav")
    pcm = (float_samples * 32767).astype(np.int16)
    wav_write(filename, SAMPLE_RATE, pcm)
    return filename


def transcribe_audio(float_samples):
    """
    Run Whisper on a float32 audio array.
    Whisper expects float32, mono, normalised.
    Our sample rate is ~12751 Hz which Whisper handles.
    Returns transcript string.
    """
    try:
        result = whisper_model.transcribe(float_samples)
        return result["text"].strip()
    except Exception as e:
        print(f"  [Whisper] Error: {e}")
        return "[transcription error]"

# ─────────────────────────────────────────────────────────────────────────────
# IN-MEMORY STATE
# ─────────────────────────────────────────────────────────────────────────────

photo_count  = 0
frame_count  = 0
server_start = datetime.datetime.now()

latest_photo = {
    "filename": None, "size": None, "ocr_text": None,
    "confidence": None, "alert": None, "timestamp": None
}
latest_frame = {
    "count": 0, "filename": None,
    "transcript": None, "timestamp": None
}

# ─────────────────────────────────────────────────────────────────────────────
# ROUTES
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/test", methods=["GET"])
def test():
    print("[GET /test] Ping received")
    return "OK", 200


@app.route("/status", methods=["GET"])
def status():
    uptime = (datetime.datetime.now() - server_start).total_seconds()
    return jsonify({
        "status":          "running",
        "frames_received": frame_count,
        "photos_received": photo_count,
        "uptime_seconds":  round(uptime, 1),
        "sample_rate":     SAMPLE_RATE,
        "whisper_model":   WHISPER_MODEL,
        "ocr_languages":   OCR_LANGUAGES,
        "conf_threshold":  CONFIDENCE_THRESHOLD,
        "recordings_dir":  RECORDINGS_DIR,
        "photos_dir":      PHOTOS_DIR
    }), 200


# ── STABILITY ENDPOINT ────────────────────────────────────────────────────────

@app.route("/stability", methods=["POST"])
def receive_stability():
    """
    Receives sensor gate events from ESP32.
    ESP32 strips "STABILITY:" prefix from K64F line before posting.
    Format: STATUS,light,motion,distance,dist_cm
    Example: READY,1,1,1,15
    """
    raw = request.data.decode("utf-8").strip()

    if not raw:
        return "empty", 400

    parts = raw.split(",")
    if len(parts) != 5:
        print(f"[STABILITY] Bad payload: '{raw}'")
        return "bad format", 400

    try:
        status   = parts[0]
        light    = int(parts[1])
        motion   = int(parts[2])
        distance = int(parts[3])
        dist_cm  = int(parts[4])
    except ValueError:
        print(f"[STABILITY] Parse error: '{raw}'")
        return "parse error", 400

    db_save_stability(status, light, motion, distance, dist_cm)

    print(f"[STABILITY] {datetime.datetime.now().strftime('%H:%M:%S')} "
          f"| {status:7} | L:{light} M:{motion} D:{distance} | {dist_cm}cm")

    return "OK", 200


# ── OCR ENDPOINT ─────────────────────────────────────────────────────────────

@app.route("/ocr", methods=["POST"])
def receive_ocr():
    """
    Receive JPEG bytes from ESP32 (Content-Type: image/jpeg).
    ESP32 posts raw camera frame buffer here.

    Pipeline:
      JPEG bytes → save → EasyOCR → confidence gate → SQLite → plain text

    Response (plain text — ESP32 forwards as TRANSCRIPT:<text> to K64F):
      HIGH confidence → OCR text  → K64F displays on LCD + scrolls Braille
      LOW  confidence → "BUZZER"  → K64F fires buzzer + shows Low Confidence
      No text found   → "BUZZER"  → treat same as low confidence
    """
    global photo_count, latest_photo

    print("\n[POST /ocr] Photo incoming...")

    raw = request.data
    if not raw or len(raw) == 0:
        print("  ERROR: Empty body")
        return "BUZZER", 200   # Return BUZZER not error — K64F handles it

    photo_count += 1
    filename     = os.path.join(PHOTOS_DIR, f"photo_{photo_count:03d}.jpg")

    with open(filename, "wb") as f:
        f.write(raw)

    print(f"  Saved  -> {filename}  ({len(raw)} bytes)")

    print("  Running EasyOCR...")
    ocr_text, confidence = run_ocr(filename)

    # Confidence gate — Milestone 5
    if not ocr_text:
        # No text detected at all → low confidence path
        alert    = "LOW"
        response = "BUZZER"
    elif confidence >= CONFIDENCE_THRESHOLD:
        alert    = "HIGH"
        response = ocr_text   # Send OCR text → K64F Braille + LCD
    else:
        alert    = "LOW"
        response = "BUZZER"   # Send BUZZER → K64F fires buzzer

    print(f"  Confidence : {confidence*100:.1f}%  "
          f"(threshold={CONFIDENCE_THRESHOLD*100:.0f}%)  -> {alert}")
    print(f"  Response   : '{response}'")

    db_save_ocr(
        os.path.basename(filename),
        ocr_text if ocr_text else "(none)",
        confidence,
        alert
    )

    latest_photo = {
        "filename":   filename,
        "size":       len(raw),
        "ocr_text":   ocr_text,
        "confidence": round(confidence, 4),
        "alert":      alert,
        "timestamp":  datetime.datetime.now().isoformat()
    }

    # Plain text — ESP32 sends this straight to K64F as TRANSCRIPT:<response>
    return response, 200, {"Content-Type": "text/plain; charset=utf-8"}


# ── AUDIO ENDPOINT ────────────────────────────────────────────────────────────

@app.route("/audio", methods=["POST"])
def receive_audio():
    """
    Receive raw int16 PCM bytes from ESP32 (Content-Type: application/octet-stream).

    K64F flow:
      ADC_ReadMic() → int16 samples → SendAudioFrame() → UART → ESP32
      ESP32 reads binary frame → POSTs raw bytes here → Whisper → plain text
      Flask returns plain text → ESP32 sends TRANSCRIPT:<text> to K64F
      K64F WaitForTranscript() gets it → LCD + Braille scroll

    Why binary not JSON:
      19000 samples as JSON would be ~150KB of text over UART.
      Binary is 38000 bytes — half the size, no parsing overhead.
    """
    global frame_count, latest_frame

    print("\n[POST /audio] Audio frame incoming...")

    raw = request.data
    if not raw or len(raw) == 0:
        print("  ERROR: Empty body")
        return "Error", 200

    # Expect int16 samples — must be even number of bytes
    if len(raw) % 2 != 0:
        print(f"  WARNING: Odd byte count {len(raw)} — trimming last byte")
        raw = raw[:-1]

    sample_count = len(raw) // 2
    print(f"  Received   : {len(raw)} bytes = {sample_count} int16 samples")

    frame_count += 1
    float_samples = binary_pcm_to_float32(raw)
    filename      = save_wav(float_samples, frame_count)

    print(f"  Saved WAV  -> {filename}")
    print(f"  Transcribing with Whisper '{WHISPER_MODEL}'...")

    transcript = transcribe_audio(float_samples)
    print(f"  Transcript : '{transcript}'")

    db_save_audio(os.path.basename(filename), transcript)

    latest_frame = {
        "count":      sample_count,
        "filename":   filename,
        "transcript": transcript,
        "timestamp":  datetime.datetime.now().isoformat()
    }

    # Plain text — ESP32 sends as TRANSCRIPT:<transcript> to K64F
    # K64F WaitForTranscript() parses it and displays on LCD + Braille
    return transcript, 200, {"Content-Type": "text/plain; charset=utf-8"}


# ── JSON API ENDPOINTS ────────────────────────────────────────────────────────

@app.route("/api/events")
def api_events():
    rows = db_get_recent_stability(50)
    return jsonify([
        {
            "timestamp": r[0], "status":   r[1],
            "light":     r[2], "motion":   r[3],
            "distance":  r[4], "dist_cm":  r[5]
        }
        for r in rows
    ])


@app.route("/api/ocr")
def api_ocr():
    rows = db_get_recent_ocr(10)
    return jsonify([
        {
            "timestamp":  r[0], "filename":   r[1],
            "ocr_text":   r[2], "confidence": r[3],
            "alert":      r[4]
        }
        for r in rows
    ])


@app.route("/api/audio")
def api_audio():
    rows = db_get_recent_audio(5)
    return jsonify([
        {"timestamp": r[0], "filename": r[1], "transcript": r[2]}
        for r in rows
    ])


# ─────────────────────────────────────────────────────────────────────────────
# DASHBOARD
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/")
def dashboard():
    threshold_pct = int(CONFIDENCE_THRESHOLD * 100)
    return f"""<!DOCTYPE html>
<html>
<head>
    <title>SEP600 Dashboard</title>
    <meta charset="UTF-8">
    <style>
        * {{ box-sizing: border-box; margin: 0; padding: 0; }}
        body {{
            font-family: 'Courier New', monospace;
            background: #0d1117;
            color: #e6edf3;
            padding: 20px;
        }}
        h1 {{
            text-align: center;
            color: #58a6ff;
            margin-bottom: 20px;
            font-size: 1.4em;
            letter-spacing: 2px;
        }}
        h2 {{
            color: #8b949e;
            font-size: 0.78em;
            letter-spacing: 2px;
            margin: 26px 0 10px 0;
            text-transform: uppercase;
        }}
        .lcd-container {{
            display: flex;
            justify-content: center;
            margin-bottom: 20px;
        }}
        .lcd {{
            background: #1a3a1a;
            border: 3px solid #2ea043;
            border-radius: 8px;
            padding: 16px 24px;
            width: 420px;
            box-shadow: 0 0 20px #2ea04355;
        }}
        .lcd-label {{ color: #2ea043; font-size: 0.7em; letter-spacing: 3px; margin-bottom: 8px; }}
        .lcd-line {{
            background: #0f2a0f;
            color: #4dff91;
            font-size: 1.3em;
            letter-spacing: 4px;
            padding: 6px 10px;
            margin: 4px 0;
            border-radius: 4px;
            min-height: 36px;
        }}
        .indicators {{
            display: flex;
            justify-content: center;
            gap: 12px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }}
        .indicator {{
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 8px;
            padding: 10px 16px;
            text-align: center;
            min-width: 95px;
        }}
        .indicator .label {{ font-size: 0.66em; color: #8b949e; letter-spacing: 2px; margin-bottom: 5px; }}
        .indicator .value {{ font-size: 1.3em; font-weight: bold; }}
        .ok   {{ color: #2ea043; }}
        .fail {{ color: #f85149; }}
        .blue {{ color: #58a6ff; }}
        .na   {{ color: #8b949e; }}
        .meter-wrap {{
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 10px;
            padding: 18px;
            margin-bottom: 14px;
        }}
        .meter-label {{
            display: flex;
            justify-content: space-between;
            font-size: 0.78em;
            color: #8b949e;
            margin-bottom: 8px;
        }}
        .meter-track {{
            background: #21262d;
            border-radius: 6px;
            height: 22px;
            overflow: hidden;
            position: relative;
        }}
        .meter-fill {{ height: 100%; border-radius: 6px; transition: width 0.6s ease, background 0.4s; }}
        .meter-threshold {{
            position: absolute;
            top: 0; bottom: 0;
            width: 2px;
            background: #f85149;
            opacity: 0.8;
        }}
        .meter-pct {{ text-align: center; font-size: 1.5em; font-weight: bold; margin-top: 8px; }}
        .alert-badge {{
            display: inline-block;
            padding: 3px 12px;
            border-radius: 20px;
            font-size: 0.8em;
            font-weight: bold;
            letter-spacing: 1px;
        }}
        .badge-high {{ background: #1f4d2b; color: #2ea043; border: 1px solid #2ea043; }}
        .badge-low  {{ background: #3d1a1a; color: #f85149; border: 1px solid #f85149; }}
        .chart-container {{
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 8px;
            padding: 14px;
            height: 100px;
            display: flex;
            align-items: flex-end;
            gap: 3px;
            overflow: hidden;
        }}
        .bar {{
            flex: 1;
            border-radius: 2px 2px 0 0;
            min-width: 4px;
            transition: height 0.3s;
        }}
        .ocr-card {{
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 8px;
            padding: 14px 18px;
            margin-bottom: 8px;
        }}
        .ocr-card.card-high {{ border-left: 4px solid #2ea043; }}
        .ocr-card.card-low  {{ border-left: 4px solid #f85149; }}
        .ocr-meta {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
            flex-wrap: wrap;
            gap: 6px;
        }}
        .ocr-time {{ color: #8b949e; font-size: 0.74em; }}
        .ocr-file {{ color: #58a6ff; font-size: 0.74em; }}
        .ocr-text-box {{
            background: #0d1117;
            border-radius: 6px;
            padding: 10px 12px;
            font-size: 0.88em;
            line-height: 1.6;
            white-space: pre-wrap;
        }}
        .ocr-conf-line {{ margin-top: 8px; font-size: 0.76em; color: #8b949e; }}
        table {{ width: 100%; border-collapse: collapse; font-size: 0.82em; }}
        th {{
            background: #161b22;
            color: #8b949e;
            padding: 8px 12px;
            text-align: left;
            border-bottom: 1px solid #30363d;
            letter-spacing: 1px;
        }}
        td {{ padding: 7px 12px; border-bottom: 1px solid #21262d; }}
        tr:hover td {{ background: #161b22; }}
        .ready   {{ color: #2ea043; font-weight: bold; }}
        .blocked {{ color: #f85149; }}
        .audio-card {{
            background: #161b22;
            border: 1px solid #30363d;
            border-left: 4px solid #58a6ff;
            border-radius: 8px;
            padding: 12px 16px;
            margin-bottom: 8px;
            font-size: 0.85em;
        }}
        .audio-meta {{ color: #8b949e; font-size: 0.76em; margin-bottom: 4px; }}
        .refresh-note {{ text-align: center; color: #484f58; font-size: 0.68em; margin-top: 24px; }}
    </style>
</head>
<body>

<h1>SEP600 — LIVE DASHBOARD</h1>

<!-- LCD Simulation -->
<div class="lcd-container">
    <div class="lcd">
        <div class="lcd-label">LCD DISPLAY</div>
        <div class="lcd-line" id="lcd0">  Waiting...     </div>
        <div class="lcd-line" id="lcd1">                 </div>
    </div>
</div>

<!-- Indicators -->
<div class="indicators">
    <div class="indicator">
        <div class="label">LIGHT</div>
        <div class="value" id="ind-light">-</div>
    </div>
    <div class="indicator">
        <div class="label">MOTION</div>
        <div class="value" id="ind-motion">-</div>
    </div>
    <div class="indicator">
        <div class="label">DISTANCE</div>
        <div class="value blue" id="ind-dist">-</div>
    </div>
    <div class="indicator">
        <div class="label">GATE</div>
        <div class="value" id="ind-gate">-</div>
    </div>
    <div class="indicator">
        <div class="label">OCR CONF</div>
        <div class="value" id="ind-conf">-</div>
    </div>
    <div class="indicator">
        <div class="label">M5 ALERT</div>
        <div class="value" id="ind-alert">-</div>
    </div>
</div>

<!-- Confidence Meter -->
<h2>OCR Confidence — Latest Capture</h2>
<div class="meter-wrap">
    <div class="meter-label">
        <span>0%</span>
        <span id="meter-text">Waiting for capture...</span>
        <span>100%</span>
    </div>
    <div class="meter-track">
        <div class="meter-fill" id="meter-fill" style="width:0%;background:#484f58"></div>
        <div class="meter-threshold" style="left:{threshold_pct}%"
             title="Threshold {threshold_pct}%"></div>
    </div>
    <div class="meter-pct na" id="meter-pct">—</div>
</div>

<!-- Distance Chart -->
<h2>Distance History (last 50 readings)</h2>
<div class="chart-container" id="dist-chart"></div>

<!-- Confidence History Chart -->
<h2>OCR Confidence History (last 10 captures)</h2>
<div class="chart-container" id="conf-chart"></div>

<!-- Stability Log Table -->
<h2>Stability Event Log</h2>
<table>
    <thead>
        <tr>
            <th>TIME</th><th>STATUS</th><th>LIGHT</th>
            <th>MOTION</th><th>DIST</th><th>DIST CM</th>
        </tr>
    </thead>
    <tbody id="event-log"></tbody>
</table>

<!-- OCR Cards -->
<h2>OCR Event Log (Milestone 5)</h2>
<div id="ocr-cards"></div>

<!-- Audio Log -->
<h2>Audio Transcript Log (Whisper)</h2>
<div id="audio-log"></div>

<div class="refresh-note">Auto-refreshes every 2 seconds</div>

<script>
const THRESHOLD = {threshold_pct};

function getLcdLine(e) {{
    if (e.status === "READY")
        return ["Conditions Met! ", "Taking Pic      "];
    if (!e.light)
        return ["Low Light!      ", "Lgt:0           "];
    if (!e.motion)
        return ["Hold Steady!    ", "Do not move     "];
    if (e.dist_cm > 30)
        return ["Move Closer!    ", "Keep <30cm      "];
    if (e.dist_cm > 0 && e.dist_cm < 7)
        return ["Too Close! Back ", "Back up >7cm    "];
    return ["Sensor Error!   ", "Check sensors   "];
}}

async function refresh() {{
    try {{
        /* ── Stability events ── */
        const stabRes = await fetch('/api/events');
        const events  = await stabRes.json();

        if (events.length > 0) {{
            const e = events[0];

            const lines = getLcdLine(e);
            document.getElementById('lcd0').textContent = lines[0];
            document.getElementById('lcd1').textContent = lines[1];

            const lEl = document.getElementById('ind-light');
            lEl.textContent = e.light ? 'OK' : 'DARK';
            lEl.className = 'value ' + (e.light ? 'ok' : 'fail');

            const mEl = document.getElementById('ind-motion');
            mEl.textContent = e.motion ? 'STABLE' : 'MOVING';
            mEl.className = 'value ' + (e.motion ? 'ok' : 'fail');

            document.getElementById('ind-dist').textContent = e.dist_cm + ' cm';

            const gEl = document.getElementById('ind-gate');
            gEl.textContent = e.status;
            gEl.className = 'value ' + (e.status === 'READY' ? 'ok' : 'fail');

            const tbody = document.getElementById('event-log');
            tbody.innerHTML = '';
            events.slice(0, 20).forEach(ev => {{
                const cls = ev.status === 'READY' ? 'ready' : 'blocked';
                tbody.innerHTML += `<tr>
                    <td>${{ev.timestamp.split(' ')[1]}}</td>
                    <td class="${{cls}}">${{ev.status}}</td>
                    <td class="${{ev.light  ? 'ok' : 'fail'}}">${{ev.light  ? 'OK' : 'DARK'}}</td>
                    <td class="${{ev.motion ? 'ok' : 'fail'}}">${{ev.motion ? 'STABLE' : 'MOVING'}}</td>
                    <td class="${{ev.distance ? 'ok' : 'fail'}}">${{ev.distance ? 'OK' : 'BAD'}}</td>
                    <td>${{ev.dist_cm}} cm</td>
                </tr>`;
            }});

            const dChart = document.getElementById('dist-chart');
            dChart.innerHTML = '';
            [...events].reverse().forEach(ev => {{
                const h = Math.min((ev.dist_cm / 35) * 100, 100);
                const color = (ev.dist_cm >= 7 && ev.dist_cm <= 30) ? '#2ea043' : '#f85149';
                dChart.innerHTML +=
                    `<div class="bar" style="height:${{h}}%;background:${{color}}"></div>`;
            }});
        }}

        /* ── OCR results ── */
        const ocrRes = await fetch('/api/ocr');
        const ocrs   = await ocrRes.json();

        if (ocrs.length > 0) {{
            const latest = ocrs[0];
            const pct    = latest.confidence ? (latest.confidence * 100).toFixed(1) : '0.0';
            const isHigh = latest.alert === 'HIGH';

            const cEl = document.getElementById('ind-conf');
            cEl.textContent = pct + '%';
            cEl.className   = 'value ' + (isHigh ? 'ok' : 'fail');

            const aEl = document.getElementById('ind-alert');
            aEl.textContent = latest.alert || '-';
            aEl.className   = 'value ' + (isHigh ? 'ok' : 'fail');

            const fill  = document.getElementById('meter-fill');
            const pctEl = document.getElementById('meter-pct');
            const txtEl = document.getElementById('meter-text');
            fill.style.width      = pct + '%';
            fill.style.background = isHigh ? '#2ea043' : '#f85149';
            pctEl.textContent  = pct + '%';
            pctEl.className    = 'meter-pct ' + (isHigh ? 'ok' : 'fail');
            txtEl.textContent  = isHigh
                ? '\u2713 High confidence \u2014 Braille displayed'
                : '\u2717 Low confidence \u2014 Buzzer triggered';

            const cChart = document.getElementById('conf-chart');
            cChart.innerHTML = '';
            [...ocrs].reverse().forEach(r => {{
                const h     = r.confidence ? (r.confidence * 100) : 0;
                const color = (r.alert === 'HIGH') ? '#2ea043' : '#f85149';
                cChart.innerHTML +=
                    `<div class="bar" style="height:${{h}}%;background:${{color}}"
                          title="${{r.timestamp}} | ${{h.toFixed(1)}}% | ${{r.alert}}"></div>`;
            }});

            const cards = document.getElementById('ocr-cards');
            cards.innerHTML = '';
            ocrs.forEach(r => {{
                const p    = r.confidence ? (r.confidence * 100).toFixed(1) : '0.0';
                const hi   = r.alert === 'HIGH';
                const bCls = hi ? 'badge-high' : 'badge-low';
                const cCls = hi ? 'card-high'  : 'card-low';
                const bTxt = hi ? '\u2713 HIGH' : '\u2717 LOW \u2014 BUZZER';
                const txt  = r.ocr_text || '(no text detected)';
                cards.innerHTML += `
                <div class="ocr-card ${{cCls}}">
                    <div class="ocr-meta">
                        <span class="ocr-time">${{r.timestamp}}</span>
                        <span class="ocr-file">${{r.filename}}</span>
                        <span class="alert-badge ${{bCls}}">${{bTxt}}</span>
                    </div>
                    <div class="ocr-text-box">${{txt}}</div>
                    <div class="ocr-conf-line">
                        Confidence: <b>${{p}}%</b>
                        &nbsp;|&nbsp; Threshold: ${{THRESHOLD}}%
                        &nbsp;|&nbsp; Words: ${{r.ocr_text ? r.ocr_text.split(' ').length : 0}}
                    </div>
                </div>`;
            }});
        }}

        /* ── Audio log ── */
        const audRes = await fetch('/api/audio');
        const audios = await audRes.json();
        const audDiv = document.getElementById('audio-log');
        audDiv.innerHTML = '';
        if (audios.length === 0) {{
            audDiv.innerHTML =
                '<div class="audio-card"><div class="audio-meta">No audio recordings yet</div></div>';
        }} else {{
            audios.forEach(a => {{
                audDiv.innerHTML += `
                <div class="audio-card">
                    <div class="audio-meta">${{a.timestamp}} \u2014 ${{a.filename}}</div>
                    ${{a.transcript}}
                </div>`;
            }});
        }}

    }} catch(err) {{
        console.error('Dashboard refresh error:', err);
    }}
}}

refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>""", 200


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    init_db()

    print("=" * 55)
    print("SEP600 M5 — Flask Server + Dashboard")
    print(f"Whisper model     : {WHISPER_MODEL}")
    print(f"OCR languages     : {OCR_LANGUAGES}")
    print(f"Confidence thresh : {CONFIDENCE_THRESHOLD*100:.0f}%")
    print(f"Sample rate       : {SAMPLE_RATE} Hz")
    print(f"Photos dir        : {PHOTOS_DIR}")
    print(f"Recordings dir    : {RECORDINGS_DIR}")
    print(f"Database          : {DB_FILE}")
    print("Endpoints:")
    print("  POST /ocr        — JPEG bytes → EasyOCR → plain text")
    print("  POST /audio      — raw int16 PCM bytes → Whisper → plain text")
    print("  POST /stability  — CSV sensor gate events → SQLite")
    print("  GET  /           — live dashboard")
    print("  GET  /api/events — stability log JSON")
    print("  GET  /api/ocr    — OCR log JSON")
    print("  GET  /api/audio  — audio log JSON")
    print("  GET  /test       — ping")
    print("  GET  /status     — uptime + config")
    print("Dashboard: http://localhost:5000")
    print("=" * 55)

    app.run(host="0.0.0.0", port=5000, debug=False)
