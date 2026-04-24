# ME-301 Measurement Lab — Setup Guide

## 1. Python setup (one time)
```bash
pip install -r requirements.txt
python measurement_app.py
```

## 2. Arduino wiring additions
| Component        | Arduino Pin |
|-----------------|-------------|
| Sync LED + 220Ω | D4 → GND    |
| Sync Button     | D5 → GND    |

The button uses the internal pull-up — no resistor needed.

## 3. Workflow

### Capture tab
1. Select the serial port → Connect
2. Click **Start Recording**
3. Press the **physical button on D5** (or click ⚡ Trigger Sync LED in the app)
   → LED flashes, a `S,<timestamp>` marker is saved in the data stream
4. Run your experiment
5. Click **Stop Recording** → **Save CSV**

### Analyse tab
1. Load your saved CSV (or click "Use live data")
2. Load your video file
3. Set sphere diameter and ruler length
4. Choose sync mode:
   - **Auto**: detects the LED flash frame automatically
   - **Manual**: enter offset in seconds if auto fails
5. Click **▶ Run Full Analysis**
   - OpenCV windows will pop up for calibration:
     - Click both ends of the ruler segment
     - Click the thread attachment point
     - Draw ROI around sphere motion area
6. Results appear with 6 synchronized plots
7. Export PNG for your report

## 4. macOS note
OpenCV GUI windows require the main thread — this is handled automatically.
If windows don't appear, make sure you're not running in a headless environment.

## 5. Serial data format (for reference)
```
D,<millis>,<weight_kg>,<accel_x>,<accel_y>,<accel_z>,<tilt_x>,<tilt_y>
S,<millis>    ← sync event (LED flash)
```
