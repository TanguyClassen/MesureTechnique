#!/usr/bin/env python3
"""
ME-301 Measurement Techniques
Integrated Arduino Capture + Pendulum Video Analysis
────────────────────────────────────────────────────
Run:  python measurement_app.py
Deps: pip install pyserial opencv-python numpy matplotlib
"""

# ══════════════════════════════════════════════════════════════════════════════
# IMPORTS
# ══════════════════════════════════════════════════════════════════════════════
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import queue
import time
import os
import csv
from datetime import datetime

import numpy as np
import cv2

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

try:
    import serial
    import serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

# ══════════════════════════════════════════════════════════════════════════════
# THEME
# ══════════════════════════════════════════════════════════════════════════════
BG      = "#0f1117"
SURFACE = "#1a1d27"
CARD    = "#21253a"
BORDER  = "#2e3250"
ACCENT  = "#4f8ef7"
GREEN   = "#3ecf8e"
ORANGE  = "#f5a623"
RED     = "#e05c5c"
PURPLE  = "#9b7fe8"
TEXT    = "#e8eaf0"
MUTED   = "#6b7280"
BAUD    = 115200
MAX_PTS = 600   # ~30 s at 20 Hz

# ══════════════════════════════════════════════════════════════════════════════
# DATA STORE  (thread-safe)
# ══════════════════════════════════════════════════════════════════════════════
class DataStore:
    def __init__(self):
        self._lock   = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self.ts       = []   # millis from Arduino
            self.weight   = []
            self.ax, self.ay, self.az = [], [], []
            self.tx, self.ty          = [], []
            self.sync_ts  = []   # millis of sync events
            self.recording = False

    def add(self, ts, w, ax, ay, az, tx, ty):
        with self._lock:
            self.ts.append(ts);    self.weight.append(w)
            self.ax.append(ax);    self.ay.append(ay);  self.az.append(az)
            self.tx.append(tx);    self.ty.append(ty)

    def add_sync(self, ts):
        with self._lock:
            self.sync_ts.append(ts)

    def snapshot(self):
        """Return numpy arrays (thread-safe copy)."""
        with self._lock:
            t  = np.array(self.ts,     dtype=float) / 1000.0
            w  = np.array(self.weight, dtype=float)
            ax = np.array(self.ax,     dtype=float)
            ay = np.array(self.ay,     dtype=float)
            az = np.array(self.az,     dtype=float)
            tx = np.array(self.tx,     dtype=float)
            ty = np.array(self.ty,     dtype=float)
            st = list(self.sync_ts)
        return t, w, ax, ay, az, tx, ty, st

    def count(self):
        with self._lock:
            return len(self.ts)

    def sync_count(self):
        with self._lock:
            return len(self.sync_ts)

    def save_csv(self, path):
        with self._lock:
            sync_set = set(self.sync_ts)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['time_ms','weight_kg','accel_x','accel_y',
                            'accel_z','tilt_x','tilt_y','sync'])
                for i, ts in enumerate(self.ts):
                    w.writerow([ts, self.weight[i],
                                self.ax[i], self.ay[i], self.az[i],
                                self.tx[i], self.ty[i],
                                1 if ts in sync_set else 0])
        print(f"[CSV] Saved {len(self.ts)} rows → {path}")


# ══════════════════════════════════════════════════════════════════════════════
# SERIAL READER THREAD
# ══════════════════════════════════════════════════════════════════════════════
class SerialReader(threading.Thread):
    """
    Reads lines from Arduino serial.
    Parses:
      D,<ms>,<w>,<ax>,<ay>,<az>,<tx>,<ty>   → sensor data
      S,<ms>                                  → sync event
    """
    def __init__(self, port, store, q):
        super().__init__(daemon=True)
        self.port  = port
        self.store = store
        self.q     = q
        self._stop = threading.Event()
        self._ser  = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, BAUD, timeout=1)
            self.q.put(("status", f"Connected  {self.port}"))
            while not self._stop.is_set():
                raw = self._ser.readline()
                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue
                if not line:
                    continue
                if line.startswith('D,'):
                    parts = line.split(',')
                    if len(parts) == 8:
                        try:
                            vals = list(map(float, parts[1:]))
                            if self.store.recording:
                                self.store.add(*vals)
                            self.q.put(("data", vals))
                        except ValueError:
                            pass
                elif line.startswith('S,'):
                    parts = line.split(',')
                    if len(parts) == 2:
                        try:
                            ts = float(parts[1])
                            if self.store.recording:
                                self.store.add_sync(ts)
                            self.q.put(("sync", ts))
                        except ValueError:
                            pass
        except Exception as e:
            self.q.put(("error", str(e)))
        finally:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self.q.put(("disconnected", ""))

    def send(self, cmd: str):
        if self._ser and self._ser.is_open:
            self._ser.write((cmd + '\n').encode())

    def stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()


# ══════════════════════════════════════════════════════════════════════════════
# CV HELPERS  (replicate interactive calibration from original script)
# ══════════════════════════════════════════════════════════════════════════════
def cv_click_two_points(frame, title, msg):
    pts, disp = [], frame.copy()
    def cb(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 2:
            pts.append((x, y))
            cv2.circle(disp, (x,y), 6, (0,255,255), -1)
            if len(pts)==2: cv2.line(disp, pts[0], pts[1], (0,255,255), 2)
            cv2.imshow(title, disp)
    cv2.putText(disp, msg, (10,32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, disp)
    cv2.setMouseCallback(title, cb)
    while len(pts) < 2: cv2.waitKey(20)
    cv2.waitKey(400); cv2.destroyWindow(title)
    return pts

def cv_click_one_point(frame, title, msg):
    pts, disp = [], frame.copy()
    def cb(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN and not pts:
            pts.append((x, y))
            cv2.circle(disp, (x,y), 6, (0,255,0), -1)
            cv2.imshow(title, disp)
    cv2.putText(disp, msg, (10,32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, disp)
    cv2.setMouseCallback(title, cb)
    while not pts: cv2.waitKey(20)
    cv2.waitKey(400); cv2.destroyWindow(title)
    return pts[0]

def cv_select_roi(frame, title="Select ROI — press ENTER when done"):
    roi = cv2.selectROI(title, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(title)
    x, y, w, h = roi
    if w == 0 or h == 0:
        raise RuntimeError("ROI selection cancelled.")
    return int(x), int(y), int(w), int(h)

def cv_detect_circles(frame, min_r, max_r):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9,9), 2)
    gray = cv2.equalizeHist(gray)
    c = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,
                         dp=1.2, minDist=40, param1=80, param2=28,
                         minRadius=min_r, maxRadius=max_r)
    if c is None: return []
    return [(int(x), int(y), int(r)) for x,y,r in c[0]]

def cv_best_circle(candidates, att, L_exp, prev_c, prev_r, L_tol):
    if not candidates: return None
    ax, ay = att
    best_s, best = -1e9, None
    for cx, cy, r in candidates:
        L = np.sqrt((cx-ax)**2 + (cy-ay)**2)
        s = 0.0
        if cy < ay: s -= 500
        if L_exp:   s -= abs(L - L_exp)
        if prev_c:  s -= 1.5 * np.sqrt((cx-prev_c[0])**2 + (cy-prev_c[1])**2)
        if prev_r:  s -= 2.0 * abs(r - prev_r)
        s += 0.5 * r
        if s > best_s: best_s, best = s, (cx, cy, r)
    return best


# ══════════════════════════════════════════════════════════════════════════════
# PENDULUM DETECTOR  (runs in background thread)
# ══════════════════════════════════════════════════════════════════════════════
def detect_led_flash(video_path):
    """Return (frame_idx, time_s) of first brightness spike, or (None, None)."""
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    brightness = []
    while True:
        ret, f = cap.read()
        if not ret: break
        brightness.append(float(np.mean(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))))
    cap.release()
    if len(brightness) < 5: return None, None
    b = np.array(brightness)
    thr = np.mean(b) + 3*np.std(b)
    spikes = np.where(b > thr)[0]
    if len(spikes) == 0: return None, None
    fi = int(spikes[0])
    return fi, fi/fps

def run_pendulum_detection(video_path, params, progress_cb):
    """
    params dict keys:
      resize, K, attachment, min_r, max_r, roi, L_expected, L_tol
    Returns dict: times, angles, L_mm, fps, n_failed, n_total
    """
    RESIZE = params['resize']
    K      = params['K']
    att    = params['attachment']
    min_r  = params['min_r']
    max_r  = params['max_r']
    roi    = params['roi']
    L_exp  = params.get('L_expected')
    L_tol  = params.get('L_tol', 40)

    cap   = cv2.VideoCapture(video_path)
    fps   = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # ── First pass: estimate pendulum length ──────────────────────────────────
    if L_exp is None:
        raw_L = []
        while True:
            ret, frame = cap.read()
            if not ret: break
            small = cv2.resize(frame, (0,0), fx=RESIZE, fy=RESIZE)
            rx,ry,rw,rh = roi
            crop = small[ry:ry+rh, rx:rx+rw]
            cands = [(cx+rx, cy+ry, r) for cx,cy,r in cv_detect_circles(crop, min_r, max_r)]
            best = cv_best_circle(cands, att, None, None, None, L_tol)
            if best:
                cx, cy, _ = best
                if cy > att[1]:
                    raw_L.append(np.sqrt((cx-att[0])**2 + (cy-att[1])**2))
        L_exp = float(np.median(raw_L)) if raw_L else 100.0
        L_tol = 0.15 * L_exp
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # ── Second pass: full extraction ──────────────────────────────────────────
    times, angles, L_mm_list = [], [], []
    prev_c, prev_r = None, None
    n_fail = 0
    fi = 0

    while True:
        ret, frame = cap.read()
        if not ret: break
        small = cv2.resize(frame, (0,0), fx=RESIZE, fy=RESIZE)
        rx,ry,rw,rh = roi
        crop  = small[ry:ry+rh, rx:rx+rw]
        cands = [(cx+rx, cy+ry, r) for cx,cy,r in cv_detect_circles(crop, min_r, max_r)]
        best  = cv_best_circle(cands, att, L_exp, prev_c, prev_r, L_tol)

        if best:
            cx, cy, r = best
            L = np.sqrt((cx-att[0])**2 + (cy-att[1])**2)
            if cy > att[1] and abs(L - L_exp) < L_tol:
                theta = np.degrees(np.arctan2(cx-att[0], cy-att[1]))
                times.append(fi / fps)
                angles.append(theta)
                L_mm_list.append(L * K)
                prev_c, prev_r = (cx,cy), r
            else:
                n_fail += 1
        else:
            n_fail += 1

        if fi % 15 == 0:
            progress_cb(fi / total * 100)
        fi += 1

    cap.release()
    return {
        'times':   np.array(times),
        'angles':  np.array(angles),
        'L_mm':    np.array(L_mm_list),
        'fps':     fps,
        'n_failed': n_fail,
        'n_total':  fi,
    }


# ══════════════════════════════════════════════════════════════════════════════
# MATPLOTLIB STYLE HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def style_axes(axes, fig):
    fig.patch.set_facecolor(BG)
    for ax in (axes if hasattr(axes, '__iter__') else [axes]):
        ax.set_facecolor(CARD)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.xaxis.label.set_color(MUTED)
        ax.yaxis.label.set_color(MUTED)
        ax.title.set_color(TEXT)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.grid(True, ls=':', lw=0.5, alpha=0.4, color=BORDER)


# ══════════════════════════════════════════════════════════════════════════════
# TAB 1 — CAPTURE
# ══════════════════════════════════════════════════════════════════════════════
class CaptureTab(ttk.Frame):
    def __init__(self, parent, store):
        super().__init__(parent)
        self.store  = store
        self.q      = queue.Queue()
        self.reader = None
        self._live_t, self._live_w, self._live_n = [], [], []
        self._build()
        self._poll()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build(self):
        # Connection
        conn = ttk.LabelFrame(self, text="  Connection  ")
        conn.pack(fill='x', padx=12, pady=(10,4))

        ttk.Label(conn, text="Port:").grid(row=0, column=0, padx=8, pady=6)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(conn, textvariable=self.port_var, width=20, state='readonly')
        self.port_cb.grid(row=0, column=1, padx=4)
        ttk.Button(conn, text="↻ Refresh", command=self._refresh).grid(row=0, column=2, padx=6)
        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=3, padx=10)
        self._refresh()

        # Recording controls
        ctrl = ttk.LabelFrame(self, text="  Recording  ")
        ctrl.pack(fill='x', padx=12, pady=4)

        self.rec_btn  = ttk.Button(ctrl, text="⏺  Start Recording",  command=self._toggle_rec,  state='disabled')
        self.sync_btn = ttk.Button(ctrl, text="⚡ Trigger Sync LED", command=self._trigger_sync, state='disabled')
        self.save_btn = ttk.Button(ctrl, text="💾 Save CSV",          command=self._save,         state='disabled')
        self.clr_btn  = ttk.Button(ctrl, text="🗑  Clear",             command=self._clear)
        for i, b in enumerate([self.rec_btn, self.sync_btn, self.save_btn, self.clr_btn]):
            b.grid(row=0, column=i, padx=10, pady=7)

        # Status strip
        sf = ttk.Frame(self)
        sf.pack(fill='x', padx=14, pady=2)
        self.sv_status = tk.StringVar(value="Disconnected")
        self.sv_count  = tk.StringVar(value="0 samples")
        self.sv_sync   = tk.StringVar(value="0 sync events")
        ttk.Label(sf, textvariable=self.sv_status, foreground=MUTED).pack(side='left')
        ttk.Label(sf, textvariable=self.sv_count).pack(side='left', padx=30)
        ttk.Label(sf, textvariable=self.sv_sync, foreground=ORANGE).pack(side='left')

        # Live plot
        pf = ttk.LabelFrame(self, text="  Live Data  ")
        pf.pack(fill='both', expand=True, padx=12, pady=6)

        self.fig_live = Figure(figsize=(9, 4), facecolor=BG)
        self.ax_w = self.fig_live.add_subplot(211)
        self.ax_n = self.fig_live.add_subplot(212, sharex=self.ax_w)
        style_axes([self.ax_w, self.ax_n], self.fig_live)
        self.ax_w.set_ylabel("Weight  kg", fontsize=8)
        self.ax_n.set_ylabel("|a|  m/s²",  fontsize=8)
        self.ax_n.set_xlabel("Time  s",    fontsize=8)
        self.line_w, = self.ax_w.plot([], [], color=ACCENT,  lw=1)
        self.line_n, = self.ax_n.plot([], [], color=GREEN,   lw=1)
        self.fig_live.tight_layout(pad=1.8)

        self.canvas_live = FigureCanvasTkAgg(self.fig_live, pf)
        self.canvas_live.get_tk_widget().pack(fill='both', expand=True)

    # ── Actions ───────────────────────────────────────────────────────────────
    def _refresh(self):
        if not SERIAL_OK:
            self.port_cb['values'] = ['pyserial not installed']; return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports: self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.reader and self.reader.is_alive():
            self.reader.stop()
            self.conn_btn.config(text="Connect")
            self.rec_btn.config(state='disabled')
            self.sync_btn.config(state='disabled')
        else:
            if not SERIAL_OK:
                messagebox.showerror("Missing library", "Run:  pip install pyserial"); return
            port = self.port_var.get()
            if not port: messagebox.showwarning("No port", "Select a port first."); return
            self.reader = SerialReader(port, self.store, self.q)
            self.reader.start()
            self.conn_btn.config(text="Disconnect")
            self.rec_btn.config(state='normal')
            self.sync_btn.config(state='normal')

    def _toggle_rec(self):
        if not self.store.recording:
            self.store.recording = True
            self.rec_btn.config(text="⏹  Stop Recording")
            self.save_btn.config(state='disabled')
            self.sv_status.set("Recording…")
        else:
            self.store.recording = False
            self.rec_btn.config(text="⏺  Start Recording")
            self.save_btn.config(state='normal')
            self.sv_status.set(f"Stopped — {self.store.count()} samples")

    def _trigger_sync(self):
        if self.reader: self.reader.send("SYNC")

    def _save(self):
        fn = f"experiment_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                   filetypes=[("CSV","*.csv")], initialfile=fn)
        if path:
            self.store.save_csv(path)
            messagebox.showinfo("Saved", f"Saved to:\n{path}")

    def _clear(self):
        if messagebox.askyesno("Clear", "Clear all recorded data?"):
            self.store.reset()
            self._live_t.clear(); self._live_w.clear(); self._live_n.clear()
            self.sv_count.set("0 samples")
            self.sv_sync.set("0 sync events")
            self._redraw_live()

    # ── Queue polling + live plot ─────────────────────────────────────────────
    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "status":
                    self.sv_status.set(payload)
                elif kind == "data":
                    ts,w,ax,ay,az,_,_ = payload
                    norm = np.sqrt(ax**2+ay**2+az**2)
                    self._live_t.append(ts/1000.0)
                    self._live_w.append(w); self._live_n.append(norm)
                    if len(self._live_t) > MAX_PTS:
                        self._live_t.pop(0); self._live_w.pop(0); self._live_n.pop(0)
                    self.sv_count.set(f"{self.store.count()} samples")
                    self._redraw_live()
                elif kind == "sync":
                    self.sv_sync.set(f"{self.store.sync_count()} sync events")
                elif kind == "error":
                    messagebox.showerror("Serial error", payload)
                elif kind == "disconnected":
                    self.sv_status.set("Disconnected")
        except queue.Empty:
            pass
        self.after(60, self._poll)

    def _redraw_live(self):
        if not self._live_t: return
        self.line_w.set_data(self._live_t, self._live_w)
        self.line_n.set_data(self._live_t, self._live_n)
        for ax in [self.ax_w, self.ax_n]: ax.relim(); ax.autoscale_view()
        self.canvas_live.draw_idle()


# ══════════════════════════════════════════════════════════════════════════════
# TAB 2 — ANALYSE
# ══════════════════════════════════════════════════════════════════════════════
class AnalyseTab(ttk.Frame):
    def __init__(self, parent, store):
        super().__init__(parent)
        self.store      = store
        self.csv_data   = None
        self.video_path = None
        self.vid_results = None
        self._build()

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build(self):
        # File loading
        ff = ttk.LabelFrame(self, text="  Data Files  ")
        ff.pack(fill='x', padx=12, pady=(10,4))

        self.sv_csv = tk.StringVar(value="No CSV loaded")
        self.sv_vid = tk.StringVar(value="No video loaded")

        ttk.Button(ff, text="📂 Load CSV",       command=self._load_csv).grid(row=0,column=0,padx=10,pady=6)
        ttk.Label(ff, textvariable=self.sv_csv, foreground=MUTED).grid(row=0,column=1,sticky='w')
        ttk.Button(ff, text="⬆ Use live data",  command=self._use_live).grid(row=0,column=2,padx=20)

        ttk.Button(ff, text="🎬 Load Video",      command=self._load_video).grid(row=1,column=0,padx=10,pady=4)
        ttk.Label(ff, textvariable=self.sv_vid, foreground=MUTED).grid(row=1,column=1,sticky='w')

        # Pendulum params
        pf = ttk.LabelFrame(self, text="  Pendulum Parameters  ")
        pf.pack(fill='x', padx=12, pady=4)

        params = [
            ("Sphere diameter (mm):", "diam",   20.0),
            ("Ruler segment  (mm):",  "ruler",  50.0),
            ("Resize factor:",        "resize",  0.5),
        ]
        self._pvars = {}
        for col, (label, key, default) in enumerate(params):
            ttk.Label(pf, text=label).grid(row=0, column=col*2,   padx=8, pady=6, sticky='e')
            v = tk.DoubleVar(value=default)
            ttk.Entry(pf, textvariable=v, width=7).grid(row=0, column=col*2+1, padx=4)
            self._pvars[key] = v

        # Sync
        sf = ttk.LabelFrame(self, text="  Synchronisation  ")
        sf.pack(fill='x', padx=12, pady=4)

        self.sync_mode   = tk.StringVar(value="auto")
        self.sync_offset = tk.DoubleVar(value=0.0)

        ttk.Radiobutton(sf, text="Auto-detect LED flash in video",
                        variable=self.sync_mode, value="auto").grid(row=0,column=0,padx=14,pady=5)
        ttk.Radiobutton(sf, text="Manual offset (s):",
                        variable=self.sync_mode, value="manual").grid(row=0,column=1,padx=10)
        ttk.Entry(sf, textvariable=self.sync_offset, width=8).grid(row=0,column=2,padx=4)
        ttk.Label(sf, text="(positive = Arduino clock ahead of video)",
                  foreground=MUTED).grid(row=0,column=3,padx=10)

        # Run bar
        rf = ttk.Frame(self)
        rf.pack(fill='x', padx=12, pady=6)

        self.run_btn    = ttk.Button(rf, text="▶  Run Full Analysis", command=self._start_analysis)
        self.export_btn = ttk.Button(rf, text="📤 Export PNG",         command=self._export, state='disabled')
        self.run_btn.pack(side='left', padx=4)
        self.export_btn.pack(side='left', padx=4)

        self.progress   = ttk.Progressbar(rf, length=280, mode='determinate')
        self.progress.pack(side='left', padx=12)
        self.sv_run = tk.StringVar(value="Ready")
        ttk.Label(rf, textvariable=self.sv_run, foreground=MUTED).pack(side='left', padx=6)

        # Results canvas
        res = ttk.LabelFrame(self, text="  Results  ")
        res.pack(fill='both', expand=True, padx=12, pady=4)

        self.fig_res = Figure(figsize=(11, 7), facecolor=BG)
        self.canvas_res = FigureCanvasTkAgg(self.fig_res, res)
        NavigationToolbar2Tk(self.canvas_res, res).update()
        self.canvas_res.get_tk_widget().pack(fill='both', expand=True)
        self._placeholder()

    # ── File loading ──────────────────────────────────────────────────────────
    def _load_csv(self):
        path = filedialog.askopenfilename(filetypes=[("CSV","*.csv"),("All","*.*")])
        if not path: return
        try:
            data = {k:[] for k in ['time_s','weight','ax','ay','az','tx','ty','sync']}
            with open(path) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Support both time_ms and time_s column names
                    if 'time_ms' in row:
                        data['time_s'].append(float(row['time_ms'])/1000.0)
                    else:
                        data['time_s'].append(float(row.get('time_s', 0)))
                    data['weight'].append(float(row.get('weight_kg', 0)))
                    data['ax'].append(float(row.get('accel_x', 0)))
                    data['ay'].append(float(row.get('accel_y', 0)))
                    data['az'].append(float(row.get('accel_z', 0)))
                    data['tx'].append(float(row.get('tilt_x', 0)))
                    data['ty'].append(float(row.get('tilt_y', 0)))
                    data['sync'].append(int(row.get('sync', 0)))
            self.csv_data = {k: np.array(v) for k,v in data.items()}
            n  = len(data['time_s'])
            ns = int(np.sum(self.csv_data['sync']))
            self.sv_csv.set(f"{os.path.basename(path)}  ({n} rows, {ns} sync events)")
        except Exception as e:
            messagebox.showerror("CSV Error", str(e))

    def _load_video(self):
        path = filedialog.askopenfilename(
            filetypes=[("Video","*.mp4 *.MOV *.mov *.avi *.mkv"),("All","*.*")])
        if not path: return
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        nf  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        self.video_path = path
        self.sv_vid.set(f"{os.path.basename(path)}  ({fps:.1f} fps, {nf/fps:.1f} s)")

    def _use_live(self):
        t,w,ax,ay,az,tx,ty,sync_ts = self.store.snapshot()
        if len(t) == 0:
            messagebox.showwarning("No data", "No live data recorded yet."); return
        sync_arr = np.zeros(len(t), int)
        for s in sync_ts:
            idx = np.argmin(np.abs(t - s/1000.0))
            sync_arr[idx] = 1
        self.csv_data = {'time_s':t,'weight':w,'ax':ax,'ay':ay,'az':az,
                         'tx':tx,'ty':ty,'sync':sync_arr}
        self.sv_csv.set(f"Live data  ({len(t)} samples, {int(sum(sync_arr))} sync events)")

    # ── Analysis pipeline ─────────────────────────────────────────────────────
    def _start_analysis(self):
        if self.csv_data is None:
            messagebox.showwarning("Missing", "Load a CSV or use live data first."); return
        if self.video_path is None:
            messagebox.showwarning("Missing", "Load a video file first."); return
        self.run_btn.config(state='disabled')
        self.sv_run.set("Opening video…")
        self.progress['value'] = 0
        # Calibration must run on main thread (OpenCV GUI requirement on macOS)
        self.after(50, self._do_calibration)

    def _do_calibration(self):
        """Interactive OpenCV calibration — runs on main thread."""
        try:
            cap = cv2.VideoCapture(self.video_path)
            ret, frame = cap.read(); cap.release()
            if not ret: raise RuntimeError("Cannot read first frame.")

            RESIZE = self._pvars['resize'].get()
            small  = cv2.resize(frame, (0,0), fx=RESIZE, fy=RESIZE)

            # 1. Ruler
            self.sv_run.set("Calibration: click ruler endpoints…")
            self.update()
            ruler_mm = self._pvars['ruler'].get()
            pts = cv_click_two_points(small, "Calibration",
                                      f"Click both ends of the {ruler_mm:.0f} mm ruler")
            ruler_px = np.sqrt((pts[1][0]-pts[0][0])**2 + (pts[1][1]-pts[0][1])**2)
            K = ruler_mm / ruler_px

            diam = self._pvars['diam'].get()
            exp_r = (diam/2.0) / K
            min_r = max(3, int(0.85*exp_r))
            max_r = max(min_r+1, int(1.15*exp_r))

            # 2. Attachment
            self.sv_run.set("Click thread attachment point…")
            self.update()
            att = cv_click_one_point(small, "Attachment",
                                     "Click the thread attachment point")

            # 3. ROI
            self.sv_run.set("Select ROI around sphere motion…")
            self.update()
            roi = cv_select_roi(small)

            self._params = {
                'K': K, 'resize': RESIZE,
                'attachment': att,
                'min_r': min_r, 'max_r': max_r,
                'roi': roi,
                'L_expected': None, 'L_tol': 40,
            }
            self.sv_run.set("Running CV detection…")
            threading.Thread(target=self._cv_thread, daemon=True).start()

        except Exception as e:
            messagebox.showerror("Calibration error", str(e))
            self.run_btn.config(state='normal')
            self.sv_run.set("Error")

    def _cv_thread(self):
        try:
            def prog(v):
                self.after(0, lambda val=v: self.progress.configure(value=val))

            results = run_pendulum_detection(self.video_path, self._params, prog)

            # Compute sync offset
            offset = 0.0
            if self.sync_mode.get() == "auto":
                _, flash_t = detect_led_flash(self.video_path)
                if flash_t is not None:
                    idx = np.where(self.csv_data['sync'] == 1)[0]
                    if len(idx) > 0:
                        csv_sync_t = self.csv_data['time_s'][idx[0]]
                        offset = csv_sync_t - flash_t
                        self.after(0, lambda o=offset: self.sync_offset.set(round(o,3)))
            else:
                offset = self.sync_offset.get()

            results['offset'] = offset
            self.vid_results   = results
            self.after(0, self._show_results)

        except Exception as e:
            self.after(0, lambda: messagebox.showerror("CV Error", str(e)))
            self.after(0, lambda: self.run_btn.config(state='normal'))
            self.after(0, lambda: self.sv_run.set("Error"))

    # ── Results plotting ──────────────────────────────────────────────────────
    def _show_results(self):
        d = self.csv_data
        r = self.vid_results
        t_csv = d['time_s']
        t_vid = r['times'] + r['offset']
        norm  = np.sqrt(d['ax']**2 + d['ay']**2 + d['az']**2)

        self.fig_res.clear()
        gs = self.fig_res.add_gridspec(3, 2, hspace=0.5, wspace=0.38,
                                       left=0.07, right=0.97, top=0.95, bottom=0.07)
        axes = [self.fig_res.add_subplot(gs[i,j]) for i in range(3) for j in range(2)]
        style_axes(axes, self.fig_res)

        sync_t = t_csv[d['sync']==1]

        # 0 — Pendulum angle
        a0 = axes[0]
        a0.plot(t_vid, r['angles'], color=ACCENT, lw=0.9, label='θ per frame')
        if len(r['angles']) > 0:
            mu = np.mean(r['angles'])
            a0.axhline(mu, color='#b4befe', lw=1.5, ls='--', label=f'Mean {mu:.2f}°')
            a0.fill_between(t_vid, mu-np.std(r['angles']), mu+np.std(r['angles']),
                            alpha=0.12, color=ACCENT)
        a0.set_ylabel('θ (°)'); a0.set_title('Pendulum deflection')
        a0.legend(fontsize=7, labelcolor=TEXT, facecolor=CARD, edgecolor=BORDER)

        # 1 — Load cell
        a1 = axes[1]
        a1.plot(t_csv, d['weight'], color=GREEN, lw=0.9)
        for st in sync_t: a1.axvline(st, color=ORANGE, lw=1.2, ls='--', alpha=0.8)
        a1.set_ylabel('Weight (kg)'); a1.set_title('Load cell  (⚡ = sync)')

        # 2 — Accel norm
        a2 = axes[2]
        a2.plot(t_csv, norm, color=ORANGE, lw=0.9)
        a2.set_ylabel('|a| (m/s²)'); a2.set_title('Acceleration magnitude')

        # 3 — Pendulum length consistency
        a3 = axes[3]
        a3.plot(t_vid, r['L_mm'], color=PURPLE, lw=0.9)
        if len(r['L_mm']) > 0:
            a3.axhline(np.mean(r['L_mm']), color='#b4befe', lw=1.5, ls='--')
        a3.set_ylabel('L (mm)'); a3.set_title('Pendulum length')

        # 4 — Tilt
        a4 = axes[4]
        a4.plot(t_csv, d['tx'], color=ACCENT, lw=0.8, label='Pitch')
        a4.plot(t_csv, d['ty'], color=RED,    lw=0.8, label='Roll')
        a4.set_ylabel('° '); a4.set_title('IMU tilt angles')
        a4.legend(fontsize=7, labelcolor=TEXT, facecolor=CARD, edgecolor=BORDER)

        # 5 — Angle vs Weight (scatter)
        a5 = axes[5]
        if len(r['angles']) > 0 and len(t_csv) > 0:
            w_i = np.interp(t_vid, t_csv, d['weight'])
            sc  = a5.scatter(w_i, r['angles'], c=t_vid, cmap='plasma', s=3, alpha=0.7)
            self.fig_res.colorbar(sc, ax=a5, label='time (s)')
        a5.set_xlabel('Weight (kg)'); a5.set_ylabel('θ (°)')
        a5.set_title('Angle  vs  Load')

        self.canvas_res.draw()
        self.progress['value'] = 100
        n_ok = len(r['angles'])
        n_tot = r['n_total']
        pct = 100*n_ok/n_tot if n_tot else 0
        self.sv_run.set(
            f"Done — {n_ok}/{n_tot} frames ({pct:.0f}%)  |  offset = {r['offset']:.3f} s")
        self.run_btn.config(state='normal')
        self.export_btn.config(state='normal')

    def _export(self):
        fn = f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        path = filedialog.asksaveasfilename(defaultextension=".png",
                   filetypes=[("PNG","*.png"),("PDF","*.pdf")], initialfile=fn)
        if path:
            self.fig_res.savefig(path, dpi=150, facecolor=BG)
            messagebox.showinfo("Exported", f"Plot saved to:\n{path}")

    def _placeholder(self):
        self.fig_res.clear()
        ax = self.fig_res.add_subplot(111)
        ax.set_facecolor(CARD)
        ax.text(0.5, 0.55, "Load data + video\nthen click  ▶ Run Full Analysis",
                ha='center', va='center', transform=ax.transAxes,
                color=MUTED, fontsize=14, family='monospace')
        ax.axis('off')
        self.canvas_res.draw()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN APP
# ══════════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ME-301  |  Measurement Lab")
        self.geometry("1150x820")
        self.configure(bg=BG)
        self._style()
        store = DataStore()
        nb = ttk.Notebook(self)
        nb.pack(fill='both', expand=True, padx=8, pady=8)
        cap_tab = CaptureTab(nb, store)
        ana_tab = AnalyseTab(nb, store)
        nb.add(cap_tab, text="   ⏺  Capture   ")
        nb.add(ana_tab, text="   📊  Analyse   ")

    def _style(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('.',             background=BG,      foreground=TEXT, fieldbackground=CARD)
        s.configure('TFrame',        background=BG)
        s.configure('TLabelframe',   background=BG,      foreground=ACCENT)
        s.configure('TLabelframe.Label', background=BG,  foreground=ACCENT,
                    font=('Helvetica', 9, 'bold'))
        s.configure('TButton',       background=CARD,    foreground=TEXT,
                    borderwidth=0, padding=7, relief='flat')
        s.map('TButton',   background=[('active', ACCENT), ('pressed','#3a6fd8')])
        s.configure('TLabel',        background=BG,      foreground=TEXT)
        s.configure('TEntry',        fieldbackground=CARD, foreground=TEXT)
        s.configure('TCombobox',     fieldbackground=CARD, foreground=TEXT)
        s.configure('TRadiobutton',  background=BG,      foreground=TEXT)
        s.configure('TNotebook',     background=BG,      tabmargins=[4,4,0,0])
        s.configure('TNotebook.Tab', background=CARD,    foreground=MUTED,  padding=[14,6])
        s.map('TNotebook.Tab',
              background=[('selected', BG)], foreground=[('selected', ACCENT)])
        s.configure('Horizontal.TProgressbar', troughcolor=CARD, background=ACCENT)


if __name__ == "__main__":
    app = App()
    app.mainloop()
