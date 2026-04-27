#!/usr/bin/env python3
"""
ME-301 Measurement Techniques — Integrated Lab App
Serial capture + Video pendulum analysis + Sync alignment
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, queue, os, csv
from datetime import datetime

import numpy as np
import cv2

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

try:
    import serial, serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

try:
    from PIL import Image, ImageTk
    PIL_OK = True
except ImportError:
    PIL_OK = False

# ══════════════════════════════════════════════════════════════════════════════
# THEME
# ══════════════════════════════════════════════════════════════════════════════
BG, CARD, BORDER = "#0f1117", "#21253a", "#2e3250"
ACCENT, GREEN, ORANGE, RED, PURPLE = "#4f8ef7", "#3ecf8e", "#f5a623", "#e05c5c", "#9b7fe8"
TEXT, MUTED = "#e8eaf0", "#6b7280"
BAUD    = 460800
MAX_PTS = 2000

def style_axes(axes, fig):
    fig.patch.set_facecolor(BG)
    for ax in (axes if hasattr(axes, '__iter__') else [axes]):
        ax.set_facecolor(CARD)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.xaxis.label.set_color(MUTED); ax.yaxis.label.set_color(MUTED)
        ax.title.set_color(TEXT)
        for sp in ax.spines.values(): sp.set_edgecolor(BORDER)
        ax.grid(True, ls=':', lw=0.5, alpha=0.4, color=BORDER)

# ══════════════════════════════════════════════════════════════════════════════
# DATA STORE
# ══════════════════════════════════════════════════════════════════════════════
class DataStore:
    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self.w_ts, self.w_kg = [], []
            self.a_ts, self.a_x, self.a_y, self.a_z = [], [], [], []
            self.a_pitch, self.a_roll = [], []
            self.sync_ts = []
            self.sr_w = 0.0; self.sr_a = 0.0
            self.recording = False

    def add_weight(self, ts, kg):
        with self._lock: self.w_ts.append(ts); self.w_kg.append(kg)

    def add_accel(self, ts, ax, ay, az, pitch, roll):
        with self._lock:
            self.a_ts.append(ts); self.a_x.append(ax)
            self.a_y.append(ay);  self.a_z.append(az)
            self.a_pitch.append(pitch); self.a_roll.append(roll)

    def add_sync(self, ts):
        with self._lock: self.sync_ts.append(ts)

    def set_rates(self, sr_w, sr_a):
        with self._lock: self.sr_w = sr_w; self.sr_a = sr_a

    def snapshot_w(self):
        with self._lock:
            return (np.array(self.w_ts, dtype=float)/1000.0,
                    np.array(self.w_kg, dtype=float))

    def snapshot_a(self):
        with self._lock:
            t = np.array(self.a_ts, dtype=float)/1000.0
            return (t,
                    np.array(self.a_x,     dtype=float),
                    np.array(self.a_y,     dtype=float),
                    np.array(self.a_z,     dtype=float),
                    np.array(self.a_pitch, dtype=float),
                    np.array(self.a_roll,  dtype=float))

    def get_rates(self):
        with self._lock: return self.sr_w, self.sr_a

    def count_w(self):
        with self._lock: return len(self.w_ts)
    def count_a(self):
        with self._lock: return len(self.a_ts)
    def sync_count(self):
        with self._lock: return len(self.sync_ts)

    def save_csv(self, path):
        with self._lock:
            sync_set = set(self.sync_ts)
            wp = path.replace('.csv', '_weight.csv')
            with open(wp, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['time_ms','weight_kg','sync'])
                for i,ts in enumerate(self.w_ts):
                    w.writerow([ts, self.w_kg[i], 1 if ts in sync_set else 0])
            ap = path.replace('.csv', '_accel.csv')
            with open(ap, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['time_ms','accel_x','accel_y','accel_z','pitch','roll','sync'])
                for i,ts in enumerate(self.a_ts):
                    w.writerow([ts, self.a_x[i], self.a_y[i], self.a_z[i],
                                self.a_pitch[i], self.a_roll[i],
                                1 if ts in sync_set else 0])
        print(f"Saved → {wp}\nSaved → {ap}")

# ══════════════════════════════════════════════════════════════════════════════
# SERIAL READER
# ══════════════════════════════════════════════════════════════════════════════
class SerialReader(threading.Thread):
    def __init__(self, port, store, q):
        super().__init__(daemon=True)
        self.port=port; self.store=store; self.q=q
        self._stop=threading.Event(); self._ser=None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, BAUD, timeout=1)
            self.q.put(("status", f"Connected  {self.port}"))
            while not self._stop.is_set():
                raw = self._ser.readline()
                try: line = raw.decode('utf-8', errors='ignore').strip()
                except: continue
                if not line: continue
                if line.startswith('W,'):
                    p = line.split(',')
                    if len(p)==3:
                        try:
                            ts,kg = float(p[1]),float(p[2])
                            if self.store.recording: self.store.add_weight(ts,kg)
                            self.q.put(("weight",(ts,kg)))
                        except: pass
                elif line.startswith('A,'):
                    p = line.split(',')
                    if len(p)==7:
                        try:
                            v = list(map(float,p[1:]))
                            if self.store.recording: self.store.add_accel(*v)
                            self.q.put(("accel",v))
                        except: pass
                elif line.startswith('S,'):
                    p = line.split(',')
                    if len(p)==2:
                        try:
                            ts = float(p[1])
                            if self.store.recording: self.store.add_sync(ts)
                            self.q.put(("sync",ts))
                        except: pass
                elif line.startswith('RATE,'):
                    p = line.split(',')
                    if len(p)==3:
                        try:
                            self.store.set_rates(float(p[1]),float(p[2]))
                            self.q.put(("rate",(float(p[1]),float(p[2]))))
                        except: pass
                elif line.startswith('INFO,') or line.startswith('ERROR,'):
                    self.q.put(("log", line))
        except Exception as e:
            self.q.put(("error", str(e)))
        finally:
            if self._ser and self._ser.is_open: self._ser.close()
            self.q.put(("disconnected",""))

    def send(self, cmd):
        if self._ser and self._ser.is_open:
            self._ser.write((cmd+'\n').encode())

    def stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open: self._ser.close()

# ══════════════════════════════════════════════════════════════════════════════
# TAB 1 — CAPTURE  (decoupled render loop)
# ══════════════════════════════════════════════════════════════════════════════
class CaptureTab(ttk.Frame):
    def __init__(self, parent, store):
        super().__init__(parent)
        self.store=store; self.q=queue.Queue(); self.reader=None
        self._wt,self._wv = [],[]
        self._at,self._ax,self._ay,self._az,self._an = [],[],[],[],[]
        self._sync_times = []
        self._needs_redraw = False
        self._build()
        self.after(20,  self._poll)    # data ingestion: 50 Hz
        self.after(150, self._render)  # rendering:       7 Hz

    def _build(self):
        # Connection
        c = ttk.LabelFrame(self, text="  Connection  ")
        c.pack(fill='x', padx=12, pady=(10,4))
        ttk.Label(c, text="Port:").grid(row=0,column=0,padx=8,pady=6)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(c, textvariable=self.port_var, width=24, state='readonly')
        self.port_cb.grid(row=0,column=1,padx=4)
        ttk.Button(c, text="↻", command=self._refresh, width=3).grid(row=0,column=2,padx=4)
        self.conn_btn = ttk.Button(c, text="Connect", command=self._toggle_connect)
        self.conn_btn.grid(row=0,column=3,padx=10)
        self._refresh()

        # Controls
        ctrl = ttk.LabelFrame(self, text="  Recording  ")
        ctrl.pack(fill='x', padx=12, pady=4)
        self.rec_btn  = ttk.Button(ctrl, text="⏺  Start Recording",  command=self._toggle_rec,           state='disabled')
        self.sync_btn = ttk.Button(ctrl, text="⚡ Flash Sync LED",    command=self._sync,                  state='disabled')
        self.tw_btn   = ttk.Button(ctrl, text="⚖ Tare Weight",       command=lambda:self._cmd("TARE_W"), state='disabled')
        self.ta_btn   = ttk.Button(ctrl, text="↕ Tare Accel",        command=lambda:self._cmd("TARE_A"), state='disabled')
        self.save_btn = ttk.Button(ctrl, text="💾 Save CSV",          command=self._save,                  state='disabled')
        self.clr_btn  = ttk.Button(ctrl, text="🗑  Clear",            command=self._clear)
        for i,b in enumerate([self.rec_btn,self.sync_btn,self.tw_btn,self.ta_btn,self.save_btn,self.clr_btn]):
            b.grid(row=0,column=i,padx=8,pady=7)

        # Status strip
        sf = ttk.Frame(self); sf.pack(fill='x', padx=14, pady=2)
        self.sv_status = tk.StringVar(value="Disconnected")
        self.sv_srw    = tk.StringVar(value="W: — Hz")
        self.sv_sra    = tk.StringVar(value="A: — Hz")
        self.sv_sync   = tk.StringVar(value="0 sync events")
        self.sv_cnt    = tk.StringVar(value="0W  0A samples")
        ttk.Label(sf, textvariable=self.sv_status, foreground=MUTED).pack(side='left')
        ttk.Label(sf, textvariable=self.sv_cnt).pack(side='left', padx=20)
        ttk.Label(sf, textvariable=self.sv_srw, foreground=ACCENT).pack(side='left', padx=8)
        ttk.Label(sf, textvariable=self.sv_sra, foreground=GREEN).pack(side='left', padx=8)
        ttk.Label(sf, textvariable=self.sv_sync, foreground=ORANGE).pack(side='left', padx=20)

        # Live plot — 4 rows
        pf = ttk.LabelFrame(self, text="  Live Data  ")
        pf.pack(fill='both', expand=True, padx=12, pady=6)
        self.fig = Figure(figsize=(10,6), facecolor=BG)
        self.axW = self.fig.add_subplot(411)
        self.axA = self.fig.add_subplot(412, sharex=self.axW)
        self.axT = self.fig.add_subplot(413, sharex=self.axW)
        self.axS = self.fig.add_subplot(414, sharex=self.axW)
        style_axes([self.axW,self.axA,self.axT,self.axS], self.fig)
        self.axW.set_ylabel("Weight kg",  fontsize=8)
        self.axA.set_ylabel("|a| m/s²",   fontsize=8)
        self.axT.set_ylabel("Tilt °",     fontsize=8)
        self.axS.set_ylabel("Sync",       fontsize=8)
        self.axS.set_xlabel("Time  s",    fontsize=8)
        self.axS.set_ylim(-0.2, 1.3); self.axS.set_yticks([])
        self.lW,  = self.axW.plot([],[],color=ACCENT, lw=1)
        self.lAN, = self.axA.plot([],[],color=GREEN,  lw=1)
        self.lTP, = self.axT.plot([],[],color=ORANGE, lw=0.8, label='Pitch')
        self.lTR, = self.axT.plot([],[],color=RED,    lw=0.8, label='Roll')
        self.axT.legend(fontsize=7, labelcolor=TEXT, facecolor=CARD, edgecolor=BORDER)
        self.srw_txt = self.axW.text(0.01,0.88,'',transform=self.axW.transAxes,color=ACCENT,fontsize=7,va='top')
        self.sra_txt = self.axA.text(0.01,0.88,'',transform=self.axA.transAxes,color=GREEN, fontsize=7,va='top')
        self.fig.tight_layout(pad=1.4)
        self.canvas = FigureCanvasTkAgg(self.fig, pf)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    # ── Actions ───────────────────────────────────────────────────────────────
    def _refresh(self):
        if not SERIAL_OK: self.port_cb['values']=['pyserial not installed']; return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports: self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.reader and self.reader.is_alive():
            self.reader.stop()
            self.conn_btn.config(text="Connect")
            for b in [self.rec_btn,self.sync_btn,self.tw_btn,self.ta_btn]: b.config(state='disabled')
        else:
            if not SERIAL_OK: messagebox.showerror("Missing","pip install pyserial"); return
            port = self.port_var.get()
            if not port: messagebox.showwarning("No port","Select a port."); return
            self.reader = SerialReader(port, self.store, self.q)
            self.reader.start()
            self.conn_btn.config(text="Disconnect")
            for b in [self.rec_btn,self.sync_btn,self.tw_btn,self.ta_btn]: b.config(state='normal')

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
            self.sv_status.set(f"Stopped — {self.store.count_w()}W  {self.store.count_a()}A")

    def _sync(self):
        if self.reader: self.reader.send("SYNC")

    def _cmd(self, cmd):
        if self.reader: self.reader.send(cmd)

    def _save(self):
        fn   = f"exp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                   filetypes=[("CSV","*.csv")], initialfile=fn)
        if path: self.store.save_csv(path); messagebox.showinfo("Saved","Files saved with _weight / _accel suffixes.")

    def _clear(self):
        if not messagebox.askyesno("Clear","Clear all recorded data?"): return
        self.store.reset()
        for b in [self._wt,self._wv,self._at,self._ax,self._ay,self._az,self._an]: b.clear()
        self._sync_times.clear()
        self.axS.cla(); style_axes(self.axS, self.fig)
        self.axS.set_ylabel("Sync",fontsize=8); self.axS.set_xlabel("Time  s",fontsize=8)
        self.axS.set_ylim(-0.2,1.3); self.axS.set_yticks([])
        self.sv_cnt.set("0W  0A samples"); self.sv_sync.set("0 sync events")
        self._needs_redraw = True

    # ── Data ingestion — runs every 20 ms, no drawing ─────────────────────────
    def _poll(self):
        try:
            for _ in range(150):  # drain fast
                kind, payload = self.q.get_nowait()
                if kind == "weight":
                    ts,kg = payload
                    self._wt.append(ts/1000.0); self._wv.append(kg)
                    if len(self._wt)>MAX_PTS: self._wt.pop(0); self._wv.pop(0)
                    self._needs_redraw = True
                elif kind == "accel":
                    ts,ax,ay,az,_,_ = payload
                    norm = np.sqrt(ax**2+ay**2+az**2)
                    self._at.append(ts/1000.0)
                    self._ax.append(ax); self._ay.append(ay)
                    self._az.append(az); self._an.append(norm)
                    if len(self._at)>MAX_PTS:
                        for b in [self._at,self._ax,self._ay,self._az,self._an]: b.pop(0)
                    self._needs_redraw = True
                elif kind == "sync":
                    t_s = payload/1000.0
                    self._sync_times.append(t_s)
                    self.axW.axvline(t_s,color=ORANGE,lw=1.5,ls='--',alpha=0.85)
                    self.axA.axvline(t_s,color=ORANGE,lw=1.5,ls='--',alpha=0.85)
                    self.axS.plot([t_s,t_s],[0,1.0],color=ORANGE,lw=3)
                    self.axS.plot([t_s],[1.0],'o',color=ORANGE,ms=5)
                    self.sv_sync.set(f"{self.store.sync_count()} sync events")
                    self._needs_redraw = True
                elif kind == "rate":
                    sr_w,sr_a = payload
                    self.sv_srw.set(f"W: {sr_w:.1f} Hz"); self.sv_sra.set(f"A: {sr_a:.1f} Hz")
                    self.srw_txt.set_text(f"SR: {sr_w:.1f} Hz"); self.sra_txt.set_text(f"SR: {sr_a:.1f} Hz")
                elif kind == "status":       self.sv_status.set(payload)
                elif kind == "log":          print("[Arduino]", payload)
                elif kind == "error":        messagebox.showerror("Serial error", payload)
                elif kind == "disconnected": self.sv_status.set("Disconnected")
        except queue.Empty:
            pass
        self.sv_cnt.set(f"{self.store.count_w()}W  {self.store.count_a()}A samples")
        self.after(20, self._poll)

    # ── Rendering — runs every 150 ms (~7 Hz), never blocks ingestion ─────────
    def _render(self):
        if self._needs_redraw:
            self._needs_redraw = False
            self._redraw()
        self.after(150, self._render)

    def _redraw(self):
        if self._wt:
            self.lW.set_data(self._wt, self._wv)
            mn,mx = min(self._wv),max(self._wv)
            pad = max((mx-mn)*0.15, 0.005)
            self.axW.set_xlim(self._wt[0], self._wt[-1])
            self.axW.set_ylim(mn-pad, mx+pad)
        if self._at:
            self.lAN.set_data(self._at, self._an)
            mn,mx = min(self._an),max(self._an)
            pad = max((mx-mn)*0.15, 0.1)
            self.axA.set_xlim(self._at[0], self._at[-1])
            self.axA.set_ylim(mn-pad, mx+pad)
            self.lTP.set_data(self._at, self._ax)
            self.lTR.set_data(self._at, self._ay)
            mn2 = min(min(self._ax), min(self._ay))
            mx2 = max(max(self._ax), max(self._ay))
            pad2 = max((mx2-mn2)*0.15, 1.0)
            self.axT.set_xlim(self._at[0], self._at[-1])
            self.axT.set_ylim(mn2-pad2, mx2+pad2)
        if self._sync_times:
            self.axS.set_xlim(self.axW.get_xlim())
        self.canvas.draw_idle()


# ══════════════════════════════════════════════════════════════════════════════
# SYNC FRAME PICKER  (Toplevel window)
# ══════════════════════════════════════════════════════════════════════════════
class SyncPicker(tk.Toplevel):
    """
    Scan video for LED flash using multiple methods, show brightness plot +
    frame preview so user can fine-tune the exact sync frame.
    """
    def __init__(self, parent, video_path, led_roi=None, callback=None):
        super().__init__(parent)
        self.title("Sync Frame Picker"); self.configure(bg=BG); self.geometry("960x680")
        self.video_path=video_path; self.led_roi=led_roi; self.callback=callback
        self._cap = cv2.VideoCapture(video_path)
        self._fps   = self._cap.get(cv2.CAP_PROP_FPS) or 30.0
        self._total = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self._cur = 0; self._tk_img = None; self._vline = None
        self._brightness = None
        self._build()
        # Scan in background so window appears immediately
        threading.Thread(target=self._scan_brightness, daemon=True).start()

    def _build(self):
        top = ttk.Frame(self); top.pack(fill='both', expand=True, padx=10, pady=8)

        # Left — frame preview
        lf = ttk.LabelFrame(top, text="  Frame Preview  ")
        lf.pack(side='left', fill='both', expand=True, padx=(0,6))
        self.vid_cv = tk.Canvas(lf, bg='black', width=480, height=320)
        self.vid_cv.pack(fill='both', expand=True, padx=4, pady=4)
        self.sv_fi = tk.StringVar(value="Scanning…")
        ttk.Label(lf, textvariable=self.sv_fi, foreground=MUTED, font=('Courier',9)).pack(pady=3)

        # Right — brightness plot
        rf = ttk.LabelFrame(top, text="  Brightness / Flash Detection  ")
        rf.pack(side='left', fill='both', expand=True)
        self.fig_b  = Figure(figsize=(4.5,4), facecolor=BG)
        self.ax_b   = self.fig_b.add_subplot(111)
        style_axes(self.ax_b, self.fig_b)
        self.ax_b.set_xlabel("Time  s",fontsize=8)
        self.ax_b.set_ylabel("Brightness",fontsize=8)
        self.ax_b.set_title("Click on spike = LED flash frame",fontsize=8,color=TEXT)
        self.fig_b.tight_layout(pad=1.5)
        self.cv_b = FigureCanvasTkAgg(self.fig_b, rf)
        self.cv_b.get_tk_widget().pack(fill='both', expand=True)
        self.cv_b.mpl_connect('button_press_event', self._on_plot_click)

        # Bottom controls
        bot = ttk.Frame(self); bot.pack(fill='x', padx=10, pady=6)
        ttk.Label(bot, text="Scrub:").pack(side='left', padx=6)
        self.slider = ttk.Scale(bot, from_=0, to=max(self._total-1,1),
                                orient='horizontal', command=self._on_slider)
        self.slider.pack(side='left', fill='x', expand=True, padx=6)
        ttk.Button(bot, text="◀ -1",  command=lambda:self._step(-1)).pack(side='left', padx=3)
        ttk.Button(bot, text="+1 ▶",  command=lambda:self._step(1)).pack(side='left', padx=3)
        ttk.Button(bot, text="◀◀ -10",command=lambda:self._step(-10)).pack(side='left', padx=3)
        ttk.Button(bot, text="+10 ▶▶",command=lambda:self._step(10)).pack(side='left', padx=3)
        ttk.Button(bot, text="✓  Set this frame as SYNC",
                   command=self._confirm).pack(side='left', padx=14)
        ttk.Button(bot, text="Cancel", command=self.destroy).pack(side='left', padx=4)

        self._show_frame(0)

    def _scan_brightness(self):
        """
        Compute per-frame brightness using THREE methods and combine them.
        This makes detection robust regardless of LED colour.
        """
        cap = cv2.VideoCapture(self.video_path)
        raw_mean, raw_max, raw_green = [], [], []

        while True:
            ret, f = cap.read()
            if not ret: break
            if self.led_roi:
                x,y,w,h = self.led_roi; region = f[y:y+h, x:x+w]
            else:
                H,W = f.shape[:2]
                region = f[H//4:3*H//4, W//4:3*W//4]
            gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
            raw_mean.append(float(np.mean(gray)))
            raw_max.append(float(np.max(gray)))
            hsv  = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
            # Green LED
            m_green  = cv2.inRange(hsv,(35,60,60),(85,255,255))
            # White/any bright LED
            m_bright = cv2.inRange(hsv,(0,0,200),(180,60,255))
            raw_green.append(float(np.sum(m_green)+np.sum(m_bright))/255.0)
        cap.release()

        # Normalise and combine
        def norm(a):
            a = np.array(a,dtype=float)
            r = a.max()-a.min()
            return (a-a.min())/r if r>0 else a*0
        combined = norm(raw_mean)*0.3 + norm(raw_max)*0.3 + norm(raw_green)*0.4
        self._brightness = combined
        self.after(0, self._plot_brightness)

    def _plot_brightness(self):
        b  = self._brightness
        t  = np.arange(len(b))/self._fps
        self.ax_b.clear(); style_axes(self.ax_b, self.fig_b)
        self.ax_b.plot(t, b, color=GREEN, lw=0.8, label='Combined brightness')
        self.ax_b.set_xlabel("Time  s",fontsize=8)
        self.ax_b.set_ylabel("Normalised brightness",fontsize=8)
        self.ax_b.set_title("Click on spike = LED flash frame",fontsize=8,color=TEXT)

        # Auto-detect peak
        thr = np.mean(b)+2.0*np.std(b)
        spikes = np.where(b>thr)[0]
        if len(spikes):
            pk = int(spikes[0])
            self.ax_b.axvline(pk/self._fps, color=ORANGE, lw=1.5, ls='--',
                              label=f'Auto peak @ {pk/self._fps:.2f}s')
            self.ax_b.axhline(thr, color=MUTED, lw=0.8, ls=':')
            # Jump slider to auto peak
            self.after(0, lambda: self.slider.set(pk))
            self.after(0, lambda: self._show_frame(pk))
        self.ax_b.legend(fontsize=7, labelcolor=TEXT, facecolor=CARD, edgecolor=BORDER)
        self.cv_b.draw()

    def _show_frame(self, fi):
        fi = max(0, min(fi, self._total-1)); self._cur = fi
        self._cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ret, frame = self._cap.read()
        if not ret: return
        if self.led_roi:
            x,y,w,h = self.led_roi
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,220,100),2)
        cw = max(self.vid_cv.winfo_width(), 400)
        ch = max(self.vid_cv.winfo_height(),280)
        fh,fw = frame.shape[:2]
        scale = min(cw/fw, ch/fh)
        nw,nh = int(fw*scale),int(fh*scale)
        rgb = cv2.cvtColor(cv2.resize(frame,(nw,nh),interpolation=cv2.INTER_NEAREST),
                           cv2.COLOR_BGR2RGB)
        if PIL_OK:
            img = ImageTk.PhotoImage(Image.fromarray(rgb))
            self._tk_img = img
            self.vid_cv.delete("all")
            self.vid_cv.create_image(cw//2,ch//2,image=img,anchor='center')
        t_s = fi/self._fps
        self.sv_fi.set(f"Frame {fi} / {self._total}   t = {t_s:.3f} s")
        if self._brightness is not None:
            if self._vline:
                try: self._vline.remove()
                except: pass
            self._vline = self.ax_b.axvline(t_s,color=RED,lw=1.5)
            self.cv_b.draw_idle()

    def _on_slider(self, val): self._show_frame(int(float(val)))
    def _step(self, d): fi=self._cur+d; self.slider.set(fi); self._show_frame(fi)
    def _on_plot_click(self, ev):
        if ev.xdata is None: return
        fi=int(ev.xdata*self._fps); self.slider.set(fi); self._show_frame(fi)

    def _confirm(self):
        t_s = self._cur/self._fps
        if self.callback: self.callback(t_s)
        self.destroy()

    def destroy(self):
        self._cap.release(); super().destroy()


# ══════════════════════════════════════════════════════════════════════════════
# CV HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def cv_click_two_points(frame, title, msg):
    pts,disp = [],frame.copy()
    def cb(ev,x,y,*_):
        if ev==cv2.EVENT_LBUTTONDOWN and len(pts)<2:
            pts.append((x,y)); cv2.circle(disp,(x,y),6,(0,255,255),-1)
            if len(pts)==2: cv2.line(disp,pts[0],pts[1],(0,255,255),2)
            cv2.imshow(title,disp)
    cv2.putText(disp,msg,(10,32),cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,255,255),2)
    cv2.namedWindow(title,cv2.WINDOW_NORMAL); cv2.imshow(title,disp)
    cv2.setMouseCallback(title,cb)
    while len(pts)<2: cv2.waitKey(20)
    cv2.waitKey(400); cv2.destroyWindow(title); return pts

def cv_click_one_point(frame, title, msg):
    pts,disp = [],frame.copy()
    def cb(ev,x,y,*_):
        if ev==cv2.EVENT_LBUTTONDOWN and not pts:
            pts.append((x,y)); cv2.circle(disp,(x,y),6,(0,255,0),-1); cv2.imshow(title,disp)
    cv2.putText(disp,msg,(10,32),cv2.FONT_HERSHEY_SIMPLEX,0.65,(255,255,255),2)
    cv2.namedWindow(title,cv2.WINDOW_NORMAL); cv2.imshow(title,disp)
    cv2.setMouseCallback(title,cb)
    while not pts: cv2.waitKey(20)
    cv2.waitKey(400); cv2.destroyWindow(title); return pts[0]

def cv_select_roi_named(frame, title):
    roi = cv2.selectROI(title,frame,showCrosshair=True,fromCenter=False)
    cv2.destroyWindow(title)
    x,y,w,h = roi
    if w==0 or h==0: raise RuntimeError("ROI cancelled")
    return int(x),int(y),int(w),int(h)

def cv_detect_circles(frame, min_r, max_r):
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(9,9),2); gray = cv2.equalizeHist(gray)
    c = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,dp=1.2,minDist=40,
                         param1=80,param2=28,minRadius=min_r,maxRadius=max_r)
    if c is None: return []
    return [(int(x),int(y),int(r)) for x,y,r in c[0]]

def cv_best_circle(candidates, att, L_exp, prev_c, prev_r, L_tol):
    if not candidates: return None
    ax,ay=att; best_s,best=-1e9,None
    for cx,cy,r in candidates:
        L=np.sqrt((cx-ax)**2+(cy-ay)**2); s=0.0
        if cy<ay: s-=500
        if L_exp: s-=abs(L-L_exp)
        if prev_c: s-=1.5*np.sqrt((cx-prev_c[0])**2+(cy-prev_c[1])**2)
        if prev_r: s-=2.0*abs(r-prev_r)
        s+=0.5*r
        if s>best_s: best_s,best=s,(cx,cy,r)
    return best

def run_pendulum_detection(video_path, params, progress_cb, frame_cb=None):
    RESIZE=params['resize']; K=params['K']; att=params['attachment']
    min_r=params['min_r']; max_r=params['max_r']; roi=params['roi']
    L_exp=params.get('L_expected'); L_tol=params.get('L_tol',40)
    cap=cv2.VideoCapture(video_path)
    fps=cap.get(cv2.CAP_PROP_FPS) or 30.0
    total=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if L_exp is None:
        raw_L=[]
        while True:
            ret,frame=cap.read()
            if not ret: break
            small=cv2.resize(frame,(0,0),fx=RESIZE,fy=RESIZE)
            rx,ry,rw,rh=roi; crop=small[ry:ry+rh,rx:rx+rw]
            cands=[(cx+rx,cy+ry,r) for cx,cy,r in cv_detect_circles(crop,min_r,max_r)]
            best=cv_best_circle(cands,att,None,None,None,9999)
            if best:
                cx,cy,_=best
                if cy>att[1]: raw_L.append(np.sqrt((cx-att[0])**2+(cy-att[1])**2))
        L_exp=float(np.median(raw_L)) if raw_L else 100.0
        L_tol=0.15*L_exp
        cap.set(cv2.CAP_PROP_POS_FRAMES,0)
    times,angles,L_mm_list=[],[],[]
    prev_c,prev_r=None,None; n_fail=0; fi=0
    while True:
        ret,frame=cap.read()
        if not ret: break
        small=cv2.resize(frame,(0,0),fx=RESIZE,fy=RESIZE)
        rx,ry,rw,rh=roi; crop=small[ry:ry+rh,rx:rx+rw]
        cands=[(cx+rx,cy+ry,r) for cx,cy,r in cv_detect_circles(crop,min_r,max_r)]
        best=cv_best_circle(cands,att,L_exp,prev_c,prev_r,L_tol)
        accepted=False
        if best:
            cx,cy,r=best; L=np.sqrt((cx-att[0])**2+(cy-att[1])**2)
            if cy>att[1] and abs(L-L_exp)<L_tol:
                theta=np.degrees(np.arctan2(cx-att[0],cy-att[1]))
                times.append(fi/fps); angles.append(theta); L_mm_list.append(L*K)
                prev_c,prev_r=(cx,cy),r; accepted=True
            else: n_fail+=1
        else: n_fail+=1
        if frame_cb and fi%3==0:
            vis=small.copy()
            rx,ry,rw,rh=roi; cv2.rectangle(vis,(rx,ry),(rx+rw,ry+rh),(0,220,220),1)
            ax2,ay2=att; cv2.circle(vis,(ax2,ay2),5,(255,220,0),-1)
            if best:
                cx2,cy2,r2=best; col=(50,220,100) if accepted else (60,60,220)
                cv2.circle(vis,(cx2,cy2),r2,col,2); cv2.circle(vis,(cx2,cy2),3,(255,255,255),-1)
                cv2.line(vis,(ax2,ay2),(cx2,cy2),col,2)
                lbl=f"{angles[-1]:.1f}" if accepted else "X"
                cv2.putText(vis,lbl,(cx2+r2+3,cy2),cv2.FONT_HERSHEY_SIMPLEX,0.4,col,1)
            cv2.putText(vis,f"{fi}/{total} ({fi/total*100:.0f}%)",(6,16),
                        cv2.FONT_HERSHEY_SIMPLEX,0.45,(180,180,180),1)
            frame_cb(cv2.cvtColor(vis,cv2.COLOR_BGR2RGB))
        if fi%10==0: progress_cb(fi/total*100)
        fi+=1
    cap.release()
    return {'times':np.array(times),'angles':np.array(angles),
            'L_mm':np.array(L_mm_list),'fps':fps,'n_failed':n_fail,'n_total':fi}


# ══════════════════════════════════════════════════════════════════════════════
# TAB 2 — ANALYSE
# ══════════════════════════════════════════════════════════════════════════════
class AnalyseTab(ttk.Frame):
    def __init__(self, parent, store):
        super().__init__(parent)
        self.store=store; self.csv_w=None; self.csv_a=None
        self.video_path=None; self.vid_results=None; self._params=None
        self.led_roi=None; self.manual_video_sync_t=None
        self._fq=queue.Queue(maxsize=6); self._tk_img=None
        self._build(); self.after(40,self._poll_frames)

    def _build(self):
        paned=ttk.PanedWindow(self,orient='horizontal')
        paned.pack(fill='both',expand=True)
        left=ttk.Frame(paned);  paned.add(left, weight=3)
        right=ttk.Frame(paned); paned.add(right,weight=1)
        self._build_left(left); self._build_right(right)

    def _build_left(self, parent):
        # Files
        ff=ttk.LabelFrame(parent,text="  Data Files  ")
        ff.pack(fill='x',padx=10,pady=(10,4))
        self.sv_w=tk.StringVar(value="No weight CSV")
        self.sv_a=tk.StringVar(value="No accel CSV")
        self.sv_v=tk.StringVar(value="No video")
        ttk.Button(ff,text="📂 Weight CSV", command=self._load_w).grid(row=0,column=0,padx=8,pady=4)
        ttk.Label(ff,textvariable=self.sv_w,foreground=MUTED).grid(row=0,column=1,sticky='w')
        ttk.Button(ff,text="⬆ Use live",   command=self._use_live).grid(row=0,column=2,padx=10)
        ttk.Button(ff,text="📂 Accel CSV",  command=self._load_a).grid(row=1,column=0,padx=8,pady=4)
        ttk.Label(ff,textvariable=self.sv_a,foreground=MUTED).grid(row=1,column=1,sticky='w')
        ttk.Button(ff,text="🎬 Load Video", command=self._load_v).grid(row=2,column=0,padx=8,pady=4)
        ttk.Label(ff,textvariable=self.sv_v,foreground=MUTED).grid(row=2,column=1,sticky='w')

        # Params
        pf=ttk.LabelFrame(parent,text="  Pendulum Parameters  ")
        pf.pack(fill='x',padx=10,pady=4)
        self._pvars={}
        for col,(lbl,key,val) in enumerate([("Sphere Ø mm","diam",20.0),("Ruler mm","ruler",50.0),("Resize","resize",0.5)]):
            ttk.Label(pf,text=lbl+":").grid(row=0,column=col*2,padx=8,pady=6,sticky='e')
            v=tk.DoubleVar(value=val)
            ttk.Entry(pf,textvariable=v,width=7).grid(row=0,column=col*2+1,padx=4)
            self._pvars[key]=v

        # Sync
        sf=ttk.LabelFrame(parent,text="  Synchronisation  ")
        sf.pack(fill='x',padx=10,pady=4)
        self.sync_mode=tk.StringVar(value="manual_picker")
        self.sync_offset=tk.DoubleVar(value=0.0)
        self.sv_sync_status=tk.StringVar(value="Not set")
        ttk.Radiobutton(sf,text="Auto: detect LED flash (green or bright) — select ROI in step 4",
                        variable=self.sync_mode,value="led_roi").grid(row=0,column=0,columnspan=3,padx=12,pady=3,sticky='w')
        ttk.Radiobutton(sf,text="Manual frame picker →",
                        variable=self.sync_mode,value="manual_picker").grid(row=1,column=0,padx=12,sticky='w')
        self.pick_btn=ttk.Button(sf,text="🎬 Open Picker",command=self._open_picker,state='disabled')
        self.pick_btn.grid(row=1,column=1,padx=8)
        ttk.Label(sf,textvariable=self.sv_sync_status,foreground=GREEN).grid(row=1,column=2,padx=8)
        ttk.Radiobutton(sf,text="Manual offset (s):",
                        variable=self.sync_mode,value="manual").grid(row=2,column=0,padx=12,pady=3,sticky='w')
        ttk.Entry(sf,textvariable=self.sync_offset,width=8).grid(row=2,column=1,padx=4,sticky='w')
        ttk.Label(sf,text="positive = Arduino ahead",foreground=MUTED).grid(row=2,column=2,padx=8)

        # Run bar
        rf=ttk.Frame(parent); rf.pack(fill='x',padx=10,pady=6)
        self.run_btn=ttk.Button(rf,text="▶  Run Analysis",command=self._start)
        self.exp_btn=ttk.Button(rf,text="📤 Export PNG",  command=self._export,    state='disabled')
        self.csv_btn=ttk.Button(rf,text="📋 Export CSV",  command=self._export_csv,state='disabled')
        self.run_btn.pack(side='left',padx=4); self.exp_btn.pack(side='left',padx=4)
        self.csv_btn.pack(side='left',padx=4)
        self.progress=ttk.Progressbar(rf,length=200,mode='determinate')
        self.progress.pack(side='left',padx=10)
        self.sv_run=tk.StringVar(value="Ready")
        ttk.Label(rf,textvariable=self.sv_run,foreground=MUTED).pack(side='left',padx=4)

        # Results
        res=ttk.LabelFrame(parent,text="  Results  ")
        res.pack(fill='both',expand=True,padx=10,pady=4)
        self.fig_res=Figure(figsize=(7,7),facecolor=BG)
        self.canvas_res=FigureCanvasTkAgg(self.fig_res,res)
        NavigationToolbar2Tk(self.canvas_res,res).update()
        self.canvas_res.get_tk_widget().pack(fill='both',expand=True)
        self._placeholder()

    def _build_right(self, parent):
        vf=ttk.LabelFrame(parent,text="  Detection Preview  ")
        vf.pack(fill='both',expand=True,padx=(0,10),pady=(10,4))
        self.vid_canvas=tk.Canvas(vf,bg='#000000')
        self.vid_canvas.pack(fill='both',expand=True)
        self.sv_fi=tk.StringVar(value="—")
        ttk.Label(vf,textvariable=self.sv_fi,foreground=MUTED,font=('Courier',8)).pack(pady=2)
        sf=ttk.LabelFrame(parent,text="  Sync Alignment  ")
        sf.pack(fill='x',padx=(0,10),pady=4)
        self.fig_sync=Figure(figsize=(4,1.8),facecolor=BG)
        self.ax_sync=self.fig_sync.add_subplot(111)
        style_axes(self.ax_sync,self.fig_sync)
        self.ax_sync.set_xlabel("Time  s",fontsize=7)
        self.ax_sync.set_title("Arduino sync  vs  Video LED",fontsize=8,color=TEXT)
        self.fig_sync.tight_layout(pad=1.2)
        self.canvas_sync=FigureCanvasTkAgg(self.fig_sync,sf)
        self.canvas_sync.get_tk_widget().pack(fill='both',expand=True)

    def _poll_frames(self):
        try:
            while True:
                rgb=self._fq.get_nowait(); self._show_vid_frame(rgb)
        except queue.Empty: pass
        self.after(40,self._poll_frames)

    def _show_vid_frame(self, rgb):
        if not PIL_OK: return
        cw=max(self.vid_canvas.winfo_width(),200)
        ch=max(self.vid_canvas.winfo_height(),150)
        h,w=rgb.shape[:2]; scale=min(cw/w,ch/h)
        nw,nh=int(w*scale),int(h*scale)
        small=cv2.resize(rgb,(nw,nh),interpolation=cv2.INTER_NEAREST)
        img=ImageTk.PhotoImage(Image.fromarray(small))
        self._tk_img=img
        self.vid_canvas.delete("all")
        self.vid_canvas.create_image(cw//2,ch//2,image=img,anchor='center')

    def _load_csv_file(self, path):
        data={}
        with open(path) as f:
            reader=csv.DictReader(f); cols=reader.fieldnames or []
            for c in cols: data[c]=[]
            for row in reader:
                for c in cols:
                    try: data[c].append(float(row[c]))
                    except: data[c].append(0.0)
        return {k:np.array(v) for k,v in data.items()}

    def _load_w(self):
        path=filedialog.askopenfilename(filetypes=[("CSV","*.csv"),("All","*.*")])
        if not path: return
        try:
            self.csv_w=self._load_csv_file(path)
            self.sv_w.set(f"{os.path.basename(path)}  ({len(next(iter(self.csv_w.values())))} rows)")
        except Exception as e: messagebox.showerror("Error",str(e))

    def _load_a(self):
        path=filedialog.askopenfilename(filetypes=[("CSV","*.csv"),("All","*.*")])
        if not path: return
        try:
            self.csv_a=self._load_csv_file(path)
            self.sv_a.set(f"{os.path.basename(path)}  ({len(next(iter(self.csv_a.values())))} rows)")
        except Exception as e: messagebox.showerror("Error",str(e))

    def _load_v(self):
        path=filedialog.askopenfilename(
            filetypes=[("Video","*.mp4 *.MOV *.mov *.avi *.mkv"),("All","*.*")])
        if not path: return
        cap=cv2.VideoCapture(path)
        fps=cap.get(cv2.CAP_PROP_FPS) or 30.0
        nf=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.release()
        self.video_path=path
        self.sv_v.set(f"{os.path.basename(path)}  ({fps:.1f} fps, {nf/fps:.1f} s)")
        self.pick_btn.config(state='normal')

    def _use_live(self):
        tw,kw=self.store.snapshot_w()
        ta,ax,ay,az,tp,tr=self.store.snapshot_a()
        if len(tw)==0 and len(ta)==0:
            messagebox.showwarning("No data","No live data yet."); return
        self.csv_w={'time_ms':tw*1000,'weight_kg':kw,'sync':np.zeros(len(tw))}
        self.csv_a={'time_ms':ta*1000,'accel_x':ax,'accel_y':ay,'accel_z':az,
                    'pitch':tp,'roll':tr,'sync':np.zeros(len(ta))}
        with self.store._lock:
            for s in self.store.sync_ts:
                if len(self.csv_w['time_ms']):
                    iw=np.argmin(np.abs(self.csv_w['time_ms']-s)); self.csv_w['sync'][iw]=1
                if len(self.csv_a['time_ms']):
                    ia=np.argmin(np.abs(self.csv_a['time_ms']-s)); self.csv_a['sync'][ia]=1
        self.sv_w.set(f"Live weight ({len(tw)} rows)")
        self.sv_a.set(f"Live accel  ({len(ta)} rows)")

    def _open_picker(self):
        if not self.video_path: messagebox.showwarning("No video","Load a video first."); return
        def on_result(t_s):
            self.manual_video_sync_t=t_s
            self.sv_sync_status.set(f"Sync frame: t = {t_s:.3f} s")
        SyncPicker(self, self.video_path, self.led_roi, callback=on_result)

    def _start(self):
        if self.csv_w is None and self.csv_a is None:
            messagebox.showwarning("Missing","Load CSV files or use live data."); return
        if self.video_path is None:
            messagebox.showwarning("Missing","Load a video file."); return
        self.run_btn.config(state='disabled'); self.sv_run.set("Calibration…")
        self.progress['value']=0; self.after(50,self._calibrate)

    def _calibrate(self):
        try:
            cap=cv2.VideoCapture(self.video_path); ret,frame=cap.read(); cap.release()
            if not ret: raise RuntimeError("Cannot read first frame.")
            RESIZE=self._pvars['resize'].get()
            small=cv2.resize(frame,(0,0),fx=RESIZE,fy=RESIZE)
            self.sv_run.set("Step 1/4 — Click ruler endpoints"); self.update()
            ruler_mm=self._pvars['ruler'].get()
            pts=cv_click_two_points(small,"STEP 1: Ruler",f"Click both ends of the {ruler_mm:.0f} mm ruler")
            ruler_px=np.sqrt((pts[1][0]-pts[0][0])**2+(pts[1][1]-pts[0][1])**2)
            K=ruler_mm/ruler_px
            exp_r=(self._pvars['diam'].get()/2.0)/K
            min_r=max(3,int(0.85*exp_r)); max_r=max(min_r+1,int(1.15*exp_r))
            self.sv_run.set("Step 2/4 — Click attachment point"); self.update()
            att=cv_click_one_point(small,"STEP 2: Attachment","Click thread attachment point")
            self.sv_run.set("Step 3/4 — Draw sphere ROI"); self.update()
            roi=cv_select_roi_named(small,"STEP 3: Sphere ROI")
            self.led_roi=None
            if self.sync_mode.get()=="led_roi":
                self.sv_run.set("Step 4/4 — Draw LED ROI"); self.update()
                try: self.led_roi=cv_select_roi_named(small,"STEP 4: LED ROI")
                except RuntimeError: self.led_roi=None
            self._params={'K':K,'resize':RESIZE,'attachment':att,'min_r':min_r,'max_r':max_r,'roi':roi}
            self.sv_run.set("Running CV detection…")
            threading.Thread(target=self._cv_thread,daemon=True).start()
        except Exception as e:
            messagebox.showerror("Calibration error",str(e))
            self.run_btn.config(state='normal'); self.sv_run.set("Error")

    def _scan_led(self, video_path, led_roi):
        """Robust LED flash detection — same triple method as SyncPicker."""
        cap=cv2.VideoCapture(video_path)
        fps=cap.get(cv2.CAP_PROP_FPS) or 30.0
        raw_mean,raw_max,raw_col=[],[],[]
        while True:
            ret,f=cap.read()
            if not ret: break
            if led_roi:
                x,y,w,h=led_roi; region=f[y:y+h,x:x+w]
            else:
                H,W=f.shape[:2]; region=f[H//4:3*H//4,W//4:3*W//4]
            gray=cv2.cvtColor(region,cv2.COLOR_BGR2GRAY)
            raw_mean.append(float(np.mean(gray))); raw_max.append(float(np.max(gray)))
            hsv=cv2.cvtColor(region,cv2.COLOR_BGR2HSV)
            m_g=cv2.inRange(hsv,(35,60,60),(85,255,255))
            m_b=cv2.inRange(hsv,(0,0,200),(180,60,255))
            raw_col.append(float(np.sum(m_g)+np.sum(m_b))/255.0)
        cap.release()
        def norm(a):
            a=np.array(a,dtype=float); r=a.max()-a.min()
            return (a-a.min())/r if r>0 else a*0
        combined=norm(raw_mean)*0.3+norm(raw_max)*0.3+norm(raw_col)*0.4
        thr=np.mean(combined)+2.0*np.std(combined)
        spikes=np.where(combined>thr)[0]
        if len(spikes): return float(spikes[0])/fps
        return None

    def _cv_thread(self):
        try:
            def prog(v): self.after(0,lambda val=v:self.progress.configure(value=val))
            def push(rgb):
                try: self._fq.put_nowait(rgb)
                except queue.Full: pass
            results=run_pendulum_detection(self.video_path,self._params,prog,push)
            offset=0.0; led_t=None
            mode=self.sync_mode.get()
            if mode=="led_roi":
                led_t=self._scan_led(self.video_path,self.led_roi)
                if led_t is not None and self.csv_w is not None and 'sync' in self.csv_w:
                    idx=np.where(self.csv_w['sync']==1)[0]
                    if len(idx):
                        key='time_ms' if 'time_ms' in self.csv_w else 'time_s'
                        div=1000.0 if key=='time_ms' else 1.0
                        offset=self.csv_w[key][idx[0]]/div-led_t
                        self.after(0,lambda o=offset:self.sync_offset.set(round(o,3)))
                else:
                    self.after(0,lambda:messagebox.showwarning("Auto sync failed",
                        "Could not detect LED flash.\n"
                        "Use 'Manual frame picker' instead — it also shows the brightness curve."))
            elif mode=="manual_picker":
                if self.manual_video_sync_t is not None and self.csv_w is not None:
                    idx=np.where(self.csv_w.get('sync',np.array([]))==1)[0]
                    if len(idx):
                        key='time_ms' if 'time_ms' in self.csv_w else 'time_s'
                        div=1000.0 if key=='time_ms' else 1.0
                        offset=self.csv_w[key][idx[0]]/div-self.manual_video_sync_t
                        led_t=self.manual_video_sync_t
                        self.after(0,lambda o=offset:self.sync_offset.set(round(o,3)))
            else:
                offset=self.sync_offset.get()
            results['offset']=offset; results['led_t']=led_t
            self.vid_results=results
            self.after(0,self._show_results)
        except Exception as e:
            self.after(0,lambda:messagebox.showerror("CV Error",str(e)))
            self.after(0,lambda:self.run_btn.config(state='normal'))
            self.after(0,lambda:self.sv_run.set("Error"))

    def _sr(self, t):
        if len(t)<2: return 0.0
        dt=np.diff(t); dt=dt[dt>0]
        return float(1.0/np.median(dt)) if len(dt) else 0.0

    def _show_results(self):
        r=self.vid_results; t_vid=r['times']+r['offset']
        def g(d,*keys):
            for k in keys:
                if d and k in d: return d[k]
            return np.array([])
        tw_ms=g(self.csv_w,'time_ms'); tw=tw_ms/1000.0 if len(tw_ms) else np.array([])
        kg=g(self.csv_w,'weight_kg')
        ta_ms=g(self.csv_a,'time_ms'); ta=ta_ms/1000.0 if len(ta_ms) else np.array([])
        ax_=g(self.csv_a,'accel_x'); ay_=g(self.csv_a,'accel_y'); az_=g(self.csv_a,'accel_z')
        pitch=g(self.csv_a,'pitch'); roll=g(self.csv_a,'roll')
        norm=np.sqrt(ax_**2+ay_**2+az_**2) if len(ax_) else np.array([])
        sw=g(self.csv_w,'sync'); sync_t=tw[sw==1] if len(tw) and len(sw) else np.array([])
        sr_w=self._sr(tw); sr_a=self._sr(ta); sr_v=self._sr(t_vid)

        # Sync timeline
        self.ax_sync.clear(); style_axes(self.ax_sync,self.fig_sync)
        self.ax_sync.set_title("Sync alignment",fontsize=8,color=TEXT)
        self.ax_sync.set_xlabel("Time  s",fontsize=7)
        for st in sync_t: self.ax_sync.axvline(st,color=ORANGE,lw=2,label='Arduino ⚡')
        if r['led_t'] is not None:
            self.ax_sync.axvline(r['led_t']+r['offset'],color=GREEN,lw=2,ls='--',
                                 label=f"LED → offset={r['offset']:.3f}s")
        self.ax_sync.set_yticks([])
        h,l=self.ax_sync.get_legend_handles_labels()
        if h: self.ax_sync.legend(fontsize=7,labelcolor=TEXT,facecolor=CARD,edgecolor=BORDER)
        self.canvas_sync.draw()

        # Results grid
        self.fig_res.clear()
        gs=self.fig_res.add_gridspec(3,2,hspace=0.55,wspace=0.38,
                                     left=0.09,right=0.97,top=0.95,bottom=0.07)
        axes=[self.fig_res.add_subplot(gs[i,j]) for i in range(3) for j in range(2)]
        style_axes(axes,self.fig_res)

        def sr_tag(ax,sr,col=MUTED):
            ax.text(0.99,0.95,f"SR: {sr:.1f} Hz",transform=ax.transAxes,
                    ha='right',va='top',fontsize=7,color=col,
                    bbox=dict(facecolor=CARD,edgecolor='none',alpha=0.7,pad=2))

        def vsync(ax):
            for st in sync_t: ax.axvline(st,color=ORANGE,lw=1,ls=':',alpha=0.7)

        a0=axes[0]
        if len(t_vid):
            a0.plot(t_vid,r['angles'],color=ACCENT,lw=0.9,label='θ')
            mu=np.mean(r['angles'])
            a0.axhline(mu,color='#b4befe',lw=1.5,ls='--',label=f'Mean {mu:.2f}°')
            a0.fill_between(t_vid,mu-np.std(r['angles']),mu+np.std(r['angles']),alpha=0.12,color=ACCENT)
        vsync(a0); a0.set_ylabel('θ (°)'); a0.set_title('Pendulum deflection')
        a0.legend(fontsize=7,labelcolor=TEXT,facecolor=CARD,edgecolor=BORDER); sr_tag(a0,sr_v,ACCENT)

        a1=axes[1]
        if len(tw): a1.plot(tw,kg,color=GREEN,lw=0.9)
        for st in sync_t: a1.axvline(st,color=ORANGE,lw=1.5,ls='--',alpha=0.85)
        if r['led_t'] is not None: a1.axvline(r['led_t']+r['offset'],color='#00ff99',lw=1.5,alpha=0.7)
        a1.set_ylabel('Weight kg'); a1.set_title('Load cell  (⚡ = sync)'); sr_tag(a1,sr_w,GREEN)

        a2=axes[2]
        if len(ta): a2.plot(ta,norm,color=ORANGE,lw=0.9)
        vsync(a2); a2.set_ylabel('|a| m/s²'); a2.set_title('Acceleration magnitude'); sr_tag(a2,sr_a,ORANGE)

        a3=axes[3]
        if len(t_vid) and len(r['L_mm']):
            a3.plot(t_vid,r['L_mm'],color=PURPLE,lw=0.9)
            a3.axhline(np.mean(r['L_mm']),color='#b4befe',lw=1.5,ls='--')
        a3.set_ylabel('L mm'); a3.set_title('Pendulum length consistency'); sr_tag(a3,sr_v,PURPLE)

        a4=axes[4]
        if len(ta):
            a4.plot(ta,pitch,color=ACCENT,lw=0.8,label='Pitch')
            a4.plot(ta,roll, color=RED,   lw=0.8,label='Roll')
        vsync(a4); a4.set_ylabel('°'); a4.set_title('IMU tilt angles')
        a4.legend(fontsize=7,labelcolor=TEXT,facecolor=CARD,edgecolor=BORDER); sr_tag(a4,sr_a,MUTED)

        a5=axes[5]
        if len(t_vid) and len(tw) and len(kg):
            w_i=np.interp(t_vid,tw,kg)
            sc=a5.scatter(w_i,r['angles'],c=t_vid,cmap='plasma',s=3,alpha=0.7)
            self.fig_res.colorbar(sc,ax=a5,label='time s')
        a5.set_xlabel('Weight kg'); a5.set_ylabel('θ °'); a5.set_title('Angle vs Load')

        self.canvas_res.draw()
        n_ok=len(r['angles']); n_tot=r['n_total']; pct=100*n_ok/n_tot if n_tot else 0
        self.sv_run.set(f"Done — {n_ok}/{n_tot} ({pct:.0f}%)  W:{sr_w:.0f}Hz A:{sr_a:.0f}Hz V:{sr_v:.0f}Hz  offset={r['offset']:.3f}s")
        self.run_btn.config(state='normal'); self.exp_btn.config(state='normal')
        self.csv_btn.config(state='normal')
        self.progress['value']=100

    def _export(self):
        fn=f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        path=filedialog.asksaveasfilename(defaultextension=".png",
               filetypes=[("PNG","*.png"),("PDF","*.pdf")],initialfile=fn)
        if path: self.fig_res.savefig(path,dpi=150,facecolor=BG); messagebox.showinfo("Exported",f"Saved:\n{path}")

    def _export_csv(self):
        """
        Export one comprehensive CSV with all data on a unified time axis.

        Columns:
          time_s              — unified time in seconds (Arduino clock, offset-corrected)
          source              — 'weight' | 'accel' | 'video'
          weight_kg           — load cell reading (NaN for accel/video rows)
          accel_x/y/z         — acceleration m/s² (NaN for weight/video rows)
          norm_accel          — |a| m/s²
          pitch_deg           — IMU pitch (NaN for weight/video rows)
          roll_deg            — IMU roll  (NaN for weight/video rows)
          angle_deg           — pendulum angle from video (NaN for arduino rows)
          L_mm                — pendulum length from video  (NaN for arduino rows)
          sync                — 1 on the row closest to a sync/trigger event, else 0
        """
        if self.vid_results is None:
            messagebox.showwarning("No results","Run the analysis first."); return

        fn=f"full_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path=filedialog.asksaveasfilename(defaultextension=".csv",
               filetypes=[("CSV","*.csv")],initialfile=fn)
        if not path: return

        r=self.vid_results; offset=r['offset']

        def g(d,*keys):
            for k in keys:
                if d and k in d: return d[k]
            return np.array([])

        # ── Build rows ────────────────────────────────────────────────────────
        rows=[]
        NaN=float('nan')

        # Weight rows
        tw_ms=g(self.csv_w,'time_ms')
        kg=g(self.csv_w,'weight_kg')
        sw=g(self.csv_w,'sync')
        for i in range(len(tw_ms)):
            rows.append({
                'time_s':   tw_ms[i]/1000.0,
                'source':   'weight',
                'weight_kg':kg[i] if i<len(kg) else NaN,
                'accel_x':  NaN,'accel_y':NaN,'accel_z':NaN,
                'norm_accel':NaN,'pitch_deg':NaN,'roll_deg':NaN,
                'angle_deg':NaN,'L_mm':NaN,
                'sync':     int(sw[i]) if i<len(sw) else 0,
            })

        # Accel rows
        ta_ms=g(self.csv_a,'time_ms')
        ax_=g(self.csv_a,'accel_x'); ay_=g(self.csv_a,'accel_y'); az_=g(self.csv_a,'accel_z')
        pitch=g(self.csv_a,'pitch'); roll=g(self.csv_a,'roll')
        sa=g(self.csv_a,'sync')
        for i in range(len(ta_ms)):
            axv=ax_[i] if i<len(ax_) else NaN
            ayv=ay_[i] if i<len(ay_) else NaN
            azv=az_[i] if i<len(az_) else NaN
            norm=np.sqrt(axv**2+ayv**2+azv**2) if not any(np.isnan([axv,ayv,azv])) else NaN
            rows.append({
                'time_s':   ta_ms[i]/1000.0,
                'source':   'accel',
                'weight_kg':NaN,
                'accel_x':  axv,'accel_y':ayv,'accel_z':azv,
                'norm_accel':norm,
                'pitch_deg':pitch[i] if i<len(pitch) else NaN,
                'roll_deg': roll[i]  if i<len(roll)  else NaN,
                'angle_deg':NaN,'L_mm':NaN,
                'sync':     int(sa[i]) if i<len(sa) else 0,
            })

        # Video rows — already offset-corrected to Arduino clock
        t_vid=r['times']+offset
        angles=r['angles']; L_mm=r['L_mm']
        for i in range(len(t_vid)):
            rows.append({
                'time_s':   t_vid[i],
                'source':   'video',
                'weight_kg':NaN,
                'accel_x':  NaN,'accel_y':NaN,'accel_z':NaN,
                'norm_accel':NaN,'pitch_deg':NaN,'roll_deg':NaN,
                'angle_deg':angles[i] if i<len(angles) else NaN,
                'L_mm':     L_mm[i]   if i<len(L_mm)   else NaN,
                'sync':     0,
            })

        # Sort by time
        rows.sort(key=lambda x: x['time_s'])

        # Write
        cols=['time_s','source','weight_kg','accel_x','accel_y','accel_z',
              'norm_accel','pitch_deg','roll_deg','angle_deg','L_mm','sync']
        with open(path,'w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=cols)
            w.writeheader()
            for row in rows:
                # Format floats nicely, keep NaN as empty string
                out={}
                for k,v in row.items():
                    if k=='source' or k=='sync': out[k]=v
                    elif isinstance(v,float) and np.isnan(v): out[k]=''
                    else: out[k]=f"{v:.6f}"
                w.writerow(out)

        n=len(rows); nw=sum(1 for r in rows if r['source']=='weight')
        na=sum(1 for r in rows if r['source']=='accel')
        nv=sum(1 for r in rows if r['source']=='video')
        messagebox.showinfo("CSV Exported",
            f"Saved {n} rows to:\n{path}\n\n"
            f"  Weight rows : {nw}\n"
            f"  Accel rows  : {na}\n"
            f"  Video rows  : {nv}\n"
            f"  Sync offset : {offset:.4f} s")

    def _placeholder(self):
        self.fig_res.clear(); ax=self.fig_res.add_subplot(111); ax.set_facecolor(CARD)
        ax.text(0.5,0.55,"Load CSV + Video\nthen ▶ Run Analysis",
                ha='center',va='center',transform=ax.transAxes,color=MUTED,fontsize=13,family='monospace')
        ax.axis('off'); self.canvas_res.draw()


# ══════════════════════════════════════════════════════════════════════════════
# MAIN APP
# ══════════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ME-301  |  Measurement Lab"); self.geometry("1300x880"); self.configure(bg=BG)
        self._style(); store=DataStore(); nb=ttk.Notebook(self)
        nb.pack(fill='both',expand=True,padx=8,pady=8)
        nb.add(CaptureTab(nb,store), text="   ⏺  Capture   ")
        nb.add(AnalyseTab(nb,store), text="   📊  Analyse   ")

    def _style(self):
        s=ttk.Style(self); s.theme_use('clam')
        s.configure('.',background=BG,foreground=TEXT,fieldbackground=CARD)
        s.configure('TFrame',background=BG)
        s.configure('TLabelframe',background=BG,foreground=ACCENT)
        s.configure('TLabelframe.Label',background=BG,foreground=ACCENT,font=('Helvetica',9,'bold'))
        s.configure('TButton',background=CARD,foreground=TEXT,borderwidth=0,padding=7,relief='flat')
        s.map('TButton',background=[('active',ACCENT),('pressed','#3a6fd8')])
        s.configure('TLabel',background=BG,foreground=TEXT)
        s.configure('TEntry',fieldbackground=CARD,foreground=TEXT)
        s.configure('TCombobox',fieldbackground=CARD,foreground=TEXT)
        s.configure('TRadiobutton',background=BG,foreground=TEXT)
        s.configure('TNotebook',background=BG,tabmargins=[4,4,0,0])
        s.configure('TNotebook.Tab',background=CARD,foreground=MUTED,padding=[14,6])
        s.map('TNotebook.Tab',background=[('selected',BG)],foreground=[('selected',ACCENT)])
        s.configure('Horizontal.TProgressbar',troughcolor=CARD,background=ACCENT)

if __name__=="__main__":
    App().mainloop()
