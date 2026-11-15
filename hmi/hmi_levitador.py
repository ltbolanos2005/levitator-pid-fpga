# -*- coding: utf-8 -*-
"""
HMI - Levitador Neumático (Fase 2: sistema real, sin simulación)
=================================================================
PC  <-- USB / Wi-Fi -->  ESP32 (maestro Modbus RTU)  <-- RS485 -->  FPGA (esclavo, PID en VHDL)

Protocolo PC <-> ESP32 (ASCII, una trama por línea, terminada en '\\n'):

  ESP32 -> PC  (telemetría, ~10 Hz)
      *PV,SP,ERR,PWM,EN,KP,KI,KD,MB,ST#
        PV / SP / ERR : ALTURAS en cm (0 = fondo del tubo; el ESP32 convierte
                        las distancias de la FPGA: altura = H0 - distancia)
        PWM           : 0-255 (duty)  EN : 0/1 (lazo)
        KP/KI/KD      : ganancias leídas de la FPGA
                        Kp [PWM/cm], Ki [PWM/(cm*s)], Kd [PWM*s/cm]
        MB            : código Modbus de la última lectura (0 = OK)
        ST            : registro STATUS de la FPGA (opcional; tramas de 9 campos
                        siguen siendo válidas)
      *ACK,<SP|PID|EN>,<código Modbus>#  (respuesta a un comando)

  PC -> ESP32  (comandos)
      *SP,20.00#              nueva referencia (cm)
      *PID,1.60,0.20,1.50#    ganancias Kp, Ki, Kd
      *EN,1#                  1 = habilitar lazo, 0 = deshabilitar

Dependencias:   pip install customtkinter pyserial matplotlib
"""

import csv
import queue
import socket
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import customtkinter as ctk
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import serial  # noqa: E402
from serial.tools import list_ports  # noqa: E402

# ----------------------------------------------------------------------------
# Configuración
# ----------------------------------------------------------------------------
APP_TITLE = "HMI - Levitador Neumático (v2 · Sistema real)"
DIST_MAX = 46.0          # cm, altura del tubo (dibujo y grafica) = H0_CM del ESP32
SP_MAX = 37.0            # cm, referencia maxima = ALT_MAX_CM del ESP32
# Límites de las ganancias en la FPGA (si se pasan, responde excepción 03)
PID_MAX = {"Kp": 159.99, "Ki": 655.35, "Kd": 7.99}
PID_DEFAULT = {"Kp": "20.00", "Ki": "3.13", "Kd": "2.00"}  # = valores de arranque de la FPGA (sintonia 2026-11-10)
TCP_PORT = 8888          # puerto del servidor TCP del ESP32 (modo Wi-Fi)
MAX_POINTS = 4000        # muestras guardadas para la gráfica
MAX_TREE = 300           # filas máximas en la tabla de histórico
MAX_LOG = 400            # líneas máximas en el monitor de datos
HIST_EVERY = 5           # 1 de cada N muestras va a la tabla (el CSV guarda todas)
STABLE_BAND = 1.0        # cm, banda para declarar "estable"
NO_DATA_TIMEOUT = 1.5    # s sin telemetría -> "Sin datos"

# Paleta (inspirada en la interfaz de referencia)
BG, PANEL, PANEL_2, BORDER = "#0b1220", "#1e293b", "#0f172a", "#334155"
TXT, MUTED = "#e2e8f0", "#94a3b8"
C_PV, C_SP, C_ERR, C_PWM, C_STATE = "#7c3aed", "#047857", "#c2410c", "#2563eb", "#0f766e"
LINE_PV, LINE_SP, LINE_PWM = "#4da3ff", "#ef4444", "#facc15"

MB_ERRORS = {
    0: "Comunicación OK",
    1: "Función ilegal",
    2: "Dirección de registro ilegal",
    3: "Valor de dato ilegal",
    4: "Falla del esclavo",
    224: "ID de esclavo inválido",
    225: "Función inválida",
    226: "Timeout: la FPGA no responde",
    227: "CRC inválido",
    255: "Comando mal formado",
}

# Bits del registro STATUS de la FPGA
ST_SENSOR, ST_TIMEOUT, ST_SAT_HI, ST_SAT_LO, ST_FRAME = 0x02, 0x04, 0x08, 0x10, 0x20


# ----------------------------------------------------------------------------
# Capa de transporte (USB-Serial o TCP/Wi-Fi) con hilo lector
# ----------------------------------------------------------------------------
class Link:
    """Entrega líneas completas a una cola. El hilo nunca toca la GUI."""

    def __init__(self, rx_queue):
        self.q = rx_queue
        self._ser = None
        self._sock = None
        self._thread = None
        self._run = False
        self._lock = threading.Lock()

    @property
    def connected(self):
        return self._run

    def open_serial(self, port, baud):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.1
        ser.dtr = False   # evita (en la mayoría de placas) reiniciar el ESP32 al abrir
        ser.rts = False
        ser.open()
        self._ser, self._sock = ser, None
        self._start()

    def open_tcp(self, host, port=TCP_PORT):
        sock = socket.create_connection((host, port), timeout=3)
        sock.settimeout(0.1)
        self._sock, self._ser = sock, None
        self._start()

    def _start(self):
        self._run = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        ser, sock = self._ser, self._sock
        buf = b""
        try:
            while self._run:
                if ser is not None:
                    chunk = ser.read(ser.in_waiting or 1)
                else:
                    try:
                        chunk = sock.recv(512)
                        if chunk == b"":
                            raise ConnectionError("el ESP32 cerró la conexión")
                    except socket.timeout:
                        chunk = b""
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    txt = line.decode("ascii", errors="replace").strip()
                    if txt:
                        self.q.put(("line", txt))
        except Exception as exc:  # noqa: BLE001
            if self._run:  # error real, no un cierre voluntario
                self.q.put(("error", str(exc)))

    def send(self, text):
        data = (text + "\n").encode("ascii")
        with self._lock:
            if self._ser is not None:
                self._ser.write(data)
            elif self._sock is not None:
                self._sock.sendall(data)
            else:
                raise ConnectionError("sin conexión")

    def close(self):
        self._run = False
        for res in (self._ser, self._sock):
            try:
                if res is not None:
                    res.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread and self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(0.5)
        self._ser = self._sock = self._thread = None


def parse_ack(line):
    """'*ACK,PID,0#' -> ('PID', 0), o None."""
    if not (line.startswith("*ACK,") and line.endswith("#")):
        return None
    parts = line[1:-1].split(",")
    if len(parts) != 3:
        return None
    try:
        return parts[1], int(parts[2])
    except ValueError:
        return None


def parse_telemetry(line):
    """'*pv,sp,err,pwm,en,kp,ki,kd,mb[,st]#' -> dict, o None si no es telemetría."""
    if not (line.startswith("*") and line.endswith("#")):
        return None
    parts = line[1:-1].split(",")
    if len(parts) not in (9, 10):
        return None
    try:
        return {
            "pv": float(parts[0]), "sp": float(parts[1]), "err": float(parts[2]),
            "pwm": int(float(parts[3])), "en": int(float(parts[4])),
            "kp": float(parts[5]), "ki": float(parts[6]), "kd": float(parts[7]),
            "mb": int(float(parts[8])),
            "st": int(float(parts[9])) if len(parts) == 10 else 0,
        }
    except ValueError:
        return None


# ----------------------------------------------------------------------------
# Widgets propios
# ----------------------------------------------------------------------------
class KpiCard(ctk.CTkFrame):
    def __init__(self, master, title, color, big=28):
        super().__init__(master, fg_color=color, corner_radius=12)
        ctk.CTkLabel(self, text=title.upper(), font=("Segoe UI", 11),
                     text_color="#e2e8f0").pack(anchor="w", padx=14, pady=(10, 0))
        self.value = ctk.CTkLabel(self, text="--", font=("Segoe UI", big, "bold"),
                                  text_color="white", anchor="w", justify="left")
        self.value.pack(anchor="w", padx=14)
        self.sub = ctk.CTkLabel(self, text=" ", font=("Segoe UI", 11), text_color="#e2e8f0")
        self.sub.pack(anchor="w", padx=14, pady=(0, 10))

    def set(self, value, sub=None, color=None):
        self.value.configure(text=value)
        if sub is not None:
            self.sub.configure(text=sub)
        if color is not None:
            self.configure(fg_color=color)


class TubeView(tk.Canvas):
    """Tubo vertical con la esfera (PV) y la línea de referencia (SP)."""

    def __init__(self, master, max_cm=DIST_MAX, w=130, h=340):
        super().__init__(master, width=w, height=h, bg=PANEL, highlightthickness=0)
        self.max_cm = max_cm
        self.top, self.bot = 20, h - 20
        self.cx, self.half, self.r = w // 2 + 14, 24, 14
        self.create_rectangle(self.cx - self.half, self.top, self.cx + self.half, self.bot,
                              outline="#64748b", dash=(4, 4), width=2)
        for cm in range(0, int(max_cm) + 1, 6):
            y = self._y(cm)
            self.create_line(self.cx - self.half - 8, y, self.cx - self.half, y, fill="#64748b")
            self.create_text(self.cx - self.half - 11, y, text=str(cm), fill=MUTED,
                             anchor="e", font=("Segoe UI", 8))
        self.sp_line = self.create_line(0, 0, 0, 0, fill=LINE_SP, dash=(5, 3), width=2)
        self.ball = self.create_oval(0, 0, 0, 0, fill=LINE_PV, outline="#bfdbfe", width=2)
        self.set(0, 0)

    def _y(self, cm):
        cm = min(max(cm, 0), self.max_cm)
        return self.bot - cm / self.max_cm * (self.bot - self.top)

    def set(self, pv, sp):
        y = min(max(self._y(pv), self.top + self.r), self.bot - self.r)
        self.coords(self.ball, self.cx - self.r, y - self.r, self.cx + self.r, y + self.r)
        ys = self._y(sp)
        self.coords(self.sp_line, self.cx - self.half - 4, ys, self.cx + self.half + 4, ys)


# ----------------------------------------------------------------------------
# Aplicación
# ----------------------------------------------------------------------------
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.title(APP_TITLE)
        self.geometry("1320x880")
        self.minsize(1000, 640)
        self.after(0, self._maximize)         # abrir maximizada
        self.configure(fg_color=BG)

        # Estado
        self.rx_q = queue.Queue()
        self.link = Link(self.rx_q)
        self.t0 = None
        self.last_rx = 0.0
        self.synced = False
        self.pending_pid = None
        self.pending_t = 0.0
        self.sample_n = 0
        self.plot_dirty = False
        self.win_s = 30
        self.last_ip = "192.168.1.50"
        self.err_hist = deque(maxlen=10)
        self.t_d = deque(maxlen=MAX_POINTS)
        self.pv_d = deque(maxlen=MAX_POINTS)
        self.sp_d = deque(maxlen=MAX_POINTS)
        self.pwm_d = deque(maxlen=MAX_POINTS)
        self.records = []

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self._build_topbar()
        self._build_cards()
        self._build_main()
        self._build_bottom()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh_ports()
        self.after(30, self._poll_queue)
        self.after(100, self._refresh_plot)
        self.after(500, self._watchdog)

    def _maximize(self):
        try:
            self.state("zoomed")               # Windows / macOS
        except tk.TclError:
            try:
                self.attributes("-zoomed", True)   # Linux
            except tk.TclError:
                pass

    # ------------------------------------------------------------------ UI
    def _build_topbar(self):
        bar = ctk.CTkFrame(self, fg_color=PANEL_2, corner_radius=0, height=58)
        bar.grid(row=0, column=0, sticky="ew")
        ctk.CTkLabel(bar, text="Control de Levitación Neumática",
                     font=("Segoe UI", 18, "bold")).pack(side="left", padx=18, pady=12)

        self.btn_conn = ctk.CTkButton(bar, text="Conectar", width=120, fg_color="#2563eb",
                                      command=self.toggle_connection)
        self.btn_conn.pack(side="right", padx=(8, 16))
        self.lbl_link = ctk.CTkLabel(bar, text="Desconectado", text_color=MUTED)
        self.lbl_link.pack(side="right", padx=(0, 4))
        self.led = ctk.CTkLabel(bar, text="●", text_color="#ef4444", font=("Segoe UI", 16))
        self.led.pack(side="right")
        ctk.CTkLabel(bar, text="Control remoto · PC", fg_color="#c2410c", corner_radius=14,
                     font=("Segoe UI", 11, "bold"), padx=12, pady=3).pack(side="right", padx=12)

        self.cmb_baud = ctk.CTkComboBox(bar, values=["9600", "19200", "57600", "115200", "230400"],
                                        width=100)
        self.cmb_baud.set("115200")
        self.cmb_baud.pack(side="right", padx=(4, 0))
        ctk.CTkLabel(bar, text="Baudios", text_color=MUTED).pack(side="right", padx=(10, 0))
        ctk.CTkButton(bar, text="⟳", width=32, fg_color="#334155", hover_color="#475569",
                      command=self.refresh_ports).pack(side="right", padx=(4, 0))
        self.cmb_port = ctk.CTkComboBox(bar, values=[""], width=130)
        self.cmb_port.pack(side="right", padx=(4, 0))
        self.lbl_port = ctk.CTkLabel(bar, text="Puerto", text_color=MUTED)
        self.lbl_port.pack(side="right", padx=(10, 0))
        self.mode = ctk.CTkOptionMenu(bar, values=["USB (Serial)", "Wi-Fi (TCP)"], width=120,
                                      command=self.on_mode)
        self.mode.pack(side="right", padx=(0, 6))

    def _build_cards(self):
        row = ctk.CTkFrame(self, fg_color="transparent")
        row.grid(row=1, column=0, sticky="ew", padx=10, pady=(14, 8))
        for i in range(5):
            row.grid_columnconfigure(i, weight=1, uniform="kpi")
        self.card_pv = KpiCard(row, "Altura actual (PV)", C_PV)
        self.card_sp = KpiCard(row, "Referencia (SP)", C_SP)
        self.card_err = KpiCard(row, "Error", C_ERR)
        self.card_pwm = KpiCard(row, "Señal de control (PWM)", C_PWM)
        self.card_state = KpiCard(row, "Estado del sistema", "#475569", big=20)
        for i, c in enumerate((self.card_pv, self.card_sp, self.card_err, self.card_pwm, self.card_state)):
            c.grid(row=0, column=i, sticky="nsew", padx=6)
        self.card_state.set("Desconectado", "Sin enlace con el ESP32")

    def _build_main(self):
        main = ctk.CTkFrame(self, fg_color="transparent")
        main.grid(row=2, column=0, sticky="nsew", padx=10, pady=4)
        main.grid_columnconfigure(2, weight=1)
        main.grid_rowconfigure(0, weight=1)
        self._build_left(main)
        self._build_tube(main)
        self._build_plot(main)

    def _section(self, parent, text):
        ctk.CTkLabel(parent, text=text, font=("Segoe UI", 11), text_color=MUTED).pack(
            anchor="w", padx=16, pady=(14, 2))

    def _build_left(self, main):
        # Panel con barra de desplazamiento: en pantallas pequeñas nada queda oculto
        left = ctk.CTkScrollableFrame(main, fg_color=PANEL, corner_radius=12, width=285)
        left.grid(row=0, column=0, sticky="ns", padx=6)

        # Control del lazo primero (es lo más importante)
        self._section(left, "CONTROL DEL LAZO")
        r = ctk.CTkFrame(left, fg_color="transparent")
        r.pack(fill="x", padx=16)
        ctk.CTkButton(r, text="Habilitar", width=120, fg_color="#15803d", hover_color="#166534",
                      font=("Segoe UI", 12, "bold"), command=lambda: self.send("*EN,1#")).pack(side="left")
        ctk.CTkButton(r, text="Deshabilitar", width=120, fg_color="#b91c1c", hover_color="#991b1b",
                      font=("Segoe UI", 12, "bold"), command=lambda: self.send("*EN,0#")).pack(side="left", padx=8)

        self._section(left, "REFERENCIA DE ALTURA (SP)")
        self.lbl_sp_big = ctk.CTkLabel(left, text="20.0 cm", font=("Segoe UI", 28, "bold"))
        self.lbl_sp_big.pack(anchor="w", padx=16)
        self.sld = ctk.CTkSlider(left, from_=0, to=SP_MAX, number_of_steps=int(SP_MAX * 10),
                                 command=self._on_slider)
        self.sld.set(20.0)
        self.sld.pack(fill="x", padx=16, pady=(6, 0))
        self.sld.bind("<ButtonRelease-1>", lambda _e: self.send_sp())
        r = ctk.CTkFrame(left, fg_color="transparent")
        r.pack(fill="x", padx=16, pady=(8, 0))
        self.ent_sp = ctk.CTkEntry(r, width=90, justify="center")
        self.ent_sp.insert(0, "20.0")
        self.ent_sp.pack(side="left")
        self.ent_sp.bind("<Return>", self._on_sp_entry)
        ctk.CTkButton(r, text="Enviar SP", width=90, fg_color="#334155", hover_color="#475569",
                      command=self._on_sp_entry).pack(side="left", padx=8)
        ctk.CTkLabel(left, text=f"Rango 0 – {SP_MAX:.0f} cm · se envía al soltar el slider",
                     font=("Segoe UI", 10), text_color=MUTED).pack(anchor="w", padx=16, pady=(4, 0))

        self._section(left, "PARÁMETROS PID")
        g = ctk.CTkFrame(left, fg_color="transparent")
        g.pack(fill="x", padx=16)
        self.ent = {}
        for i, (name, default) in enumerate(PID_DEFAULT.items()):
            ctk.CTkLabel(g, text=name, font=("Segoe UI", 12, "bold")).grid(row=0, column=2 * i, padx=(0, 3))
            e = ctk.CTkEntry(g, width=54, justify="center")
            e.insert(0, default)
            e.grid(row=0, column=2 * i + 1, padx=(0, 7))
            self.ent[name] = e
        ctk.CTkButton(left, text="Aplicar parámetros", fg_color="#334155", hover_color="#475569",
                      command=self.apply_pid).pack(fill="x", padx=16, pady=(10, 2))
        self.lbl_pid_fpga = ctk.CTkLabel(left, text="FPGA → Kp -- · Ki -- · Kd --",
                                         font=("Consolas", 11), text_color=MUTED)
        self.lbl_pid_fpga.pack(anchor="w", padx=16)
        self.lbl_pid_status = ctk.CTkLabel(left, text=" ", font=("Segoe UI", 11))
        self.lbl_pid_status.pack(anchor="w", padx=16, pady=(0, 10))

    def _build_tube(self, main):
        tube = ctk.CTkFrame(main, fg_color=PANEL, corner_radius=12)
        tube.grid(row=0, column=1, sticky="ns", padx=6)
        ctk.CTkLabel(tube, text="LEVITADOR", font=("Segoe UI", 11), text_color=MUTED).pack(pady=(12, 4))
        self.tube = TubeView(tube)
        self.tube.pack(padx=8)
        ctk.CTkLabel(tube, text="● altura (PV)\n- - - referencia (SP)", font=("Segoe UI", 10),
                     text_color=MUTED, justify="left").pack(pady=(4, 10))

    def _build_plot(self, main):
        plot = ctk.CTkFrame(main, fg_color=PANEL, corner_radius=12)
        plot.grid(row=0, column=2, sticky="nsew", padx=6)
        hdr = ctk.CTkFrame(plot, fg_color="transparent")
        hdr.pack(fill="x", padx=14, pady=(10, 0))
        ctk.CTkLabel(hdr, text="MONITOREO EN TIEMPO REAL · PV, SP y PWM", font=("Segoe UI", 11),
                     text_color=MUTED).pack(side="left")
        ctk.CTkButton(hdr, text="Exportar CSV", width=100, height=26, fg_color="#334155",
                      hover_color="#475569", command=self.export_csv).pack(side="right")
        ctk.CTkButton(hdr, text="Limpiar", width=70, height=26, fg_color="#334155",
                      hover_color="#475569", command=self.clear_data).pack(side="right", padx=6)
        win_menu = ctk.CTkOptionMenu(hdr, values=["10 s", "30 s", "60 s", "120 s"], width=80, height=26,
                                     command=self._on_window)
        win_menu.set(f"{self.win_s} s")
        win_menu.pack(side="right")
        ctk.CTkLabel(hdr, text="Ventana", font=("Segoe UI", 11), text_color=MUTED).pack(side="right", padx=6)
        # Gráfica
        fig = Figure(figsize=(6, 3.6), dpi=100, facecolor=PANEL)
        self.ax = fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        for a in (self.ax, self.ax2):
            a.set_facecolor(PANEL_2)
            a.tick_params(colors=MUTED, labelsize=8)
            for s in a.spines.values():
                s.set_color(BORDER)
        self.ax.grid(True, color=BORDER, alpha=0.5, lw=0.5)
        self.ax.set_xlabel("tiempo (s)", color=MUTED, fontsize=9)
        self.ax.set_ylabel("altura (cm)", color=MUTED, fontsize=9)
        self.ax2.set_ylabel("PWM (%)", color=MUTED, fontsize=9)
        self.ax.set_ylim(0, DIST_MAX + 4)
        self.ax2.set_ylim(0, 105)
        self.ax.set_xlim(0, self.win_s)
        (self.l_pv,) = self.ax.plot([], [], color=LINE_PV, lw=2, label="PV (altura)")
        (self.l_sp,) = self.ax.plot([], [], color=LINE_SP, lw=1.5, ls="--", label="SP")
        (self.l_pwm,) = self.ax2.plot([], [], color=LINE_PWM, lw=1.5, ls=":", label="PWM (%)")
        self.ax2.legend(handles=[self.l_pv, self.l_sp, self.l_pwm], loc="upper left", ncol=3,
                        facecolor=PANEL, edgecolor=BORDER, labelcolor=TXT, fontsize=8)
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=plot)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

    def _build_bottom(self):
        tabs = ctk.CTkTabview(self, height=170, fg_color=PANEL, corner_radius=12)
        tabs.grid(row=3, column=0, sticky="ew", padx=16, pady=(8, 14))
        t_hist = tabs.add("Histórico")
        t_mon = tabs.add("Monitor de datos")

        # Histórico (tabla)
        st = ttk.Style()
        st.theme_use("clam")
        st.configure("Treeview", background=PANEL_2, fieldbackground=PANEL_2, foreground=TXT,
                     rowheight=22, borderwidth=0)
        st.configure("Treeview.Heading", background=BORDER, foreground="white", relief="flat",
                     font=("Segoe UI", 10, "bold"))
        st.map("Treeview", background=[("selected", "#2563eb")])
        box = ctk.CTkFrame(t_hist, fg_color="transparent")
        box.pack(fill="both", expand=True)
        cols = ("Hora", "SP (cm)", "PV (cm)", "Error (cm)", "PWM (%)")
        self.tree = ttk.Treeview(box, columns=cols, show="headings", height=5)
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, anchor="center", width=120)
        sb = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        # Monitor de datos (tramas crudas)
        self.txt = ctk.CTkTextbox(t_mon, font=("Consolas", 12), fg_color=BG, text_color="#86efac",
                                  state="disabled")
        self.txt.pack(fill="both", expand=True, padx=4, pady=(2, 4))
        row = ctk.CTkFrame(t_mon, fg_color="transparent")
        row.pack(fill="x", padx=4)
        self.ent_cmd = ctk.CTkEntry(row, placeholder_text="Comando manual, p. ej. *SP,18.50#")
        self.ent_cmd.pack(side="left", fill="x", expand=True)
        self.ent_cmd.bind("<Return>", self._send_manual)
        ctk.CTkButton(row, text="Enviar", width=70, command=self._send_manual).pack(side="left", padx=6)
        self.var_pause = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(row, text="Pausar", variable=self.var_pause, width=70).pack(side="left", padx=6)
        ctk.CTkButton(row, text="Limpiar", width=70, fg_color="#334155", hover_color="#475569",
                      command=self._clear_log).pack(side="left")

    # --------------------------------------------------------- Conexión
    def refresh_ports(self):
        if not self.mode.get().startswith("USB"):
            return
        ports = [p.device for p in list_ports.comports()]
        self.cmb_port.configure(values=ports or [""])
        if ports and self.cmb_port.get() not in ports:
            self.cmb_port.set(ports[0])

    def on_mode(self, mode):
        if mode.startswith("USB"):
            self.lbl_port.configure(text="Puerto")
            self.cmb_baud.configure(state="normal")
            self.refresh_ports()
        else:
            self.lbl_port.configure(text="IP ESP32")
            self.cmb_baud.configure(state="disabled")
            self.cmb_port.configure(values=[self.last_ip])
            self.cmb_port.set(self.last_ip)

    def toggle_connection(self):
        if self.link.connected:
            self.disconnect("Desconectado por el usuario")
            return
        usb = self.mode.get().startswith("USB")
        target = self.cmb_port.get().strip()
        if not target:
            messagebox.showwarning("Conexión", "Seleccione un puerto o escriba la IP del ESP32.")
            return
        try:
            if usb:
                self.link.open_serial(target, int(self.cmb_baud.get()))
            else:
                self.last_ip = target
                self.link.open_tcp(target, TCP_PORT)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Conexión", f"No se pudo conectar:\n{exc}")
            return
        self.clear_data()
        self.synced = False
        self.last_rx = time.monotonic()
        self.btn_conn.configure(text="Desconectar", fg_color="#b91c1c", hover_color="#991b1b")
        self._set_led("#22c55e", "Conectado")
        self.card_state.set("Esperando datos…", "Sin telemetría aún", "#92400e")
        self.log("SYS · ", f"Conectado a {target}", True)

    def disconnect(self, reason="Desconectado"):
        self.link.close()
        with self.rx_q.mutex:
            self.rx_q.queue.clear()
        self.btn_conn.configure(text="Conectar", fg_color="#2563eb", hover_color="#1d4ed8")
        self._set_led("#ef4444", "Desconectado")
        self.card_state.set("Desconectado", "Sin enlace con el ESP32", "#475569")
        self.log("SYS · ", reason, True)

    def _set_led(self, color, text):
        self.led.configure(text_color=color)
        self.lbl_link.configure(text=text)

    def on_close(self):
        self.link.close()
        self.destroy()

    # ------------------------------------------------------ Envío de comandos
    def send(self, frame):
        if not self.link.connected:
            self.log("SYS · ", "No hay conexión: comando no enviado", True)
            return False
        try:
            self.link.send(frame)
            self.log("TX → ", frame, True)
            return True
        except Exception as exc:  # noqa: BLE001
            self.log("SYS · ", f"Error al enviar: {exc}", True)
            return False

    def _on_slider(self, v):
        self.lbl_sp_big.configure(text=f"{v:.1f} cm")
        self.ent_sp.delete(0, "end")
        self.ent_sp.insert(0, f"{v:.1f}")

    def send_sp(self):
        self.send(f"*SP,{round(self.sld.get(), 1):.2f}#")

    def _on_sp_entry(self, _evt=None):
        try:
            v = float(self.ent_sp.get().replace(",", "."))
        except ValueError:
            messagebox.showwarning("Referencia", "Ingrese un número válido.")
            return
        if not 0 <= v <= SP_MAX:
            messagebox.showwarning("Referencia", f"La referencia debe estar entre 0 y {SP_MAX:.0f} cm.")
            return
        self.sld.set(v)
        self.lbl_sp_big.configure(text=f"{v:.1f} cm")
        self.send(f"*SP,{v:.2f}#")

    def apply_pid(self):
        try:
            vals = [float(self.ent[n].get().replace(",", ".")) for n in ("Kp", "Ki", "Kd")]
        except ValueError:
            self.lbl_pid_status.configure(text="⚠ Valores inválidos", text_color="#f87171")
            return
        for n, v in zip(("Kp", "Ki", "Kd"), vals):
            if not 0 <= v <= PID_MAX[n]:
                self.lbl_pid_status.configure(text=f"⚠ {n} debe estar entre 0 y {PID_MAX[n]}",
                                              text_color="#f87171")
                return
        if self.send(f"*PID,{vals[0]:.2f},{vals[1]:.2f},{vals[2]:.2f}#"):
            self.pending_pid = tuple(round(v, 2) for v in vals)
            self.pending_t = time.monotonic()
            self.lbl_pid_status.configure(text="… enviando a la FPGA", text_color=MUTED)

    def _send_manual(self, _evt=None):
        cmd = self.ent_cmd.get().strip()
        if cmd:
            self.send(cmd)
            self.ent_cmd.delete(0, "end")

    # ------------------------------------------------------ Recepción
    def _poll_queue(self):
        try:
            for _ in range(200):
                kind, payload = self.rx_q.get_nowait()
                if kind == "line":
                    self._on_line(payload)
                elif kind == "error":
                    self.disconnect(f"Enlace perdido: {payload}")
        except queue.Empty:
            pass
        self.after(30, self._poll_queue)

    def _on_line(self, line):
        self.log("RX ← ", line)
        d = parse_telemetry(line)
        if d:
            self.handle_telemetry(d)
            return
        ack = parse_ack(line)
        if ack and ack[1] != 0:
            what, code = ack
            msg = MB_ERRORS.get(code, f"código {code}")
            if what == "PID":
                self.pending_pid = None
                self.lbl_pid_status.configure(text=f"⚠ La FPGA rechazó las ganancias: {msg}",
                                              text_color="#f87171")
            self.log("SYS · ", f"Comando {what} rechazado: {msg}", True)

    def handle_telemetry(self, d):
        now = time.monotonic()
        self.last_rx = now
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        pwm_pct = d["pwm"] / 255.0 * 100.0

        # Tarjetas y levitador
        self.card_pv.set(f"{d['pv']:.1f} cm", "Sensor ultrasónico")
        self.card_sp.set(f"{d['sp']:.1f} cm", "Consigna en la FPGA")
        self.card_err.set(f"{round(d['err'], 1) + 0.0:.1f} cm", "SP − PV")
        self.card_pwm.set(f"{pwm_pct:.0f} %", f"Duty {d['pwm']}/255")
        self.tube.set(d["pv"], d["sp"])

        # Estado del sistema
        self.err_hist.append(d["err"])
        self._update_state(d)

        # Primera trama: sincroniza controles con lo que realmente tiene la FPGA
        if not self.synced:
            self.synced = True
            self.sld.set(min(max(d["sp"], 0), SP_MAX))
            self._on_slider(self.sld.get())
            for name, key in (("Kp", "kp"), ("Ki", "ki"), ("Kd", "kd")):
                self.ent[name].delete(0, "end")
                self.ent[name].insert(0, f"{d[key]:.2f}")

        # Confirmación de ganancias (lectura de vuelta desde la FPGA)
        self.lbl_pid_fpga.configure(text=f"FPGA → Kp {d['kp']:.2f} · Ki {d['ki']:.2f} · Kd {d['kd']:.2f}")
        if self.pending_pid:
            if all(abs(a - b) < 0.006 for a, b in zip(self.pending_pid, (d["kp"], d["ki"], d["kd"]))):
                self.lbl_pid_status.configure(text="✔ Parámetros aplicados en la FPGA", text_color="#4ade80")
                self.pending_pid = None
            elif now - self.pending_t > 3.0:
                self.lbl_pid_status.configure(text="⚠ La FPGA no confirmó los valores", text_color="#fbbf24")
                self.pending_pid = None

        # Datos para gráfica / histórico / CSV
        self.t_d.append(t)
        self.pv_d.append(d["pv"])
        self.sp_d.append(d["sp"])
        self.pwm_d.append(pwm_pct)
        self.plot_dirty = True
        hora = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.records.append((hora, round(t, 3), d["sp"], d["pv"], d["err"], round(pwm_pct, 1)))
        if self.sample_n % HIST_EVERY == 0:
            self.tree.insert("", 0, values=(hora[:8], f"{d['sp']:.1f}", f"{d['pv']:.1f}",
                                            f"{round(d['err'], 1) + 0.0:.1f}", f"{pwm_pct:.0f}"))
            kids = self.tree.get_children()
            if len(kids) > MAX_TREE:
                self.tree.delete(kids[-1])
        self.sample_n += 1

    def _update_state(self, d):
        st = d.get("st", 0)
        if d["mb"] != 0:
            self.card_state.set("Falla\nModbus", MB_ERRORS.get(d["mb"], f"Código {d['mb']}"), "#b91c1c")
        elif st & ST_TIMEOUT:
            self.card_state.set("Falla\nArduino", "La FPGA no recibe mediciones", "#b91c1c")
        elif st & ST_SENSOR:
            self.card_state.set("Falla\nsensor", "Ultrasónico sin eco: ventilador apagado", "#b91c1c")
        elif not d["en"]:
            self.card_state.set("Control\ndeshabilitado", "Lazo abierto", "#475569")
        else:
            full = len(self.err_hist) == self.err_hist.maxlen
            stable = full and all(abs(e) <= STABLE_BAND for e in self.err_hist)
            sat = " · PWM saturado" if st & ST_SAT_HI else ""
            self.card_state.set("Control\nhabilitado",
                                "Lazo cerrado · " + ("estable" if stable else "ajustando") + sat, C_STATE)

    def _watchdog(self):
        if self.link.connected and time.monotonic() - self.last_rx > NO_DATA_TIMEOUT:
            self._set_led("#f59e0b", "Sin datos")
            self.card_state.set("Sin datos", "Revise ESP32 / FPGA", "#92400e")
        elif self.link.connected:
            self._set_led("#22c55e", "Conectado")
        self.after(500, self._watchdog)

    # ------------------------------------------------------ Gráfica y datos
    def _on_window(self, v):
        self.win_s = int(v.split()[0])
        self.plot_dirty = True

    def _refresh_plot(self):
        if self.plot_dirty:
            self.plot_dirty = False
            t = list(self.t_d)
            self.l_pv.set_data(t, list(self.pv_d))
            self.l_sp.set_data(t, list(self.sp_d))
            self.l_pwm.set_data(t, list(self.pwm_d))
            tmax = t[-1] if t else 0
            self.ax.set_xlim(max(0, tmax - self.win_s), max(self.win_s, tmax))
            self.canvas.draw_idle()
        self.after(100, self._refresh_plot)

    def clear_data(self):
        for d in (self.t_d, self.pv_d, self.sp_d, self.pwm_d, self.err_hist):
            d.clear()
        self.records.clear()
        self.tree.delete(*self.tree.get_children())
        self.t0 = None
        self.sample_n = 0
        self.plot_dirty = True

    def export_csv(self):
        if not self.records:
            messagebox.showinfo("Exportar", "Todavía no hay datos para exportar.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"levitador_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["hora", "t_s", "SP_cm", "PV_cm", "error_cm", "PWM_pct"])
            w.writerows(self.records)
        self.log("SYS · ", f"CSV guardado: {path}", True)

    # ------------------------------------------------------ Monitor de datos
    def log(self, tag, text, force=False):
        if self.var_pause.get() and not force:
            return
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt.configure(state="normal")
        self.txt.insert("end", f"[{ts}] {tag}{text}\n")
        if int(self.txt.index("end-1c").split(".")[0]) > MAX_LOG:
            self.txt.delete("1.0", "51.0")
        self.txt.configure(state="disabled")
        self.txt.see("end")

    def _clear_log(self):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")


if __name__ == "__main__":
    App().mainloop()
