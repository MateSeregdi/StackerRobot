import threading
import tkinter as tk
from tkinter import ttk

import api
from state import AppState

# ── Bootstrap ─────────────────────────────────────────────────────────────────
root = tk.Tk()
root.title("Stacker Robot")
root.geometry("960x740")

state = AppState()
state.start_drain(root)

# ── Status bar ────────────────────────────────────────────────────────────────
tk.Label(root, textvariable=state.status,
         relief=tk.SUNKEN, anchor="w", bg="#e8e8e8"
         ).pack(side=tk.BOTTOM, fill=tk.X, padx=2, pady=2)

# ── Notebook ──────────────────────────────────────────────────────────────────
notebook = ttk.Notebook(root)
notebook.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

manual_frame = tk.Frame(notebook)
auto_frame = tk.Frame(notebook)
notebook.add(manual_frame, text="Manual")
notebook.add(auto_frame,   text="Automatic")


# ═══════════════════════════ MANUAL TAB ══════════════════════════════════════

def _thread(fn, *args):
    threading.Thread(target=fn, args=args, daemon=True).start()


# ── Load Cell ─────────────────────────────────────────────────────────────────
lc_frame = tk.LabelFrame(manual_frame, text="Load Cell", padx=6, pady=6)
lc_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

tk.Label(lc_frame, text="Reading:", font=("Arial", 9, "bold"), fg="#555").pack()
tk.Label(lc_frame, textvariable=state.lc_reading,
         font=("Arial", 11, "bold")).pack(pady=(0, 6))

tk.Label(lc_frame, text="Cal. Factor:", font=("Arial", 9, "bold"), fg="#555").pack()
tk.Label(lc_frame, textvariable=state.cal_factor,
         font=("Courier", 10)).pack(pady=(0, 4))

_cf_entry = tk.Entry(lc_frame, width=16)
_cf_entry.pack(pady=(0, 2))
tk.Button(lc_frame, text="Set Cal. Factor", width=18,
          command=lambda: _thread(api.set_calibration_factor, _cf_entry.get(), state)
          ).pack(pady=(2, 8))

tk.Button(lc_frame, text="Tare", width=18,
          command=lambda: _thread(api.tare_load_cell, state)
          ).pack(pady=2)


# ── Stepper motor panels ──────────────────────────────────────────────────────
def stepper_section(parent, name, display, row, col):
    frame = tk.LabelFrame(parent, text=display, padx=6, pady=6)
    frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

    mode = tk.StringVar(value="speed")

    if (name == "lead_screw"):
        tk.Button(frame, text="Home Lead Screw",
                  command=lambda: _thread(api.home_lead_screw, state)).pack(pady=(2, 2))

    rb_row = tk.Frame(frame)
    rb_row.pack(fill=tk.X)
    tk.Radiobutton(rb_row, text="Speed Control",    variable=mode, value="speed",
                   command=lambda: switch()).pack(side=tk.LEFT)
    tk.Radiobutton(rb_row, text="Position Control", variable=mode, value="position",
                   command=lambda: switch()).pack(side=tk.LEFT)

    # Speed panel
    spd_panel = tk.Frame(frame)
    spd_var = tk.IntVar(value=0)
    tk.Label(spd_panel, textvariable=state.motor_speed[name],
             font=("Arial", 11, "bold"), width=18).pack(pady=(4, 0))
    tk.Scale(
        spd_panel, from_=-300, to=300, orient=tk.HORIZONTAL,
        variable=spd_var, length=250,
    ).pack()
    tk.Button(
        spd_panel, text="Apply Speed",
        command=lambda: _thread(api.set_motor_speed,
                                name, spd_var.get(), state),
    ).pack(pady=(5, 2))

    def stop():
        _thread(api.set_motor_speed, name, 0, state)
        spd_var.set(0)
        state.motor_speed[name].set("0")

    tk.Button(
        spd_panel, text="Stop",
        command=stop
    ).pack(pady=(5, 2))

    # Position panel
    pos_panel = tk.Frame(frame)
    tk.Label(pos_panel, text="Current:", font=(
        "Arial", 9, "bold"), fg="#555").pack(pady=(4, 0))
    tk.Label(pos_panel, textvariable=state.motor_position[name],
             font=("Arial", 10, "bold")).pack(pady=(0, 6))

    inp_row = tk.Frame(pos_panel)
    inp_row.pack()
    tk.Label(inp_row, text="Target step:").pack(side=tk.LEFT)
    tgt_entry = tk.Entry(inp_row, width=10)
    tgt_entry.pack(side=tk.LEFT, padx=4)

    def move():
        raw = tgt_entry.get().strip()
        try:
            pos = int(raw)
        except ValueError:
            state.update(status="Enter a valid integer step position")
            return
        _thread(api.move_motor_to, name, pos, state)

    btn_row = tk.Frame(pos_panel)
    btn_row.pack(pady=5)
    tk.Button(btn_row, text="Move",             command=move).pack(
        side=tk.LEFT, padx=4)

    def switch():
        if mode.get() == "speed":
            pos_panel.pack_forget()
            spd_panel.pack(fill=tk.X)
        else:
            spd_panel.pack_forget()
            pos_panel.pack(fill=tk.X)

    spd_panel.pack(fill=tk.X)


# ── Fork Belt (DC) ────────────────────────────────────────────────────────────
def dc_section(parent, row, col):
    frame = tk.LabelFrame(parent, text="Fork Belt", padx=6, pady=6)
    frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

    spd_var = tk.IntVar(value=0)
    dir_var = tk.StringVar(value="forward")

    tk.Label(frame, textvariable=state.fork_belt_speed,
             font=("Arial", 10, "bold")).pack(pady=(4, 0))
    tk.Scale(
        frame, from_=0, to=255, orient=tk.HORIZONTAL,
        variable=spd_var, length=250,
    ).pack()

    dir_row = tk.Frame(frame)
    dir_row.pack(pady=6)
    tk.Label(dir_row, text="Direction:").pack(side=tk.LEFT, padx=(0, 8))
    tk.Radiobutton(dir_row, text="Forward",  variable=dir_var,
                   value="backward").pack(side=tk.LEFT)
    tk.Radiobutton(dir_row, text="Backward", variable=dir_var,
                   value="forward").pack(side=tk.LEFT)

    tk.Button(
        frame, text="Apply", width=14,
        command=lambda: _thread(
            api.set_fork_belt, spd_var.get(), dir_var.get(), state),
    ).pack(pady=(2, 4))


stepper_section(manual_frame, "main_belt",    "Main Belt",    row=0, col=1)
stepper_section(manual_frame, "lead_screw",   "Lead Screw",   row=1, col=0)
stepper_section(manual_frame, "fork_forward", "Fork Forward", row=1, col=1)
dc_section(manual_frame, row=2, col=0)

for _r in range(3):
    manual_frame.rowconfigure(_r, weight=1)
for _c in range(3):
    manual_frame.columnconfigure(_c, weight=1)

# Optical Sensor
os_frame = tk.LabelFrame(manual_frame, text="Optical Sensor", padx=6, pady=6)
os_frame.grid(row=2, column=1, padx=5, pady=5, sticky="nsew")
tk.Label(os_frame, text="Distance (cm)", font=("Arial", 9, "bold"),
         fg="#555").pack(pady=4)
tk.Label(os_frame, textvariable=state.optical_sensor, font=("Courier", 13),
         anchor="e").pack()
tk.Button(os_frame, text="Start Readings", width=14,
          command=lambda: _thread(api.enable_optical_sensor_measuring, state)
          ).pack(pady=4)
tk.Button(os_frame, text="Stop Readings", width=14, command=lambda: _thread(
    api.disable_optical_sensor_measuring, state)
).pack(pady=4)


# ═══════════════════════════ TOF SHARED HELPERS ═══════════════════════════════

_TOF_GRID = 16
_TOF_CELL = 20
_COLOR_OCCUPIED = "#e74c3c"
_COLOR_FREE = "#2ecc71"
_COLOR_UNKNOWN = "#555555"


def _make_tof_canvas(parent):
    """Build a 16x16 canvas wired to state.tof_occupancy. Returns the canvas."""
    size = _TOF_GRID * _TOF_CELL
    canvas = tk.Canvas(parent, width=size, height=size,
                       bg="#222", highlightthickness=0)
    canvas.pack(padx=4, pady=4)

    rects = [
        canvas.create_rectangle(
            x * _TOF_CELL, y * _TOF_CELL,
            (x + 1) * _TOF_CELL, (y + 1) * _TOF_CELL,
            fill=_COLOR_UNKNOWN, outline="#111", width=1,
        )
        for y in range(_TOF_GRID) for x in range(_TOF_GRID)
    ]

    def _redraw(*_):
        data = state.tof_occupancy.get()
        for i, rect in enumerate(rects):
            if i >= len(data):
                canvas.itemconfig(rect, fill=_COLOR_UNKNOWN)
            elif data[i] == "1":
                canvas.itemconfig(rect, fill=_COLOR_OCCUPIED)
            else:
                canvas.itemconfig(rect, fill=_COLOR_FREE)

    state.tof_occupancy.trace_add("write", _redraw)
    return canvas


# ── Manual tab: ToF full controls ────────────────────────────────────────────
_tof_manual = tk.LabelFrame(manual_frame, text="ToF Sensor", padx=6, pady=6)
_tof_manual.grid(row=0, column=2, rowspan=3, padx=5, pady=5, sticky="n")

_ena_row = tk.Frame(_tof_manual)
_ena_row.pack(pady=(0, 2))
tk.Button(_ena_row, text="Enable",  width=9,
          command=lambda: _thread(api.enable_tof,  state)).pack(side=tk.LEFT, padx=2)
tk.Button(_ena_row, text="Disable", width=9,
          command=lambda: _thread(api.disable_tof, state)).pack(side=tk.LEFT, padx=2)

tk.Button(_tof_manual, text="Calibrate Floor", width=20,
          command=lambda: _thread(api.calibrate_tof, state)).pack(pady=2)
tk.Button(_tof_manual, text="Read Data",       width=20,
          command=lambda: _thread(api.read_tof_data, state)).pack(pady=(2, 6))

_make_tof_canvas(_tof_manual)


# ═══════════════════════════ AUTOMATIC TAB ════════════════════════════════════

def _auto_card(parent, title, row, col):
    f = tk.LabelFrame(parent, text=title, padx=10, pady=10)
    f.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
    return f


def _stat_row(card, label_text, var, row, big=False):
    font_val = ("Courier", 18, "bold") if big else ("Courier", 13)
    pad = (0 if row == 0 else 8, 0)
    tk.Label(card, text=label_text, font=("Arial", 9, "bold"),
             fg="#555").grid(row=row, column=0, sticky="w", pady=pad)
    tk.Label(card, textvariable=var, font=font_val,
             anchor="e").grid(row=row, column=1, sticky="e", padx=(16, 0), pady=pad)


# ── Auto mode control ─────────────────────────────────────────────────────────
def handleAutoEnable():
    if state.lead_screw_homed.get() == "true":
        _thread(api.enable_auto_mode, state)
    else:
        state.update(
            status="Lead screw is not homed, can't start automatic mode")


_auto_ctrl = tk.Frame(auto_frame)
_auto_ctrl.grid(row=0, column=0, columnspan=3,
                padx=5, pady=(4, 2), sticky="ew")
_enable_color = "#2ecc71" if state.lead_screw_homed == "true" else "#646565"

tk.Button(
    _auto_ctrl, text="Enable Auto Mode", width=20,
    bg=_enable_color, activebackground="#27ae60",
    command=handleAutoEnable

).pack(side=tk.LEFT, padx=8)
tk.Button(
    _auto_ctrl, text="Disable Auto Mode", width=20,
    bg="#e44937", activebackground="#c0392b", fg="white",
    command=lambda: _thread(api.disable_auto_mode, state),
).pack(side=tk.LEFT, padx=4)


for _name, _display, _r, _c in [
    ("main_belt",    "Main Belt",    1, 0),
    ("lead_screw",   "Lead Screw",   1, 1),
    ("fork_forward", "Fork Forward", 2, 0),
]:
    _card = _auto_card(auto_frame, _display, _r, _c)

    if (_name == "lead_screw"):
        _homed_indicator = tk.Label(_card, text="● Not Homed",
                                    fg="#e74c3c", font=("Arial", 10, "bold"))
        _homed_indicator.grid(row=0)

        def _refresh_homed_indicator(*_):
            if state.lead_screw_homed.get() == "true":
                _homed_indicator.config(text="● Homed",     fg="#2ecc71")
            else:
                _homed_indicator.config(text="● Not Homed", fg="#e74c3c")
        state.lead_screw_homed.trace_add("write", _refresh_homed_indicator)

    _stat_row(_card, "Position (steps)",
              state.motor_position[_name], row=1, big=True)
    _stat_row(_card, "Speed",            state.motor_speed[_name],    row=2)


_fb = _auto_card(auto_frame, "Fork Belt (DC)", row=2, col=1)
_stat_row(_fb, "Speed",     state.fork_belt_speed, row=0, big=True)
_stat_row(_fb, "Direction", state.fork_belt_dir,   row=1)

_lc = _auto_card(auto_frame, "Load Cell", row=3, col=0)
_stat_row(_lc, "Reading",     state.lc_reading, row=0, big=True)
_stat_row(_lc, "Cal. Factor", state.cal_factor, row=1)

for _r in range(4):
    auto_frame.rowconfigure(_r, weight=1)
for _c in range(3):
    auto_frame.columnconfigure(_c, weight=1)

# ── Automatic tab: ToF display-only ──────────────────────────────────────────
_tof_auto = tk.LabelFrame(auto_frame, text="ToF Occupancy", padx=6, pady=6)
_tof_auto.grid(row=1, column=2, rowspan=3, padx=5, pady=5, sticky="n")

_cal_indicator = tk.Label(_tof_auto, text="● Not calibrated",
                          fg="#e74c3c", font=("Arial", 10, "bold"))
_cal_indicator.pack(pady=(0, 4))


def _refresh_cal_indicator(*_):
    if state.tof_calibrated.get() == "true":
        _cal_indicator.config(text="● Calibrated",     fg="#2ecc71")
    else:
        _cal_indicator.config(text="● Not calibrated", fg="#e74c3c")


state.tof_calibrated.trace_add("write", _refresh_cal_indicator)
bw_row = tk.Frame(_tof_auto)
bw_row.pack()
tk.Label(bw_row, text="Approximate Box Width (mm) ").pack(side=tk.LEFT)
tk.Label(bw_row, textvariable=state.box_width).pack(side=tk.RIGHT)
_make_tof_canvas(_tof_auto)

# OPTICAL SENSOR
os_card = _auto_card(auto_frame, "Optical Sensor", 3, 1)
_stat_row(os_card, "Distance ", state.optical_sensor, 0)


# ═══════════════════════════ CONNECTIONS ══════════════════════════════════════

threading.Thread(target=api.start_websocket,
                 args=(state,), daemon=True).start()

threading.Thread(target=api.get_calibration_factor,
                 args=(state,), daemon=True).start()

root.mainloop()
