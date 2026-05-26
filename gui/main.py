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

tk.Label(lc_frame, text="Reading:", font=(
    "Arial", 9, "bold"), fg="#555").pack()
tk.Label(lc_frame, textvariable=state.lc_reading,
         font=("Arial", 11, "bold")).pack(pady=(0, 6))

for _label, _action in [
    ("Start Calibration", "start"),
    ("Factor +",          "increase"),
    ("Factor −",          "decrease"),
    ("Stop Calibration",  "stop"),
]:
    tk.Button(
        lc_frame, text=_label, width=18,
        command=lambda a=_action: _thread(api.calibrate_load_cell, a, state),
    ).pack(pady=1)

tk.Button(
    lc_frame, text="Read", width=18,
    command=lambda: _thread(api.read_load_cell, state),
).pack(pady=(6, 2))


# ── Stepper motor panels ──────────────────────────────────────────────────────
def stepper_section(parent, name, display, row, col):
    frame = tk.LabelFrame(parent, text=display, padx=6, pady=6)
    frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")

    mode = tk.StringVar(value="speed")

    rb_row = tk.Frame(frame)
    rb_row.pack(fill=tk.X)
    tk.Radiobutton(rb_row, text="Speed Control",    variable=mode, value="speed",
                   command=lambda: switch()).pack(side=tk.LEFT)
    tk.Radiobutton(rb_row, text="Position Control", variable=mode, value="position",
                   command=lambda: switch()).pack(side=tk.LEFT)

    # Speed panel
    spd_panel = tk.Frame(frame)
    spd_var = tk.IntVar(value=0)
    spd_lbl = tk.Label(spd_panel, text="0  steps / sec", width=18)
    spd_lbl.pack(pady=(4, 0))
    tk.Scale(
        spd_panel, from_=-300, to=300, orient=tk.HORIZONTAL,
        variable=spd_var, length=250,
        command=lambda v: spd_lbl.config(text=f"{v}  steps / sec"),
    ).pack()
    tk.Button(
        spd_panel, text="Apply Speed",
        command=lambda: _thread(api.set_motor_speed,
                                name, spd_var.get(), state),
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

    spd_lbl = tk.Label(frame, text="Speed: 0 / 255",
                       font=("Arial", 10, "bold"))
    spd_lbl.pack(pady=(4, 0))
    tk.Scale(
        frame, from_=0, to=255, orient=tk.HORIZONTAL,
        variable=spd_var, length=250,
        command=lambda v: spd_lbl.config(text=f"Speed: {v} / 255"),
    ).pack()

    dir_row = tk.Frame(frame)
    dir_row.pack(pady=6)
    tk.Label(dir_row, text="Direction:").pack(side=tk.LEFT, padx=(0, 8))
    tk.Radiobutton(dir_row, text="Forward",  variable=dir_var,
                   value="forward").pack(side=tk.LEFT)
    tk.Radiobutton(dir_row, text="Backward", variable=dir_var,
                   value="backward").pack(side=tk.LEFT)

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
for _c in range(2):
    manual_frame.columnconfigure(_c, weight=1)


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


for _name, _display, _r, _c in [
    ("main_belt",    "Main Belt",    0, 0),
    ("lead_screw",   "Lead Screw",   0, 1),
    ("fork_forward", "Fork Forward", 1, 0),
]:
    _card = _auto_card(auto_frame, _display, _r, _c)
    _stat_row(_card, "Position (steps)",
              state.motor_position[_name], row=0, big=True)
    _stat_row(_card, "Speed",            state.motor_speed[_name],    row=1)

_fb = _auto_card(auto_frame, "Fork Belt (DC)", row=1, col=1)
_stat_row(_fb, "Speed",     state.fork_belt_speed, row=0, big=True)
_stat_row(_fb, "Direction", state.fork_belt_dir,   row=1)

_lc = _auto_card(auto_frame, "Load Cell", row=2, col=0)
_stat_row(_lc, "Reading",     state.lc_reading, row=0, big=True)
_stat_row(_lc, "Cal. Factor", state.cal_factor, row=1)

for _r in range(3):
    auto_frame.rowconfigure(_r, weight=1)
for _c in range(2):
    auto_frame.columnconfigure(_c, weight=1)


# ═══════════════════════════ CONNECTIONS ══════════════════════════════════════

threading.Thread(target=api.start_websocket,
                 args=(state,), daemon=True).start()


def _auto_poll_lc():
    if notebook.index(notebook.select()) == 1:
        _thread(api.read_load_cell, state)
    root.after(5000, _auto_poll_lc)


root.after(5000, _auto_poll_lc)

root.mainloop()
