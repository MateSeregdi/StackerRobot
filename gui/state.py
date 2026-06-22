import tkinter as tk
import queue as _queue


class AppState:
    """
    Single source of truth for all UI state.

    Thread-safe: call state.update(**kwargs) from any thread.
    The main thread drains the queue every 50 ms via start_drain().

    Key conventions for update():
        Flat key  → maps to a StringVar attribute:
            state.update(status="OK", lc_reading="1.23 kg")

        Double-underscore → indexes into a dict of StringVars:
            state.update(motor_position__main_belt="512")
            state.update(motor_speed__lead_screw="+120 sps")
    """

    def __init__(self):
        self._q = _queue.Queue()

        self.status = tk.StringVar(value="Ready")
        self.lc_reading = tk.StringVar(value="—")
        self.cal_factor = tk.StringVar(value="—")
        self.optical_sensor = tk.StringVar(value="—")

        self.motor_position = {
            "main_belt":    tk.StringVar(value="—"),
            "lead_screw":   tk.StringVar(value="—"),
            "fork_forward": tk.StringVar(value="—"),
        }
        self.motor_speed = {
            "main_belt":    tk.StringVar(value="—"),
            "lead_screw":   tk.StringVar(value="—"),
            "fork_forward": tk.StringVar(value="—"),
        }
        self.lead_screw_homed = tk.StringVar(value="false")

        self.fork_belt_speed = tk.StringVar(value="—")
        self.fork_belt_dir = tk.StringVar(value="—")

        self.tof_occupancy = tk.StringVar(
            value="")   # 256-char string of "0"/"1"
        self.tof_calibrated = tk.StringVar(value="false")

        self.box_width = tk.StringVar(value="—")

    def update(self, **kwargs) -> None:
        """Post one or more state changes from any thread."""
        self._q.put(kwargs)

    def start_drain(self, root: tk.Tk) -> None:
        """Attach the update pump to the Tk event loop. Call once after root exists."""
        def _tick():
            try:
                while True:
                    self._apply(**self._q.get_nowait())
            except _queue.Empty:
                pass
            except Exception as e:
                print(f"[state] _apply error: {e}")
            finally:
                root.after(50, _tick)
        root.after(50, _tick)

    def _apply(self, **kwargs) -> None:
        for key, value in kwargs.items():
            if "__" in key:
                attr, sub = key.split("__", 1)
                getattr(self, attr)[sub].set(str(value))
            else:
                getattr(self, key).set(str(value))
