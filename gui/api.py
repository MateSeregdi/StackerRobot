"""
Network / hardware action stubs.

Implement each function body. Use state.update(...) to push results back
to the UI from any thread — it is thread-safe.
All blocking I/O must run off the main thread (caller wraps in threading.Thread).
"""

import requests
import websocket
import time

# ── Connection settings ───────────────────────────────────────────────────────
ESP32_IP = "172.20.10.2"
BASE_URL = f"http://{ESP32_IP}"
WS_URL = f"ws://{ESP32_IP}:81"


def api_get(state, path, params=None):
    try:
        resp = requests.get(f"{BASE_URL}{path}", params=params, timeout=2.0)
        state.update(status=f"OK {path}")
        return resp
    except requests.exceptions.RequestException as e:
        state.update(status=f"Error: {e}")
        return None


def get_calibration_factor(state) -> None:
    resp = api_get(state, "/load_cell/calibration_factor")
    if resp is not None and resp.ok:
        state.update(cal_factor=resp.text)


def set_calibration_factor(value: str, state) -> None:
    try:
        float(value)
    except ValueError:
        state.update(status="Invalid calibration factor — enter a number")
        return
    resp = api_get(state, "/load_cell/set_calibration_factor", {"value": value})
    if resp is not None and resp.ok:
        state.update(cal_factor=resp.text, status="Calibration factor updated")


def tare_load_cell(state) -> None:
    resp = api_get(state, "/load_cell/tare")
    if resp is not None and resp.ok:
        state.update(status="Tared")


def set_motor_speed(name: str, steps_per_second: int, state) -> None:
    resp = api_get(state, f"/motor/{name}/set_speed",
                   {"steps_per_second": steps_per_second})
    if resp is not None and resp.ok:
        state.update(**{f"motor_speed__{name}": f"{steps_per_second:+d} sps"})


def move_motor_to(name: str, position: int, state) -> None:
    # GET /motor/<name>/move_to?position=<position>
    # On error → state.update(status=f"Move error: {e}")
    resp = api_get(state, f"/motor/{name}/move_to", {"position": position})
    if resp is not None and resp.ok:
        state.update(status=resp.text)


def set_fork_belt(speed: int, direction: str, state) -> None:
    # GET /motor/fork_belt/set_speed?speed=<speed>&direction=<direction>
    # On success → state.update(fork_belt_speed=f"{speed} / 255", fork_belt_dir=direction)
    # On error   → state.update(status=f"Fork belt error: {e}")
    resp = api_get(state, f"/motor/fork_belt/set_speed",
                   {"speed": speed, "direction": direction})
    if resp is not None and resp.ok:
        state.update(fork_belt_speed=speed,
                     fork_belt_dir=direction, status=resp.text)
    pass


def enable_tof(state) -> None:
    resp = api_get(state, "/tof/enable")
    if resp is not None and resp.ok:
        state.update(status="ToF sensor enabled")


def disable_tof(state) -> None:
    resp = api_get(state, "/tof/disable")
    if resp is not None and resp.ok:
        state.update(status="ToF sensor disabled")


def calibrate_tof(state) -> None:
    resp = api_get(state, "/tof/calibrate")
    if resp is not None and resp.ok:
        state.update(status="ToF calibration triggered")


def read_tof_data(state) -> None:
    import json
    resp = api_get(state, "/tof/data")
    if resp is None or not resp.ok:
        return
    data = json.loads(resp.text)
    state.update(tof_calibrated=str(data["calibrated"]).lower())
    occ = "".join(str(v) for row in data["occupancy"] for v in row)
    state.update(tof_occupancy=occ)


def enable_optical_sensor_measuring(state) -> None:
    resp = api_get(state, "/optical_sensor/enable")
    if resp is not None and resp.ok:
        state.update(status="Optical sensor readings started")


def disable_optical_sensor_measuring(state) -> None:
    resp = api_get(state, "/optical_sensor/disable")
    if resp is not None and resp.ok:
        state.update(status="Optical sensor readings stopped")


def enable_auto_mode(state) -> None:
    resp = api_get(state, "/auto/enable")
    if resp is not None and resp.ok:
        state.update(status="Automatic mode enabled")


def disable_auto_mode(state) -> None:
    resp = api_get(state, "/auto/disable")
    if resp is not None and resp.ok:
        state.update(status="Automatic mode disabled")


def home_lead_screw(state) -> None:
    resp = api_get(state, "/lead_screw/home")
    if resp is not None and resp.ok:
        state.update(status="Lead screw is homing", lead_screw_homed="false")


def start_websocket(state) -> None:
    while True:
        try:
            websocket.WebSocketApp(
                WS_URL,
                on_open=lambda ws: _ws_on_open(state),
                on_message=lambda ws, msg: _ws_on_message(state, msg),
                on_error=lambda ws, err: _ws_on_error(state, err),
                on_close=lambda ws, code, msg: _ws_on_close(state),
            ).run_forever()
        except Exception as e:
            print(f"[ws] exception: {type(e).__name__}: {e}")
            state.update(
                status=f"WebSocket exception: {type(e).__name__}: {e} — reconnecting in 5s")
        time.sleep(5)


def _ws_on_open(state):
    state.update(status="WebSocket Connected")


def _ws_on_close(state):
    state.update(status="WebSocket disconnected, reconnecting...")


def _ws_on_error(state, error):
    state.update(status=f"WebSocket error: {error}")


def _ws_on_message(state, message):
    if isinstance(message, bytes):
        message = message.decode("utf-8")
    message = message.strip()
    arr = message.split("/")
    match arr[0]:
        case "motor":
            motor_name = arr[1]
            dat = arr[2].split(",")
            speed = dat[0].split(":")[1]
            position = dat[1].split(":")[1]
            state.update(**{f"motor_position__{motor_name}": position})
            state.update(**{f"motor_speed__{motor_name}": speed})
        case "load_cell":
            match arr[1]:
                case "calibration_factor":
                    state.update(cal_factor=arr[2])
                case "reading":
                    state.update(lc_reading=arr[2])
                case _:
                    state.update(status="Invalid load cell message")
        case "optical_sensor":
            if (arr[1] == "-1.00"):
                state.update(optical_sensor="Out of range")
            else:
                state.update(optical_sensor=arr[1])
        case "tof":
            parts = arr[1].split(":", 1)
            if parts[0] == "occupancy" and len(parts) == 2:
                state.update(tof_occupancy=parts[1])
        case "lead_screw_homed":
            state.update(lead_screw_homed="true")
        case _:
            state.update(status="Invalid WebSocket message")


# websocket format:
# "motor/<motor_name>/speed:<speed>,position:<position>"
# "load_cell/calibration_factor/<calibration_factor>"
# "load_cell/reading/<reading>"
# "optical_sensor/<reading>"
