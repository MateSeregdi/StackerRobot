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


def calibrate_load_cell(action: str, state) -> None:
    resp = api_get(state, f"/load_cell/calibrate",
                   {"action": action})
    if resp is not None and resp.ok:
        if action == "increase" or action == "decrease":
            s = resp.text.split(":")
            calibration_factor = s[0]
            reading = s[1]
            state.update(lc_reading=reading, cal_factor=calibration_factor)
        elif action == "start" or action == "stop":
            state.update(status=f"{resp.text}")
        else:
            state.update(status="Invalid action in load cell calibration")


def read_load_cell(state) -> None:
    resp = api_get(state, f"/load_cell/read")
    if resp is not None and resp.ok:
        state.update(lc_reading=resp.text)


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
    resp = api_get(state, f"motor/fork_belt/set_speed",
                   {"speed": speed, "direction": direction})
    if resp is not None and resp.ok:
        state.update(fork_belt_speed=speed,
                     fork_belt_dir=direction, status=resp.text)
    pass


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
            state.update(optical_sensor=arr[1])
        case _:
            state.update(status="Invalid WebSocket message")


# websocket format:
# "motor/<motor_name>/speed:<speed>,position:<position>"
# "load_cell/calibration_factor/<calibration_factor>"
# "load_cell/reading/<reading>"
# "optical_sensor/<reading>"
