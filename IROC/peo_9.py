from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import threading
import sys
from pymavlink import mavutil
from picamera2 import Picamera2
import requests

sys.stdout.reconfigure(line_buffering=True)

TARGET_ALTITUDE = 2
HOVER_DURATION = 10
CONNECTION_STRING = '/dev/ttyACM0'


RC_NEUTRAL_HOVER = 1500
RC_CLIMB_PWM     = 1700
RC_DEADZONE      = 0.15 

FAILSAFE_TIMEOUT     = 4 
WATCHDOG_POLL_INTERVAL = 1.0

BATTERY_LOW_VOLTAGE  = 15.4
BATTERY_POLL_INTERVAL = 2.0


_last_heartbeat_time   = None
_last_heartbeat_lock   = threading.Lock()
_failsafe_triggered    = threading.Event()
_watchdog_stop         = threading.Event()
_battery_watchdog_stop = threading.Event()
_pilot_takeover        = threading.Event()


RC_TAKEOVER_NEUTRAL_PWM  = 1000
RC_TAKEOVER_THRESHOLD    = 120
RC_TAKEOVER_CONFIRM_COUNT = 3
_rc_takeover_counter = 0


def _read_rc_channel_pwm(vehicle, channel='3'):
    try:
        value = vehicle.channels.get(channel)
        if value is None:
            value = vehicle.channels.get(int(channel))
        if value is None:
            return None
        value = int(value)
        if 800 <= value <= 2200:
            return value
    except Exception:
        pass
    return None


def _clear_rc_overrides(vehicle):
    for ch in ('1', '2', '3', '4', '5', '6', '7', '8'):
        try:
            vehicle.channels.overrides[ch] = None
        except Exception:
            try:
                vehicle.channels.overrides[int(ch)] = None
            except Exception:
                pass


def _pilot_takeover_requested(vehicle):
    global _rc_takeover_counter
    if _pilot_takeover.is_set():
        return True
    pwm = _read_rc_channel_pwm(vehicle, '3')
    if pwm is None:
        _rc_takeover_counter = 0
        return False
    if abs(pwm - RC_TAKEOVER_NEUTRAL_PWM) >= RC_TAKEOVER_THRESHOLD:
        _rc_takeover_counter += 1
    else:
        _rc_takeover_counter = 0
    if _rc_takeover_counter >= RC_TAKEOVER_CONFIRM_COUNT:
        _pilot_takeover.set()
        _clear_rc_overrides(vehicle)
        try:
            # Hand back to FLOW_HOLD so the FC still holds position for the pilot
            vehicle.mode = VehicleMode("FLOWHOLD")
        except Exception:
            pass
        return True
    return False
# Deep's code end


def connect_to_drone():
    """
    GPS ke bina kaam karta hai — FLOW_HOLD needs optical flow, not GPS.
    """
    print("Drone se connect ho rahe hain...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    print("Drone se successfully connected!")
    return vehicle


def setup_ardupilot_gcs_failsafe(vehicle, timeout_sec=FAILSAFE_TIMEOUT):
    """
    LAYER 1 — Hardware-level GCS failsafe (ArduPilot side).

    Parameters set:
      FS_GCS_ENABLE  = 1  -> enable GCS failsafe, action = LAND
      FS_GCS_TIMEOUT      -> seconds of silence before failsafe fires
    """
    print("Setting up ArduPilot GCS failsafe parameters (Layer 1)...")
    try:
        vehicle.parameters['FS_GCS_ENABLE']  = 1
        vehicle.parameters['FS_GCS_TIMEOUT'] = timeout_sec
        print(f"   FS_GCS_ENABLE  = 1 (Land on GCS loss)")
        print(f"   FS_GCS_TIMEOUT = {timeout_sec}s")
        print("ArduPilot GCS failsafe configured!")
    except Exception as e:
        print(f"WARNING: Could not set GCS failsafe parameters: {e}")
        print("Firmware may not support FS_GCS_ENABLE. Layer 2 (Python watchdog) still active.")


def setup_ardupilot_battery_failsafe(vehicle, low_voltage=BATTERY_LOW_VOLTAGE):
    """
    LAYER 1 — Hardware-level battery failsafe (ArduPilot side).

    Parameters set:
      FS_BATT_ENABLE  = 1    -> enable battery failsafe
      FS_BATT_VOLTAGE = 15.4 -> trigger voltage (V) for a 4S LiPo
      FS_BATT_MAH     = 0    -> disable mAh-based failsafe (voltage only)
    """
    print("Setting up ArduPilot battery failsafe parameters (Layer 1)...")
    try:
        vehicle.parameters['FS_BATT_ENABLE']  = 1
        vehicle.parameters['FS_BATT_VOLTAGE'] = low_voltage
        vehicle.parameters['FS_BATT_MAH']     = 0
        print(f"   FS_BATT_ENABLE  = 1 (Land on low battery)")
        print(f"   FS_BATT_VOLTAGE = {low_voltage:.1f} V")
        print(f"   FS_BATT_MAH     = 0 (disabled)")
        print("ArduPilot battery failsafe configured!")
    except Exception as e:
        print(f"WARNING: Could not set battery failsafe parameters: {e}")
        print("Firmware may not support FS_BATT_ENABLE. Layer 2 (Python battery watchdog) still active.")


def setup_flow_hold_parameters(vehicle):
    """
    Verify and configure the ArduPilot parameters required for FLOW_HOLD.

    FLOW_HOLD needs:
      FLOW_ENABLE    = 1  — optical flow sensor enabled
      EK3_SRC1_VELXY = 5  — optical flow as XY velocity source for EKF3

    These are checked here and a warning is printed if they look wrong,
    but we do NOT abort — the operator may have set them via a GCS already.
    """
    print("Checking FLOW_HOLD ArduPilot parameters...")
    issues = []

    try:
        flow_enable = int(vehicle.parameters.get('FLOW_ENABLE', -1))
        print(f"   FLOW_ENABLE    = {flow_enable}  (required: 1)")
        if flow_enable != 1:
            issues.append("FLOW_ENABLE is not 1 — optical flow sensor may be disabled.")
            vehicle.parameters['FLOW_ENABLE'] = 1
            print("   → Set FLOW_ENABLE = 1")
    except Exception as e:
        print(f"   WARNING: Could not read/set FLOW_ENABLE: {e}")

    try:
        velxy_src = int(vehicle.parameters.get('EK3_SRC1_VELXY', -1))
        print(f"   EK3_SRC1_VELXY = {velxy_src}  (required: 5 for optical flow)")
        if velxy_src != 5:
            issues.append("EK3_SRC1_VELXY is not 5 — EKF may not be using optical flow for XY.")
            vehicle.parameters['EK3_SRC1_VELXY'] = 5
            print("   → Set EK3_SRC1_VELXY = 5")
    except Exception as e:
        print(f"   WARNING: Could not read/set EK3_SRC1_VELXY: {e}")

    if issues:
        print("\nWARNING: The following FLOW_HOLD parameter issues were found:")
        for i in issues:
            print(f"   • {i}")
        print("Parameters have been updated. A reboot may be required for changes to take effect.\n")
    else:
        print("FLOW_HOLD parameters look good!")


# ─── GCS / HEARTBEAT FAILSAFE (LAYER 2) ──────────────────────────────────────

def _heartbeat_callback(self, name, message):
    global _last_heartbeat_time
    with _last_heartbeat_lock:
        _last_heartbeat_time = time.time()


def _watchdog_thread(vehicle, timeout):
    """
    LAYER 2 — Python-side GCS watchdog thread.
    If no heartbeat for `timeout` seconds: sets failsafe, commands LAND, exits.
    """
    print(f"GCS failsafe watchdog started (timeout={timeout}s, poll={WATCHDOG_POLL_INTERVAL}s)")

    while not _watchdog_stop.is_set():
        time.sleep(WATCHDOG_POLL_INTERVAL)

        with _last_heartbeat_lock:
            last = _last_heartbeat_time

        if last is None:
            continue

        elapsed = time.time() - last

        if elapsed >= timeout:
            print("\n" + "!" * 60)
            print("FAILSAFE TRIGGERED — TELEMETRY CONNECTION LOST")
            print(f"No heartbeat received for {elapsed:.1f}s (limit={timeout}s)")
            print("!" * 60)

            _failsafe_triggered.set()

            try:
                print("Commanding LAND mode via serial link...")
                vehicle.mode = VehicleMode("LAND")
                print("LAND command sent.")
            except Exception as e:
                print(f"Could not send LAND command: {e}")
                print("Relying on ArduPilot GCS failsafe (Layer 1).")

            break

    print("GCS failsafe watchdog stopped.")


def start_failsafe_watchdog(vehicle, timeout=FAILSAFE_TIMEOUT):
    global _last_heartbeat_time

    with _last_heartbeat_lock:
        _last_heartbeat_time = time.time()

    vehicle.add_message_listener('HEARTBEAT', _heartbeat_callback)

    thread = threading.Thread(
        target=_watchdog_thread,
        args=(vehicle, timeout),
        daemon=True,
        name="gcs-failsafe-watchdog"
    )
    thread.start()
    return thread


def stop_failsafe_watchdog(vehicle):
    _watchdog_stop.set()
    try:
        vehicle.remove_message_listener('HEARTBEAT', _heartbeat_callback)
    except Exception:
        pass
    print("GCS failsafe watchdog deactivated.")



def _battery_watchdog_thread(vehicle, low_voltage):
    """
    LAYER 2 — Python-side battery voltage watchdog thread.
    Polls voltage every BATTERY_POLL_INTERVAL seconds.
    Commands LAND and sets failsafe if pack drops below low_voltage.
    """
    print(
        f"Battery failsafe watchdog started "
        f"(threshold={low_voltage:.1f} V, poll={BATTERY_POLL_INTERVAL}s)"
    )

    while not _battery_watchdog_stop.is_set():
        time.sleep(BATTERY_POLL_INTERVAL)

        try:
            voltage = vehicle.battery.voltage
        except Exception:
            voltage = None

        if voltage is None:
            print("Battery watchdog: voltage reading unavailable, retrying...")
            continue

        print(f"Battery: {voltage:.2f} V  (limit: {low_voltage:.1f} V)")

        if voltage < low_voltage:
            print("\n" + "!" * 60)
            print("BATTERY FAILSAFE TRIGGERED — LOW VOLTAGE DETECTED")
            print(f"Pack voltage: {voltage:.2f} V  |  Threshold: {low_voltage:.1f} V")
            print("Initiating automatic landing NOW!")
            print("!" * 60 + "\n")

            _failsafe_triggered.set()

            try:
                vehicle.mode = VehicleMode("LAND")
                print("Battery failsafe: LAND command sent.")
            except Exception as e:
                print(f"Battery failsafe: Could not send LAND command: {e}")
                print("Relying on ArduPilot battery failsafe (Layer 1).")

            break

    print("Battery failsafe watchdog stopped.")


def start_battery_watchdog(vehicle, low_voltage=BATTERY_LOW_VOLTAGE):
    thread = threading.Thread(
        target=_battery_watchdog_thread,
        args=(vehicle, low_voltage),
        daemon=True,
        name="battery-failsafe-watchdog"
    )
    thread.start()
    return thread


def stop_battery_watchdog():
    _battery_watchdog_stop.set()
    print("Battery failsafe watchdog deactivated.")



def safe_mode_name(vehicle):
    try:
        m = vehicle.mode
    except Exception:
        return "UNKNOWN"
    try:
        return m.name
    except Exception:
        try:
            return str(m)
        except Exception:
            return "UNKNOWN"


def check_flow_sensor_health(vehicle):
    """
    Read OPTICAL_FLOW MAVLink messages to verify the sensor is reporting.
    Waits up to 5 seconds for at least one valid flow reading.
    Returns True if healthy, False if no data received.
    """
    print("Checking optical flow sensor health...")
    flow_received = threading.Event()

    def _flow_cb(self, name, msg):
        # quality > 0 means the sensor is tracking surface features
        if hasattr(msg, 'quality') and msg.quality > 0:
            flow_received.set()

    vehicle.add_message_listener('OPTICAL_FLOW', _flow_cb)
    flow_received.wait(timeout=5)
    vehicle.remove_message_listener('OPTICAL_FLOW', _flow_cb)

    if flow_received.is_set():
        print("Optical flow sensor: OK (receiving valid flow data)")
        return True
    else:
        print("WARNING: No valid optical flow data received in 5s.")
        print("   Check sensor wiring, FLOW_ENABLE=1, and surface texture below drone.")
        return False


def arm_drone(vehicle, force=False):
    """
    Set FLOW_HOLD mode and arm the drone.

    FLOW_HOLD does not need GPS but does need:
      - Optical flow sensor healthy and reporting
      - EKF3 initialised with optical flow as XY velocity source
      - Barometer (or rangefinder) for altitude

    force=True skips the is_armable pre-arm check (useful for bench tests
    but should NOT be used for real flights without confirming sensor health).
    """
    print("Current flight mode:", safe_mode_name(vehicle))
    print("Drone ko FLOW_HOLD mode mein set kar rahe hain...")

    try:
        vehicle.mode = VehicleMode("FLOWHOLD")
    except Exception as ex:
        print(f"ERROR setting FLOW_HOLD mode: {ex}")
        raise

    # Wait until the FC confirms the mode switch
    timeout = 15
    while safe_mode_name(vehicle).upper() != "FLOWHOLD" and timeout > 0:
        print("   FLOW_HOLD mode ka wait kar rahe hain...")
        time.sleep(1)
        timeout -= 1

    if safe_mode_name(vehicle).upper() != "FLOWHOLD":
        raise RuntimeError(
            "FC did not switch to FLOW_HOLD. "
            "Check FLOW_ENABLE=1, EK3_SRC1_VELXY=5, and sensor health."
        )
    print("FLOW_HOLD mode set ho gaya!")

    print("Drone arm ho raha hai (motors start ho rahe hain)...")

    if not force:
        while not vehicle.is_armable:
            print("   Drone armable hone ka wait kar rahe hain...")
            try:
                print(f"   EKF OK: {vehicle.ekf_ok}")
            except Exception:
                pass
            time.sleep(1)
    else:
        print("WARNING: Force arming requested — skipping pre-arm checks!")

    vehicle.armed = True

    while not vehicle.armed:
        print("   Arming ka wait kar rahe hain...")
        time.sleep(1)

    print("Drone ARMED hai! Motors spin ho rahe hain!")


def set_thrust(vehicle, thrust):
    send_attitude_thrust(vehicle, roll_deg=0.0, pitch_deg=0.0, thrust=thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    roll_rad  = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad   = math.radians(yaw)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def get_velocity_ne(vehicle):
    try:
        vel = vehicle.velocity
        if vel and len(vel) >= 2:
            vn = float(vel[0]) if vel[0] is not None else 0.0
            ve = float(vel[1]) if vel[1] is not None else 0.0
            if math.isfinite(vn) and math.isfinite(ve):
                return vn, ve
    except Exception:
        pass
    return 0.0, 0.0


def send_attitude_thrust(vehicle, roll_deg=0.0, pitch_deg=0.0, thrust=0.5, yaw_deg=None):
    if yaw_deg is None:
        try:
            yaw_deg = float(vehicle.heading)
        except Exception:
            yaw_deg = 0.0

    q = to_quaternion(roll_deg, pitch_deg, yaw_deg)
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0b00000111,
        q,
        0, 0, 0,
        thrust
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def set_rc_throttle(vehicle, pwm):
    """Set channel 3 RC override. No-op if pilot has taken over."""
    if _pilot_takeover.is_set():
        return
    try:
        vehicle.channels.overrides['3'] = pwm
    except Exception:
        try:
            vehicle.channels.overrides[3] = pwm
        except Exception as e:
            print(f"Failed to set RC throttle override: {e}")


def climb_to(vehicle, target_altitude, use_rc_fallback=False):
    """
    Climb to target_altitude in FLOW_HOLD mode using thrust commands.

    FLOW_HOLD holds XY position via the optical flow sensor during the
    entire climb — we only need to manage the throttle to gain altitude.

    Falls back to RC channel-3 override if attitude-target commands have
    no effect (same logic as ALT_HOLD script).
    """
    print(f"FLOW_HOLD: climbing to {target_altitude}m")
    prev_alt = None
    no_change_count = 0
    using_rc_fallback = use_rc_fallback

    while True:
        if _pilot_takeover_requested(vehicle):
            print("Pilot RC takeover detected during climb. Returning control to pilot.")
            return using_rc_fallback

        if _failsafe_triggered.is_set():
            print("Failsafe active — aborting climb.")
            return using_rc_fallback

        alt = vehicle.location.global_relative_frame.alt
        print(f"   Altitude: {alt:.2f} / {target_altitude}m  | Mode: {safe_mode_name(vehicle)}")

        if alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break

        if prev_alt is None:
            prev_alt = alt
        else:
            if alt - prev_alt > 0.02:
                no_change_count = 0
            else:
                no_change_count += 1
            prev_alt = alt

        if not using_rc_fallback:
            set_thrust(vehicle, 0.85)
        else:
            print("Using RC throttle override fallback")
            set_rc_throttle(vehicle, RC_CLIMB_PWM)

        time.sleep(0.3)

        if not using_rc_fallback and no_change_count >= 4:
            print("No altitude change detected — enabling RC throttle override fallback")
            using_rc_fallback = True

    # Altitude reached — neutralise throttle and let FLOW_HOLD hold position.
    if using_rc_fallback:
        print(f"Setting neutral RC throttle ({RC_NEUTRAL_HOVER}) for hover phase")
        set_rc_throttle(vehicle, RC_NEUTRAL_HOVER)
    else:
        _clear_rc_overrides(vehicle)

    return using_rc_fallback


PHOTO_INTERVAL = 3  # seconds between photos during hover


def hover_flow_hold(vehicle, duration, target_altitude, use_rc_fallback=False):
    """
    Hover for `duration` seconds in FLOW_HOLD mode.

    The FC handles XY position hold via optical flow — Python just monitors
    altitude and applies a small throttle correction if needed (same RC-override
    or thrust approach as the original ALT_HOLD hover, since FLOW_HOLD still
    needs manual throttle management from the GCS when not using a pilot RC).

    Takes one photo every PHOTO_INTERVAL seconds.
    Returns a list of captured image file paths.
    """
    print(f"FLOW_HOLD hover: {duration}s at ~{target_altitude:.2f}m")
    print(f"Camera: 1 photo every {PHOTO_INTERVAL}s.")

    # Confirm we are in FLOW_HOLD before starting the hover loop.
    if safe_mode_name(vehicle).upper() != "FLOWHOLD":
        print("WARNING: Not in FLOW_HOLD — switching now...")
        vehicle.mode = VehicleMode("FLOWHOLD")
        wait = 10
        while safe_mode_name(vehicle).upper() != "FLOWHOLD" and wait > 0:
            time.sleep(1)
            wait -= 1
        if safe_mode_name(vehicle).upper() != "FLOWHOLD":
            print("WARNING: Could not confirm FLOW_HOLD — proceeding in current mode.")
        else:
            print("FLOW_HOLD confirmed for hover.")

    update_interval = 0.2
    total_steps = int(duration / update_interval)

    captured_images = []
    hover_start    = time.time()
    last_photo_time = hover_start - PHOTO_INTERVAL  # trigger first photo immediately

    for step in range(total_steps):
        if _pilot_takeover_requested(vehicle):
            print("Pilot RC takeover detected during hover. Returning control to pilot.")
            return captured_images

        if _failsafe_triggered.is_set():
            print("Failsafe active — stopping hover loop immediately.")
            return captured_images

        alt   = vehicle.location.global_relative_frame.alt
        error = target_altitude - alt

        # FLOW_HOLD manages XY on the FC; throttle is still our responsibility
        # when flying via MAVLink (no pilot RC).
        if use_rc_fallback:
            pwm = RC_NEUTRAL_HOVER
            if error > RC_DEADZONE:
                pwm = RC_CLIMB_PWM
            elif error < -RC_DEADZONE:
                pwm = 1450          # slight descend PWM
            set_rc_throttle(vehicle, pwm)
        else:
            thrust = 0.72 + (error * 0.1)
            thrust = clamp(thrust, 0.65, 0.80)
            # No manual XY correction needed — FLOW_HOLD handles that.
            send_attitude_thrust(vehicle, roll_deg=0.0, pitch_deg=0.0, thrust=thrust)

        if step % int(1 / update_interval) == 0:
            remaining = max(0, duration - int(step * update_interval))
            vn, ve = get_velocity_ne(vehicle)
            print(
                f"   {remaining}s remaining | Mode: {safe_mode_name(vehicle)} | "
                f"Alt: {alt:.2f}m | Err: {error:+.2f}m | "
                f"Vn: {vn:+.2f} Ve: {ve:+.2f}"
            )

        now = time.time()
        if now - last_photo_time >= PHOTO_INTERVAL:
            photo_index = len(captured_images) + 1
            image_path  = f"image_{photo_index}.jpg"
            _cam = None
            try:
                print(f"   [Camera] Initializing camera for photo {photo_index}...")
                _cam = Picamera2()
                _cam.configure(_cam.create_still_configuration())
                _cam.start()
                time.sleep(1)
                _cam.capture_file(image_path)
                captured_images.append(image_path)
                elapsed_s = int(now - hover_start)
                print(f"   [Camera] Photo {photo_index} captured at t={elapsed_s}s -> {image_path}")
            except Exception as e:
                print(f"   [Camera] WARNING: Failed to capture photo {photo_index}: {e}")
            finally:
                if _cam is not None:
                    try:
                        _cam.stop()
                        _cam.close()
                        print(f"   [Camera] Camera released after photo {photo_index}.")
                    except Exception:
                        pass
            last_photo_time = now

        time.sleep(update_interval)

    print(f"FLOW_HOLD hover complete. {len(captured_images)} photo(s) captured.")
    return captured_images


def send_images(image_paths):
    """
    Upload all captured images together to the server after landing.
    Each image is sent as a separate field in a single multipart POST request.
    """
    if not image_paths:
        print("No images to upload.")
        return

    url = "http://10.35.182.236:8000"
    print(f"Uploading {len(image_paths)} image(s) to {url} ...")

    open_files = []
    try:
        files = []
        for path in image_paths:
            fh = open(path, 'rb')
            open_files.append(fh)
            files.append(('file', (path, fh, 'image/jpeg')))

        response = requests.post(url, files=files, timeout=10)
        print(f"Upload successful ({len(image_paths)} files). Response: {response.text}")

    except requests.exceptions.ConnectionError:
        print("WARNING: Server unreachable — upload skipped. Images saved locally.")
    except requests.exceptions.Timeout:
        print("WARNING: Upload timed out after 10s — upload skipped. Images saved locally.")
    except Exception as e:
        print(f"WARNING: Upload failed ({e}) — upload skipped. Images saved locally.")
    finally:
        for fh in open_files:
            try:
                fh.close()
            except Exception:
                pass


def land_drone(vehicle):
    print("LANDING mode activate kar rahe hain...")
    _clear_rc_overrides(vehicle)
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"   Landing... Current Altitude: {current_alt:.2f}m")
        time.sleep(1)

    print("Drone successfully LAND ho gaya aur DISARMED hai!")


def close_connection(vehicle):
    print("Connection close kar rahe hain...")
    vehicle.close()
    print("Connection safely closed!")


def main():
    print("YO deep code chalu.... (FLOW_HOLD edition — peo_9)")

    vehicle = None

    try:
        vehicle = connect_to_drone()

        print(f"   Mode:   {safe_mode_name(vehicle)}")
        print(f"   Armed:  {vehicle.armed}")
        print(f"   EKF OK: {vehicle.ekf_ok}")
        try:
            velxy_src = int(vehicle.parameters.get('EK3_SRC1_VELXY', -1))
            flow_en   = int(vehicle.parameters.get('FLOW_ENABLE', -1))
            print(f"   FLOW_ENABLE    = {flow_en}  (need 1)")
            print(f"   EK3_SRC1_VELXY = {velxy_src}  (need 5 for optical flow)")
        except Exception:
            pass

        setup_flow_hold_parameters(vehicle)

        flow_ok = check_flow_sensor_health(vehicle)
        if not flow_ok:
            print("WARNING: Optical flow sensor not healthy. Proceeding with caution.")
            print("         XY position hold will be unreliable without a working flow sensor.")

        setup_ardupilot_gcs_failsafe(vehicle, timeout_sec=FAILSAFE_TIMEOUT)
        setup_ardupilot_battery_failsafe(vehicle, low_voltage=BATTERY_LOW_VOLTAGE)
        watchdog_thread = start_failsafe_watchdog(vehicle, timeout=FAILSAFE_TIMEOUT)
        battery_thread  = start_battery_watchdog(vehicle, low_voltage=BATTERY_LOW_VOLTAGE)

        arm_drone(vehicle, force=True)

        used_rc_fallback = climb_to(vehicle, TARGET_ALTITUDE, use_rc_fallback=True)

        # Only hover if no failsafe fired during climb
        captured_images = []
        if not _failsafe_triggered.is_set() and not _pilot_takeover.is_set():
            captured_images = hover_flow_hold(
                vehicle, HOVER_DURATION, TARGET_ALTITUDE,
                use_rc_fallback=used_rc_fallback
            )

        if _pilot_takeover.is_set():
            print("Pilot has taken over. Script will not command LAND.")
            stop_failsafe_watchdog(vehicle)
            stop_battery_watchdog()
            while vehicle.armed:
                current_alt = vehicle.location.global_relative_frame.alt
                print(f"   Pilot control active... Altitude: {current_alt:.2f}m")
                time.sleep(1)
            print("Pilot flight complete, drone disarmed.")
            return

        if not _failsafe_triggered.is_set():
            stop_failsafe_watchdog(vehicle)
            stop_battery_watchdog()
            land_drone(vehicle)
            send_images(captured_images)
        else:
            print("Waiting for failsafe landing to complete...")
            while vehicle.armed:
                current_alt = vehicle.location.global_relative_frame.alt
                print(f"   Failsafe landing... Altitude: {current_alt:.2f}m")
                time.sleep(1)
            print("Failsafe landing complete, drone disarmed.")
            stop_failsafe_watchdog(vehicle)
            stop_battery_watchdog()

    except KeyboardInterrupt:
        print("\nUser ne flight cancel ki (Ctrl+C)!")
        stop_failsafe_watchdog(vehicle)
        stop_battery_watchdog()
        if vehicle and vehicle.armed:
            print("Emergency landing kar rahe hain...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
            print("Emergency landing complete!")

    except Exception as e:
        print(f"\nERROR: {e}")
        if vehicle:
            stop_failsafe_watchdog(vehicle)
            stop_battery_watchdog()
        if vehicle and vehicle.armed:
            print("Error ke baad emergency landing kar rahe hain...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
            print("Emergency landing complete!")

    finally:
        if vehicle:
            close_connection(vehicle)

    print("Saras")


if __name__ == "__main__":
    main()
