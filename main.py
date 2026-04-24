"""
main_integrated.py  —  Tri-Factor Biometric System Bridge
═════════════════════════════════════════════════════════════
Full Hardware Watchdog: Camera, Fingerprint Sensor, and R3.
All operations pause and resume automatically on device disconnect/reconnect.
"""

import sys, os, time, shutil, threading, subprocess, json, sqlite3, queue as _queue_mod

# Force unbuffered output so all background-thread print() calls appear immediately
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Dependencies must be pre-installed (no runtime pip for air-gapped deployments)

from arduino.app_utils import Bridge, App
import serial
import cam_utils
import face_enrollment  as fe
import face_recognition as fr

USER_DB = "User_Database"
os.makedirs(USER_DB, exist_ok=True)
VOTE_DB_FILE = os.path.join(USER_DB, "vote_ledger.db")

# ── R3 Connection (via TCP Socket) ───────────────────────────────
UNO_R3_PORT = 'socket://172.17.0.1:9000'
r3_serial = None
# Thread-safe response queue — eliminates PING/PONG contamination race condition (Issue #3 & #7)
r3_response_q: "_queue_mod.Queue[str]" = _queue_mod.Queue()
r3_lock = threading.Lock()

print("[BRIDGE] R3 socat managed exclusively in ballot mode.")

# Atomic reconnect guard — threading.Event prevents duplicate threads
_r3_reconnect_ev = threading.Event()
_r3_watchdog_active = False
_ballot_mode_active = False          # socat only runs while this is True

# ── Shared port-finding / socat helpers ────────────────────────────────
def _find_r3_port_candidates():
    """Returns ordered list of candidate serial ports for R3 probing."""
    try:
        from serial.tools import list_ports
        candidates = [p.device for p in list_ports.comports()]
    except Exception:
        import glob
        candidates = (glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
                      + glob.glob("/dev/ttyS[0-9]*") + glob.glob("/dev/ttyO*"))
    return sorted(candidates)

_r3_confirmed_port = None   # Once verified, go straight here on reconnect

def _try_connect_r3() -> bool:
    """Probes each candidate serial port with LOCK→OK handshake to confirm R3.
    Uses cached port on reconnect to skip probing. Returns True on success."""
    global r3_serial, _r3_confirmed_port

    # Build probe list: try confirmed port first (fast reconnect), then all others
    candidates = _find_r3_port_candidates()
    if not candidates:
        print("[BALLOT] No serial ports found at all.")
        return False

    print(f"[BALLOT] Probing ports: {candidates}")
    if _r3_confirmed_port and _r3_confirmed_port not in candidates:
        _r3_confirmed_port = None   # Device was removed

    probe_order = []
    if _r3_confirmed_port:
        probe_order.append(_r3_confirmed_port)  # Try known port first
    probe_order += [p for p in candidates if p != _r3_confirmed_port]

    for port in probe_order:
        print(f"[BALLOT] Probing {port}...")
        os.system("killall socat 2>/dev/null")
        time.sleep(0.3)
        os.system(f"socat TCP-LISTEN:9000,fork,reuseaddr FILE:{port},b115200,raw,echo=0,nonblock 2>/dev/null &")
        try:
            time.sleep(1.2)   # Give socat + any Arduino boot time
            new_serial = serial.serial_for_url(UNO_R3_PORT, baudrate=115200, timeout=2)

            # Handshake: send LOCK, expect OK — proves R3 is on this port
            new_serial.reset_input_buffer()
            new_serial.write(b"LOCK\n")
            new_serial.flush()
            deadline = time.time() + 2.5
            confirmed = False
            while time.time() < deadline:
                if new_serial.in_waiting > 0:
                    resp = new_serial.readline().decode('utf-8', errors='ignore').strip()
                    if resp == "OK":
                        confirmed = True
                        break
                time.sleep(0.05)

            if confirmed:
                with r3_lock:
                    r3_serial = new_serial
                _r3_confirmed_port = port
                # Drain stale queue
                while not r3_response_q.empty():
                    try: r3_response_q.get_nowait()
                    except: break
                print(f"[BALLOT] ✅ R3 confirmed and connected on {port}")
                return True
            else:
                print(f"[BALLOT] No R3 response on {port} — trying next...")
                try: new_serial.close()
                except: pass
                os.system("killall socat 2>/dev/null")
        except Exception as e:
            print(f"[BALLOT] {port} probe failed: {e}")
            os.system("killall socat 2>/dev/null")

    print("[BALLOT] R3 not found on any port.")
    return False

def _reconnect_r3_bg():
    """Polls every 1 second for the R3 port and reconnects immediately when plugged in.
    Stops when _ballot_mode_active goes False."""
    print("[WATCHDOG] R3 disconnected — scanning for port every 1s...")
    attempt = 1
    while _ballot_mode_active:
        if _try_connect_r3():
            print("[WATCHDOG] R3 reconnected.")
            _r3_reconnect_ev.clear()
            return
        print(f"[WATCHDOG] R3 not found — retry {attempt}")
        attempt += 1
        time.sleep(1)
    _r3_reconnect_ev.clear()
    print("[WATCHDOG] Ballot mode ended — stopping reconnect loop.")


def _r3_send(command, cancel_event=None):
    """Send command to R3. Lock is released before I/O to prevent deadlock (Issue #4)."""
    global r3_serial

    if cancel_event and cancel_event.is_set():
        return "CANCELLED"

    # Snapshot the serial object under lock, then release before I/O
    with r3_lock:
        local_serial = r3_serial

    if local_serial:
        try:
            local_serial.reset_input_buffer()
            local_serial.write((command + "\n").encode('utf-8'))
            local_serial.flush()
            print(f"[R3 TX] {command}")
            return "OK"
        except Exception as e:
            print(f"[R3 ERROR] Write failed: {e}. Connection lost.")
            try: local_serial.close()
            except: pass
            with r3_lock:
                if r3_serial == local_serial:
                    r3_serial = None

    # ── WATCHDOG: R3 Connection Drop ──
    if _r3_watchdog_active:
        _set(lcd1="Disconnected", lcd2="Waiting...")
        print("[WATCHDOG] R3 offline — waiting up to 30s for reconnect before aborting.")
        if not _r3_reconnect_ev.is_set():
            _r3_reconnect_ev.set()
            threading.Thread(target=_reconnect_r3_bg, daemon=True).start()
        # Block up to 30s — if R3 comes back, retry the send and let the session continue
        reconnect_deadline = time.time() + 30.0
        while time.time() < reconnect_deadline:
            if cancel_event and cancel_event.is_set():
                break
            with r3_lock:
                if r3_serial is not None:
                    break
            time.sleep(0.5)
        with r3_lock:
            recovered = r3_serial
        if recovered:
            try:
                recovered.reset_input_buffer()
                recovered.write((command + "\n").encode('utf-8'))
                recovered.flush()
                print(f"[R3 TX] {command} (after reconnect)")
                _set(lcd1="R3 Restored!", lcd2="Resuming...")
                return "OK"
            except Exception as e:
                print(f"[R3 ERROR] Retry after reconnect failed: {e}")
        _set(lcd1="R3 TIMEOUT", lcd2="Session aborted")
    return "TIMEOUT"

def _r3_wait_response(expected, timeout_s=5.0):
    """Drains the response queue until expected token found or timeout (Issue #3)."""
    deadline = time.time() + timeout_s
    FAILURE_TOKENS = {"FAIL", "TIMEOUT", "ERROR", "MISSING"}
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return False
        try:
            token = r3_response_q.get(timeout=min(remaining, 0.5))
            if token == expected:
                return True
            if token in FAILURE_TOKENS and expected not in FAILURE_TOKENS:
                # "ERROR" comes from the listener on disconnect — if a reconnect is
                # already in progress, wait for it instead of failing immediately (Gap 3)
                if token == "ERROR" and _r3_reconnect_ev.is_set():
                    reconnect_wait = min(remaining, 30.0)
                    print(f"[R3] Disconnect mid-response — waiting up to {reconnect_wait:.0f}s for reconnect...")
                    wait_deadline = time.time() + reconnect_wait
                    while time.time() < wait_deadline:
                        if not _r3_reconnect_ev.is_set():
                            break   # reconnect finished — resume draining queue
                        time.sleep(0.3)
                    deadline = max(deadline, time.time() + 2.0)  # extend outer deadline
                    continue    # drain queue again — don't return False yet
                return False
            # Any other token: keep draining (don't discard unknown)
        except _queue_mod.Empty:
            pass
    return False

def _r3_listener_loop():
    """Reads R3 responses into the queue. Lock released before I/O to prevent deadlock (Issue #4).
    PONG responses are silently discarded to prevent session-state contamination (Issue #7)."""
    global r3_serial
    last_ping = time.time()
    while True:
        with r3_lock:
            local_serial = r3_serial
        if not local_serial:
            time.sleep(1)
            continue
        try:
            if local_serial.in_waiting > 0:
                # Release lock before blocking I/O
                line = local_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    if line == "PONG":
                        pass  # Silently discard — never contaminate session state (Issue #7)
                    else:
                        r3_response_q.put(line)  # Thread-safe enqueue (Issue #3)
                    print(f"[R3 RX] {line}")
            else:
                # Keepalive Heartbeat — 2s interval, lock released before write (Issue #4)
                now = time.time()
                if (now - last_ping) >= 2.0:
                    local_serial.write("PING\n".encode())
                    local_serial.flush()
                    last_ping = now
        except Exception as e:
            # Always log disconnect
            print(f"[R3 ERROR] Listener lost connection: {e}")
            try: local_serial.close()
            except: pass
            with r3_lock:
                if r3_serial == local_serial:
                    r3_serial = None
            r3_response_q.put("ERROR")
            # Reconnect only during ballot mode — no spurious threads during idle/enroll/delete
            if _ballot_mode_active and not _r3_reconnect_ev.is_set():
                _r3_reconnect_ev.set()
                threading.Thread(target=_reconnect_r3_bg, daemon=True).start()
            last_ping = time.time()  # Reset ping timer for new connection
        time.sleep(0.01)

threading.Thread(target=_r3_listener_loop, daemon=True).start()

# ── Shared state ─────────────────────────────────────────────────
_lock  = threading.Lock()
_state = { "lcd1": "Biometric Sys", "lcd2": "Ready", "result": "IDLE", "busy": False, "cur_uid": "" }
_cancel     = threading.Event()
_fp_done    = threading.Event()
_fp_ok_flag = [False]

def _set(lcd1=None, lcd2=None, result=None):
    with _lock:
        if lcd1 is not None: 
            _state["lcd1"] = lcd1[:16]
        if lcd2 is not None: 
            _state["lcd2"] = lcd2[:16]
        if result is not None: 
            _state["result"] = result

def _status_cb(l1, l2, beep=None): 
    _set(lcd1=l1, lcd2=l2)

# ══════════════════════════════════════════════════════════════
def check_user_exists(user_id: str) -> str:
    try:
        with sqlite3.connect(VOTE_DB_FILE) as conn:
            res = conn.execute("SELECT 1 FROM faces WHERE uid = ?", (str(user_id),)).fetchone()
            return "YES" if res else "NO"
    except Exception: return "NO"

def start_enrollment(user_id: str) -> str:
    with _lock:
        if _state["busy"]: return "BUSY"
        _state["busy"]    = True
        _state["result"]  = "WORKING"
        _state["cur_uid"] = user_id
    _cancel.clear()
    _fp_done.clear()
    _fp_ok_flag[0] = False
    _set(lcd1="Face Enroll", lcd2=f"ID:{user_id}")

    def _on_capture_done():
        _set(lcd1="Scan finger now", lcd2="(3 tries)", result="CAPTURE_DONE")

    def _run():
        try:
            ok = fe.enroll_user(user_id, status_cb=_status_cb, cancel_event=_cancel, capture_done_cb=_on_capture_done, fp_done_event=_fp_done)
            if ok: _set(lcd1="Enrolled OK!", lcd2=f"ID:{user_id}", result="FACE_DONE")
            else:  _set(lcd1="Enroll FAILED", lcd2="Try again", result="FACE_ERROR")
        finally:
            with _lock: _state["busy"] = False

    try:
        threading.Thread(target=_run, daemon=True).start()
    except Exception:
        with _lock: _state["busy"] = False
        return "ERROR"
    return "STARTED"

def fp_success(user_id: str) -> str:
    _fp_ok_flag[0] = True
    _fp_done.set()
    _set(lcd1="FP done!", lcd2="Finishing math")
    return "OK"

def fp_failed(user_id: str) -> str:
    _fp_ok_flag[0] = False
    _cancel.set()
    _fp_done.set()
    _set(lcd1="FP FAILED", lcd2="Rolled back", result="FACE_ERROR")
    with _lock: _state["busy"] = False
    return "OK"

# start_recognition removed per final technical audit

_template_crc = 0

def perform_jit_handshake(user_id: str, cancel_ev=None) -> str:
    global _template_crc, _r3_watchdog_active
    _template_crc = 0
    _r3_watchdog_active = True
    try:
        send_result = _r3_send(f"START:{user_id}", cancel_ev)
        if send_result in ["CANCELLED", "TIMEOUT"]:
            _r3_watchdog_active = False
            return "FAILED"
        if not _r3_wait_response("OK", timeout_s=15.0): 
            _r3_watchdog_active = False
            return "FAILED"
        return "OK"
    except Exception: 
        _r3_watchdog_active = False
        return "FAILED"

def extract_template(user_id: str) -> str: 
    if perform_jit_handshake(user_id) == "OK": return "EXTRACT"
    return "ERROR"

def send_template_chunk(chunk_hex: str) -> str:
    global _template_crc
    try:
        bs = bytes.fromhex(chunk_hex)
        _template_crc += sum(bs)
    except ValueError as e:
        # Malformed hex from Arduino — CRC is now wrong, abort the transfer
        print(f"[TEMPLATE] Hex decode failed: {e!r} for chunk '{chunk_hex[:16]}...' — aborting")
        return "ERROR"
    send_result = _r3_send(f"D:{chunk_hex}", _cancel)
    if send_result in ["CANCELLED", "TIMEOUT"]:
        return "ERROR"
    if not _r3_wait_response("OK", timeout_s=15.0):
        return "ERROR"
    return "OK"

def finish_template_transfer(user_id: str) -> str:
    global _template_crc, _r3_watchdog_active
    send_result = _r3_send(f"FINISH:{_template_crc:X}", _cancel)
    if send_result in ["CANCELLED", "TIMEOUT"]: 
        _r3_watchdog_active = False
        return "ERROR"
    if not _r3_wait_response("SAVED", timeout_s=15.0): 
        _r3_watchdog_active = False
        return "ERROR"
    _r3_watchdog_active = False
    return "OK"

def abort_template_transfer() -> str:
    _r3_send("ABORT", _cancel)
    return "OK"

# check_fp_sensor removed per unused placeholder

# ══════════════════════════════════════════════════════════════
def get_lcd1() -> str:
    with _lock: return _state["lcd1"]
def get_lcd2() -> str:
    with _lock: return _state["lcd2"]
def get_result() -> str:
    with _lock: return _state["result"]

def get_face_count() -> str:
    try:
        with sqlite3.connect(VOTE_DB_FILE) as conn:
             res = conn.execute("SELECT COUNT(uid) FROM faces WHERE uid != '1'").fetchone()
             return str(res[0]) if res else "0"
    except Exception: return "0"

def wipe_all_auth_data() -> str:
    try:
        for u in os.listdir(USER_DB):
            if u != "vote_ledger.db" and os.path.isdir(os.path.join(USER_DB, u)):
                shutil.rmtree(os.path.join(USER_DB, u))
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM faces")
            conn.execute("DELETE FROM votes")  # FIX: also wipe the python vote records
            conn.execute("DELETE FROM config")  # clears admin_finger so verifyAdmin shows fresh name
        _set(lcd1="Face DB wiped", lcd2="All clear", result="IDLE")
        with _lock: _state["busy"] = False
        return "DONE"
    except Exception as e:
        print(f"[ADMIN] wipe_all_auth_data failed mid-operation: {e}")
        return "ERROR"

def delete_single_user(uid: str) -> str:
    """Returns ERROR if deletion fails — a silent success lie is a security flaw (Issue #5)."""
    if str(uid) == "1": return "DENIED"
    try:
        p = os.path.join(USER_DB, str(uid))
        if os.path.exists(p): shutil.rmtree(p)
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM faces WHERE uid = ?", (str(uid),))
            conn.execute("DELETE FROM config WHERE key = ?", (f'voter_finger_{uid}',))
            conn.execute("DELETE FROM votes WHERE uid = ?", (str(uid),))
        return "DONE"
    except Exception as e:
        print(f"[DB] delete_single_user failed for uid={uid}: {e}")
        return "ERROR"

def delete_admin_face() -> str:
    """Wipes admin (uid=1) face data so enrollAdminFlow can re-enroll cleanly."""
    try:
        p = os.path.join(USER_DB, "1")
        if os.path.exists(p): shutil.rmtree(p)
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM faces WHERE uid = '1'")
        print("[ENROLL] Admin face data cleared for re-enrollment.")
    except Exception as e:
        print(f"[ENROLL] delete_admin_face error: {e}")
    return "OK"

def delete_non_admins() -> str:
    try:
        for u in os.listdir(USER_DB):
            if u != "1" and u != "vote_ledger.db" and os.path.isdir(os.path.join(USER_DB, u)):
                shutil.rmtree(os.path.join(USER_DB, u))
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM faces WHERE uid != '1'")
            conn.execute("DELETE FROM config WHERE key NOT IN ('admin_finger')")
            conn.execute("DELETE FROM votes")
        return "DONE"
    except Exception as e:
        print(f"[ADMIN] delete_non_admins failed mid-operation: {e}")
        return "ERROR"

def verify_admin_face() -> str:
    with _lock:
        if _state["busy"]: return "BUSY"
        _state["busy"]   = True
        _state["result"] = "WORKING"
        _state["cur_uid"] = "1"
    _cancel.clear()
    
    def _run():
        try:
            _set(lcd1="Officer Check", lcd2="Verifying ID: 1")
            # Wrapper: pass all messages through EXCEPT the generic 'Face Denied'
            # which recognize_user emits just before returning DENIED — we suppress
            # it because verify_admin_face sets its own clearer message right after.
            def _admin_cb(l1, l2="", beep=None):
                if l1 == "Face Denied":   # suppress; outer sets "Admin Face NG"
                    if beep: _r3_send(f"BEEP:{beep}")  # still beep
                    return
                _status_cb(l1, l2, beep)

            res = fr.recognize_user("1", status_cb=_admin_cb, cancel_event=_cancel)
            if res != "GRANTED":
                _set(lcd1="Admin Face NG", lcd2="Access denied", result="FACE_DENIED")
                return "FACE_DENIED"
            _set(lcd1="Admin Verified", lcd2="Unlock granted", result="FACE_GRANTED")
            return "GRANTED"
        except Exception as e:
            print(f"[ERROR] Admin face check failed: {e}")
            _set(lcd1="Error", lcd2=str(e)[:16], result="FACE_ERROR")
        finally:
            with _lock:
                _state["busy"] = False
                if _state["result"] == "WORKING": _state["result"] = "IDLE"
    
    try:
        threading.Thread(target=_run, daemon=True).start()
    except Exception:
        with _lock: _state["busy"] = False
        return "ERROR"
    return "OK"

def cancel_operation() -> str:
    _cancel.set(); _fp_done.set(); _set(lcd1="Cancelled", lcd2="", result="IDLE")
    with _lock: _state["busy"] = False
    return "OK"

def notify_access(result: str) -> str:
    verdict = "GRANTED" if result == "GRANTED" else "DENIED"
    if verdict == "GRANTED": _set(lcd1="Match Found!", lcd2="Welcome")
    else: _set(lcd1="No Match",  lcd2="Access Denied")
    _set(result="IDLE")
    with _lock: _state["busy"] = False
    return "OK"

def run_ballot_session(uid: str) -> str:
    global _r3_watchdog_active
    with _lock:
        if _state["busy"]: return "BUSY"
        _state["busy"] = True
        _state["result"] = "WORKING"
    _cancel.clear()
    _r3_watchdog_active = True

    def _run():
        global _r3_watchdog_active
        try:
            # Step 2: WAKE R3
            _set(lcd1="Voter Ready", lcd2="Starting Unit")
            if _r3_send(f"WAKE:{uid}") != "OK":
                _set(lcd1="Unit Error", lcd2="Please Retry", result="BALLOT_ABORT")
                with _lock: _state["busy"] = False
                _r3_watchdog_active = False
                return
                
            if not _r3_wait_response("AWAKE_OK", 28.0):  # 24s worst (3×8s WAKE FP) + 4s buffer
                _set(lcd1="Timeout", lcd2="Retry Scan", result="BALLOT_ABORT")
                with _lock: _state["busy"] = False
                _r3_watchdog_active = False
                return

            # Step 3: FACE CHECK
            _set(lcd1="Checking Face", lcd2="Please Wait...")
            user_dir = os.path.join(USER_DB, uid)
            if not os.path.exists(user_dir):
                _set(lcd1="Face Error", lcd2="Identity Missing", result="BALLOT_ABORT")
                _r3_send("LOCK")
                with _lock: _state["busy"] = False
                return

            try:
                def _ballot_status_cb(l1, l2, beep=None):
                    _set(lcd1=l1, lcd2=l2)
                    # Snapshot under lock, release before I/O to prevent deadlock (Issue #4)
                    with r3_lock:
                        local_s = r3_serial
                    if local_s is not None:
                        try: 
                            local_s.write(f"LCD:{l1}|{l2}\n".encode())
                            if beep:
                                local_s.write(f"BEEP:{beep}\n".encode())
                        except: pass

                res = fr.recognize_user(uid, status_cb=_ballot_status_cb, cancel_event=_cancel)
                if res != "GRANTED":
                    _set(lcd1="Auth Rejected", lcd2="Officer Needed", result="BALLOT_ABORT")
                    _r3_send("BEEP:LONG")
                    _r3_send("LOCK")
                    with _lock: _state["busy"] = False
                    return
            except Exception as e:
                _set(lcd1="Face Error", lcd2=str(e)[:16], result="BALLOT_ABORT")
                _r3_send("LOCK")
                with _lock: _state["busy"] = False
                return

            _set(lcd1="Face Match OK", lcd2="Unlocking...")
            _r3_send("BEEP:DOUBLE")
            
            # Step 4, 5, 6: UNLOCK AND CAPTURE VOTE
            if _r3_send(f"UNLOCK:{uid}") != "OK":
                _set(lcd1="Unit Error", lcd2="Please Retry", result="BALLOT_ABORT")
                with _lock: _state["busy"] = False
                return

            # ── Live Camera Monitoring ────────────────────────────
            # Runs in background: aborts session if >1 face detected for 3s
            _monitor_stop   = threading.Event()
            _monitor_reason = [None]

            def _on_monitor_abort(reason):
                """Called by monitor thread on intrusion detection — fires once."""
                if not _monitor_stop.is_set():
                    _monitor_reason[0] = reason
                    _monitor_stop.set()

            # Fix Issue #1: correct function name + correct 4-arg signature
            threading.Thread(
                target=fr.continuous_monitoring,
                args=(uid, _on_monitor_abort, _ballot_status_cb, _monitor_stop),
                daemon=True
            ).start()

            _set(lcd1="Voting Active", lcd2="Keep Looking")

            # ── Vote Result Wait Loop ─────────────────────────────
            # Drains r3_response_q — never reads r3_latest_response (critical fix for queue migration)
            # 55s Python timeout — worst case: 15s select + 5s change + (3×8s seal FP) = 44s + 11s buffer.
            VOTE_TIMEOUT_S = 55.0
            deadline_t = time.time() + VOTE_TIMEOUT_S
            ABORT_TOKENS = {"FAIL", "CANCELED", "TIMEOUT", "ERROR", "MISSING"}

            while time.time() < deadline_t:

                # ── Intrusion abort path ──
                if _monitor_stop.is_set() and _monitor_reason[0]:
                    reason = _monitor_reason[0]
                    print(f"[BALLOT] Monitor abort: {reason}")
                    _set(lcd1="INTRUSION ALERT!", lcd2=reason[:16], result="BALLOT_ABORT")
                    _r3_send("LOCK")
                    with _lock: _state["busy"] = False
                    return

                # ── Drain queue for vote outcome ──
                try:
                    resp = r3_response_q.get(timeout=0.1)
                except _queue_mod.Empty:
                    continue

                if resp.startswith("CAST:"):
                    _monitor_stop.set()
                    _set(lcd1="Vote Casted!", lcd2="Session Closed", result="BALLOT_SUCCESS")
                    mark_as_voted(uid)
                    with _lock: _state["busy"] = False
                    return
                if resp in ABORT_TOKENS:
                    _monitor_stop.set()
                    _set(lcd1="Session Aborted", lcd2=resp, result="BALLOT_ABORT")
                    with _lock: _state["busy"] = False
                    return
                # Any other token (e.g. stray LCD echo): keep draining

            # Python-side timeout: stop monitor, lock R3, mark abort
            _monitor_stop.set()
            _set(lcd1="Vote Timeout", lcd2="Session expired", result="BALLOT_ABORT")
            _r3_send("LOCK")
                 
        except Exception as e:
            _set(lcd1="Fatal Error", lcd2=str(e)[:16], result="BALLOT_ABORT")
        finally:
            _r3_watchdog_active = False
            # Always stop the monitor thread — guard against NameError if
            # exception fired before the UNLOCK section created _monitor_stop.
            try: _monitor_stop.set()
            except NameError: pass
            with _lock:
                _state["busy"] = False
                if _state["result"] == "WORKING": _state["result"] = "IDLE"

    try:
        threading.Thread(target=_run, daemon=True).start()
    except Exception:
        with _lock: _state["busy"] = False
        return "ERROR"
    return "OK"

def get_r3_status() -> str:
    """Pure connection status check. Socat is managed exclusively by
    begin_ballot_mode / end_ballot_mode / _reconnect_r3_bg."""
    with r3_lock:
        if r3_serial is not None:
            try:
                _ = r3_serial.in_waiting
                return "ONLINE"
            except Exception:
                pass
    return "OFFLINE"

def begin_ballot_mode() -> str:
    """Called by Uno Q at the start of recognizeFlow.
    Starts socat and connects R3. If R3 isn't plugged in yet, starts the background
    reconnect loop so the Uno Q's get_r3_status polling exits as soon as it's plugged in."""
    global _ballot_mode_active
    _ballot_mode_active = True
    print("[BALLOT] Entering ballot mode — starting R3 connection.")
    if not _try_connect_r3():
        # R3 not found yet — keep trying every 1s in background
        if not _r3_reconnect_ev.is_set():
            _r3_reconnect_ev.set()
            threading.Thread(target=_reconnect_r3_bg, daemon=True).start()
    return "OK"

def end_ballot_mode() -> str:
    """Called by Uno Q when leaving recognizeFlow (normal exit or cancel).
    Kills socat and closes the serial link so nothing runs outside ballot mode."""
    global _ballot_mode_active, r3_serial
    _ballot_mode_active = False
    print("[BALLOT] Leaving ballot mode — cleaning up R3 connection.")
    with r3_lock:
        s = r3_serial
        r3_serial = None
    if s:
        try: s.close()
        except: pass
    os.system("killall socat 2>/dev/null")
    return "OK"

def lock_r3() -> str:
    _r3_send("LOCK")
    return "OK"

def begin_cam_mode() -> str:
    """Pre-opens the camera once at mode entry so all face calls reuse it instantly.
    Mirrors begin_ballot_mode for socat — same lifecycle pattern for camera.
    Guards against double-open if called while a shared camera already exists."""
    with cam_utils._shared_cap_lock:
        if cam_utils._shared_cap is not None and cam_utils._shared_cap.isOpened():
            print("[CAM] begin_cam_mode: camera already shared, reusing.")
            return "OK"
    cap = cam_utils.open_cam()
    if cap is None:
        print("[CAM] begin_cam_mode: camera not found")
        return "ERROR"
    cam_utils.set_shared_cam(cap)
    print("[CAM] Camera pre-warmed and shared.")
    return "OK"

def end_cam_mode() -> str:
    """Releases the shared camera on mode exit.
    Mirrors end_ballot_mode for socat."""
    with cam_utils._shared_cap_lock:
        cap = cam_utils._shared_cap
        cam_utils._shared_cap = None
    if cap is not None:
        try: cap.release()
        except: pass
    print("[CAM] Shared camera released.")
    return "OK"

# ── Vote Tracking Ledger (SQLite) ────────────────────────────────
def _get_db_conn():
    """Returns a WAL-mode connection with a busy timeout for concurrent thread safety (Issue #5)."""
    conn = sqlite3.connect(VOTE_DB_FILE, timeout=10.0)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA busy_timeout=5000")
    return conn

def _init_db():
    try:
        with _get_db_conn() as conn:
            conn.execute("CREATE TABLE IF NOT EXISTS votes (uid TEXT PRIMARY KEY, timestamp DATETIME DEFAULT CURRENT_TIMESTAMP)")
            conn.execute("CREATE TABLE IF NOT EXISTS faces (uid TEXT PRIMARY KEY, threshold REAL, vectors BLOB, names BLOB, voted INTEGER DEFAULT 0)")
            conn.execute("CREATE TABLE IF NOT EXISTS config (key TEXT PRIMARY KEY, value TEXT)")
    except Exception as e:
        print(f"[DB] _init_db failed: {e}")
_init_db()

def get_admin_finger() -> str:
    try:
        with _get_db_conn() as conn:
            row = conn.execute("SELECT value FROM config WHERE key='admin_finger'").fetchone()
            return row[0] if row else ""
    except: return ""

def set_admin_finger(finger: str) -> str:
    try:
        with _get_db_conn() as conn:
            conn.execute("INSERT OR REPLACE INTO config (key, value) VALUES ('admin_finger', ?)", (finger,))
        return "OK"
    except: return "ERROR"

def set_voter_finger(data: str) -> str:
    try:
        parts = data.split("|")
        if len(parts) == 2:
            uid, finger = parts
            with _get_db_conn() as conn:
                conn.execute("INSERT OR REPLACE INTO config (key, value) VALUES (?, ?)", (f'voter_finger_{uid}', finger))
            return "OK"
    except Exception: pass
    return "ERROR"

def get_voter_finger(uid: str) -> str:
    try:
        with _get_db_conn() as conn:
            row = conn.execute("SELECT value FROM config WHERE key=?", (f'voter_finger_{uid}',)).fetchone()
            return row[0] if row else ""
    except Exception: return ""

def mark_as_voted(uid: str) -> str:
    try:
        with _get_db_conn() as conn:
            conn.execute("INSERT OR IGNORE INTO votes (uid) VALUES (?)", (str(uid),))
            conn.execute("UPDATE faces SET voted = 1 WHERE uid = ?", (str(uid),))
    except Exception as e:
        # Log but don't crash — R3 EEPROM already records the vote, preventing double voting
        print(f"[LEDGER] mark_as_voted failed for uid={uid}: {e}")
    return "OK"

def check_has_voted(uid: str) -> str:
    """Returns 'YES', 'NO', or 'ERROR'. Never returns 'NO' on a DB failure
    (fail-open would allow a voted voter to vote again if the DB is unavailable)."""
    try:
        with _get_db_conn() as conn:
            res = conn.execute("SELECT 1 FROM votes WHERE uid = ?", (str(uid),)).fetchone()
            return "YES" if res else "NO"
    except Exception as e:
        print(f"[DB][CRITICAL] check_has_voted FAILED for uid={uid}: {e} — returning ERROR (fail-safe)")
        return "ERROR"  # Caller must treat this as DENY, not as 'not voted'

def clear_vote_status(uid: str) -> str:
    try:
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM votes WHERE uid = ?", (str(uid),))
            conn.execute("UPDATE faces SET voted = 0 WHERE uid = ?", (str(uid),))
        _r3_send(f"CLEAR_VOTE:{uid}")
        return "OK"
    except Exception as e:
        print(f"[DB] clear_vote_status error uid={uid}: {e}")
        return "ERROR"

def clear_all_vote_status() -> str:
    try:
        with _get_db_conn() as conn:
            conn.execute("DELETE FROM votes")
            conn.execute("UPDATE faces SET voted = 0")
        return "OK"
    except Exception as e:
        print(f"[DB] clear_all_vote_status error: {e}")
        return "ERROR"

def get_vote_count() -> str:
    try:
        with _get_db_conn() as conn:
            res = conn.execute("SELECT COUNT(*) FROM votes").fetchone()
            return str(res[0]) if res else "0"
    except Exception as e:
        print(f"[DB] get_vote_count error: {e}")
        return "0"

def get_unvoted_count() -> str:
    try:
        with _get_db_conn() as conn:
            res = conn.execute("SELECT COUNT(*) FROM faces WHERE voted = 0").fetchone()
            return str(res[0]) if res else "0"
    except Exception: return "0"

# ── Register Bridge ──────────────────────────────────────────────
Bridge.provide("get_unvoted_count", get_unvoted_count)
Bridge.provide("mark_as_voted",     mark_as_voted)
Bridge.provide("check_has_voted",   check_has_voted)
Bridge.provide("clear_vote_status", clear_vote_status)
Bridge.provide("clear_all_vote_status", clear_all_vote_status)
Bridge.provide("get_vote_count",    get_vote_count)
Bridge.provide("check_user_exists", check_user_exists)
Bridge.provide("start_enrollment",  start_enrollment)
Bridge.provide("run_ballot_session",run_ballot_session)
Bridge.provide("fp_success",        fp_success)
Bridge.provide("fp_failed",         fp_failed)
Bridge.provide("extract_template",  extract_template)
Bridge.provide("send_template_chunk", send_template_chunk)
Bridge.provide("finish_template_transfer", finish_template_transfer)
Bridge.provide("get_lcd1",          get_lcd1)
Bridge.provide("get_lcd2",          get_lcd2)
Bridge.provide("get_result",        get_result)
Bridge.provide("get_face_count",    get_face_count)
Bridge.provide("wipe_all_auth_data",wipe_all_auth_data)
Bridge.provide("delete_single_user", delete_single_user)
Bridge.provide("delete_non_admins",  delete_non_admins)
Bridge.provide("verify_admin_face",  verify_admin_face)
Bridge.provide("cancel_operation",  cancel_operation)
Bridge.provide("notify_access",     notify_access)
Bridge.provide("get_r3_status",     get_r3_status)
Bridge.provide("lock_r3",           lock_r3)
Bridge.provide("abort_template_transfer", abort_template_transfer)
Bridge.provide("begin_ballot_mode", begin_ballot_mode)
Bridge.provide("end_ballot_mode",   end_ballot_mode)
Bridge.provide("begin_cam_mode",    begin_cam_mode)
Bridge.provide("end_cam_mode",      end_cam_mode)
def log_bridge(msg: str) -> str:
    print(f"\n[UNO Q LCD / TRACE] ➔ {msg}\n")
    return "OK"

Bridge.provide("get_admin_finger",    get_admin_finger)
Bridge.provide("set_admin_finger",    set_admin_finger)
Bridge.provide("get_voter_finger",    get_voter_finger)
Bridge.provide("set_voter_finger",    set_voter_finger)
Bridge.provide("log_bridge",          log_bridge)
Bridge.provide("delete_admin_face",   delete_admin_face)

def check_voted_r3(uid: str) -> str:
    """Drains queue directly so NOT_VOTED returns in ~100ms instead of 5s timeout."""
    send_result = _r3_send(f"CHECK_VOTED:{uid}")
    if send_result != "OK": return "ERROR"
    STOP_TOKENS = {"VOTED", "NOT_VOTED", "FAIL", "ERROR", "TIMEOUT"}
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            token = r3_response_q.get(timeout=min(0.5, deadline - time.time()))
            if token in STOP_TOKENS:
                return token  # Returns "VOTED" or "NOT_VOTED" immediately on receipt
        except _queue_mod.Empty:
            pass
    return "ERROR"
Bridge.provide("check_voted_r3", check_voted_r3)

print("=" * 62)
print("  TRI-FACTOR BIOMETRIC SYSTEM BRIDGE  —  Starting Up")
print("  WITH FULL HARDWARE WATCHDOG (Cam, FP, R3)")
print("=" * 62)

bridge_boot_status = "Python Init"
def _warmup():
    global bridge_boot_status
    try:
        bridge_boot_status = "Loading Face Model"
        from deepface import DeepFace
        DeepFace.build_model("Facenet512")
        print("[BRIDGE] Facenet512 warm and ready.")
        bridge_boot_status = "System Ready"
    except Exception as e: 
        bridge_boot_status = "Model Boot Error"
threading.Thread(target=_warmup, daemon=True).start()

Bridge.provide("get_boot_status", lambda: bridge_boot_status)

def loop(): time.sleep(0.01)
App.run(user_loop=loop)
