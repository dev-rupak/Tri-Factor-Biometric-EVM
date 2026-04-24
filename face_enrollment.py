"""
face_enrollment.py
──────────────────
Face enrollment module with Hardware Watchdog.  
Pauses and waits if the camera drops during enrollment.
"""

import cv2
import mediapipe as mp
import time, os, math, json, shutil, threading, concurrent.futures
import numpy as np
import warnings
from collections import deque
from deepface import DeepFace

warnings.filterwarnings("ignore")

# ── Config ──────────────────────────────────────────────────────
USER_DB           = "User_Database"
TOTAL_FRAMES      = 50
IMAGES_TO_AVERAGE = 5
CAL_SAMPLES       = 5
CAL_MULTIPLIER    = 3.0
CAL_MAX_CAP       = 0.45   # Strict Cosine Distance ceiling for Facenet512
CAL_MIN_FLOOR     = 0.35   
FRAUD_CHECK_FRAMES = 7     
GRACE             = 10.0
BLINK_EAR         = 0.28
BLINK_TIMEOUT_S   = 120.0  # opencv backend is 2-3x slower than skip on ARM — needs more time
# CAM_W/H, cam_utils.CAM_FAILURE_THRESHOLD, EMBED_TIMEOUT_S → cam_utils (shared with recognition)

PHASES = [
    (17, "STRAIGHT", "Look Straight"),
    (34, "LEFT",     "Turn Left"),
    (50, "RIGHT",    "Turn Right"),
]
POSE_BLOCKS = [
    ("straight",  0,  16),
    ("left",     17,  33),
    ("right",    34,  49),
]

os.makedirs(USER_DB, exist_ok=True)

# ── Camera ──────────────────────────────────────────────────────
import cam_utils

def _pose(lms, w, h, target="STRAIGHT"):
    yaw = (lms[1].x*w - lms[234].x*w) / (lms[454].x*w - lms[1].x*w + 1e-6)
    
    # Hysteresis: We widen the target's acceptance window to make it extremely stable
    if target == "STRAIGHT":
        if yaw > 1.25: return "LEFT"
        if yaw < 0.75: return "RIGHT"
        return "STRAIGHT"
    elif target == "LEFT":
        if yaw > 1.10: return "LEFT"  # Very forgiving left turn
        return "STRAIGHT"
    elif target == "RIGHT":
        if yaw < 0.90: return "RIGHT" # Very forgiving right turn
        return "STRAIGHT"
        
    return "STRAIGHT"

def _flush(cap, n=10):
    for _ in range(n): cap.grab()

def _sharpness(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None: return 0.0
    return cv2.Laplacian(img, cv2.CV_64F).var()

def _best_frames(folder, user_id, start, end, n):
    scored = []
    for i in range(start, end + 1):
        p = f"{folder}/{user_id}_{i}.jpg"
        if os.path.exists(p):
            scored.append((_sharpness(p), p))
    scored.sort(reverse=True)
    if not scored: return []
    return [p for _, p in scored[:n]]

EMBED_TIMEOUT_S = cam_utils.EMBED_TIMEOUT_S  # from cam_utils shared config

def _avg_vec(paths):
    if not paths: raise ValueError("No paths provided for averaging")
    vecs = []
    for p in paths:
        if not os.path.exists(p): continue
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                fut = pool.submit(cam_utils.embed_image, p)
                try:
                    vecs.append(fut.result(timeout=EMBED_TIMEOUT_S))
                except concurrent.futures.TimeoutError:
                    print(f"[ENROLL] embed timeout ({EMBED_TIMEOUT_S}s) for {p} — skipping")
        except Exception as e: print(f"[ENROLL] embed err {p}: {e}")
    if not vecs: raise ValueError("Extracted vectors are empty")
    return cam_utils.l2_normalize(np.mean(vecs, axis=0))

# ══════════════════════════════════════════════════════════════
def enroll_user(user_id, status_cb=None, cancel_event=None,
                capture_done_cb=None, fp_done_event=None):
    if cancel_event is None: cancel_event = threading.Event()
    if fp_done_event is None: fp_done_event = threading.Event()

    folder = os.path.join(USER_DB, user_id)
    cap = None
    _cam_owned = True  # safe default: release_cam() will own-release if acquire fails mid-path

    def st(l1, l2="", beep=None):
        l1,l2 = l1[:16], l2[:16]
        if status_cb: status_cb(l1, l2, beep)
        print(f"  [LCD] {l1} | {l2}")

    def cancelled(): return cancel_event.is_set()

    if os.path.exists(folder):
        st("ID EXISTS!", "Delete first")
        return False

    os.makedirs(folder, exist_ok=True)
    success = False

    try:
        model_ready = threading.Event()
        def _load():
            try: DeepFace.build_model("Facenet512")
            except Exception as e: print(f"[ENROLL] model load err: {e}")
            finally: model_ready.set()
        threading.Thread(target=_load, daemon=True).start()
        st("Startup", "Please Wait...")

        st("Opening camera", "")
        cap, _cam_owned = cam_utils.acquire_cam(st, cancel_event=cancel_event)
        if cap is None:
            st("Camera FAIL!", "No camera found")
            return False

        fm = cam_utils.create_face_mesh()  # shared config from cam_utils
        existing = [u for u in os.listdir(USER_DB) if u != user_id and os.path.isdir(os.path.join(USER_DB, u))]

        # ── PHASE 1-3: Capture 50 frames ─────────────────────────
        count=0; active_phase=None; bad_start=None; no_face_streak=0
        cam_failures = 0
        st("Ready", "Look at Camera")
        time.sleep(2); _flush(cap)

        while count < TOTAL_FRAMES:
            if cancelled(): st("Cancelled",""); return False

            target = ""
            inst = ""
            for lim,pose,ins in PHASES:
                if count < lim: target,inst = pose,ins; break

            if target != active_phase:
                active_phase=target; bad_start=None; no_face_streak=0
                n = ["STRAIGHT","LEFT","RIGHT"].index(target)+1
                st(f"Step {n}/3", inst)        # e.g. "Step 1/3" | "Look Straight"
                _flush(cap); time.sleep(1); continue

            ret,frame = cap.read()

            # ── WATCHDOG: Camera Disconnect ──
            if not ret:
                cam_failures += 1
                time.sleep(0.1)
                if cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                    st("Camera Lost!", "Reconnect it")
                    print("[WATCHDOG] Camera disconnected. Attempting to reopen...")
                    cap.release()
                    cam_utils.set_shared_cam(None)   # clear stale shared ref before re-acquiring
                    pause_start = time.time()
                    while cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                        if cancelled(): return False
                        time.sleep(1)
                        new_cap, new_owned = cam_utils.acquire_cam(cancel_event=cancel_event)
                        if new_cap is not None:
                            cap = new_cap
                            _cam_owned = new_owned
                            cam_failures = 0
                            pause_duration = time.time() - pause_start
                            st("Camera Back!", "Continuing...")
                            if bad_start: bad_start += pause_duration
                            break
                continue
            cam_failures = 0

            # Flush stale V4L2 buffer frames before processing
            for _ in range(2): cap.grab()

            res = fm.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if not res.multi_face_landmarks:
                no_face_streak += 1
                if no_face_streak >= 5:
                    if bad_start is None: bad_start = time.time()
                    left = GRACE - (time.time() - bad_start)
                    if left <= 0: st("No face!","Timeout"); return False
                    st(f"No face {left:.0f}s","Move closer")
                time.sleep(0.1)
                continue
            
            if len(res.multi_face_landmarks) > 1:
                st("Multiple Faces", "Only 1 allowed", beep="LONG")
                no_face_streak = 0
                time.sleep(2.0)
                continue

            no_face_streak = 0
            for lm in res.multi_face_landmarks:
                h, w, _ = frame.shape
                p = _pose(lm.landmark, w, h, target)
                ok = (p == target)
                if not ok:
                    if bad_start is None: bad_start = time.time()
                    left = GRACE - (time.time() - bad_start)
                    if left <= 0: st("Hold pose!","Timeout"); return False
                    st(f"Hold: {inst}", f"{left:.0f}s left")
                else:
                    bad_start = None
                    crp = cam_utils.crop_face(frame, lm.landmark)
                    if crp is None:
                        st("Move back!", "Too close/edge")
                        time.sleep(0.1)
                    else:
                        st(f"Frame {count+1}/{TOTAL_FRAMES}", target)
                        cv2.imwrite(f"{folder}/{user_id}_{count}.jpg", crp)
                        count += 1; time.sleep(0.03)



        st("Capture OK", "Checking...")

        # ── Anti-fraud check ──────────────────────────────────────
        is_fraud = False
        fraud_paths = (
            _best_frames(folder, user_id, 0, 16, 1) +
            _best_frames(folder, user_id, 17, 33, 1) +
            _best_frames(folder, user_id, 34, 49, 1)
        )
        try:
            fraud_vec = _avg_vec(fraud_paths)
        except ValueError:
            fraud_vec = None
        
        if fraud_vec is not None:
            lv = cam_utils.l2_normalize(fraud_vec)
            import sqlite3
            try:
                with sqlite3.connect(os.path.join(USER_DB, "vote_ledger.db")) as conn:
                    with conn:
                        rows = conn.execute("SELECT uid, vectors FROM faces").fetchall()
                        for r_uid, r_vecs in rows:
                            try:
                                vec_list = json.loads(r_vecs.decode('utf-8'))
                                for pm in vec_list:
                                    dist = 1.0 - np.dot(np.array(pm), lv)
                                    if dist < 0.50:  # Strict threshold (higher net) to firmly reject loosely similar faces as fraud.
                                        is_fraud=True; break
                            except Exception: pass
                            if is_fraud: break
            except Exception as e: print(f"[ENROLL] SQLite read err: {e}")
            
        if is_fraud:
            st("Duplicate Face!", "Already enrolled")
            return False

        st("Unique Face OK", "Processing...")

        # ── Background: master vectors ────────────────────────────
        master_meta=[]; master_vecs=[]; math_done=threading.Event(); math_err=[None]
        def _compute():
            try:
                for pname,si,ei in POSE_BLOCKS:
                    if cancelled(): math_err[0]="Cancelled"; return
                    paths = _best_frames(folder, user_id, si, ei, IMAGES_TO_AVERAGE)
                    if not paths:
                        math_err[0] = f"{pname}: no frames"
                        return
                    v=_avg_vec(paths)
                    master_vecs.append(v)
                    master_meta.append((pname,paths,v))
            except Exception as e: math_err[0]=str(e)
            finally: math_done.set()
        threading.Thread(target=_compute,daemon=True).start()

        # ── PHASE 5: Blink calibration ────────────────────────────
        _flush(cap); buf=deque(maxlen=10); cooldown=0; blink_frames=[]; n=0
        st("Final Step", "Blink slowly")
        deadline=time.time()+BLINK_TIMEOUT_S
        cam_failures = 0

        while n < CAL_SAMPLES:
            if cancelled(): st("Cancelled",""); return False
            if time.time()>deadline: st("Blink timeout!",""); return False
            
            ret,frame=cap.read()
            
            # ── WATCHDOG: Camera Disconnect ──
            if not ret: 
                cam_failures += 1
                time.sleep(0.1)
                if cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                    st("Camera Lost!", "Reconnect it")
                    cap.release()
                    cam_utils.set_shared_cam(None)   # clear stale shared ref before re-acquiring
                    pause_start = time.time()
                    while cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                        if cancelled(): return False
                        time.sleep(1)
                        new_cap, new_owned = cam_utils.acquire_cam(cancel_event=cancel_event)
                        if new_cap is not None:
                            cap = new_cap
                            _cam_owned = new_owned
                            cam_failures = 0
                            pause_duration = time.time() - pause_start
                            st("Camera Back!", "Continuing...")
                            deadline += pause_duration
                            break
                continue
            cam_failures = 0
            
            if cooldown>0: cooldown-=1; continue
            res=fm.process(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            if not res.multi_face_landmarks: continue
            for lm in res.multi_face_landmarks:
                h,w,_=frame.shape
                ev=(cam_utils.calc_ear([33,160,158,133,153,144],lm.landmark,w,h)+
                    cam_utils.calc_ear([362,385,387,263,373,380],lm.landmark,w,h))/2
                buf.append((frame.copy(),list(lm.landmark), ev))
                bg="math done" if math_done.is_set() else "computing"
                st(f"Blink {n+1}/{CAL_SAMPLES}", "Keep Going")
                if ev < BLINK_EAR and len(buf) >= 5:
                    buf_list = list(buf)
                    offset = min(6, len(buf_list) - 1)
                    pf = buf_list[-(offset + 1)][0]
                    pl = buf_list[-(offset + 1)][1]
                            
                    crp = cam_utils.crop_face(pf, pl)
                    if crp is None:
                        st(f"Blink {n+1}/{CAL_SAMPLES}", "Move back!")
                        cooldown = 30; buf.clear(); break
                    blink_frames.append(crp)
                    n += 1; st(f"Blink {n}/{CAL_SAMPLES}", "Captured!")
                    cooldown = 30; buf.clear(); break

        cap.release(); cap=None
        if capture_done_cb: capture_done_cb()

        st("Scan finger now", "Please wait...")
        fp_ok = fp_done_event.wait(timeout=120.0)
        if not fp_ok:
            st("No Finger!", "Timed out")
            if cancel_event: cancel_event.set()
            return False
        if cancelled():
            st("Cancelled",""); return False

        if not math_done.is_set():
            st("Finishing...", "Please wait")
            math_done.wait(timeout=80.0)
        if math_err[0]:
            st("Math Error!",math_err[0][:16]); return False

        master_names = []
        master_vec_list = []
        for pname,paths_used,v in master_meta:
            master_names.append(pname)
            master_vec_list.append(v.tolist())

        distances=[]
        for i, bf in enumerate(blink_frames):
            if bf is None:
                print(f"[ENROLL] blink frame {i} is None (edge crop) — skipping")
                continue
            tmp = f"/tmp/cal_{user_id}_{i}.jpg"
            cv2.imwrite(tmp, bf)
            try:
                with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                    fut = pool.submit(cam_utils.embed_image, tmp)
                    try:
                        lv = cam_utils.l2_normalize(fut.result(timeout=EMBED_TIMEOUT_S))
                        distances.append(min((1.0 - np.dot(v, lv)) for _, _, v in master_meta))
                    except concurrent.futures.TimeoutError:
                        print(f"[ENROLL] blink embed {i} timed out ({EMBED_TIMEOUT_S}s) — skipping")
            except Exception as e:
                print(f"[ENROLL] blink embed err: {e}")
            finally:
                if os.path.exists(tmp): os.remove(tmp)

        if len(distances)<2: thr=CAL_MIN_FLOOR
        else:
            arr=np.array(distances)
            med = float(np.median(arr))
            raw = med + 0.15  # +0.15 margin covers distance/angle variance between enroll and recognition
            thr=round(max(min(raw,CAL_MAX_CAP),CAL_MIN_FLOOR),4)

        import sqlite3
        try:
            with sqlite3.connect(os.path.join(USER_DB, "vote_ledger.db")) as conn:
                with conn:
                    conn.execute(
                        "INSERT OR REPLACE INTO faces (uid, threshold, vectors, names) VALUES (?, ?, ?, ?)",
                        (str(user_id), thr, json.dumps(master_vec_list).encode('utf-8'), json.dumps(master_names).encode('utf-8'))
                    )
        except Exception as e:
            st("Save Error!", str(e)[:16])
            return False

        st("Enroll Success", "Identity Saved")
        success=True; return True

    except Exception as e:
        import traceback; traceback.print_exc()
        st("ERROR",str(e)[:16]); return False

    finally:
        cam_utils.release_cam(cap, _cam_owned)
        if not success and os.path.exists(folder): shutil.rmtree(folder)