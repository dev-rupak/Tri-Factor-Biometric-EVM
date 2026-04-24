"""
face_recognition.py
────────────────────
Face recognition module with Hardware Watchdog.
Gives the user up to 3 blink-trials before returning "DENIED".
Will pause and wait if the camera is unplugged.

API:
    recognize_user(user_id, status_cb=None, cancel_event=None) -> str
    Returns: "GRANTED" | "DENIED" | "ERROR"
"""

import cv2
import mediapipe as mp
import time, os, json, math, threading, concurrent.futures
import numpy as np
import warnings
from collections import deque
from deepface import DeepFace

warnings.filterwarnings("ignore")

# ── Config ──────────────────────────────────────────────────────
USER_DB           = "User_Database"
GLOBAL_THRESHOLD  = 0.40
BLINK_EAR         = 0.28
BLINK_TIMEOUT_S   = 10.0   # Per-trial budget: each trial ≤ 10s
MAX_FACE_TRIALS   = 3

FRAUD_THRESHOLD   = 0.40
FRAUD_MARGIN      = 0.08
AMBIGUITY_MARGIN  = 0.05

ANTISPOOF_THRESH  = 20.0   # static fallback if calibration finds no face
ANTISPOOF_FLOOR   = 8.0    # always reject below this (printed photo = <5)
ANTISPOOF_CAP     = 50.0   # ceiling so bright rooms don't over-tighten
# CAM_W/H, cam_utils.CAM_FAILURE_THRESHOLD, cam_utils.EMBED_TIMEOUT_S → cam_utils (shared with enrollment)

import cam_utils

def _spoof_check(frame, threshold=None):
    """Returns (laplacian_var, is_real). threshold overrides ANTISPOOF_THRESH."""
    if threshold is None: threshold = ANTISPOOF_THRESH
    if frame is None or frame.size == 0: return 0.0, False
    var = cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    return var, var >= threshold

def _load_user(uid):
    vecs, names = [], []
    thr = GLOBAL_THRESHOLD
    import sqlite3
    try:
        with sqlite3.connect(os.path.join(USER_DB, "vote_ledger.db")) as conn:
            res = conn.execute("SELECT threshold, vectors, names FROM faces WHERE uid = ?", (str(uid),)).fetchone()
            if res:
                thr = float(res[0])
                import json
                try:
                    vec_data = json.loads(res[1].decode('utf-8'))
                    name_data = json.loads(res[2].decode('utf-8'))
                    
                    if not isinstance(vec_data, list) or len(vec_data) == 0:
                        print(f"[RECOG] Invalid vector data for {uid}")
                        return [], [], thr
                    if not isinstance(name_data, list) or len(name_data) != len(vec_data):
                        print(f"[RECOG] Vector/name count mismatch for {uid}")
                        return [], [], thr
                    
                    for v in vec_data:
                        vecs.append(cam_utils.l2_normalize(np.array(v)))
                    for n in name_data:
                        names.append(n)
                except ValueError as e:
                    print(f"[RECOG] Normalization error for {uid}: {e}")
                    return [], [], thr
                except Exception as e:
                    print(f"[RECOG] load vectors err: {e}")
    except Exception as e: print(f"[RECOG] load_user sql err: {e}")
    return vecs, names, thr

def _load_all_others(claimed_uid):
    others = {}
    import sqlite3
    try:
        with sqlite3.connect(os.path.join(USER_DB, "vote_ledger.db")) as conn:
            rows = conn.execute("SELECT uid, threshold, vectors FROM faces WHERE uid != ?", (str(claimed_uid),)).fetchall()
            for r_uid, r_thr, r_vecs in rows:
                try:
                    import json
                    vec_data = json.loads(r_vecs.decode('utf-8'))
                    if not isinstance(vec_data, list) or len(vec_data) == 0:
                        raise ValueError(f"Invalid vector data for {r_uid}")
                    vecs = [cam_utils.l2_normalize(np.array(v)) for v in vec_data]
                    others[r_uid] = {"vecs": vecs, "thr": float(r_thr)}
                except Exception as e:
                    print(f"[RECOG] load others err ({r_uid}): {e}")
    except Exception as e:
        print(f"[RECOG] load_all_others sql err: {e}")
    return others

# ══════════════════════════════════════════════════════════════
def _fast_sharpness(frame):
    """Laplacian on a tiny thumbnail — ~64x faster than full-res crop on ARM (Issue #11).
    Good enough for ranking 10 frames relative to each other; absolute score not needed."""
    thumb = cv2.resize(frame, (80, 60), interpolation=cv2.INTER_NEAREST)
    return cv2.Laplacian(cv2.cvtColor(thumb, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()

def recognize_user(user_id, status_cb=None, cancel_event=None):
    if cancel_event is None: cancel_event = threading.Event()

    def st(l1, l2="", beep=None):
        l1, l2 = l1[:16], l2[:16]
        if status_cb: status_cb(l1, l2, beep)
        print(f"  [LCD] {l1} | {l2}")

    if not os.path.exists(os.path.join(USER_DB, user_id)):
        st("User NOT FOUND", user_id[:16])
        return "DENIED"

    master_vecs = []; master_names = []; thr_box = [GLOBAL_THRESHOLD]
    other_users = {}   
    data_ev = threading.Event()

    def _load():
        v, n, t = _load_user(user_id)
        master_vecs.extend(v)
        master_names.extend(n)
        thr_box[0] = t
        other_users.update(_load_all_others(user_id))
        data_ev.set()

    threading.Thread(target=_load, daemon=True).start()

    st("Biometric Check", "Please Wait...")
    cap, _cam_owned = cam_utils.acquire_cam(st, cancel_event=cancel_event)
    if cap is None:
        st("Camera FAIL!", "No camera found")
        return "ERROR"

    fm = cam_utils.create_face_mesh()  # shared config from cam_utils
    for _ in range(5): cap.grab()

    antispoof_thr = ANTISPOOF_THRESH

    try:
        for trial in range(1, MAX_FACE_TRIALS + 1):
            if cancel_event.is_set(): st("Cancelled", ""); return "DENIED"

            st(f"Face Check {trial}/{MAX_FACE_TRIALS}", "Blink your eyes")
            buf = deque(maxlen=10)
            frames_to_process = []
            deadline  = time.time() + BLINK_TIMEOUT_S
            cam_failures = 0

            while True:
                if cancel_event.is_set(): st("Cancelled", ""); return "DENIED"

                ret, frame = cap.read()
                
                # ── WATCHDOG: Camera Disconnect ──
                if not ret: 
                    cam_failures += 1
                    time.sleep(0.1)
                    if cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                        st("Camera Lost!", "Reconnect it")
                        print("[WATCHDOG] Camera disconnected. Attempting to reopen...")
                        cap.release()
                        pause_start = time.time()
                        while cam_failures > cam_utils.CAM_FAILURE_THRESHOLD:
                            if cancel_event.is_set(): return "DENIED"
                            time.sleep(1)
                            new_cap, new_owned = cam_utils.acquire_cam(cancel_event=cancel_event)
                            if new_cap is not None:
                                cap = new_cap
                                _cam_owned = new_owned
                                cam_utils.set_shared_cam(new_cap)   # keep shared state current
                                cam_failures = 0
                                pause_duration = time.time() - pause_start
                                st("Camera Back!", "Continuing...")
                                deadline += pause_duration  # Push deadline forward
                                break
                    continue
                cam_failures = 0
                
                # Check timeout only if camera is functioning
                if time.time() > deadline:
                    st(f"Try {trial} timeout", ""); break

                # Downscale to 640×480 for MediaPipe — 3× faster on Uno Q ARM.
                # Full 720p frame is kept in buf for high-quality Facenet512 input.
                res = fm.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                if not res.multi_face_landmarks:
                    st("No Face Found", "Move closer")
                    time.sleep(0.1)
                    continue
                
                if len(res.multi_face_landmarks) > 1:
                    st("Multiple Faces", "Only 1 allowed", beep="LONG")
                    time.sleep(2.0)
                    continue

                for lm in res.multi_face_landmarks:
                    h, w, _ = frame.shape
                    
                    # Prevent out-of-bounds faces from entering the blink buffer
                    if cam_utils.crop_face(frame, lm.landmark) is None:
                        st("Move back!", "Too close/edge")
                        continue

                    ev = (cam_utils.calc_ear([33,160,158,133,153,144], lm.landmark, w, h) +
                          cam_utils.calc_ear([362,385,387,263,373,380], lm.landmark, w, h)) / 2
                    # Pre-score sharpness per frame so blink-time sort is O(n) not O(n × Laplacian)
                    sharp = _fast_sharpness(frame)
                    buf.append((frame.copy(), list(lm.landmark), ev, sharp))
                    rdy = "READY" if data_ev.is_set() else "Loading"
                    st("Processing...", "Keep Looking")

                    if ev < BLINK_EAR and len(buf) >= 4:
                        # Sort by pre-stored sharpness score — zero extra computation at blink time
                        buf_sorted = sorted(buf, key=lambda x: x[3], reverse=True)
                        frames_to_process = [
                            buf_sorted[0][0:2],
                            buf_sorted[1][0:2],
                            buf_sorted[2][0:2]
                        ]
                        break

                if frames_to_process: break

            if not frames_to_process:
                continue   

            if not data_ev.is_set():
                st("Loading data", "Please wait")
                if not data_ev.wait(timeout=15.0):
                    st("Load error", "Timeout")
                    return "ERROR"
            if not master_vecs:
                st("No face data!", "Re-enroll")
                return "ERROR"

            valid_frames = []
            too_dark = False
            for fi, (f, lms) in enumerate(frames_to_process):
                cropped = cam_utils.crop_face(f, lms)
                lap, real = _spoof_check(cropped, antispoof_thr)
                if lap < 15.0:
                    too_dark = True
                elif real:
                    valid_frames.append((f, lms))
                    print(f"[RECOG] Frame {fi} OK (Laplacian={lap:.1f})")
                else:
                    print(f"[RECOG] Frame {fi} REJECTED - too blurry (Laplacian={lap:.1f})")
            
            if too_dark:
                st("Face Too Dark", "Turn on light", beep="LONG")
                print(f"[RECOG] Frames rejected due to low light (Laplacian < 15.0)")
                time.sleep(2.0)
                continue

            if len(valid_frames) < 1:  # Need at least 1 good frame
                st("Face Blurry", "Trying again...", beep="LONG")
                print(f"[RECOG] Only {len(valid_frames)}/3 frames passed sharpness check")
                time.sleep(2.0)
                continue
            
            frames_to_process = valid_frames

            st("Analyzing Face", "Hold steady...")
            try:
                vecs = []
                for i, (f, lms) in enumerate(frames_to_process):
                    tmp = None
                    try:
                        crp = cam_utils.crop_face(f, lms)
                        if crp is None:
                            print(f"[RECOG] Frame {i} skipped — face too close to frame edge")
                            continue
                        tmp = f"/tmp/auth_{user_id}_{i}_{int(time.time()*1000)}.jpg"
                        if not cv2.imwrite(tmp, crp):
                            raise IOError(f"Failed to write {tmp}")
                        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                            fut = pool.submit(cam_utils.embed_image, tmp)
                            try:
                                v = fut.result(timeout=cam_utils.EMBED_TIMEOUT_S)
                                vecs.append(np.array(v))
                            except concurrent.futures.TimeoutError:
                                print(f"[RECOG] Frame {i} embed timed out after {cam_utils.EMBED_TIMEOUT_S}s — skipping")
                    except Exception as e:
                        print(f"[RECOG] Error embedding frame {i}: {e}")
                    finally:
                        if tmp and os.path.exists(tmp): os.remove(tmp)

                        
                if not vecs:
                    raise ValueError("Failed to extract embeddings")
                    
                lv = cam_utils.l2_normalize(np.mean(vecs, axis=0))
                dists = [(1.0 - np.dot(mv, lv)) for mv in master_vecs]
                best  = min(dists)
                pose  = master_names[dists.index(best)]
                # Use the database threshold (which already has an enrollment margin).
                # Clamp to an absolute ceiling of 0.45 to patch old, permissive database entries.
                thr = round(min(thr_box[0], 0.45), 4)

                print(f"[RECOG] trial={trial} dist={best:.3f} ({pose}) thr={thr} (base={thr_box[0]:.4f}) | all_dists: {dict(zip(master_names, [round(d,3) for d in dists]))}")

                impostor_uid = None
                ambiguous = False
                for other_uid, other_data in other_users.items():
                    other_vecs = other_data["vecs"]
                    other_best = min((1.0 - np.dot(v, lv)) for v in other_vecs)
                    print(f"[RECOG] cross-check vs {other_uid}: dist={other_best:.3f}")
                    
                    if other_best < FRAUD_THRESHOLD and other_best < best:
                        impostor_uid = other_uid
                        print(f"[RECOG] FRAUD: Other user {other_uid} matches better ({other_best:.3f} vs {best:.3f})")
                        break
                    if best < thr and other_best < FRAUD_THRESHOLD and abs(best - other_best) < AMBIGUITY_MARGIN:
                        ambiguous = True

                if impostor_uid:
                    st("IDENTITY FRAUD!", f"Matches ID:{impostor_uid}", beep="LONG")
                    print(f"[RECOG] BLOCKED: face matches user {impostor_uid}")
                    time.sleep(2.0)
                    continue 
                    
                if ambiguous:
                    st("AMBIGUOUS FACE", "Move/Try again", beep="LONG")
                    print(f"[RECOG] AMBIGUOUS: Margin to another user is < 0.05. Forcing retry.")
                    time.sleep(2.0)
                    if trial == MAX_FACE_TRIALS:
                        st("Auth Rejected", "Too Ambiguous", beep="LONG")
                        time.sleep(2.5)
                        return "DENIED"
                    continue

                if best < thr:
                    st("Match Found!", "Identity OK", beep="DOUBLE")
                    return "GRANTED"
                else:
                    st("No Match Found", "Retry Face Scan", beep="LONG")
                    time.sleep(2.0) 

            except Exception as e:
                import traceback; traceback.print_exc()
                st("Embed Error", str(e)[:16])
                return "ERROR"

        st("Face Denied", "Officer Needed", beep="LONG")
        time.sleep(2.5)
        return "DENIED"

    except Exception as e:
        import traceback; traceback.print_exc()
        st("ERROR", str(e)[:16])
        return "ERROR"
    finally:
        cam_utils.release_cam(cap, _cam_owned)


# ══════════════════════════════════════════════════════════════
def _verify_imposter(cropped_frame, master_vecs, auth_thr):
    """Runs asynchronously to check if the face matches the master vectors. Returns True if imposter."""
    import tempfile, os
    import numpy as np
    import cam_utils
    
    fd, path = tempfile.mkstemp(suffix=".jpg")
    os.close(fd)
    try:
        cv2.imwrite(path, cropped_frame)
        vec = cam_utils.embed_image(path)
        vec = cam_utils.l2_normalize(vec)
        
        best_dist = float('inf')
        for mv in master_vecs:
            dist = 1.0 - np.dot(mv, vec)
            if dist < best_dist:
                best_dist = dist
        
        if best_dist > auth_thr:
            print(f"[MONITOR-AUTH] Imposter detected! dist={best_dist:.3f} > {auth_thr:.3f}")
            return True
        else:
            print(f"[MONITOR-AUTH] Voter verified (dist={best_dist:.3f})")
            return False
    except Exception as e:
        print(f"[MONITOR-AUTH] Background auth error: {e}")
        return False
    finally:
        try: os.remove(path)
        except: pass

def continuous_monitoring(user_id, lock_callback, status_callback, cancel_event):
    """
    Runs in parallel with the active voting window.
    1. Detects MULTI_PERSON instantly at 30 FPS.
    2. Runs background identity checks using an asynchronous worker.
    """
    CONSECUTIVE_THRESHOLD = 60

    # Load master vectors for the background authentication
    master_vecs, _, base_thr = _load_user(user_id)
    # Add a margin to the threshold because angles during voting are not ideal
    auth_thr = base_thr + FRAUD_MARGIN + 0.05

    cap, _cam_owned = None, True
    for _ in range(5):
        cap, _cam_owned = cam_utils.acquire_cam(None, cancel_event=cancel_event)
        if cap:
            break
        time.sleep(0.5)

    if not cap:
        print("[MONITOR] Camera unavailable — monitoring disabled for this session")
        return

    fm = mp.solutions.face_mesh.FaceMesh(
        max_num_faces=3,
        refine_landmarks=True,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.5
    )

    violation_buffer = []
    
    # State for asynchronous background authentication
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    auth_future = None
    imposter_violations = 0

    try:
        while not cancel_event.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            res = fm.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if res.multi_face_landmarks:
                num_faces = len(res.multi_face_landmarks)
                if num_faces > 1:
                    violation_buffer.append("MULTI_PERSON")
                    face_msg = f"{num_faces} People!"
                    if len(violation_buffer) % 15 == 0:
                        status_callback(face_msg[:16], "Area monitored", "LONG")
                        print(f"[MONITOR] {num_faces} faces detected — buffer length: {len(violation_buffer)}")
                else:
                    violation_buffer.clear()
                    
                    # Exact 1 face found: if background worker is idle, submit a verification job
                    if (auth_future is None or auth_future.done()) and len(master_vecs) > 0:
                        crp = cam_utils.crop_face(frame, res.multi_face_landmarks[0].landmark)
                        if crp is not None:
                            auth_future = executor.submit(_verify_imposter, crp, master_vecs, auth_thr)
            else:
                violation_buffer.clear()

            # Handle MULTI_PERSON lock
            if len(violation_buffer) >= CONSECUTIVE_THRESHOLD:
                reason = "MULTI_PERSON"
                print(f"[MONITOR] LOCK TRIGGERED: {reason}")
                violation_buffer.clear()
                lock_callback(reason)
                break
                
            # Handle Background Authentication result
            if auth_future is not None and auth_future.done():
                try:
                    is_imposter = auth_future.result()
                    if is_imposter:
                        imposter_violations += 1
                        if imposter_violations >= 2:
                            print(f"[MONITOR] LOCK TRIGGERED: IDENTITY_FRAUD")
                            lock_callback("IDENTITY_FRAUD")
                            break
                    else:
                        imposter_violations = 0
                except Exception as e:
                    print(f"[MONITOR] Future error: {e}")
                auth_future = None

            time.sleep(0.02)
    except Exception as e:
        print(f"[MONITOR] Unexpected error: {e}")
    finally:
        executor.shutdown(wait=False)
        cam_utils.release_cam(cap, _cam_owned)
        print("[MONITOR] Monitoring thread exited")