import cv2
import math
import numpy as np
import time
from deepface import DeepFace

import mediapipe as mp

CAM_W, CAM_H           = 640, 480
CAM_FAILURE_THRESHOLD  = 15    # consecutive read failures before watchdog triggers
EMBED_TIMEOUT_S        = 8.0   # max seconds per Facenet512 inference on Pi

# ── Shared camera lifecycle ─────────────────────────────────────────────
# Pre-warmed by begin_cam_mode() from main.py; released by end_cam_mode().
# acquire_cam() returns the shared cap if available, otherwise opens a new one.
# release_cam(cap, owned) only releases if the caller opened it themselves.
_shared_cap_lock = __import__('threading').Lock()
_shared_cap = None

def set_shared_cam(cap):
    """Called by begin_cam_mode / end_cam_mode in main.py."""
    global _shared_cap
    with _shared_cap_lock:
        _shared_cap = cap

def acquire_cam(st_fn=None, cancel_event=None):
    """Returns (cap, owned). owned=False means the caller must NOT release it."""
    with _shared_cap_lock:
        if _shared_cap is not None and _shared_cap.isOpened():
            print("[CAM_UTILS] Reusing pre-warmed shared camera.")
            return _shared_cap, False   # caller must not release
    cap = open_cam(st_fn, cancel_event=cancel_event)
    return cap, True                    # caller owns it, must release

def release_cam(cap, owned: bool):
    """Releases cap only if owned=True (caller opened it, not the shared one)."""
    if owned and cap is not None:
        try: cap.release()
        except: pass

def create_face_mesh():
    """
    Single source of truth for FaceMesh config used by both
    face_enrollment.py and face_recognition.py.
    Tune detection distance / sensitivity here only.
    """
    return mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=False,       # iris model not needed; EAR uses basic landmarks
        min_detection_confidence=0.2, # detects faces at 1in–4ft in 640×480
        min_tracking_confidence=0.2
    )

def find_cam():
    try:
        import subprocess
        out = subprocess.check_output(
            ['v4l2-ctl', '--list-devices'], text=True, stderr=subprocess.DEVNULL)
        for block in out.split('\n\n'):
            if 'USB' in block or 'FINGERS' in block:
                for line in block.split('\n'):
                    if '/dev/video' in line:
                        return int(line.strip().replace('/dev/video', ''))
    except Exception: pass
    return 0

def open_cam(st_fn=None, cancel_event=None):
    attempt = 1
    while True:
        if cancel_event and cancel_event.is_set():
            print("[CAM_UTILS] Camera search cancelled.")
            return None
            
        idx = find_cam()
        for _ in range(3):
            cap = cv2.VideoCapture(idx)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            if cap.isOpened():
                for _ in range(5):
                    ret, _ = cap.read()
                    if ret:
                        print(f"[CAM_UTILS] Camera ok on index {idx}")
                        return cap
                    time.sleep(0.2)
            cap.release()
            if cancel_event and cancel_event.is_set(): return None
            if st_fn: st_fn(f"Cam missing", f"Retry {attempt}")
            time.sleep(1)
            
        for i in range(4):
            if i == idx: continue
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
                ret, _ = cap.read()
                if ret:
                    print(f"[CAM_UTILS] Camera fallback on index {i}")
                    return cap
            cap.release()
            
        print(f"[CAM_UTILS] Camera not found, waiting 5s (retry {attempt})...")
        if st_fn: st_fn(f"Cam missing", f"Wait 5s... ({attempt})")
        
        # Sleep in small increments to check cancel_event
        for _ in range(10):
            if cancel_event and cancel_event.is_set(): return None
            time.sleep(0.5)
            
        attempt += 1

def crop_face(frame, lms):
    h, w, _ = frame.shape
    xs = [int(lm.x * w) for lm in lms]
    ys = [int(lm.y * h) for lm in lms]
    x1, x2 = max(0, min(xs)), min(w, max(xs))
    y1, y2 = max(0, min(ys)), min(h, max(ys))
    
    fw = x2 - x1
    fh = y2 - y1
    size = max(fw, fh)
    
    pad = int(size * 0.15)
    padded_size = size + (2 * pad)
    
    cx = x1 + (fw // 2)
    cy = y1 + (fh // 2)
    
    ideal_left   = cx - (padded_size // 2)
    ideal_right  = ideal_left + padded_size
    ideal_top    = cy - (padded_size // 2)
    ideal_bottom = ideal_top + padded_size

    # Clip guard: if face is pushed too far to the edge, skip
    clip_x_tol = int((x2 - x1) * 0.20)
    clip_y_tol = int((y2 - y1) * 0.20)
    if ((-ideal_top > clip_y_tol) or (ideal_bottom - h > clip_y_tol) or
            (-ideal_left > clip_x_tol) or (ideal_right - w > clip_x_tol)):
        return None

    # Calculate padding needed if bounding box is outside frame
    pad_top = max(0, -ideal_top)
    pad_bottom = max(0, ideal_bottom - h)
    pad_left = max(0, -ideal_left)
    pad_right = max(0, ideal_right - w)

    # Calculate safe crop coordinates
    top = max(0, ideal_top)
    bottom = min(h, ideal_bottom)
    left = max(0, ideal_left)
    right = min(w, ideal_right)

    c = frame[top:bottom, left:right]
    
    if c.size == 0: 
        return None

    # Apply padding to keep face perfectly centered (REPLICATE prevents artificial sharpness spikes)
    if pad_top > 0 or pad_bottom > 0 or pad_left > 0 or pad_right > 0:
        c = cv2.copyMakeBorder(c, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_REPLICATE)

    # ── Lighting normalisation (CLAHE on L channel) ──────────────
    # Equalises brightness/contrast in LAB space so Facenet512 embeddings are
    # stable across different room lighting conditions.  Applied at both
    # enroll-time and recognition-time so distances stay comparable.
    try:
        lab  = cv2.cvtColor(c, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        lab   = cv2.merge([clahe.apply(l), a, b])
        c     = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    except Exception:
        pass  # if CLAHE fails for any reason, return the raw crop

    return c




def calc_ear(idx, lms, w, h):
    p = [(int(lms[i].x*w), int(lms[i].y*h)) for i in idx]
    d = lambda a, b: math.hypot(b[0]-a[0], b[1]-a[1])
    hz = d(p[0], p[3])
    return 0.0 if hz == 0 else (d(p[1],p[5]) + d(p[2],p[4])) / (2*hz)

def l2_normalize(v):
    n = np.sqrt(np.dot(v, v))
    if n < 1e-10:
        raise ValueError("Cannot normalize zero or near-zero vector")
    return v/n

def embed_image(path):
    # detector_backend="skip" — MediaPipe's 468-landmark crop already gives a clean
    # face region. DeepFace embeds it directly without re-running detection,
    # which is 2-3x faster on the Uno Q ARM and works on all pose angles.
    result = DeepFace.represent(
        img_path=path, model_name="Facenet512",
        detector_backend="skip", enforce_detection=False
    )
    return np.array(result[0]['embedding'])
