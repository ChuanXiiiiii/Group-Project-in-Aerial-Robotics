# vision.py — camera + AI detection
# dual backend: ultralytics YOLO on laptop / TFLite on pi / NCNN on pi-optimised
# interface: detect_in_image(frame) -> (found, x, y, conf)

import os
import cv2
import numpy as np
import sys

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# default camera size when config.py is missing
DEFAULT_CAM_W = 640
DEFAULT_CAM_H = 480

# default conf when config.py is missing
DEFAULT_CONF_THRESHOLD = 0.4

# YOLOv8 input size (all exported models use 640x640)
YOLO_INPUT_SIZE = 640

# values above this are treated as pixel coords (0-640), below as normalised (0-1)
PIXEL_COORD_THRESHOLD = 1.5

# camera warm-up frames to discard (AWB / exposure convergence)
CAMERA_WARMUP_FRAMES = 10

# SAR classes for our custom multi-class model (5 classes)
SAR_NAMES = {0: "dummy", 1: "pants", 2: "tshirt", 3: "backpack", 4: "cone"}

# subset of COCO used by human.tflite and similar multi-class models
# single-class custom models short-circuit this and just report "dummy"
COCO_NAMES = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
    5: "bus", 6: "train", 7: "truck", 8: "boat", 9: "traffic light",
    10: "fire hydrant", 11: "stop sign", 12: "parking meter", 13: "bench",
    14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep", 19: "cow",
    20: "elephant", 24: "backpack", 25: "umbrella", 26: "handbag", 27: "tie",
    28: "suitcase", 39: "bottle", 56: "chair", 57: "couch",
    58: "potted plant", 59: "bed", 60: "dining table", 62: "tv", 63: "laptop",
    64: "mouse", 67: "cell phone",
}

# ---------------------------------------------------------------------------
# Backend availability detection (runs once at import)
# ---------------------------------------------------------------------------

# 1: ultralytics YOLO, richest backend, for dev laptops
YOLO = None
try:
    from ultralytics import YOLO as _YOLO
    YOLO = _YOLO
except ImportError:
    pass

# 2: TFLite direct. always try to load even on laptop, because ultralytics
# mangles bbox coords when wrapping a .tflite model
TFLiteInterpreter = None
try:
    from tflite_runtime.interpreter import Interpreter
    TFLiteInterpreter = Interpreter
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter
        TFLiteInterpreter = Interpreter
    except ImportError:
        try:
            import tensorflow as tf
            TFLiteInterpreter = tf.lite.Interpreter
        except ImportError:
            pass

# 3: NCNN, ARM-optimised, fastest on Pi 5
ncnn_available = False
try:
    import ncnn as _ncnn
    ncnn_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# VisionSystem
# ---------------------------------------------------------------------------

class VisionSystem:
    """Camera + AI detection wrapper. One instance per script.

    Does three things: grab frames (OpenCV or picamera2), run YOLO (ultralytics /
    TFLite / NCNN, whichever is available), and return (found, cx, cy, conf)
    from detect_in_image().
    """

    def __init__(self, camera_index=0, model_path="best.tflite", backend=None, undistort=True):
        self.cap = None
        self._picam = None
        self._requested_backend = backend  # "ncnn" / "tflite" / None (auto)
        self.undistort_enabled = undistort  # False => ignore calibration_data.npz
        cam_w, cam_h = DEFAULT_CAM_W, DEFAULT_CAM_H

        if camera_index is not None:
            cam_w, cam_h = self._open_camera(camera_index, cam_w, cam_h)
        else:
            # no camera, fall back to config resolution so video frames match
            try:
                import config as _cfg
                cam_w, cam_h = _cfg.IMAGE_W, _cfg.IMAGE_H
            except Exception:
                pass

        # global toggle in config can force undistortion off
        try:
            import config as _cfg
            if not getattr(_cfg, 'UNDISTORT_ENABLED', True):
                undistort = False
        except Exception:
            pass

        if undistort:
            self._init_undistortion(cam_w, cam_h)
        else:
            self._undistort_map1 = None
            self._undistort_map2 = None
            print("[VISION] Undistortion DISABLED")
        self._init_model(model_path)

    # ------------------------------------------------------------------
    # Camera initialisation helpers
    # ------------------------------------------------------------------

    def _open_camera(self, camera_index, cam_w, cam_h):
        # try OpenCV first, fall back to picamera2 on Pi
        try:
            import config as _cfg
            cam_w, cam_h = _cfg.IMAGE_W, _cfg.IMAGE_H
        except Exception:
            pass

        print(f"[VISION] Opening Camera Index {camera_index} ({cam_w}x{cam_h})...")
        if sys.platform == 'win32':
            self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(camera_index)
        if self.cap.isOpened():
            # first read at whatever default is, just to prove the camera works
            ret, test_frame = self.cap.read()
            if ret:
                native_h, native_w = test_frame.shape[:2]
                # now try the resolution we actually want
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_h)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                # second read to see what we actually got
                ret2, test_frame2 = self.cap.read()
                if ret2:
                    actual_h, actual_w = test_frame2.shape[:2]
                else:
                    # second read failed, drop back to native and reopen
                    actual_w, actual_h = native_w, native_h
                    self.cap.release()
                    if sys.platform == 'win32':
                        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
                    else:
                        self.cap = cv2.VideoCapture(camera_index)
                if actual_w != cam_w or actual_h != cam_h:
                    print(f"[VISION] Camera doesn't support {cam_w}x{cam_h}, using {actual_w}x{actual_h}")
                    cam_w, cam_h = actual_w, actual_h
                print(f"[VISION] Camera opened via OpenCV ({cam_w}x{cam_h})")
                return cam_w, cam_h
            else:
                self.cap.release()
                self.cap = None
        else:
            self.cap = None

        # fall back to picamera2 on the Pi
        self._open_picamera2(cam_w, cam_h)
        return cam_w, cam_h

    def _open_picamera2(self, cam_w, cam_h):
        try:
            from picamera2 import Picamera2
            import time

            self._picam = Picamera2()
            self._picam.configure(self._picam.create_preview_configuration(
                main={"size": (cam_w, cam_h), "format": "RGB888"}
            ))
            self._picam.start()
            time.sleep(1)

            # eat a few frames so AWB/exposure has a chance to settle
            for _ in range(CAMERA_WARMUP_FRAMES):
                self._picam.capture_array()
            print(f"[VISION] Warmup: discarded {CAMERA_WARMUP_FRAMES} frames")

            self._configure_awb()
            print("[VISION] Camera opened via picamera2")
        except Exception as e:
            print(f"[VISION] No camera available: {e}")

    def _configure_awb(self):
        try:
            import config as _cfg
            import time

            awb_mode = getattr(_cfg, "CAMERA_AWB_MODE", "daylight")
            if awb_mode == "manual":
                gains = getattr(_cfg, "CAMERA_COLOUR_GAINS", (1.5, 1.2))
                self._picam.set_controls({
                    "AwbEnable": False,
                    "ColourGains": gains,
                })
                print(f"[VISION] AWB: manual gains R={gains[0]} B={gains[1]}")
            elif awb_mode != "auto":
                from libcamera import controls
                awb_map = {
                    "daylight": controls.AwbModeEnum.Daylight,
                    "cloudy": controls.AwbModeEnum.Cloudy,
                    "indoor": controls.AwbModeEnum.Indoor,
                    "tungsten": controls.AwbModeEnum.Tungsten,
                    "fluorescent": controls.AwbModeEnum.Fluorescent,
                }
                mode = awb_map.get(awb_mode, controls.AwbModeEnum.Daylight)
                self._picam.set_controls({"AwbMode": mode})
                print(f"[VISION] AWB: {awb_mode}")
            else:
                print("[VISION] AWB: auto")
            time.sleep(0.5)  # let AWB settle
        except Exception as e:
            print(f"[VISION] AWB setup skipped: {e}")

    # ------------------------------------------------------------------
    # Lens undistortion
    # ------------------------------------------------------------------

    def _init_undistortion(self, cam_w, cam_h):
        # load lens calibration and precompute the remap maps (~1-2ms/frame once built)
        # auto-scales the camera matrix if the npz was captured at a different resolution
        self._undistort_map1 = None
        self._undistort_map2 = None

        calib_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "calibration_data.npz"
        )
        if not os.path.exists(calib_path):
            return

        try:
            calib = np.load(calib_path)
            mtx = calib["camera_matrix"].copy()
            dist = calib["dist_coeffs"]

            # rescale camera matrix if calibrated at a different resolution
            if "image_size" in calib:
                cal_w, cal_h = int(calib["image_size"][0]), int(calib["image_size"][1])
                if cal_w != cam_w or cal_h != cam_h:
                    sx, sy = cam_w / cal_w, cam_h / cal_h
                    mtx[0, 0] *= sx  # fx
                    mtx[1, 1] *= sy  # fy
                    mtx[0, 2] *= sx  # cx
                    mtx[1, 2] *= sy  # cy
                    print(f"[VISION] Calibration scaled {cal_w}x{cal_h} -> {cam_w}x{cam_h}")

            new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                mtx, dist, (cam_w, cam_h), 0, (cam_w, cam_h)
            )
            self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                mtx, dist, None, new_mtx, (cam_w, cam_h), cv2.CV_16SC2
            )
            print(f"[VISION] Lens undistortion loaded (RMS={float(calib['rms_error']):.3f})")
        except Exception as e:
            print(f"[VISION] Lens calibration skipped: {e}")

    def undistort(self, frame):
        # apply lens undistort if a calibration is loaded, otherwise passthrough
        if self._undistort_map1 is not None and frame is not None:
            return cv2.remap(frame, self._undistort_map1, self._undistort_map2,
                             cv2.INTER_LINEAR)
        return frame

    # ------------------------------------------------------------------
    # AI model initialisation
    # ------------------------------------------------------------------

    def _init_model(self, model_path):
        # pick whichever backend actually loads
        self.model = None
        self.using_ai = False
        self._use_tflite_direct = False
        self._use_ncnn = False
        self._ncnn_net = None
        self.last_bbox_w = 0
        self.last_bbox_h = 0
        self.last_class_name = ""
        self.backend_name = "none"

        try:
            import config as _cfg
            self._conf_thresh = getattr(_cfg, "CONFIDENCE_THRESHOLD", DEFAULT_CONF_THRESHOLD)
        except Exception:
            self._conf_thresh = DEFAULT_CONF_THRESHOLD

        if not os.path.exists(model_path):
            print(f"[VISION] WARNING: Model file not found: {model_path}")
            print(f"[VISION] Detection DISABLED — mission will fly but never detect targets")
            return

        # priority order: NCNN -> Ultralytics -> TFLite
        if self._try_load_ncnn(model_path):
            return
        if self._try_load_ultralytics(model_path):
            return
        if self._try_load_tflite(model_path):
            return

        print("[VISION] No AI backend available (install ultralytics or tflite-runtime)")

    def _try_load_ncnn(self, model_path):
        # NCNN: triggered by backend="ncnn", --backend ncnn on CLI, a .param path, or an ncnn folder
        path_looks_ncnn = False
        if os.path.isdir(model_path):
            if os.path.exists(os.path.join(model_path, "model.ncnn.param")):
                path_looks_ncnn = True
        elif model_path.endswith('.param'):
            path_looks_ncnn = True

        # horrible, revisit later — sniffs sys.argv because some callers never pass backend=
        ncnn_requested = (self._requested_backend == "ncnn" or
                          ("--backend" in " ".join(sys.argv) and "ncnn" in " ".join(sys.argv)) or
                          path_looks_ncnn)
        if not (ncnn_available and ncnn_requested):
            return False

        # work out which dir holds the .param / .bin
        ncnn_model_dir = None
        if model_path.endswith('.tflite'):
            candidate = os.path.join(os.path.dirname(model_path), "ncnn", "best_ncnn_model")
            if os.path.exists(candidate):
                ncnn_model_dir = candidate
        elif model_path.endswith('.param'):
            ncnn_model_dir = os.path.dirname(model_path)
        elif os.path.isdir(model_path):
            ncnn_model_dir = model_path

        if ncnn_model_dir is None:
            return False

        try:
            param_path = os.path.join(ncnn_model_dir, "model.ncnn.param")
            bin_path = os.path.join(ncnn_model_dir, "model.ncnn.bin")
            if not (os.path.exists(param_path) and os.path.exists(bin_path)):
                print(f"[VISION] NCNN model files not found in {ncnn_model_dir}")
                return False

            print(f"[VISION] Loading NCNN model from {ncnn_model_dir}...")
            self._ncnn_net = _ncnn.Net()
            self._ncnn_net.opt.num_threads = 4
            self._ncnn_net.opt.use_vulkan_compute = False
            self._ncnn_net.load_param(param_path)
            self._ncnn_net.load_model(bin_path)
            self._use_ncnn = True
            self.using_ai = True
            self.model = True
            self.backend_name = "ncnn"

            # warmup
            mat_in = _ncnn.Mat(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3)
            ex = self._ncnn_net.create_extractor()
            ex.input("in0", mat_in)
            ex.extract("out0")
            print("[VISION] NCNN loaded! Warmup complete.")
            return True
        except Exception as e:
            print(f"[VISION] NCNN Load Failed: {e}")
            return False

    def _try_load_ultralytics(self, model_path):
        if self.using_ai or YOLO is None:
            return False

        # ultralytics breaks bbox coords when wrapping a .tflite (it returns
        # frame dims instead of detection coords), so punt to the direct TFLite path
        if model_path.endswith('.tflite'):
            print(f"[VISION] Skipping Ultralytics for .tflite — using direct TFLite backend")
            return False

        try:
            print(f"[VISION] Loading Model via Ultralytics: {model_path}...")
            self.model = YOLO(model_path, task='detect')
            self.model(np.zeros((100, 100, 3), dtype=np.uint8), verbose=False)
            self.using_ai = True
            self.backend_name = "ultralytics"
            print("[VISION] AI Engine Loaded Successfully (Ultralytics)!")
            return True
        except Exception as e:
            print(f"[VISION] Ultralytics Load Failed: {e}")
            self.model = None
            return False

    def _try_load_tflite(self, model_path):
        if TFLiteInterpreter is None or not model_path.endswith('.tflite'):
            return False

        try:
            print(f"[VISION] Loading TFLite Model: {model_path}...")
            self.interpreter = TFLiteInterpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            self._input_details = self.interpreter.get_input_details()
            self._output_details = self.interpreter.get_output_details()
            self._input_shape = self._input_details[0]['shape']
            self._input_dtype = self._input_details[0]['dtype']
            self._use_tflite_direct = True
            self.using_ai = True
            self.model = True  # so the model-is-not-None checks pass
            self.backend_name = "tflite"
            print(f"[VISION] TFLite Loaded! Input: {self._input_shape} dtype={self._input_dtype}")
            return True
        except Exception as e:
            print(f"[VISION] TFLite Load Failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def detect_in_image(self, frame):
        """Run detection on one BGR frame. Returns (found, cx, cy, conf).

        cx/cy are pixel coords in the original frame. The bounding box is
        drawn onto the frame in place. Also updates last_bbox_w/h/class_name.
        """
        if frame is None or not self.using_ai:
            return False, 0, 0, 0.0

        # undistort before inference
        frame = self.undistort(frame)

        # belt-and-braces: if using_ai got set but no backend actually loaded,
        # bail out instead of NPE-ing downstream
        if not self._use_ncnn and not self._use_tflite_direct and self.model is None:
            return False, 0, 0, 0.0

        if self._use_ncnn:
            return self._detect_ncnn(frame)
        if self._use_tflite_direct:
            return self._detect_tflite(frame)
        return self._detect_ultralytics(frame)

    # ------------------------------------------------------------------
    # Backend-specific inference
    # ------------------------------------------------------------------

    def _detect_ncnn(self, frame):
        # NCNN path, fastest on Pi 5
        h, w = frame.shape[:2]

        # preprocess: resize to model input, BGR -> RGB, normalise
        img = cv2.resize(frame, (YOLO_INPUT_SIZE, YOLO_INPUT_SIZE))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        mat_in = _ncnn.Mat.from_pixels(img_rgb, _ncnn.Mat.PixelType.PIXEL_RGB,
                                        YOLO_INPUT_SIZE,YOLO_INPUT_SIZE)
        mean_vals = [0.0, 0.0, 0.0]
        norm_vals = [1 / 255.0, 1 / 255.0, 1 / 255.0]
        mat_in.substract_mean_normalize(mean_vals, norm_vals)

        ex = self._ncnn_net.create_extractor()
        ex.input("in0", mat_in)
        ret, mat_out = ex.extract("out0")

        # YOLOv8 output: [num_attrs, num_detections] or the transpose of that
        output = np.array(mat_out)
        if output.ndim == 2:
            preds = output
        else:
            preds = output.reshape(-1, output.shape[-1]) if output.ndim == 3 else output

        if preds.shape[0] < preds.shape[-1]:
            preds = preds.T  # -> [num_detections, 5+nclass]

        best_conf, best_det = self._pick_best_detection(preds)
        if best_det is None:
            return False, 0, 0, 0.0

        raw_cx, raw_cy = best_det[0], best_det[1]
        raw_bw, raw_bh = best_det[2], best_det[3]

        # NCNN output sometimes comes in pixel coords (0-640), sometimes normalised (0-1)
        if raw_cx > PIXEL_COORD_THRESHOLD:
            cx = int(raw_cx * w / YOLO_INPUT_SIZE)
            cy = int(raw_cy * h / YOLO_INPUT_SIZE)
            bw = int(raw_bw * w / YOLO_INPUT_SIZE)
            bh = int(raw_bh * h / YOLO_INPUT_SIZE)
        else:
            cx = int(raw_cx * w)
            cy = int(raw_cy * h)
            bw = int(raw_bw * w)
            bh = int(raw_bh * h)

        self._store_bbox_metadata(best_det, bw, bh)
        self._draw_detection(frame, cx, cy, bw, bh, w, h, best_conf)
        return True, cx, cy, float(best_conf)

    def _detect_tflite(self, frame):
        # direct TFLite path, used on Pi
        h, w = frame.shape[:2]
        input_h, input_w = self._input_shape[1], self._input_shape[2]

        # preprocess: BGR -> RGB, resize, normalise
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (input_w, input_h))
        if self._input_dtype == np.float32:
            img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        # inference
        self.interpreter.set_tensor(self._input_details[0]['index'], img)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self._output_details[0]['index'])

        # YOLOv8 output can arrive in two shapes:
        #   standard:  [1, 5+nclass, 8400]  -> transpose to [8400, 5+nclass], cols = cx,cy,w,h,scores...
        #   end2end:   [1, 300, 6]          -> already [300, 6], cols = x1,y1,x2,y2,conf,class_id
        preds = output[0]
        end2end = (len(preds.shape) == 2 and preds.shape[-1] == 6)
        if not end2end and preds.shape[0] < preds.shape[-1]:
            preds = preds.T

        if end2end:
            # end2end: [x1, y1, x2, y2, confidence, class_id]
            # this is a bit hacky but it works — manual loop instead of vectorising
            best_conf = 0.0
            best_det = None
            for det in preds:
                conf = float(det[4])
                if conf > self._conf_thresh and conf > best_conf:
                    best_conf = conf
                    best_det = det
            if best_det is None:
                return False, 0, 0, 0.0
            # x1y1x2y2 -> cx,cy,w,h in model input space
            raw_x1, raw_y1, raw_x2, raw_y2 = best_det[0], best_det[1], best_det[2], best_det[3]
            raw_cx = (raw_x1 + raw_x2) / 2
            raw_cy = (raw_y1 + raw_y2) / 2
            raw_bw = raw_x2 - raw_x1
            raw_bh = raw_y2 - raw_y1
            # scale into the real frame
            cx = int(raw_cx * w / input_w)
            cy = int(raw_cy * h / input_h)
            bw = int(raw_bw * w / input_w)
            bh = int(raw_bh * h / input_h)
            # end2end single-class model is always "dummy"
            self.last_class_name = "dummy"
            self.last_bbox_w = bw
            self.last_bbox_h = bh
        else:
            # standard [cx, cy, w, h, class_scores...]
            best_conf, best_det = self._pick_best_detection(preds)
            if best_det is None:
                return False, 0, 0, 0.0
            raw_cx, raw_cy = best_det[0], best_det[1]
            raw_bw, raw_bh = best_det[2], best_det[3]
            # same pixel-vs-normalised ambiguity as NCNN
            if raw_cx > PIXEL_COORD_THRESHOLD:
                cx = int(raw_cx * w / input_w)
                cy = int(raw_cy * h / input_h)
                bw = int(raw_bw * w / input_w)
                bh = int(raw_bh * h / input_h)
            else:
                cx = int(raw_cx * w)
                cy = int(raw_cy * h)
                bw = int(raw_bw * w)
                bh = int(raw_bh * h)
            self._store_bbox_metadata(best_det, bw, bh)

        self._draw_detection(frame, cx, cy, bw, bh, w, h, best_conf)
        return True, cx, cy, float(best_conf)

    def _detect_ultralytics(self, frame):
        # ultralytics YOLO, primary on laptop
        results = self.model(frame, conf=self._conf_thresh, verbose=False)
        if not results[0].boxes:
            return False, 0, 0, 0.0

        best_box = max(results[0].boxes, key=lambda b: b.conf[0])
        conf = float(best_box.conf[0])

        # xyxy is reliable on .pt models (ultralytics breaks it on .tflite, see _try_load_ultralytics)
        x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        bw = int(x2 - x1)
        bh = int(y2 - y1)

        self.last_bbox_w = bw
        self.last_bbox_h = bh
        cls_id = int(best_box.cls[0])
        names = self.model.names if hasattr(self.model, 'names') else {}
        self.last_class_name = names.get(cls_id, f"cls{cls_id}")

        label = f"AI {conf:.2f} [{self.last_class_name}]"
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return True, cx, cy, conf

    # ------------------------------------------------------------------
    # Shared detection helpers
    # ------------------------------------------------------------------

    def _pick_best_detection(self, preds):
        # return (best_conf, best_row) from a [N, 4+C] prediction array
        best_conf = 0.0
        best_det = None
        for det in preds:
            cls_id = int(np.argmax(det[4:]))
            conf = float(det[4 + cls_id])
            if conf > self._conf_thresh and conf > best_conf:
                best_conf = conf
                best_det = det
        return best_conf, best_det

    def _store_bbox_metadata(self, det, bw, bh):
        self.last_bbox_w = bw
        self.last_bbox_h = bh
        num_classes = len(det) - 4
        if num_classes == 1:
            self.last_class_name = "dummy"
        elif num_classes == 5:
            cls_id = int(np.argmax(det[4:]))
            self.last_class_name = SAR_NAMES.get(cls_id, f"cls{cls_id}")
        else:
            cls_id = int(np.argmax(det[4:]))
            self.last_class_name = COCO_NAMES.get(cls_id, f"cls{cls_id}")

    def _draw_detection(self, frame, cx, cy, bw, bh, frame_w, frame_h, conf):
        # draws bbox + label straight onto the frame (mutates)
        x1 = max(0, cx - bw // 2)
        y1 = max(0, cy - bh // 2)
        x2 = min(frame_w, cx + bw // 2)
        y2 = min(frame_h, cy + bh // 2)
        label = f"AI {conf:.2f} [{self.last_class_name}]"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def get_frame(self):
        """Grab a single BGR frame. Returns None if no camera."""
        if self.cap:
            ret, frame = self.cap.read()
            if not ret:
                return None
        elif self._picam:
            frame = self._picam.capture_array()
            # IMX296 global shutter: sensor actually returns BGR despite the RGB888 label,
            # so we don't cvtColor here
        else:
            return None

        # undistort up front so everything downstream sees a clean image
        frame = self.undistort(frame)

        # optional colour correction and 180 flip, driven from config
        try:
            import config as _cfg
            if getattr(_cfg, "CAMERA_COLOR_CORRECTION", False):
                frame = self._gray_world(frame)
            if getattr(_cfg, "CAMERA_FLIP_180", False):
                frame = cv2.flip(frame, -1)
        except Exception:
            pass

        return frame

    def process_frame_manually(self, frame):
        # legacy alias, several older scripts still call this
        return self.detect_in_image(frame)

    def release(self):
        if self.cap:
            self.cap.release()
        if self._picam:
            try:
                self._picam.stop()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Image processing utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _gray_world(frame):
        """Gray-world AWB. Scales each BGR channel so its mean matches the overall mean."""
        f = frame.astype(np.float32)
        avg_b, avg_g, avg_r = f[:, :, 0].mean(), f[:, :, 1].mean(), f[:, :, 2].mean()
        avg_all = (avg_b + avg_g + avg_r) / 3.0
        # FIXME: this isn't gamma-aware, will over-correct on very dark frames
        if avg_b > 0:
            f[:, :, 0] = np.clip(f[:, :, 0] * (avg_all / avg_b), 0, 255)
        if avg_g > 0:
            f[:, :, 1] = np.clip(f[:, :, 1] * (avg_all / avg_g), 0, 255)
        if avg_r > 0:
            f[:, :, 2] = np.clip(f[:, :, 2] * (avg_all / avg_r), 0, 255)
        return f.astype(np.uint8)
