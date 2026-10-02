"""Shared YAML validation, V4L2 camera initialization and chessboard geometry."""
from pathlib import Path
import math
import os
import sys
import warnings

import cv2
import numpy as np
import yaml

PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = PROJECT_DIR / 'config/calibration.yaml'


def load_config(path=DEFAULT_CONFIG):
    """Relative data paths are resolved against the project, never the shell cwd."""
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict):
        raise ValueError('Config must be a YAML mapping')
    required = {'camera': ('device', 'width', 'height', 'fps', 'fourcc'),
                'chessboard': ('cols', 'rows', 'square_size_mm'),
                'capture': ('target_images', 'save_dir'),
                'calibration': ('output',)}
    for section, keys in required.items():
        if not isinstance(cfg.get(section), dict):
            raise ValueError(f'Missing config section: {section}')
        for key in keys:
            if key not in cfg[section]:
                raise ValueError(f'Missing config key: {section}.{key}')
    for section, key in [('camera','width'), ('camera','height'),
                         ('chessboard','cols'), ('chessboard','rows'),
                         ('capture','target_images')]:
        value = cfg[section][key]
        if type(value) is not int or value <= 0:
            raise ValueError(f'{section}.{key} must be a positive integer')
    if min(cfg['chessboard']['cols'], cfg['chessboard']['rows']) < 2:
        raise ValueError('Chessboard must have at least two inner corners per axis')
    if cfg['capture']['target_images'] < 3:
        raise ValueError('capture.target_images must be at least 3 (20–30 recommended)')
    for section, key in [('camera','fps'), ('chessboard','square_size_mm')]:
        value = cfg[section][key]
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{section}.{key} must be finite and positive')
    device = cfg['camera']['device']
    if not ((type(device) is int and device >= 0) or (isinstance(device,str) and device.strip())):
        raise ValueError('camera.device must be a nonnegative index or a device path')
    fourcc = cfg['camera']['fourcc']
    if not isinstance(fourcc,str) or len(fourcc) != 4 or not fourcc.isascii():
        raise ValueError('camera.fourcc must contain four ASCII characters')
    for section,key in [('capture','save_dir'), ('calibration','output')]:
        value = cfg[section][key]
        if not isinstance(value,str) or not value.strip():
            raise ValueError(f'{section}.{key} must be a nonempty path')
        dest = Path(value).expanduser()
        cfg[section][key] = str(dest if dest.is_absolute() else PROJECT_DIR / dest)
    return cfg


def fourcc_text(value):
    code = int(value)
    return ''.join(chr((code >> (8*i)) & 255) for i in range(4))


def open_camera(settings):
    device = settings['device']
    backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
    cap = cv2.VideoCapture(device, backend)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'Cannot open camera {device!r}. Check camera.device in YAML, /dev/video*, permissions and whether the camera is busy.')
    try:
        properties = [('FOURCC',cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*settings['fourcc'])),
                      ('width',cv2.CAP_PROP_FRAME_WIDTH,settings['width']),
                      ('height',cv2.CAP_PROP_FRAME_HEIGHT,settings['height']),
                      ('FPS',cv2.CAP_PROP_FPS,settings['fps'])]
        for name, prop, value in properties:
            if not cap.set(prop,value):
                warnings.warn(f'Camera backend rejected requested {name}')
        actual = {'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                  'fps': float(cap.get(cv2.CAP_PROP_FPS)),
                  'fourcc': fourcc_text(cap.get(cv2.CAP_PROP_FOURCC)),
                  'backend': cap.getBackendName()}
        print(f"Requested: {settings['width']}x{settings['height']} @ {settings['fps']:g} FPS\n"
              f"Actual: {actual['width']}x{actual['height']} @ {actual['fps']:g} FPS\n"
              f"FOURCC: {actual['fourcc']} | Backend: {actual['backend']}\n"
              'FPS above is backend readback, not measured capture throughput.',flush=True)
        if (actual['width'],actual['height']) != (settings['width'],settings['height']):
            raise RuntimeError('Negotiated resolution differs from YAML. Choose a supported mode; calibration will not silently use a different size.')
        if actual['fourcc'] != settings['fourcc']:
            warnings.warn('Negotiated FOURCC differs from the request')
        if abs(actual['fps']-settings['fps']) > .1:
            warnings.warn('Negotiated FPS differs from the request')
        return cap, actual
    except Exception:
        cap.release()
        raise


def read_frame(cap, settings):
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError('Camera read failed or device disconnected')
    if frame.shape != (settings['height'],settings['width'],3):
        raise RuntimeError(f'Unexpected frame shape {frame.shape}; refusing mixed-resolution samples')
    return frame


def make_object_points(board):
    # Planar chessboard coordinates in millimetres. x advances across columns;
    # y advances across rows; z=0. The returned tvecs therefore also use mm.
    points = np.zeros((board['cols']*board['rows'],3),dtype=np.float32)
    points[:,:2] = np.mgrid[0:board['cols'],0:board['rows']].T.reshape(-1,2)
    points[:,:2] *= board['square_size_mm']
    return points


class ChessboardDetector:
    def __init__(self, board):
        self.pattern = (board['cols'],board['rows'])
        self.use_sb = hasattr(cv2,'findChessboardCornersSB')

    def detect(self, gray):
        if self.use_sb:
            try:
                found, corners = cv2.findChessboardCornersSB(gray,self.pattern,cv2.CALIB_CB_NORMALIZE_IMAGE)
                # SB already returns subpixel-refined coordinates. Running the
                # classic cornerSubPix again is unnecessary for this estimator.
                return bool(found), np.asarray(corners,dtype=np.float32).reshape(-1,1,2) if found else None
            except (cv2.error,TypeError,AttributeError) as exc:
                warnings.warn(f'SB detector unavailable/incompatible; using classic detector: {exc}')
                self.use_sb = False
        found,corners = cv2.findChessboardCorners(gray,self.pattern,cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),
                                      (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,30,.001))
        return bool(found), np.asarray(corners,dtype=np.float32).reshape(-1,1,2) if found else None


def create_window(name):
    if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('A graphical desktop is required for interactive preview')
    fonts = Path('/usr/share/fonts/truetype/dejavu')
    if fonts.is_dir() and not Path(os.environ.get('QT_QPA_FONTDIR','/missing')).is_dir():
        os.environ['QT_QPA_FONTDIR'] = str(fonts)
    cv2.namedWindow(name,cv2.WINDOW_NORMAL)


def load_calibration(path, size):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data,dict): raise ValueError('Calibration must be a YAML mapping')
    if (data.get('image_width'),data.get('image_height')) != tuple(size):
        raise ValueError('Calibration resolution does not match camera config. Recalibrate at this resolution; intrinsics will not be silently scaled.')
    if data.get('distortion_model') != 'plumb_bob':
        raise ValueError('Only the five-coefficient plumb_bob model is supported')
    try:
        km,dm = data['camera_matrix'],data['distortion_coefficients']
        if (km['rows'],km['cols']) != (3,3) or (dm['rows'],dm['cols']) != (1,5):
            raise ValueError('Expected K 3x3 and distortion 1x5')
        k = np.asarray(km['data'],dtype=np.float64).reshape(3,3)
        d = np.asarray(dm['data'],dtype=np.float64).reshape(1,5)
    except (KeyError,TypeError,ValueError) as exc:
        raise ValueError(f'Invalid calibration matrix: {exc}') from exc
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]):
        raise ValueError('Calibration matrix contains invalid intrinsics/distortion')
    return k,d
