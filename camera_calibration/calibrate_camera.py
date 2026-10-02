#!/usr/bin/env python3
"""Interactively collect RAW chessboard frames and calibrate a USB camera."""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

from camera_utils import (DEFAULT_CONFIG,ChessboardDetector,create_window,
                          load_config,make_object_points,open_camera,read_frame)


def save_raw_frame(directory,frame):
    """Never save preview overlays; never overwrite a previous sample."""
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    index = max((int(p.stem) for p in directory.glob('*.jpg') if p.stem.isdigit()),default=0)+1
    ok,encoded = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95])
    if not ok: raise IOError('JPEG encoding failed; sample not collected')
    while True:
        path = directory / f'{index:06d}.jpg'
        try:
            with path.open('xb') as stream:
                stream.write(encoded.tobytes())
            return path
        except FileExistsError:
            index += 1


def calibrate_samples(cfg,image_points,image_paths,size,actual=None):
    if len(image_points) < 3:
        raise ValueError('At least 3 samples are required; collect 20–30 varied views for a useful calibration.')
    if len(image_points) != len(image_paths): raise ValueError('Sample/image filename count mismatch')
    if len(image_points) < 20:
        print('WARNING: Fewer than 20 views. Capture more varied positions, tilts and distances.')
    obj = make_object_points(cfg['chessboard'])
    if any(np.asarray(p).shape != (len(obj),1,2) or not np.isfinite(p).all() for p in image_points):
        raise ValueError('Invalid chessboard image points')
    objects = [obj.copy() for _ in image_points]
    # Default pinhole model: k1,k2,p1,p2,k3 (no rational/fisheye flags).
    rms,k,d,rvecs,tvecs = cv2.calibrateCamera(objects,image_points,tuple(size),None,None)
    d = d.reshape(-1)
    if len(d)!=5 or not np.isfinite(k).all() or not np.isfinite(d).all() or not np.isfinite(rms):
        raise ValueError('Calibration produced non-finite values or an unexpected distortion model; output not written')
    if k[0,0]<=0 or k[1,1]<=0:
        raise ValueError('Invalid fx/fy (must be positive); output not written')
    per_image = []
    for points,path,rvec,tvec in zip(image_points,image_paths,rvecs,tvecs):
        projected,_ = cv2.projectPoints(obj,rvec,tvec,k,d)
        # Euclidean distance in pixels for each corner, then the arithmetic mean.
        # This is NOT norm(residuals)/N, and is distinct from OpenCV's RMS.
        distances = np.linalg.norm(points.reshape(-1,2)-projected.reshape(-1,2),axis=1)
        per_image.append({'image':Path(path).name,
                          'mean_reprojection_error':float(distances.mean()),
                          'rms_reprojection_error':float(np.sqrt(np.mean(distances**2))),
                          'rvec':np.asarray(rvec).reshape(-1).tolist(),
                          'tvec_mm':np.asarray(tvec).reshape(-1).tolist()})
    mean = float(np.mean([row['mean_reprojection_error'] for row in per_image]))
    notes = []
    if not (0 <= k[0,2] < size[0] and 0 <= k[1,2] < size[1]):
        notes.append('WARNING: Principal point is outside the image. Check board dimensions and view diversity.')
    if mean > 1.0:
        notes.append('WARNING: High reprojection error. Consider capturing better calibration images.')
    elif mean < .5:
        notes.append('Calibration quality good (reprojection metric only; not independent validation).')
    result = {'image_width':int(size[0]),'image_height':int(size[1]),
              'camera_name':cfg['camera'].get('name','usb_camera'),
              'camera_matrix':{'rows':3,'cols':3,'data':k.reshape(-1).tolist()},
              'distortion_model':'plumb_bob',
              'distortion_coefficients':{'rows':1,'cols':5,'data':d.tolist()},
              'intrinsics':{'fx':float(k[0,0]),'fy':float(k[1,1]),'cx':float(k[0,2]),'cy':float(k[1,2])},
              'calibration':{'rms_error':float(rms),'mean_reprojection_error':mean,
                             'images_used':len(image_points),
                             'error_definition':'Mean Euclidean corner distance in pixels; all views have equal corner counts',
                             'per_image':per_image,'quality_messages':notes,
                             'timestamp':datetime.now().astimezone().isoformat(),
                             'opencv_version':cv2.__version__,
                             'chessboard':dict(cfg['chessboard']),
                             'camera_requested':dict(cfg['camera']),
                             'camera_actual':actual,
                             'raw_images_directory':cfg['capture']['save_dir'],
                             'source_images':'RAW distorted color JPEGs; no corner overlays or undistortion'}}
    best = min(per_image,key=lambda row:row['mean_reprojection_error'])
    worst = max(per_image,key=lambda row:row['mean_reprojection_error'])
    print('\n'+'='*40+'\nCamera Calibration Result\n'+'='*40)
    print(f'Resolution: {size[0]} x {size[1]}\nImages used: {len(image_points)}\nRMS: {rms:.6f} px\nMean reprojection error: {mean:.6f} px\n\nCamera Matrix:')
    for key,value in result['intrinsics'].items(): print(f'{key} = {value:.6f}')
    print('\nDistortion:')
    for key,value in zip(['k1','k2','p1','p2','k3'],d): print(f'{key} = {value:.9g}')
    print('\nPer-image mean reprojection error:')
    for row in per_image: print(f"  {row['image']}: {row['mean_reprojection_error']:.6f} px")
    print(f"\nWorst image: {worst['image']}\nError: {worst['mean_reprojection_error']:.6f} px\nBest image: {best['image']}\nError: {best['mean_reprojection_error']:.6f} px")
    for note in notes: print(note)
    print('='*40)
    return result


def save_result(path,result):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    # Keep previous geometry before atomically replacing the requested output.
    if path.exists():
        backup = path.with_name(path.name+'.'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.bak')
        with backup.open('xb') as stream: stream.write(path.read_bytes())
        print(f'Previous calibration preserved: {backup}')
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(yaml.safe_dump(result,sort_keys=False),encoding='utf-8')
    temporary.replace(path)
    print(f'Calibration saved: {path}')


def run(cfg):
    cap = None
    window = 'Camera Calibration'
    image_points,image_paths = [],[]
    board = cfg['chessboard']
    target = cfg['capture']['target_images']
    detector = ChessboardDetector(board)
    message = 'Move the board between samples; keep all inner corners visible.'
    message_until = time.monotonic()+5
    try:
        create_window(window)
        cap,actual = open_camera(cfg['camera'])
        while True:
            raw = read_frame(cap,cfg['camera'])
            found,corners = detector.detect(cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY))
            preview = raw.copy()  # All overlays remain separate from saved RAW.
            if found: cv2.drawChessboardCorners(preview,detector.pattern,corners,found)
            lines = ['Camera Calibration',f'Resolution: {raw.shape[1]}x{raw.shape[0]}',
                     f"Chessboard: {board['cols']} x {board['rows']} inner corners",
                     f"Square: {board['square_size_mm']} mm",f'Captured: {len(image_points)} / {target}',
                     'CHESSBOARD DETECTED' if found else 'NO CHESSBOARD',
                     'SPACE Capture | C Calibrate | R Reset | Q Quit']
            if time.monotonic()<message_until: lines.append(message)
            for index,line in enumerate(lines):
                position = (12,28+index*28)
                cv2.putText(preview,line,position,cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),4)
                cv2.putText(preview,line,position,cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0) if found else (0,220,255),1)
            cv2.imshow(window,preview)
            key = cv2.waitKey(1)&0xff
            if key in (ord('q'),ord('Q')) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1: break
            if key == 32:
                if not found:
                    message = 'Chessboard not detected - image NOT saved'
                elif len(image_points)>=target:
                    message = 'Target reached. Press C to calibrate or R to reset.'
                else:
                    path = save_raw_frame(cfg['capture']['save_dir'],raw)
                    image_points.append(corners.copy())
                    image_paths.append(path)
                    message = f'Saved {path.name} ({len(image_points)} / {target})'
                print(message,flush=True)
                message_until = time.monotonic()+4
            elif key in (ord('r'),ord('R')):
                image_points.clear()
                image_paths.clear()
                message = 'Samples reset; existing RAW files retained. Numbering continues.'
                print(message,flush=True)
                message_until = time.monotonic()+5
            elif key in (ord('c'),ord('C')):
                try:
                    result = calibrate_samples(cfg,image_points,image_paths,(raw.shape[1],raw.shape[0]),actual)
                    save_result(cfg['calibration']['output'],result)
                    message = 'Calibration saved. See terminal for errors and quality checks.'
                except (ValueError,cv2.error) as exc:
                    message = f'Calibration not saved: {exc}'
                    print(message,file=sys.stderr,flush=True)
                message_until = time.monotonic()+6
    finally:
        if cap is not None: cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--check-config',action='store_true',help='Validate YAML/imports without opening a camera or GUI')
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        if args.check_config:
            print(yaml.safe_dump(cfg,sort_keys=False))
            print(f"Config OK; object points: {make_object_points(cfg['chessboard']).shape}, units mm")
        else: run(cfg)
        return 0
    except KeyboardInterrupt:
        return 0
    except (OSError,ValueError,RuntimeError,cv2.error,yaml.YAMLError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        return 1


if __name__ == '__main__': sys.exit(main())
