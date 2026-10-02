#!/usr/bin/env python3
"""Live RAW | UNDISTORTED comparison; never writes or modifies RAW images."""
import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

from camera_utils import (DEFAULT_CONFIG,create_window,load_calibration,
                          load_config,open_camera,read_frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--calibration',type=Path,help='Default: calibration.output from config YAML')
    parser.add_argument('--check-config',action='store_true',help='Validate config/imports only; no calibration file, camera or GUI needed')
    args = parser.parse_args()
    cap = None
    window = 'RAW | UNDISTORTED - Q to quit'
    opened_window = False
    try:
        cfg = load_config(args.config)
        if args.check_config:
            print(f"Config OK. Camera: {cfg['camera']}; calibration: {args.calibration or cfg['calibration']['output']}")
            return 0
        size = (cfg['camera']['width'],cfg['camera']['height'])
        k,d = load_calibration(args.calibration or cfg['calibration']['output'],size)
        # Keep K and output resolution unchanged. Black borders are expected;
        # no crop, resize, or ROI adjustment is applied to camera geometry.
        map_x,map_y = cv2.initUndistortRectifyMap(k,d,None,k,size,cv2.CV_32FC1)
        create_window(window)
        opened_window = True
        cap,_ = open_camera(cfg['camera'])
        while True:
            raw = read_frame(cap,cfg['camera'])
            undistorted = cv2.remap(raw,map_x,map_y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            comparison = np.hstack((raw,undistorted))
            for text,x in [('RAW',15),('UNDISTORTED',size[0]+15)]:
                cv2.putText(comparison,text,(x,35),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),5)
                cv2.putText(comparison,text,(x,35),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
            cv2.imshow(window,comparison)
            key = cv2.waitKey(1)&0xff
            if key in (ord('q'),ord('Q')) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1: break
        return 0
    except KeyboardInterrupt:
        return 0
    except (OSError,ValueError,RuntimeError,cv2.error,yaml.YAMLError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        return 1
    finally:
        if cap is not None: cap.release()
        if opened_window: cv2.destroyAllWindows()


if __name__ == '__main__': sys.exit(main())
