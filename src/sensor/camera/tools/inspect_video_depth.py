#!/usr/bin/env python3
"""Inspect synchronous RGB and metric predictions from recorded videos."""
import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from camera.depth_estimate import DepthEstimate
from video_rgb_depth_compare import depth_view


class Viewer:
    window = 'Version6 RGB | predicted depth (meters)'

    def __init__(self, model, videos, args):
        self.model, self.videos, self.args = model, videos, args
        self.cap = None
        self.video_index = 0
        self.mouse = None
        self.paused = True
        self.open_video(0)

    def open_video(self, index):
        if self.cap is not None:
            self.cap.release()
        self.video_index = index % len(self.videos)
        self.path = self.videos[self.video_index]
        self.cap = cv2.VideoCapture(str(self.path))
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open {self.path}')
        self.total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(self.fps) or self.fps <= 0:
            self.fps = 30
        self.frame = -1
        self.mouse = None
        self.paused = True
        if not self.read_frame():
            raise RuntimeError(f'Empty video: {self.path}')

    def read_frame(self, target=None):
        if target is not None:
            target = max(0, min(self.total - 1, target))
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, target)
        ok, bgr = self.cap.read()
        if not ok:
            self.paused = True
            return False
        self.frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
        self.bgr = bgr
        self.depth = self.model.predict(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), None)
        h, w = bgr.shape[:2]
        self.dw = min(w, self.args.panel_width)
        self.dh = round(h * self.dw / w)
        self.base = np.hstack((cv2.resize(bgr, (self.dw, self.dh)),
                              cv2.resize(depth_view(self.depth, self.args.max_depth), (self.dw, self.dh))))
        return True

    def on_mouse(self, event, x, y, flags, userdata):
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN):
            if 0 <= x < 2 * self.dw and 0 <= y < self.dh:
                h, w = self.depth.shape
                self.mouse = (min(w - 1, int((x % self.dw) * w / self.dw)),
                              min(h - 1, int(y * h / self.dh)))
                if event == cv2.EVENT_LBUTTONDOWN:
                    self.paused = True
            else:
                self.mouse = None

    def render(self):
        canvas = cv2.copyMakeBorder(self.base.copy(), 0, 118, 0, 0, cv2.BORDER_CONSTANT)
        value = 'Move mouse over RGB or depth; click to pause and inspect'
        if self.mouse is not None:
            x, y = self.mouse
            z = float(self.depth[y, x])
            value = f'Pixel ({x}, {y})  predicted depth: {z:.3f} m / {z*100:.1f} cm' if np.isfinite(z) and z > 0 else f'Pixel ({x}, {y}) INVALID'
            px, py = int(x * self.dw / self.depth.shape[1]), int(y * self.dh / self.depth.shape[0])
            for offset in (0, self.dw):
                cv2.drawMarker(canvas, (px + offset, py), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
        lines = [f'{self.path.name} | frame {self.frame+1}/{self.total} | {self.frame/self.fps:.2f}s | {"PAUSED" if self.paused else "PLAYING"}',
                 f'Left: RGB | Right: prediction | color scale 0-{self.args.max_depth:g} m (values not clipped)',
                 value,
                 'Space: play/pause  A/D: frame  J/L: -/+5s  N/P: video  S: save  Q/Esc: quit']
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (10, self.dh + 23 + i*27), cv2.FONT_HERSHEY_SIMPLEX, .49, (255,255,255), 1, cv2.LINE_AA)
        return canvas

    def save(self):
        folder = self.args.output_dir / self.path.stem
        folder.mkdir(parents=True, exist_ok=True)
        stem = folder / f'frame_{self.frame:06d}'
        np.save(str(stem) + '_depth_m.npy', self.depth)
        cv2.imwrite(str(stem) + '_rgb.png', self.bgr)
        cv2.imwrite(str(stem) + '_compare.png', self.render())
        print(f'Saved RGB, comparison and float32 meters: {stem}', flush=True)

    def run(self):
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, self.on_mouse)
        try:
            while True:
                started = time.monotonic()
                if not self.paused:
                    self.read_frame()
                cv2.imshow(self.window, self.render())
                delay = 20 if self.paused else max(1, round(1000 / self.fps - 1000 * (time.monotonic()-started)))
                key = cv2.waitKey(delay) & 0xff
                if key in (27, ord('q')) or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord(' '):
                    self.paused = not self.paused
                elif key in (ord('a'), ord('d'), ord('j'), ord('l')):
                    self.paused = True
                    delta = {ord('a'): -1, ord('d'): 1, ord('j'): -round(5*self.fps), ord('l'): round(5*self.fps)}[key]
                    self.read_frame(self.frame + delta)
                elif key in (ord('n'), ord('p')):
                    self.open_video(self.video_index + (1 if key == ord('n') else -1))
                elif key == ord('s'):
                    self.save()
        finally:
            self.cap.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = PACKAGE.parents[3]
    parser.add_argument('videos', nargs='*', type=Path)
    parser.add_argument('--input-dir', type=Path, default=root / 'mp4_of_realsense')
    parser.add_argument('--checkpoint', type=Path, default=PACKAGE / 'model/version6/last.pt')
    parser.add_argument('--output-dir', type=Path, default=root / 'mp4_of_realsense/version6_inspection')
    parser.add_argument('--device', default='cuda')
    parser.add_argument('--max-depth', type=float, default=20)
    parser.add_argument('--panel-width', type=int, default=640)
    parser.add_argument('--smoke-test', action='store_true', help='Infer and save first frame of every video without GUI')
    args = parser.parse_args()
    if not np.isfinite(args.max_depth) or args.max_depth <= 0 or args.panel_width < 320:
        parser.error('max-depth must be positive and panel-width must be >= 320')
    videos = args.videos or [p for p in sorted(args.input_dir.glob('*.mp4')) if not p.stem.endswith(('_rgb_depth', '_depth_preview'))]
    if not videos:
        parser.error(f'No videos in {args.input_dir}')
    cv2.setNumThreads(1)
    print(f'Loading {args.checkpoint}', flush=True)
    model = DepthEstimate(args.checkpoint, args.device)
    viewer = Viewer(model, videos, args)
    if args.smoke_test:
        try:
            for i in range(len(videos)):
                if i:
                    viewer.open_video(i)
                assert viewer.depth.shape == viewer.bgr.shape[:2]
                assert np.isfinite(viewer.depth).all() and (viewer.depth > 0).any()
                viewer.save()
                print(f'{viewer.path.name}: {viewer.depth.shape}, range {viewer.depth.min():.3f}..{viewer.depth.max():.3f} m', flush=True)
        finally:
            viewer.cap.release()
    else:
        viewer.run()


if __name__ == '__main__':
    main()
