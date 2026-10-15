#!/usr/bin/env python3
"""Run the metric depth model on MP4 files and save RGB/depth comparisons."""

import argparse
from pathlib import Path
import time

import cv2
import numpy as np

from camera.depth_estimate import DepthEstimate


def depth_view(depth, max_depth):
    valid = np.isfinite(depth) & (depth > 0)
    scaled = np.zeros(depth.shape, dtype=np.uint8)
    scaled[valid] = (
        np.clip(depth[valid] / max_depth, 0.0, 1.0) * 255.0
    ).astype(np.uint8)
    view = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    view[~valid] = 0
    return view


def put_text(image, lines):
    y = 26
    for line in lines:
        cv2.putText(
            image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            image, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (0, 0, 0), 1, cv2.LINE_AA)
        y += 27


def process_video(path, model, max_depth):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(fps) or fps <= 0:
        fps = 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    side_path = path.with_name(path.stem + "_rgb_depth.mp4")
    depth_path = path.with_name(path.stem + "_depth_preview.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    side_writer = cv2.VideoWriter(str(side_path), fourcc, fps, (2 * width, height))
    depth_writer = cv2.VideoWriter(str(depth_path), fourcc, fps, (width, height))
    if not side_writer.isOpened() or not depth_writer.isOpened():
        cap.release()
        raise RuntimeError("cannot create output video writer")

    count = 0
    total_time = 0.0
    global_min = float("inf")
    global_max = 0.0
    valid_count = 0
    pixel_count = 0
    started = time.perf_counter()
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        t0 = time.perf_counter()
        depth = model.predict(rgb, None)
        elapsed = time.perf_counter() - t0
        total_time += elapsed

        valid = np.isfinite(depth) & (depth > 0)
        if np.any(valid):
            global_min = min(global_min, float(np.min(depth[valid])))
            global_max = max(global_max, float(np.max(depth[valid])))
            valid_count += int(valid.sum())
        pixel_count += int(depth.size)

        dview = depth_view(depth, max_depth)
        side = np.hstack((bgr, dview))
        put_text(bgr, [f"RGB | frame {count}"])
        put_text(dview, [
            f"Depth | display 0-{max_depth:g} m",
            f"median {float(np.median(depth[valid])):.2f} m" if np.any(valid) else "no valid depth",
        ])
        side = np.hstack((bgr, dview))
        side_writer.write(side)
        depth_writer.write(dview)
        count += 1
        if count % 30 == 0:
            print(
                f"{path.name}: {count}/{total if total > 0 else '?'} frames, "
                f"model={count / total_time:.1f} FPS", flush=True)

    cap.release()
    side_writer.release()
    depth_writer.release()
    wall = time.perf_counter() - started
    print(
        f"DONE {path.name}: frames={count}, input_fps={fps:.2f}, "
        f"model_fps={count / total_time if total_time else 0:.2f}, "
        f"wall_fps={count / wall if wall else 0:.2f}, "
        f"valid_ratio={valid_count / pixel_count if pixel_count else 0:.6f}, "
        f"depth_range={global_min:.3f}..{global_max:.3f} m, "
        f"side={side_path.name}, depth={depth_path.name}",
        flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir", type=Path,
        default=Path("/home/jimmy/farm_ws/mp4_of_realsense"))
    parser.add_argument(
        "--checkpoint", type=Path,
        default=Path("/home/jimmy/farm_ws/farm_ws/src/sensor/camera/model/version4/best.pt"))
    parser.add_argument("--max-depth", type=float, default=20.0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("videos", nargs="*", type=Path)
    args = parser.parse_args()
    videos = args.videos or sorted(args.input_dir.glob("*.mp4"))
    videos = [p for p in videos if not any(
        p.stem.endswith(suffix) for suffix in ("_rgb_depth", "_depth_preview"))]
    if not videos:
        raise SystemExit(f"No input MP4 files found in {args.input_dir}")
    print(f"Loading checkpoint: {args.checkpoint}", flush=True)
    model = DepthEstimate(str(args.checkpoint), args.device)
    for path in videos:
        process_video(path, model, args.max_depth)


if __name__ == "__main__":
    main()
