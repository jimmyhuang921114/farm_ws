#!/usr/bin/env python3
"""USB RGB dataset recorder. All camera and disk work stays off the UI thread."""
import argparse
import csv
from datetime import datetime
import os
from pathlib import Path
import queue
import re
import signal
import sys
import threading
import time

import cv2
import yaml

from scripts.camera_test import ROOT, controls, detect, open_camera, usb_id, v4l


def control_snapshot(device):
    try:
        values, raw = controls(device)
        error = None
    except Exception as exc:
        values, raw, error = {}, None, str(exc)
    def active_value(name):
        if raw and re.search(r'^\s*'+name+r'\s+.*flags=.*inactive', raw, re.M):
            return None
        return values.get(name)
    return dict(exposure_mode=values.get('auto_exposure'),
                exposure_value=active_value('exposure_time_absolute'),
                gain=values.get('gain'),
                white_balance={'automatic': values.get('white_balance_automatic'),
                               'temperature': active_value('white_balance_temperature')},
                focus={'automatic': values.get('focus_automatic_continuous'),
                       'absolute': active_value('focus_absolute')},
                controls_readback=values, controls_raw=raw, controls_error=error)


def set_control(device, name, value):
    _, raw = controls(device)
    line = next((s for s in raw.splitlines() if re.match(r'^\s*'+name+r'\s+0x',s)),None)
    if line is None: raise ValueError(f'Control {name} is unavailable')
    fields = dict((k,int(v)) for k,v in re.findall(r'(min|max|step)=(-?\d+)',line))
    if 'min' in fields and not fields['min'] <= value <= fields['max']:
        raise ValueError(f'{name}={value} outside {fields}')
    if fields.get('step') and (value-fields['min']) % fields['step']:
        raise ValueError(f'{name}={value} violates step {fields["step"]}')
    if name == 'auto_exposure':
        block = raw.split(line,1)[1].split('\n\n',1)[0]
        if not re.search(r'^\s*'+str(value)+r':',block,re.M):
            raise ValueError(f'Exposure menu entry {value} is unavailable')
    v4l(device,f'--set-ctrl={name}={value}')
    got, _ = controls(device)
    if got.get(name) != value: raise RuntimeError(f'{name} readback differs: {got.get(name)}')


def apply_controls(device, args, saved):
    before, _ = controls(device)
    names = ['auto_exposure','exposure_time_absolute','gain','power_line_frequency']
    saved.update({k:before[k] for k in names if k in before})
    if args.exposure_mode == 'manual':
        set_control(device,'auto_exposure',1)  # verified menu, never assumed silently
        set_control(device,'exposure_time_absolute',args.exposure)
    elif args.exposure_mode == 'auto':
        set_control(device,'auto_exposure',3)
    if args.gain is not None: set_control(device,'gain',args.gain)
    if args.power_line_frequency is not None:
        set_control(device,'power_line_frequency',args.power_line_frequency)


def restore_controls(device, saved):
    if 'exposure_time_absolute' in saved:
        set_control(device,'auto_exposure',1)
        set_control(device,'exposure_time_absolute',saved['exposure_time_absolute'])
    for name in ['gain','power_line_frequency','auto_exposure']:
        if name in saved: set_control(device,name,saved[name])


def save_metadata(path, data):
    temporary = path.with_suffix('.yaml.tmp')
    temporary.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    temporary.replace(path)


class Session:
    def __init__(self, args, base):
        args.output.mkdir(parents=True,exist_ok=True)
        stem = 'session_'+datetime.now().strftime('%Y%m%d_%H%M%S')
        for n in range(10000):
            self.path = args.output / (stem if n == 0 else f'{stem}_{n:02d}')
            try:
                self.path.mkdir()
                break
            except FileExistsError:
                continue
        else: raise RuntimeError('Cannot allocate a unique session directory')
        (self.path/'rgb').mkdir()
        self.args = args
        self.queue = queue.Queue(maxsize=args.queue_size)
        self.closing = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.captured = self.saved = self.dropped = self.write_failed = 0
        self.avi_frames = 0
        self.error = self.avi_error = None
        self.read_ms = 0.0
        self.start = time.monotonic()
        self.end = None
        self.meta = dict(base, recording_start_time=datetime.now().astimezone().isoformat(),
                         recording_start_unix=time.time(), recording_start_monotonic=self.start,
                         status='initializing', jpeg_quality=args.jpeg_quality,
                         queue_capacity=args.queue_size, avi_requested=args.avi,
                         timestamp_semantics='Host read completion, not sensor exposure time',
                         motion_blur_validation='NOT TESTED - camera could not be moved')
        save_metadata(self.path/'metadata.yaml',self.meta)
        self.thread = threading.Thread(target=self._write,name='dataset-writer',daemon=True)
        self.thread.start()
        if not self.ready.wait(10):
            self.closing.set()
            raise RuntimeError(f'Writer initialization timed out: {self.path}')
        if self.error:
            self.close()
            raise RuntimeError(self.error)
        self.start = time.monotonic()
        self.meta.update(recording_start_monotonic=self.start,recording_start_unix=time.time(),
                         recording_start_time=datetime.now().astimezone().isoformat(),status='recording')
        save_metadata(self.path/'metadata.yaml',self.meta)
        print(f'Recording: {self.path}',flush=True)

    def offer(self, frame, mono, unix, sequence, read_ms):
        with self.lock:
            self.captured += 1
            self.read_ms += read_ms
            try: self.queue.put_nowait((frame,mono,unix,sequence))
            except queue.Full: self.dropped += 1

    def _write(self):
        video = None
        try:
            with (self.path/'timestamps.csv').open('x',newline='') as f:
                out = csv.writer(f)
                out.writerow(['frame_id','timestamp_monotonic','timestamp_unix','capture_sequence'])
                f.flush()
                if self.args.avi:
                    video = cv2.VideoWriter(str(self.path/'preview.avi'),cv2.VideoWriter_fourcc(*'MJPG'),self.args.fps,(self.args.width,self.args.height))
                    if not video.isOpened():
                        self.avi_error = 'VideoWriter could not open; JPEG recording continues'
                        video.release()
                        video = None
                self.ready.set()
                while not self.closing.is_set() or not self.queue.empty():
                    try: frame,mono,unix,sequence = self.queue.get(timeout=.1)
                    except queue.Empty: continue
                    try:
                        if self.error:
                            self.write_failed += 1
                            continue
                        frame_id = self.saved
                        if not cv2.imwrite(str(self.path/'rgb'/f'{frame_id:08d}.jpg'),frame,[cv2.IMWRITE_JPEG_QUALITY,self.args.jpeg_quality]):
                            raise IOError('JPEG write returned false')
                        out.writerow([f'{frame_id:08d}',f'{mono:.9f}',f'{unix:.9f}',sequence])
                        f.flush()
                        with self.lock: self.saved += 1
                        if video is not None:
                            try:
                                video.write(frame)
                                self.avi_frames += 1
                            except Exception as exc:
                                self.avi_error = str(exc)
                                video.release()
                                video = None
                    except Exception as exc:
                        self.error = str(exc)
                        self.write_failed += 1
                    finally: self.queue.task_done()
        except Exception as exc:
            self.error = str(exc)
        finally:
            if video is not None: video.release()
            self.ready.set()

    def close(self, device=None):
        self.end = time.monotonic()
        end_unix = time.time()
        self.closing.set()
        self.thread.join(timeout=30)
        if self.thread.is_alive(): self.error = 'Writer did not drain within 30 seconds; session incomplete'
        duration = max(self.end-self.start,1e-9)
        self.meta.update(status='incomplete' if self.error or self.meta.get('recorder_error') else 'complete', duration_seconds=duration,
                         recording_end_monotonic=self.end,recording_end_unix=end_unix,
                         captured_frames=self.captured,saved_frames=self.saved,
                         dropped_frames=self.dropped,write_failed_frames=self.write_failed,
                         unsaved_frames=self.captured-self.saved,
                         measured_fps=self.captured/duration,effective_saved_fps=self.saved/duration,
                         average_read_latency_ms=self.read_ms/max(self.captured,1),
                         writer_error=self.error,avi_error=self.avi_error,avi_frames=self.avi_frames,
                         drain_seconds=time.monotonic()-self.end)
        if device: self.meta['controls_at_stop'] = control_snapshot(device)
        save_metadata(self.path/'metadata.yaml',self.meta)
        print(f'Duration: {duration:.3f}s | Captured: {self.captured} | Saved: {self.saved} | '
              f'Dropped: {self.dropped} | Average FPS: {self.captured/duration:.3f} | '
              f'Effective saved FPS: {self.saved/duration:.3f}',flush=True)
        if self.error: print(f'ERROR: {self.error}',file=sys.stderr)
        return self.meta


class Capture:
    def __init__(self, cap, args):
        self.cap, self.args = cap, args
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.active = None
        self.latest = None
        self.total = self.failed = 0
        self.read_ms = 0.0
        self.last_success = time.monotonic()
        self.error = None
        self.thread = threading.Thread(target=self._run,name='camera-capture',daemon=True)

    def _run(self):
        try:
            while not self.stop.is_set():
                start = time.monotonic()
                ok, frame = self.cap.read()
                mono, unix = time.monotonic(),time.time()
                ms = (mono-start)*1000
                if not ok:
                    with self.lock: self.failed += 1
                    self.stop.wait(.005)
                    continue
                if frame.shape != (self.args.height,self.args.width,3):
                    raise RuntimeError(f'Unexpected frame shape: {frame.shape}')
                with self.lock:
                    self.total += 1
                    self.read_ms += ms
                    self.last_success = mono
                    self.latest = frame
                    if self.active and mono >= self.active.start:
                        self.active.offer(frame,mono,unix,self.total,ms)
        except Exception as exc: self.error = str(exc)
        finally: self.cap.release()


def arguments():
    p = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--device',help='Override USB VID/PID discovery')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--height',type=int,default=480)
    p.add_argument('--fps',type=float,default=60,help='Native requested FPS; actual FPS is measured')
    p.add_argument('--output',type=Path,default=ROOT/'recordings')
    p.add_argument('--jpeg-quality',type=int,default=95)
    p.add_argument('--buffers',type=int,default=2,help='1 is accepted but halves throughput on the tested camera')
    p.add_argument('--queue-size',type=int,default=120)
    p.add_argument('--no-preview',action='store_true',help='Headless; automatically start recording')
    p.add_argument('--avi',action='store_true',help='Also save MJPG preview.avi in the writer thread')
    p.add_argument('--duration',type=float,help='Automatically start and stop after this many recording seconds')
    p.add_argument('--exposure-mode',choices=['manual','auto','keep'],default='manual')
    p.add_argument('--exposure',type=int,default=156,help='Candidate V4L2 exposure units; requires moving-camera validation')
    p.add_argument('--gain',type=int,default=255,help='Tested high-FPS candidate; scene still needs better lighting')
    p.add_argument('--power-line-frequency',type=int,choices=[0,1,2],default=1,help='V4L2 menu: disabled / 50 Hz / 60 Hz')
    a = p.parse_args()
    if not 1 <= a.jpeg_quality <= 100: p.error('--jpeg-quality must be 1..100')
    if min(a.width,a.height,a.fps,a.buffers,a.queue_size) <= 0: p.error('Dimensions, FPS, buffers and queue size must be positive')
    if a.duration is not None and a.duration <= 0: p.error('--duration must be positive')
    if not a.no_preview and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        p.error('No display available; use --no-preview')
    return a


def main():
    args = arguments()
    cv2.setNumThreads(1)
    device = args.device
    args.output = args.output.expanduser().resolve()
    cap = capture = session = None
    saved_controls = {}
    quit_event = threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_: quit_event.set())
    exit_code = 0
    try:
        device = device or detect()
        # Always try the requested low-latency single buffer first; then use the tested override.
        cap, settings = open_camera(device,args.width,args.height,args.fps,1)
        settings['single_buffer_accepted'] = settings['buffersize_accepted']
        settings['single_buffer_reported'] = settings['buffersize_reported']
        if args.buffers != 1:
            settings['buffersize_accepted'] = bool(cap.set(cv2.CAP_PROP_BUFFERSIZE,args.buffers))
            settings['buffersize_reported'] = cap.get(cv2.CAP_PROP_BUFFERSIZE)
        apply_controls(device,args,saved_controls)
        print(f'{device}: MJPG {args.width}x{args.height} requested {args.fps:g} FPS | buffers {settings["buffersize_reported"]}',flush=True)
        print('Exposure is a static-test candidate; Requires moving-camera validation. Current scene may be too dark.',flush=True)
        capture = Capture(cap,args)
        capture.thread.start()
        warmup = time.monotonic()+2
        while time.monotonic()<warmup and not quit_event.wait(.02):
            if capture.error: raise RuntimeError(capture.error)
        vid,pid = usb_id(device)
        base = dict(device=device,usb_vid=vid,usb_pid=pid,width=args.width,height=args.height,
                    requested_fps=args.fps,measured_fps=None,pixel_format='MJPG',FOURCC='MJPG',
                    backend='CAP_V4L2',opencv_version=cv2.__version__,negotiated=settings)
        if not args.no_preview:
            if not Path(os.environ.get('QT_QPA_FONTDIR','/missing')).is_dir() and Path('/usr/share/fonts/truetype/dejavu').is_dir():
                os.environ['QT_QPA_FONTDIR'] = '/usr/share/fonts/truetype/dejavu'
            cv2.namedWindow('USB RGB Camera | R record | S stop | Q quit',cv2.WINDOW_AUTOSIZE)
        def start_session():
            new = Session(args,dict(base,**control_snapshot(device)))
            with capture.lock: capture.active = new
            return new
        def stop_session(current):
            with capture.lock: capture.active = None
            current.meta['capture_failed_reads_at_stop'] = capture.failed
            return current.close(device)
        if not quit_event.is_set() and (args.no_preview or args.duration is not None):
            session = start_session()
        last_stats = time.monotonic()
        previous_total, previous_ms, previous_saved = capture.total,capture.read_ms,0
        while not quit_event.is_set():
            now = time.monotonic()
            if capture.error: raise RuntimeError(capture.error)
            if now-capture.last_success > 5: raise RuntimeError('No successful camera read for five seconds')
            if session and session.error: raise RuntimeError(f'Writer failed: {session.error}')
            if session and args.duration and now-session.start >= args.duration: break
            with capture.lock:
                frame = capture.latest
                total,latency,failed = capture.total,capture.read_ms,capture.failed
            if now-last_stats >= 5:
                current_saved = session.saved if session else 0
                saved_delta = max(0,current_saved-previous_saved)
                print(f'Capture: {(total-previous_total)/(now-last_stats):.1f} FPS | '
                      f'Recorded: {saved_delta/(now-last_stats):.1f} FPS | '
                      f'Dropped: {session.dropped if session else 0} | '
                      f'Queue: {session.queue.qsize() if session else 0}/{args.queue_size} | '
                      f'Read: {(latency-previous_ms)/max(1,total-previous_total):.1f} ms | Failed: {failed}',flush=True)
                previous_total,previous_ms,previous_saved,last_stats = total,latency,current_saved,now
            if args.no_preview:
                quit_event.wait(.005)
                continue
            if frame is not None:
                display = frame.copy()
                cv2.putText(display,'REC' if session else 'R: record  S: stop  Q: quit',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255) if session else (0,255,0),2)
                cv2.imshow('USB RGB Camera | R record | S stop | Q quit',display)
            key = cv2.waitKey(1)&0xff
            if key in (ord('q'),ord('Q')): break
            if key in (ord('r'),ord('R')) and session is None:
                session = start_session()
                previous_saved = 0
            if key in (ord('s'),ord('S')) and session:
                summary = stop_session(session)
                session = None
                previous_saved = 0
                if summary['writer_error']: raise RuntimeError(summary['writer_error'])
            if cv2.getWindowProperty('USB RGB Camera | R record | S stop | Q quit',cv2.WND_PROP_VISIBLE)<1: break
    except Exception as exc:
        print(f'ERROR: {exc}',file=sys.stderr,flush=True)
        if session: session.meta['recorder_error'] = str(exc)
        exit_code = 1
    finally:
        if capture:
            with capture.lock: capture.active = None
            if session:
                session.meta['capture_failed_reads_at_stop'] = capture.failed
                summary = session.close(device)
                if summary['writer_error']: exit_code = 1
            capture.stop.set()
            capture.thread.join(timeout=6)
            if capture.thread.is_alive():
                print('ERROR: capture read is blocked; exiting with incomplete camera shutdown',file=sys.stderr)
                exit_code = 1
        elif cap: cap.release()
        try:
            if saved_controls: restore_controls(device,saved_controls)
        except Exception as exc:
            print(f'ERROR restoring controls: {exc}',file=sys.stderr)
            exit_code = 1
        if not args.no_preview: cv2.destroyAllWindows()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
