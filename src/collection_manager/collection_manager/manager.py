import os, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from collection_interfaces.msg import CollectionState, CollectionEvent
from collection_interfaces.srv import PreflightCollection, StartCollection, StopCollection, AddMarker
from .session import SessionStore

class CollectionManager(Node):
    def __init__(self):
        super().__init__('collection_manager')
        self.declare_parameter('session_root','/workspace/farm_ws/mapping_sessions')
        self.store=SessionStore(self.get_parameter('session_root').value); self.state=CollectionState.IDLE
        self.duration=0.; self.start_at=None; self.countdown_at=None
        self.pub=self.create_publisher(CollectionState,'/collection/state',10)
        self.events=self.create_publisher(CollectionEvent,'/collection/events',10)
        self.create_service(PreflightCollection,'/collection/preflight',self.preflight)
        self.create_service(StartCollection,'/collection/start',self.start)
        self.create_service(StopCollection,'/collection/stop',self.stop)
        self.create_service(AddMarker,'/collection/add_marker',self.marker)
        self.create_timer(0.1,self.tick)
    def emit(self,msg,level='INFO',marker=''):
        self.store.write_event(level,msg,marker); e=CollectionEvent(); e.stamp=self.get_clock().now().to_msg(); e.level=level;e.message=msg;e.marker=marker;self.events.publish(e)
    def preflight(self,req,res):
        if self.state in (CollectionState.COUNTDOWN,CollectionState.RECORDING): res.success=False;res.message='Collection active';return res
        self.state=CollectionState.PREFLIGHT; self.duration=max(0.1,float(req.duration_sec))
        cfg=dict(session_name=req.session_name,location=req.location,duration_sec=self.duration,profile=req.profile,note=req.note,ui_only=bool(req.ui_only))
        if not req.ui_only: self.state=CollectionState.FAILED;res.success=False;res.message='Real rosbag2 recording is not enabled yet';return res
        path=self.store.create(cfg); self.state=CollectionState.IDLE;res.success=True;res.message='Ready';res.session_path=str(path);return res
    def start(self,req,res):
        if not self.store.path: res.success=False;res.message='Run preflight first';return res
        if self.state not in (CollectionState.IDLE,CollectionState.COMPLETED): res.success=False;res.message='Collection active';return res
        self.state=CollectionState.COUNTDOWN;self.countdown_at=time.monotonic();self.start_at=None;self.emit('Countdown started');res.success=True;res.message='3 second countdown';return res
    def finish(self,reason='Duration completed'):
        self.state=CollectionState.FINALIZING;self.emit(reason);self.state=CollectionState.VERIFYING
        # UI-only verification explicitly rejects accidental MCAP creation.
        if any(self.store.path.rglob('*.mcap')): self.state=CollectionState.FAILED;self.emit('Unexpected MCAP in UI-only session','ERROR')
        else: self.state=CollectionState.COMPLETED;self.emit('Session completed')
    def stop(self,req,res):
        if self.state not in (CollectionState.COUNTDOWN,CollectionState.RECORDING):res.success=False;res.message='Not recording';return res
        self.finish('Stopped by user');res.success=True;res.message='Stopped';return res
    def marker(self,req,res):
        if not self.store.path:res.success=False;res.message='No session';return res
        self.emit('Marker added','INFO',req.label or 'marker');res.success=True;res.message='Marker saved';return res
    def tick(self):
        now=time.monotonic()
        if self.state==CollectionState.COUNTDOWN and now-self.countdown_at>=3.: self.state=CollectionState.RECORDING;self.start_at=now;self.emit('Recording timer started')
        if self.state==CollectionState.RECORDING and now-self.start_at>=self.duration:self.finish()
        m=CollectionState();m.state=self.state;m.session_path=str(self.store.path or '')
        if self.start_at: m.elapsed_sec=float(max(0.,now-self.start_at));m.remaining_sec=float(max(0.,self.duration-m.elapsed_sec))
        try:m.disk_free_gb=os.statvfs(self.store.root).f_bavail*os.statvfs(self.store.root).f_frsize/1e9
        except OSError:m.disk_free_gb=0.
        m.detail='UI-only; no MCAP is created';self.pub.publish(m)
def main():
    rclpy.init();node=CollectionManager()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
