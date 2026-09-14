import json, re
from datetime import datetime
from pathlib import Path
import yaml

def safe_name(value):
    value=re.sub(r'[^A-Za-z0-9_.-]+','_',value.strip()).strip('._')
    return value or 'session'

class SessionStore:
    def __init__(self, root): self.root=Path(root); self.path=None; self.events=[]
    def create(self, cfg):
        self.events=[]
        stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path=self.root/f'{stamp}_{safe_name(cfg["session_name"])}'
        for d in ('config','calibration','reports','markers','preview'):
            (self.path/d).mkdir(parents=True,exist_ok=True)
        now = datetime.now().astimezone().isoformat()
        cfg={**cfg,'session_id':self.path.name,
             'mode':'ui_only' if cfg['ui_only'] else 'recording',
             'contains_real_sensor_data':False,'contains_mcap':False,
             'contains_rosbag':False,'created_at':now,'start_time':None,
             'end_time':None,'calibration':str(self.path/'calibration')}
        self._write_metadata(cfg)
        self.write_event('INFO','Preflight completed')
        return self.path
    def update_config(self, **updates):
        if not self.path: return
        path=self.path/'metadata.yaml'
        cfg=yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        cfg.update(updates)
        self._write_metadata(cfg)
    def _write_metadata(self, cfg):
        text=yaml.safe_dump(cfg,sort_keys=False)
        (self.path/'metadata.yaml').write_text(text,encoding='utf-8')
        # Keep the original filename for clients that already consume it.
        (self.path/'session.yaml').write_text(text,encoding='utf-8')
    def write_event(self, level, message, marker=''):
        event={'timestamp':datetime.now().astimezone().isoformat(),'level':level,'message':message,'marker':marker}
        self.events.append(event)
        if self.path: (self.path/'markers'/'events.json').write_text(json.dumps(self.events,indent=2),encoding='utf-8')
