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
        for d in ('raw','config','calibration','reports','markers'): (self.path/d).mkdir(parents=True,exist_ok=True)
        cfg={**cfg,'mode':'ui_only' if cfg['ui_only'] else 'recording','contains_real_sensor_data':False,'contains_mcap':False,'created_at':datetime.now().astimezone().isoformat()}
        (self.path/'session.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False),encoding='utf-8')
        self.write_event('INFO','Preflight completed')
        return self.path
    def write_event(self, level, message, marker=''):
        event={'timestamp':datetime.now().astimezone().isoformat(),'level':level,'message':message,'marker':marker}
        self.events.append(event)
        if self.path: (self.path/'markers'/'events.json').write_text(json.dumps(self.events,indent=2),encoding='utf-8')
