"""Publish service events for the optional PIE cargo appearance bridge."""
import json
import os
from pathlib import Path


class CargoVisualEvents:
    def __init__(self, path):
        self.path = Path(path)
        self.cursor = 0
        self.commands = []

    def update(self, events):
        changed = False
        for event in events[self.cursor:]:
            if event['event'] != 'SERVICING' or event.get('operation') not in ('LOADING', 'UNLOADING'):
                continue
            self.commands.append(dict(id=len(self.commands), vehicle=event['vehicle_id'],
                                      operation=event['operation'], site=event['site'],
                                      service_time_s=event['time']))
            changed = True
        self.cursor = len(events)
        if changed:
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'commands': self.commands})+'\n')
            os.replace(temporary, self.path)
