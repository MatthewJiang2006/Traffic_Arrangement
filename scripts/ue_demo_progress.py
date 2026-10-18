"""Display demo progress in the AirSim UE viewport without recording."""


class DemoProgress:
    def __init__(self, client, total_tasks, total_vehicles):
        self.client = client
        self.total_tasks = total_tasks
        self.total_vehicles = total_vehicles
        self.last_update = -1.0

    def start(self):
        self.client.stopRecording()
        if not self.client.simRunConsoleCommand('EnableAllScreenMessages'):
            raise RuntimeError('UE rejected enabling on-screen progress')
        self.update('PREPARING', force=True)

    def update(self, status, completed=0, moving=0, parked=0, elapsed=0,
               failure='', force=False):
        if not force and elapsed - self.last_update < 1.0:
            return
        minutes, seconds = divmod(int(elapsed), 60)
        text = (
            f'{status} | Tasks {completed}/{self.total_tasks} '
            f'({100 * completed / max(1, self.total_tasks):.0f}%) | '
            f'Moving {moving}/{self.total_vehicles} | '
            f'Parked {parked}/{self.total_vehicles} | '
            f'Elapsed {minutes:02d}:{seconds:02d}'
        )
        if failure:
            text += f' | {failure}'
        # A fixed message key replaces the previous line rather than stacking it.
        self.client.simPrintLogMessage('Factory demo: ', text, 2 if status == 'FAIL' else 0)
        self.last_update = elapsed
