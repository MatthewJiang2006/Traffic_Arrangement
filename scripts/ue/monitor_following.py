"""Install an independent full-body/tire monitor for each moving test truck."""
import builtins
from pathlib import Path
import runpy
for name in ('Car_0_0', 'Car_0_1'):
    builtins.short_cargo_monitor_request = {'vehicle': name, 'prefix': 'following_'+name,
                                          'handle': 'following_monitor_'+name}
    runpy.run_path(str(Path(__file__).with_name('monitor_short_cargo.py')))
del builtins.short_cargo_monitor_request
