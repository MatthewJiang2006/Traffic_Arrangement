import builtins
import unreal

handle = getattr(builtins, "short_cargo_monitor", None)
if handle is not None:
    unreal.unregister_slate_post_tick_callback(handle)
    builtins.short_cargo_monitor = None
