import builtins
import unreal
for name in ('Car_0_0','Car_0_1','Car_0_2','Car_1_0'):
    key='four_monitor_'+name
    handle=getattr(builtins,key,None)
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        setattr(builtins,key,None)
