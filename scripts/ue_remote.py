import sys,time,socket,json
sys.path.insert(0,"/home/aipc/UnrealEngine-4.27/Engine/Plugins/Experimental/PythonScriptPlugin/Content/Python")
original_loads=json.loads
def loads(*args,**kwargs):
    kwargs.pop("encoding",None)
    return original_loads(*args,**kwargs)
json.loads=loads
import remote_execution as re
config=re.RemoteExecutionConfig()
with socket.socket() as s:
    s.bind(("127.0.0.1",0));config.command_endpoint=("127.0.0.1",s.getsockname()[1])
r=re.RemoteExecution(config);r.start()
try:
    for i in range(30):
        if r.remote_nodes:break
        time.sleep(.1)
    if not r.remote_nodes:raise RuntimeError("No UE Python remote node")
    r.open_command_connection(r.remote_nodes[0]["node_id"])
    print(json.dumps(r.run_command(sys.argv[1],raise_on_failure=True),ensure_ascii=False))
finally:r.stop()
