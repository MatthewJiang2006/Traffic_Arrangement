import sys
from types import SimpleNamespace
import pytest
from logistics_park.adapters.airsim import RPCTransport,_decode_rpc_result


def test_native_collision_label_keeps_invalid_bytes_without_corrupting_fields():
    decoded=_decode_rpc_result({b'has_collided':True,b'object_name':b'TrafficRamp_\xff',b'object_id':107,b'time_stamp':123})
    assert decoded==dict(has_collided=True,object_name='TrafficRamp_\\xff',object_id=107,time_stamp=123,object_name_utf8_valid=False)
    assert decoded['object_name']!='TrafficRamp_'


def test_valid_unicode_and_nested_rpc_state_are_preserved():
    assert _decode_rpc_result({b'object_name':'路面'.encode(),b'rows':[{b'x_val':1.5}]})=={'object_name':'路面','rows':[{'x_val':1.5}]}


@pytest.mark.parametrize('value',[{b'\xff':1},{b'vehicle_name':b'\xff'}])
def test_malformed_protocol_keys_and_other_text_remain_errors(value):
    with pytest.raises(UnicodeDecodeError):_decode_rpc_result(value)


def test_transport_unpacks_binary_before_decoding_complete_result(monkeypatch):
    class Client:
        def __init__(self,address,**kwargs):assert kwargs['unpack_encoding'] is None
        def call(self,method,*args):return {b'object_name':b'Ramp\xff',b'object_id':2}
        def close(self):pass
    monkeypatch.setitem(sys.modules,'msgpackrpc',SimpleNamespace(Client=Client,Address=lambda *a:a))
    t=RPCTransport('localhost',1,2)
    assert t.call('simGetCollisionInfo','truck')=={'object_name':'Ramp\\xff','object_id':2,'object_name_utf8_valid':False}
