"""Reject concurrent validators that would mutate the same run or UE scene."""
from contextlib import contextmanager
import fcntl
from pathlib import Path


@contextmanager
def exclusive_run(path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('a') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise RuntimeError('Another validation already owns '+str(path)) from exc
        try:yield
        finally:fcntl.flock(stream,fcntl.LOCK_UN)
