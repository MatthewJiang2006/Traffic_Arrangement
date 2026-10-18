"""Check the native geometry assets declared by a UE run configuration."""
import hashlib
import os
from pathlib import Path


def geometry_fingerprints(expected, *, description='UE geometry asset'):
    actual = {}
    for filename, digest in expected.items():
        path = Path(filename)
        if not path.is_file():
            raise RuntimeError('Required ' + description + ' is missing: ' + filename)
        actual[filename] = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual[filename] != digest:
            raise RuntimeError(description + ' differs from verified configuration: ' + filename)
    return actual


def native_dependency_fingerprints(expected, *, mappings=None):
    """Pin source/binary versions and reject an editor retaining an old .so."""
    actual = geometry_fingerprints(expected, description='UE native dependency')
    if mappings is not None:
        for filename in actual:
            path = Path(filename)
            if path.suffix != '.so':continue
            stat = path.stat()
            loaded = False
            for line in mappings.splitlines():
                fields = line.split(None, 5)
                if len(fields) != 6 or 'x' not in fields[1]:continue
                if fields[5] != str(path):continue
                major, minor = (int(n, 16) for n in fields[3].split(':'))
                loaded = (os.makedev(major, minor) == stat.st_dev and int(fields[4]) == stat.st_ino)
                if loaded:break
            if not loaded:
                raise RuntimeError('UE native library is not the verified loaded file; restart editor: ' + filename)
    return actual
