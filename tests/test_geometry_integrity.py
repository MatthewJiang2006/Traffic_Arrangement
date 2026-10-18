import hashlib
import os

import pytest

from logistics_park.runtime.geometry_integrity import geometry_fingerprints, native_dependency_fingerprints


def test_geometry_change_is_detected_even_when_the_map_file_is_unchanged(tmp_path):
    path = tmp_path / 'road.uasset'
    path.write_bytes(b'verified road geometry')
    expected = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()}
    assert geometry_fingerprints(expected) == expected
    path.write_bytes(b'modified road geometry')
    with pytest.raises(RuntimeError, match='differs from verified configuration'):
        geometry_fingerprints(expected)


def test_missing_geometry_asset_is_rejected(tmp_path):
    with pytest.raises(RuntimeError, match='geometry asset is missing'):
        geometry_fingerprints({str(tmp_path / 'missing.uasset'): '0' * 64})


def test_scenarios_without_declared_assets_remain_usable():
    assert geometry_fingerprints({}) == {}


@pytest.mark.parametrize('loaded', ['current', 'replaced_inode', 'deleted', 'missing', 'not_executable'])
def test_native_library_must_be_the_file_actually_loaded_by_the_editor(tmp_path, loaded):
    path = tmp_path / 'AirSim.so'
    path.write_bytes(b'verified synchronized collision snapshot implementation')
    expected = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()}
    stat = path.stat()
    inode = stat.st_ino + (loaded == 'replaced_inode')
    permissions = 'r--p' if loaded == 'not_executable' else 'r-xp'
    name = str(path) + (' (deleted)' if loaded == 'deleted' else '')
    mappings = f'1000-2000 {permissions} 00000000 {os.major(stat.st_dev):x}:{os.minor(stat.st_dev):x} {inode} {name}'
    if loaded == 'missing': mappings = ''
    if loaded == 'current':
        assert native_dependency_fingerprints(expected, mappings=mappings) == expected
    else:
        with pytest.raises(RuntimeError, match='restart editor'):
            native_dependency_fingerprints(expected, mappings=mappings)


def test_changed_native_source_requires_revalidation_even_if_library_is_unchanged(tmp_path):
    source = tmp_path / 'PawnSimApi.cpp'
    source.write_bytes(b'verified source')
    expected = {str(source): hashlib.sha256(source.read_bytes()).hexdigest()}
    assert native_dependency_fingerprints(expected) == expected
    source.write_bytes(b'new source not yet built')
    with pytest.raises(RuntimeError, match='native dependency differs'):
        native_dependency_fingerprints(expected)
