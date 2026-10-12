import pytest
from logistics_park.runtime.run_lock import exclusive_run

def test_same_output_cannot_run_twice_and_lock_is_released_after_error(tmp_path):
    path=tmp_path/'run.lock'
    with pytest.raises(ValueError):
        with exclusive_run(path):
            with pytest.raises(RuntimeError):
                with exclusive_run(path):pass
            with exclusive_run(tmp_path/'independent.lock'):pass
            raise ValueError('simulate interrupted validation')
    with exclusive_run(path):pass
