"""The native Params API reports syscall failures by status, not exceptions."""
from unittest.mock import patch

import pytest

from openpilot.common import params as params_module
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.tests.test_angle_autocal import DT, PLATFORM_GAIN_HIGH, _evidenced_pipe


@pytest.mark.parametrize('boolean', [False, True])
@pytest.mark.parametrize('code', [-1, -20])
def test_native_failure_status_is_reported_to_python(tmp_path, boolean, code):
  params = params_module.Params(str(tmp_path / 'params'))
  key = 'FordAngleAutoCal' if boolean else 'FordLowSpeedFactor_ang'
  binding = 'params_put_bool' if boolean else 'params_put'
  with patch.object(params_module, binding, return_value=code):
    with pytest.raises(OSError, match=key):
      if boolean:
        params.put_bool(key, True, block=True)
      else:
        params.put(key, 1.05, block=True)


def test_state_fsync_error_stops_gain_even_if_new_state_reads_back(tmp_path):
  params = params_module.Params(str(tmp_path / 'params'))
  params.put_bool('FordAngleAutoCal', True, block=True)
  ctl = AutoCalController(DT)
  ctl.poll_params(params, 1., 1., PLATFORM_GAIN_HIGH)
  ctl.pipeline = _evidenced_pipe()
  trial = ctl.pipeline.recommend(1., 1.)
  native_put = params_module.params_put

  def failure_after_write(handle, key, value, size, block):
    result = native_put(handle, key, value, size, block)
    return -1 if key == b'FordAngleAutoCalState' and block else result

  with patch.object(params_module, 'params_put', side_effect=failure_after_write):
    assert not ctl._apply_nudge(trial)
  assert params.get('FordLowSpeedFactor_ang', return_default=True) == 1.
  assert params.get('FordHighSpeedFactor_ang', return_default=True) == 1.
  assert 'state save failed' in params.get('FordAngleAutoCalError', block=True)
