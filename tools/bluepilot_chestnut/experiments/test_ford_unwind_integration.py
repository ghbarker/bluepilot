"""Offline experiment through the real angle strategy, including reset paths."""
from pathlib import Path
import types
from unittest import mock

import pytest

from opendbc.sunnypilot.car.ford import lateral_angle_ext
from opendbc.sunnypilot.car.ford.tests import test_lateral_angle_ext as f
from tools.bluepilot_chestnut.experiments.ford_unwind import instrument_source


def candidate_harness(cp, variant):
  module = types.ModuleType('offline_unwind_integration')
  source = instrument_source(Path(lateral_angle_ext.__file__).read_text(), variant)
  exec(compile(source, 'offline_unwind_integration', 'exec'), module.__dict__)

  class Candidate(f.LateralCurvExt, module.LateralAngleExt):
    def __init__(self):
      self.CP = cp
      with mock.patch.object(f.lateral_curv_ext.messaging, 'SubMaster', f._FakeSubMaster):
        f.LateralCurvExt.__init__(self, cp, None)
      module.LateralAngleExt.__init__(self, cp)
  return Candidate()


def setup(variant):
  cp = f._explorer_cp()
  candidate = candidate_harness(cp, variant)
  baseline = f._Harness(cp)
  for ext in (candidate, baseline):
    ext.model = f._Model()
    ext.human_turn_detector = f._ForcedDetector(False)
  return cp, candidate, baseline, f._CS(vEgo=5., vEgoRaw=5.)


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('blend', [0., .5, 1.])
@pytest.mark.parametrize('variant', ['proportional', 'net-progress'])
def test_real_strategy_entry_and_constant_curve_match(direction, blend, variant):
  cp, candidate, baseline, cs = setup(variant)
  for desired in [.0001, .0005, .001, .002, .004] + [.004] * 15:
    outputs = []
    for ext in (candidate, baseline):
      ext.path_angle_blend_ratio = blend
      ext.model.orientationRate.z = [direction * (desired + .002) * cs.out.vEgoRaw] * 33
      outputs.append(ext.update_angle_strategy(f._CC(), cs, f._Actuators(direction * desired), cp))
    assert outputs[0] == outputs[1]
    assert candidate.bp_kappa_cmd == baseline.bp_kappa_cmd


@pytest.mark.parametrize('reason', ['inactive', 'override', 'stall', 'invalid', 'dead', 'malformed', 'nonfinite'])
@pytest.mark.parametrize('variant', ['proportional', 'net-progress'])
def test_real_strategy_resets_candidate_state(reason, variant):
  cp, candidate, _, cs = setup(variant)
  candidate.model.orientationRate.z = [.006 * cs.out.vEgoRaw] * 33
  for desired in (.004, .002):
    candidate.update_angle_strategy(f._CC(), cs, f._Actuators(desired), cp)
  assert candidate.unwind_candidate.scale == .5
  cc = f._CC()
  if reason == 'inactive':
    cc.latActive = False
  elif reason == 'override':
    candidate.human_turn_detector = f._ForcedDetector(True)
  elif reason == 'stall':
    candidate.stall_blip_frames_left = 2
  elif reason == 'invalid':
    candidate.sm.valid['modelV2'] = False
  elif reason == 'dead':
    candidate.sm.alive['modelV2'] = False
  elif reason == 'malformed':
    candidate.model.orientationRate.z = [0.]
  else:
    candidate.model.orientationRate.z = [float('nan')] * 33
  candidate.update_angle_strategy(cc, cs, f._Actuators(.001), cp)
  assert candidate.unwind_candidate.scale == 1.
  assert vars(candidate.unwind_candidate) == vars(type(candidate.unwind_candidate)())


def test_instrumentation_refuses_changed_controller_source():
  with pytest.raises(ValueError):
    instrument_source('unexpected controller source')
