"""Compare isolated unwind experiment with current and incident controllers.

Latest published inputs are not exact subscriber inputs; recorded vehicle motion
does not change with the alternative command. This is not a trajectory replay.
"""
import json
import argparse
import hashlib
import statistics
import subprocess
import sys
import types
from pathlib import Path
from unittest import mock

import numpy as np
import zstandard
from openpilot.cereal import log
from opendbc.sunnypilot.car.ford.tests import test_lateral_angle_ext as f
from tools.bluepilot_chestnut.experiments.ford_unwind import instrument_source

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input-dir', type=Path, required=True, help='Directory containing route--segment.rlog.zst files')
parser.add_argument('--output-dir', type=Path, required=True)
parser.add_argument('--route', required=True)
parser.add_argument('--segments', type=int, nargs='+', required=True)
parser.add_argument('--incident-ref', required=True, help='Local git revision running during the captured incident')
parser.add_argument('--candidate', choices=('proportional', 'net-progress'), required=True)
args = parser.parse_args()
HERE = args.input_dir.resolve()
OUTPUT = args.output_dir.resolve()
OUTPUT.mkdir(parents=True, exist_ok=True)
ROOT = Path(__file__).resolve().parents[3]
ANGLE = 'opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py'
modules = {}
candidate_name = args.candidate
output_prefix = 'unwind-candidate' if candidate_name == 'proportional' else 'unwind-recovery'
current_source = (ROOT / ANGLE).read_text()
for name, source in (
  ('incident', subprocess.check_output(['git', 'show', args.incident_ref + ':' + ANGLE], cwd=ROOT).decode()),
  ('baseline', current_source),
  ('candidate', instrument_source(current_source, candidate_name)),
):
  module = types.ModuleType('unwind_replay_' + name)
  exec(compile(source, ANGLE, 'exec'), module.__dict__)
  modules[name] = module


class LiveSM(f._FakeSubMaster):
  def __init__(self):
    super().__init__()
    self.data = {}

  def __getitem__(self, key):
    return self.data[key]


def controller(module, cp, cp_sp):
  class Harness(f.LateralCurvExt, module.LateralAngleExt):
    def __init__(self):
      self.CP = cp
      with mock.patch.object(f.lateral_curv_ext.messaging, 'SubMaster', f._FakeSubMaster):
        f.LateralCurvExt.__init__(self, cp, cp_sp)
      module.LateralAngleExt.__init__(self, cp, cp_sp)
  return Harness()


def log_path(segment):
  flat = HERE / f'{args.route}--{segment}.rlog.zst'
  return flat if flat.exists() else HERE / f'{args.route}--{segment}' / 'rlog.zst'


def load(segment):
  path = log_path(segment)
  with path.open('rb') as source, zstandard.ZstdDecompressor().stream_reader(source) as stream:
    return sorted(log.Event.read_multiple_bytes(stream.read()), key=lambda event: event.logMonoTime)


KEYS = ('FordHighSpeedDampening_ang', 'FordLowSpeedFactor_ang', 'FordHighSpeedFactor_ang',
        'lane_change_factor_high_ang', 'custom_path_offset_ang', 'lane_centering_strength_ang', 'enable_lane_positioning_ang')
CAPTURE = ('desired_curvature', 'predicted_curvature', 'requested_curvature', 'nominal_request',
           'unwind_scale', 'kappa_cmd', 'current_curvature', '_kappa_cmd_pre_error_clip', 'path_angle')
summaries = []
for segment in args.segments:
  events = load(segment)
  cp = next(e.carParams for e in events if e.which() == 'carParams')
  cp_sp = next(e.carParamsSP for e in events if e.which() == 'carParamsSP')
  initial = next(e.initData for e in events if e.which() == 'initData')
  start = min(e.logMonoTime / 1e9 for e in events if e.which() == 'carState')
  if log_path(segment - 1).exists():
    events = sorted(load(segment - 1) + events, key=lambda e: e.logMonoTime)
  values = {p.key: bytes(p.value).decode() for p in initial.params.entries if p.key in KEYS}
  values['enable_lane_positioning_ang'] = values.get('enable_lane_positioning_ang') == '1'
  values['FordAngleAutoCal'] = False  # Hold recorded gains. No parameter writes.
  controllers = {name: controller(module, cp, cp_sp) for name, module in modules.items()}
  for ext in controllers.values():
    ext.update_angle_params(f._FakeParams(values))
    ext.sm = LiveSM()
  last, capture, rows = {}, {}, []
  previous_frame = None

  def trace(frame, event, arg):
    if event == 'return' and frame.f_code.co_name == 'update_angle_strategy':
      capture.update({key: float(value) for key, value in frame.f_locals.items() if key in CAPTURE})

  for event in events:
    kind = event.which()
    if kind in ('carState', 'carControl', 'modelV2', 'vehicleParameters', 'lateralDelay', 'selfdriveState'):
      last[kind] = getattr(event, kind)
    if kind != 'controllerStateBP' or not all(k in last for k in ('carState', 'carControl', 'modelV2', 'vehicleParameters', 'lateralDelay')):
      continue
    bp = event.controllerStateBP
    frame = bp.fordSteeringCommand.frame
    if frame == previous_frame:
      continue
    previous_frame = frame
    cs = types.SimpleNamespace(out=last['carState'])
    cc = last['carControl']
    row = {'mono': event.logMonoTime / 1e9, 'frame': frame, 'mph': cs.out.vEgoRaw / .44704,
           'recordedWire': bp.fordSteeringCommand.pathAngle, 'driverPressed': cs.out.steeringPressed, 'variants': {}}
    for name, ext in controllers.items():
      ext.sm.data = last
      ext.model, ext.lp = last['modelV2'], last['vehicleParameters']
      ext.VM.update_params(max(ext.lp.stiffnessFactor, .1), max(ext.lp.steerRatio, .1))
      ext.low_speed_curv_factor = bp.bmsLowSpeedAdjustmentFactor
      ext.high_speed_curv_factor = bp.bmsHighSpeedAdjustmentFactor
      capture.clear()
      sys.setprofile(trace)
      try:
        out = ext.update_angle_strategy(cc, cs, cc.actuators, cp)
      finally:
        sys.setprofile(None)
      row['variants'][name] = {**capture, 'wireAngle': -out.path_angle,
        'mode0': ext.angle_stall_blip_active or ext.angle_human_turn_active or not cc.latActive,
        'deviationLimited': ext.bp_curvature_deviation_limited, 'rateLimited': ext.bp_angle_rate_limited}
    if row['mono'] >= start:
      rows.append(row)

  # Skip initial two seconds in summary comparisons, but preserve every row.
  comparisons = rows[40:]
  errors = [abs(r['recordedWire'] - r['variants']['incident']['wireAngle']) for r in comparisons]
  active = changed = extra_steps = new_mode_zero = growing_entry_changed = entry = 0
  maximum_extra_step = 0.
  previous = None
  for row in comparisons:
    base, candidate = (row['variants'][key] for key in ('baseline', 'candidate'))
    if candidate['mode0'] != base['mode0']:
      new_mode_zero += 1
    if not base['mode0'] and not candidate['mode0'] and 'desired_curvature' in base:
      active += 1
      changed += abs(base['wireAngle'] - candidate['wireAngle']) > .0005
      if previous is not None and row['frame'] - previous['frame'] == 5:
        pb, pc = (previous['variants'][key] for key in ('baseline', 'candidate'))
        if not pb['mode0'] and not pc['mode0'] and 'desired_curvature' in pb:
          extra = abs(candidate['wireAngle'] - pc['wireAngle']) - abs(base['wireAngle'] - pb['wireAngle'])
          extra_steps += extra > .005
          maximum_extra_step = max(maximum_extra_step, extra)
          is_entry = base['desired_curvature'] * pb['desired_curvature'] > 0 and abs(base['desired_curvature']) > abs(pb['desired_curvature']) + .00005
          entry += is_entry
          growing_entry_changed += is_entry and abs(base['wireAngle'] - candidate['wireAngle']) > .0005
    previous = row
  summary = dict(segment=segment, rows=len(rows), active=active, changedWireAboveOneCanUnit=changed,
                 modeDifferences=new_mode_zero, wireStepIncreaseAbove5mrad=extra_steps,
                 largestAdditionalWireStep=maximum_extra_step, growingPlannerFrames=entry,
                 changedGrowingPlannerFrames=growing_entry_changed,
                 incidentMeanReconstructionError=statistics.mean(errors), incidentP95ReconstructionError=float(np.percentile(errors, 95)))
  summaries.append(summary)
  (OUTPUT / f'{output_prefix}-replay-{segment}.json').write_text(json.dumps({'summary': summary, 'rows': rows}, indent=2))
  print(json.dumps(summary), flush=True)

(OUTPUT / f'{output_prefix}-replay-summary.json').write_text(json.dumps({
  'candidate': candidate_name + '; known release-blocking acceptance failures',
  'sourceHead': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT).decode().strip(),
  'angleSourceSha256': hashlib.sha256(current_source.encode()).hexdigest(),
  'incidentRef': args.incident_ref,
  'route': args.route,
  'scope': 'Latest-published recorded inputs; fixed recorded vehicle motion and gains; no trajectory prediction or production enabling.',
  'reportingThresholds': {'wireChange': .0005, 'extraWireStep': .005, 'plannerRise': .00005},
  'segments': summaries,
}, indent=2))
