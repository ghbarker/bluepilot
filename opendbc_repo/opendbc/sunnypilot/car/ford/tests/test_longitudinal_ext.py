"""Longitudinal extension regression tests, isolated from native messaging/CAN.

Run the production module with the actual Ford scalar limits extracted from
values.py. These tests verify controller decisions, not vehicle dynamics or CAN.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


FORD_DIR = Path(__file__).resolve().parents[1]


def load_longitudinal():
  values_path = FORD_DIR.parents[2] / "car" / "ford" / "values.py"
  values_tree = ast.parse(values_path.read_text(encoding="utf-8"))
  params = next(node for node in values_tree.body if isinstance(node, ast.ClassDef) and node.name == "CarControllerParams")
  names = {"ACCEL_MIN", "ACCEL_MAX", "MIN_GAS", "INACTIVE_GAS"}
  limits = {node.targets[0].id: ast.literal_eval(node.value) for node in params.body
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id in names}
  assert limits.keys() == names
  path = FORD_DIR / "longitudinal_ext.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  tree.body = [node for node in tree.body
               if not (isinstance(node, ast.ImportFrom) and node.module == "opendbc.car.ford.values")]
  namespace = {"CarControllerParams": SimpleNamespace(**limits)}
  exec(compile(tree, str(path), "exec"), namespace)
  return namespace["LongitudinalExt"], namespace["CarControllerParams"]


LongitudinalExt, LIMITS = load_longitudinal()


def make_lead(**kwargs):
  return SimpleNamespace(**({"present": True, "dRel": 65., "vRel": 0., "vLead": 27.} | kwargs))


class RadarMessages(dict):
  def __init__(self, lead):
    super().__init__(radarState=SimpleNamespace(leadOne=lead))
    self.valid = {"radarState": True}
    self.alive = {"radarState": True}
    self.updated = {"radarState": False}


class TestLongitudinalExt(unittest.TestCase):
  def setUp(self):
    self.controller = LongitudinalExt(None, None)
    self.controller.sm = RadarMessages(make_lead())
    self.cc = SimpleNamespace(longActive=True)
    self.cs = SimpleNamespace(out=SimpleNamespace(vEgo=27., gasPressed=False, brakePressed=False))

  def update(self, accel=0.5, gas=0.5, pitch=0., speed=60.):
    return self.controller.update(self.cc, self.cs, accel, gas, pitch, speed, False, 145.)

  def test_current_and_legacy_leads_enable_follow_caps(self):
    for lead in (make_lead(), SimpleNamespace(status=True, dRel=65., vRel=0., vLead=27.)):
      with self.subTest(lead=lead):
        self.controller.sm["radarState"].leadOne = lead
        result = self.update()
        self.assertTrue(result.bp_long_used)
        self.assertEqual(result.gas, 0.2)
        self.assertEqual(result.accel, 0.5)

  def test_present_false_takes_precedence_over_legacy_status(self):
    self.controller.sm["radarState"].leadOne = make_lead(present=False, status=True)
    result = self.update()
    self.assertFalse(result.bp_long_used)
    self.assertEqual(result.gas, 0.5)

  def test_no_lead_preserves_braking_from_first_tick(self):
    for lead in (None, make_lead(present=False)):
      with self.subTest(lead=lead):
        self.controller.sm["radarState"].leadOne = lead
        result = self.update(accel=-1., gas=LIMITS.INACTIVE_GAS)
        self.assertEqual(result.accel, -1.)
        self.assertTrue(result.brake_actuate)
        self.assertFalse(result.bp_long_used)

  def test_unhealthy_radar_falls_back_to_upstream(self):
    for check in (self.controller.sm.valid, self.controller.sm.alive):
      with self.subTest(check=check):
        check["radarState"] = False
        result = self.update()
        self.assertFalse(result.bp_long_used)
        self.assertEqual(result.gas, 0.5)
        self.assertEqual(self.update(accel=-1.).accel, -1.)
        check["radarState"] = True

  def test_healthy_radar_remains_usable_between_updates(self):
    self.assertFalse(self.controller.sm.updated["radarState"])
    self.assertTrue(self.update().bp_long_used)

  def test_malformed_lead_values_fall_back(self):
    for field in ("dRel", "vRel", "vLead"):
      for value in (float("nan"), float("inf"), -float("inf")):
        with self.subTest(field=field, value=value):
          self.controller.sm["radarState"].leadOne = make_lead(**{field: value})
          result = self.update()
          self.assertFalse(result.bp_long_used)
          self.assertEqual(result.gas, 0.5)
    for lead in (make_lead(dRel=0.), make_lead(dRel=-1.), SimpleNamespace(present=True)):
      with self.subTest(lead=lead):
        self.controller.sm["radarState"].leadOne = lead
        self.assertFalse(self.update().bp_long_used)

  def test_follow_comfort_never_weakens_upstream_braking(self):
    # Long TTC previously applied only -0.002 on the first -1.0 braking request.
    for lead in (make_lead(dRel=100., vRel=-1.), make_lead(dRel=30., vRel=-0.2), make_lead()):
      with self.subTest(lead=lead):
        self.controller.sm["radarState"].leadOne = lead
        for accel in (-1., -0.1, -2.5, 0., -0.7):
          self.assertEqual(self.update(accel=accel).accel, accel)

  def test_follow_caps_never_increase_gas_or_reactivate_inactive_gas(self):
    for lead in (make_lead(), make_lead(dRel=30., vRel=-1.)):
      for gas in (LIMITS.INACTIVE_GAS, -0.4, 0., 0.1, 0.5):
        with self.subTest(lead=lead, gas=gas):
          self.controller.sm["radarState"].leadOne = lead
          result = self.update(gas=gas)
          self.assertLessEqual(result.gas, gas)
          if gas == LIMITS.INACTIVE_GAS:
            self.assertEqual(result.gas, gas)

  def test_gas_caps_for_closing_and_pacing_but_not_trailing_lead(self):
    for lead, expected_gas in ((make_lead(dRel=30., vRel=-1.), 0.),
                                (make_lead(dRel=100., vRel=-1.), 0.5),
                                (make_lead(vRel=0.), 0.2),
                                (make_lead(vRel=1.), 0.5)):
      with self.subTest(lead=lead):
        self.controller.sm["radarState"].leadOne = lead
        self.assertEqual(self.update().gas, expected_gas)

  def test_brake_and_precharge_hysteresis_persist_until_release(self):
    results = [self.update(accel=a) for a in (-0.2, -0.1, -0.07, -0.05)]
    self.assertEqual([r.brake_actuate for r in results], [True, True, True, False])
    self.assertEqual([r.precharge_actuate for r in results], [True, True, True, False])
    self.assertTrue(all(r.gas == LIMITS.INACTIVE_GAS for r in results[:3]))

  def test_precharge_can_latch_before_braking(self):
    results = [self.update(accel=a) for a in (-0.13, -0.10, -0.05)]
    self.assertEqual([r.brake_actuate for r in results], [False, False, False])
    self.assertEqual([r.precharge_actuate for r in results], [True, True, False])

  def test_pitch_compensation_is_consistent_with_upstream_brake_decision(self):
    result = self.update(accel=-0.2, pitch=0.2)
    self.assertFalse(result.brake_actuate)
    self.assertFalse(result.precharge_actuate)
    result = self.update(accel=0., pitch=-0.2)
    self.assertTrue(result.brake_actuate)
    self.assertTrue(result.precharge_actuate)

  def test_radar_loss_and_recovery_do_not_release_brakes_in_deadband(self):
    self.assertTrue(self.update(accel=-0.2).brake_actuate)
    self.controller.sm.alive["radarState"] = False
    fallback = self.update(accel=-0.1)
    self.assertTrue(fallback.brake_actuate)
    self.assertTrue(fallback.precharge_actuate)
    self.assertFalse(fallback.bp_long_used)
    self.controller.sm.alive["radarState"] = True
    recovered = self.update(accel=-0.1)
    self.assertTrue(recovered.brake_actuate)
    self.assertTrue(recovered.precharge_actuate)
    self.assertTrue(recovered.bp_long_used)

  def test_inactive_and_pedal_overrides_reset_state_and_reporting(self):
    for target, attribute in ((self.cc, "longActive"), (self.cs.out, "gasPressed"), (self.cs.out, "brakePressed")):
      with self.subTest(attribute=attribute):
        self.assertTrue(self.update(accel=-0.2).brake_actuate)
        setattr(target, attribute, attribute != "longActive")
        result = self.update(accel=-0.1)
        self.assertFalse(result.bp_long_used)
        self.assertFalse(result.brake_actuate)
        self.assertFalse(result.precharge_actuate)
        self.assertFalse(self.controller._bp_long_active_last)
        setattr(target, attribute, attribute == "longActive")
        resumed = self.update(accel=-0.1)
        self.assertFalse(resumed.brake_actuate)
        self.assertFalse(resumed.precharge_actuate)

  def test_inactive_always_sends_inactive_gas(self):
    self.cc.longActive = False
    self.assertEqual(self.update().gas, LIMITS.INACTIVE_GAS)

  def test_speed_hysteresis_and_slow_lead_fallback(self):
    self.assertFalse(self.update(speed=47.).bp_long_used)
    self.assertTrue(self.update(speed=51.).bp_long_used)
    self.assertTrue(self.update(speed=47.).bp_long_used)
    self.assertFalse(self.update(speed=44.).bp_long_used)
    self.assertFalse(self.update(speed=47.).bp_long_used)
    self.controller.sm["radarState"].leadOne = make_lead(vLead=15.)
    result = self.update()
    self.assertFalse(result.bp_long_used)
    self.assertEqual(result.gas, 0.5)

  def test_disabled_falls_back_without_losing_brake_latch(self):
    self.assertTrue(self.update(accel=-0.2).brake_actuate)
    self.controller.disable_BP_long_UI = True
    result = self.update(accel=-0.1)
    self.assertFalse(result.bp_long_used)
    self.assertTrue(result.brake_actuate)
    self.assertTrue(result.precharge_actuate)
    result = self.update()
    self.assertEqual(result.gas, 0.5)

  def test_existing_acceleration_limits_and_passthrough_fields(self):
    for accel, expected in ((10., LIMITS.ACCEL_MAX), (-10., LIMITS.ACCEL_MIN)):
      with self.subTest(accel=accel):
        result = self.update(accel=accel)
        self.assertEqual(result.accel, expected)
        self.assertEqual(result.accel_pred_send, LIMITS.INACTIVE_GAS)
        self.assertEqual(result.target_speed, 145.)
        self.assertFalse(result.stopping)


if __name__ == "__main__":
  unittest.main()
