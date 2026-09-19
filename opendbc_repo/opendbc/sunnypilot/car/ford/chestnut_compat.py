"""Final Ford CAN curvature envelope for the Chestnut safety API."""
import numpy as np

from opendbc.car.ford.values import CarControllerParams


def limit_curvature_for_chestnut(requested, previous, measured, speed, active):
  # The BP strategy runs first, with the previous value actually sent on CAN.
  # The final command must also satisfy the new base's measured-error, lateral
  # acceleration and per-frame jerk limits, including after driver overrides.
  if speed > 9:
    requested = float(np.clip(requested, measured - CarControllerParams.CURVATURE_ERROR,
                              measured + CarControllerParams.CURVATURE_ERROR))
  return CarControllerParams.CURVATURE_LIMITS.apply_limits(
    requested, previous, speed, 0., active, CarControllerParams.STEER_STEP)
