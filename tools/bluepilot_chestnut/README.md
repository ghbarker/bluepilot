# BluePilot Ford overlay on sunnypilot Chestnut

This branch ports the pinned BluePilot `bp-dev` Ford controls, vehicle support,
settings, dashboard, sounds, telemetry, and Portal onto the packaged sunnypilot
`staging-chestnut` base. It is a source-build candidate. Target-device installation,
Panda flashing, Chestnut model loading, and driving have not been qualified.

## Pinned inputs

- Release base: `sunnypilot/sunnypilot` staging-chestnut
  `dd29072b71a06c9ce125a481ced431ba995b84f6`.
- Exact source for that release: `a5f44653d7f43ad57fef2f546f3916ec4cbf3c56`.
- Donor: `BluePilotDev/bluepilot` bp-dev
  `e22afa6be9b881fa784c92ebb316db47728a3d81`. No bp-7.0 code is used.
- Current opendbc: `f95f996f5917dcbbf2e32fe51b606a24cf836af6`.
- Donor comparison: main common ancestor
  `01a843e0acbe74d566a7eee9fe0f12f227ae81ed`, with old opendbc
  `b9712d20efd4fb7b1c29378ee153013d2a32a9f1`.

The branch descends from the packaged release. Existing fork branches, including
`llf`, are preserved. `candidate_inventory.json` is the historical screening
inventory, not the final diff; the PR diff is authoritative.

## Integration

The overlay includes Ford curvature- and angle-primary control, pinion measurement,
lane centering, human-turn handling, follow-aware longitudinal control, radar/HUD
extensions, VIN matching and Edge/Mondeo support. BluePilot settings and presentation
extend the current TICI/MICI interfaces. The current Models and sunnylink panels,
Body layouts, model processes, Chestnut loading/fallback code, and AGNOS version
are retained. Portal and its offroad route preprocessor use the new module layout
and typed Params API.

The donor's legacy Portal Settings tab references JSON menus that are absent from
the pinned donor. That pre-existing limitation is retained; Ford settings are
available in the device UI and current sunnylink schema. Portal's parameter API
is adapted and tested, but the missing legacy web menus are not claimed as working.

Driver-monitoring policy and current shared safety/longitudinal disengagement code
are retained. This port does not introduce automatic longitudinal re-engagement
after brake release or an ACC MAIN-OFF exception.

## Safety adaptation

Ford's private BluePilot extension is selected by SP safety bit `0x4000`, which
fits the current signed 16-bit schema. Existing pinion/geometry bits retain their
meaning. Without this flag the current upstream Ford path runs unchanged.

The extension preserves donor four-signal bounds, angle corroboration, shadow
deviation checks, and path-angle rate limits. Actual wire curvature additionally
passes the current curvature safety API, including acceleration, jerk,
measurement, speed consistency, control gating, and transmit-rate limits. Angle
mode keeps zero wire curvature and uses its corroborated shadow measurement, with
an additional current acceleration bound. Shadow telemetry is not a second
actuator rate limiter. Rejected messages do not advance donor actuator history.

The donor's blanket reset bypass is removed. Controller output is adapted to the
current curvature envelope, and control-mode changes send a neutral inactive
frame before activating the new strategy. Current shared `safety/lateral.h` is
unchanged. Passing software tests is not proof of vehicle-level safety or tuning.

## Build and validation

The packaged release omitted its source build definitions. The 25 restored files
in `restored_build_inputs.json` come from exact release source/dependency commits,
including panda `74a0adced421e8b7acd728d0f9988ce225423f13`, msgq
`e7396e76dadbb49e374d4b664ff6bbb43a39bcb0`, and rednose
`28d4a7f69e80e1c3e0d24ca0733d7daeaeade3d0`.
The driver-monitoring ONNX input is restored from the exact source LFS object;
resolved donor sound assets are recorded in `restored_assets.json`.

On Linux, with Python 3.12, uv, Clang, a C/C++ compiler, libffi, and Mesa installed:

```sh
uv sync --locked --extra submodules --extra tools
export PATH="$PWD/.venv/bin:$PATH"
export SKIP_TINYGRAD_COMPILE=1
python tools/bluepilot_chestnut/preflight.py
scons --minimal -j4
uv pip install pytest pytest-xdist scipy pillow ruff ruamel.yaml jsonschema
python -m pytest -q \
  opendbc_repo/opendbc/safety/tests/test_ford.py \
  opendbc_repo/opendbc/safety/tests/test_ford_bluepilot.py \
  opendbc_repo/opendbc/car/ford/tests \
  opendbc_repo/opendbc/sunnypilot/car/ford/tests \
  opendbc_repo/opendbc/sunnypilot/car/tests/test_car_list.py \
  openpilot/selfdrive/ui/bp/tests tools/bluepilot_chestnut
python tools/bluepilot_chestnut/check_python.py
python tools/bluepilot_chestnut/preflight.py --models-only
```

The full minimal native build, including Panda ARM firmware, has passed locally
in Ubuntu 24.04. TICI and MICI layouts have been constructed and rendered under
Xvfb. Hardware networking and camera/vehicle operation were not exercised there.
The workflow repeats the build and focused tests from a clean Linux checkout.

All 18 precompiled 1B driving-model chunks and their manifest are inherited
unchanged and SHA-256 checked against `model_artifacts.json`. Neither local
validation nor CI compiles the 1B model. The small driver-monitoring model and
native consumers are rebuilt; the skip flag deliberately does not skip DM.

The `prebuilt` marker is removed. Launch verifies model integrity, rebuilds native
consumers and Panda for the device, and stops on build failure. Tracked packaged
ARM binaries remain base artifacts until that rebuild; they are not evidence of
a completed overlay firmware flash. Locally produced x86 binaries are not shipped.
Do not re-add `prebuilt` without producing and qualifying a matching ARM package.

Target-device build/flash verification, model loading and fallback, current DM,
brake/cancel/MAIN-OFF behavior, and Ford control traces remain required before
calling this an install-ready driving release. The upstream destructive release
script has not been run, and no device installation has been performed.
