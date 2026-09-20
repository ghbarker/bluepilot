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
Body layouts, model processes, Chestnut inference/fallback code, and AGNOS version
are retained. The startup readiness correction below is explicitly pinned by
preflight. Portal and its offroad route preprocessor use the new module layout
and typed Params API.

The donor's legacy Portal Settings tab references JSON menus that are absent from
the pinned donor. That pre-existing limitation is retained; Ford settings are
available in the device UI and current sunnylink schema. Portal's parameter API
is adapted and tested, but the missing legacy web menus are not claimed as working.

The Ford Edge legacy fingerprint `FORD EDGE 2ND GEN` maps to `FORD_EDGE_MK2`.
The generated vehicle menu restores the donor's 2019-24 label. This is a menu and
saved-identity correction; the existing VIN matching rules and actuator parameters
are unchanged, and it does not establish qualification for additional model years.

## Restored supporting features

Device startup uses the BP build progress and error screen, adapted to both display
sizes with touch scrolling. Before compiling, the build entry points synchronize
the Python environment when `uv.lock` changes. The successful lock digest is stored
inside the project venv (including an explicit `UV_PROJECT_ENVIRONMENT`). Sync uses
`uv sync --frozen --inexact`, retains non-conflicting extras, and records success
only after completion. A failed update stops startup with an error and is retried
on the next launch; it never proceeds to manager with a knowingly failed sync.

GPS setup requests Quectel multi-constellation configuration and reads it back.
The donor used `gnssconfig=4` despite its all-constellations comment; the EC25/EG25
and EM12 documentation specifies `1` for GPS, GLONASS, Galileo and BeiDou. An
unsupported optional setting is reported without preventing GNSS startup. Modem
diagnostic disconnects, including EOF from the current serial library, close the
old port and retry setup; failed setup attempts release their exclusive handle.
Actual modem firmware acceptance and reacquisition still require device testing.
See the [Quectel GNSS command manual](https://forums.quectel.com/uploads/short-url/abgizBnqe7XbS8LCbfj0erAtZSu.pdf)
and [uv synchronization reference](https://docs.astral.sh/uv/concepts/projects/sync/).

The three offline Ford tools live under `openpilot/tools` and run as modules:

```sh
python -m openpilot.tools.ford_lmc_safety_replay '<dongleid>|<route>'
python -m openpilot.tools.ford_yaw_health_check '<dongleid>|<route>'
FORD_REPLAY_DONGLE_ID='<dongleid>' python -m openpilot.tools.ford_pinion_replay '<route>'
```

The safety replay defaults to the current compiled Ford hooks and supports CAN and
CAN FD. It preserves recorded safety parameters and explicitly enables the private
BluePilot flag. Missing SP metadata requires `--param-sp`; do not guess the pinion
geometry bits. Full rlogs are required. The `--legacy-bpdev` mode restores the
historical Explorer CAN simulator, including the donor's removed reset bypass;
its per-check report and optional JSON are not current firmware safety verdicts.
The separate pinion replay also models only the historical Explorer CAN geometry
and rejects other platforms. Missing TX echoes alone do not prove a safety block.
The yaw tool reports consistency or suspected faults and treats absent independent
IMU/turning evidence as inconclusive; it does not automatically recommend a sensor
toggle or certify sensor health. No real-route qualification is claimed here.

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

### Steering review, 2026-09-19

Independent review and compiled-hook reproductions found two defects:

- A Ford-local rejection could advance the shared curvature history even though
  the complete command never reached the EPS. At 25 m/s, five rejected commands
  could prepare an accepted 90-CAN-unit step from zero; a direct step was blocked.
  Ford-local rejections now restore that history when the shared curvature check
  itself passed. The shared check's own resets and transmit counters are retained.
- The angle strategy used asymmetric wire bounds before the controller negated
  its result. An internal +0.5096 rad could wrap from requested -0.5096 to encoded
  +0.5145 rad and be persistently rejected. Internal limits now invert the unchanged
  DBC range, including the hard-saturation thresholds. Real CAN and CAN FD packing
  and compiled safety reproduced the failure and accepted the corrected sequence.

The numerical firmware limits were not changed by these fixes. A compiled sweep
against the pinned BP donor found a mixed envelope, not general equivalence:

- At 5 m/s, the largest accepted curvature step from zero was 0.00252 1/m in BP
  and 0.01122 1/m in this branch, on CAN and CAN FD.
- At 25 m/s on CAN FD, the accepted steady curvature ceiling was 0.00386 1/m in
  BP and 0.00624 1/m here. At that speed the step from zero was 0.00032 1/m in both.
- At 35 m/s, the step from zero decreased from 0.00020 to 0.00016 1/m. Classic
  CAN also gains the current speed-dependent absolute cap that the donor lacked.

These are isolated firmware acceptance probes with initialized measurements and
command history, not vehicle acceleration measurements. The new cap uses a speed
tolerance, so its accepted curvature ceiling is not simply 3.6 divided by the
reported speed squared. The final Python adapter can increase the donor's result
to track measured curvature: in a synthetic 25 m/s CAN FD case it changed
0.00385824 to 0.00435824 1/m. The older nominal cap is therefore not a final bound.

One inherited behavior remains unqualified: after a 0.6-second driver steering
press in a synthetic 25 m/s curve (curvature 0.003 1/m), the proactive stall blip
produced six inactive frames, or 300 ms, followed by a ramp from zero. Its
path-angle threshold does not establish straight driving. The behavior is
unchanged; the vehicle's response and the reset's benefit require recorded-route,
bench, and controlled vehicle evidence before fleet release.

## Build and validation

### Owner review, 2026-09-19

**Release blocked.** This review does not establish vehicle safety or compliance
with [comma's fork safety policy](https://docs.comma.ai/concepts/safety/).
It supersedes any inference of readiness from installation or green CI.

### Command-consistency repair, 2026-09-20

The angle controller now intersects its existing learned-measurement command band
with the firmware's fixed-geometry pinion band (0.003 1/m). It uses an unlearned
vehicle model, raw pinion angle and raw speed for this second reference. It limits
the curvature input used to calculate the actual path-angle request, not only the
shadow value. Requests inside both bands retain their original values. If the
bands do not overlap, the original request remains subject to the unchanged
firmware rejection and takeover path; the controller does not invent a matching
shadow value or widen either band. Learned driving parameters remain in use.

On ticks where both LKA status and LMC steering messages are due, status is queued
first. The existing 33.3 Hz and 20 Hz schedules and message counts are unchanged.
Other tick phases still use the previously latched status; this is not an atomic
protocol or a guarantee against delayed or dropped CAN messages.

Private offline replay covered 23 captured segments in four contiguous blocks
(one recording has a missing segment). It fed recorded CAN inputs through the
unchanged compiled safety hooks while regenerating the actual angle-controller
requests. The original recorded stream and the reconstructed old controller both
reproduced the 30-command pinion-deviation rejection sequence. The corrected
controller produced zero rejections in that block. Across 26,130 regenerated
steering frames, 34 actual command payloads changed, all in the affected block.
The largest path-angle change in the recorded failure window was 0.007 rad of
encoded path-angle input; this is not a steering-wheel angle measurement.

The other blocks retained their independent rejection cases. Fresh status made
the existing acceleration cap reject one frame earlier during a driver steering
press: the shadow value was 404 CAN units against a cap of 376. That protection
was retained. No safety limit, safety-hook source, driver-monitoring policy,
engagement permission, takeover alert or model artifact changes are included.

This is counterfactual command-acceptance evidence, not a vehicle simulation or
device validation. The reconstructed baseline differs from some recorded frames
because process scheduling and internal state are not reproduced exactly. The
comparison uses the same reconstruction for old and corrected controllers, and
the original stream separately reproduces the target fault. It does not prove
that Ford's later ramp-out would have been prevented. Private recordings are not
included in the repository.

Regression tests exercise both steering directions, unchanged in-band requests,
all supported pinion geometries, fixed versus learned calibration, non-overlapping
bands, stale values, the independent acceleration cap, and all 15 relative
message-schedule phases. The full vehicle safety suite and C safety checks remain
required. The qualification gap described below still blocks a blanket claim of
comma-policy compliance or readiness for driving/fleet release.

The firmware checks curvature, path angle, path offset and curvature-rate inputs
separately. In angle mode the additional curvature check reads host-supplied shadow
telemetry; it does not derive that curvature from the actual path-angle command.
The mode flag and shadow come from the same host that produces steering commands.
They therefore do not independently establish the steering command's effect.
In an isolated command-acceptance test at 25 m/s, with zero measured, wire and
shadow curvature, accepted 0.0005-rad path-angle steps reached +/-0.25 rad in
curvature mode and +/-0.5 rad in angle mode, on both CAN and CAN FD. No nonzero
history was pre-seeded. These are encoded path-angle inputs, not steering-wheel
angles. The test does not simulate a moving vehicle or RX liveness and does not
establish a resulting lateral acceleration. It demonstrates why the curvature
check alone cannot qualify all four steering inputs. This design is inherited
from BP-DEV; retaining it is not independent safety evidence.

The pinned Cppcheck 2.21.0 package (comma dependencies commit
`b7253ddb101ed6add04fc2a2ce0688d5e21a55ee`) was run with the repository's MISRA
options on isolated safety source from the pinned base and `d78cc7a358`.
The base passed with zero findings. This branch returned exit 2 with 23 findings:
21 MISRA findings (block scope, composite casts and multiple returns) and two
constant-condition findings. These are coding-check failures, not 23 demonstrated
vehicle-control failures. These findings are now corrected with block-local
constants, explicit conversion intermediates, single-return dispatch and
compile-time debug configuration. BP steering modes, limits and warning thresholds
are preserved. The unchanged analyzer now reports zero findings. A new workflow
step also requires the analyzer to reject deliberately reintroduced Ford composite
casts and early returns; it checks the pinned MISRA coverage table and uses the
same analysis options and suppressions as the upstream safety script. Both release
and debug configurations are now checked explicitly. The debug configuration also
exposed an inherited Rivian signed flag-type mismatch; matching its type to the
unsigned safety parameter corrects that diagnostic without changing the flag value
or actuation limits.

An isolated before/after comparison of compiled release and debug hooks matched
all 65,536 shadow conversion values in each build, 24,000 metadata frames, 24,000
RX/TX frames and 128,000 steering frames with checked state histories. This is
regression evidence for the refactor, not proof that the inherited steering
envelope is safe. The local safety and Ford controller suites passed 8,103 tests
and 17,183 subtests, with 3,399 skips.

For context, CI at `d78cc7a358` passed 8,274 tests and 17,183 subtests, with 3,405
skips. That earlier result excluded the static-analysis failures and did not
resolve the auxiliary-actuator qualification gap. The unchanged shared
`safety/lateral.h` and driver-monitoring source were also checked. Preserving
those files does not prove that all extensions respect their intended safeguards.

The Ford angle controller's warning logic matches the BP-DEV donor, and the
steering-required event block matches the pinned upstream base. The warning
requires sustained steering tracking error or curvature clipping, more than 20%
undershoot, desired lateral acceleration above 1 m/s^2 and no recent driver
steering input. Completing a turn later does not establish that an earlier
warning was spurious. No warning threshold or actuator limit was changed by the
display corrections. A recording of the reported warning is still needed.

The arc estimates steering effort against `CarParams.maxLateralAccel`; for the
Mach-E the inherited table uses a guessed 1.5 m/s^2. It is not measured EPS torque,
remaining steering range or a common scale for all firmware limits. The review
reproduced a wrong-direction display when road-bank compensation exceeded a small
turn demand, and stale direction after a reversal. Corrections constrain the
display to the requested direction, clear invalid/dead/non-finite readings,
handle curvature-state messages, and explain the estimate in settings. Rendering
tests cover both display sizes, turn directions, reversals, data loss and invalid
normalization. Accurate physical saturation would require validated vehicle
feedback and controller telemetry, including limits on the auxiliary inputs.

Ford limit-status feedback is documented in the
[upstream DBC](https://github.com/commaai/opendbc/blob/master/opendbc/dbc/ford_lincoln_base_pt.dbc):
`Lane_Assist_Data3_FD1.LatCtlLim_D_Stat` encodes `LimitNotReached` (0),
`LimitClose` (1), `LimitReached` (2), and `LimitWithDriverActive` (3).
These are reported states, not a continuous percentage of remaining capacity.
Before the display correction below, the CAN FD parser already received the
enclosing message, but the arc did not consume this field. The angle strategy
still reads `CS.lat_ctl_lim_stat` with a zero
fallback, while neither this branch nor the pinned BP donor assigns that attribute
in production Python. Therefore the current fallback cannot establish that the
vehicle is reporting zero, and the inherited comment that the signal does not fire
in angle mode is not independent evidence of its behavior on this Mach-E.
Recorded CAN and active-mode correlation are still needed. Raw recordings remained
on the unreachable comma during this check; earlier local diagnostic summaries
did not retain this signal. No limit-feedback wiring or control behavior was
changed by that documentation review.

The subsequent display correction publishes the received Ford CAN FD limit state
and its original CAN timestamp in `carStateBP.fordSteeringLimit`. It deliberately
does not assign the controller's `CS.lat_ctl_lim_stat` attribute or change steering
commands, firmware limits, or takeover events. The display rejects missing,
invalid, future-dated, and older-than-150-ms feedback, and only interprets it while
lateral control is active and the PSCM reports continuous control in progress.
Republishing cached CAN values cannot refresh their age.

On Ford, amber now means the PSCM reports `LimitClose`; red means `LimitReached`
or `LimitWithDriverActive`. A visible label distinguishes these conditions.
Zero/missing/unsupported feedback displays `DEMAND / CAPACITY UNKNOWN` in neutral
colors. The estimated demand length no longer selects Ford limit colors, including
with a colored theme. This is a reported-limit indicator, **not a calibrated
remaining-capacity gauge**. Arc length still represents estimated demand.

There is documented capacity information: upstream `car/ford/values.py` describes
an approximate 2.0 m/s^2 EPS curvature limit and speed-dependent curvature rate.
That does not establish a combined-input saturation scale for BP path-angle mode.
The BP angle controller references `bluepilot/agent_info/20_FORD_PSCM_KNOWLEDGE_PACK.md`,
but that document was absent from the checked local history and current upstream
BP tree. Vehicle/mode validation and the applicable calibration evidence remain
necessary before claiming continuous physical headroom. These display changes
have not been installed or validated on the user's comma.

Additional schema checks found the older display still using `liveCalibration`
instead of `extrinsicsCalibration`, and diagnostic rows using removed brake and
longitudinal proportional-gain fields. Those display consumers are corrected and
tested using current message types. These are source-level corrections; the
owner-review changes have not been installed or verified on a vehicle.

### Ford warning presentation, 2026-09-19

The Ford steering warning now distinguishes a tracking shortfall ("Steering Not
Keeping Up") from fresh PSCM feedback reporting a near/reached steering limit.
The event trigger, priority, steering-required HUD signal, visual lifetime,
controller requests, limits, and safety checks are unchanged. This is presentation
only; the optional feedback subscription cannot gate engagement or health checks.

A retained warning changes to "Steering Alert / Check Steering Response" only
after the trigger clears and 0.3 seconds of fresh, active, fault-free telemetry
confirms tracking within the existing angle/acceleration criteria and Ford reports
no limit. Driver override, low speed, stale/missing data, or a reported limit does
not qualify. After at least one second of the initial warning, the recovered tail
stops requesting repeated sound. A renewed trigger restores the takeover wording
and sound immediately. Critical alerts are untouched.

Offline replay of 23 captured full log segments retained noise throughout every
active warning trigger, including the recorded steering-command rejection/fault
window. The known recovered transient quieted only during its retained tail.
Unit/integration tests cover stale data, overrides, faults, non-finite inputs,
limits, recurrence, alert priority/lifetime, non-Ford behavior, and unchanged
requests. Both screen sizes were rendered. This does not fix the separately
observed command rejection or qualify the branch for road/fleet use.

### Device startup corrections, 2026-09-19

A comma four/Mach-E installation exposed two migration defects. The BP menu used
a removed MICI button width method; actual widget rendering now exercises the
replacement API. The Ford `Steer_Assist_Data` radar path assigned three removed
`RadarPoint` fields and crashed `card` as soon as a lead appeared. Those assignments
are removed without changing lead distance, velocity, or track calculations.
Packed-CAN tests cover detection, tracking, loss, reacquisition, and the camera bus.
Offline replay of captured CAN reproduced the old crash and completed 6,521 radar
updates with the correction, including 802 with leads. Some source recording data
was truncated; these results cover only the readable captured data.

The inherited stock model daemon also waited for `chestnutState` before loading,
although it is itself the publisher and starts publishing only after model load.
Startup now reads the same firmware voltage, supply-fault and PCIe-link telemetry
directly over USB EP0 within the existing wait budget. It preserves the existing
readiness predicate and rejects missing devices, bad/short reads, power faults,
low supply voltage, and a link that is not ready. Handles are closed before GPU
initialization; the probe does not claim an interface, change USB configuration,
power-cycle the GPU, or initialize tinygrad. The complete corrected modeld and
probe sources are hash-pinned in preflight; model artifacts remain unchanged.

On-device read-only telemetry confirmed the expected firmware, adequate supply,
no supply fault, and an L0 link. Model startup and fallback under cold boot,
power loss, and disconnect still require parked-device testing. This probe does
not repair or power on an unavailable link. No claim of driving qualification or
verified resolution of the vehicle's Pre-Collision Assist warning is made.

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
uv pip install 'cppcheck @ git+https://github.com/commaai/dependencies.git@b7253ddb101ed6add04fc2a2ce0688d5e21a55ee#subdirectory=cppcheck'
python tools/bluepilot_chestnut/check_safety_misra.py
python -m pytest -q \
  opendbc_repo/opendbc/safety/tests \
  --ignore=opendbc_repo/opendbc/safety/tests/misra \
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
The workflow repeats the build, all vehicle safety-hook tests, and focused Ford
integration tests from a clean Linux checkout. It now also runs safety MISRA
analysis and two deterministic Ford analyzer-mutation checks through
`check_safety_misra.py`. The upstream randomized mutation runner is still excluded
from pytest; the new gate uses its untouched safety sources, pinned analyzer,
coverage table, analysis flags and suppression list without its dependency setup.

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
script has not been run. The first user installation exposed the issues above;
installation alone is not qualification of the corrections.
