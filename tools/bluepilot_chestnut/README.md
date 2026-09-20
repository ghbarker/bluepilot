# BluePilot + Chestnut: blocked preparation draft

**This branch does not implement the requested BluePilot overlay and is not an
install-ready BluePilot release.** Its executable runtime is still the unmodified
sunnypilot `staging-chestnut` release. The additions here are development checks and
an inventory, not a claim that Ford control, UI, or Portal has been ported.

The port stopped at the user's explicit safety condition: keep current upstream
safety, and do not restore old safety code or weaken checks to resolve interfaces.
An unchanged donor header does not implement the current curvature safety API.
A semantics-preserving, validated integration has not been established. This does
not prove that a safe integration is impossible.

## Pinned inputs

- Release base: `sunnypilot/sunnypilot`, `staging-chestnut`,
  `dd29072b71a06c9ce125a481ced431ba995b84f6`.
- Exact source for that release: `sunnypilot/sunnypilot`,
  `a5f44653d7f43ad57fef2f546f3916ec4cbf3c56`.
- Ford donor: `BluePilotDev/bluepilot`, `bp-dev`,
  `e22afa6be9b881fa784c92ebb316db47728a3d81`. No `bp-7.0` code was used.
- Main repository common ancestor:
  `01a843e0acbe74d566a7eee9fe0f12f227ae81ed`.
- Old opendbc submodule at that ancestor:
  `b9712d20efd4fb7b1c29378ee153013d2a32a9f1`.
- Current source opendbc:
  `f95f996f5917dcbbf2e32fe51b606a24cf836af6`.

The branch starts at the packaged release, not at BluePilot or plain master.
Existing fork branches, including `llf`, are preserved.

## Safety incompatibility

Current `opendbc/safety/declarations.h` separates `AngleSteeringLimits` and
`CurvatureSteeringLimits`. Current Ford uses `steer_curvature_cmd_checks` and
`curvature_state.meas`; the donor uses `steer_angle_cmd_checks` and `angle_meas` for
curvature. The donor references five fields removed from `AngleSteeringLimits`:
`max_angle_error`, `angle_error_min_speed`, `angle_is_curvature`,
`enforce_angle_error`, and `inactive_angle_is_zero`.

Current curvature checks enforce lateral acceleration, lateral jerk, rolling
transmit-rate limits, and curvature-error behavior. These are semantic changes,
not only renamed fields. Current Ford tests exercise these protections on CAN as
well as CAN-FD. Restoring the old shared lateral implementation would lose newer
behavior and affect other makes.

BluePilot angle-primary operation also holds the transmitted curvature signal at
zero and separately corroborates shadow curvature. Calling the current check on
that zero alone would not validate the effective angle-mode demand. A port needs
to establish which demand is checked, how state advances once per command, and
how shadow freshness/corroboration, pinion measurement, inactive/reset behavior,
path-angle rate limits, and the current transmit-rate accounting interact.
No replacement algorithm has been guessed here.

Do not disable driver monitoring, longitudinal disengagement on brake, or ACC MAIN
OFF disengagement. Do not add automatic longitudinal re-engagement on brake
release. Neither these policies nor any runtime safety files have been changed.

## Packaging and the rebuild path that remains to be qualified

The release is a flattened, prebuilt ARM distribution. It has `prebuilt`, native
libraries, generated Cereal C++, and `panda/board/obj/panda_h7.bin.signed`, but no
root SConstruct, dependency gitlinks, or SConscript files. The launcher skips its
build while `prebuilt` exists. Editing a header, Params key, or schema in this tree
does **not** update those binaries.

A source-complete build must be reconstructed from the exact source revision and
its exact dependency revisions, retaining the release's resolved assets and
Chestnut model chunks. The source gitlinks are:

- panda: `74a0adced421e8b7acd728d0f9988ce225423f13`
- opendbc_repo: `f95f996f5917dcbbf2e32fe51b606a24cf836af6`
- msgq_repo: `e7396e76dadbb49e374d4b664ff6bbb43a39bcb0`
- rednose_repo: `28d4a7f69e80e1c3e0d24ca0733d7daeaeade3d0`
- teleoprtc_repo: `1aa8fc433bef1519a95c0700c96258c3be6dfb34`
- tinygrad_repo: `f6fc4e3f2c3db5fae1e19cbfbc3ad9fc579a12ae`

Also follow the source's `openpilot/sunnypilot/neural_network_data` gitlink. Do not
substitute dependency heads or copy June-era stock files. Missing build inputs
include the root SConstruct and 16 main-repository SConscript files, as well as
dependency build inputs. Some missing ONNX inputs are LFS assets, not ordinary
Git blobs.

The exact source `openpilot/selfdrive/modeld/SConscript` supports
`SKIP_TINYGRAD_COMPILE=1` for the driving-model compile loop. **That flag does not
skip driver-monitoring metadata/model/warp compilation**: restoring SConstruct
alone is insufficient, and an x86 build cannot be shipped as a comma ARM build.
Keep the 18 precompiled 1B chunks plus their manifest unchanged; do not compile
the USB+AMD model in CI. `model_artifacts.json` pins their SHA-256 digests and sizes.

After a validated source port, the qualified ARM build/package must rebuild Params,
Cereal, native consumers, and Panda firmware, and demonstrate that the device
actually flashes the resulting firmware. The upstream release script distinguishes
release signing (`CERT=/data/pandaextra/certs/release RELEASE=1`) from a debug
Panda build. Signing credentials and a target-device build have not been verified
here; do not claim the existing signed binary contains the overlay. Do not copy
upstream release scripts blindly: they remove `/data/openpilot` and force-push
release refs. A non-destructive packaging procedure still needs validation.

## Candidate inventory and remaining integration

`candidate_inventory.json` records a three-way text trial outside the runnable
checkout: 499 screened candidates, comprising 441 new-at-mapped-path files,
38 text merges, 19 conflicts, and one identical file. This is **not** a complete
overlay manifest. A new mapped path can be a moved upstream file, not a genuinely
copy-only donor file; a clean text merge does not establish API compatibility.

The trial covered the donor `bluepilot/`, main code/assets roots, BP version files,
Ford vehicle/extension files, safety files, and selected shared opendbc touchpoints.
It excluded donor root launch/build/release scripts, most documentation/tooling,
and non-selected shared opendbc changes. Before completing the port, separately
review the full opendbc diff, including `car/fingerprints.py`, torque overrides,
`sunnypilot/car/car_list.json`, and DBC changes; do not blindly import unrelated
ESR/Mazda churn. BP theme and other assets need LFS resolution checks.

Text conflicts include Ford controller/values and safety/header tests; card and
selfdrived; TICI/MICI main UI and MICI settings; sound/UI state; Params migration;
registration; hardware; process registration; GPS; Wi-Fi; and a sunnylink test.
The old AGNOS manifest conflict must resolve to current upstream, not old firmware.

Additional semantic work includes:

- Map old `cereal`, package entry points, hardware imports, and filesystem asset
  paths to the current `openpilot/` layout. The donor's `is_bluepilot()` searches
  relative to `common`, so BPVERSION placement must follow the new layout.
- Adapt Params use: current code uses `openpilot.common.params` with a native
  library; donor UI imports the old `params_pyx.UnknownKeyName` interface.
- Keep both current Models settings panels reachable. Adapt BP subclasses to the
  current UI contracts instead of replacing the stock panels wholesale.
- Preserve every stock/model process registration and predicate. Add only the two
  BP Portal processes with the correct current Python module names.
- Merge Cereal BP fields/services and all consumers, with schema compatibility
  validation and rebuilt native consumers.
- Preserve Ford extension algorithms and test angle/shadow behavior, VIN matching,
  controller/CAN compatibility, and all current plus donor safety cases.

No trial merge output was installed into the runtime.

## Checks

From the repository root:

```sh
python3 -m unittest tools.bluepilot_chestnut.test_preflight
python3 tools/bluepilot_chestnut/preflight.py
```

The second command intentionally exits **2**, reporting `BLOCKED_NOT_INSTALL_READY`.
Exit 1 means a check failed; it is not an accepted blocked result. The preparation
CI checks that distinction. A green preparation job is not release qualification.

With the pinned release, donor, main common ancestor, and old opendbc commit
available locally, `python3 tools/bluepilot_chestnut/inventory.py --compare`
reproduces the screened inventory. It reads Git blobs and merges only in a
temporary directory; it does not restore files into the checkout.

On Linux with a C compiler, after fetching the pinned donor commit:

```sh
python3 tools/bluepilot_chestnut/preflight.py --probe-donor
```

This places only the donor header in a temporary include tree and compiles it
against the current shared safety API. It expects the specific removed-field
diagnostic. It is a negative compatibility probe, not a proposed safety merge.
The workflow also runs the **unchanged upstream** Ford safety suite; it does not
claim donor extension tests or a merged safety suite passed.

Local checks: three preflight tests passed; all 18 chunks and the manifest were
hashed; the tracked runtime matches the pinned release. Linux-native validation
is delegated to the included GitHub workflow. Local Ubuntu-24.04 cannot start
because its registered F:\\wsl virtual disk path is missing. Docker's WSL distro
starts, but its Linux engine API was unavailable. Existing WSL registrations and
disk layout were not repaired or changed. No device or driving tests were done.

## Installation status

The requested candidate URL is
`https://installer.comma.ai/ghbarker/cursor/chestnut-ford-overlay-4d25`.
**Do not use it as a BluePilot + Chestnut installer:** this branch only contains
preparation tooling on top of the unchanged sunnypilot runtime. No overlay release
has been packaged or qualified.

The previously selected interim upstream installer remains
`https://install.sunnypilot.ai/staging-chestnut`; that is sunnypilot without the
BluePilot Ford extras. Once a proper overlay release is built and validated,
prove big-model loading with Alpha Longitudinal disabled before enabling that
separate experimental feature. Device model load, fallback, DM, brake/cancel/main
off behavior, Panda flashing, and UI/Portal operation remain unverified here.
