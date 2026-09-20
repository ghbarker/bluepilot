# Migration PR scope

This branch targets BluePilotDev/bluepilot `bp-dev`. It imports the complete
sunnypilot `staging-chestnut` release at `dd29072b71a06c9ce125a481ced431ba995b84f6`
and ports BP donor `e22afa6be9b881fa784c92ebb316db47728a3d81` onto that layout.

The release has separate Git history. The first commit records both the existing
BP development branch and the immutable release snapshot as parents. Its tree is
exactly the release tree. Subsequent commits carry the BP overlay and migration
repairs. This creates a normal common ancestor for GitHub review without rewriting
the installed SP-BPDEV branch. It is a packaged-base migration, not a source-only
merge; the native build and pinned driving-model checks are required.

Included: current-schema and typed-parameter compatibility; BP Ford control/UI/
Portal integration; existing Edge identity and menu labels; existing diagnostics,
build screen and dependency synchronization, GPS setup/recovery; menu/radar/lead/
calibration/debug crash fixes; the Chestnut startup readiness correction; signed
angle bounds, accepted-command history and re-engagement safety-state repairs;
safety static-analysis repairs and regression gates.

Excluded until the follow-up: revised arc direction/freshness behavior, the new
read-only Ford limit-feedback field and its arc presentation, and recovered-warning
wording/sound changes. No StarPilot code or tuning comparison changes are included.

The migration is a draft for maintainer review. Known command rejection followed
by temporary loss of steering assistance remains unresolved. Vehicle and actuator
qualification remain blocked. No route data is made public by this PR.
