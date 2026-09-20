# Steering display and warning follow-up

Base: `pr/chestnut-rebase-fixes`. Submit this follow-up after the BluePilot
migration PR is merged so its review contains only these additions.

This branch adds the corrected arc direction/freshness behavior, optional
read-only Ford limit feedback and corresponding display labels/colors, and
recovery-aware warning wording/sound. The migration PR already contains existing
BP features and compatibility repairs, including menu/radar/lead/schema crashes.

No controller requests, safety hooks, steering limits, warning triggers, or
critical-alert definitions change relative to the migration branch. The optional
feedback service cannot gate engagement. Unknown/stale feedback does not imply
available capacity. Arc length remains an estimate of demand, not a measured
continuous steering-capacity gauge.

The runtime sources reproduce SP-BPDEV at
`77655fb70e33649c00dbc995e8d009f16ca853a1`; only PR documentation and CI branch
selection differ. That combined behavior has offline log-replay evidence and
limited comma four/Mach-E startup observation. This split branch has not been
installed separately. No StarPilot code or tuning changes are included.

The separately recorded steering-command rejection and temporary loss of
assistance remain unresolved. This is a review candidate, not fleet or driving
qualification. No private route files are included.
