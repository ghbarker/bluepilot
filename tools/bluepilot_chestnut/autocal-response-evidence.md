# Ford Angle calibration response evidence

The previous recent-response EMA forgot qualified observations while waiting for
the next clean turn. At 0.05 weighted seconds of admitted evidence per elapsed
second, its approximately 89-second effective time constant cannot reach the
6-second trial or 12-second requalification requirements. A trustworthy long-term
fit could therefore remain unable to start a trial.

Qualified observations now accumulate without exponential weight decay in a
runtime window. Each anchor keeps at most 24 weighted seconds, and observations
expire after 600 elapsed seconds, including inactive time. The fit itself retains
its existing long-term forgetting and confidence requirements. The window and
its freshness are never restored across process restarts.

Verification compares shared speed and turn-direction bins with the same weight
on each side of the adjustment. Both responses must meet the existing 0.06 error
threshold, and at least 6 weighted seconds must match. The measured response must
move strictly closer to the request. A change in the mix of turns alone cannot
confirm a trial. Speed matching uses the pure low/high endpoints and three bands
between them; it does not account for every possible road or weather difference.

Normal trials start only in the currently observed speed range on a clear frame.
The factor pair stays fixed while a trial is being evaluated. A factor change
clears staged and recent responses, so pre-change measurements cannot confirm it.
An unfinished trial whose baseline expires, crosses a process restart, or loses
delay readiness is scheduled to return to its prior value. Writes remain blocked
while delay is learning. Return steps retain the existing 0.05 bound and manual
edits take precedence. A measured worsening additionally reduces the retry step;
expiry alone does not establish that the adjustment direction was wrong.

The existing driver, disturbance, saturation, lag alignment, fit confidence,
left/right consistency, and rollback protections remain in force. This change
does not raise steering limits, select a model, or alter Panda safety. It also
does not guarantee calibration on drives without sufficient suitable turns.
Status now identifies missing fit evidence, response inconsistency, fresh-turn
requirements, matching verification turns, and other active blockers.

## Validation and limits

Tests cover intermittent turns through the actual admission pipeline, mismatched
speed/direction mixtures, stale and restarted trials, delay loss, driver and
limit flags, manual edits, write failures, bounded rollback, smaller retries,
closed-loop convergence, native parameter/message integration, and real widget
rendering. The existing disabled-calibration test compares actual Ford steering
outputs to the pinned pre-calibration implementation.

A separate synthetic comparison against `a790c27c65de52b0ba20f191f7ef5445a6df11bb`
used six-second turns separated by 54-second straights, alternating speed ranges
and direction, with an existing trustworthy fit and fresh-response requalification.
The previous pipeline made no adjustment in 900 simulated seconds. This pipeline
started its first 0.05 low-speed trial at 243.35 seconds. Those are noiseless
synthetic observations, not a route replay or a prediction of an individual's
calibration time. Device installation, live calibration behavior, and the reported
lane-crossing issue require separate vehicle evidence.

## Persistence fault handling

The controller checkpoints the pending trial or recovery before changing a factor.
It requires a successful blocking save and an exact readback, then reads back each
factor write. All state writes are ordered and blocking so an older asynchronous
checkpoint cannot overwrite the recovery record. The pre-write record's `applied`
field describes the observed old pair; the pipeline's `frm`/`to` records protect
recovery until a later checkpoint records the newly observed values.

Each successful partial write updates the controller's own-write record. A later
failure cannot masquerade as a driver edit and discard verification. Observed
manual changes between a frame, checkpoint, and factor writes cancel the trial;
the same applies to an external edit present at restart. This is read-before-write
detection, not an atomic transaction with another process editing the same key.

The Python Params wrapper now raises on nonzero native write return codes,
including failures after rename such as directory fsync. Successful calls retain
their API behavior. Errors occurring later in native asynchronous writes still
cannot be reported to their original caller.

Fault tests cover failed or dropped checkpoints, failed/partial factor writes,
process death after either factor is persisted, manual edits, and disable during
a checkpoint. Native Params tests cover error codes and a simulated failure after
the state file became readable. Readback alone would miss that durability failure.

Unchanged factors are no longer rewritten, so a normal one-anchor trial needs one
checkpoint and one factor write. The 30-second evidence checkpoint is now blocking;
device filesystem latency and control-loop timing still require on-device
validation. Host tests do not establish real-time performance or road behavior.
