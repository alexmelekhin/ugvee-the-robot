# Research: `waveshareteam/ugv_base_ros` issue #10 and PR #11

Date: 2026-07-15. Sources: GitHub API (`gh api`, `gh pr diff`, `gh issue view`) and raw files
from upstream default branch `main` at HEAD `2e7df97c331ff63e1481afb34feec52014a3632a`
(latest commit, 2025-11-28). All line numbers below refer to that HEAD unless stated.

## Summary

- Issue #10 ("Very poor driving experience!", open, no maintainer response) reports exactly our
  symptoms: smooth motion with open-loop `T=11`, stuttering/fluctuating motion with closed-loop
  `T=1`/`T=13`, and unusable noisy `T=1001` velocity feedback.
- PR #11 (closed unmerged) is a 1-commit fix (`538f384c61002ef00cbb2c0e1c0e6352237f61c4`,
  +145/−29, 3 files) by Justin Davis (@JuzzyDee), field-tested on a **UGV Rover**. It was
  **closed by its own author with zero comments and zero reviews** — there is no discussion
  explaining why; it was never rejected by a maintainer. The author's fork
  [`JuzzyDee/ugv_base_ros_smooth`](https://github.com/JuzzyDee/ugv_base_ros_smooth) still exists.
- Every defect claimed in the handoff exists in current upstream HEAD, with two corrections:
  (a) the PID *compute* period is not actually unbounded — the PID_v2 library computes at its own
  default 100 ms sample time; the real defect is that the velocity *input* it samples is a
  per-loop-tick encoder delta (sub-ms window, quantised to 0–2 pulses); (b) PR #11 adds the
  heartbeat refresh to `T=13` *and* input validation — upstream `T=13` today both bypasses the
  watchdog refresh and passes unvalidated JSON straight to `rosCtrl`.
- Upstream has had **no commits since 2025-11-28** — i.e. no activity since before issue #10 was
  even filed. GitHub reports PR #11 as `mergeable: true, mergeable_state: "clean"` against HEAD.
- The firmware selects robot geometry via `mainType` (1 RaspRover / 2 UGV Rover / 3 UGV Beast).
  There is no dedicated UGV02 type; per upstream README and commit `8a9517c1` ("change
  one_circle_pulse for UGV02 and UGV Rover"), the current-generation UGV02 uses this repo with the
  `mainType = 2` parameters. PR #11's changes contain no `mainType` conditionals and apply to the
  UGV02 configuration as-is; only its tuning constants (Ki=120, feed-forward 34/60) were tuned on
  a UGV Rover and may need re-tuning on UGV02.

## 1. Issue #10

Source: <https://github.com/waveshareteam/ugv_base_ros/issues/10> (via
`gh issue view 10 --repo waveshareteam/ugv_base_ros`).

| Field | Value |
|---|---|
| Title | **"Very poor driving experience!"** |
| Author | @mibcat |
| Created | 2026-03-12T18:55:37Z (updated 2026-03-12T18:58:18Z) |
| State | OPEN |
| Labels / assignees | none |
| Comments | **0** — no maintainer response of any kind |

Hardware: the author writes *"I'm using the UGV02 Rover with the latest firmware version 0.96 and
running my own ROS2 software on a Raspberry Pi 4 communicating via GPIO UART."* The phrase
"UGV02 Rover" is ambiguous (UGV02 and UGV Rover are distinct Waveshare products); the reporter
does not clarify further.

Reported symptoms (verbatim structure of the issue body):

1. Motors run at constant speed only **without** speed control (`T=11`). With closed-loop control
   (`T=1` or `T=13`, and equally via the stock web UI), motors run with *"audible fluctuations or
   stuttering at low target speeds"*.
2. `T=1001` base-feedback RPM/velocity *"fluctuates wildly with large outliers"*; with `T=1`/`T=13`
   the feedback is *"practically unusable"*.
3. Reporter's own hypotheses: unreliable speed measurement, unsuitable controller parameters,
   ESP32 overload.

The attached logs support the quantisation diagnosis: with a constant command, reported `L`/`R`
velocity jumps between discrete levels ≈ 0.190, 0.333, 0.381, 0.571 m/s — i.e. integer pulse
counts over a tiny fixed window, not a continuous measurement.

## 2. PR #11

Source: <https://github.com/waveshareteam/ugv_base_ros/pull/11> (via
`gh api repos/waveshareteam/ugv_base_ros/pulls/11`, `.../pulls/11/commits`, `.../pulls/11/reviews`,
`.../issues/11/comments`, `.../issues/11/timeline`, and `gh pr diff 11`).

| Field | Value |
|---|---|
| Title | "Fix poor low-speed driving — cogging & wheel stalls in closed-loop velocity control (#10)" |
| Author | @JuzzyDee (commit author: Justin Davis; commit co-authored by "Claude Opus 4.7", PR body notes it was generated with Claude Code) |
| Created | 2026-05-24T02:02:24Z |
| Closed | 2026-06-15T10:53:18Z, `merged_at: null` |
| Commits | 1 — `538f384c61002ef00cbb2c0e1c0e6352237f61c4` |
| Size | +145 / −29 across 3 files: `ROS_Driver/movtion_module.h`, `ROS_Driver/uart_ctrl.h`, `ROS_Driver/ugv_config.h` |
| Base | `waveshareteam:main` @ `2e7df97c` (identical to current HEAD) |
| Mergeability | `mergeable: true`, `mergeable_state: "clean"` |

**Why closed:** there is no recorded reason. The PR has **0 issue comments, 0 review comments,
0 reviews**; the timeline shows only `committed` → `closed` (actor: JuzzyDee, 2026-06-15T10:53:18Z)
→ `head_ref_deleted` (actor: JuzzyDee, 16 s later). I.e. the author closed his own PR and deleted
the branch after ~3 weeks with no maintainer engagement. His fork
`JuzzyDee/ugv_base_ros_smooth` remains public (last pushed 2026-06-15, same minute).

**Testing claimed:** *"Flashed and field-tested on a UGV Rover (ESP32). Low-speed straight-line
and rotate-in-place driving are smooth; the per-wheel stalls are gone. Compiles clean against
esp32 arduino core 3.1.3."* (PR body.) So: tested on **UGV Rover**, not UGV02.

### 2.1 Change-by-change analysis of the diff

All verified against `gh pr diff 11 --repo waveshareteam/ugv_base_ros` (commit `538f384c`).

**`ROS_Driver/movtion_module.h`**

1. **64-bit encoder counters.** `int lastEncoderA/B` → `int64_t`, and the locals in
   `getLeftSpeed()`/`getRightSpeed()` from `long` → `int64_t`, matching
   `ESP32Encoder::getCount()`'s `int64_t` return. Confirmed in diff. (On 32-bit reads the counter
   itself won't overflow in practical sessions; this is correctness hygiene, not the driving fix.)

2. **Fixed 50 ms velocity sampling window.** New constants
   `SPEED_SAMPLE_INTERVAL_US = 50000` and `SPEED_FILTER_ALPHA = 0.3`. `getLeftSpeed()` /
   `getRightSpeed()` are restructured: odometry (`en_odom_l/r`) is still updated from the absolute
   count on every call ("unchanged behaviour" per diff comment), but the velocity estimate now
   returns early unless ≥ 50 ms has elapsed since the last sample; only then is
   `pulseDelta / elapsed` computed. Diff comment: a 50 ms sample gives ~13 pulses at 0.1 m/s on
   the UGV Rover. Confirmed — matches the handoff.

3. **EMA smoothing, alpha 0.3, shared with telemetry.**
   `speedFilteredX = 0.3·measured + 0.7·speedFilteredX; speedGetX = speedFilteredX;` — the same
   smoothed value feeds both the PID and the `T=1001` feedback (diff comment: "addresses #10").
   Confirmed.

4. **Speed-estimate reset on stop/reversal.** New `resetLeftSpeedEstimate()` /
   `resetRightSpeedEstimate()` (re-latch encoder count + timestamp, zero `speedGet*` and
   `speedFiltered*`). Called from `setGoalSpeed()` whenever the new setpoint is 0 or its sign
   differs from the previous setpoint. `initEncoders()` is also extended to initialise the
   latched counts/timestamps and zero the estimates. Confirmed.

5. **PID sample time.** `pidA.SetSampleTime(50)` and `pidB.SetSampleTime(50)` added in
   `pidControllerInit()`, aligning the PID compute period with the new 50 ms feedback rate.
   Confirmed. (Without this the PID_v2 library computes on its default 100 ms internal sample
   time — see §3.1.)

6. **Feed-forward replacing deadband zeroing.** New
   `motorFeedForward(setpoint) = sign(setpoint) · (MOTOR_MIN_FEEDFORWARD_PWM + |setpoint|·MOTOR_SPEED_FEEDFORWARD_PWM)`
   (returns 0 for setpoint 0) and `clampMotorOutput()` (±255). In
   `Left/RightPidControllerCompute()` the old logic

   ```c
   outputA = pidA.Run(speedGetA);
   if (abs(outputA)<THRESHOLD_PWM) { outputA = 0; }
   if (setpointA == 0 && speedGetA == 0) { outputA = 0; }
   ```

   becomes

   ```c
   double pidOutput = pidA.Run(speedFilteredA);
   if (setpointA == 0) { outputA = 0; }
   else { outputA = clampMotorOutput(motorFeedForward(setpointA) + pidOutput); }
   ```

   Confirmed. Two consequences the handoff did not spell out: (a) the `THRESHOLD_PWM` deadband
   zeroing is **removed entirely** (the macro stays defined in `ugv_config.h` but becomes dead);
   (b) stop behaviour changes from "PID actively brakes until measured speed reads exactly 0" to
   "**output forced to 0 whenever setpoint is 0**" — a coast stop. This is what eliminates the
   reverse twitch on stop, at the cost of no active braking.

7. **`setGoalSpeed()` setpoint-buffer bug fix.** Upstream writes
   `setpointA_buffer = inputLeft` (the raw input) after commanding `pidA.Setpoint(setpointA)`
   (the `spd_rate`-scaled value), so the change-detection compare is against the wrong variable
   whenever `spd_rate_A ≠ 1`. PR stores the scaled `setpointA` instead. Confirmed in diff —
   **missed by the handoff summary**.

8. **`rosCtrl()` rewrite.** Upstream `rosCtrl()` overwrites the *global* `setpointA/B` with the
   kinematic result and then passes them to `setGoalSpeed()` — destroying the previous-setpoint
   values the new reversal-detection needs. PR computes into locals, and additionally snaps
   `|value| < 1e-4` to exactly 0 so a wheel that should cancel to zero doesn't reach
   `motorFeedForward()` as float residue and get kicked to `MOTOR_MIN_FEEDFORWARD_PWM` (a wheel
   meant to be still would twitch). Confirmed — the handoff's "setpoint/near-zero fixes".

**`ROS_Driver/uart_ctrl.h`**

9. **`CMD_ROS_CTRL` (`T=13`) validation + heartbeat refresh.** Upstream case (HEAD
   `uart_ctrl.h:27-29`) calls `rosCtrl(jsonCmdReceive["X"], jsonCmdReceive["Z"])` with no key
   checks and no watchdog refresh. PR wraps it in `containsKey("X"/"Z")` + `is<float>()` checks
   and sets `heartbeatStopFlag = false; lastCmdRecvTime = millis();` before calling `rosCtrl` —
   the same pattern `CMD_SPEED_CTRL` (`T=1`) already uses at `uart_ctrl.h:9-20`. Confirmed.

**`ROS_Driver/ugv_config.h`**

10. **`__ki` 2000 → 120** (`__kp = 20`, `__kd = 0`, `windup_limits = 255` unchanged). Confirmed.
11. **New constants** `MOTOR_MIN_FEEDFORWARD_PWM = 34.0`, `MOTOR_SPEED_FEEDFORWARD_PWM = 60.0`.
    Confirmed. (`THRESHOLD_PWM 23` remains defined but is now unused.)

**What the diff does *not* do** (worth knowing for our fork):

- It does not reset the PID's internal state (integral / last-input / last-time) when
  `usePIDCompute` toggles from false (after `T=11`) back to true (`T=0`/`T=1`/`T=13`); the stale
  controller state is only masked by the unconditional `setpoint == 0 → output = 0` path and the
  speed-estimate reset.
- It does not touch `CMD_EMERGENCY_STOP` (`T=0`), the main `loop()` scheduling, `heartBeatCtrl()`
  itself, `json_cmd.h`, or `http_server.h`.
- It does not change the `heartbeatStopFlag` latch semantics (see defect F below).

## 3. Defect verification against current upstream HEAD (`2e7df97c`)

Files fetched from
`https://raw.githubusercontent.com/waveshareteam/ugv_base_ros/2e7df97c.../ROS_Driver/<file>`.

**A. Velocity estimated from per-tick encoder deltas — CONFIRMED.**
`getLeftSpeed()` / `getRightSpeed()` (`movtion_module.h:145-171`) compute
`speedGet = plusesRate · Δpulses / Δt` with `Δt` = time since the *previous loop iteration*, and
are called once per `loop()` pass (`ROS_Driver.ino:364,368`). The cooperative loop also runs IMU
FIFO reads, HTTP handling, OLED updates and serial feedback (`ROS_Driver.ino:293-379`), so the
window is tiny and irregular; at low speed each window sees 0–2 pulses
(`plusesRate = π·0.08/660 ≈ 3.8×10⁻⁴ m/pulse`, `movtion_module.h:135`, `ugv_config.h:343-344`),
producing exactly the quantised feedback levels visible in issue #10's logs.

**B. "No fixed control period" — PARTIALLY WRONG as stated.**
The PID objects are `PID_v2` (`movtion_module.h:177-178`); `pidControllerInit()`
(`movtion_module.h:195-207`) never calls `SetSampleTime`, and the Arduino PID library
(Beauregard's PID, wrapped by PID_v2) computes internally at a default **100 ms** sample time,
returning the previous output between samples
(<https://github.com/imax9000/Arduino-PID-Library> — `PID::PID` sets `SampleTime = 100`).
So the PID compute period *is* fixed; the defect is that whichever loop tick coincides with the
PID's 100 ms boundary supplies a velocity estimate computed over that single sub-ms tick (defect
A). PR #11's `SetSampleTime(50)` aligns compute with the new 50 ms measurement.

**C. Poor PID defaults — CONFIRMED.**
`__kp = 20.0; __ki = 2000.0; __kd = 0;` at `ugv_config.h:315-317`; `THRESHOLD_PWM 23` at
`ugv_config.h:323`, applied as output zeroing at `movtion_module.h:294-296` and `309-311`. With
quantised feedback, Ki=2000 winds up between pulses and the deadband stalls the wheel below
PWM 23, re-triggering the wind-up — the cogging cycle described in PR #11's body.

**D. `T=13` does not refresh the heartbeat (and does not validate input) — CONFIRMED.**
`uart_ctrl.h:27-29`: `case CMD_ROS_CTRL: rosCtrl(jsonCmdReceive["X"], jsonCmdReceive["Z"]); break;`
— no `lastCmdRecvTime`/`heartbeatStopFlag` update, unlike `CMD_SPEED_CTRL`
(`uart_ctrl.h:14-15`) and `CMD_PWM_INPUT` (`uart_ctrl.h:22-23`). Same handler serves the web UI
(`http_server.h:13-22` routes `/js` into `jsonCmdReceiveHandler()`).

**E. Consequence of D — heartbeat is effectively *disabled* under pure `T=13` control.**
`heartBeatCtrl()` (`movtion_module.h:333-340`, called every loop from `ROS_Driver.ino:378`) stops
the robot only on the `heartbeatStopFlag` false→true transition. Under `T=13`-only traffic the
timer (`HEART_BEAT_DELAY = 3000`, `ugv_config.h:365`) is always expired, so ~3 s after boot (or
after the last `T=1`/`T=11`) the flag latches true — producing **one spurious stop mid-drive** —
and thereafter the watchdog never fires again, so losing the `T=13` stream will **not** stop the
rover. This matches our observed "T=13 not stopping on heartbeat timeout" and is fixed by PR #11
change 9 (refreshing both the flag and the timestamp on every valid `T=13`).

**F. Unsafe open-loop → closed-loop stop transition (reverse twitch on `T=0` after `T=11`) —
CONFIRMED as a plausible mechanism in HEAD code.**
`T=11` sets `usePIDCompute = false` (`uart_ctrl.h:21`), so `pidX.Run()` stops being called
(`movtion_module.h:289-291`) and the controller's internal state goes stale while PWM drives the
wheels. `T=0` calls `emergencyStopProcessing()` + `setGoalSpeed(0,0)` (`uart_ctrl.h:5-8`), and
`setGoalSpeed()` unconditionally re-enables the PID (`usePIDCompute = true`,
`movtion_module.h:264`). The PID then resumes with setpoint 0 while the wheels still spin;
upstream only forces output to 0 when `setpoint == 0 && speedGet == 0`
(`movtion_module.h:297-299`) — and with per-tick quantised `speedGet` bouncing between 0 and
large values, Ki=2000 drives a negative (reversing) output on the nonzero samples. The stochastic
reverse twitch follows directly. PR #11 removes the twitch by forcing output to 0 whenever
`setpoint == 0` and resetting the speed estimate on stop (changes 4 and 6); it does *not*
otherwise re-initialise the PID (see §2.1 "does not do").

**G. Noisy `T=1001` telemetry — CONFIRMED**, same root cause as A: `baseInfoFeedback()` publishes
`speedGetA/B` directly (see `json_cmd.h` feedback path; values in issue #10's logs step in
per-pulse quanta). PR #11 change 3 makes telemetry publish the EMA-smoothed value.

## 4. Upstream activity since PR #11

- Last commit on `main`: `2e7df97c` "Update web_page.h", 2025-11-28T02:54:57Z
  (`gh api repos/waveshareteam/ugv_base_ros/commits`); repo `pushed_at: 2025-11-28`. **No commits
  at all since before issue #10 (2026-03-12) was filed or PR #11 (2026-05-24) was opened.** The
  motion-control files have not changed since Sep 2024 (`c967326f`) / May 2024 (`8a9517c1`).
- Full issue/PR list (`gh api .../issues?state=all`): #10 is the only motion-control issue; the
  only other open code-defect issue is #9 (IMU calibration applies the X-gyro bias to Y and Z,
  2026-02-23, also unanswered). #6 (build errors on Arduino core 2.x), #5 (ROS support question),
  #4, #2, #1 are unrelated. #8 "Pathfinder v2" (closed PR, 2025-09-21) and #11 are the only PRs;
  neither was merged.
- No maintainer has commented on #9, #10 or #11. The repository appears effectively unmaintained
  since late 2025 (the 2025-11-28 commit touched only the embedded web page).

## 5. Applicability to UGV02

- The firmware has **no dedicated UGV02 type**. Robot geometry is selected by `mainType`
  (`ugv_config.h:35-38`, default `2`) in `mm_settings()` (`movtion_module.h:346-401`):
  1 = RaspRover (WHEEL_D 0.0800, 2100 pulses, track 0.125), 2 = UGV Rover (0.0800, 660, 0.172),
  3 = UGV Beast (0.0523, 1092, 0.141). `SET_MOTOR_DIR = false` for types 1–2.
- The upstream README states the repo serves *"UGV Rover, UGV Beast, RaspRover, UGV02\*"* with the
  footnote *"The old version of UGV02 is driven by General Driver"*
  (<https://github.com/waveshareteam/ugv_base_ros/blob/main/README.md>). Commit `8a9517c1`
  ("change one_circle_pulse for UGV02 and UGV Rover", 2024-05-15) changed only the `mainType = 2`
  entry (1650 → 660 pulses), implying **UGV02 runs as `mainType = 2`, sharing the UGV Rover's
  wheel/encoder parameters** in this codebase. Whether UGV02's physical track width really equals
  0.172 m is not verifiable from the repo (open question below).
- PR #11 contains **no `mainType`-conditional code**; all changes are in the shared control path,
  and GitHub reports the PR clean-mergeable against current HEAD (base sha == HEAD). It therefore
  applies to the UGV02 configuration without adaptation. Caveats: `MOTOR_MIN_FEEDFORWARD_PWM = 34`,
  `MOTOR_SPEED_FEEDFORWARD_PWM = 60` and `__ki = 120` were tuned on a UGV Rover; UGV02 uses the
  same motor-driver PWM range and (per this firmware) the same wheel/encoder constants, so the
  50 ms sampling math holds, but static-friction feed-forward values are chassis/motor dependent
  and should be re-verified on our unit. The 50 ms window comment ("~13 pulses at 0.1 m/s")
  assumes 660 pulses/rev × 0.08 m wheels — identical for UGV02's `mainType = 2` settings.

## Open questions

1. Why did @JuzzyDee close PR #11? Nothing on the record; possibilities (perceived upstream
   abandonment, keeping the fork instead) are speculation. His fork `ugv_base_ros_smooth` is the
   living version of the patch.
2. Is UGV02's physical track width really 0.172 m (the `mainType = 2` value)? Needs checking
   against the Waveshare UGV02 wiki/measurement; wrong TRACK_WIDTH skews `T=13` angular-velocity
   kinematics (`rosCtrl`, `movtion_module.h:327-331`).
3. Issue #10's reporter said "UGV02 Rover" — ambiguous which product; doesn't affect the defect
   analysis since both run this firmware with `mainType = 2` motion code.
4. PR #11 leaves the `heartbeatStopFlag` latch design and the un-reset PID internal state across
   `T=11`→closed-loop transitions untouched; our fork may want to fix both properly (refresh
   semantics + `pid.Start()`/state reset when `usePIDCompute` re-enables).
5. Whether `T=0` after `T=11` still shows any twitch with PR #11 applied was only implicitly
   tested upstream ("stalls are gone" on UGV Rover); worth explicit testing on our UGV02.
