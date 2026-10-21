# DWAFP: results for the paper

Living document: every result that may go into the paper, with its configuration, the raw data
file and the command that reproduces it. Newest results are added to the relevant section; the
"Claims" table at the end says what each result supports.

Last update: 2026-10-21 (source-only baseline corrected: now trigger-task-only).

> **Correction (2026-10-21).** Until this date, the source-only baseline could also edit
> *overdue descendants* of the deviating task: tasks that the reference says should already have
> started. The impact analysis rebuilds them in the expected state, so they appeared among the
> impact "sources", and source-only edited every impact source. The baseline is now
> **trigger-task-only (source-only)** repair: edits restricted to the task whose feedback generated
> the trigger, as documented. The code requires that task explicitly and refuses the scope without
> it, so no caller can fall back to the wider set. Every closed-loop result involving source-only
> was re-run (1.4, 1.5, 1.8). Results of no intervention, reactive, DWAFP and mission-wide are
> unchanged, run for run. The open-loop Stage I ablation (Section 2) was re-run too and is
> unchanged (identical counts). The corrected baseline is weaker, and DWAFP now beats it on
> transient delays (1.4, 1.8).

---

## How to reproduce the results

From the workspace root (`~/Documents/ROS2_training/ros2_ws`), after building the `actions` package:

```bash
cd ~/Documents/ROS2_training/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select actions

./build/actions/dwafp_experiments 200 src/results/main_noise0.csv 0      # controlled
./build/actions/dwafp_experiments 200 src/results/main_noise10.csv 0.1   # robustness
./build/actions/dwafp_experiments sweeps 200 src/results/sweeps.csv > src/results/sweeps.txt   # Section 1.8
```

`sweeps` runs the five Step 6 experiments of Section 1.8 (about 45 s); each can also be run alone:
`deadlines`, `noise`, `latency`, `estimator`, `timeout`.

| Argument | Meaning |
|---|---|
| `200` | paired scenarios per deviation type (each one executed under the five methods) |
| `src/results/main_noise0.csv` | output file, one line per scenario and method (`-` = no CSV) |
| `0` / `0.1` | duration noise: 0 = controlled configuration (Section 1.2), 0.1 = 10 % noise (Section 1.3) |

Each run takes a few seconds and prints the tables of Section 1; redirect the output to keep it
(`> src/results/main_noise0.txt`). The runs are deterministic: the same command gives the same
results.

---

## How to read the tables

**Rows.** Most tables have one row per **deviation type**, the single event injected into each
scenario:

| Row | What happens to the source task | Warning in the robot feedback? |
|---|---|---|
| Transient delay | the robot is blocked for a while (0.2–1.0 × the task's duration), then continues | yes: BLOCKED, the predicted finish slips |
| Persistent obstruction | the robot can never get past the obstacle; another robot can still do the task | yes: BLOCKED, plus "same-robot retry not feasible" |
| Outcome failure | the task suddenly ends without its result | no (sudden failure) |
| Robot breakdown | a robot suddenly stops working for good | no (sudden failure) |
| Mix 35/25/25/15 | one of the four above, drawn with these probabilities | — (overall workload) |

Delays have a warning, so they test **prevention**. Sudden failures have none, so they test the
**common recovery** and work as **controls** (the methods should be equal there).

**The methods** compared are defined in 1.1. "Baseline" means the method DWAFP is compared with.

**Tasks around a deviation.** *Event source*: the task whose deviation generated the event (its
feedback fired the trigger). *Affected task*: a task whose prediction changed through
propagation. *Threatened task*: a task that acquired a violated condition (deadline, input or
robot); the set A_i. A *downstream edit* edits a task other than the event source.

**Columns and terms.**

| Term | Meaning |
|---|---|
| Run | one scenario (mission, plan, deviation, noise) executed under one method |
| Paired | every scenario is run under every method; a comparison uses the same scenarios for both |
| Misses | deadline misses in one run: tasks that have a deadline and were not completed by it (a task never completed counts as a miss); lower is better |
| A − B | mean over the 200 scenarios of (misses of A − misses of B), same scenario for both. **Negative = A is better.** Example: −0.34 = A has 0.34 fewer deadline misses per run on average |
| [95 % CI] | bootstrap 95 % confidence interval of that mean (2000 resamples). If it does not contain 0, the difference is statistically significant; whether it **matters** also depends on its size |
| W / T / L | wins / ties / losses over the 200 scenarios: in how many A had fewer, the same, or more misses than B. 39 / 161 / 0 = better in 39, equal in 161, worse in none |
| Success | share of runs in which every task completed and every deadline was met |
| Miss rate | misses / tasks that have a deadline, averaged over runs (comparable across mission sizes) |
| Completion rate | completed tasks / all tasks, averaged over runs |
| Completed-task tardiness | sum of (end − deadline) over the tasks that completed late, in seconds; tasks never completed are **not** in it, so read it with the completion rate |
| Makespan | time at which the last task completed, averaged only over runs where every task completed (their number in brackets) |
| Harmful | runs in which the method had **more** misses than no intervention at all |
| Unneeded | runs in which the method edited the plan although no intervention would have succeeded |
| Realized | runs in which the injected deviation actually happened (a method that moves the task before the event avoids it) |
| n | number of scenarios in that row |
| p50 / p95 / p99 | 50 %, 95 % and 99 % of the decisions took at most this long (one decision = one analysis and repair search) |

---

## 1. Closed-loop comparison (simulator, executed outcomes)

### 1.1 Setup

Every scenario is **executed** in the discrete-event simulator under five methods, on exactly the
same mission, plan, deviation and noise (paired comparison):

| Method | Prevention (before failure) | Recovery (after an actual failure, timeout, breakdown or deadline miss) |
|---|---|---|
| No intervention | none | none |
| Reactive | none | common recovery |
| Trigger-task-only (source-only) | edits restricted to the event source (the task whose feedback generated the trigger) | common recovery |
| **DWAFP** | edits inside the repair region | common recovery |
| Mission-wide | edits on any unfinished task | common recovery |

The common recovery is identical for every repair-enabled method, so **DWAFP − reactive measures the
value of acting before the failure**, not a difference in recovery.

Controlled configuration (primary):

| Parameter | Value |
|---|---|
| Fleet | 4 robots, 1.0 / 1.5 / 0.8 / 1.0 s per unit of work |
| Mission | 20 tasks, work 3–7 units, the four workflow shapes in turn (random DAG, chains, fork-join, rendezvous) |
| Deadlines | on every task; d_j = f̂⁰_j + u_j p_j, u_j ~ U[0.5, 3] (duration-normalized additive slack) |
| Deviation | one per scenario, source = a task with a dependency successor, starting in the first 60 % of the makespan |
| Transient delay | U[0.2, 1.0] × the source's nominal duration |
| Persistent obstruction | the robot–task pair never progresses; the robot reports that same-robot retry is infeasible |
| Execution timeout | 2 × the attempt's nominal duration (robot-side budget, independent of the deadline) |
| Duration noise | **0 %** (controlled) / 10 % (robustness) |
| Controller | nominal remaining-time estimate, no dispatch latency, trigger k = 3, ε = 0, τ = 2 s |
| Runs | 200 paired scenarios per deviation type (same seeds, same graphs, for every type) |

Primary metric: **deadline misses per run** (tasks with a deadline not completed by it; a task never
completed counts as a miss). Differences are DWAFP − baseline (negative = DWAFP better), with a
bootstrap 95 % confidence interval (2000 resamples) and wins / ties / losses over the 200 scenarios.

### 1.2 Main result: DWAFP vs reactive recovery, no noise

Rows: deviation types. Columns: the paired difference in misses (DWAFP − reactive, negative =
DWAFP better) with its interval; W / T / L of DWAFP against reactive; the success rate of reactive,
then of DWAFP; DWAFP's harmful and unneeded runs; in how many scenarios the deviation was realized.

| Deviation | Misses: DWAFP − reactive [95 % CI] | W / T / L | Success: reactive → DWAFP | Harmful / unneeded (DWAFP) | Realized |
|---|---|---|---|---|---|
| Transient delay | **−0.34** [−0.47, −0.23] | 39 / 161 / 0 | 70 % → **86 %** | 0 / 0 | 200 / 200 |
| Persistent obstruction | **−2.06** [−2.46, −1.70] | 115 / 85 / 0 | 8 % → **65 %** | 0 / 0 | 200 / 200 |
| Outcome failure | 0.00 [0.00, 0.00] | 0 / 200 / 0 | 92 % → 92 % | 0 / 0 | 200 / 200 |
| Robot breakdown | 0.00 [0.00, 0.00] | 0 / 200 / 0 | 68 % → 68 % | 3* / 0 | 200 / 200 |
| Mix 35/25/25/15 | **−0.72** [−0.96, −0.52] | 46 / 154 / 0 | 56 % → **78 %** | 0 / 0 | 200 / 200 |

\* the same 3 runs for every repair-enabled method, reactive included (see 4.3).

Reading:
- **Delays (advance warning exists): large, significant gains and no loss in 400 scenarios.**
  Under a persistent obstruction DWAFP moves the blocked task before the execution timeout:
  success rises from 8 % to 65 %.
- **Unannounced outcome failures and breakdowns: identical executions** (200 ties each). Under the
  same realized failure, all repair-enabled methods use the same recovery and produce the same
  result. This is the expected, informative outcome: prevention cannot act without warning.
- Harmful (more misses than no intervention) and unneeded (an edit in a run that would have
  succeeded without intervention) preventive edits: **none**.

Other outcome metrics (no noise, means per run). One row per (deviation, method); the rates and
tardiness are averages over the 200 runs of that method:

| Deviation | Method | Miss rate | Completion rate | Completed-task tardiness (s) | Makespan (s, fully completed runs) |
|---|---|---|---|---|---|
| Transient delay | reactive | 2.9 % | 100 % | 0.74 | 59.7 (200 runs) |
| | DWAFP | 1.2 % | 100 % | 0.26 | 59.1 (200 runs) |
| Persistent obstruction | no intervention | 45.2 % | 55.5 % | 0.31 | — (0 runs) |
| | reactive | 18.4 % | 100 % | 13.90 | 62.9 (200 runs) |
| | DWAFP | 8.1 % | 100 % | 7.13 | 60.1 (200 runs) |

### 1.3 Robustness: the same comparison with 10 % duration noise

Same rows and columns as 1.2, on the same scenarios, but every task runs up to 10 % faster or slower
than planned (the controller does not know by how much).

| Deviation | Misses: DWAFP − reactive [95 % CI] | W / T / L | Success: reactive → DWAFP | Harmful / unneeded (DWAFP) |
|---|---|---|---|---|
| Transient delay | **−0.34** [−0.47, −0.24] | 37 / 163 / 0 | 68 % → 82 % | 0 / 7 |
| Persistent obstruction | **−2.10** [−2.50, −1.74] | 118 / 81 / 1 | 8 % → 63 % | 1 / 0 |
| Outcome failure | −0.06 [−0.10, −0.03] | 10 / 190 / 0 | 85 % → 90 % | 0 / 0 |
| Robot breakdown | −0.06 [−0.12, −0.01] | 6 / 193 / 1 | 70 % → 71 % | 3 / 0 |
| Mix 35/25/25/15 | **−0.77** [−1.00, −0.56] | 55 / 144 / 1 | 53 % → 77 % | 0 / 0 |

Reading:
- **The main effect is unchanged under noise** (−0.34 and −2.10 vs −0.34 and −2.06 without noise).
- The small gains on outcome failures and breakdowns come from prevention of **noise-induced**
  delays, not from the injected failure (noise-free, these differences are exactly 0).
- Noise is what can make prevention unneeded: 7 of 200 transient-delay runs had a DWAFP edit
  although no intervention would also have succeeded; 1 persistent run was harmful.

Suggested sentence: *"The improvement over reactive recovery is unchanged under 10 % duration noise
(−0.34 and −2.10 deadline misses per run for transient and persistent blockages)."*

### 1.4 DWAFP vs source-only and vs mission-wide repair

Rows: deviation types. Each cell: the paired difference in misses (DWAFP − that method, negative =
DWAFP better), its interval, then W / T / L. "200 ties" = identical outcome in every scenario.

| Deviation | DWAFP − source-only, no noise | DWAFP − source-only, 10 % noise | DWAFP − mission-wide (both) |
|---|---|---|---|
| Transient delay | **−0.10** [−0.15, −0.04], 14 / 186 / 0 | **−0.10** [−0.17, −0.04], 12 / 187 / 1 | 0.00, 200 ties |
| Persistent obstruction | 0.00, 200 ties | −0.01 [−0.07, +0.02], 2 / 196 / 2 | 0.00, 200 ties |
| Outcome failure | 0.00, 200 ties | −0.01 [−0.03, +0.01], 3 / 196 / 1 | 0.00, 200 ties |
| Robot breakdown | 0.00, 200 ties | −0.04 [−0.09, 0.00], 3 / 197 / 0 | 0.00, 200 ties |
| Mix 35/25/25/15 | −0.04 [−0.09, −0.01], 4 / 196 / 0 | −0.04 [−0.09, 0.00], 6 / 192 / 2 | 0.00, 200 ties |

Success under a transient delay: source-only 80 %, DWAFP 86 % (no noise).

- **Transient delays: DWAFP beats source-only** (14 wins, no loss). When the source cannot be moved
  usefully, DWAFP can still edit a task downstream of it, typically before that task starts.
  Source-only can act on that task only once the task itself runs late, which is often too late.
- **Persistent obstructions: identical.** The obstructed source never finishes on its robot, so
  its output exists only if the source itself moves; no single edit of a descendant can restore
  it. Both methods find that move.
- **DWAFP = mission-wide** in all 2000 paired runs. State it narrowly: *"In the evaluated
  single-deviation scenarios, restricting edits to the repair region did not exclude the candidate
  selected by mission-wide single-edit search."*

### 1.5 By source class (no noise)

The delay scenarios split by where the consequences of the source task can go (sudden failures are
not split). Rows: deviation type × source class. Columns: how many scenarios (n), then the paired
differences in misses against reactive and against source-only.

| Source class | Meaning |
|---|---|
| downstream dependency (same robot) | tasks depend on the source, all on the source's own robot |
| cross-robot handoff | at least one task that depends on it (directly or not) runs on another robot |

| Deviation | Source class | n | DWAFP − reactive [95 % CI] | DWAFP − source-only [95 % CI] |
|---|---|---|---|---|
| Transient delay | downstream dependency (same robot) | 41 | −0.20 [−0.39, −0.05] | −0.05 [−0.12, 0.00] |
| | cross-robot handoff | 159 | −0.38 [−0.52, −0.25] | **−0.11** [−0.18, −0.04] |
| Persistent obstruction | downstream dependency (same robot) | 41 | −1.15 [−1.71, −0.66] | 0.00 |
| | cross-robot handoff | 159 | −2.30 [−2.75, −1.86] | 0.00 |

The preventive gain is about twice as large when the consequences cross robots, against reactive
and against source-only alike.

### 1.6 Decision latency (per decision, no noise, µs)

Rows: deviation types. Columns: the per-decision time percentiles of DWAFP, then of mission-wide
search (wall clock on the development PC, microseconds; 1000 µs = 1 ms; they vary slightly from
run to run, the values below are those of the committed `main_noise0.txt`).

| Deviation | DWAFP p50 / p95 / p99 | Mission-wide p50 / p95 / p99 |
|---|---|---|
| Transient delay | 16 / 34 / 341 | 16 / 33 / 618 |
| Persistent obstruction | 52 / 683 / 964 | 51 / 879 / 1047 |
| Mix | 16 / 140 / 665 | 16 / 371 / 772 |

All decisions are around or below 1 ms on 20-task missions. The difference between local and
mission-wide search is small at this size; it must be measured at larger task counts (Step 6).

### 1.7 Data

Commands: see "How to reproduce the results" at the top. Data: [main_noise0.csv](main_noise0.csv), [main_noise10.csv](main_noise10.csv) (one line per
scenario and method, with the configuration in every line); full terminal output:
[main_noise0.txt](main_noise0.txt), [main_noise10.txt](main_noise10.txt).

---

### 1.8 Sensitivity and robustness (Step 6)

One factor is changed at a time; everything else is the controlled configuration of 1.1, on the
same 200 paired scenarios per row (shapes in turn). Columns: success of reactive / source-only /
DWAFP; deadline-miss rate of reactive / DWAFP; paired differences in misses per run (negative =
DWAFP better) with 95 % CI and W / T / L; harmful / unneeded DWAFP runs. DWAFP − mission-wide was
0.00 (all ties) in **every** row. Data: [sweeps.csv](sweeps.csv), full output: [sweeps.txt](sweeps.txt).

#### Deadline structure: every task vs handoff and final tasks only

| Deadlines | Deviation | Success R / S / D | Miss rate R / D | DWAFP − reactive | DWAFP − source-only | Harm / unneeded |
|---|---|---|---|---|---|---|
| every task | transient delay | 70 / 80 / 86 % | 2.9 / 1.2 % | −0.34 [−0.47, −0.23], 39/161/0 | **−0.10** [−0.15, −0.04], 14/186/0 | 0 / 0 |
| every task | persistent obstruction | 8 / 65 / 65 % | 18.4 / 8.1 % | −2.06 [−2.46, −1.70], 115/85/0 | 0.00, 200 ties | 0 / 0 |
| handoff + final | transient delay | 78 / 87 / 91 % | 3.0 / 0.9 % | −0.24 [−0.34, −0.15], 31/169/0 | **−0.06** [−0.10, −0.03], 10/190/0 | 0 / 0 |
| handoff + final | persistent obstruction | 20 / 79 / 79 % | 20.8 / 6.3 % | −1.58 [−1.91, −1.28], 117/83/0 | 0.00, 200 ties | 0 / 0 |

(Sudden failures and breakdowns: 200 ties against reactive and source-only in both structures.)

Per shape (delays): the gain over reactive holds for every shape, largest for fork-join
(persistent: −3.37 every task, −2.94 handoff + final) and smallest for rendezvous (−0.71 / −0.58).
Against source-only, every shape gains on transient delays (DWAFP − source-only, every task /
handoff + final):

| Shape | Every task | Handoff + final |
|---|---|---|
| random DAG | −0.17 [−0.28, −0.09], 16/184/0 | −0.12 [−0.22, −0.05], 13/187/0 |
| chains | −0.15 [−0.23, −0.08], 19/181/0 | −0.04 [−0.07, −0.01], 6/194/0 |
| fork-join | −0.14 [−0.23, −0.07], 16/184/0 | −0.14 [−0.21, −0.07], 16/184/0 |
| rendezvous | −0.06 [−0.10, −0.03], 12/187/1 | −0.05 [−0.09, −0.02], 11/188/1 |

Persistent: 200 ties against source-only in every shape. Rendezvous is the weakest shape: the
transient gain over reactive is small (−0.10 / −0.09), DWAFP loses 1 scenario to reactive and 1 to
source-only, and 1–4 of its runs are harmful (an edit on a rendezvous partner can delay the
meeting).

**Finding.** With the corrected baseline, DWAFP beats source-only on transient delays in every
shape and both deadline structures, with at most 1 loss per 200 scenarios. Handoff-and-final
deadlines do not enlarge the gap. Which conditions make downstream edits matter is the subject of
the repair-location study (pending, Section 5).

#### Duration noise

| Noise | Transient: DWAFP − reactive | Persistent: DWAFP − reactive | Failure / breakdown: DWAFP − reactive | DWAFP − source-only (transient / persistent) | Unneeded (transient) |
|---|---|---|---|---|---|
| 0 % | −0.34 [−0.47, −0.23] | −2.06 [−2.46, −1.70] | 0.00 / 0.00 | **−0.10** / 0.00 | 0 |
| 5 % | −0.34 [−0.46, −0.24] | −2.12 [−2.52, −1.76] | −0.03 / −0.04 | **−0.10** / −0.01 | 3 |
| 10 % | −0.34 [−0.47, −0.24] | −2.10 [−2.50, −1.74] | −0.06 / −0.06 | **−0.10** / −0.01 | 7 |
| 20 % | −0.39 [−0.52, −0.28] | −1.95 [−2.33, −1.57] | −0.17 / −0.20 | **−0.17** / **−0.14** | 7 |

The preventive gain on delays is stable up to 20 % noise, and so is the gain over source-only on
transient delays (every interval excludes 0; at 20 % also on persistent obstructions,
−0.14 [−0.24, −0.04]). With noise DWAFP loses a few scenarios (at most 2 / 200 to reactive, at most
3 / 200 to source-only). The growing gain on failures and
breakdowns comes from preventing noise-induced delays, not from the injected event. Unneeded
interventions stay rare (at most 7 / 200); harmful ones at most 4 / 200 (20 %, breakdowns).

#### Dispatch latency (decision to activation, the same for every method)

| Latency | Transient: DWAFP − reactive | Persistent: DWAFP − reactive | Persistent success R / D | Transient: DWAFP − source-only |
|---|---|---|---|---|
| 0 s | −0.34 [−0.47, −0.23] | −2.06 [−2.46, −1.70] | 8 / 65 % | −0.10 [−0.15, −0.04], 14/186/0 |
| 1 s | −0.36 [−0.49, −0.24] | −1.59 [−1.95, −1.27] | 5 / 48 % | −0.12 [−0.20, −0.07], 17/183/0 |
| 3 s | −0.33 [−0.47, −0.20] | −1.11 [−1.47, −0.79] | 2 / 26 % | **−0.23 [−0.35, −0.12], 24/176/0** |

Latency hurts prevention of persistent obstructions (the rescued task starts later), but the gain
over reactive remains significant at 3 s. The gain over source-only on transient delays grows with
latency (−0.10 → −0.12 → −0.23, never a loss): downstream edits can absorb the latency (their task
waits for its inputs anyway), which edits of the source cannot.

#### Remaining-time estimator while blocked

| Estimator | Transient: DWAFP − reactive | Persistent: DWAFP − reactive | Transient: DWAFP − source-only | Unneeded (transient) |
|---|---|---|---|---|
| nominal (primary) | −0.34 [−0.47, −0.23] | −2.06 [−2.46, −1.70] | −0.10 [−0.15, −0.04], 14/186/0 | 0 |
| conservative | −0.35 [−0.48, −0.25] | −2.06 [−2.46, −1.70] | −0.04 [−0.10, +0.01], 9/189/2 | 12 |

The benefit over reactive does not depend on the estimation assumption. The conservative estimate
adds 12 unneeded interventions for transient delays and makes the gain over source-only smaller and
not significant (2 losses), which supports the
nominal estimate as the primary configuration. Persistent obstructions are unaffected, since both
estimators use the robot's explicit diagnostic.

#### Execution timeout λ (robot-side budget, the same for every method)

| λ | Transient: DWAFP − reactive | Persistent: DWAFP − reactive | Persistent success R / D | Transient success R / D |
|---|---|---|---|---|
| 1.5 | −0.17 [−0.26, −0.09], 21/177/2 | −1.15 [−1.39, −0.91] | 18 / 65 % | 51 / 55 % |
| 2.0 (primary) | −0.34 [−0.47, −0.23], 39/161/0 | −2.06 [−2.46, −1.70] | 8 / 65 % | 70 / 86 % |
| 3.0 | −0.34 [−0.47, −0.23], 39/161/0 | −3.00 [−3.50, −2.54] | 1 / 65 % | 70 / 86 % |

For persistent obstructions the gain over reactive grows with the timeout: the longer the robot waits before
declaring failure, the later reactive recovery starts, while DWAFP acts on the feedback either way.
A tight budget (1.5) turns many transient delays into timeouts for every method (success 51–56 %)
and halves the transient gain (λ = 3 changes nothing for transient delays, which end before 2×); DWAFP then loses 2 of 200 scenarios. The gain remains significant
for every λ. λ = 2 is used for the main results without tuning it for any method. Against
source-only on transient delays: −0.06 [−0.14, +0.01] (12/183/5) at λ = 1.5, −0.10 at λ = 2 and 3.

---

## 2. Stage I at the prediction level (open loop, earlier)

These results evaluate single-edit candidates on **predictions** (no execution); program
`stage1_experiments`, 300 events per configuration. Kept for the locality and complexity claims.

- **Region vs global single-edit search:** the repair region kept **100 %** of the repairs found by
  mission-wide single-edit search in every configuration (normal, tight and very tight slack), with
  about 40 % fewer candidates and search time.
- **Weak scaling (5 tasks per robot):** the repair region stayed small while the mission grew 8×:
  share of remaining tasks in the region (L_R) 76 % → 58 % → 41 % → **23 %** for 2 / 4 / 8 / 16 robots
  (10 / 20 / 40 / 80 tasks); mean decision time 0.09 → 8.1 ms, worst 43 ms.
- **Queue pressure (3 robots, 10 → 80 tasks):** L_R stays around 65 %; long fixed queues propagate
  delays to every later task of the queue.
- **Infeasibility certificate:** never claimed infeasibility where the mission-wide search found a
  repair (400 randomized events); identifies up to 60 % of events under very tight slack as locally
  unrepairable without search.
- **Dispatch latency:** at 3 s, single-edit coverage drops from 86 % to 57 %; the certificate
  proves 22 % of events locally infeasible. Retry suffers most (it repairs upstream).

Caveat: the open-loop scaling used cycling robot speeds; the closed-loop scaling (Step 6) uses
homogeneous rates or fleet sizes that preserve the speed mix.

---

## 3. Claims

One row per claim the paper may make. Evidence: the sections supporting it. Status: **supported**
(the evidence is clear), **promising** (consistent so far, but limited), **not yet supported**
(current results do not show it), **pending** (the experiment is still to run).

| Claim | Evidence | Status |
|---|---|---|
| Prevention improves over reactive recovery | 1.2, 1.3: −0.34 and −2.06 misses/run on delays, no losses; 1.8: significant for every deadline structure, shape, noise level up to 20 %, latency up to 3 s, both estimators and every timeout | **Supported** |
| Shared recovery behaves identically after unannounced failures | 1.2: 200 ties for failures and breakdowns | **Supported** |
| The benefit is robust to duration noise | 1.3, 1.8: stable from 0 to 20 % noise | **Supported** |
| The benefit does not depend on the blockage estimator | 1.8: nominal and conservative give the same gains | **Supported** (nominal has fewer unneeded edits) |
| The gain over reactive depends on the execution timeout | 1.8: −1.15 / −2.06 / −3.00 for λ = 1.5 / 2 / 3 (persistent) | **Reported** (λ = 2 untuned) |
| Dependency-aware repair outperforms trigger-task-only repair | 1.4: −0.10 [−0.15, −0.04] (14/186/0) on transient delays; 1.8: every shape, deadline structure, noise level and latency | **Supported for transient delays**; persistent obstructions: identical by construction (1.4) |
| Local search preserves global single-edit quality | 1.4: 2000 ties; 2: 100 % kept | **Promising** (single-deviation scenarios only) |
| Local search substantially reduces computation | 1.6: small difference at 20 tasks; 2: −40 % candidates | **Pending** (closed-loop scaling) |
| No harmful or unneeded preventive edits | 1.2: none without noise; 1.3: 7 unneeded / 200 transient runs with noise | **Supported**, with the noise caveat |

---

## 4. Statements to phrase carefully

1. **Persistent obstruction gain depends on the robot's diagnostic.** *"The robot reports whether
   same-robot retry remains feasible; DWAFP uses this information before the execution timeout."*
   The gain does not depend on the remaining-time estimator (1.8), only on this diagnostic.
2. **Mission-wide equality is empirical, not a guarantee** (see the sentence in 1.4).
3. **Shared recovery can be harmful in rare breakdown cases** (3 / 200 runs, every repair-enabled
   method alike): it inserts the stranded tasks at the front of other robots' queues, which can
   delay those robots' own tasks past their deadlines. A property of the greedy recovery, not of
   DWAFP.
4. **Tardiness** is of completed tasks only; always read it together with the completion rate.
   **Makespan** is compared only over runs where every task completed.

---

## 5. Pending (Step 6)

Done: deadline structure, noise sweep, dispatch latency, estimator, execution timeout (1.8).

Still to run:
1. Repair-location factorial study: when do downstream edits matter (final-only deadlines, source
   progress, idle-spare availability, execution-time ratio, slack, the same latency for every
   method)?
2. Closed-loop scaling, weak scaling N = qR with R ∈ {3, 6, 9, 12} and q ≈ 5 or 10 tasks per robot,
   homogeneous rates (optionally fixed N with varying R): region share, validation-horizon share,
   candidates per decision for DWAFP and mission-wide, decision-time p50 / p95 / p99, miss
   difference DWAFP − mission-wide; release build, with processor, compiler and optimization flags.
3. Failure taxonomy of the events DWAFP cannot prevent (7 classes, with a pair oracle).
