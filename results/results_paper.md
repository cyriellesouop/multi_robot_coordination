# DWAFP: results for the paper

Living document: every result that may go into the paper, with its configuration, the raw data
file and the command that reproduces it. Newest results are added to the relevant section; the
"Claims" table at the end says what each result supports.

Last update: 2026-10-21.

---

## How to reproduce the results

From the workspace root (`~/Documents/ROS2_training/ros2_ws`), after building the `actions` package:

```bash
cd ~/Documents/ROS2_training/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select actions

./build/actions/dwafp_experiments 200 src/results/main_noise0.csv 0      # controlled
./build/actions/dwafp_experiments 200 src/results/main_noise10.csv 0.1   # robustness
```

| Argument | Meaning |
|---|---|
| `200` | paired scenarios per deviation type (each one executed under the five methods) |
| `src/results/main_noise0.csv` | output file, one line per scenario and method (`-` = no CSV) |
| `0` / `0.1` | duration noise: 0 = controlled configuration (Section 1.2), 0.1 = 10 % noise (Section 1.3) |

Each run takes a few seconds and prints the tables of Section 1; redirect the output to keep it
(`> src/results/main_noise0.txt`). The runs are deterministic: the same command gives the same
results.

---

## 1. Closed-loop comparison (simulator, executed outcomes)

### 1.1 Setup

Every scenario is **executed** in the discrete-event simulator under five methods, on exactly the
same mission, plan, deviation and noise (paired comparison):

| Method | Prevention (before failure) | Recovery (after an actual failure, timeout, breakdown or deadline miss) |
|---|---|---|
| No intervention | none | none |
| Reactive | none | common recovery |
| Source-only | edits restricted to the deviating task | common recovery |
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

Other outcome metrics (no noise, means per run):

| Deviation | Method | Miss rate | Completion rate | Completed-task tardiness (s) | Makespan (s, fully completed runs) |
|---|---|---|---|---|---|
| Transient delay | reactive | 2.9 % | 100 % | 0.74 | 59.7 (200 runs) |
| | DWAFP | 1.2 % | 100 % | 0.26 | 59.1 (200 runs) |
| Persistent obstruction | no intervention | 45.2 % | 55.5 % | 0.31 | — (0 runs) |
| | reactive | 18.4 % | 100 % | 13.90 | 62.9 (200 runs) |
| | DWAFP | 8.1 % | 100 % | 7.13 | 60.1 (200 runs) |

### 1.3 Robustness: the same comparison with 10 % duration noise

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

| Deviation | DWAFP − source-only, no noise | DWAFP − source-only, 10 % noise | DWAFP − mission-wide (both) |
|---|---|---|---|
| Transient delay | −0.02 [−0.06, 0.00], 2 / 198 / 0 | −0.03 [−0.07, +0.01], 4 / 195 / 1 | 0.00, 200 ties |
| Persistent obstruction | 0.00, 200 ties | −0.02 [−0.07, +0.01], 2 / 197 / 1 | 0.00, 200 ties |
| Outcome failure | 0.00, 200 ties | −0.01 [−0.03, +0.01] | 0.00, 200 ties |
| Robot breakdown | 0.00, 200 ties | −0.04 [−0.09, 0.00] | 0.00, 200 ties |

- **DWAFP ≈ source-only** in this configuration (deadlines on every task): the best edit is almost
  always on the deviating task itself. Not yet evidence for repair at alternative locations.
- **DWAFP = mission-wide** in all 2000 paired runs. State it narrowly: *"In the evaluated
  single-deviation scenarios, restricting edits to the repair region did not exclude the candidate
  selected by mission-wide single-edit search."*

### 1.5 By source class (no noise)

| Deviation | Source class | n | DWAFP − reactive [95 % CI] | DWAFP − source-only [95 % CI] |
|---|---|---|---|---|
| Transient delay | downstream dependency (same robot) | 41 | −0.20 [−0.39, −0.05] | 0.00 |
| | cross-robot handoff | 159 | −0.38 [−0.52, −0.25] | −0.03 [−0.08, 0.00] |
| Persistent obstruction | downstream dependency (same robot) | 41 | −1.15 [−1.71, −0.66] | 0.00 |
| | cross-robot handoff | 159 | −2.30 [−2.75, −1.86] | 0.00 |

The preventive gain is about twice as large when the consequences cross robots.

### 1.6 Decision latency (per decision, no noise, µs)

| Deviation | DWAFP p50 / p95 / p99 | Mission-wide p50 / p95 / p99 |
|---|---|---|
| Transient delay | 15 / 32 / 317 | 15 / 32 / 623 |
| Persistent obstruction | 54 / 729 / 1043 | 53 / 924 / 1210 |
| Mix | 16 / 141 / 682 | 16 / 387 / 783 |

All decisions are around or below 1 ms on 20-task missions. The difference between local and
mission-wide search is small at this size; it must be measured at larger task counts (Step 6).

### 1.7 Data

Commands: see "How to reproduce the results" at the top. Data: [main_noise0.csv](main_noise0.csv), [main_noise10.csv](main_noise10.csv) (one line per
scenario and method, with the configuration in every line); full terminal output:
[main_noise0.txt](main_noise0.txt), [main_noise10.txt](main_noise10.txt).

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

| Claim | Evidence | Status |
|---|---|---|
| Prevention improves over reactive recovery | 1.2, 1.3: −0.34 and −2.06 misses/run on delays, no losses | **Supported** |
| Shared recovery behaves identically after unannounced failures | 1.2: 200 ties for failures and breakdowns | **Supported** |
| The benefit is robust to duration noise | 1.3 | **Supported (10 %)**; noise sweep pending |
| Dependency-aware repair outperforms source-only repair | 1.4: nearly identical | **Not yet supported** (handoff+final deadlines pending) |
| Local search preserves global single-edit quality | 1.4: 2000 ties; 2: 100 % kept | **Promising** (single-deviation scenarios only) |
| Local search substantially reduces computation | 1.6: small difference at 20 tasks; 2: −40 % candidates | **Pending** (closed-loop scaling) |
| No harmful or unneeded preventive edits | 1.2: none without noise; 1.3: 7 unneeded / 200 transient runs with noise | **Supported**, with the noise caveat |

---

## 4. Statements to phrase carefully

1. **Persistent obstruction gain depends on the robot's diagnostic.** *"The robot reports whether
   same-robot retry remains feasible; DWAFP uses this information before the execution timeout."*
   Estimator sensitivity (nominal vs conservative) is pending.
2. **Mission-wide equality is empirical, not a guarantee** (see the sentence in 1.4).
3. **Shared recovery can be harmful in rare breakdown cases** (3 / 200 runs, every repair-enabled
   method alike): it inserts the stranded tasks at the front of other robots' queues, which can
   delay those robots' own tasks past their deadlines. A property of the greedy recovery, not of
   DWAFP.
4. **Tardiness** is of completed tasks only; always read it together with the completion rate.
   **Makespan** is compared only over runs where every task completed.

---

## 5. Pending (Step 6)

1. Deadline structure: handoff + final deadlines (where alternative repair locations should matter).
2. Noise sweep: 0, 5, 10, 20 %.
3. Dispatch latency: 0, 1, 3 s.
4. Estimator: nominal vs conservative.
5. Execution timeout λ ∈ {1.5, 2, 3} (λ = 2 primary, the same for every method).
6. Closed-loop scaling with homogeneous rates or R ∈ {3, 6, 9, 12}.
7. Failure taxonomy of the events DWAFP cannot prevent (7 classes, with a pair oracle).
