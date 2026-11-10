# Preemptive Flexible Job Shop Scheduling Problem

**Vincent Boyer, María Angélica Salazar Aguilar.** A fragment-free constraint programming approach with preemptive
edge-finding for the preemptive flexible job shop scheduling problem. Submitted.

This folder holds the detailed results per instance and the best schedules found in the paper. The citation and the
arXiv link will be added here, and the source code of the solver (CPW) will be added with the preprint.

## Contents

| Folder | Content |
|---|---|
| `results/` | the raw results per instance of the main experiments (CSV) |
| `solutions/` | the best schedule found for every instance (JSON) and `check_solution.py`, which decodes and checks them |

The benchmarks are the 276 pFJSSP instances of Juvin, Houssin & Lopez (2023) (the flexible job shop benchmarks of
Barnes & Chambers, Brandimarte, Dauzère-Pérès & Paulli, Fattahi et al., Hurink et al. and Kacem et al.) and 78
preemptive job shop (pJSSP) instances: the 63 of Juvin, Hebrard, Houssin & Lopez (CP 2023) and swv01-15, from JSPLIB.

## Results (`results/`)

All runs: Apple M4 (macOS, native arm64), IBM ILOG CPLEX Studio 22.2.0, one thread per run, four runs in parallel.
The time limit includes the assignment MILP (10% of the limit).

**Note.** Results may differ slightly from one run to another and on other machines or CPLEX versions: the search is
time-limited, so its outcome depends on the speed of the computer, and the assignments returned by the MILP (which
determine the starting solutions) can change with the platform and the CPLEX version.

| File | Experiment |
|---|---|
| `cpw_pfjssp276_600s.csv` | CPW, default configuration (`PreemptiveNoOverlap` propagator), the 276 pFJSSP instances, 600 s |
| `cpw_lazy_pfjssp276_600s.csv` | same, with lazy overload cuts instead of the `PreemptiveNoOverlap` propagator |
| `cpw_pjssp78_60s.csv`, `cpw_pjssp78_600s.csv` | CPW, default configuration, the 78 pJSSP instances, 60 and 600 s |
| `cpw_lazy_pjssp78_60s.csv`, `cpw_lazy_pjssp78_600s.csv` | same, with lazy overload cuts |

| Column | Meaning |
|---|---|
| `instance`, `family`, `jobs`, `machines`, `operations` | instance and its size |
| `time_limit_s` | time limit of the run, MILP included |
| `status`, `proven` | `OPTIMAL` (= `proven` 1: UB = LB, or the search completed) or `FEASIBLE` |
| `ub`, `lb` | final upper and lower bounds (makespan) |
| `time_to_best_s` | time at which the final upper bound was found |
| `run_time_s`, `cpu_time_s` | wall-clock and CPU time of the run (one thread) |
| `wall_time_includes_sleep` | 1 if the computer slept during the run (4 runs of the lazy variant): the wall time includes the sleep, the CPU time does not; use `cpu_time_s` |
| `milp_lb`, `milp_proven`, `milp_seconds`, `milp_rounds`, `milp_constraints`, `milp_pool_size` | the assignment MILP: its lower bound, whether it was solved to optimality, its time, its rounds of lazy Carlier constraints, the constraints in its final model, the size of its solution pool |
| `start_best`, `start_milp_edd`, `start_pool_edd`, `start_wgreedy` | makespan of the starting solution, and of each candidate: EDD on the MILP incumbent, best EDD on the other pool assignments, window greedy (empty when not built) |
| `cp_branches`, `cp_fails`, `searches`, `solutions` | CP Optimizer's branches and failures, searches started (1 with the propagator; one per round of cuts in the lazy variant), solutions found |
| `overload_constraints`, `repairs`, `repair_kept` | lazy variant: overload constraints posted, EDD repairs of violated solutions, repairs that kept the solution's makespan |
| `pno_calls`, `pno_pruning`, `pno_fails`, `pno_seconds` | `PreemptiveNoOverlap`: calls, calls that pruned, failures, time |
| `fragments`, `preempted_ops` | fragments and preempted operations of the final schedule |

The lazy variant stops between two searches, so some of its runs exceed the time limit in wall-clock time: by up
to 10 s on the pFJSSP (never in CPU time) and up to 21 s on the pJSSP, where three runs also exceed it in CPU time
(swv11 and swv15 at 60 s, swv15 at 600 s, by 19 s at most).

## Best schedules (`solutions/`)

`solutions_pfjssp276.json` (276 instances) and `solutions_pjssp78.json` (78 instances) give, for every instance, the
**best schedule found by the two methods of the paper**: CPW in its default configuration (with the
`PreemptiveNoOverlap` propagator) and CPW with lazy overload cuts, over all the runs above. Each entry gives:

| Key | Meaning |
|---|---|
| `makespan` | makespan of the schedule |
| `best_lb` | best lower bound of the two methods |
| `proven_optimal` | `makespan` = `best_lb` |
| `method` | the method that returned the schedule: `CPW (default configuration)`, or `CPW with lazy overload cuts instead of PreemptiveNoOverlap` where that method found a better schedule (28 pFJSSP instances) |
| `time_limit_s` | time limit of the run that returned it |
| `jobs` | the schedule, in compact form (below) |

On 28 pFJSSP instances the schedule thus comes from the lazy variant and is better than the upper bound of the
default configuration reported in the paper's main table.

**Compact form.** `jobs[j][k]` = `[machine, window end]` for the k-th operation of job j, in job order (machines
numbered from 0, as in the instance files). The windows of a job are back to back: an operation's window starts at
the end of its job predecessor's window, at 0 for the first operation. On each machine, Jackson's preemptive
schedule (at any time, process the released operation with the earliest window end) rebuilds the fragments of the
operations; by Horn's theorem it meets every window end if and only if the machine is feasible.

**Decoding and checking.** `check_solution.py` (Python 3, no dependencies) rebuilds the fragments and checks the
schedule: eligible machines, processing times, windows (hence the job order), no overlap on any machine, and the
makespan. The instance files are not included in this folder; it reads them in either format, detected automatically:

- pFJSSP: the usual flexible job shop text format (first line `<jobs> <machines>`; then one line per job:
  `<operations>`, then for each operation `<eligible machines>` followed by the pairs `<machine> <processing time>`,
  machines numbered from 0);
- pJSSP: the standard job shop format of JSPLIB (`github.com/tamy0612/JSPLIB`, files `instances/<name>`: lines
  starting with `#` ignored, `<jobs> <machines>`, then one line per job of `<machine> <processing time>` pairs,
  machines numbered from 0), or the same instance in the flexible format with one eligible machine per operation.

```sh
# one instance of a collection; --fragments prints the fragments [start, end) of every operation
python3 check_solution.py mk01.txt solutions_pfjssp276.json mk01 --fragments   # pFJSSP
python3 check_solution.py JSPLIB/instances/ft10 solutions_pjssp78.json ft10    # pJSSP, JSPLIB file
```

Notes on the instances: `dpp04` is the file `04a.txt` of the Dauzère-Pérès & Paulli set (bounds as reported for
`dpp04` by Juvin et al.); the operation of orb07 (and of e-, r-, v-orb7) with a processing time of 0 has no fragment.
All 354 schedules are feasible with the makespan they claim.
