#!/usr/bin/env python3
"""Decode and check pFJSSP solutions in the compact window format, independently of the solver.

A solution gives, for each job j and each of its operations k (in job order), the pair [machine, window end]
(machines numbered from 0, as in the instance files). The windows of a job are back to back: operation k's window
is [end of operation k-1's window, its own end), starting at 0 for the first operation. On each machine, Jackson's
preemptive schedule (at any time, run the released operation with the earliest window end) rebuilds the fragments;
by Horn's theorem it meets every window end if and only if the machine is feasible.

Checks: every machine is eligible for its operation; every window can hold its operation; Jackson's schedule meets
every window end (so each operation is fully processed inside its window, which enforces the job order); no machine
processes two operations at once; and the makespan equals the claimed one.

Usage:
  check_solution.py INSTANCE_FILE SOLUTION_JSON [--fragments]
  check_solution.py INSTANCE_FILE SOLUTIONS_JSON NAME [--fragments]  (one instance of a collection)
  check_solution.py --all SOLUTIONS_JSON INSTANCES_DIR        (a collection: {"instance": {...}, ...}; names resolved
                                                               through INSTANCES_DIR/instances.json, or as
                                                               INSTANCES_DIR/pjssp/<name>.fjs)
A solution object is {"makespan": int, "jobs": [[[machine, end], ...], ...]}; other keys are ignored.
"""
import heapq
import json
import os
import sys


def read_instance(path):
    """Returns (n_machines, jobs), jobs[j][k] = {machine: processing time} of the k-th operation of job j.
    Reads the flexible job shop format, or the standard job shop format of JSPLIB (lines starting with # ignored;
    `<jobs> <machines>`, then one line per job of <machine> <processing time> pairs, machines numbered from 0)."""
    with open(path) as f:
        lines = [l.split() for l in f if l.strip() and not l.lstrip().startswith("#")]
    head = lines[0]
    if len(head) == 2 and len(lines) == 1 + int(head[0]) and all(len(l) == 2 * int(head[1]) for l in lines[1:]):
        jobs = [[{int(l[2 * k]): int(l[2 * k + 1])} for k in range(int(head[1]))] for l in lines[1:]]
        return int(head[1]), jobs
    tokens = [t for l in lines for t in l]
    pos = 0

    def nxt():
        nonlocal pos
        pos += 1
        return int(float(tokens[pos - 1]))

    n_jobs, n_machines = nxt(), nxt()
    jobs = []
    for _ in range(n_jobs):
        ops = []
        for _ in range(nxt()):
            alts = {}
            for _ in range(nxt()):
                m = nxt()
                alts[m] = nxt()
            ops.append(alts)
        jobs.append(ops)
    return n_machines, jobs


def jackson(tasks):
    """Preemptive EDD on one machine. tasks: list of (release, deadline, p, name). Returns ({name: fragments}, missed)
    with fragments a list of [start, end), and missed the names completed after their deadline."""
    tasks = sorted(tasks, key=lambda t: (t[0], t[1]))
    frags, missed, ready, t, i = {}, [], [], 0, 0
    remaining = {}
    while i < len(tasks) or ready:
        if not ready and t < tasks[i][0]:
            t = tasks[i][0]
        while i < len(tasks) and tasks[i][0] <= t:
            r, d, p, name = tasks[i]
            remaining[name] = p
            heapq.heappush(ready, (d, name))
            i += 1
        d, name = ready[0]
        nxt_release = tasks[i][0] if i < len(tasks) else None
        run = remaining[name] if nxt_release is None else min(remaining[name], nxt_release - t)
        if run > 0:
            fr = frags.setdefault(name, [])
            if fr and fr[-1][1] == t:
                fr[-1][1] = t + run          # continues its previous fragment
            else:
                fr.append([t, t + run])
            remaining[name] -= run
            t += run
        if remaining[name] == 0:
            heapq.heappop(ready)
            if t > d:
                missed.append(name)
    return frags, missed


def check(instance_path, sol, show=False):
    n_machines, jobs = read_instance(instance_path)
    errors = []
    if len(sol["jobs"]) != len(jobs):
        return False, [f"{len(sol['jobs'])} jobs in the solution, {len(jobs)} in the instance"], None
    per_machine = {m: [] for m in range(n_machines)}
    for j, (ops, sops) in enumerate(zip(jobs, sol["jobs"])):
        if len(sops) != len(ops):
            errors.append(f"job {j}: {len(sops)} operations, {len(ops)} expected")
            continue
        start = 0
        for k, (alts, (m, end)) in enumerate(zip(ops, sops)):
            if m not in alts:
                errors.append(f"job {j} op {k}: machine {m} not eligible")
            elif end - start < alts[m]:
                errors.append(f"job {j} op {k}: window [{start}, {end}) shorter than p = {alts[m]}")
            else:
                per_machine[m].append((start, end, alts[m], (j, k)))
            start = end
    if errors:
        return False, errors, None
    makespan, fragments = 0, {}
    for m, tasks in per_machine.items():
        frags, missed = jackson(tasks)
        for name in missed:
            errors.append(f"machine {m}: job {name[0]} op {name[1]} cannot be processed within its window (Horn)")
        busy = sorted((a, b, name) for name, fr in frags.items() for a, b in fr)
        for (a1, b1, n1), (a2, b2, n2) in zip(busy, busy[1:]):
            if a2 < b1:
                errors.append(f"machine {m}: overlap of {n1} and {n2}")
        for (start, end, p, name) in tasks:
            fr = frags.get(name, [])
            if sum(b - a for a, b in fr) != p or any(a < start or b > end for a, b in fr):
                errors.append(f"job {name[0]} op {name[1]}: fragments {fr} not a processing of {p} in [{start}, {end})")
            fragments[name] = (m, fr)
            makespan = max(makespan, max((b for _, b in fr), default=0))
    if makespan != sol["makespan"]:
        errors.append(f"makespan {makespan}, claimed {sol['makespan']}")
    if show:
        for (j, k), (m, fr) in sorted(fragments.items()):
            print(f"  job {j} op {k}: machine {m}, fragments {fr}")
    return not errors, errors, makespan


def resolve(name, inst_dir):
    reg = os.path.join(inst_dir, "instances.json")
    if os.path.exists(reg):
        for e in json.load(open(reg)):
            if e["name"] == name:
                path = "dauzere/04a.txt" if name == "dpp04" else e["path"]  # see instances/README.md
                return os.path.join(inst_dir, path)
    path = os.path.join(inst_dir, "pjssp", name + ".fjs")
    if os.path.exists(path):
        return path
    raise FileNotFoundError(f"instance {name} not found in {inst_dir}")


def main(argv):
    if len(argv) >= 3 and argv[0] == "--all":
        sols = json.load(open(argv[1]))
        bad = 0
        for name, sol in sols.items():
            ok, errors, ms = check(resolve(name, argv[2]), sol)
            if not ok:
                bad += 1
                print(f"{name}: INFEASIBLE: {errors[:3]}")
        print(f"{len(sols) - bad} of {len(sols)} solutions feasible with the claimed makespan")
        return 1 if bad else 0
    if len(argv) >= 2:
        sol = json.load(open(argv[1]))
        if "jobs" not in sol:  # a collection: the instance name follows
            names = [a for a in argv[2:] if not a.startswith("--")]
            if not names or names[0] not in sol:
                print("SOLUTION_JSON is a collection: give the name of the instance, e.g. mk01")
                return 2
            sol = sol[names[0]]
        ok, errors, ms = check(argv[0], sol, show="--fragments" in argv)
        print(f"{'feasible' if ok else 'INFEASIBLE'}, makespan {ms}" + ("" if ok else f": {errors[:5]}"))
        return 0 if ok else 1
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
