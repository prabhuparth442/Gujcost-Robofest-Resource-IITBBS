#!/usr/bin/env python3
"""
daily_push.py — Gradual GitHub push scheduler
==============================================
Pushes a new batch of files to GitHub each day according to push_schedule.json.
Run this once per day — manually or via cron.

HOW IT WORKS
------------
On first run, it records today as "day 0" in .push_state.json.
On subsequent runs it computes how many days have passed and pushes
all batches up to the current day (so skipped days catch up automatically).

USAGE
-----
    python3 scripts/daily_push.py           # push today's batch
    python3 scripts/daily_push.py --dry-run # preview what would be pushed
    python3 scripts/daily_push.py --status  # show progress without pushing
    python3 scripts/daily_push.py --day 5   # manually push a specific day

CRON (optional) — runs at 10am daily:
    0 10 * * * cd /path/to/Gujcost_Files && python3 scripts/daily_push.py >> logs/push.log 2>&1

REQUIREMENTS
------------
- git must be installed and on PATH
- GitHub remote 'origin' must be set (run 00_git_init.sh first)
- No authentication prompts: set up SSH key or git credential helper
"""

import json
import os
import subprocess
import sys
from datetime import date, datetime
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent
REPO_ROOT    = SCRIPT_DIR.parent
SCHEDULE_FILE = SCRIPT_DIR / "push_schedule.json"
STATE_FILE    = REPO_ROOT / ".push_state.json"   # gitignored — tracks progress
LOG_DIR       = REPO_ROOT / "logs"

# ── Helpers ───────────────────────────────────────────────────

def run(cmd, check=True, capture=False):
    """Run a shell command in the repo root."""
    result = subprocess.run(
        cmd, shell=True, cwd=REPO_ROOT,
        capture_output=capture, text=True)
    if check and result.returncode != 0:
        print(f"[ERROR] Command failed: {cmd}")
        if capture:
            print(result.stderr)
        sys.exit(1)
    return result

def load_schedule():
    with open(SCHEDULE_FILE) as f:
        data = json.load(f)
    # Support both a bare list and {"days": [...]} wrapper format
    return data["days"] if isinstance(data, dict) else data

def load_state():
    if STATE_FILE.exists():
        with open(STATE_FILE) as f:
            return json.load(f)
    return None

def save_state(state):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)

def days_since(date_str):
    start = date.fromisoformat(date_str)
    return (date.today() - start).days

def files_exist(files):
    """Return list of files that actually exist in the repo."""
    existing = []
    missing  = []
    for f in files:
        p = REPO_ROOT / f
        if p.exists():
            existing.append(f)
        else:
            missing.append(f)
    return existing, missing

def already_tracked(files):
    """Return files that git already knows about (committed or staged)."""
    result = run("git ls-files", capture=True)
    tracked = set(result.stdout.strip().splitlines())
    return [f for f in files if f in tracked]

# ── Core push logic ───────────────────────────────────────────

DEVLOG = REPO_ROOT / "DEVLOG.md"

def write_devlog(day_num, label, message):
    """Append a dated entry to DEVLOG.md. Creates the file on first call."""
    today = date.today().strftime("%Y-%m-%d")
    entry = f"\n## Day {day_num} — {today}\n**{label}**\n\n{message}\n"
    with open(DEVLOG, "a") as f:
        if DEVLOG.stat().st_size == 0 if DEVLOG.exists() else True:
            pass  # header written below on first creation
        f.write(entry)

def ensure_devlog_header():
    if not DEVLOG.exists() or DEVLOG.stat().st_size == 0:
        with open(DEVLOG, "w") as f:
            f.write("# Gujcost Robofest 6.0 — Development Log\n\n"
                    "Daily progress notes pushed incrementally to keep the repo active.\n")

def push_day(batch, dry_run=False):
    """Stage and commit one batch. Returns True on success."""
    day_num  = batch["day"]
    label    = batch["label"]
    message  = batch["message"]
    files    = batch["files"]

    print(f"\n{'='*55}")
    print(f" Day {day_num:>2}: {label}")
    print(f"{'='*55}")

    existing, missing = files_exist(files)
    already = already_tracked(existing)
    new_files = [f for f in existing if f not in already]

    if missing:
        print(f"  [WARN] Files not found (skipping): {missing}")

    # Always write a devlog entry — guarantees a real commit even when
    # all code files are already tracked (e.g. after a bulk push).
    ensure_devlog_header()
    write_devlog(day_num, label, message)
    new_files.append("DEVLOG.md")

    print(f"  Files to commit:")
    for f in new_files:
        tag = "+" if f != "DEVLOG.md" else "~"
        print(f"    {tag} {f}")

    if dry_run:
        print(f"  [DRY RUN] Would commit with message:")
        print(f"    {message.splitlines()[0]}")
        # Undo devlog write in dry-run so file isn't dirtied
        if DEVLOG.exists():
            lines = DEVLOG.read_text().splitlines()
            # Remove the entry we just appended (back to last blank line before ##)
            cut = next((i for i in range(len(lines)-1, -1, -1)
                        if lines[i].startswith("## Day")), None)
            if cut is not None:
                DEVLOG.write_text("\n".join(lines[:cut]).rstrip() + "\n")
        return True

    # Stage everything
    for f in new_files:
        run(f'git add "{f}"')

    # Verify something is staged
    staged_check = run("git diff --cached --quiet", check=False)
    if staged_check.returncode == 0:
        print(f"  [WARN] Nothing staged after add — skipping commit.")
        return True

    # Commit
    escaped = message.replace('"', '\\"')
    run(f'git commit -m "{escaped}"')
    print(f"  [OK] Committed {len(new_files)} file(s)")

    # Push
    result = run("git push origin main", check=False, capture=True)
    if result.returncode != 0:
        print(f"  [ERROR] Push failed:\n{result.stderr}")
        print("  Tip: check your internet connection and GitHub authentication.")
        return False

    print(f"  [OK] Pushed to GitHub ✓")
    return True

# ── Status display ────────────────────────────────────────────

def show_status(schedule, state):
    if state is None:
        print("No pushes made yet. Run 00_git_init.sh first, then run this script.")
        return

    start_date = state["start_date"]
    last_day   = state["last_day_pushed"]
    elapsed    = days_since(start_date)
    total      = len(schedule)

    print(f"\n{'='*55}")
    print(f" Gujcost GitHub Push Progress")
    print(f"{'='*55}")
    print(f" Start date   : {start_date}")
    print(f" Today        : {date.today()}")
    print(f" Days elapsed : {elapsed}")
    print(f" Last pushed  : Day {last_day} of {total}")
    print(f" Remaining    : {total - last_day} batches")
    print(f"{'='*55}")
    print()

    for batch in schedule:
        d = batch["day"]
        status = "✓ done" if d <= last_day else ("→ next" if d == last_day + 1 else "  pending")
        print(f"  Day {d:>2}  [{status}]  {batch['label']}")

    print()

# ── Main ──────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]
    dry_run    = "--dry-run" in args
    status_only = "--status" in args
    force_day  = None
    if "--day" in args:
        idx = args.index("--day")
        force_day = int(args[idx + 1])

    schedule = load_schedule()
    state    = load_state()

    if status_only:
        show_status(schedule, state)
        return

    # ── First run: initialise state ───────────────────────────
    if state is None:
        # Check git is initialised
        if not (REPO_ROOT / ".git").exists():
            print("[ERROR] No git repo found. Run 00_git_init.sh first.")
            sys.exit(1)

        state = {
            "start_date": date.today().isoformat(),
            "last_day_pushed": 0,
            "push_log": []
        }
        save_state(state)
        print(f"[INFO] First run. Start date set to {state['start_date']}.")

    # ── Determine which days to push ──────────────────────────
    if force_day is not None:
        days_to_push = [b for b in schedule if b["day"] == force_day]
        if not days_to_push:
            print(f"[ERROR] No batch defined for day {force_day}.")
            sys.exit(1)
    else:
        elapsed    = days_since(state["start_date"])
        last_pushed = state["last_day_pushed"]

        # Push everything from last_pushed+1 up to today's day number
        # (catches up if script wasn't run for a few days)
        target_day = min(elapsed, len(schedule))
        days_to_push = [b for b in schedule
                        if last_pushed < b["day"] <= target_day]

        if not days_to_push:
            next_batch = next((b for b in schedule if b["day"] > last_pushed), None)
            if next_batch:
                wait = next_batch["day"] - elapsed
                print(f"[INFO] Nothing to push today.")
                print(f"       Next batch (Day {next_batch['day']}: {next_batch['label']}) "
                      f"in {wait} day(s).")
            else:
                print("[INFO] All batches pushed! Repository is complete.")
            show_status(schedule, state)
            return

    # ── Push each batch ───────────────────────────────────────
    if dry_run:
        print("[DRY RUN MODE] No changes will be made.\n")

    LOG_DIR.mkdir(exist_ok=True)
    success_count = 0

    for batch in days_to_push:
        ok = push_day(batch, dry_run=dry_run)
        if ok and not dry_run:
            state["last_day_pushed"] = batch["day"]
            state["push_log"].append({
                "day": batch["day"],
                "label": batch["label"],
                "pushed_at": datetime.now().isoformat(timespec="seconds")
            })
            save_state(state)
            success_count += 1

    print(f"\n{'='*55}")
    if dry_run:
        print(f" [DRY RUN] Would have pushed {len(days_to_push)} batch(es).")
    else:
        print(f" Done: pushed {success_count}/{len(days_to_push)} batch(es) today.")
        remaining = len(schedule) - state["last_day_pushed"]
        print(f" Remaining batches: {remaining}")
        if remaining > 0:
            print(f" Run this script again tomorrow for the next batch.")
    print(f"{'='*55}\n")


if __name__ == "__main__":
    main()
