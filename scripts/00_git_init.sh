#!/bin/bash
# =============================================================
#  00_git_init.sh  —  One-time GitHub setup
#  Run this ONCE from inside the Gujcost_Files/ folder.
#
#  What it does:
#    1. Initialises a local git repo
#    2. Makes the first commit (just README + .gitignore)
#    3. Pushes to GitHub
#
#  Before running:
#    1. Create an empty repo on GitHub (no README, no .gitignore)
#    2. Copy the repo URL (https or ssh)
#    3. Run:  bash scripts/00_git_init.sh <your-github-url>
#
#  Example:
#    bash scripts/00_git_init.sh https://github.com/yourname/gujcost-robofest.git
#    bash scripts/00_git_init.sh git@github.com:yourname/gujcost-robofest.git
# =============================================================

set -e   # exit immediately if any command fails

REPO_URL="$1"

if [ -z "$REPO_URL" ]; then
    echo "ERROR: Please provide your GitHub repo URL."
    echo "Usage: bash scripts/00_git_init.sh https://github.com/yourname/repo.git"
    exit 1
fi

# Make sure we're in the repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

echo ""
echo "=== Gujcost Robofest — GitHub Init ==="
echo "Repo root : $REPO_ROOT"
echo "Remote    : $REPO_URL"
echo ""

# ── Step 1: git init ──────────────────────────────────────────
if [ -d ".git" ]; then
    echo "[SKIP] Git repo already initialised."
else
    git init
    echo "[OK] git init"
fi

# ── Step 2: Set identity if not set globally ──────────────────
if ! git config user.email > /dev/null 2>&1; then
    read -p "Git email (for commits): " GIT_EMAIL
    read -p "Git name  (for commits): " GIT_NAME
    git config user.email "$GIT_EMAIL"
    git config user.name  "$GIT_NAME"
fi

# ── Step 3: Add remote ────────────────────────────────────────
if git remote get-url origin > /dev/null 2>&1; then
    echo "[SKIP] Remote 'origin' already set."
else
    git remote add origin "$REPO_URL"
    echo "[OK] Remote added: $REPO_URL"
fi

# ── Step 4: First commit — only README + .gitignore ──────────
# The rest will be pushed gradually by daily_push.py
git add README.md .gitignore Drone_Robofest_PS.pdf
git commit -m "Initial commit: project overview and gitignore

Gujcost Robofest 6.0 — Aerial Robotics Minefield Navigation

Setting up the repository for the drone swarm project.
Problem statement PDF included.

Team: Gujcost 2026"

echo ""
echo "[OK] Initial commit created."

# ── Step 5: Push ──────────────────────────────────────────────
git branch -M main
git push -u origin main

echo ""
echo "======================================="
echo " Setup complete!"
echo " Next step: run  python3 scripts/daily_push.py"
echo " Run it once per day (or set up a cron job)."
echo "======================================="
