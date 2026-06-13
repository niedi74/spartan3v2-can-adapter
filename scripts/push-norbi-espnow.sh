#!/usr/bin/env bash
# Push 4 branches to niedi74/norbi-espnow and mirror waveshare to waveshare-vdo-clock.
# Run from repo root: bash scripts/push-norbi-espnow.sh
set -euo pipefail

NORBI_REPO="${NORBI_REPO:-https://github.com/niedi74/norbi-espnow.git}"
WAVE_REPO="${WAVE_REPO:-https://github.com/niedi74/waveshare-vdo-clock.git}"
M5_REPO="${M5_REPO:-https://github.com/niedi74/m5stack-123.git}"
M5_BRANCH="${M5_BRANCH:-feature/spartan-live-display}"
WAVE_BRANCH="${WAVE_BRANCH:-cursor/webgui-ota-c56e}"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

WS="$WORKDIR/waveshare-seed"
M5_DIR="$WORKDIR/m5"
MONO="$WORKDIR/monorepo"
SEED="$ROOT/scripts/monorepo/waveshare-seed"

mkdir -p "$WS"/{src,include,assets/t2b-clock,docs} "$MONO"

echo "==> Clone M5 source ($M5_BRANCH)"
git clone --depth 1 -b "$M5_BRANCH" "$M5_REPO" "$M5_DIR"

echo "==> Build waveshare seed"
cp "$SEED/README.md" "$SEED/platformio.ini" "$SEED/.gitignore" "$WS/"
cp "$SEED/src/main.cpp" "$WS/src/"
cp "$ROOT/include/spartan_cockpit_frame.h" "$WS/include/"
cp "$ROOT/assets/t2b-clock/"* "$WS/assets/t2b-clock/"
cp "$ROOT/docs/waveshare-round-cockpit.md" "$ROOT/docs/espnow-gateway-architecture.md" "$WS/docs/"

cp "$ROOT/scripts/monorepo/README.md" "$MONO/README.md"

copy_project() {
  local dest="$1" src="$2"
  rm -rf "$dest"
  mkdir -p "$dest"
  cp -a "$src"/. "$dest"/
  rm -rf "$dest/.git"
}

import_branch() {
  local name="$1" src="$2" msg="$3"
  local stage="$WORKDIR/stage-$name"
  copy_project "$stage" "$src"
  cd "$MONO"
  git checkout --orphan "$name"
  find . -mindepth 1 -maxdepth 1 ! -name '.git' ! -name '.' -print0 | xargs -0 rm -rf
  cp -a "$stage"/. .
  git add -A
  git commit -m "$msg"
  echo "  $name: $(git ls-files | wc -l) files"
}

cd "$MONO"
git init -q
git config user.email "${GIT_AUTHOR_EMAIL:-niedi74@users.noreply.github.com}"
git config user.name "${GIT_AUTHOR_NAME:-niedi74}"

git add README.md
git commit -m "docs: monorepo index — hub, m5, waveshare branches"
git branch -M main

echo "==> Import branches"
import_branch hub "$ROOT" "import: motorraum hub from spartan3v2-can-adapter main"
import_branch m5 "$M5_DIR" "import: M5 Dial from m5stack-123 $M5_BRANCH"
import_branch waveshare "$WS" "import: Waveshare round cockpit seed (ESP-NOW + VDO assets)"

git remote add origin "$NORBI_REPO"
echo "==> Push norbi-espnow (main hub m5 waveshare)"
git push -u origin main hub m5 waveshare

echo "==> Mirror waveshare -> waveshare-vdo-clock ($WAVE_BRANCH)"
git checkout waveshare
git remote add waveshare "$WAVE_REPO" 2>/dev/null || git remote set-url waveshare "$WAVE_REPO"
git push -u waveshare HEAD:"$WAVE_BRANCH"
git push -u waveshare HEAD:main

echo "Done."
echo "  git clone -b hub $NORBI_REPO"
echo "  git clone -b m5 $NORBI_REPO"
echo "  git clone -b waveshare $WAVE_REPO"
