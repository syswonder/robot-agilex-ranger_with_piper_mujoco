#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env.sh"
cd "$ROOT"
npm ci
npx playwright install chromium
docker compose -f sim/compose.yaml build
for package_dir in \
  primitives/chassis \
  primitives/lidar \
  primitives/imu \
  primitives/front_camera \
  primitives/wrist_camera \
  primitives/arm \
  skills/pick; do
  rbnx validate "$package_dir"
done
rbnx build -f robonix_manifest.yaml --no-update-check
echo "Build completed. Start the simulator with: bash sim/start.sh"
