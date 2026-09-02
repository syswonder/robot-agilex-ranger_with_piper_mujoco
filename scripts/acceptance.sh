#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/env.sh"
cd "$ROOT"

curl -fsS http://127.0.0.1:8766/health | python3 -c 'import json,sys; h=json.load(sys.stdin); assert h["ok"], h; print("bridge:", h["environment"], h["frames"])'

mapfile -t pick_args < <(python3 - <<'PY'
import json
import urllib.request

health = json.load(urllib.request.urlopen("http://127.0.0.1:8766/health"))
manifest = json.load(open("assets/environments/manifest.json", encoding="utf-8"))
environment = next(item for item in manifest["environments"] if item["id"] == health["environment"])
acceptance = environment.get("acceptance")
if not acceptance:
    raise SystemExit(f"environment {environment['id']} has no pick acceptance configuration")
print("--pick-object")
print(acceptance["pickObject"])
for point in acceptance.get("pickWaypoints", []):
    print("--pick-waypoint")
    print(point[0])
    print(point[1])
print("--pick-base-position")
print(acceptance["pickBasePosition"][0])
print(acceptance["pickBasePosition"][1])
PY
)

caps="$(rbnx caps)"
for provider in ranger_chassis mid360_lidar mid360_imu front_camera wrist_camera piper_ctl mapping nav2 scene soma; do
  grep -Eq "^● ${provider} \[ACTIVE\]" <<<"$caps" || { echo "provider is not ACTIVE: $provider" >&2; exit 1; }
done

docker exec \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
  "$ROBONIX_SIM_CONTAINER" \
  bash -lc 'source /opt/ros/humble/setup.bash && python3 /workspace/sim/tests/ros_acceptance.py "$@"' _ \
  --require-map --require-task-objects --motion --pick --pick-via-nav2 "${pick_args[@]}"

for node in controller_server planner_server bt_navigator behavior_server smoother_server waypoint_follower velocity_smoother; do
  state=""
  active=false
  for _ in $(seq 1 10); do
    if state="$(docker exec -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" "$ROBONIX_SIM_CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 lifecycle get /$node" 2>&1)" \
      && grep -q 'active \[3\]' <<<"$state"; then
      active=true
      break
    fi
    sleep 0.5
  done
  [[ "$active" == true ]] || { echo "$node is not active: $state" >&2; exit 1; }
done

echo "Robonix, ROS 2 sensors, mapping, Nav2, base motion, and stable pick/place checks passed."
