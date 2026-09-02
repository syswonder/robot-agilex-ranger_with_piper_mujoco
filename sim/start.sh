#!/usr/bin/env bash
# Start only the MuJoCo simulator. Run `rbnx boot` separately from the
# repository root after this script reports that the bridge is ready.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNTIME="$ROOT/.runtime"

if [[ -f "$ROOT/.env" ]]; then
  set -a
  source "$ROOT/.env"
  set +a
fi

export ROBONIX_SOURCE_PATH="${ROBONIX_SOURCE_PATH:-$ROOT/../robonix}"
export ROBONIX_SIM_CONTAINER="${ROBONIX_SIM_CONTAINER:-mujoco_robonix_sim}"
# Do not inherit a generic ROS shell's RMW_IMPLEMENTATION. The Humble bridge
# image installs Fast DDS; other Robonix workspaces commonly export Zenoh.
export RMW_IMPLEMENTATION="${MUJOCO_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
HOST_PYTHON="${HOST_PYTHON:-/usr/bin/python3}"

environment="${SIM_ENVIRONMENT:-scenesmith_house_187}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --environment|-e)
      [[ $# -ge 2 ]] || { echo "[sim/start] --environment requires an ID" >&2; exit 2; }
      environment="$2"
      shift 2
      ;;
    --help|-h)
      echo "Usage: $0 [--environment ENVIRONMENT_ID]"
      exit 0
      ;;
    *)
      echo "[sim/start] unknown argument: $1" >&2
      echo "Usage: $0 [--environment ENVIRONMENT_ID]" >&2
      exit 2
      ;;
  esac
done

if [[ ! -d "$ROOT/node_modules/playwright" ]]; then
  echo "[sim/start] dependencies are missing; run: bash scripts/bootstrap.sh" >&2
  exit 1
fi
if [[ ! -x "$HOST_PYTHON" ]]; then
  echo "[sim/start] host Python is unavailable: $HOST_PYTHON" >&2
  exit 1
fi

mkdir -p "$RUNTIME"
if [[ -f "$RUNTIME/sim.pid" ]]; then
  previous_pid="$(cat "$RUNTIME/sim.pid")"
  if kill -0 "$previous_pid" 2>/dev/null; then
    echo "[sim/start] simulator launcher is already running (pid=$previous_pid)" >&2
    exit 1
  fi
fi

# Clean stale browser/server/container state left by an interrupted launcher.
bash "$SCRIPT_DIR/stop.sh" >/dev/null 2>&1 || true
echo $$ >"$RUNTIME/sim.pid"

cleanup() {
  status=$?
  trap - EXIT INT TERM HUP
  bash "$SCRIPT_DIR/stop.sh"
  exit "$status"
}
trap cleanup EXIT INT TERM HUP

cd "$ROOT"
compose=(docker compose -f sim/compose.yaml)
bridge_log="$RUNTIME/bridge.log"
: >"$bridge_log"

capture_bridge_log() {
  local attempt="$1"
  {
    echo "===== bridge startup attempt $attempt ====="
    docker inspect "$ROBONIX_SIM_CONTAINER" \
      --format 'status={{.State.Status}} exit={{.State.ExitCode}} restarts={{.RestartCount}} error={{.State.Error}}' \
      2>&1 || true
    docker logs "$ROBONIX_SIM_CONTAINER" 2>&1 || true
  } >>"$bridge_log"
}

bridge_http_ready() {
  local health
  health="$(curl -fsS http://127.0.0.1:8766/health 2>/dev/null || true)"
  [[ -n "$health" ]] && "$HOST_PYTHON" -c \
    'import json,sys; h=json.loads(sys.argv[1]); assert isinstance(h.get("browserConnected"), bool) and "frames" in h' \
    "$health" >/dev/null 2>&1
}

bridge_ready=false
for attempt in 1 2 3; do
  echo "[sim/start] starting ROS 2 bridge container (attempt $attempt/3)"
  "${compose[@]}" up -d

  # The browser cannot connect until both Bridge listeners exist. Detect a
  # crash loop here so its stderr can be saved before cleanup removes it.
  for _ in $(seq 1 60); do
    if bridge_http_ready; then
      bridge_ready=true
      break
    fi
    state="$(docker inspect "$ROBONIX_SIM_CONTAINER" --format '{{.State.Status}}' 2>/dev/null || true)"
    if [[ "$state" == "exited" || "$state" == "dead" || "$state" == "restarting" ]]; then
      break
    fi
    sleep 0.25
  done
  if [[ "$bridge_ready" == true ]]; then
    break
  fi

  capture_bridge_log "$attempt"
  echo "[sim/start] bridge failed to become ready; recreating container" >&2
  "${compose[@]}" down >/dev/null 2>&1 || true
done

if [[ "$bridge_ready" != true ]]; then
  echo "[sim/start] bridge failed after 3 attempts; inspect $bridge_log" >&2
  tail -80 "$bridge_log" >&2 || true
  exit 1
fi

echo "[sim/start] starting static web server"
"$HOST_PYTHON" scripts/serve.py --bind 0.0.0.0 --port 5180 >"$RUNTIME/web.log" 2>&1 &
web_pid=$!
echo "$web_pid" >"$RUNTIME/web.pid"

for _ in $(seq 1 60); do
  curl -fsS http://127.0.0.1:5180/ >/dev/null 2>&1 && break
  kill -0 "$web_pid" 2>/dev/null || break
  sleep 0.25
done
curl -fsS http://127.0.0.1:5180/ >/dev/null || {
  echo "[sim/start] web server failed; inspect $RUNTIME/web.log" >&2
  exit 1
}

sim_url="${SIM_URL:-http://127.0.0.1:5180/?environment=$environment}"
echo "[sim/start] launching browser: $sim_url"
SIM_URL="$sim_url" node scripts/launch-browser.mjs >"$RUNTIME/browser.log" 2>&1 &
browser_pid=$!
echo "$browser_pid" >"$RUNTIME/browser.pid"

for _ in $(seq 1 360); do
  if curl -fsS http://127.0.0.1:8766/health 2>/dev/null \
    | "$HOST_PYTHON" -c 'import json,sys; h=json.load(sys.stdin); ready=h.get("browserConnected", False) and h.get("environment")==sys.argv[1] and h.get("frames", {}).get("state", 0)>0; raise SystemExit(not ready)' "$environment" 2>/dev/null; then
    break
  fi
  kill -0 "$browser_pid" 2>/dev/null || break
  sleep 0.5
done
health_json="$(curl -fsS http://127.0.0.1:8766/health 2>/dev/null || true)"
if [[ -z "$health_json" ]] || ! "$HOST_PYTHON" -c \
  'import json,sys; h=json.loads(sys.argv[2]); assert h.get("browserConnected", False) and h.get("environment")==sys.argv[1] and h.get("frames", {}).get("state", 0)>0; print("[sim/start] bridge ready:", h.get("environment"), h.get("frames"))' \
  "$environment" "$health_json"; then
  capture_bridge_log "browser-connect"
  echo "[sim/start] browser did not connect; inspect $RUNTIME/browser.log and $bridge_log" >&2
  exit 1
fi

echo
echo "[sim/start] MuJoCo UI:      $sim_url"
echo "[sim/start] Bridge health: http://127.0.0.1:8766/health"
echo "[sim/start] In terminal 2:  source scripts/env.sh && rbnx boot"
echo "[sim/start] Ctrl-C stops only the simulator."

# Keep this terminal tied to the browser, matching the Webots example's
# foreground simulator lifecycle.
wait "$browser_pid"
