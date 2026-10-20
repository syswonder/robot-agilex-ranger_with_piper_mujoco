#!/usr/bin/env bash

# Resolve the Robonix checkout without assuming where this body package is
# installed. This file is sourced by scripts/env.sh and sim/start.sh.
resolve_robonix_paths() {
  local project_root="${1:?project root required}"
  local api_path="${ROBONIX_API_PATH:-}"
  local source_path="${ROBONIX_SOURCE_PATH:-${HUB_ROBONIX_SRC:-}}"
  local candidate

  if [[ -z "$api_path" && -n "$source_path" ]]; then
    api_path="$source_path/pylib/robonix-api"
  fi

  if [[ -z "$api_path" ]] && command -v rbnx >/dev/null 2>&1; then
    api_path="$(rbnx path robonix-api 2>/dev/null || true)"
  fi

  if [[ -z "$api_path" ]]; then
    for candidate in \
      "$project_root/../robonix/pylib/robonix-api" \
      "$project_root/../../robonix/pylib/robonix-api" \
      "${XDG_DATA_HOME:-$HOME/.local/share}/robonix-console/robonix/pylib/robonix-api"; do
      if [[ -f "$candidate/robonix_api/__init__.py" ]]; then
        api_path="$candidate"
        break
      fi
    done
  fi

  if [[ ! -f "$api_path/robonix_api/__init__.py" ]]; then
    cat >&2 <<EOF
[robonix] unable to locate robonix-api.
[robonix] resolved path: ${api_path:-<empty>}
[robonix] set one of these in $project_root/.env:
[robonix]   ROBONIX_SOURCE_PATH=/absolute/path/to/robonix
[robonix]   ROBONIX_API_PATH=/absolute/path/to/robonix/pylib/robonix-api
EOF
    return 1
  fi

  api_path="$(cd "$api_path" && pwd -P)"
  export ROBONIX_API_PATH="$api_path"

  if [[ -z "$source_path" ]]; then
    source_path="$(cd "$api_path/../.." && pwd -P)"
  elif [[ ! -d "$source_path" ]]; then
    echo "[robonix] ROBONIX_SOURCE_PATH is not a directory: $source_path" >&2
    return 1
  else
    source_path="$(cd "$source_path" && pwd -P)"
  fi
  export ROBONIX_SOURCE_PATH="$source_path"
}
