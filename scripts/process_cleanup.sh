#!/usr/bin/env bash
# Source this library; arguments are process groups created by this runner.
stop_process_groups() {
  local pid parent group signal deadline alive changed rows own_group
  local roots=("$@") groups=()
  local -A owned=() group_set=()
  own_group=$(ps -o pgid= -p "$BASHPID")
  own_group=${own_group//[[:space:]]/}
  for pid in "${roots[@]}"; do
    [[ "$pid" =~ ^[1-9][0-9]*$ && "$pid" != "$own_group" ]] || continue
    owned[$pid]=1
    group_set[$pid]=1
  done
  # PTY helpers can create additional sessions. Capture owned descendants
  # before signaling their parents so those children cannot become orphans.
  rows=$(ps -eo pid=,ppid=,pgid=)
  changed=true
  while [[ "$changed" == true ]]; do
    changed=false
    while read -r pid parent group; do
      [[ -n "$pid" && -n "${owned[$parent]:-}" && -z "${owned[$pid]:-}" ]] || continue
      owned[$pid]=1
      [[ "$group" == "$own_group" ]] || group_set[$group]=1
      changed=true
    done <<< "$rows"
  done
  groups=("${!group_set[@]}")
  for signal in INT TERM KILL; do
    for pid in "${groups[@]}"; do
      [[ "$pid" =~ ^[1-9][0-9]*$ ]] || continue
      kill -"$signal" -- "-$pid" 2>/dev/null || true
    done
    deadline=$((SECONDS + 2))
    while (( SECONDS < deadline )); do
      alive=false
      for pid in "${groups[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then alive=true; fi
      done
      [[ "$alive" == false ]] && break
      sleep 0.1
    done
  done
  for pid in "${roots[@]}"; do
    if kill -0 -- "-$pid" 2>/dev/null; then
      echo "Owned process group $pid did not exit after bounded cleanup" >&2
    else
      wait "$pid" 2>/dev/null || true
    fi
  done
}
