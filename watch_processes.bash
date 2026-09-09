#!/usr/bin/env bash
#
# watch_processes.bash - htop-style monitor for processes matching a string.
#
# Continuously lists all running process instances whose command line matches
# a given pattern (default: nuXmv), refreshing the display at a fixed interval.
# For every instance it shows the PID, how long it has been running (elapsed
# time) and the "config" (the exp_config_* segment of the nuXmv command line).
# New instances appear automatically.
# Terminated instances are kept in the list (marked "done") for a while: the
# most recently finished N instances remain visible with their final runtime.
# For finished nuXmv runs the result is shown too: "CEX" (a counterexample was
# found) or "blind" (nuXmv reported "no counterexample found").
#
# Usage:
#   ./watch_processes.bash [-i INTERVAL] [-n KEEP] [PATTERN]
#
#   PATTERN     String (extended regex) to match against the command line.
#               Defaults to "nuXmv".
#   -i INTERVAL Refresh interval in seconds (default: 2).
#   -n KEEP     Number of terminated instances to keep showing (default: 10,
#               use 0 to disable).
#
# Examples:
#   ./watch_processes.bash                 # watch nuXmv instances
#   ./watch_processes.bash vfm             # watch processes matching "vfm"
#   ./watch_processes.bash -i 1 nuXmv      # refresh every second
#   ./watch_processes.bash -n 20 nuXmv     # keep last 20 finished instances
#
# Press Ctrl-C to quit.

set -u

interval=2
keep=20

# --- Parse options ------------------------------------------------------------
while getopts ":i:n:h" opt; do
  case "${opt}" in
    i)
      interval="${OPTARG}"
      ;;
    n)
      keep="${OPTARG}"
      ;;
    h)
      grep '^#' "$0" | sed 's/^#//'
      exit 0
      ;;
    \?)
      echo "Invalid option: -${OPTARG}" >&2
      exit 1
      ;;
    :)
      echo "Option -${OPTARG} requires an argument." >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

pattern="${1:-nuXmv}"

if ! [[ "${interval}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Interval must be a positive number, got: ${interval}" >&2
  exit 1
fi

if ! [[ "${keep}" =~ ^[0-9]+$ ]]; then
  echo "Keep count must be a non-negative integer, got: ${keep}" >&2
  exit 1
fi

# --- Cleanup on exit ----------------------------------------------------------
cleanup() {
  # Show cursor again and move to a fresh line.
  printf '\033[?25h'
  echo
  exit 0
}
trap cleanup INT TERM

# Hide the cursor while running for a cleaner display.
printf '\033[?25l'

# --- Helpers ------------------------------------------------------------------
# Extract the "config" of a nuXmv invocation, i.e. the path segment that starts
# with "exp_config" (e.g. "exp_config_vehlen=8_vehwidth=6"). Falls back to "?"
# if no such segment is present. The result is truncated to the given width.
extract_config() {
  local args="$1" width="$2" cfg half
  if [[ "${args}" =~ (exp_config[^/[:space:]]*) ]]; then
    cfg="${BASH_REMATCH[1]}"
  else
    cfg="?"
  fi
  if (( ${#cfg} > width )); then
    half=$(( (width - 1) / 2 ))
    cfg="${cfg:0:half}…${cfg: -half}"
  fi
  printf '%s' "${cfg}"
}

# Determine the result of a finished nuXmv run by inspecting the counterexample
# file ("debug_trace_array.txt") that vfm writes into the config directory once
# the checker returns. A single run checks many invariants, so the file may hold
# several "no counterexample found" lines AND a counterexample; therefore we look
# for the positive counterexample marker first:
#   * "cex N" if the file contains a counterexample trace
#             ("Trace Type: Counterexample" / "as demonstrated by ..."), where N
#             is the CEX length = number of trace states ("-> State:" lines),
#   * "blind" if it only contains "no counterexample found" lines.
# To avoid reading a stale file from a previous run we only accept the file if
# it was modified at/after the process' start time. Echoes "cex N", "blind", or
# nothing (result not available yet).
resolve_result() {
  local dir="$1" min_mtime="$2" file mtime nstates
  [[ -z "${dir}" ]] && return 0
  file="${dir}/debug_trace_array.txt"
  [[ -f "${file}" ]] || return 0
  mtime="$(stat -c %Y "${file}" 2>/dev/null || echo 0)"
  (( mtime < min_mtime )) && return 0            # stale (from a previous run)
  if grep -q -E "Trace Type: Counterexample|as demonstrated by the following" "${file}" 2>/dev/null; then
    nstates="$(grep -c -E -- '-> State:' "${file}" 2>/dev/null)"
    printf 'cex %s' "${nstates}"
  elif grep -q "no counterexample found" "${file}" 2>/dev/null; then
    printf 'blind'
  fi
}

# --- Cross-iteration state ----------------------------------------------------
# We remember instances seen in the previous frame so we can detect when one
# disappears (terminates) and keep it in the list for a while.
declare -A prev_etime prev_short prev_start prev_dir  # live instances, prev frame
declare -A term_etime term_short term_start term_dir term_status term_len  # kept, done
term_order=()                      # terminated PIDs, oldest first (FIFO)

# Remove a PID from the terminated bookkeeping.
drop_terminated() {
  local target="$1" p new=()
  for p in "${term_order[@]}"; do
    [[ "${p}" == "${target}" ]] && continue
    new+=("${p}")
  done
  term_order=("${new[@]}")
  unset 'term_etime[${target}]' 'term_short[${target}]' \
        'term_start[${target}]' 'term_dir[${target}]' 'term_status[${target}]' \
        'term_len[${target}]'
}

# --- Main loop ----------------------------------------------------------------
while true; do
  now="$(date '+%Y-%m-%d %H:%M:%S')"

  # Collect matching processes. We match against the full command line (args).
  #
  # To avoid the matching pipeline (grep) matching *itself*, we wrap the first
  # character of the pattern in a character class, e.g. "nuXmv" -> "[n]uXmv".
  # The grep process' own command line then contains the literal "[n]uXmv",
  # which the regex "[n]uXmv" (= "nuXmv") does not match. We also drop this
  # script's own command line.
  #
  # ps fields:
  #   pid    - process id
  #   etime  - elapsed running time ([[dd-]hh:]mm:ss)
  #   etimes - elapsed running time in whole seconds (for sorting/summary)
  #   args   - full command line
  #
  # We also drop shell wrappers (e.g. `sh -c "LD_LIBRARY_PATH=... nuXmv ..."`)
  # that popen() spawns around the real binary, so each instance shows once.
  safe_pattern="[${pattern:0:1}]${pattern:1}"
  mapfile -t rows < <(
    ps -eo pid=,etime=,etimes=,args= 2>/dev/null \
      | grep -E -- "${safe_pattern}" \
      | grep -v -E -- "watch_processes" \
      | grep -v -E -- "(^|[[:space:]])sh -c " \
      | sort -k3,3 -n -r
  )

  count="${#rows[@]}"

  # Width available for the (truncated) config column, so rows never wrap.
  term_cols="${COLUMNS:-$(tput cols 2>/dev/null || echo 100)}"
  cmd_width=$((term_cols - 37))
  ((cmd_width < 20)) && cmd_width=20

  now_epoch="$(date +%s)"

  # Collect the current live instances into arrays (keyed by PID) so we can
  # both render them and diff against the previous frame.
  declare -A cur_etime=() cur_short=() cur_start=() cur_dir=()
  cur_pids=()
  for row in "${rows[@]}"; do
    read -r pid etime _etimes args <<<"${row}"
    short="$(extract_config "${args}" "${cmd_width}")"
    cur_pids+=("${pid}")
    cur_etime["${pid}"]="${etime}"
    cur_short["${pid}"]="${short}"
    # Approximate process start time (for stale-file detection later).
    cur_start["${pid}"]=$(( now_epoch - _etimes ))
    # Resolve the absolute config directory while the process is still alive
    # (so we can locate its result file after it terminates).
    reldir=""
    [[ "${args}" =~ ([^[:space:]]*exp_config[^/[:space:]]*) ]] && reldir="${BASH_REMATCH[1]}"
    dir=""
    if [[ -n "${reldir}" ]]; then
      if [[ "${reldir}" == /* ]]; then
        dir="${reldir}"
      else
        cwd="$(readlink /proc/${pid}/cwd 2>/dev/null || true)"
        [[ -n "${cwd}" ]] && dir="${cwd}/${reldir}" || dir="${reldir}"
      fi
    fi
    cur_dir["${pid}"]="${dir}"
    # If this PID was being kept as terminated (PID reused), un-keep it.
    [[ -n "${term_etime[${pid}]:-}" ]] && drop_terminated "${pid}"
  done

  # Detect terminations: any PID present last frame but not now has finished.
  if (( keep > 0 )); then
    for pid in "${!prev_etime[@]}"; do
      if [[ -z "${cur_etime[${pid}]:-}" ]]; then
        # Newly terminated (and not already recorded).
        if [[ -z "${term_etime[${pid}]:-}" ]]; then
          term_order+=("${pid}")
          term_etime["${pid}"]="${prev_etime[${pid}]}"
          term_short["${pid}"]="${prev_short[${pid}]}"
          term_start["${pid}"]="${prev_start[${pid}]:-0}"
          term_dir["${pid}"]="${prev_dir[${pid}]:-}"
          term_status["${pid}"]=""
        fi
      fi
    done
    # Keep only the most recently terminated N instances (drop oldest).
    while (( ${#term_order[@]} > keep )); do
      drop_terminated "${term_order[0]}"
    done
    # Try to resolve the result (CEX vs blind) of any still-pending instance.
    for pid in "${term_order[@]}"; do
      if [[ -z "${term_status[${pid}]}" ]]; then
        st="$(resolve_result "${term_dir[${pid}]}" "${term_start[${pid}]:-0}")"
        if [[ -n "${st}" ]]; then
          term_status["${pid}"]="${st%% *}"          # "cex" or "blind"
          [[ "${st}" == cex\ * ]] && term_len["${pid}"]="${st#* }"  # CEX state count
        fi
      fi
    done
  fi

  # Build the frame in a buffer, then print it all at once to avoid flicker.
  buffer=""
  buffer+=$'\033[H\033[2J'  # move cursor home + clear screen
  buffer+=$(printf '\033[1mProcess monitor\033[0m  pattern="%s"  interval=%ss  %s\n' \
    "${pattern}" "${interval}" "${now}")
  buffer+=$'\n'
  buffer+=$(printf '\033[1m%-8s  %-14s  %-9s  %s\033[0m\n' "PID" "RUNNING" "RESULT" "CONFIG")
  buffer+=$'\n'

  if [[ "${count}" -eq 0 && "${#term_order[@]}" -eq 0 ]]; then
    buffer+=$(printf '  (no processes matching "%s")\n' "${pattern}")
    buffer+=$'\n'
  else
    # Live instances first.
    for row in "${rows[@]}"; do
      read -r pid etime _etimes args <<<"${row}"
      buffer+=$(printf '%-8s  %-14s  \033[2m%-9s\033[0m  %s\n' \
        "${pid}" "${etime}" "running" "${cur_short[${pid}]}")
      buffer+=$'\n'
    done
    # Then recently terminated instances, colored by result and marked "done".
    for pid in "${term_order[@]}"; do
      case "${term_status[${pid}]}" in
        cex)   color=$'\033[32m'; res="CEX (${term_len[${pid}]:-?})" ;;  # green: CEX + length
        blind) color=$'\033[33m'; res="blind" ;;  # yellow: no counterexample
        *)     color=$'\033[2m';  res="…"     ;;  # dim: result not yet known
      esac
      buffer+=$(printf '%s%-8s  %-14s  %-9s  %s (done)\033[0m\n' \
        "${color}" "${pid}" "${term_etime[${pid}]}" "${res}" "${term_short[${pid}]}")
      buffer+=$'\n'
    done
  fi

  buffer+=$'\n'
  buffer+=$(printf '\033[2m%d running, %d finished (kept). Press Ctrl-C to quit.\033[0m\n' \
    "${count}" "${#term_order[@]}")

  printf '%s' "${buffer}"

  # Carry the current live set forward as "previous" for the next iteration.
  unset prev_etime prev_short prev_start prev_dir
  declare -A prev_etime=() prev_short=() prev_start=() prev_dir=()
  for pid in "${cur_pids[@]}"; do
    prev_etime["${pid}"]="${cur_etime[${pid}]}"
    prev_short["${pid}"]="${cur_short[${pid}]}"
    prev_start["${pid}"]="${cur_start[${pid}]}"
    prev_dir["${pid}"]="${cur_dir[${pid}]}"
  done

  sleep "${interval}"
done
