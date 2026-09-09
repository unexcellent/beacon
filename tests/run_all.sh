#!/usr/bin/env bash
#
# run_all.sh — run the hardware-in-the-loop Python tests (tests/test_*.py) with
# docker-style live output: each test shows its name and a rolling window of its
# last few output lines while it runs, then collapses to a one-line PASS/FAIL/SKIP.
#
# Only the tests/test_*.py files are run (no Rust unit tests — those belong on a
# dev machine). The OTA tests need a firmware app image; it is taken from --image
# / BEACON_OTA_IMAGE / a staged /tmp/beacon_ota.bin, and tests that require one
# skip themselves if none is available.
#
# Usage: tests/run_all.sh [--image PATH] [FILTER]
#   --image PATH   app image for the OTA tests
#   FILTER         only run test files whose path contains FILTER
#
set -uo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."  # repo root

image="${BEACON_OTA_IMAGE:-}"
filter=""
while [ $# -gt 0 ]; do
    case "$1" in
        --image) image="$2"; shift ;;
        -h|--help) sed -n '2,17p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) filter="$1" ;;
    esac
    shift
done
[ -z "$image" ] && [ -f /tmp/beacon_ota.bin ] && image=/tmp/beacon_ota.bin
[ -n "$image" ] && export BEACON_OTA_IMAGE="$image"

PY=.venv/bin/python
[ -x "$PY" ] || PY=python3

# collect test files (portable; no mapfile)
files=()
for f in tests/test_*.py; do
    [ -e "$f" ] || continue
    if [ -n "$filter" ]; then case "$f" in *"$filter"*) ;; *) continue ;; esac; fi
    files+=("$f")
done
[ ${#files[@]} -gt 0 ] || { echo "no test files match"; exit 1; }

WINDOW=5
pass=0; fail=0; skip=0
is_tty=0; [ -t 1 ] && is_tty=1
cols=$( { tput cols; } 2>/dev/null || echo 100 )
[ -n "$image" ] && echo "OTA image: $image"

# run one test file, updating pass/fail/skip. Uses a rolling window on a TTY.
run_one() {
    local file="$1" name start rc dur icon color label
    name="$(basename "$file" .py)"
    start=$SECONDS
    local rcfile; rcfile="$(mktemp)"

    if [ "$is_tty" -eq 1 ]; then
        printf '\033[1m⏳ %s\033[0m\n' "$name"
        local i; for ((i=0; i<WINDOW; i++)); do printf '\n'; done
        { "$PY" -u "$file" 2>&1; echo $? >"$rcfile"; } | {
            local -a ring=()
            while IFS= read -r line; do
                ring+=("$line")
                while [ ${#ring[@]} -gt "$WINDOW" ]; do ring=("${ring[@]:1}"); done
                printf '\033[%dA' "$WINDOW"
                local j content
                for ((j=0; j<WINDOW; j++)); do
                    content="${ring[j]:-}"
                    content="${content:0:$((cols-4))}"
                    printf '\033[2K\033[90m   %s\033[0m\n' "$content"
                done
            done
        }
        rc="$(cat "$rcfile")"
        printf '\033[%dA' "$((WINDOW+1))"   # up to the header line
    else
        printf '== %s ==\n' "$name"
        { "$PY" -u "$file" 2>&1; echo $? >"$rcfile"; } | sed 's/^/  /'
        rc="$(cat "$rcfile")"
    fi
    rm -f "$rcfile"
    dur=$((SECONDS-start))

    case "$rc" in
        0)  icon='✔'; color=32; label=PASS; pass=$((pass+1)) ;;
        77) icon='○'; color=33; label=SKIP; skip=$((skip+1)) ;;
        *)  icon='✖'; color=31; label=FAIL; fail=$((fail+1)) ;;
    esac
    if [ "$is_tty" -eq 1 ]; then
        printf '\033[2K\033[%dm%s %s\033[0m  (%ds)\n' "$color" "$icon" "$name" "$dur"
        printf '\033[J'   # erase the transient window below
    else
        printf '%s %s (%ds)\n' "$label" "$name" "$dur"
    fi
}

for f in "${files[@]}"; do
    run_one "$f"
done

echo
printf 'Summary: \033[32m%d passed\033[0m, \033[31m%d failed\033[0m, \033[33m%d skipped\033[0m\n' \
    "$pass" "$fail" "$skip"
[ "$fail" -eq 0 ]
