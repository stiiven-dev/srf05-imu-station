#!/usr/bin/env bash
# capture-filter-log.sh — records defmt-serial output to a file for later
# plotting (see tools/plot_filters.py), while tolerating the board
# resetting/re-enumerating mid-capture (same reconnect approach as
# watch-defmt.sh). Ctrl+C to stop; the file is flushed as it goes, so a
# hard exit still leaves you a usable log.
#
# Usage:
#   ./capture-filter-log.sh [elf_path] [output_file]
#
# Defaults:
#   elf_path    -> target/thumbv6m-none-eabi/debug/srf05-imu-station
#   output_file -> filter_log_<timestamp>.txt

set -u

ELF="${1:-target/thumbv6m-none-eabi/debug/srf05-imu-station}"
OUT="${2:-filter_log_$(date +%Y%m%d_%H%M%S).txt}"
CANDIDATES=(/dev/ttyACM0 /dev/ttyACM1)
BAUD=115200

if [[ ! -f "$ELF" ]]; then
    echo "warning: ELF not found at '$ELF' — pass the correct path as an argument" >&2
fi

find_port() {
    for dev in "${CANDIDATES[@]}"; do
        if [[ -e "$dev" ]]; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

trap 'echo; echo "Stopped capturing. Saved to $OUT"; exit 0' INT

echo "watching for ${CANDIDATES[*]} -> $OUT (ctrl-c to stop)"
echo "---"

while true; do
    port=$(find_port) || { sleep 0.2; continue; }
    echo "[connected]    $port" | tee -a "$OUT"
    socat "${port},rawer,b${BAUD}" STDOUT | defmt-print -e "$ELF" | tee -a "$OUT"
    echo "[disconnected] $port — rescanning..." | tee -a "$OUT"
    sleep 0.3
done