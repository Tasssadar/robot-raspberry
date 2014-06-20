#!/bin/sh

LOG_PATH="/home/pi/logs"

print_help() {
    echo "Usage: $0 PATH_TO_BINARY"
    exit 0
}

BIN=""
for arg in "$@"; do
    case $arg in
        --help|-h)
            print_help $0
            ;;
        *)
            BIN="$arg"
            ;;
    esac
done

([ -n "$BIN" ]) || print_help $0

mkdir -p "$LOG_PATH"

while [ 1 ]; do
    log_file="${LOG_PATH}/$(basename "$BIN")-$(date +%y%m%d-%H:%M:%S).txt"
    echo "Starting program $BIN, logging to $log_file"
    "$BIN" > "$log_file" 2>&1
    exit_code="$?"
    echo "Program $BIN exited with code $exit_code, restarting"
    printf "\nProgram $BIN exited with code $exit_code\n" >> "$log_file"
    sleep 1
done