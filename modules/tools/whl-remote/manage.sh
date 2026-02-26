#!/bin/bash
# Usage: manage.sh [start|stop|status]

set -u

INSTALL_DIR="${INSTALL_DIR:-/opt/frp/car}"
if [ -d "$INSTALL_DIR" ]; then
    WORK_DIR="$INSTALL_DIR"
else
    WORK_DIR="$PWD"
fi

BIN="$WORK_DIR/frpc"
CONF="$WORK_DIR/frpc.toml"
LOG="$WORK_DIR/frpc.log"
PROC="frpc"
PID_FILE="$WORK_DIR/${PROC}.pid"
ACTION="${1:-}"

is_running() {
    if [ -f "$PID_FILE" ]; then
        pid="$(cat "$PID_FILE")"
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            # ensure the process is frpc (not PID reuse)
            if command -v ps >/dev/null 2>&1; then
                cmd="$(ps -p "$pid" -o comm= 2>/dev/null)"
                if [ "$cmd" = "frpc" ]; then
                    return 0
                fi
            else
                # fallback: assume running if kill -0 succeeded
                return 0
            fi
        fi
    fi
    return 1
}

case "$ACTION" in
    start)
        if [ ! -x "$BIN" ]; then
            echo "Error: executable not found: $BIN"
            exit 1
        fi
        if [ ! -f "$CONF" ]; then
            echo "Error: configuration file not found: $CONF"
            exit 1
        fi
        if is_running; then
            echo "$PROC is already running (PID: $(cat "$PID_FILE"))"
            exit 0
        fi

        nohup "$BIN" -c "$CONF" > "$LOG" 2>&1 &
        echo $! > "$PID_FILE"
        sleep 1
        if is_running; then
            echo "$PROC started in background (PID: $(cat "$PID_FILE")), view logs at $LOG"
        else
            echo "Start failed, please check logs: $LOG"
            rm -f "$PID_FILE"
            exit 1
        fi
        ;;
    stop)
        if is_running; then
            pid="$(cat "$PID_FILE")"
            kill "$pid" 2>/dev/null || true
            sleep 1
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
                sleep 1
            fi
            if kill -0 "$pid" 2>/dev/null; then
                # still alive after SIGKILL
                echo "Failed to stop $PROC (pid $pid still running)" >&2
                exit 1
            fi
            rm -f "$PID_FILE"
            echo "$PROC stopped"
        else
            echo "$PROC is not running"
            rm -f "$PID_FILE"
        fi
        ;;
    status)
        echo "--- Process Info ---"
        if is_running; then
            pid="$(cat "$PID_FILE")"
            echo "Status: [Running] PID=$pid"
        else
            echo "Status: [Not running]"
        fi

        echo "--- Network Connections ---"
        if is_running; then
            pid="$(cat "$PID_FILE")"
            if command -v lsof >/dev/null 2>&1; then
                lsof -nP -a -p "$pid" -iTCP -sTCP:LISTEN || echo "No listening ports"
            elif command -v netstat >/dev/null 2>&1; then
                netstat -an | grep LISTEN || echo "No listening ports"
            else
                echo "lsof/netstat not found, cannot show listening ports"
            fi
        else
            echo "No listening ports"
        fi

        echo "--- Recent Logs ---"
        if [ -f "$LOG" ]; then
            tail -n 5 "$LOG"
        else
            echo "Log file does not exist: $LOG"
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac
