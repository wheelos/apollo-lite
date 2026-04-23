#!/bin/bash
# Usage: manage.sh [start|stop|status|autostart <on|off|status>]

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
SUB_ACTION="${2:-}"
SERVICE_NAME="frpc.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"

run_root() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        sudo "$@"
    fi
}

ensure_systemd_available() {
    if ! command -v systemctl >/dev/null 2>&1; then
        echo "Error: systemctl not found; autostart management requires systemd." >&2
        exit 1
    fi
}

install_service_file() {
    ensure_systemd_available

    if [ ! -x "$BIN" ]; then
        echo "Error: executable not found: $BIN" >&2
        exit 1
    fi
    if [ ! -f "$CONF" ]; then
        echo "Error: configuration file not found: $CONF" >&2
        exit 1
    fi

    tmp_service_file="$(mktemp)"
    cat > "$tmp_service_file" <<EOF
[Unit]
Description=FRP Client Service (Car)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=$WORK_DIR
ExecStart=$BIN -c $CONF
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
EOF

    run_root cp -f "$tmp_service_file" "$SERVICE_FILE"
    rm -f "$tmp_service_file"
    run_root systemctl daemon-reload
}

set_autostart_on() {
    install_service_file
    run_root systemctl enable --now "$SERVICE_NAME"
    echo "Autostart enabled: $SERVICE_NAME"
}

set_autostart_off() {
    ensure_systemd_available
    run_root systemctl disable --now "$SERVICE_NAME" >/dev/null 2>&1 || true
    echo "Autostart disabled: $SERVICE_NAME"
}

show_autostart_status() {
    ensure_systemd_available

    enabled="no"
    active="no"

    if systemctl is-enabled "$SERVICE_NAME" >/dev/null 2>&1; then
        enabled="yes"
    fi
    if systemctl is-active "$SERVICE_NAME" >/dev/null 2>&1; then
        active="yes"
    fi

    echo "Autostart (enabled): $enabled"
    echo "Service running      : $active"
}

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
    autostart)
        case "$SUB_ACTION" in
            on)
                set_autostart_on
                ;;
            off)
                set_autostart_off
                ;;
            status)
                show_autostart_status
                ;;
            *)
                echo "Usage: $0 autostart {on|off|status}"
                exit 1
                ;;
        esac
        ;;
    *)
        echo "Usage: $0 {start|stop|status|autostart <on|off|status>}"
        exit 1
        ;;
esac
