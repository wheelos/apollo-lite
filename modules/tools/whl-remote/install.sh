#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

INSTALL_DIR="${INSTALL_DIR:-/opt/frp/car}"
FRP_VER="${FRP_VER:-0.54.0}"

SSH_REMOTE_BASE="${SSH_REMOTE_BASE:-60000}"

LOCAL_SSH_IP="${LOCAL_SSH_IP:-127.0.0.1}"
LOCAL_SSH_PORT="${LOCAL_SSH_PORT:-22}"

LOCAL_APP_IP="${LOCAL_APP_IP:-127.0.0.1}"
LOCAL_APP_PORT="${LOCAL_APP_PORT:-8888}"

if [ "$(uname -s)" != "Linux" ]; then
    echo "Error: this installer downloads the Linux build of frp; please run on a Linux host."
    exit 1
fi

if ! mkdir -p "$INSTALL_DIR"; then
    echo "Error: failed to create install directory: $INSTALL_DIR"
    echo "Tip: run with sudo or set INSTALL_DIR to a user-writable location."
    exit 1
fi
if ! cd "$INSTALL_DIR"; then
    echo "Error: failed to enter install directory: $INSTALL_DIR"
    exit 1
fi

TARGET_BIN="frpc"
if [ -x "$TARGET_BIN" ]; then
    echo "Detected existing $TARGET_BIN, skipping download."
else
    arch="$(uname -m)"
    case "$arch" in
        x86_64)
            suffix="linux_amd64"
            ;;
        aarch64|arm64)
            suffix="linux_arm64"
            ;;
        *)
            echo "Error: unsupported architecture: $arch"
            exit 1
            ;;
    esac

    PRIMARY_URL="${FRP_PRIMARY_URL:-https://dl.wheelos.cn/assets/third-party/frp_${FRP_VER}_${suffix}.tar.gz}"
    FALLBACK_URL="${FRP_FALLBACK_URL:-https://github.com/fatedier/frp/releases/download/v${FRP_VER}/frp_${FRP_VER}_${suffix}.tar.gz}"

    tmpdir="$(mktemp -d)"
    trap 'rm -rf "$tmpdir"' EXIT

    download_ok=0
    for url in "$PRIMARY_URL" "$FALLBACK_URL"; do
        [ -n "$url" ] || continue
        echo "Attempting download: $url"
        if command -v curl >/dev/null 2>&1; then
            if curl -fL "$url" -o "$tmpdir/frp.tar.gz"; then
                download_ok=1
                break
            fi
        elif command -v wget >/dev/null 2>&1; then
            if wget "$url" -O "$tmpdir/frp.tar.gz"; then
                download_ok=1
                break
            fi
        else
            echo "Error: no download tool found; please install curl or wget"
            exit 1
        fi
        rm -f "$tmpdir/frp.tar.gz"
    done

    if [ "$download_ok" -ne 1 ]; then
        echo "Error: both download sources failed."
        echo "Tried:"
        echo "  1) $PRIMARY_URL"
        echo "  2) $FALLBACK_URL"
        exit 1
    fi

    if ! tar -zxf "$tmpdir/frp.tar.gz" -C "$tmpdir"; then
        echo "Error: failed to extract frp package"
        exit 1
    fi

    if ! cp "$tmpdir/frp_${FRP_VER}_${suffix}/frpc" .; then
        echo "Error: failed to copy frpc from archive"
        exit 1
    fi

    chmod +x frpc
    echo "frpc installed to: $INSTALL_DIR/frpc"
fi

read -r -p "Enter server IP: " S_IP
read -r -p "Enter this car's ID (e.g. 1): " C_ID
read -r -s -p "Enter auth.token: " AUTH_TOKEN
echo

if [ -z "$S_IP" ]; then
    echo "Error: server IP cannot be empty"
    exit 1
fi

if ! [[ "$C_ID" =~ ^[0-9]+$ ]]; then
    echo "Error: car ID must be numeric"
    exit 1
fi

if [ -z "$AUTH_TOKEN" ]; then
    echo "Error: auth.token cannot be empty"
    exit 1
fi

REMOTE_PORT_SSH=$((SSH_REMOTE_BASE + C_ID))
REMOTE_PORT_APP=$((SSH_REMOTE_BASE + C_ID + 1))

validate_port() {
    local port="$1"
    local name="$2"
    if [ "$port" -lt 1 ] || [ "$port" -gt 65535 ]; then
        echo "Error: computed $name port $port is outside the valid TCP port range (1-65535)."
        exit 1
    fi
}

validate_port "$LOCAL_SSH_PORT" "local ssh"
validate_port "$LOCAL_APP_PORT" "local app"
validate_port "$REMOTE_PORT_SSH" "remote ssh"
validate_port "$REMOTE_PORT_APP" "remote app"

if [ "$REMOTE_PORT_SSH" -eq "$REMOTE_PORT_APP" ]; then
    echo "Error: remote SSH port and remote APP port conflict: $REMOTE_PORT_SSH"
    echo "Please adjust SSH_REMOTE_BASE or SSH_REMOTE_BASE."
    exit 1
fi

cat <<EOF > frpc.toml
serverAddr = "$S_IP"
serverPort = 7000
auth.token = "$AUTH_TOKEN"

[[proxies]]
name = "car_${C_ID}_ssh"
type = "tcp"
localIP = "$LOCAL_SSH_IP"
localPort = $LOCAL_SSH_PORT
remotePort = $REMOTE_PORT_SSH

[[proxies]]
name = "car_${C_ID}_app_8888"
type = "tcp"
localIP = "$LOCAL_APP_IP"
localPort = $LOCAL_APP_PORT
remotePort = $REMOTE_PORT_APP
EOF

chmod 600 frpc.toml

echo
echo "Car installer finished."
echo "Install directory : $INSTALL_DIR"
echo "Config file       : $INSTALL_DIR/frpc.toml"
echo "SSH mapping       : ${LOCAL_SSH_IP}:${LOCAL_SSH_PORT} -> ${S_IP}:${REMOTE_PORT_SSH}"
echo "APP mapping       : ${LOCAL_APP_IP}:${LOCAL_APP_PORT} -> ${S_IP}:${REMOTE_PORT_APP}"
echo
echo "Start command:"
echo "  cd $INSTALL_DIR && ./frpc -c frpc.toml"
echo "Autostart command:"
echo "  bash $SCRIPT_DIR/manage.sh autostart on"
