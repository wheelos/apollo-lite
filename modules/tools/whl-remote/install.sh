#!/bin/bash

set -u

INSTALL_DIR="${INSTALL_DIR:-/opt/frp/car}"
FRP_VER="${FRP_VER:-0.54.0}"

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

    download_ok=0
    for url in "$PRIMARY_URL" "$FALLBACK_URL"; do
        [ -n "$url" ] || continue
        echo "Attempting download: $url"
        if command -v curl >/dev/null 2>&1; then
            if curl -fL "$url" -o frp.tar.gz; then
                download_ok=1
                break
            fi
        elif command -v wget >/dev/null 2>&1; then
            if wget "$url" -O frp.tar.gz; then
                download_ok=1
                break
            fi
        else
            echo "Error: no download tool found; please install curl or wget"
            exit 1
        fi
        rm -f frp.tar.gz
    done

    if [ "$download_ok" -ne 1 ]; then
        echo "Error: both download sources failed."
        echo "Tried:"
        echo "  1) $PRIMARY_URL"
        echo "  2) $FALLBACK_URL"
        exit 1
    fi

    if ! tar -zxf frp.tar.gz; then
        echo "Error: failed to extract frp.tar.gz"
        exit 1
    fi
    if ! cp "frp_${FRP_VER}_${suffix}/frpc" .; then
        echo "Error: failed to copy frpc from archive"
        exit 1
    fi
    chmod +x frpc
    rm -rf "frp.tar.gz" "frp_${FRP_VER}_${suffix}"
fi

read -p "Enter server IP: " S_IP
read -p "Enter this car's ID (e.g. 1): " C_ID
read -s -p "Enter auth.token: " AUTH_TOKEN
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

REMOTE_PORT=$((60000 + C_ID))
# Validate that the computed remote port is within the valid TCP port range.
if [ "$REMOTE_PORT" -lt 1 ] || [ "$REMOTE_PORT" -gt 65535 ]; then
    echo "Error: computed remote port $REMOTE_PORT is outside the valid TCP port range (1-65535)."
    echo "Please choose a car ID such that 60000 + C_ID is within 1-65535 (e.g., C_ID in the range 0..5535)."
    exit 1
fi

cat <<EOF > frpc.toml
serverAddr = "$S_IP"
serverPort = 7000
auth.token = "$AUTH_TOKEN"
[[proxies]]
name = "car_${C_ID}_ssh"
type = "tcp"
localIP = "127.0.0.1"
localPort = 22
remotePort = $REMOTE_PORT
bindAddr = "127.0.0.1"
EOF
chmod 600 frpc.toml
echo "Car installer finished. Directory: $INSTALL_DIR, assigned port: $REMOTE_PORT"
