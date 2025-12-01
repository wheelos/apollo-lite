#!/bin/bash
set -e

# 1. Dynamically create users identical to those on the host machine
USER_NAME=${USER_NAME:-apollo_user}
USER_ID=${USER_ID:-1000}
GROUP_ID=${GROUP_ID:-1000}

if ! getent group "$USER_NAME" >/dev/null; then
    groupadd -g "$GROUP_ID" "$USER_NAME" 2>/dev/null || groupmod -g "$GROUP_ID" $(getent group "$GROUP_ID" | cut -d: -f1)
fi

if ! id -u "$USER_NAME" >/dev/null 2>&1; then
    useradd -u "$USER_ID" -g "$GROUP_ID" -m -s /bin/bash "$USER_NAME"
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
fi

# 2. Correct critical directory permissions
chown "$USER_NAME":"$USER_NAME" /apollo
[ -d "/var/cache/bazel" ] && chown -R "$USER_NAME":"$USER_NAME" /var/cache/bazel

# 3. Load Apollo environment
if [ -f "/apollo/cyber/setup.bash" ]; then
    source /apollo/cyber/setup.bash
fi

# 4. Business logic branch
# If AUTO_BOOTSTRAP=true is set (usually for one-click startup in Dev mode)
if [[ "${AUTO_BOOTSTRAP}" == "true" ]]; then
    echo "[Entrypoint] Auto-starting Dreamview..."
    nohup bash /apollo/scripts/bootstrap.sh start > /apollo/data/log/bootstrap.log 2>&1 &
fi

# 5. Keep running the container
echo ">>> Container is ready for user: $USER_NAME"
exec tail -f /dev/null
