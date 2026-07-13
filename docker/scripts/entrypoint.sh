#!/bin/bash
set -e

# 1. Dynamically create users identical to those on the host machine
USER_NAME=${USER_NAME:-apollo}
USER_ID=${USER_ID:-1000}
GROUP_ID=${GROUP_ID:-1000}

if ! getent group "$USER_NAME" >/dev/null; then
    groupadd -g "$GROUP_ID" "$USER_NAME" 2>/dev/null || groupmod -g "$GROUP_ID" $(getent group "$GROUP_ID" | cut -d: -f1)
fi

if ! id -u "$USER_NAME" >/dev/null 2>&1; then
    useradd -u "$USER_ID" -g "$GROUP_ID" -m -s /bin/bash "$USER_NAME"
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
fi

# user add to video group on aarch64
if [[ -e "/dev/nvmap" ]]; then
    NVMAP_GID="$(stat -c '%g' /dev/nvmap 2>/dev/null || true)"
    if [[ -n "${NVMAP_GID}" ]]; then
        NVMAP_GROUP_NAME="$(getent group "${NVMAP_GID}" | cut -d: -f1 || true)"
        if [[ -z "${NVMAP_GROUP_NAME}" ]]; then
            # Prefer the conventional name when available.
            if ! getent group video >/dev/null; then
                groupadd -g "${NVMAP_GID}" video 2>/dev/null || true
            fi
            NVMAP_GROUP_NAME="$(getent group "${NVMAP_GID}" | cut -d: -f1 || true)"
            if [[ -z "${NVMAP_GROUP_NAME}" ]]; then
                groupadd -g "${NVMAP_GID}" nvmap 2>/dev/null || true
                NVMAP_GROUP_NAME="$(getent group "${NVMAP_GID}" | cut -d: -f1 || true)"
            fi
        fi
        if [[ -n "${NVMAP_GROUP_NAME}" ]]; then
            usermod -aG "${NVMAP_GROUP_NAME}" "${USER_NAME}" 2>/dev/null || true
        fi
    fi
fi

# 2. Correct critical directory permissions
chown "$USER_NAME":"$USER_NAME" /apollo
if [[ -n "${BAZEL_CACHE_DIR:-}" && -d "${BAZEL_CACHE_DIR}" ]]; then
  chown -R "$USER_NAME":"$USER_NAME" "${BAZEL_CACHE_DIR}"
fi

# 3. Load Apollo environment
# setup rc files
USER_HOME="$(getent passwd ${USER_NAME} | cut -d: -f6)"
cp /etc/skel/.{profile,bashrc,bash_logout} "${USER_HOME}/"
RCFILES_DIR="/opt/apollo/rcfiles"
if [[ -d "${RCFILES_DIR}" ]]; then
  for entry in ${RCFILES_DIR}/*; do
    rc=$(basename "${entry}")
    if [[ "${rc}" = user.* ]]; then
      cp -rf "${entry}" "${USER_HOME}/${rc##user}"
    fi
  done
fi

find "${USER_HOME}" -maxdepth 1 -name ".*" ! -name "." ! -name " .. " -exec chown -R "${USER_ID}:${GROUP_ID}" {} +
chown "${USER_ID}:${GROUP_ID}" "${USER_HOME}"

if [ -f "/apollo/cyber/setup.bash" ]; then
    source /apollo/cyber/setup.bash
fi

# 4. Business logic branch
if [[ "${AUTO_BOOTSTRAP}" == "true" ]]; then
    echo "[Entrypoint] Auto-starting Dreamview..."
    runuser -u ${USER_NAME} -- bash -l -c "cd /apollo && ./scripts/bootstrap.sh start > /apollo/data/log/bootstrap.log 2>&1" || true
fi

# 5. Keep running the container
echo ">>> Container is ready for user: $USER_NAME"
exec "$@"
