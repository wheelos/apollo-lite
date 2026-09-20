#!/bin/bash
set -e

# Responsibility: perform privileged container bootstrap, prepare lightweight
# directory ownership, install automatic runtime loading, and drop to the
# configured non-root user. It does not build or clean Bazel state.

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
USER_HOME="$(getent passwd "${USER_NAME}" | cut -d: -f6)"

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
chown "${USER_ID}:${GROUP_ID}" /apollo

function ensure_user_owns_path() {
  local path="$1"
  if [[ -d "${path}" ]]; then
    # Cache paths are created and mounted by whl. Only fix the directory
    # itself; never recursively scan Bazel state during container startup.
    chown "${USER_ID}:${GROUP_ID}" "${path}"
  fi
}

# Keep the cache roots writable by the user that runs the build. Existing
# legacy root-owned files require a one-time maintenance action, not startup
# recursion.
ensure_user_owns_path "${BAZEL_CACHE_DIR:-}"
ensure_user_owns_path "/apollo/.cache/bazel/repo_cache"
ensure_user_owns_path "/apollo/.cache/bazel/disk_cache"
ensure_user_owns_path "${USER_HOME}/.cache/bazel"

# 3. Configure automatic runtime loading for the non-root login shell.
# The entrypoint does not source the runtime itself: all runtime discovery and
# environment changes must run as the user that owns the Bazel outputs.
cat > /etc/profile.d/apollo-runtime.sh <<'EOF'
if [[ -f /apollo/setup.bash ]]; then
  source /apollo/setup.bash
fi
EOF
chmod 0644 /etc/profile.d/apollo-runtime.sh

# setup rc files
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

find "${USER_HOME}" -mindepth 1 -maxdepth 1 -name ".*" \
    ! -name "." ! -name ".." \
    -exec chown --no-dereference "${USER_ID}:${GROUP_ID}" {} +
chown "${USER_ID}:${GROUP_ID}" "${USER_HOME}"

# 4. Business logic branch
if [[ "${AUTO_BOOTSTRAP}" == "true" ]]; then
    echo "[Entrypoint] Auto-starting Dreamview..."
    runuser -u "${USER_NAME}" -- bash -l -c "cd /apollo && ./scripts/bootstrap.sh start > /apollo/data/log/bootstrap.log 2>&1" || true
fi

# 5. Keep running the container
echo ">>> Container is ready for user: $USER_NAME"
if [[ "$(id -u)" == "0" && "${USER_NAME}" != "root" && \
      ( "${1:-}" != "runuser" || "${2:-}" != "-u" || \
        "${3:-}" != "${USER_NAME}" || "${4:-}" != "--" ) ]]; then
    exec runuser -u "${USER_NAME}" -- "$@"
fi
exec "$@"
