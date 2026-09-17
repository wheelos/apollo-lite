#!/usr/bin/env bash

ISSUES_LINK="https://github.com/wheelos/apollo-lite"
SUPPORT_EMAIL="support@wheelos.cn"

ASCII_WHEELOS="$(cat <<'EOF'
           __              __
 _      __/ /_  ___  ___  / /___  _____
| | /| / / __ \/ _ \/ _ \/ / __ \/ ___/
| |/ |/ / / / /  __/  __/ / /_/ (__  )
|__/|__/_/ /_/\___/\___/_/\____/____/
EOF
)"

function build_welcome_text() {
    printf '%s\n\nGitHub: %s\nEmail : %s\n' \
        "${ASCII_WHEELOS}" "${ISSUES_LINK}" "${SUPPORT_EMAIL}"
}

function show_welcome() {
    build_welcome_text
}
