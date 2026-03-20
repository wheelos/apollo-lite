## Quick Start (Client/Car)

### Install

```bash
cd modules/tools/whl-remote/ && sudo bash install.sh
```

The installer will interactively prompt for:

- Server IP
- Car number (numeric)
- auth.token

### Run / Management

```bash
# cd modules/tools/whl-remote

# Start
bash manage.sh start

# Check status
bash manage.sh status

# Stop
bash manage.sh stop

# Enable autostart on boot (systemd)
bash manage.sh autostart on

# Disable autostart on boot
bash manage.sh autostart off

# Check autostart status
bash manage.sh autostart status
```

If you want `frpc` to start automatically on boot, run `bash manage.sh autostart on` after installation.
