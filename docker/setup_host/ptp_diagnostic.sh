#!/bin/bash

# ==============================================================================
# Apollo-Lite PTP Sync Diagnostic Tool
# Purpose: Deep analysis of PTP Master health and synchronization chain.
# Usage: sudo ./ptp_diagnostic.sh [interface]
# ==============================================================================

IFACE=${1:-"eth0"}
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[34m'
NC='\033[0m'

echo -e "${BOLD}${BLUE}=== Apollo-Lite PTP Health Dashboard ===${NC}"

# --- Step 1: Network Interface & Link Status ---
echo -e "\n${BOLD}>>> Step 1: Interface & Link Status [ $IFACE ]${NC}"
if ! ip link show "$IFACE" > /dev/null 2>&1; then
    echo -e "[${RED}FAIL${NC}] Interface $IFACE does not exist."
    exit 1
fi

LINK_STATE=$(cat /sys/class/net/"$IFACE"/operstate)
if [[ "$LINK_STATE" == "up" ]]; then
    echo -e "[${GREEN}OK${NC}] Interface link is UP."
else
    echo -e "[${RED}ERROR${NC}] Link is $LINK_STATE. Please check cable/connection."
fi

# --- Step 2: Hardware Timestamping Capability ---
echo -e "\n${BOLD}>>> Step 2: PHC Hardware Capability${NC}"
ETHTOOL_OUT=$(ethtool -T "$IFACE" 2>&1)
if [[ $ETHTOOL_OUT == *"SOF_TIMESTAMPING_TX_HARDWARE"* ]]; then
    echo -e "[${GREEN}OK${NC}] Hardware Timestamping supported."
    PTP_DEV=$(echo "$ETHTOOL_OUT" | grep "PTP Hardware Clock" | awk '{print $NF}')
    echo -e "[${BLUE}INFO${NC}] Using PTP Device: /dev/ptp$PTP_DEV"
else
    echo -e "[${RED}ERROR${NC}] $IFACE lacks Hardware Timestamping. PTP precision will be poor (Software only)."
fi

# --- Step 3: Upstream Time Source (Chrony) ---
echo -e "\n${BOLD}>>> Step 3: Upstream Sync (Master Source)${NC}"
if command -v chronyc &> /dev/null; then
    CHRONY_STATE=$(chronyc tracking)
    if echo "$CHRONY_STATE" | grep -q "Reference ID"; then
        STRATUM=$(echo "$CHRONY_STATE" | grep "Stratum" | awk '{print $3}')
        echo -e "[${GREEN}OK${NC}] Chrony is synced (Stratum: $STRATUM)."
    else
        echo -e "[${YELLOW}WARNING${NC}] Chrony is active but NOT synced to external NTP. PTP time may drift."
    fi
else
    echo -e "[${RED}ERROR${NC}] Chrony not found. System clock reference is unreliable."
fi

# --- Step 4: Service Health & Process Check ---
echo -e "\n${BOLD}>>> Step 4: Systemd Service Status${NC}"
for SERVICE in ptp4l phc2sys; do
    if systemctl is-active --quiet "$SERVICE"; then
        CONF_FILE=$(ps aux | grep "$SERVICE" | grep -v grep | sed 's/.*-f //;s/ .*//')
        echo -e "[${GREEN}OK${NC}] $SERVICE is running. (Config: ${CONF_FILE:-"Default CLI"})"
    else
        echo -e "[${RED}FAIL${NC}] $SERVICE is NOT running!"
        echo -e "${YELLOW}Hint: Run 'journalctl -u $SERVICE -e' for logs.${NC}"
    fi
done

# --- Step 5: Real-time Sync Analysis ---
echo -e "\n${BOLD}>>> Step 5: Synchronization Precision Analysis${NC}"
PHC_LOG=$(journalctl -u phc2sys -n 20 --no-pager)

if [[ $PHC_LOG == *"s2"* ]]; then
    LAST_LINE=$(echo "$PHC_LOG" | grep "offset" | tail -n 1)
    OFFSET=$(echo "$LAST_LINE" | grep -oP 'offset\s+\K[-0-9]+')
    FREQ=$(echo "$LAST_LINE" | grep -oP 'freq\s+\K[-0-9]+')

    : ${OFFSET:=99999}

    echo -e "[${GREEN}LOCKED${NC}] phc2sys status: s2 (Phase Locked)."
    echo -e "  - Current Offset: ${BOLD}${OFFSET}${NC} ns"
    echo -e "  - Freq Adjust  : ${FREQ} ppb"

    if [ ${OFFSET#-} -lt 500 ]; then
        echo -e "${GREEN}>>> SYSTEM HEALTHY: High-precision sync established. <<<${NC}"
    else
        echo -e "${YELLOW}>>> WARNING: Offset > 500ns. Check system load or interrupt binding. <<<${NC}"
    fi
else
    echo -e "[${RED}SYNCING${NC}] phc2sys has not reached 's2' state yet."
    echo -e "${YELLOW}Action: Check if ptp4l has found any Slaves or if domainNumber matches.${NC}"
fi

echo -e "\n${BLUE}=========================================${NC}"
