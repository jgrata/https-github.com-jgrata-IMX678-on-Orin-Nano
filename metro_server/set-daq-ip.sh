#!/usr/bin/env bash
# =============================================================================
# set-daq-ip.sh -- give the Jetson's wired ethernet a static IP for the DAQ link
# (direct cable or switch to the host PC). Point-to-point/link subnet only:
# it does NOT set a gateway and never becomes the default route, so your normal
# internet/Wi-Fi routing is untouched.
#
#   sudo ./set-daq-ip.sh 192.168.99.3              # /24, auto-detect the wired NIC
#   sudo ./set-daq-ip.sh 192.168.99.3 24 eth0      # explicit prefix + interface
#   sudo ./set-daq-ip.sh --dhcp                    # revert the wired NIC to DHCP
#
# After setting the IP, the web UI (once run.sh is up) is at  http://<ip>:8080
# =============================================================================
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then echo "run with sudo (needs nmcli con mod/up)"; exit 1; fi
command -v nmcli >/dev/null || { echo "nmcli not found (this helper targets NetworkManager / JetPack)"; exit 1; }

MODE="static"
if [ "${1:-}" = "--dhcp" ]; then MODE="dhcp"; shift; fi
IP="${1:-}"; PREFIX="${2:-24}"; DEV="${3:-}"
if [ "$MODE" = "static" ] && [ -z "$IP" ]; then
    echo "usage: sudo $0 <ip> [prefix=24] [interface]   |   sudo $0 --dhcp [interface]"; exit 1
fi

# --- pick the wired ethernet device (first 'ethernet' device if not given) ----
if [ -z "$DEV" ]; then
    DEV=$(nmcli -t -f DEVICE,TYPE dev status | awk -F: '$2=="ethernet"{print $1; exit}')
fi
[ -n "$DEV" ] || { echo "no wired ethernet device found; pass one explicitly"; exit 1; }
echo "device: $DEV"

# --- find (or create) its NetworkManager connection --------------------------
CON=$(nmcli -t -f NAME,DEVICE con show | awk -F: -v d="$DEV" '$2==d{print $1; exit}')
if [ -z "$CON" ]; then
    CON="daq-$DEV"
    echo "no active connection on $DEV; creating '$CON'"
    nmcli con add type ethernet ifname "$DEV" con-name "$CON" >/dev/null
fi
echo "connection: $CON"

if [ "$MODE" = "dhcp" ]; then
    nmcli con mod "$CON" ipv4.method auto ipv4.addresses "" ipv4.gateway "" ipv4.never-default no
    nmcli con up "$CON" >/dev/null; sleep 1
    echo "== reverted $DEV to DHCP =="
else
    # static IP, no gateway, never the default route (leave internet routing alone)
    nmcli con mod "$CON" \
        ipv4.addresses "$IP/$PREFIX" \
        ipv4.method manual \
        ipv4.gateway "" \
        ipv4.never-default yes \
        ipv6.method ignore
    nmcli con up "$CON" >/dev/null; sleep 1
    echo "== applied static $IP/$PREFIX on $DEV =="
fi
ip -4 -o addr show "$DEV" | awk '{print "  "$2"  "$4}'
[ "$MODE" = "static" ] && echo "web UI (once ./run.sh is up):  http://$IP:8080"
