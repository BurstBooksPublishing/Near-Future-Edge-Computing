#!/usr/bin/env bash
# Production-ready snippet: enable hw timestamping, start ptp4l, set mqprio.
set -euo pipefail

IF="eth0"                      # adjust to actual interface
PTP_CONF="/etc/linuxptp/ptp4l.conf"

# Enable hardware timestamping (requires NIC driver support)
ethtool -T "$IF" | grep -q "hardware clock" || {
  echo "Interface $IF lacks PHC support" >&2
  exit 1
}

# Start gPTP (ptp4l uses IEEE 802.1AS profile)
systemctl restart ptp4l.service   # assumes /etc/linuxptp configured for gPTP
systemctl restart phc2sys.service # synchronize system clock to PHC

# Map traffic priorities (PCP 0..7) to 3 traffic classes; map index order depends on device
tc qdisc replace dev "$IF" root mqprio num_tc 3 \
  map 0 0 0 1 1 1 2 2 2 \
  queues 1@0 1@1 1@2 hw 0
# Explanation:
# - num_tc 3: three traffic classes
# - 'map' assigns PCP values to classes
# - 'queues' binds one hardware queue per TC starting at q0,q1,q2

# Configure VLAN PCP on outgoing traffic for scheduled streams (example uses tc filter)
# Tag control traffic (match by DSCP) to PCP 6 for scheduled class
tc filter add dev "$IF" protocol ip parent ffff: prio 1 u32 \
  match ip tos 0xb8 0xff keep   \
  action skbedit priority 6
# Verify qdisc
tc -s qdisc show dev "$IF"