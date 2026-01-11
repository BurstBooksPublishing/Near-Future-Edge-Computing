#!/usr/bin/env bash
set -euo pipefail

# Configurable parameters
NET_IF="eth0"                 # physical NIC connected to DU / fronthaul
VF_COUNT=4                    # number of VFs to create
HUGEPAGES_MB=1024             # total hugepages in MB
UPF_IMAGE="free5gc/upf:latest" # container image for UPF
CPUSET="2-5"                  # CPUs reserved for DPDK/UPF
CONTAINER_NAME="edge-upf"

# Enable SR-IOV VFs
echo "Configuring SR-IOV on ${NET_IF}"
PCI=$(ethtool -i ${NET_IF} | awk '/bus-info/ {print $2}')
if [ -z "$PCI" ]; then
  echo "ERROR: cannot determine PCI bus for ${NET_IF}" >&2
  exit 1
fi
echo ${VF_COUNT} > /sys/bus/pci/devices/${PCI}/sriov_numvfs

# Reserve hugepages
echo "Reserving ${HUGEPAGES_MB}MB hugepages"
sysctl -w vm.nr_hugepages=$((HUGEPAGES_MB/2)) >/dev/null

# Bind VFs to vfio-pci using dpdk-devbind script if available
DPDK_BIND="/usr/local/bin/dpdk-devbind.py"
if [ -x "${DPDK_BIND}" ]; then
  for vf in $(ls /sys/bus/pci/devices/${PCI}/virtfn*); do
    pciaddr=$(readlink -f ${vf} | awk -F'/' '{print $(NF)}')
    ${DPDK_BIND} -b vfio-pci ${pciaddr}
  done
else
  echo "Warning: ${DPDK_BIND} not found. Ensure VFs are bound to vfio-pci."
fi

# Set IRQ affinity for physical NIC RX queues to CPU set start
echo "Setting IRQ affinity for ${NET_IF}"
for irq in $(grep irq /proc/interrupts | grep ${NET_IF} | awk '{print $1}' | tr -d ':'); do
  # pin to the lowest CPU in CPUSET for deterministic handling
  echo 2 > /proc/irq/${irq}/smp_affinity_list
done

# Run UPF container with CPU pinning and hugepages mounted
echo "Launching UPF container ${CONTAINER_NAME}"
docker run -d --name ${CONTAINER_NAME} \
  --cpuset-cpus="${CPUSET}" \
  --privileged \
  --net=host \
  --device=/dev/vfio/0 \
  -v /dev/hugepages:/dev/hugepages:rw \
  -e RUNTIME=dpdk \
  ${UPF_IMAGE}

echo "Bootstrap complete. Verify DPDK PMDs and UPF logs for runtime health."