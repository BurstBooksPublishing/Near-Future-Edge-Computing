#!/bin/bash
# run_rt_app.sh: launch a real-time edge binary with CPU affinity and SCHED_FIFO
APP=/usr/local/bin/edge_rt_controller           # production binary path
CPU=0                                            # isolated CPU for RT work
PRIO=80                                          # FIFO priority (1-99)

# ensure the CPU is isolated via kernel cmdline: isolcpus=$CPU
# create a transient systemd scope that sets scheduling and affinity
exec systemd-run --scope \
  -p CPUSchedulingPolicy=fifo \
  -p CPUSchedulingPriority=$PRIO \
  -p CPUAffinity="$CPU" \
  --slice=machine.slice \
  $APP   # systemd will manage cgroup and enforce scheduling
# alternative: chrt --fifo $PRIO taskset -c $CPU $APP