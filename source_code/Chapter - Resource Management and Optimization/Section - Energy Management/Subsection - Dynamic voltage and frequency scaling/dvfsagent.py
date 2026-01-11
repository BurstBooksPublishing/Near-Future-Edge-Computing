#!/usr/bin/env python3
# Minimal adaptive DVFS agent for Linux edge nodes.
import os, time, logging, glob, errno, signal, sys

CPU_GLOB = "/sys/devices/system/cpu/cpu[0-9]*/cpufreq"
SAMPLE_INTERVAL = 0.25
SMOOTH = 0.6  # exponential smoothing factor
MIN_GOV = "userspace"  # require userspace governor to set freq
FREQ_STEP = 100000  # Hz step for incremental change
MAX_CHANGE_PER_SEC = 500000  # avoid frequency thrash

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
stop = False
def handle_sig(signum, frame):
    global stop; stop = True
signal.signal(signal.SIGINT, handle_sig); signal.signal(signal.SIGTERM, handle_sig)

def write_file(path, value):
    tmp = f"{value}\n".encode()
    with open(path, "wb") as f: f.write(tmp)

def read_file(path):
    with open(path, "r") as f: return f.read().strip()

def cpu_dirs():
    return [d for d in glob.glob(CPU_GLOB) if os.path.isdir(d)]

def set_freq(cpu_dir, target_hz):
    gov = read_file(os.path.join(cpu_dir, "scaling_governor"))
    if gov != MIN_GOV:
        try: write_file(os.path.join(cpu_dir, "scaling_governor"), MIN_GOV)
        except OSError: logging.warning("Cannot set governor for %s", cpu_dir)
    # clamp and write
    minf = int(read_file(os.path.join(cpu_dir, "scaling_min_freq")))
    maxf = int(read_file(os.path.join(cpu_dir, "scaling_max_freq")))
    tgt = max(minf, min(maxf, int(target_hz)))
    try: write_file(os.path.join(cpu_dir, "scaling_setspeed"), str(tgt))
    except OSError as e:
        logging.error("Failed to set freq %s: %s", cpu_dir, e)

def read_cpu_util():
    with open("/proc/stat", "r") as f:
        vals = [line.split() for line in f if line.startswith("cpu")]
    total = 0; idle = 0
    for cpu in vals[1:]:  # skip aggregate
        fields = list(map(int, cpu[1:8]))
        total += sum(fields); idle += fields[3]
    return 1.0 - (idle/total if total else 0)

def main():
    cpu_dirs_list = cpu_dirs()
    if not cpu_dirs_list: logging.error("No cpufreq dirs found"); return 1
    smooth_util = read_cpu_util()
    last_t = time.time()
    last_freq = {d: int(read_file(os.path.join(d,"scaling_cur_freq"))) for d in cpu_dirs_list}
    while not stop:
        now = time.time(); dt = max(1e-6, now - last_t); last_t = now
        util = read_cpu_util(); smooth_util = SMOOTH*smooth_util + (1-SMOOTH)*util
        # simple policy: map util to frequency linear between min and max
        for d in cpu_dirs_list:
            minf = int(read_file(os.path.join(d,"scaling_min_freq")))
            maxf = int(read_file(os.path.join(d,"scaling_max_freq")))
            target = minf + smooth_util * (maxf - minf)
            # limit rate of change
            max_delta = MAX_CHANGE_PER_SEC * dt
            delta = target - last_freq[d]
            if abs(delta) > max_delta:
                target = last_freq[d] + (max_delta if delta>0 else -max_delta)
            set_freq(d, target)
            last_freq[d] = int(target)
        time.sleep(SAMPLE_INTERVAL)
    return 0

if __name__ == "__main__":
    sys.exit(main())