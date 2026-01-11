from kubernetes import config, client
import math

# Load in-cluster Kubernetes config (service account)
config.load_incluster_config()
core_api = client.CoreV1Api()
metrics = client.CustomObjectsApi()

CPU_PER_DEVICE_MILLICORES = 50  # per-device steady CPU demand in millicores
CONTROL_PLANE_OVERHEAD_MILLICORES = 200  # reserve for control-plane tasks

def millicores_to_cores(m):
    return m / 1000.0

def get_node_cpu_capacity():
    nodes = core_api.list_node().items
    total_capacity = 0
    total_allocatable = 0
    for n in nodes:
        cap = int(n.status.capacity['cpu']) * 1000
        alloc = int(n.status.allocatable['cpu']) * 1000
        total_capacity += cap
        total_allocatable += alloc
    return total_capacity, total_allocatable

def get_node_cpu_usage():
    # metrics.k8s.io API group exposed by metrics-server
    usage = metrics.list_cluster_custom_object("metrics.k8s.io", "v1beta1", "nodes")
    total_usage = 0
    for item in usage['items']:
        cpu_str = item['usage']['cpu']  # e.g., "123m"
        if cpu_str.endswith('n'):
            # fallback: convert nanocores to millicores
            total_usage += int(cpu_str[:-1]) / 1e6
        elif cpu_str.endswith('m'):
            total_usage += int(cpu_str[:-1])
        else:
            total_usage += int(cpu_str) * 1000
    return total_usage

def estimate_max_devices():
    _, allocatable = get_node_cpu_capacity()
    used = get_node_cpu_usage()
    available = allocatable - used - CONTROL_PLANE_OVERHEAD_MILLICORES
    if available <= 0:
        return 0
    return math.floor(available / CPU_PER_DEVICE_MILLICORES)

if __name__ == "__main__":
    print("Estimated max devices:", estimate_max_devices())