import asyncio, time
import psutil, requests
# minimal gRPC client stub import placeholder
# from remote_pb2_grpc import RemoteExecutorStub

class OffloadDecider:
    def __init__(self, edge_url, cloud_url, weights):
        self.edge_url = edge_url
        self.cloud_url = cloud_url
        self.alpha, self.beta, self.gamma, self.delta = weights
        self.deadline = 0.1  # seconds, configurable

    async def measure_bandwidth(self, url):
        # simple GET to estimate RTT and throughput for small payloads
        t0 = time.time()
        r = requests.get(url + "/probe", timeout=1.0)
        rtt = time.time() - t0
        # use Content-Length or payload size to estimate B if server supports
        return max(rtt, 1e-3)

    def local_compute_time(self, cycles):
        # estimate using current CPU freq and utilization
        freq = psutil.cpu_freq().current * 1e6  # Hz
        util = max(psutil.cpu_percent(interval=0.05)/100, 0.01)
        effective_f = freq * (1 - 0.5*util)  # conservative
        return cycles / effective_f

    async def remote_latency(self, url, size_bytes, remote_compute_ms, mu, lambd):
        rtt = await self.measure_bandwidth(url)
        Bu = 20e6  # conservative uplink; could be measured
        Tu = size_bytes / Bu + rtt/2
        Dq = 1.0 / max(mu - lambd, 1e-6)  # avoid div0
        Tc = remote_compute_ms / 1000.0
        Td = 0.001  # assume tiny result download
        return Tu + Dq + Tc + Td

    async def decide(self, task):
        # task: dict with 'cycles','size_bytes','privacy_score'
        t_local = self.local_compute_time(task['cycles'])
        t_edge = await self.remote_latency(self.edge_url, task['size_bytes'],
                                           remote_compute_ms=20, mu=50, lambd=20)
        t_cloud = await self.remote_latency(self.cloud_url, task['size_bytes'],
                                           remote_compute_ms=10, mu=200, lambd=50)
        # energy and reliability models are placeholders; plug telemetry-driven models
        E_local, E_edge, E_cloud = 1.0, 0.3, 0.5
        R_local, R_edge, R_cloud = 0.99, 0.995, 0.999
        P_local, P_edge, P_cloud = 0.0, task['privacy_score'], task['privacy_score']*2

        costs = {
            'local': self.alpha*t_local + self.beta*E_local + self.gamma*(1-R_local) + self.delta*P_local,
            'edge':  self.alpha*t_edge  + self.beta*E_edge  + self.gamma*(1-R_edge)  + self.delta*P_edge,
            'cloud': self.alpha*t_cloud + self.beta*E_cloud + self.gamma*(1-R_cloud) + self.delta*P_cloud
        }
        feasible = {k:v for k,v in costs.items() 
                    if (t_local if k=='local' else (t_edge if k=='edge' else t_cloud)) <= self.deadline}
        if not feasible:
            return 'local'  # fallback policy for safety
        return min(feasible, key=feasible.get)