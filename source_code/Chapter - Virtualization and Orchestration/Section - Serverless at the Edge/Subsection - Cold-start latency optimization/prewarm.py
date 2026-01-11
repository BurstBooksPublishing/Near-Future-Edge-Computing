#!/usr/bin/env python3
# Maintains a pool of pre-warmed pods in a namespace.
import os, time, logging
from kubernetes import client, config
from kubernetes.client.rest import ApiException

N = int(os.getenv("POOL_SIZE", "3"))
NS = os.getenv("NAMESPACE", "default")
IMAGE = os.getenv("IMAGE", "registry.local/myfunc:prewarm")
LABELS = {"app": "prewarm-pool", "prewarm": "true"}

logging.basicConfig(level=logging.INFO)
config.load_incluster_config()  # runs on edge K8s node
v1 = client.CoreV1Api()

def list_pool():
    return v1.list_namespaced_pod(NS, label_selector="prewarm=true").items

def make_pod(i):
    return client.V1Pod(
        metadata=client.V1ObjectMeta(name=f"prewarm-{i}", labels=LABELS),
        spec=client.V1PodSpec(
            containers=[client.V1Container(
                name="warm",
                image=IMAGE,
                args=["--idle"],  # function runtime accepts idle arg
                readiness_probe=client.V1Probe(exec=client.V1ExecAction(command=["/bin/true"]), initial_delay_seconds=1)
            )],
            restart_policy="Always"
        )
    )

def reconcile():
    pods = list_pool()
    if len(pods) < N:
        for i in range(len(pods), N):
            try:
                v1.create_namespaced_pod(NS, make_pod(i))
                logging.info("Created prewarm-%d", i)
            except ApiException as e:
                logging.warning("Create failed: %s", e)
    elif len(pods) > N:
        for p in pods[N:]:
            try:
                v1.delete_namespaced_pod(p.metadata.name, NS)
            except ApiException:
                pass

if __name__ == "__main__":
    while True:
        try:
            reconcile()
        except Exception as e:
            logging.exception("reconcile error")
        time.sleep(5)