#!/usr/bin/env python3
# Minimal, production-ready control loop: predict target, pre-warm pod, atomically switch service.
from kubernetes import client, config, watch
import time, logging

config.load_kube_config()  # Works on MEC control node with kubeconfig
v1 = client.CoreV1Api()
apps = client.AppsV1Api()

# Parameters
NAMESPACE = "edge-apps"
DEPLOYMENT_NAME = "perception"
WARM_SUFFIX = "-warm"
CHECK_TIMEOUT = 10

def create_warm_deployment(node_name):
    # Create a clone of deployment with nodeSelector to pin to target node.
    body = apps.read_namespaced_deployment(DEPLOYMENT_NAME, NAMESPACE)
    body.metadata.name = DEPLOYMENT_NAME + WARM_SUFFIX
    body.metadata.resource_version = None
    body.spec.template.spec.node_selector = {"kubernetes.io/hostname": node_name}
    body.spec.replicas = 1
    # Add tight readiness probe to ensure warm instance is ready.
    apps.create_namespaced_deployment(namespace=NAMESPACE, body=body)

def wait_ready_pod(label_selector, timeout=CHECK_TIMEOUT):
    end = time.time() + timeout
    w = watch.Watch()
    for event in w.stream(v1.list_namespaced_pod, namespace=NAMESPACE,
                          label_selector=label_selector, timeout_seconds=timeout):
        pod = event['object']
        if pod.status.phase == "Running":
            ready = any((c.ready for c in pod.status.container_statuses or []))
            if ready:
                w.stop()
                return pod.metadata.name
        if time.time() > end:
            break
    raise TimeoutError("Warm pod not ready")

def switch_service(target_pod_name):
    # Update service selector to point to warm pod's labels (atomic change).
    svc = v1.read_namespaced_service("perception-svc", NAMESPACE)
    new_selector = {"app": DEPLOYMENT_NAME + WARM_SUFFIX}
    svc.spec.selector = new_selector
    v1.patch_namespaced_service("perception-svc", NAMESPACE, svc)

def cleanup_warm():
    try:
        apps.delete_namespaced_deployment(DEPLOYMENT_NAME + WARM_SUFFIX, NAMESPACE)
    except client.exceptions.ApiException:
        pass

def control_loop(predict_target_fn):
    while True:
        target_node = predict_target_fn()  # Implement ML prediction externally
        if not target_node:
            time.sleep(0.5); continue
        create_warm_deployment(target_node)
        pod_name = wait_ready_pod(label_selector=f"app={DEPLOYMENT_NAME + WARM_SUFFIX}")
        switch_service(pod_name)
        # Optional: scale down old pods gracefully via deployment rollout.
        cleanup_warm()
        time.sleep(0.1)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    control_loop(lambda: "edge-node-17")  # Replace with real predictor