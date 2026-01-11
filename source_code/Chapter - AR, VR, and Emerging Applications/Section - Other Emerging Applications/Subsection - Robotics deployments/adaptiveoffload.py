#!/usr/bin/env python3
# Production-ready ROS2 node: adaptive local/remote inference decision.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import psutil, time, grpc, numpy as np
import onnxruntime as ort  # ONNX Runtime for local inference

# gRPC stub import would go here: from rpc_pb2_grpc import InferenceStub

class AdaptiveInfer(Node):
    def __init__(self):
        super().__init__('adaptive_infer')
        self.declare_parameter('model_path', '/opt/models/det.onnx')
        self.declare_parameter('mec_address', '10.0.0.10:50051')
        self.model_path = self.get_parameter('model_path').value
        self.mec_address = self.get_parameter('mec_address').value
        self.session = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider','CPUExecutionProvider'])
        self.grpc_ch = grpc.insecure_channel(self.mec_address)
        # self.stub = InferenceStub(self.grpc_ch)
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb_image, 10)
        self.pub_local = self.create_publisher(Bool, 'inference/local', 10)
        self.cpu_thresh = 0.75
        self.latency_budget = 0.04  # seconds (40ms)
    def measure_rtt(self):
        # lightweight RTT probe to MEC; replace with application-level ping.
        start = time.time()
        try:
            grpc.channel_ready_future(self.grpc_ch).result(timeout=0.01)
            rtt = (time.time() - start)
        except Exception:
            rtt = float('inf')
        return rtt
    def should_offload(self, local_infer_time):
        rtt = self.measure_rtt()
        cpu = psutil.cpu_percent(interval=0.01)/100.0
        remote_time = rtt + 0.006  # assume 6ms server inference
        # Decision: prefer local if CPU high or local faster and meets budget.
        if cpu > self.cpu_thresh:
            return True
        if local_infer_time + 0.010 < remote_time:  # add comm/act margins
            return False
        return remote_time < self.latency_budget
    def local_infer(self, img_np):
        t0 = time.time()
        # Pre/postprocessing omitted for brevity; run ONNX inference.
        outputs = self.session.run(None, {'input': img_np})
        return time.time()-t0, outputs
    def cb_image(self, msg):
        # Convert ROS Image to numpy; omitted conversion code for clarity.
        img_np = np.zeros((1,3,224,224), dtype=np.float32)  # placeholder
        t_local, outputs = self.local_infer(img_np)
        if self.should_offload(t_local):
            # Offload via gRPC; stub call omitted.
            self.pub_local.publish(Bool(data=False))
        else:
            # Use local outputs for control loop
            self.pub_local.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveInfer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()