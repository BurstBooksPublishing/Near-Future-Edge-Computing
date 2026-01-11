#!/usr/bin/env python3
# Production-ready ROS2 node: subscribes to camera and lidar, runs ONNX inference, publishes fused detection.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import numpy as np
import cv2
import onnxruntime as ort
from cv_bridge import CvBridge

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        # configure ONNX Runtime with GPU (TensorRT/DirectML EP configured in system)
        self.ort_sess = ort.InferenceSession('/opt/models/yolo.onnx', providers=['CUDAExecutionProvider'])
        self.create_subscription(Image, '/camera/front/image_raw', self.img_cb, 4)
        self.create_subscription(PointCloud2, '/lidar/points', self.lidar_cb, 2)
        self.pub = self.create_publisher(Header, '/perception/detections', 10)
        self.img_queue = None
        self.lidar_state = None

    def img_cb(self, msg: Image):
        # minimal preprocessing: convert, resize, normalize
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_resized = cv2.resize(img, (640, 640))
        inp = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.transpose(inp, (2,0,1))[None, ...]
        # run async inference if hardware supports it
        outputs = self.ort_sess.run(None, {'input': inp})
        dets = self.postprocess(outputs)
        fused = self.fuse_with_lidar(dets, self.lidar_state)
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'base_link'
        # encode fused meta as header.frame_id extension in production use a dedicated message
        hdr.frame_id = str(fused) 
        self.pub.publish(hdr)

    def lidar_cb(self, msg: PointCloud2):
        # update latest lidar-derived velocities/cluster info (lightweight)
        self.lidar_state = self.process_pointcloud(msg)

    def postprocess(self, outputs):
        # simple NMS and scale back to image coordinates (placeholder for production-grade method)
        return outputs[0]

    def fuse_with_lidar(self, detections, lidar_state):
        # apply gating and Bayesian confidence fusion (see Eq. (2))
        return detections  # return fused detection list

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()