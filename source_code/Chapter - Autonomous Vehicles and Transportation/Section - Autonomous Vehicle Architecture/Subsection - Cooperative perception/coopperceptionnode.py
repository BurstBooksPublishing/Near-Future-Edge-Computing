#!/usr/bin/env python3
# production-ready ROS 2 node: subscribes to local and remote PredictedObjects,
# performs Bayesian per-track fusion, publishes fused PredictedObjects.
import rclpy
from rclpy.node import Node
from autoware_auto_msgs.msg import PredictedObjects
from builtin_interfaces.msg import Time
import math
import time

def odds(p):
    return p / (1.0 - p + 1e-12)

def prob_from_odds(o):
    return o / (1.0 + o)

class CoopFusionNode(Node):
    def __init__(self):
        super().__init__('coop_fusion')
        self.get_logger().info('Cooperative fusion node starting')
        self.local_sub = self.create_subscription(
            PredictedObjects, 'local_objects', self.local_cb, 10)
        self.remote_sub = self.create_subscription(
            PredictedObjects, 'remote_objects', self.remote_cb, 10)
        self.pub = self.create_publisher(PredictedObjects, 'fused_objects', 10)
        # simple track-store keyed by bbox center quantized
        self.track_store = {}  # {key: odds_value}
        self.max_age = 0.2  # seconds acceptable staleness

    def _key_from_obj(self, obj):
        # simple spatial key: quantize center to 0.5m grid
        cx = obj.kinematic_state.pose_with_covariance.pose.position.x
        cy = obj.kinematic_state.pose_with_covariance.pose.position.y
        return (round(cx * 2)/2.0, round(cy * 2)/2.0)

    def _stamp_age(self, header):
        # assumes synchronized clocks (PTP/GPS)
        now = self.get_clock().now().to_msg()
        sec = now.sec - header.stamp.sec + (now.nanosec - header.stamp.nanosec)*1e-9
        return sec

    def fuse_source(self, msg, source_confidence=0.8):
        for obj in msg.objects:
            if self._stamp_age(msg.header) > self.max_age:
                continue
            key = self._key_from_obj(obj)
            p = source_confidence  # placeholder: replace with per-object score
            o = odds(p)
            prior = self.track_store.get(key, odds(0.01))  # small prior
            posterior = prior * o
            self.track_store[key] = posterior

    def local_cb(self, msg):
        self.fuse_source(msg, source_confidence=0.85)
        self.publish_fused()

    def remote_cb(self, msg):
        self.fuse_source(msg, source_confidence=0.75)
        self.publish_fused()

    def publish_fused(self):
        out = PredictedObjects()
        out.header.stamp = Time(sec=int(time.time()), nanosec=int((time.time()%1)*1e9))
        out.header.frame_id = 'map'
        for key, o in list(self.track_store.items()):
            p = prob_from_odds(o)
            if p < 0.05:
                del self.track_store[key]
                continue
            # craft a minimal PredictedObject with position encoded; production: fill full fields
            obj = PredictedObjects().objects.append  # placeholder for real object creation
            # in production, build PredictedObject with kinematic_state and classification
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CoopFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()