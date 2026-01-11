#!/usr/bin/env python3
import cv2, time, json, sys
import numpy as np
import paho.mqtt.client as mqtt
import onnxruntime as ort

MODEL_PATH = "/opt/models/pcb_yolov7_nano.onnx"
CAMERA_URI = "rtsp://192.168.1.20/stream"
MQTT_BROKER = "192.168.1.10"
MQTT_TOPIC = "factory/line1/defects"
CONF_THRESH = 0.35
NMS_THRESH = 0.45

# Select best execution provider: TensorRT/CUDA if available, else CPU.
providers = []
if "TensorrtExecutionProvider" in ort.get_available_providers():
    providers = ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]
elif "CUDAExecutionProvider" in ort.get_available_providers():
    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
else:
    providers = ["CPUExecutionProvider"]

sess = ort.InferenceSession(MODEL_PATH, providers=providers)
input_name = sess.get_inputs()[0].name
input_shape = sess.get_inputs()[0].shape  # e.g., [1,3,320,320]
H, W = input_shape[-2], input_shape[-1]

# MQTT setup
client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

def preprocess(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (W, H))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2,0,1))[None, ...]  # NHWC->NCHW
    return img

def non_max_suppression(preds, conf_thresh=CONF_THRESH, iou_thresh=NMS_THRESH):
    # preds shape: Nx( x,y,w,h,conf, class_probs... ); implement simple NMS
    boxes, scores, classes = [], [], []
    for p in preds:
        conf = p[4]
        if conf < conf_thresh: continue
        cls = int(np.argmax(p[5:]))
        score = conf * p[5+cls]
        if score < conf_thresh: continue
        cx, cy, w, h = p[0:4]
        x1 = cx - w/2; y1 = cy - h/2; x2 = cx + w/2; y2 = cy + h/2
        boxes.append([x1,y1,x2,y2]); scores.append(score); classes.append(cls)
    if not boxes: return []
    idxs = cv2.dnn.NMSBoxes(boxes, scores, conf_thresh, iou_thresh)
    result = []
    for i in idxs:
        i = i[0] if isinstance(i, (list,tuple,np.ndarray)) else i
        result.append((boxes[i], scores[i], classes[i]))
    return result

cap = cv2.VideoCapture(CAMERA_URI)
# Warm-up frames
for _ in range(5):
    ret, _ = cap.read()

while True:
    t0 = time.time()
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.02); continue
    img_in = preprocess(frame)
    t1 = time.time()
    preds = sess.run(None, {input_name: img_in})[0]  # model-specific output parsing
    t2 = time.time()
    detections = non_max_suppression(preds.reshape(-1, preds.shape[-1]))
    # Publish compact events and log latency
    for box, score, cls in detections:
        event = {"timestamp": time.time(), "class_id": int(cls), "score": float(score)}
        client.publish(MQTT_TOPIC, json.dumps(event), qos=1)
    # Optional: actuate line-stop via RTOS or PLC interface if severe defect
    t3 = time.time()
    # Profiling log
    print(f"capture {t1-t0:.3f}s preprocess {t1-t0:.3f}s infer {t2-t1:.3f}s post {t3-t2:.3f}s")