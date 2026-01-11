#!/usr/bin/env python3
import cv2
import time
import hashlib
import socket
import ssl
import threading
import onnxruntime as ort

# Load ONNX face detector (MobileNet-SSD style) with TensorRT provider on Jetson
providers = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
sess = ort.InferenceSession('face_detector.onnx', providers=providers)

# Inference pre/post helpers
def preprocess(frame, size=(300,300)):
    blob = cv2.resize(frame, size)
    blob = blob[:, :, ::-1].astype('float32') / 127.5 - 1.0
    return blob.transpose(2,0,1)[None, ...]

def infer(session, inp):
    return session.run(None, {'input': inp})[0]

# Secure metadata sender (TLS)
def send_metadata(metadata_json, host='collector.example.local', port=4433):
    context = ssl.create_default_context()
    with socket.create_connection((host, port)) as sock:
        with context.wrap_socket(sock, server_hostname=host) as ssock:
            ssock.sendall(metadata_json.encode('utf-8'))

# Camera capture & processing loop (threaded)
def processing_loop(cam_idx=0):
    cap = cv2.VideoCapture(cam_idx)
    fps_timer = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        inp = preprocess(frame)
        dets = infer(sess, inp)  # model-specific output parsing required
        metadata = []
        for d in parse_detections(dets):  # parse to (x,y,w,h,score)
            x,y,w,h,score = d
            if score < 0.5: continue
            face = frame[y:y+h, x:x+w]
            # blur face in-place to avoid raw image transmission
            frame[y:y+h, x:x+w] = cv2.GaussianBlur(face, (51,51), 0)
            # send minimal, non-identifying metadata: hashed bbox + score
            bbox_str = f"{x}:{y}:{w}:{h}:{int(time.time())}"
            bbox_hash = hashlib.sha256(bbox_str.encode()).hexdigest()
            metadata.append({'bbox_hash': bbox_hash, 'score': float(score)})
        # asynchronously transmit metadata
        if metadata:
            threading.Thread(target=send_metadata, args=(str(metadata),)).start()
        # display for local operator (optional)
        cv2.imshow('anonymized', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    processing_loop()