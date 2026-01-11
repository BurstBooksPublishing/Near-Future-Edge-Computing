#!/usr/bin/env python3
# Minimal production-ready inference server for Edge TPU or CPU fallback.
import os, signal, sys, logging
from typing import Optional
from fastapi import FastAPI
import uvicorn
from tflite_runtime.interpreter import Interpreter, load_delegate

MODEL_PATH = os.environ.get("MODEL_PATH", "model.tflite")
NUM_THREADS = int(os.environ.get("NUM_THREADS", "2"))

app = FastAPI()
logger = logging.getLogger("inference")
logger.setLevel(logging.INFO)

def make_interpreter(model_path: str) -> Interpreter:
    # Try Edge TPU delegate, fallback to CPU interpreter.
    delegate = None
    try:
        delegate = load_delegate("libedgetpu.so.1")
        logger.info("Edge TPU delegate loaded")
    except Exception:
        logger.info("Edge TPU not available; using CPU")
    if delegate:
        return Interpreter(model_path=model_path, experimental_delegates=[delegate],
                           num_threads=NUM_THREADS)
    return Interpreter(model_path=model_path, num_threads=NUM_THREADS)

interpreter = make_interpreter(MODEL_PATH)
interpreter.allocate_tensors()

@app.get("/health")
def health():
    return {"status": "ok"}

@app.post("/infer")
def infer(payload: dict):
    # Expect preprocessed input array in payload["input"].
    inp = payload["input"]
    input_details = interpreter.get_input_details()[0]
    interpreter.set_tensor(input_details["index"], inp)
    interpreter.invoke()  # blocking inference
    out = interpreter.get_tensor(interpreter.get_output_details()[0]["index"])
    return {"output": out.tolist()}

def shutdown(signal_no, frame):
    logger.info("Shutting down")
    sys.exit(0)

signal.signal(signal.SIGTERM, shutdown)
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)