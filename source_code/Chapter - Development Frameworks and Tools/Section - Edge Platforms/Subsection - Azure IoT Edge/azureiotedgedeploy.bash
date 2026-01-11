#!/usr/bin/env bash
# Prereqs: az cli extension azure-iot, jq installed; user logged in with az login
HUB="my-iothub.example.com"               # IoT Hub name
DEVICE_ID="jetson-xavier-nx-edge1"        # Edge device id
IMAGE="myacr.azurecr.io/vision-onnx:prod"  # OCI image with ONNX Runtime + CUDA
DEPLOY_NAME="vision-deploy"

# Create IoT Edge device identity (edge enabled)
az iot hub device-identity create --hub-name "$HUB" \
  --device-id "$DEVICE_ID" --edge-enabled true

# Retrieve deployment manifest template and customize -- here-doc contains JSON
az iot edge deployment create --config-id "$DEPLOY_NAME" --hub-name "$HUB" \
  --content @- <<'JSON'
{
  "modulesContent": {
    "$edgeAgent": {
      "properties.desired": {
        "modules": {
          "visionModule": {
            "settings": {
              "image": "myacr.azurecr.io/vision-onnx:prod", 
              "createOptions": "{\"deviceRequests\": [{\"Driver\": \"nvidia\", \"Count\": -1, \"Capabilities\": [[\"gpu\"]]}]}"
            },
            "type": "docker",
            "status": "running",
            "restartPolicy": "always"
          }
        },
        "runtime": {
          "settings": {
            "minDockerVersion": "v1.25"
          },
          "type": "containerd"
        }
      }
    },
    "$edgeHub": {
      "properties.desired": {
        "routes": {
          "telemetryToIoTHub": "FROM /messages/* INTO $upstream"
        },
        "storeAndForwardConfiguration": {
          "timeToLiveSecs": 7200
        }
      }
    }
  }
}
JSON
# Note: ensure device is provisioned with IoT Edge runtime and has NVIDIA container runtime configured.