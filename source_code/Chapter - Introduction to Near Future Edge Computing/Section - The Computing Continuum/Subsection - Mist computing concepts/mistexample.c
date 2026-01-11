#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "model_data.h" // binary TFLite model array

static const char *TAG = "mist_node";
static esp_mqtt_client_handle_t mqtt_client = NULL;

void publish_alert(const char *msg){
  // QoS 1, retained false; topic under device namespace
  esp_mqtt_client_publish(mqtt_client, "factory/motor/alerts", msg, 0, 1, 0);
}

// TinyML task: setup interpreter once, run inference in loop
void tinyml_task(void *pv){
  constexpr int tensor_arena_size = 32*1024;
  static uint8_t tensor_arena[tensor_arena_size];
  tflite::MicroErrorReporter micro_error_reporter;
  tflite::AllOpsResolver resolver;
  const tflite::Model* model = tflite::GetModel(model_data);
  static tflite::MicroInterpreter interpreter(model, resolver,
                                             tensor_arena, tensor_arena_size,
                                             µ_error_reporter);
  interpreter.AllocateTensors();
  while(1){
    float *input = interpreter.input(0)->data.f;
    // read sensor into input buffer (blocking or DMA callback recommended)
    read_accel_samples(input, interpreter.input(0)->bytes / sizeof(float));
    interpreter.Invoke();
    float *output = interpreter.output(0)->data.f;
    if(output[0] > 0.7f){ // threshold for anomaly
      publish_alert("{\"anomaly\":1,\"ts\":1234567890}");
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // duty cycle for energy control
  }
}

void app_main(void){
  // initialize secure provisioning, MQTT TLS config, and OTA hooks
  esp_mqtt_client_config_t mqtt_cfg = { .uri = "mqtts://broker.example.com:8883" };
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_start(mqtt_client);
  xTaskCreatePinnedToCore(tinyml_task, "tinyml", 8*1024, NULL, 5, NULL, 0);
}