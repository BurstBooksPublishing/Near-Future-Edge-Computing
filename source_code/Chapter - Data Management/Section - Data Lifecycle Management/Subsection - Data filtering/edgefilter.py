#!/usr/bin/env python3
# Requires: paho-mqtt
import time, math
import paho.mqtt.client as mqtt

class AdaptiveFilter:
    def __init__(self, alpha=0.2, delta=0.5):
        self.alpha = alpha
        self.delta = delta
        self.x = None
        self.last_sent = None

    def update(self, y):
        if self.x is None:                      # initialize
            self.x = y
            self.last_sent = y
            return True, y
        self.x = self.alpha*y + (1-self.alpha)*self.x
        # delta compression decision
        if abs(self.x - self.last_sent) > self.delta:
            self.last_sent = self.x
            return True, self.x
        return False, self.x

def mqtt_publish(client, topic, payload):
    client.publish(topic, payload, qos=1)     # qos=1 for delivery guarantee

def main():
    broker = "mqtt.example.local"
    topic = "plant/motor1/vib"
    client = mqtt.Client()
    client.connect(broker, 1883, 60)

    filt = AdaptiveFilter(alpha=0.05, delta=0.2)
    try:
        while True:
            y = read_sensor()                   # implement hardware read
            send, val = filt.update(y)
            if send:
                mqtt_publish(client, topic, f"{val:.3f}")
            time.sleep(0.01)                   # 100 Hz loop as example
    finally:
        client.disconnect()

# Placeholder sensor read for clarity
def read_sensor():
    # Real code reads ADC or I2C sensor; here simulate sine+noise
    t = time.time()
    return math.sin(2*math.pi*10*t) + 0.01*(2*(time.time()%1)-1)

if __name__ == "__main__":
    main()