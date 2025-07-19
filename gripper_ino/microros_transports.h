#ifndef MICROROS_TRANSPORTS_H
#define MICROROS_TRANSPORTS_H

#include <Arduino.h>
#include <micro_ros_arduino.h>

#define SERIAL_BUFFER_SIZE 512

// Forward declaration
bool serial_open(struct uxrCustomTransport * transport);
bool serial_close(struct uxrCustomTransport * transport);
size_t serial_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, int timeout);
size_t serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout);

// Custom transport setup
bool set_microros_transports()
{
  return rmw_uros_set_custom_transport(
    true,
    (void *) &Serial,
    serial_open,
    serial_close,
    serial_write,
    serial_read
  );
}

// Actual implementations
bool serial_open(struct uxrCustomTransport * transport) {
  Serial.begin(115200);
  return true;
}

bool serial_close(struct uxrCustomTransport * transport) {
  Serial.end();
  return true;
}

size_t serial_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, int timeout) {
  return Serial.write(buf, len);
}

size_t serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout) {
  size_t read_len = 0;
  int t_start = millis();
  while (read_len < len && (millis() - t_start) < timeout) {
    if (Serial.available()) {
      buf[read_len++] = Serial.read();
    }
  }
  return read_len;
}

#endif // MICROROS_TRANSPORTS_H
