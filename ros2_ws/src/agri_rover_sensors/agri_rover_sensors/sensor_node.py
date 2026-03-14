"""
agri_rover_sensors — sensor_node

Reads agricultural sensors and publishes at 1 Hz.

Current sensors (add drivers as hardware is wired):
  - Tank level (ultrasonic or load cell → ADC)
  - Temperature / Humidity (DHT22 or BME280)
  - Pressure (BME280)

Publishes:
  ~/sensors  (SensorData)  1 Hz
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from agri_rover_interfaces.msg import SensorData


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate', 1.0)

        rate = self.get_parameter('publish_rate').value

        # ── Publisher ────────────────────────────────────────────────────────
        self.pub = self.create_publisher(SensorData, 'sensors', 10)
        self.create_timer(1.0 / rate, self._publish)

        # TODO: initialise I2C/SPI drivers for real sensors here
        self.get_logger().info('Sensor node ready (stub — add real drivers)')

    # ── Read helpers (stub — replace with real sensor reads) ──────────────────

    def _read_tank(self) -> float:
        # TODO: ADC → distance → volume → percent
        return 100.0

    def _read_temperature(self) -> float:
        # TODO: DHT22 / BME280
        return 25.0

    def _read_humidity(self) -> float:
        # TODO: DHT22 / BME280
        return 60.0

    def _read_pressure(self) -> float:
        # TODO: BME280
        return 1013.25

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        msg          = SensorData()
        msg.tank_level   = self._read_tank()
        msg.temperature  = self._read_temperature()
        msg.humidity     = self._read_humidity()
        msg.pressure     = self._read_pressure()
        msg.stamp        = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
