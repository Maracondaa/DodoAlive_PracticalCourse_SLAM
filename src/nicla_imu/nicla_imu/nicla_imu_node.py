import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu


class NiclaImuNode(Node):
    def __init__(self):
        super().__init__('nicla_imu_node')

        self.imu_topic = '/nicla/imu'
        self.imu_frame_id = 'nicla_imu_frame'
        self.publish_frequency = 50.0  # Hz

        self.port = '/dev/serial/by-id/usb-Arduino_Nicla_Sense_CMSIS-DAP_7EC3A000-if01'
        self.baud = 460800
        self.reconnect_delay = 2.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(Imu, self.imu_topic, qos)

        self.ser = None
        self.last_attempt_time = 0.0
        self.running = True

        self.latest_sample = None
        self.latest_sample_seq = 0
        self.last_published_seq = 0
        self.sample_lock = threading.Lock()

        self.serial_thread = threading.Thread(
            target=self.serial_read_loop,
            daemon=True
        )
        self.serial_thread.start()

        self.timer_ = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_timer_callback
        )

        self.get_logger().info(
            f'Nicla IMU Node started, publishing at {self.publish_frequency:.1f} Hz.'
        )

    def connect_to_serial(self):
        now = time.time()

        if self.ser and self.ser.is_open:
            return True

        if now - self.last_attempt_time < self.reconnect_delay:
            return False

        self.last_attempt_time = now

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=1.0
            )
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to Nicla on {self.port}')
            return True

        except serial.SerialException as e:
            self.get_logger().warn(f'Could not connect to Nicla: {e}')
            self.ser = None
            return False

    def serial_read_loop(self):
        while self.running:
            if not self.connect_to_serial():
                time.sleep(0.1)
                continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                line = line.rstrip(',')

                fields = [f.strip() for f in line.split(',') if f.strip() != '']

                if len(fields) != 10:
                    self.get_logger().warn(
                        f'Bad IMU line ({len(fields)} fields): {line}'
                    )
                    continue

                try:
                    parts = [float(f) for f in fields]
                except ValueError:
                    self.get_logger().warn(f'Bad IMU line (parse error): {line}')
                    continue

                if all(abs(v) < 1e-9 for v in parts):
                    self.get_logger().warn(f'Rejecting all-zero IMU packet: {line}')
                    continue

                qx, qy, qz, qw = parts[0:4]
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)

                orientation_valid = True
                packet_valid = True

                if norm < 1e-6:
                    motion_zero = all(abs(v) < 1e-9 for v in parts[4:10])
                    if motion_zero:
                        self.get_logger().warn(
                            f'Rejecting invalid zero IMU packet: {line}'
                        )
                        packet_valid = False
                        continue

                    self.get_logger().warn(
                        f'Invalid quaternion norm={norm:.10e}, raw=({qx},{qy},{qz},{qw})'
                    )
                    orientation_valid = False
                else:
                    parts[0] /= norm
                    parts[1] /= norm
                    parts[2] /= norm
                    parts[3] /= norm

                with self.sample_lock:
                    self.latest_sample = (tuple(parts), orientation_valid, packet_valid)
                    self.latest_sample_seq += 1

            except serial.SerialException as e:
                self.get_logger().warn(
                    f'Serial error: {e}. Closing port and retrying.'
                )
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

            except Exception as e:
                self.get_logger().warn(f'Unexpected serial error: {e}')

    def publish_timer_callback(self):
        with self.sample_lock:
            if self.latest_sample is None:
                return

            if self.latest_sample_seq == self.last_published_seq:
                return

            data = self.latest_sample
            sample_seq = self.latest_sample_seq

        if not isinstance(data, tuple) or len(data) != 3:
            self.get_logger().warn(f'Unexpected IMU data format: {data}')
            return

        sample, orientation_valid, packet_valid = data

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        if orientation_valid:
            imu_msg.orientation.x = sample[0]
            imu_msg.orientation.y = sample[1]
            imu_msg.orientation.z = sample[2]
            imu_msg.orientation.w = sample[3]

            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
        else:
            imu_msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]

        if packet_valid:
            accel_scale = 9.80665 / 4096.0
            imu_msg.linear_acceleration.x = sample[4] * accel_scale
            imu_msg.linear_acceleration.y = sample[5] * accel_scale
            imu_msg.linear_acceleration.z = sample[6] * accel_scale

            gyro_scale = (1.0 / 16.4) * 0.0174533
            imu_msg.angular_velocity.x = sample[7] * gyro_scale
            imu_msg.angular_velocity.y = sample[8] * gyro_scale
            imu_msg.angular_velocity.z = sample[9] * gyro_scale

            imu_msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001
            ]
            imu_msg.angular_velocity_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]
        else:
            imu_msg.angular_velocity_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
            imu_msg.angular_velocity_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]

        self.publisher_.publish(imu_msg)
        self.last_published_seq = sample_seq

    def destroy_node(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NiclaImuNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

