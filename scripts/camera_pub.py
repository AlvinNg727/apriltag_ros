#!/usr/bin/env python3
"""Publish a V4L2 camera as /down_camera/image_raw and /down_camera/camera_info.

Ported from camera.py: frames are read in a background thread so the ROS timer
never blocks on the driver. Calibrate with:
    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
        image:=/down_camera/image_raw camera:=/down_camera
then save the result over apriltag_ros/calibration/camera.yaml.
"""

import threading
import time

import cv2
import rclpy
from camera_info_manager import CameraInfoManager, CameraInfoMissingError
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ThreadedCamera:
    """Reads frames in a separate thread and measures true hardware FPS."""

    def __init__(self, src, width, height, fps):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {src} via V4L2")

        # Set MJPEG format first
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Disable auto-exposure to prevent frame rate drops in low light
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

        # Manual image controls for the new camera
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        self.cap.set(cv2.CAP_PROP_SATURATION, 0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 10000)

        self.cap.set(cv2.CAP_PROP_GAIN, 0)

        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.fps = 0.0
        # Sequence number lets the publisher skip frames it has already sent.
        self.seq = 0
        self.lock = threading.Lock()

        # Dedicated background thread for frame reading
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        prev_time = time.time()
        frame_count = 0

        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
                break

            with self.lock:
                self.ret, self.frame = ret, frame
                self.seq += 1

            # Calculate FPS based on actual hardware reads
            frame_count += 1
            curr_time = time.time()
            elapsed = curr_time - prev_time

            # Update FPS calculation every 0.5 seconds for a stable readout
            if elapsed >= 0.5:
                self.fps = frame_count / elapsed
                frame_count = 0
                prev_time = curr_time

    def read(self):
        with self.lock:
            return self.ret, self.frame, self.fps, self.seq

    def release(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter(
            "video_device",
            "/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0",
        )
        self.declare_parameter("image_width", 1600)
        self.declare_parameter("image_height", 1200)
        self.declare_parameter("framerate", 60.0)
        self.declare_parameter("frame_id", "down_camera_optical_frame")
        self.declare_parameter("camera_name", "down_camera")
        self.declare_parameter(
            "camera_info_url",
            "package://apriltag_ros/cfg/camera/down_camera.yaml",
        )

        self.frame_id = self.get_parameter("frame_id").value
        fps = float(self.get_parameter("framerate").value)

        self.bridge = CvBridge()
        self.cam = ThreadedCamera(
            self.get_parameter("video_device").value,
            self.get_parameter("image_width").value,
            self.get_parameter("image_height").value,
            fps,
        )

        self.image_pub = self.create_publisher(
            Image, "image_raw", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "camera_info", qos_profile_sensor_data
        )
        self.last_seq = -1

        self.info_mgr = CameraInfoManager(
            self,
            cname=self.get_parameter("camera_name").value,
            url=self.get_parameter("camera_info_url").value,
        )
        self.calibrated = False
        try:
            self.info_mgr.loadCameraInfo()
            self.calibrated = self.info_mgr.isCalibrated()
        except (CameraInfoMissingError, OSError) as e:
            self.get_logger().error(f"Calibration load failed: {e}")
        if not self.calibrated:
            self.get_logger().warn(
                "Camera is uncalibrated, publishing null CameraInfo."
            )

        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(
            f"Publishing {self.get_parameter('video_device').value} on {self.get_namespace()}/image_raw"
        )

    def publish_frame(self):
        ret, frame, fps, seq = self.cam.read()
        if self.cam.stopped or not ret or frame is None:
            self.get_logger().warn("No frame from camera", throttle_duration_sec=5.0)
            return

        # Only publish when the capture thread produced a new frame. Publishing
        # the same frame repeatedly at the timer rate just multiplies load and
        # makes apriltag miss exact image/camera_info pairs.
        if seq == self.last_seq:
            return
        self.last_seq = seq

        # self.get_logger().info(f"FPS: {fps}")

        now = self.get_clock().now().to_msg()

        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        try:
            info = self.info_mgr.getCameraInfo()
        except CameraInfoMissingError:
            info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = self.frame_id
        self.info_pub.publish(info)

    def destroy_node(self):
        self.cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
