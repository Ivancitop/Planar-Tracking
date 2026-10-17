"""Segmenta el objetivo mediante profundidad y publica máscara y ROI 2D."""
import cv2 as cv
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, RegionOfInterest


class DepthSegmenter(Node):
    def __init__(self):
        super().__init__('depth_segmenter')
        self.bridge = CvBridge()

        self.declare_parameter('near_mm', 500.0)
        self.declare_parameter('far_mm', 1300.0)
        self.declare_parameter('kernel_size', 3)
        self.declare_parameter('min_area_px', 2000.0)
        self.near_mm = float(self.get_parameter('near_mm').value)
        self.far_mm = float(self.get_parameter('far_mm').value)
        self.kernel_size = int(self.get_parameter('kernel_size').value)
        self.min_area = float(self.get_parameter('min_area_px').value)
        self.add_on_set_parameters_callback(self.on_params)

        rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.sync_callback)

        self.mask_pub = self.create_publisher(Image, '/target/mask', 10)
        self.roi_pub = self.create_publisher(RegionOfInterest, '/target/roi', 10)

    def on_params(self, params):
        near, far, kernel, area = self.near_mm, self.far_mm, self.kernel_size, self.min_area
        for p in params:
            if p.name == 'near_mm':
                near = float(p.value)
            elif p.name == 'far_mm':
                far = float(p.value)
            elif p.name == 'kernel_size':
                kernel = int(p.value)
            elif p.name == 'min_area_px':
                area = float(p.value)
        if near >= far:
            return SetParametersResult(successful=False, reason='near_mm debe ser menor que far_mm')
        if kernel < 1 or kernel % 2 == 0:
            return SetParametersResult(successful=False, reason='kernel_size debe ser impar y >= 1')
        if area < 0:
            return SetParametersResult(successful=False, reason='min_area_px debe ser >= 0')
        self.near_mm, self.far_mm = near, far
        self.kernel_size, self.min_area = kernel, area
        return SetParametersResult(successful=True)

    def sync_callback(self, rgb_msg, depth_msg):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')  # bag: 16UC1, mm
        mask = cv.inRange(depth, self.near_mm, self.far_mm)
        mask[depth == 0] = 0  # cero significa lectura inválida, no profundidad cercana
        kernel = cv.getStructuringElement(
            cv.MORPH_RECT, (self.kernel_size, self.kernel_size))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        candidates = [c for c in contours if cv.contourArea(c) >= self.min_area]
        clean = np.zeros_like(mask)
        roi = RegionOfInterest()
        if candidates:
            contour = max(candidates, key=cv.contourArea)
            cv.drawContours(clean, [contour], -1, 255, cv.FILLED)
            x, y, w, h = cv.boundingRect(contour)
            roi.x_offset, roi.y_offset = int(x), int(y)
            roi.width, roi.height = int(w), int(h)

        out = self.bridge.cv2_to_imgmsg(clean, 'mono8')
        out.header = depth_msg.header
        self.mask_pub.publish(out)
        self.roi_pub.publish(roi)


def main(args=None):
    rclpy.init(args=args)
    node = DepthSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
