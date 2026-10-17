"""Estima la pose 3D del rectángulo y publica PoseStamped y TF."""
import cv2 as cv
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from tf2_ros import TransformBroadcaster


def make_object_points(width_m, height_m):
    """Esquinas 3D coplanares en orden TL, TR, BR, BL; origen en el centro."""
    w, h = width_m / 2.0, height_m / 2.0
    return np.array([[-w, -h, 0.0], [w, -h, 0.0],
                     [w, h, 0.0], [-w, h, 0.0]], dtype=np.float32)


def contour_to_quad(contour):
    """Aproxima el contorno convexo a cuatro esquinas con eps=0.02."""
    hull = cv.convexHull(contour)
    approx = cv.approxPolyDP(hull, 0.02 * cv.arcLength(hull, True), True)
    if len(approx) == 4:
        return approx.reshape(4, 2).astype(np.float32)
    return cv.boxPoints(cv.minAreaRect(contour)).astype(np.float32)


def order_corners(points):
    """Orden cíclico y comienzo en la esquina superior izquierda de la imagen."""
    points = np.asarray(points, dtype=np.float32)
    center = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(
        points[:, 1] - center[1], points[:, 0] - center[0]))]
    return np.roll(points, -int(np.argmin(points.sum(axis=1))), axis=0)


def rotmat_to_quat(rotation):
    """Convierte una matriz de rotación 3x3 en cuaternión unitario (x,y,z,w)."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        qx = 0.25 * s
        qw, qy, qz = ((rotation[2, 1] - rotation[1, 2]) / s,
                      (rotation[0, 1] + rotation[1, 0]) / s,
                      (rotation[0, 2] + rotation[2, 0]) / s)
    elif rotation[1, 1] > rotation[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        qy = 0.25 * s
        qw, qx, qz = ((rotation[0, 2] - rotation[2, 0]) / s,
                      (rotation[0, 1] + rotation[1, 0]) / s,
                      (rotation[1, 2] + rotation[2, 1]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        qz = 0.25 * s
        qw, qx, qy = ((rotation[1, 0] - rotation[0, 1]) / s,
                      (rotation[0, 2] + rotation[2, 0]) / s,
                      (rotation[1, 2] + rotation[2, 1]) / s)
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.bridge = CvBridge()
        self.declare_parameter('target_width_m', 0.1056)
        self.declare_parameter('target_height_m', 0.074)
        self.declare_parameter('target_frame', 'target_frame')
        self.obj_pts = make_object_points(
            float(self.get_parameter('target_width_m').value),
            float(self.get_parameter('target_height_m').value))
        self.target_frame = self.get_parameter('target_frame').value
        self.K = None
        self.D = None
        self.roi = None

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(RegionOfInterest, '/target/roi', self.roi_cb, 10)
        rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        mask_sub = message_filters.Subscriber(self, Image, '/target/mask')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, mask_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb)

        self.pose_pub = self.create_publisher(PoseStamped, '/target/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def info_cb(self, msg):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def roi_cb(self, msg):
        self.roi = msg

    def cb(self, rgb_msg, mask_msg):
        if self.K is None:
            return
        mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv.contourArea)
        corners = np.ascontiguousarray(order_corners(contour_to_quad(contour)))
        try:
            ok, rvec, tvec = cv.solvePnP(
                self.obj_pts, corners, self.K, self.D, flags=cv.SOLVEPNP_IPPE)
        except cv.error as exc:
            self.get_logger().warning(f'solvePnP falló: {exc}')
            return
        if not ok:
            self.get_logger().warning('solvePnP no encontró una solución para este frame')
            return

        projected, _ = cv.projectPoints(self.obj_pts, rvec, tvec, self.K, self.D)
        errors = np.linalg.norm(projected.reshape(4, 2) - corners, axis=1)
        rms = float(np.sqrt(np.mean(errors ** 2)))
        rotation, _ = cv.Rodrigues(rvec)
        qx, qy, qz, qw = rotmat_to_quat(rotation)
        translation = tvec.ravel()
        self.get_logger().info(
            f'RMS reproyección = {rms:.2f} px | por esquina (TL,TR,BR,BL) = '
            f'{np.round(errors, 2).tolist()} | tz = {translation[2] * 1000:.0f} mm')

        pose = PoseStamped()
        pose.header.stamp = rgb_msg.header.stamp
        pose.header.frame_id = rgb_msg.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, translation)
        pose.pose.orientation.x, pose.pose.orientation.y = float(qx), float(qy)
        pose.pose.orientation.z, pose.pose.orientation.w = float(qz), float(qw)
        self.pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
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
