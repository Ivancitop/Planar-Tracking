import numpy as np, cv2 as cv, rclpy, message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

W, H = 0.1056, 0.074   # metros (105.6 mm x 74.0 mm)

def order_corners(p):            # -> TL, TR, BR, BL
    s = p.sum(axis=1); d = np.diff(p, axis=1).ravel()
    return np.array([p[np.argmin(s)], p[np.argmin(d)],
                     p[np.argmax(s)], p[np.argmax(d)]], dtype=np.float32)

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')
        self.bridge = CvBridge()
        self.K = self.D = None
        # mismo orden que order_corners: TL, TR, BR, BL (origen en el centro, Z=0)
        self.obj_pts = np.array([[-W/2,-H/2,0],[ W/2,-H/2,0],
                                 [ W/2, H/2,0],[-W/2, H/2,0]], dtype=np.float32)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self.info_cb, qos_profile_sensor_data)
        rgb = message_filters.Subscriber(self, Image, '/camera/color/image_raw',
                                         qos_profile=qos_profile_sensor_data)
        mask = message_filters.Subscriber(self, Image, '/target/mask')
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, mask], 10, 0.05)
        self.sync.registerCallback(self.cb)
        self.pose_pub = self.create_publisher(PoseStamped, '/target/pose', 10)
        self.tf_br = TransformBroadcaster(self)

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def cb(self, rgb_msg, mask_msg):
        if self.K is None:
            return
        mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return
        c = max(cnts, key=cv.contourArea)
        approx = cv.approxPolyDP(c, 0.02 * cv.arcLength(c, True), True)
        pts = approx.reshape(-1, 2) if len(approx) == 4 else cv.boxPoints(cv.minAreaRect(c))
        img_pts = order_corners(pts.astype(np.float32))

        ok, rvec, tvec = cv.solvePnP(self.obj_pts, img_pts, self.K, self.D,
                                     flags=cv.SOLVEPNP_IPPE)
        if not ok:
            return

        # error de reproyección (RMS en px)
        proj, _ = cv.projectPoints(self.obj_pts, rvec, tvec, self.K, self.D)
        per_corner = np.linalg.norm(proj.reshape(4, 2) - img_pts, axis=1)
        rms = float(np.sqrt(np.mean(per_corner ** 2)))
        self.get_logger().info(f'RMS={rms:.2f}px  por esquina={np.round(per_corner,2)}')

        Rm, _ = cv.Rodrigues(rvec)
        q = R.from_matrix(Rm).as_quat()          # x, y, z, w
        q /= np.linalg.norm(q)

        stamp = rgb_msg.header.stamp             # tiempo de la IMAGEN, no now()
        frame = rgb_msg.header.frame_id          # frame óptico
        t = tvec.ravel()

        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, t)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
        self.pose_pub.publish(ps)

        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = 'target_frame'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t)
        tf.transform.rotation = ps.pose.orientation
        self.tf_br.sendTransform(tf)