'''
    This node is responsible for isolating the object from the background and publishing a clean mathematical mask.
'''
#import the modules we need for ros messages and vision pipeline
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


class Depth_Segmenter (Node):
    def __init__(self):
        super().__init__('depth_segmenter')#call constructor parent to create node object
        self.brige_ = CvBridge() #cv2 bridge from ros to opencv

        #parameters
        self.declare_parameter('depth',10.0)
        self.declare_parameter('near_mm',10.0)
        self.declare_parameter('far_mm',10.0)
        self.depth_ = self.get_parameter('depth').value
        self.near_mm_ = self.get_parameter('near_mm').value
        self.far_mm_ = self.get_parameter('far_mm').value


        #suscriptions
        self.create_subscription(CameraInfo,'/camera_info',self.info_callback,10)
        self.create_subscription(Image,'/rgb',self.rgb_callback,10)
        self.create_subscription(Image,'/depth',self.depth_callback,10)


    def isolate_rectangle(self,depth:float,near_mm:int,far_mm:int)->None:
        self.brige_.inRange(depth,near_mm,far_mm)

    def info_callback(self,msg):
        self.k_ = msg.k

    def rgb_callback(self,msg)->None:
        pass

    def depth_callback(self,msg)->None:
        if self.k_ is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, '32FC1')#Parse the ros image msg to cv2 image
        h, w = image.shape #get the dimensions of image
        #Get the intrinsic parameters of the camera
        fx = self.k[0]
        fy = self.k[4]
        cx = self.k[2]
        cy = self.k[5]
        u, v = np.mgrid[h // 4 : 3 * h // 4 : 4, w // 4 : 3 * w // 4 : 4]#we define a grid for define to iterate the image domain
        z = image[u, v]#we get the depth value for the sampled domain
        valid = np.isfinite(z) & (z > 0) #validation for inf and 0 values

        '''xyz = np.column_stack(
            ((u[valid] - cx) * z[valid]/fx,
            (v[valid] - cy) * z[valid]/fy,
            z[valid])).astype(np.float32) #caulcule the coordinates of each pointcloude
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg.header, xyz))'''


def main(args=None):
    rclpy.init(args=args)
    node = Depth_Segmenter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()