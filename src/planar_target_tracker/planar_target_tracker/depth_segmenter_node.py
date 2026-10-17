'''
    This node is responsible for isolating the object from the background and publishing a clean mathematical mask.
'''
#import the modules we need for ros messages and vision pipeline
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
import cv2 as cv
from rcl_interfaces.msg import SetParametersResult

class Depth_Segmenter (Node):
    def __init__(self):
        super().__init__('depth_segmenter')#call constructor parent to create node object
        self.bridge_ = CvBridge() #cv2 bridge from ros to opencv

        #parameters
        self.declare_parameter('near_mm', 300.0)
        self.declare_parameter('far_mm', 1500.0)
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('min_area_px', 2000.0)
        self.near_mm = self.get_parameter('near_mm').value
        self.far_mm = self.get_parameter('far_mm').value
        self.kernel_size = self.get_parameter('kernel_size').value
        self.min_area = self.get_parameter('min_area_px').value
        self.add_on_set_parameters_callback(self.on_params) #para cambiar de forma dinámica
        
        self.k_=None

        #suscriptions
        self.create_subscription(CameraInfo,'/camera/color/camera_info',self.info_callback,10)
        self.create_subscription(Image,'/camera/color/image_raw',self.rgb_callback,10)
        self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',self.depth_callback,10)

        #publisher
        self.mask_pub = self.create_publisher(Image, '/target/mask', 10)
        self.roi_pub = self.create_publisher(RegionOfInterest, '/target/roi', 10)


    def on_params(self, params):
        for p in params:
            if p.name == 'near_mm': self.near_mm = p.value
            elif p.name == 'far_mm': self.far_mm = p.value
            elif p.name == 'kernel_size': self.kernel_size = max(1, int(p.value))
            elif p.name == 'min_area_px': self.min_area = p.value
        return SetParametersResult(successful=True)

    def isolate_rectangle(self,depth:any,near_mm:int,far_mm:int)->any:
        return cv.inRange(depth,near_mm,far_mm)

    def info_callback(self,msg):
        self.k_ = msg.k

    def rgb_callback(self,msg)->None:
        pass

    def clousure(self,img)->any:
        if img is not None:
            kernel = cv.getStructuringElement(cv.MORPH_RECT, (self.kernel_size, self.kernel_size))
            img = cv.morphologyEx(img,cv.MORPH_OPEN,kernel)
            return cv.morphologyEx(img,cv.MORPH_CLOSE,kernel)


    def depth_callback(self,msg)->None:
        #if self.k_ is None:
            #return
        image = self.bridge_.imgmsg_to_cv2(msg, '16UC1')#Parse the ros image msg to cv2 image
        image = np.nan_to_num(image, nan=0.0, posinf=0.0, neginf=0.0)

        self.isoimage_ = self.isolate_rectangle(image,self.near_mm,self.far_mm)
        self.isoimage_[image<=0] = 0
        #clousure morphology operation
        self.image_final_=self.clousure(self.isoimage_)
        h, w = self.image_final_.shape #get the dimensions of image
        #get figure contours
        self.contours_,_ = cv.findContours(image,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv.contourArea(c) > self.min_area]
        self.image_final_ = np.zeros_like(self.image_final_)
        roi = RegionOfInterest()
        if contours:
            c = max(contours, key=cv.contourArea)
            cv.drawContours(self.image_final_, [c], -1, 255, cv.FILLED)
            x, y, w, h = cv.boundingRect(c)
            roi.x_offset, roi.y_offset, roi.width, roi.height = x, y, w, h

        out = self.bridge.cv2_to_imgmsg(self.image_final_, 'mono8')
        out.header = msg.header          # conserva stamp original
        self.mask_pub.publish(out)
        self.roi_pub.publish(roi)              # width=0 -> no se detectó objeto

        


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