import traceback
import cv2
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from detect_result.msg import AIRes

import numpy as np
from cv_bridge import CvBridge

from ultralytics import YOLO

class BuoyBoxDetectionPublisher(Node):
    def __init__(self):
        self._model = YOLO('best.pt')
        super().__init__('ai')
        
        # publish to processed node
        self.publisher_ = self.create_publisher(
            msg_type=AIRes, 
            topic='processed', 
            qos_profile=10
        )

        # subscribe to image processing node
        self.subscription_ = self.create_subscription(
            msg_type=Image,
            topic='/color/image_raw',
            callback=self.image_callback,
            qos_profile=10
        )

        conf_param_desc = ParameterDescriptor(
            description='Sets the minimum threshold for confidence in predictions')

        self.declare_parameter('conf_thresh', 85, conf_param_desc)
        
        
    def image_callback(self, msg):
        print("test")

        msg = AIRes()
        msg.name = "TEST"
        msg.index = 0
        msg.x = 0
        msg.y = 0
        msg.w = 0
        msg.h = 0

        self.publisher_.publish(msg)

        return

        try:
            # Convert the ROS image message to OpenCV format
            bridge = CvBridge()
            np_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Pass the image through the model for inference
            result = self._model(np_image)

            # Get the visualized image with bounding boxes and labels
            #visualized_image = result[0].plot()

            # Convert the visualized image to OpenCV format
            #cv_image = cv2.cvtColor(np.array(visualized_image), cv2.COLOR_RGB2BGR)

            # Convert the OpenCV image to ROS Image message
            #modified_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher_.publish(modified_msg)
        except Exception as e:
            self.get_logger.error(f'Error: {e}')
            self.get_logger.debug(f'Error traceback: {traceback.format_exc}')

def main(args=None):
    print('Hi from buoy_detection.')
    
    rclpy.init(args=args)

    buoy_box_detection_publisher = BuoyBoxDetectionPublisher()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(buoy_box_detection_publisher)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        buoy_box_detection_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
