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
            topic='/ai/results', 
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

        self.declare_parameter('conf_thresh', 0.85, conf_param_desc)
        
        
    def image_callback(self, msg):
        conf_thresh = self.get_parameter('conf_thresh').value

        """msg = AIRes()
        msg.name = "TEST"
        msg.index = 0
        msg.x = 0
        msg.y = 0
        msg.w = 0
        msg.h = 0

        self.publisher_.publish(msg)"""

        try:
            # Convert the ROS image message to OpenCV format
            bridge = CvBridge()
            np_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Pass the image through the model for inference
            results = self._model(np_image)

            #results = results.filter(conf=conf_thresh)

            for pred in results:
                names = pred.names

                for i in range(len(pred.boxes)):
                    name = names.get(int(pred.boxes.cls[i]))
                    confidence = pred.boxes.conf[i]
                    bounding_box = pred.boxes[i].xywh[0]

                    #print(f"{name} {int(confidence*100)}% {bounding_box}")
                    
                    print(type(bounding_box[0].item()));

                    msg = AIRes()
                    msg.name = name
                    msg.index = i
                    msg.x = int(round(bounding_box[0].item()))
                    msg.y = int(round(bounding_box[1].item()))
                    msg.w = int(round(bounding_box[2].item()))
                    msg.h = int(round(bounding_box[3].item()))

                    print(msg)

                    self.publisher_.publish(msg)

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
