import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import numpy as np
from PIL import Image as PILImage

import torch
from torchvision.transforms import functional as F

from ultralytics import YOLO


def main():
    print('Hi from buoy_detection.')


if __name__ == '__main__':
    main()
