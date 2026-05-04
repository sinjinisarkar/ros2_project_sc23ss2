import cv2
import threading
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import signal


class colourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.callback,
            10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Green detection
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Red detection
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Blue detection
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        combined_mask = green_mask | red_mask | blue_mask
        filtered = cv2.bitwise_and(cv_image, cv_image, mask=combined_mask)

        cv2.namedWindow('camera feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera feed', 320, 240)
        cv2.imshow('camera feed', filtered)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    cI = colourIdentifier()

    def signal_handler(sig, frame):
        cI.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            pass
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()