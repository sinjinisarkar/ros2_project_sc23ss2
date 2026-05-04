import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import cv2
import numpy as np
import threading
import signal
import time
from rclpy.exceptions import ROSInterruptException
from math import sin, cos


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.red_detected = False
        self.green_detected = False
        self.blue_detected = False
        self.blue_area = 0
        self.blue_cx = 0
        self.image_width = 640

        self.navigating = False
        self.task_complete = False
        self.all_waypoints_visited = False
        self.current_waypoint = 0

        self.spinning = False
        self.has_spun = False
        self.spin_start_time = None
        self.spin_duration = 6.5

        self.waypoints = [
            (-2.12, -4.99, 0.0),   # central point - can see red and green
            (-5.96, -9.06, 0.0),   # near blue box
        ]

        self.get_logger().info('Robot Controller started!')

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        self.image_width = cv_image.shape[1]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([5, 255, 255])
        lower_red2 = np.array([175, 150, 100])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        lower_green = np.array([40, 80, 80])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        lower_blue = np.array([105, 150, 100])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        display = cv_image.copy()

        for c in cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) > 500:
                if not self.red_detected:
                    self.get_logger().info('RED detected!')
                self.red_detected = True
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(display, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(display, 'RED', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        for c in cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) > 500:
                if not self.green_detected:
                    self.get_logger().info('GREEN detected!')
                self.green_detected = True
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(display, 'GREEN', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        for c in cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
            area = cv2.contourArea(c)
            if area > 500:
                if not self.blue_detected:
                    self.get_logger().info('BLUE detected!')
                self.blue_detected = True
                self.blue_area = area
                x, y, w, h = cv2.boundingRect(c)
                M = cv2.moments(c)
                if M['m00'] > 0:
                    self.blue_cx = int(M['m10'] / M['m00'])
                cv2.rectangle(display, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(display, 'BLUE', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.namedWindow('camera feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera feed', 320, 240)
        cv2.imshow('camera feed', display)
        cv2.waitKey(3)

    def send_goal(self, x, y, theta):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Action server not available!')
            self.navigating = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.navigating = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigating = False
            return
        self.get_logger().info('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Reached waypoint!')
        self.navigating = False
        if self.current_waypoint == 1 and not self.has_spun:
            self.has_spun = True
            self.spin_start_time = time.time()
            self.spinning = True
            self.get_logger().info('Spinning to detect colours...')
        if self.current_waypoint >= len(self.waypoints):
            self.all_waypoints_visited = True

    def spin(self):
        elapsed = time.time() - self.spin_start_time
        if elapsed < self.spin_duration:
            twist = Twist()
            twist.angular.z = 1.0
            self.publisher.publish(twist)
        else:
            self.stop()
            self.spinning = False
            self.get_logger().info('Spin complete!')

    def move_towards_blue(self):
        twist = Twist()
        error = self.blue_cx - self.image_width / 2
        twist.angular.z = -float(error) / 500.0

        if self.blue_area < 40000:
            twist.linear.x = 0.2
            self.get_logger().info(f'Moving towards blue, area: {self.blue_area}')
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.task_complete = True
            self.get_logger().info('Reached blue box! Stopping within 1m.')

        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot = RobotController()

    def signal_handler(sig, frame):
        robot.stop()
        robot.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.task_complete:
                robot.stop()
                robot.get_logger().info('Task complete! Staying within 1m of blue box.')
                break

            if robot.spinning:
                robot.spin()
                time.sleep(0.05)
            elif robot.all_waypoints_visited and robot.blue_detected and not robot.task_complete:
                robot.move_towards_blue()
            elif not robot.navigating and not robot.all_waypoints_visited and not robot.spinning:
                if robot.current_waypoint < len(robot.waypoints):
                    wp = robot.waypoints[robot.current_waypoint]
                    robot.get_logger().info(f'Navigating to waypoint {robot.current_waypoint}: {wp}')
                    robot.send_goal(wp[0], wp[1], wp[2])
                    robot.current_waypoint += 1
                else:
                    robot.all_waypoints_visited = True

    except ROSInterruptException:
        pass

    robot.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()