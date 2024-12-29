import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from auro_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemHolder, ItemHolders, ItemLog, ItemList, Item, RobotList, Robot, Zone, ZoneList


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter(
            'x').get_parameter_value().double_value
        self.initial_y = self.get_parameter(
            'y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter(
            'yaw').get_parameter_value().double_value

        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        self.scan_triggered = [False] * 4
        self.SCAN_THRESHOLD = 0.5
        self.SCAN_FRONT = 0
        self.SCAN_LEFT = 1
        self.SCAN_BACK = 2
        self.SCAN_RIGHT = 3

        self.item_to_collect = None
        self.holding_item = False
        self.target_zone = None
        self.ZONE_NUMBER = 1
        self.ITEM_COLOR = 'RED'

        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer_period = 0.1  # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.robot_name}/cmd_vel', 10)

        # Odom subscriber
        self.create_subscription(Odometry, f'{self.robot_name}/odom',
                                 self.odom_callback, 10, callback_group=timer_callback_group)

        # Item Manager
        # FIXME: Check queue size
        self.create_subscription(ItemLog, '/item_log', self.item_log, 100, callback_group=timer_callback_group)
        self.create_subscription(ItemHolders, '/item_holders', self.item_holder,
                                 100, callback_group=timer_callback_group)

        # Item Manager Services
        self.pick_up_item = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        while not self.pick_up_item.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pick up service not available, waiting again...')

        self.offload_item = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)
        while not self.pick_up_item.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('offload service not available, waiting again...')

        self.req = ItemRequest.Request()

        # Item Sensor
        self.create_subscription(ItemList, f'/{self.robot_name}/items',
                                 self.items, 100, callback_group=timer_callback_group)
        self.create_subscription(Image, f'/{self.robot_name}/camera/image_items',
                                 self.image_items, 100, callback_group=timer_callback_group)

        # Robot sensor
        self.create_subscription(RobotList, f'/{self.robot_name}/robots',
                                 self.robots, 100, callback_group=timer_callback_group)
        self.create_subscription(Image, f'/{self.robot_name}/camera/image_robots',
                                 self.image_robots, 100, callback_group=timer_callback_group)

        # Zone sensor
        self.create_subscription(ZoneList, f'/{self.robot_name}/zone', self.zone,
                                 100, callback_group=timer_callback_group)
        self.create_subscription(Image, f'/{self.robot_name}/camera/image_zone',
                                 self.image_zone, 100, callback_group=timer_callback_group)

        # LaserScanner subscriber
        self.create_subscription(LaserScan, f'{self.robot_name}/scan', self.scan_callback,
                                 QoSPresetProfiles.SENSOR_DATA.value, callback_group=timer_callback_group)


    ##################################
    # Controller
    ##################################
    def controller(self, item=False):
        twist = Twist()

        # Avoid obstacles dynamically
        if self.scan_triggered[self.SCAN_FRONT]:
            # self.get_logger().info("Obstacle detected in front! Turning right...")
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Turn right
            return
        elif self.scan_triggered[self.SCAN_LEFT]:
            # self.get_logger().info("Obstacle detected on the left! Turning right...")
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Turn right
            return
        elif self.scan_triggered[self.SCAN_RIGHT]:
            # self.get_logger().info("Obstacle detected on the right! Turning left...")
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn left
            return

        if item:
            estimated_distance = 69.0 * float(self.item_to_collect.diameter) ** -0.89
            twist.linear.x = 0.25 * estimated_distance
            twist.angular.z = self.item_to_collect.x / 320.0

            if estimated_distance <= 0.35:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.item_to_collect = None
                self.holding_item = True
                self.pick_up_item_action()
                self.get_logger().info("Item picked up!")

        else:
            # TODO: Fix zone
            estimated_distance = 69.0 * float(self.target_zone.size) ** -0.89
            twist.linear.x = 0.00025 * estimated_distance
            twist.linear.x = 0.30
            twist.angular.z = self.target_zone.x / 320.0

            self.get_logger().warn(f"Estimated distance: {estimated_distance}")

            if estimated_distance <= 70:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.target_zone = None
                self.offload_item_action()
                self.get_logger().info("Item offloaded!")

        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        if not self.holding_item:
            if self.item_to_collect:
                self.navigate_to_item()
            else:
                # TODO: Search for items
                self.get_logger().info("Searching for items...")
                self.search_for_items()
        else:
            if self.target_zone:
                self.navigate_to_zone()
            else:
                # TODO: Search for Zone
                self.get_logger().info("Searching for zones...")
                self.search_for_zones()

    ##################################
    # Navigation
    ##################################
    def navigate_to_item(self):
        if self.item_to_collect:
            self.get_logger().info(f"Navigating to item: {self.item_to_collect}")
            # TODO: Logic to navigate to item
            self.controller(item=True)
            # self.pick_up_item_action()

    def navigate_to_zone(self):
        if self.target_zone:
            self.get_logger().info(f"Navigating to zone: {self.target_zone}")
            # TODO: Move the robot to the target location
            self.controller(item=False)
            # self.offload_item_action()

    ##################################
    # Searching
    ##################################
    def search_for_items(self):
        """Logic to search for items."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate to scan for items
        self.cmd_vel_pub.publish(twist)

    def search_for_zones(self):
        """Logic to search for items."""
        # TODO: make only one full circle then move to a random location
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate to scan for items
        self.cmd_vel_pub.publish(twist)

    ##################################
    # Item Manager Services
    ##################################
    def pick_up_item_action(self):
        request = ItemRequest.Request()
        request.robot_id = self.robot_name

        future = self.pick_up_item.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            self.get_logger().info("Item successfully picked up!")
            self.holding_item = True
        else:
            self.get_logger().error("Failed to pick up the item.")

    def offload_item_action(self):
        request = ItemRequest.Request()
        request.robot_id = self.robot_name

        future = self.offload_item.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.success:
            self.get_logger().info("Item successfully offloaded!")
            self.holding_item = False
        else:
            self.get_logger().error("Failed to offload the item.")

    ##################################
    # Item Manager
    ##################################
    def item_log(self, msg):
        pass

    def item_holder(self, msg):
        pass

    ##################################
    # Item Sensor
    ##################################
    def items(self, msg):
        # Process visible items
        for item in msg.data:
            # TODO: If holding then goto another of same color
            if item.colour == self.ITEM_COLOR:
                self.item_to_collect = item
                self.get_logger().info(f"Item detected: {item}")
                # raise RuntimeError
                break

    def image_items(self, msg):
        pass

    ##################################
    # Robot Sensor
    ##################################
    def robots(self, msg):
        pass

    def image_robots(self, msg):
        pass

    ##################################
    # Zone Sensor
    ##################################
    def zone(self, msg):
        if not self.holding_item:
            return

        # Process visible zones
        for zone in msg.data:
            # TODO: Set target zone
            if self.holding_item:
                if zone.zone == self.ZONE_NUMBER:
                    self.target_zone = zone
                    self.get_logger().info(f"Target zone: {zone}")
                    break

    def image_zone(self, msg):
        pass

    ##################################
    # Laser Scanner
    ##################################
    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30]  # 30 to 331 degrees (30 to -30 degrees)
        left_ranges = msg.ranges[31:90]  # 31 to 90 degrees (31 to 90 degrees)
        back_ranges = msg.ranges[91:270]  # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330]  # 271 to 330 degrees (-30 to -91 degrees)

        self.scan_triggered[self.SCAN_FRONT] = min(front_ranges) < self.SCAN_THRESHOLD
        self.scan_triggered[self.SCAN_LEFT] = min(left_ranges) < self.SCAN_THRESHOLD
        self.scan_triggered[self.SCAN_BACK] = min(back_ranges) < self.SCAN_THRESHOLD
        self.scan_triggered[self.SCAN_RIGHT] = min(right_ranges) < self.SCAN_THRESHOLD

    def odom_callback(self, msg):
        """Callback to update robot pose from /odom."""
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
