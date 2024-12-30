import sys
import random
import math
import numpy as np
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from auro_interfaces.srv import ItemRequest
from assessment_interfaces.msg import ItemHolders, ItemLog, ItemList, RobotList, ZoneList


class RobotController(Node):

    def __init__(self, robot_name='robot1', zone_number=1, item_color='RED'):
        super().__init__('robot_controller')

        self.declare_parameter('robot_name', robot_name)
        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        self.other_robots_data = []
        self.other_robots_positions = {}
        self.lock = Lock()

        self.COLLISION_THRESHOLD = 10

        self.scan_triggered = [False] * 4
        self.SCAN_THRESHOLD = 0.6
        self.SCAN_FRONT = 0
        self.SCAN_LEFT = 1
        self.SCAN_BACK = 2
        self.SCAN_RIGHT = 3

        self.item_to_collect = None
        self.holding_item = False
        self.target_zone = None
        self.ZONE_NUMBER = zone_number
        self.ITEM_COLOR = item_color

        item_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer_period = 0.1  # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.robot_name}/cmd_vel', 10)

        # Odom subscriber
        self.create_subscription(Odometry, f'/{self.robot_name}/odom',
                                 self.odom_callback, 10, callback_group=timer_callback_group)

        # Odom subscribers for other robots
        other_robot_names = [f'robot{i}' for i in range(1, 4) if f'robot{i}' != self.robot_name]

        for other_robot_name in other_robot_names:
            self.create_subscription(Odometry, f'/{other_robot_name}/odom', lambda msg: self.other_robot_odom_callback(
                msg=msg, robot_name=other_robot_name), 10, callback_group=timer_callback_group)

        # Item Manager
        self.create_subscription(ItemLog, '/item_log', self.item_log, 100, callback_group=timer_callback_group)
        self.create_subscription(ItemHolders, '/item_holders', self.item_holder,
                                 100, callback_group=timer_callback_group)

        # Item Manager Services
        # self.pick_up_item = self.create_client(ItemRequest, '/pick_up_item', callback_group=timer_callback_group)
        self.pick_up_item = self.create_client(ItemRequest, '/pick_up_item', callback_group=item_callback_group)
        while not self.pick_up_item.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pick up service not available, waiting again...')

        # self.offload_item = self.create_client(ItemRequest, '/offload_item', callback_group=timer_callback_group)
        self.offload_item = self.create_client(ItemRequest, '/offload_item', callback_group=item_callback_group)
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

    ###########################################################################
    # Controller
    ###########################################################################
    # def avoid_collision_with_robots(self):
    #     twist = Twist()
    #     for robot in self.other_robots_data:
    #         if robot.size > 0.10:
    #             twist.linear.x = -0.3
    #             twist.angular.z = -1 * robot.x / 320.0
    #
    #             self.cmd_vel_pub.publish(twist)
    #             return True
    #     return False

    def avoid_collision_with_robots(self):
        twist = Twist()
        for robot in self.other_robots_data:
            distance = math.sqrt(robot.x**2 + robot.y**2)
            self.get_logger().warn(f"Robot: {robot}\nDistance: {distance}")

            # if distance < self.COLLISION_THRESHOLD:
            if abs(robot.x) < self.COLLISION_THRESHOLD and abs(robot.y) < self.COLLISION_THRESHOLD:
                twist.linear.x = -0.3
                twist.angular.z = -1 * robot.x / 320.0

                self.cmd_vel_pub.publish(twist)
                return True
        return False

    # def avoid_collision_with_robots(self):
    #     twist = Twist()
    #     with self.lock:
    #         # Initialize repulsion vector
    #         repulsion_x = 0.0
    #         repulsion_y = 0.0
    #
    #         for robot_name, (other_x, other_y) in self.other_robots_positions.items():
    #             distance = math.sqrt((self.x - other_x)**2 + (self.y - other_y)**2)
    #
    #             self.get_logger().warn(f"Distance to {robot_name}: {distance}")
    #             if distance < self.COLLISION_THRESHOLD:
    #                 self.get_logger().warn(f"Potential collision with {robot_name}! Moving away...")
    #
    #                 # Compute direction vector away from the other robot
    #                 delta_x = self.x - other_x
    #                 delta_y = self.y - other_y
    #
    #                 # Normalize and add to the repulsion vector
    #                 if distance > 0:  # Avoid division by zero
    #                     repulsion_x += delta_x / distance
    #                     repulsion_y += delta_y / distance
    #
    #         # If repulsion vector is non-zero, apply it to the twist
    #         if repulsion_x != 0.0 or repulsion_y != 0.0:
    #             # Compute the angle to move away
    #             repulsion_angle = math.atan2(repulsion_y, repulsion_x)
    #
    #             # Update the twist to move away
    #             twist.linear.x = -1.1  # Move backward slowly
    #             twist.angular.z = 0.5 * np.sign(repulsion_angle)  # Rotate away from the repulsion direction
    #
    #             self.get_logger().warn(f"Repulsion vector: ({repulsion_x}, {repulsion_y})")
    #             return True
    #
    #     return False

    def controller(self, item=False, random_motion=False):
        twist = Twist()

        if item:
            twist = self.navigate_to_item(twist)
        else:
            twist = self.navigate_to_zone(twist)

        # Avoid obstacles dynamically
        if self.scan_triggered[self.SCAN_FRONT]:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif self.scan_triggered[self.SCAN_LEFT]:
            twist.linear.x = 0.5
            twist.angular.z = -0.5
        elif self.scan_triggered[self.SCAN_RIGHT]:
            twist.linear.x = 0.5
            twist.angular.z = 0.5

        if not self.avoid_collision_with_robots():
            self.cmd_vel_pub.publish(twist)

            if random_motion:
                # Move around to reposition the robot
                if self.reposition_timer < 20:
                    twist.linear.x = 0.15
                    twist.angular.z = random.choice([-0.3, 0.3])
                    self.cmd_vel_pub.publish(twist)
                    self.reposition_timer += 1
                else:
                    self.moving_to_reposition = False

    # NOTE: The below function is for debugging purposes only
    def rotate_and_move_forward(self, sign):
        if not hasattr(self, 'rotate_and_move_forward_angle'):
            self.rotate_and_move_forward_angle = 0.0
            self.rotate_and_move_forward_moving = False

        twist = Twist()
        if not self.rotate_and_move_forward_moving:
            twist.linear.x = 0.0
            twist.angular.z = 0.4 if sign else -0.4
            self.cmd_vel_pub.publish(twist)

            self.rotate_and_move_forward_angle += abs(twist.angular.z) * self.timer_period

            # If a full 360-degree rotation is completed
            if self.rotate_and_move_forward_angle >= math.pi:
                self.rotate_and_move_forward_angle = 0.0
                self.rotate_and_move_forward_moving = True
                self.get_logger().info("Moving forward...")

        else:
            twist.linear.x = 0.30
            twist.angular.z = 0.0
            if not self.avoid_collision_with_robots():
                self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        # NOTE: For debug purposes
        # if self.robot_name == "robot2":
        #     self.rotate_and_move_forward(False)
        #     return
        # elif self.robot_name == "robot3":
        #     self.rotate_and_move_forward(True)
        #     return
        # else:
        #     return

        if not self.holding_item:
            if self.item_to_collect:
                self.get_logger().info(f"Navigating to item: {self.item_to_collect}")
                self.controller(item=True)
            else:
                self.get_logger().info("Searching for items...")
                self.search_for_items()
        else:
            if self.target_zone:
                self.get_logger().info(f"Navigating to zone: {self.target_zone}")
                self.controller(item=False)
            else:
                self.get_logger().info("Searching for zones...")
                self.search_for_zones()

    ###########################################################################
    # Navigation
    ###########################################################################
    def navigate_to_item(self, twist):
        if not self.item_to_collect:
            return twist

        estimated_distance = 69.0 * float(self.item_to_collect.diameter) ** -0.89
        # twist.linear.x = 0.25 * estimated_distance
        twist.linear.x = 0.30
        twist.angular.z = self.item_to_collect.x / 320.0

        if estimated_distance <= 0.35:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.item_to_collect = None
            self.holding_item = True
            self.get_logger().info("Item picked up!")
            self.pick_up_item_action()

        return twist

    def navigate_to_zone(self, twist):
        if not self.target_zone:
            return twist

        twist.linear.x = 0.30
        twist.angular.z = self.target_zone.x / 320.0

        if self.target_zone.size >= 0.995:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.target_zone = None
            self.holding_item = False
            self.get_logger().info("Item offloaded!")
            self.offload_item_action()

        return twist

    ###########################################################################
    # Searching
    ###########################################################################
    def search_for_items(self):
        if not hasattr(self, 'item_search_rotation'):
            self.item_search_rotation = 0.0
            self.moving_to_reposition = False
            self.reposition_timer = 0

        twist = Twist()
        if not self.moving_to_reposition:
            twist.linear.x = 0.0
            twist.angular.z = 0.4
            self.cmd_vel_pub.publish(twist)

            self.item_search_rotation += abs(twist.angular.z) * self.timer_period

            # If a full 360-degree rotation is completed
            if self.item_search_rotation >= 2 * math.pi:
                self.item_search_rotation = 0.0
                self.moving_to_reposition = True
                self.reposition_timer = 0
                self.get_logger().info("Item not found after 360-degree search. Moving to reposition...")

        else:
            self.controller(random_motion=True)

    def search_for_zones(self):
        if not hasattr(self, 'zone_search_rotation'):
            self.zone_search_rotation = 0.0
            self.moving_to_reposition = False
            self.reposition_timer = 0

        twist = Twist()
        if not self.moving_to_reposition:
            twist.linear.x = 0.0
            twist.angular.z = 0.4
            self.cmd_vel_pub.publish(twist)

            self.zone_search_rotation += abs(twist.angular.z) * self.timer_period

            # If a full 360-degree rotation is completed
            if self.zone_search_rotation >= 2 * math.pi:
                self.zone_search_rotation = 0.0
                self.moving_to_reposition = True
                self.reposition_timer = 0
                self.get_logger().info("Zone not found after 360-degree search. Moving to reposition...")

        else:
            self.controller(random_motion=True)

    ###########################################################################
    # Item Manager Services
    ###########################################################################
    def pick_up_item_action(self):
        request = ItemRequest.Request()
        request.robot_id = self.robot_name

        future = self.pick_up_item.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            self.get_logger().info("Item successfully picked up!")
            self.holding_item = True
            self.item_to_collect = None
            self.search_for_zones()
        else:
            self.get_logger().error(f"Failed to pick up the item. {response}")
            self.search_for_items()

    def offload_item_action(self):
        request = ItemRequest.Request()
        request.robot_id = self.robot_name

        future = self.offload_item.call_async(request)
        self.executor.spin_until_future_complete(future)
        response = future.result()

        if response.success:
            self.get_logger().info("Item successfully offloaded!")
            self.holding_item = False
            self.target_zone = None
            self.search_for_items()
        else:
            self.get_logger().error(f"Failed to offload the item. {response}")
            # self.search_for_zones()

    ###########################################################################
    # Item Manager
    ###########################################################################
    def item_log(self, msg):
        pass

    def item_holder(self, msg):
        pass

    ###########################################################################
    # Item Sensor
    ###########################################################################
    def items(self, msg):
        if self.holding_item:
            return

        # Process visible items
        for item in msg.data:
            # TODO: If holding then goto another of same color
            if item.colour == self.ITEM_COLOR:
                self.item_to_collect = item
                # self.get_logger().info(f"Item detected: {item}")
                break

    def image_items(self, msg):
        pass

    ###########################################################################
    # Robot Sensor
    ###########################################################################
    def robots(self, msg):
        self.other_robots_data = msg.data

    def image_robots(self, msg):
        pass

    ###########################################################################
    # Zone Sensor
    ###########################################################################
    def zone(self, msg):
        # Process visible zones
        for zone in msg.data:
            if self.holding_item:
                if zone.zone == self.ZONE_NUMBER:
                    self.target_zone = zone
                    # self.get_logger().info(f"Target zone: {zone}")
                    break

    def image_zone(self, msg):
        pass

    ###########################################################################
    # Laser Scanner
    ###########################################################################
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

    ###########################################################################
    # Odometry
    ###########################################################################
    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def other_robot_odom_callback(self, msg, robot_name):
        pose = msg.pose.pose
        other_robot_x = pose.position.x
        other_robot_y = pose.position.y

        with self.lock:
            self.other_robots_positions[robot_name] = (other_robot_x, other_robot_y)

    def destroy_node(self):
        super().destroy_node()


###############################################################################
# Main
###############################################################################
def main(robot_name='robot1', zone_number=1, item_color='RED', args=None):

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = RobotController(robot_name, zone_number, item_color)
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

###############################################################################
# Controller Entry Points
###############################################################################
def controller0(args=None):
    main('robot1', 1, 'RED', args)

def controller1(args=None):
    main('robot2', 2, 'GREEN', args)

def controller2(args=None):
    main('robot3', 3, 'BLUE', args)


if __name__ == '__main__':
    main()
