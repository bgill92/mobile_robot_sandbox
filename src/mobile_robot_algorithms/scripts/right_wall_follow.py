#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy

class RightWallFollow(Node):
    """! Performs a right wall follow algorithm given a laserscan topic.

    Subscribes to a LaserScan topic and publishes twist commands based on the laserscan readings
    """

    def __init__(self, scan_topic):
        """! Initialize LaserScan subscriber and Twist publisher.

        @param scan_topic topic name to use for JointState publishing

        @return Instance of the RightWallFollow node
        """

        super().__init__("right_wall_follow")

        self.scan_sub_ = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 1
        )
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/diff_controller/cmd_vel_unstamped", 1)

        self.left_spin_flag = False

        self.right_turn_start_time = None

    def scan_callback(self, laserscan_msg):
        """! Subscription callback to run for incoming LaserScan messages

        """

        # speed_scale_factor = 2.5

        FWD_SPD = 0.5
        RT_TRN_SPD = -3.0
        LFT_TRN_SPD = 0.75
        LFT_SPN_SPD = 5.0

        LFT_SPN_DURATION = 1.0
        RT_TRN_MV_FWD_DURATION = 0.6

        FWD_LSR_IDX = 0
        RT_LSR_IDX = 630

        RT_LSR_MIN_BND = 0.27
        RT_LSR_MAX_BND = 0.31

        FWD_LSR_MIN_BND = 0.22

        rt_lsr_rdng = laserscan_msg.ranges[RT_LSR_IDX]
        fwd_lsr_rdng = laserscan_msg.ranges[FWD_LSR_IDX]

        cmd_vel = Twist()

        if self.left_spin_flag:            
            if (rt_lsr_rdng < RT_LSR_MAX_BND):
                print("Continue Spin left", file=sys.stderr)
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = LFT_SPN_SPD
                self.cmd_vel_pub_.publish(cmd_vel)
                return
            else:
                print("Left Spin complete", file=sys.stderr)
                self.left_spin_flag = False

        if self.right_turn_start_time is not None:
            if (fwd_lsr_rdng > FWD_LSR_MIN_BND):
                if (time.time() - self.right_turn_start_time) < RT_TRN_MV_FWD_DURATION:
                    print("Continue move forward portion of right turn", file=sys.stderr)
                    cmd_vel.linear.x = FWD_SPD
                    self.cmd_vel_pub_.publish(cmd_vel)
                    return
                else:
                    if (rt_lsr_rdng > RT_LSR_MAX_BND):
                        print("Actual right turn", file=sys.stderr)
                        cmd_vel.linear.x = FWD_SPD
                        cmd_vel.angular.z = RT_TRN_SPD
                        self.cmd_vel_pub_.publish(cmd_vel)
                        return
                    else:
                        print("Right turn done", file=sys.stderr)
                        self.right_turn_start_time = None
            else:
                print("Cancelling right turn, spinning left", file=sys.stderr)
                self.right_turn_start_time = None
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = LFT_SPN_SPD
                self.left_spin_flag = time.time()
                self.cmd_vel_pub_.publish(cmd_vel)
                return

        if (rt_lsr_rdng > RT_LSR_MIN_BND) and (rt_lsr_rdng < RT_LSR_MAX_BND):
            print("Go Forward", file=sys.stderr)
            cmd_vel.linear.x = FWD_SPD
        elif (rt_lsr_rdng > RT_LSR_MAX_BND):
            print("Turn right, start moving forward first", file=sys.stderr)
            cmd_vel.linear.x = FWD_SPD
            self.right_turn_start_time = time.time()
            # cmd_vel.angular.z = RT_TRN_SPD
        elif (rt_lsr_rdng < RT_LSR_MIN_BND):
            print("Turn left", file=sys.stderr)
            cmd_vel.linear.x = FWD_SPD
            cmd_vel.angular.z = LFT_TRN_SPD

        if (fwd_lsr_rdng < FWD_LSR_MIN_BND):
            print("Spin left", file=sys.stderr)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = LFT_SPN_SPD
            self.left_spin_flag = True

        self.cmd_vel_pub_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)

    right_wall_follow_node = RightWallFollow("scan")

    rclpy.spin(right_wall_follow_node)

    right_wall_follow_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()