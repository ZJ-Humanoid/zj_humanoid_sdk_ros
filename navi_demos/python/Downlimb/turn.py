#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 自启后，语音调用
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys # Import sys to access command-line arguments
import time

class AutoController:
    def __init__(self):
        # Generate a unique node name by appending the current timestamp
        unique_node_name = f'auto_controller_{int(time.time())}'
        rospy.init_node(unique_node_name, anonymous=True)

        # 初始化发布者
        self.gmp_pub = rospy.Publisher('/load_gmp_controller', Float32, queue_size=1)
        self.cmd_web_vel_pub = rospy.Publisher('/cmd_web_vel', Twist, queue_size=1)
        rospy.loginfo(f"{unique_node_name} initialized.")

    def run(self, direction, target_distance_param): # Added direction parameter
        # 创建消息 (still needed for gmp_pub)
        float_msg = Float32()
        float_msg.data = 2.0  # 与joy.yaml中的scale值保持一致

        rospy.loginfo("Waiting briefly after startup...")
        rospy.sleep(2.0) # Increased initial delay slightly, can be adjusted

        # --- Automatically enter GMP controller mode ---
        rospy.loginfo("Attempting to enter GMP controller mode automatically...")
        self.gmp_pub.publish(float_msg)
        rospy.loginfo("GMP Controller command sent. Waiting for transition...")
        rospy.sleep(1.0)  # 等待1秒让机器人准备就绪 (Adjust as needed)
        rospy.loginfo("Assumed GMP controller mode is active.")
        # --- End of automatic GMP entry ---

        # 控制机器人移动
        cmd_web_vel = Twist()
        if direction == 0:
            cmd_web_vel.linear.x = 0.0
            cmd_web_vel.angular.z = 0.5  # 左转
        elif direction == 1:
            cmd_web_vel.linear.x = 0.0
            cmd_web_vel.angular.z = -0.5  # 右转
        else:
            rospy.logwarn(f"Invalid direction provided ({direction}). Must be 0 (left) or 1 (right). Aborting movement.")
            return

        start_time = rospy.get_time()
        angle_rotated = 0.0
        angular_speed = 0.27
        rate = rospy.Rate(10)  # 10Hz的控制频率

        if target_distance_param <= 0:
            rospy.logwarn(f"Target distance ({target_distance_param}m) must be positive. Aborting movement.")
            return

        print(f"开始旋转 {target_distance_param} 弧度...")

        # Loop until the estimated angle is reached or ROS shuts down
        while angle_rotated < target_distance_param and not rospy.is_shutdown():
            self.cmd_web_vel_pub.publish(cmd_web_vel)
            elapsed_time = rospy.get_time() - start_time
            angle_rotated = angular_speed * elapsed_time # Estimate angle based on time and speed
            rate.sleep()

        # Ensure the loop didn't exit due to shutdown before stopping
        if not rospy.is_shutdown():
            # 停止机器人
            cmd_web_vel.angular.z = 0.0
            self.cmd_web_vel_pub.publish(cmd_web_vel)
            rospy.loginfo("Stop command sent.")
            # Publish stop command a few times to ensure it's received
            for _ in range(5):
                if rospy.is_shutdown(): break
                self.cmd_web_vel_pub.publish(cmd_web_vel)
                rospy.sleep(0.1)

            print(f"机器人已完成约 {angle_rotated:.2f} 弧度旋转任务并停止") # Used angle_rotated
        else:
            print("ROS shutdown requested during movement.")


if __name__ == '__main__':
    controller = None
    DEFAULT_DIRECTION = 0
    DEFAULT_DISTANCE = 1.60
    direction_main = DEFAULT_DIRECTION
    target_distance_main = DEFAULT_DISTANCE

    if len(sys.argv) > 2:
        try:
            arg_direction = int(sys.argv[1])
            if arg_direction in [0, 1]:
                direction_main = arg_direction
                print(f"Using provided direction: {'Left' if direction_main == 0 else 'Right'}.")
            else:
                print(f"Invalid direction provided ({sys.argv[1]}). It must be 0 (left) or 1 (right). Using default: {'Left' if DEFAULT_DIRECTION == 0 else 'Right'}.")

            arg_dist = float(sys.argv[2])
            if arg_dist > 0:
                target_distance_main = arg_dist
                print(f"Using provided target distance: {target_distance_main} meters.")
            else:
                print(f"Invalid distance provided ({sys.argv[2]}). It must be a positive number. Using default: {DEFAULT_DISTANCE} meters.")
        except ValueError:
            print(f"Invalid argument(s) provided. Usage: {sys.argv[0]} [direction (0/1)] [distance_in_meters]")
    else:
        print(f"Not enough arguments provided. Using defaults: direction={'Left' if DEFAULT_DIRECTION == 0 else 'Right'}, distance={DEFAULT_DISTANCE} meters.")
        print(f"Usage: {sys.argv[0]} [direction (0/1)] [distance_in_meters]")


    try:
        rospy.sleep(0.5) # Give ROS master/core services a moment
        controller = AutoController()
        controller.run(direction_main, target_distance_main) # Pass the determined direction and distance to run()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception caught. Exiting.")
        pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught. Exiting.")
        if controller and hasattr(controller, 'cmd_web_vel_pub') and controller.cmd_web_vel_pub is not None:
            try:
                rospy.logwarn("Attempting to stop robot due to KeyboardInterrupt...")
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                for _ in range(5):
                    if rospy.is_shutdown(): break
                    controller.cmd_web_vel_pub.publish(stop_cmd)
                    rospy.sleep(0.1)
                print("Sent stop command due to KeyboardInterrupt.")
            except Exception as e:
                print(f"Could not send stop command on exit: {e}")
        else:
            try:
                if not rospy.is_shutdown():
                    stop_pub = rospy.Publisher('/cmd_web_vel', Twist, queue_size=1, latch=True)
                    stop_cmd = Twist()
                    stop_pub.publish(stop_cmd)
                    rospy.sleep(0.5)
                    print("Sent stop command (fallback) due to KeyboardInterrupt.")
            except Exception as e:
                print(f"Could not send stop command (fallback) on exit: {e}")
        pass



