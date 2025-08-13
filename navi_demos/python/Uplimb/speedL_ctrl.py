#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
from std_msgs.msg import Float32MultiArray

right = 1
left = 0
v = 0.04  # 初始加速度
vj = 0.1
time_ctrl = 0.1 #控制周期，ms
pub_left = rospy.Publisher('/left_arm_vel_cmd', Float32MultiArray, queue_size=10)
pub_right = rospy.Publisher('/right_arm_vel_cmd', Float32MultiArray, queue_size=10)

def speedL_Ctrl(event):
    global v
    key_pressed = sys.stdin.read(1)
    if key_pressed:
        key = key_pressed
        key_pressed = None  # Reset the key press

        # 创建一个 Float32MultiArray 消息
        msg = Float32MultiArray()

        if key == 'q':
            msg.data = [0,0,v,0,0,0]
            print("Vz+ ",v)
        elif key == 'e':
            msg.data = [0,0,-v,0,0,0]
            print("Vz- ",v)

        elif key == 'a':
            msg.data = [0,v,0,0,0,0]
            print("Vy+ ",v)
        elif key == 'd':
            msg.data = [0,-v,0,0,0,0]
            print("Vy- ",v)

        elif key == 'w':
            msg.data = [v,0,0,0,0,0]
            print("Vx+ ",v)
        elif key == 's':
            msg.data = [-v,0,0,0,0,0]
            print("Vx- ",v)

        elif key == 'j':
            msg.data = [0,0,0,v,0,0]
            print("Wx+ ",v)
        elif key == 'l':
            msg.data = [0,0,0,-v,0,0]
            print("Wx- ",v)

        elif key == 'k':
            msg.data = [0,0,0,0,v,0]
            print("Wy+ ",v)
        elif key == 'i':
            msg.data = [0,0,0,0,-v,0]
            print("Wy- ",v)

        elif key == 'u':
            msg.data = [0,0,0,0,0,v]
            print("Wz+ ",v)
        elif key == 'o':
            msg.data = [0,0,0,0,0,-v]
            print("Wz- ",v)

        elif key == '+':
            v = v + 0.2
        elif key == '-':
            v = v - 0.2

        # pub_right.publish(msg)
        pub_left.publish(msg)

def main():
    # 设置终端为cbreak模式
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # 初始化 ROS 节点
    rospy.init_node('SpeedL_listener', anonymous=True)
    
    # 创建定时器
    rospy.Timer(rospy.Duration(time_ctrl), speedL_Ctrl) 
    
    # 保持节点运行
    rospy.spin()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
