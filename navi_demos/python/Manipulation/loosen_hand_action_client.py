#!/usr/bin/env python3
import rospy
import actionlib
from naviai_manip_actions.msg import LoosenHandAction, LoosenHandGoal


class LoosenHandClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('loosen_hand_action', LoosenHandAction)
        self.count = 0
        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started!")

        # 设置条件变量以判断是否停止任务
        self.stop_condition_met = False

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Received feedback: {feedback.loosen_hand_flag}")
        # 如果满足某种条件，则停止任务
        if feedback.loosen_hand_flag:
            self.count += 1
            print(self.count)
            if self.count >= 5:
                rospy.loginfo("Condition meet, stopping action...")
                self.stop_condition_met = True
                self.client.cancel_goal()
        else:
            self.count = 0

    def send_goal(self, g):
        goal = LoosenHandGoal()
        goal.task_goal = g
        self.stop_condition_met = False  # 重置条件变量

        # 发送目标，并注册反馈回调
        rospy.loginfo("Sending goal to action server...")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # 等待结果
        rospy.loginfo("Waiting for result...")
        self.client.wait_for_result()

        # 获取结果
        result = self.client.get_result()
        rospy.loginfo(f"Action result: {result.success}")

        return result.success


def main():
    rospy.init_node('loosen_hand_client')
    client = LoosenHandClient()

    # 测试目标标签
    test_goal = 'aaa'

    # 发送目标并处理结果
    success = client.send_goal(test_goal)
    if success:
        rospy.loginfo("Task completed successfully.")
    else:
        rospy.loginfo("Task failed.")


if __name__ == "__main__":
    main()
