#!/usr/bin/env python3
"""
ROS Topic Test Script

Topic: /zj_humanoid/audio/asr_text
Type: std_msgs/String
Description: 语音转文字

Usage:
    python3 topic_test.py <data.yaml>

This script publishes the message data from a YAML file to the topic at 10Hz.
Press Ctrl+C to stop.
"""

import rospy
import yaml
import sys
from std_msgs.msg import String


def load_yaml_data(yaml_file):
    """Load message data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)


def dict_to_msg(data, msg_class):
    """Convert dictionary to ROS message."""
    msg = msg_class()
    for key, value in data.items():
        if hasattr(msg, key):
            setattr(msg, key, value)
    return msg


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('topic_test_zj_humanoid_audio_asr_text', anonymous=True)
    
    # Load data from YAML
    data = load_yaml_data(yaml_file)
    
    # Create publisher
    pub = rospy.Publisher('/zj_humanoid/audio/asr_text', String, queue_size=10)
    
    # Wait for subscribers
    rospy.sleep(1.0)
    
    # Create message
    msg = String()
    msg.data = data.get('data', '')
    
    # Publish at 10Hz
    rate = rospy.Rate(10)
    
    rospy.loginfo(f"Publishing to /zj_humanoid/audio/asr_text at 10Hz...")
    rospy.loginfo(f"Message type: std_msgs/String")
    rospy.loginfo(f"Data: {msg.data}")
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
