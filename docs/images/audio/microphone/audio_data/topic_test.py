#!/usr/bin/env python3
"""
ROS Topic Test Script

Topic: /zj_humanoid/audio/audio_data
Type: audio/AudioData
Description: 音频流数据

Usage:
    python3 topic_test.py <data.yaml>

This script publishes the message data from a YAML file to the topic at 10Hz.
Press Ctrl+C to stop.
"""

import rospy
import yaml
import sys
from audio.msg import AudioData
from std_msgs.msg import Header


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
    
    # Handle header
    if 'header' in data:
        header = Header()
        header_data = data['header']
        if 'seq' in header_data:
            header.seq = header_data['seq']
        if 'stamp' in header_data:
            stamp = header_data['stamp']
            if 'secs' in stamp:
                header.stamp.secs = stamp['secs']
            if 'nsecs' in stamp:
                header.stamp.nsecs = stamp['nsecs']
        if 'frame_id' in header_data:
            header.frame_id = header_data['frame_id']
        msg.header = header
    
    # Handle other fields
    for key, value in data.items():
        if key != 'header' and hasattr(msg, key):
            setattr(msg, key, value)
    
    return msg


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('topic_test_zj_humanoid_audio_audio_data', anonymous=True)
    
    # Load data from YAML
    data = load_yaml_data(yaml_file)
    
    # Create publisher
    pub = rospy.Publisher('/zj_humanoid/audio/audio_data', AudioData, queue_size=10)
    
    # Wait for subscribers
    rospy.sleep(1.0)
    
    # Create message
    msg = dict_to_msg(data, AudioData)
    
    # Set header timestamp if not set
    if msg.header.stamp.secs == 0 and msg.header.stamp.nsecs == 0:
        msg.header.stamp = rospy.Time.now()
    
    # Publish at 10Hz
    rate = rospy.Rate(10)
    
    rospy.loginfo(f"Publishing to /zj_humanoid/audio/audio_data at 10Hz...")
    rospy.loginfo(f"Message type: audio/AudioData")
    rospy.loginfo(f"Channel: {msg.channel}, VAD: {msg.vad_status}, Final: {msg.is_final}")
    
    while not rospy.is_shutdown():
        # Update timestamp
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

