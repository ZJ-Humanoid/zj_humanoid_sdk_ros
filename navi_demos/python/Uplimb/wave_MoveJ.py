#!/usr/bin/env python3
import rospy
from navi_types.srv import Uplimb_MoveJ, Uplimb_MoveJRequest
import time

def move_right_arm(trajectory):
    # Wait for the service to become available
    rospy.wait_for_service('/right_arm_movej_service')
    
    try:
        # Create a client for the service
        client = rospy.ServiceProxy('/right_arm_movej_service', Uplimb_MoveJ)
        
        # Iterate over each point in the trajectory and send a MoveJ request
        for point in trajectory:
            srv_request = Uplimb_MoveJRequest()
            srv_request.jnt_angle = point
            srv_request.not_wait = False
            
            # Call the service and check if it was successful
            response = client(srv_request)
            
            rospy.loginfo(f"Moved to position: {point}. Finish: {response.finish.data}")
            
            # Sleep for a short duration before sending the next request
            time.sleep(0.01)
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call service right_arm_movej_service: {e}")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('right_arm_trajectory_client_node')
    
    # Define a trajectory as a series of joint positions for waving
    trajectory = [
        [-0.9, -1.6, -0.5, -1.5, 0.0, 0.0, 0.0],  # Start position (down)
        [-0.9, -1.6, -0.5, -0.5, 0.0, 0.0, 0.0],  # Start position (down)
        [-0.9, -1.6, -0.5, -1.5, 0.0, 0.0, 0.0],  # Start position (down)
        [-0.9, -1.6, -0.5, -0.5, 0.0, 0.0, 0.0],  # Start position (down)
        [-0.0, -0.33, 0.0, -0.0, 0.0, -0.0, -0.0]
    ]
    
    # Move the right arm along the defined trajectory
    move_right_arm(trajectory)



