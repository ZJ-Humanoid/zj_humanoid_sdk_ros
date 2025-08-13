import rospy
from navi_types.srv import Uplimb_MoveJRequest,Uplimb_MoveJ
if __name__=="__main__":
    rospy.init_node("neck_movej_service_test_node")
    client = rospy.ServiceProxy("neck_movej_service",Uplimb_MoveJ)
    client.wait_for_service()
    req = Uplimb_MoveJRequest()
    req.jnt_angle = [0.3,0]
    req.not_wait = False
    res = client.call(req)

    rospy.loginfo(f"result:{res.finish}")

    #-0.174 0.349  


