import rospy

class Handler:
    rospy.init_node("perception_node")
    

if __name__ == "__main__":
    try:
        Handler()
    except rospy.ROSInterruptException():
        pass