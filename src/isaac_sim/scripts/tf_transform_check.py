import tf2_ros
import rospy

rospy.init_node("tf_transform_check")
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
while not rospy.is_shutdown():
    check = buffer.can_transform("Carter_0/odom", "Carter_0/carter_lidar", rospy.Time(0))
    if check:
        print("success")
    else:
        print("failure")
    rospy.sleep(2.0)
