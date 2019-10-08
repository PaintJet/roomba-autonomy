#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("roomba_transform_publisher")

    # Create a tf listener for the transform from the map frame to the camera frame
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    # 'world' frame assumed to be the base of the collection bin (side by the outer wall,
    # closest to the corner of the competition area)

    # Marker board transform (relative to world - FIXED)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    board_tf = tf2_ros.TransformStamped()
    board_tf.header.stamp = rospy.Time.now()
    board_tf.header.frame_id = "camera"
    board_tf.child_frame_id = "base_footprint"
    board_tf.transform.translation.x = 0
    board_tf.transform.translation.y = 0
    board_tf.transform.translation.z = 0
    board_tf.transform.rotation.x = 0.5
    board_tf.transform.rotation.y = -0.5
    board_tf.transform.rotation.z = 0.5
    board_tf.transform.rotation.w = 0.5
    broadcaster.sendTransform(board_tf)

    # (OTHER KINECT FRAMES AUTOMATICALLY PUBLISHED RELATIVE TO KINECT BASE)

    rospy.spin()