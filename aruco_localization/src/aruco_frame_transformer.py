#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray


def aruco_pose_callback(fiducial_transform_msg):
    if(fiducial_transform_msg.transforms != []):
        # Create broadcaster
        b = tf2_ros.TransformBroadcaster()

        # Create transform message
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "camera"
        tf_msg.child_frame_id = "aruco_marker"
        tf_msg.transform.translation.x = fiducial_transform_msg.transforms[0].transform.translation.x
        tf_msg.transform.translation.y = fiducial_transform_msg.transforms[0].transform.translation.y
        tf_msg.transform.translation.z = fiducial_transform_msg.transforms[0].transform.translation.z
        tf_msg.transform.rotation.x = fiducial_transform_msg.transforms[0].transform.rotation.x
        tf_msg.transform.rotation.y = fiducial_transform_msg.transforms[0].transform.rotation.y
        tf_msg.transform.rotation.z = fiducial_transform_msg.transforms[0].transform.rotation.z
        tf_msg.transform.rotation.w = fiducial_transform_msg.transforms[0].transform.rotation.w

        # Publish transform
        b.sendTransform(tf_msg)

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transform_publisher")

    # 'world' frame assumed to be the base of the collection bin (side by the outer wall,
    # closest to the corner of the competition area)

    # Marker board transform (relative to world - FIXED)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    board_tf = tf2_ros.TransformStamped()
    board_tf.header.stamp = rospy.Time.now()
    board_tf.header.frame_id = "aruco_marker"
    board_tf.child_frame_id = "map"
    board_tf.transform.translation.x = 0
    board_tf.transform.translation.y = 0
    board_tf.transform.translation.z = 0
    board_tf.transform.rotation.x = -0.7070727
    board_tf.transform.rotation.w = 0.7070727
    broadcaster.sendTransform(board_tf)

    # Aruco marker camera transform (relative to marker board)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, aruco_pose_callback)

    # (OTHER KINECT FRAMES AUTOMATICALLY PUBLISHED RELATIVE TO KINECT BASE)

    rospy.spin()