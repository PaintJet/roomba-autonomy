import rospy
import tf2_ros
import argparse


# Get distance to marker and angle of marker as input arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--distance", required=True, help="Distance between the camera and the ArUco marker")
ap.add_argument("-a", "--angle", required = True, help="Angle of the marker relative to the camera")
ap.add_argument("-n", "--num-samples", required = False, default = "1000", help = "Number of samples to collect")
args = ap.parse_args()

# Get distance as a float
distance = float(args.distance)

# Get number of samples as integer
num_samples = int(args.num_samples)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("error_benchmarker")

    rate = rospy.Rate(20.0)

    # Instantiate TF listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Instantiate array to store collected data
    error = [None] * num_samples

    i = 0
    while i < num_samples:
        try:
            trans = tfBuffer.lookup_transform('camera', 'aruco_marker', rospy.Time())
            error[i] = trans.transform.translation.z - distance
            i += 1

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Unable to find transform")

        rate.sleep()

    filename = "{}_meters-{}_degrees.csv".format(distance, args.angle)
    with open(filename, "w+") as f:
        f.write("\n".join(map(str, error)))





