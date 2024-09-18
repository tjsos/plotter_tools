#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import visualization_msgs.msg
from tf2_geometry_msgs import do_transform_point
import csv
import os

class MarkerFrameConverter:
    def __init__(self):
        rospy.init_node('marker_frame_converter')

        # Initialize a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber to the input marker
        self.marker_sub = rospy.Subscriber('/alpha_rise/helm/path_3d/segment', visualization_msgs.msg.Marker, self.marker_callback)

        # CSV file path
        self.csv_file_path = 'transformed_points.csv'

        # Ensure the CSV file is created with headers
        self.create_csv_file()

    def create_csv_file(self):
        # Create a CSV file and write headers
        with open(self.csv_file_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['x', 'y', 'z'])  # Header

    def marker_callback(self, msg):
        target_frame = "alpha_rise/odom"  # Replace with your target frame
        transformed_points = []
        rospy.loginfo_once("callback")
        try:
            # Get the transform from the original frame to the target frame
            transform = self.tf_buffer.lookup_transform(target_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Transform points in the marker
            for point in msg.points:
                # Create a point for transformation
                point_stamped = geometry_msgs.msg.PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.header.stamp = rospy.Time(0)
                point_stamped.point = point

                # Transform the point
                transformed_point = do_transform_point(point_stamped, transform)
                transformed_points.append(transformed_point.point)

            # Save the transformed points to the CSV file
            self.save_transformed_points_to_csv(transformed_points)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 Exception: %s", e)

    def save_transformed_points_to_csv(self, points):
        # Append the transformed points to the CSV file
        with open(self.csv_file_path, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            for point in points:
                csv_writer.writerow([point.x, point.y, point.z])

if __name__ == '__main__':
    try:
        converter = MarkerFrameConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass