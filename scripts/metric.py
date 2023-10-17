#!/usr/bin/python3

import rospy
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped)
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf
from math import sqrt, fabs
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class PoseComparisonNode(object):
    def __init__(self):
        self._estimated_pose = None
        self._ground_truth_pose = None

        self._ground_truth_subscriber = rospy.Subscriber("/base_pose_ground_truth", Odometry, self._ground_truth_callback)
        self._estimated_pose_subscriber = rospy.Subscriber("/estimatedpose", PoseStamped, self._estimated_pose_callback)
        self._adjusted_ground_truth_publisher = rospy.Publisher("/adjusted_ground_truth", Odometry, queue_size=1)
        self._initial_pose_subscriber = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._initial_pose_callback)
        # self._map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
        
        self.transform = None
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.total_position_error = 0
        self.total_scans = 0
        self.total_orientation_error = 0
        self.results_path = rospy.get_param("/Metrics/results_file", "results.txt")
        self.result_file = open(self.results_path, "w")
        self.result_file.write("TEST 2")

    def _initial_pose_callback(self, pose):
        self.total_scans = 0
        self.total_orientation_error = 0
        self.total_position_error = 0

    # def _map_callback(self, map):
    #     OccupancyGrid().info.origin.position.

    def _ground_truth_callback(self, odom):
        if not (self.transform):
            try:
                print("Getting transform")
                self.transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(1.0))
            except tf2_ros.LookupException as e:
                rospy.logwarn_throttle(1.0, "Transform lookup failed: %s", str(e))
    
        if (self.transform):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time().now()
            pose.pose = odom.pose.pose
            pose = do_transform_pose(pose, self.transform)
            odom.header = pose.header
            odom.pose.pose = pose.pose
        else: return
        ground_truth_pose = odom.pose.pose
        self._ground_truth_pose = {
            "x": ground_truth_pose.position.x,
            "y": ground_truth_pose.position.y,
            "yaw": euler_from_quaternion([
                ground_truth_pose.orientation.x,
                ground_truth_pose.orientation.y,
                ground_truth_pose.orientation.z,
                ground_truth_pose.orientation.w
            ])[2]  # Get yaw angle (orientation) from quaternion
        }
        self._adjusted_ground_truth_publisher.publish(odom)
        self.compare_poses()

    def _estimated_pose_callback(self, pose):
        self._estimated_pose = {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "yaw": euler_from_quaternion([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])[2]  # Get yaw angle (orientation) from quaternion
        }
        self.compare_poses()

    def compare_poses(self):
        if self._ground_truth_pose and self._estimated_pose:
            ground_truth_x = self._ground_truth_pose["x"]
            ground_truth_y = self._ground_truth_pose["y"]
            ground_truth_yaw = self._ground_truth_pose["yaw"]

            estimated_x = self._estimated_pose["x"]
            estimated_y = self._estimated_pose["y"]
            estimated_yaw = self._estimated_pose["yaw"]

            # Calculate Euclidean distance between ground truth and estimated positions
            position_error = sqrt((ground_truth_x - estimated_x) ** 2 + (ground_truth_y - estimated_y) ** 2)

            # Calculate absolute difference in yaw angles
            orientation_error = fabs(ground_truth_yaw - estimated_yaw)
            self.total_position_error += position_error
            self.total_orientation_error += orientation_error
            self.total_scans += 1
            position_mean = self.total_position_error / self.total_scans
            orientation_mean = self.total_orientation_error / self.total_scans

            print("Position Error: {:.2f} (mean {:.2f}) meters".format(position_error, position_mean))
            print("Orientation Error: {:.2f} radians".format(orientation_error, orientation_mean))
            if (self.result_file):
                self.result_file.seek(0)
                self.result_file.truncate()
                self.result_file.write("\n".join([
                    "POS_ERROR_FINAL:{:.4f}".format(position_error),
                    "POS_ERROR_MEAN:{:.4f}".format(position_mean),
                    "POS_ERROR_TOTAL:{:.4f}".format(self.total_position_error),
                    "ORIENTATION_ERROR_FINAL:{:.4f}".format(orientation_error),
                    "ORIENTATION_ERROR_MEAN:{:.4f}".format(orientation_mean),
                    "ORIENTATION_ERROR_TOTAL:{:.4f}".format(self.total_position_error),
                    "CALCULATIONS_TOTAL:{}".format(self.total_scans)
                ]))

if __name__ == '__main__':
    try:
        rospy.init_node("pose_comparison_node")
        node = PoseComparisonNode()

        # Run the comparison logic at a specific frequency (e.g., 1 Hz)
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            node.compare_poses()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass