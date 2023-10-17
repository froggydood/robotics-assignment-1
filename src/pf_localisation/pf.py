from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from . pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from . util import rotateQuaternion, getHeading
from random import Random
import time

# Author: Group 22
class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = rospy.get_param("/PF_Localizer/num_particles", 80) # Number of readings to predict
        self.RANDOM_PARTICLE_MULTIPLIER = 20                      # The multiplier for readings when generating the random particle grid when robot is lost
        self.LOST_PARTICLE_MULTIPLIER = 5

        # ----- Error noise distribution parameters
        self.GAUSSIAN_POSITION_ERROR_STANDARD_DEVIATION = 0.1     # Standard deviation of the normal distribution for position error
        self.GAUSSIAN_POSITION_ERROR_STANDARD_MEAN = 0            # Mean of the normal distribution for position error

        self.GAUSSIAN_ORIENTATION_ERROR_STANDARD_DEVIATION = 0.1  # Standard deviation of the normal distribution for orientation error
        self.GAUSSIAN_ORIENTATION_ERROR_STANDARD_MEAN = 0         # Mean of the normal distribution for orientation error
        self.LOST_MEAN_SENSOR_THRESHOLD = rospy.get_param("/PF_Localizer/lost_sensor_threshold", 6)                       # The threshold for the mean sensor weights to decide if the robot is lost
        self.FOUND_MEAN_SENSOR_THRESHOLD = rospy.get_param("/PF_Localizer/found_sensor_threshold", 10)
        self.LOST_TIME_RESET = 1000                                 # The max in seconds time a particle can be lost for without regenerating the grid
        self.LOST_ERROR_MULTIPLIER = 2
        self.LOST_MULTIPLIER_DECAY = 0.95

        self.FRACTION_RANDOM_PARTICLES = 0.1
        self.LOST_FRACTION_RANDOM_PARTICLES_MULTIPLIER = 0

        self._time_lost_at = 0
        self._lost_behaviour = False
        self._random_map_part = False
        self._unoccupied_coords = []
        self.map_publisher = rospy.Publisher("/unoccupied_map", OccupancyGrid)

    # Overwrite the default set_map function to regenerate the unoccupied coords
    def set_map(self, occupancy_map):
        """ Set the map for localisation """
        self.occupancy_map = occupancy_map
        self.sensor_model.set_map(occupancy_map)
        # ----- Map has changed, so we should reinitialise the particle cloud
        self.particlecloud = self.initialise_particle_cloud(self.estimatedpose)
        self.particlecloud.header.frame_id = "map"
        self.generate_unoccupied_coords()

    def generate_unoccupied_coords(self):
        rospy.loginfo("Getting unoccupied map")
        self._unoccupied_coords = []
        width = self.occupancy_map.info.width
        resolution = self.occupancy_map.info.resolution
        rospy.loginfo(self.occupancy_map.info.width)
        test_map = []
        for i in range(len(self.occupancy_map.data)):
            data_item = self.occupancy_map.data[i]
            if (data_item == 0):
                test_map.append(0)
                self._unoccupied_coords.append([
                    (i % width) * resolution + self.occupancy_map.info.origin.position.x,
                    math.floor(i / width) * resolution + self.occupancy_map.info.origin.position.x
                ])
            else:
                test_map.append(-1)
        grid = OccupancyGrid()
        grid.info = self.occupancy_map.info
        grid.data = test_map
        print("Published")
        self.map_publisher.publish(grid)

    def sample_error_position(self):
        return np.random.normal(
            self.GAUSSIAN_POSITION_ERROR_STANDARD_MEAN,
            self.GAUSSIAN_POSITION_ERROR_STANDARD_DEVIATION * (self.LOST_ERROR_MULTIPLIER if self._lost_behaviour else 1)
        )
    
    def sample_error_orientation(self):
        return np.random.normal(
            self.GAUSSIAN_ORIENTATION_ERROR_STANDARD_MEAN,
            self.GAUSSIAN_ORIENTATION_ERROR_STANDARD_DEVIATION * (self.LOST_ERROR_MULTIPLIER if self._lost_behaviour else 1)
        )

    def apply_noise_to_pose(self, pose):
        noiseX = self.sample_error_position()
        noiseY = self.sample_error_position()
        noiseOrientation = self.sample_error_orientation()
        newPose = Pose(
            Point(
                pose.position.x + noiseX,
                pose.position.y + noiseY,
                pose.position.z
            ),
            rotateQuaternion(pose.orientation, noiseOrientation)
        )

        return newPose
    
    def generate_pose_noise_array(self, pose):
        newPoseArray = PoseArray()

        for i in range(self.NUMBER_PREDICTED_READINGS):
            newPose = self.apply_noise_to_pose(pose)
            newPoseArray.poses.append(newPose)

        return newPoseArray
    
    def generate_particle_grid(self, num_particles_out):
        newPoseArray = PoseArray()

        numOrientations = 6
        num_particles_out = math.floor(num_particles_out / numOrientations)
        step = math.floor((len(self._unoccupied_coords) - 1) / num_particles_out)

        defaultOrientation = Quaternion(0, 0, 0, 1)


        for i in range(num_particles_out):
            rotation = rotateQuaternion(defaultOrientation, np.random.uniform(-np.pi, np.pi))
            for j in range(numOrientations):
                noiseX = self.sample_error_position()
                noiseY = self.sample_error_position()
                cell = self._unoccupied_coords[i * step]
                newPose = Pose(
                    Point(
                        cell[0] + noiseX,
                        cell[1] + noiseY,
                        0
                    ),
                    rotateQuaternion(rotation, 2 * np.pi * i / numOrientations)
                )
                newPoseArray.poses.append(newPose)

        return newPoseArray
    
    def initialise_particle_cloud(self, initialPose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

        newPoseArray = self.generate_pose_noise_array(initialPose.pose.pose)
        return newPoseArray

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """


        """
        Steps:

        1. Loop through particles
        2. Loop through all map ranges
        3. Calculate the probability of each range, from the particle pose
        4. Get the product of all probabilities
        5. Use as a weight for each particle
        6. Resample all particles based on weight
        7. Add noise to each particle to 
        """

        # Get weights for each particle
        total = 0
        weights = []
        for pose in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, pose)
            total += weight
            weights.append(weight)
        
        _mean = total / len(weights)
        num_particles_out = self.NUMBER_PREDICTED_READINGS

        # if (self._lost_behaviour):
        #     print((time.time() - self._time_lost_at), self.LOST_TIME_RESET)
        #     print("Mean", _mean)

        if (not self._lost_behaviour and _mean <= self.LOST_MEAN_SENSOR_THRESHOLD) or (self._lost_behaviour and time.time() - self._time_lost_at > self.LOST_TIME_RESET):
            rospy.loginfo("Now lost")
            self._time_lost_at = time.time()
            self._lost_behaviour = True
            # self._random_map_part = True
            self._current_lost_multiplier = self.LOST_PARTICLE_MULTIPLIER

        if self._lost_behaviour == True:
            if self._random_map_part == True:
                self.particlecloud = self.generate_particle_grid(self.NUMBER_PREDICTED_READINGS * self.RANDOM_PARTICLE_MULTIPLIER)
                self._random_map_part = False
                return
            else:
                num_particles_out = math.floor(num_particles_out * self._current_lost_multiplier)
                self._current_lost_multiplier = self._current_lost_multiplier * self.LOST_MULTIPLIER_DECAY
                if (self._current_lost_multiplier < 1): self._current_lost_multiplier = 1

        if self._lost_behaviour and _mean >= self.FOUND_MEAN_SENSOR_THRESHOLD:
            self._lost_behaviour = False
            rospy.loginfo("Now found")

        # 1. Decide if is_lost
        # 2. Scatter a bunch of particles
        # 3. Try and figure out when found
        # 4. Try and figure out when lost again (rescatter)
        # 5. Set is_lost accordingly

        # Normalize weights
        for i in range(len(weights)):
            weights[i] = weights[i] / total

        # Resample
        new_cloud = PoseArray()
        new_cloud.poses = self.systematic_resampling(weights, num_particles_out)

        # Introduce extra 7% random particles
        num_random_particles = int(self.FRACTION_RANDOM_PARTICLES * (self.LOST_FRACTION_RANDOM_PARTICLES_MULTIPLIER if self._lost_behaviour else 1) * num_particles_out)
        random_particles = self.generate_random_particles(num_random_particles)
        new_cloud.poses.extend(random_particles)
        
        self.particlecloud = new_cloud

    def systematic_resampling(self, weights, num_particles_out):
        particles_out = []

        cdf = np.cumsum(weights)

        threshold = np.random.uniform(0, 1/num_particles_out)  # incorrect interval [0, M^-1)
        i = 0
        for j in range(num_particles_out):
            while (threshold > cdf[i] and i + 1 < len(self.particlecloud.poses)):
                i += 1
            new_particle = self.apply_noise_to_pose(self.particlecloud.poses[i])
            particles_out.append(new_particle)
            threshold += 1/num_particles_out
        return particles_out
    
    def generate_random_particles(self, num_particles):
        particles = []
        defaultOrientation = Quaternion(0, 0, 0, 1)

        # Basically doing the same as the add_noise function but to the random particles
        for _ in range(num_particles):

            # Pick a random position from our list of unoccupied spaces.
            idx = np.random.randint(len(self._unoccupied_coords))
            cell = self._unoccupied_coords[idx]
            
            noiseX = self.sample_error_position()
            noiseY = self.sample_error_position()
            rotation = np.random.uniform(-np.pi, np.pi)
            
            newPose = Pose(
                Point(
                    cell[0] + noiseX,
                    cell[1] + noiseY,
                    0
                ),
                rotateQuaternion(defaultOrientation, rotation)
            )
            particles.append(newPose)
        return particles

    def estimate_pose(self):
        # return self.particlecloud.poses[0]
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        # return self.particlecloud.poses[0]
        # Convert particle poses to a list of data points for clustering
        data_points = []
        for pose in self.particlecloud.poses:
                data_points.append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    
        
        cluster_labels, cluster_centroids = self.kmeans_clustering(data_points, num_clusters=3)
        
        # Identify the densest cluster
        unique_labels, counts = np.unique(cluster_labels, return_counts=True)
        densest_cluster_label = unique_labels[np.argmax(counts)]
            
        
        # Initialize an empty list to store particles that belong to the densest cluster
        particles_in_densest_cluster = []

        # Iterate through each particle and its corresponding label
        for idx, pose in enumerate(self.particlecloud.poses):
            
            # Check if the particle's label matches the label of the densest cluster
            if cluster_labels[idx] == densest_cluster_label:
                
                # If it matches, add the particle to our list
                particles_in_densest_cluster.append(pose)

        cluster = particles_in_densest_cluster

        # Calculate average position of particles in the densest cluster
        sum_x, sum_y, sum_z = 0, 0, 0
        sum_qx, sum_qy, sum_qz, sum_qw = 0, 0, 0, 0
        
        for p in cluster:
            sum_x += p.position.x
            sum_y += p.position.y
            sum_z += p.position.z
            
            sum_qx += p.orientation.x
            sum_qy += p.orientation.y
            sum_qz += p.orientation.z
            sum_qw += p.orientation.w
            
        avg_x = sum_x / len(cluster)
        avg_y = sum_y / len(cluster)
        avg_z = sum_z / len(cluster)
        
        avg_qx = sum_qx / len(cluster)
        avg_qy = sum_qy / len(cluster)
        avg_qz = sum_qz / len(cluster)
        avg_qw = sum_qw / len(cluster)

        # Normalize the quaternion
        norm = math.sqrt(avg_qx**2 + avg_qy**2 + avg_qz**2 + avg_qw**2)
        avg_qx /= norm
        avg_qy /= norm
        avg_qz /= norm
        avg_qw /= norm

        # Set the estimated pose with the computed average position and orientation
        estimated_pose = Pose()
        estimated_pose.position.x = avg_x
        estimated_pose.position.y = avg_y
        estimated_pose.position.z = avg_z
        estimated_pose.orientation.x = avg_qx
        estimated_pose.orientation.y = avg_qy
        estimated_pose.orientation.z = avg_qz
        estimated_pose.orientation.w = avg_qw

        return estimated_pose

    def kmeans_clustering(self, data_points, num_clusters=3, max_iterations=100):

        if (self._lost_behaviour):
            max_iterations = 10
        else:
            max_iterations = 30

        # Randomly select initial centroids from the data points
        initial_centroids_indices = np.random.choice(len(data_points), size=num_clusters, replace=False)
        centroids = np.array(data_points)[initial_centroids_indices]

        previous_centroids = np.zeros_like(centroids)

        # Array to store cluster label for each data point
        cluster_labels = np.zeros(len(data_points))

        for iteration in range(max_iterations):
            # For each data point, find the nearest centroid and assign it to the corresponding cluster
            for idx, current_point in enumerate(data_points):
                # Calculate distance of current point to each centroid
                distances = np.linalg.norm(current_point - centroids, axis=1)
                
                # Find the index of the nearest centroid
                closest_centroid_index = np.argmin(distances)
                
                # Add closest centroid index to cluster labels array
                cluster_labels[idx] = closest_centroid_index

            previous_centroids = centroids.copy()

            
            for i in range(num_clusters):
                
                # Loop through the labels to find points that belong to the current cluster
                points_in_cluster = []
                for index, label in enumerate(cluster_labels):
                    if label == i:
                        points_in_cluster.append(data_points[index])
                points_in_cluster = np.array(points_in_cluster)
                
                # Calculate new centroid for each cluster from the mean
                centroids[i] = np.mean(points_in_cluster, axis=0)

            # If centroids don't change, then its done
            if np.all(centroids == previous_centroids):
                break

        return cluster_labels, centroids