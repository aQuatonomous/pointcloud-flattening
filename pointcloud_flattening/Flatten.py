#!/usr/bin/env python3
'''
    This script takes a pointcloud from a velodyne lidar and flattens it so that all points have z=0.
    Copyright (c) August 2023 Sabrina Button

    NOTES FOR USE BY AQUATONOMOUS: 
    - If you want to use this with out lidar, you will need to change the indicated subscriber to the correct topic
    - This doesn't currently publish the flattened pointcloud, you will need to add that in, also want to change the flat cloud back to PointCloud2 message maybe
    - You also probably want tiered flattening... i.e. if z>150 go to top level, if z<150 go to mid level, if z<50 go to bottom level (for walls vs buoys)

    Message sabrina.button@queensu.ca with any questions or concerns (Sabrina on Slack)
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2, NavSatFix

import numpy as np
import statistics
import PCReader

class PCFlattening(Node):

    def __init__(self):
        super().__init__('pc_flattening')

        # Change this to be a subscriber to the correct topic, i.e. replace 'velodyne_points' with the correct topic
        self.lidar_subscriber = self.create_subscription(PointCloud2, 'velodyne_points', self.LidarProcessingModule, 10)
        self.flat_cloud = None

    # Pointcloud topic received -> use this callback
    def LidarProcessingModule(self, pointcloud):
        
        # The PC Reader just gets the pointcloud into a more usable format 
        pcd_as_numpy_array = np.array(list(PCReader.read_points(pointcloud)))

        # Project the pointcloud to the image plane by setting the z value to 0
        # Here is where you would look at the different z levels and maybe have several levels of flattened pointclouds
        pcd_as_numpy_array[:,2] = 0 

        # Get the azimuth and distance of each point save to array
        pcd_as_numpy_array_az_dist = np.zeros((pcd_as_numpy_array.shape[0], 5))

        # populate with x,y,z, azimuth, distance
        pcd_as_numpy_array_az_dist[:,0:3] = pcd_as_numpy_array[:,0:3]
        pcd_as_numpy_array_az_dist[:,3] = np.arctan2(pcd_as_numpy_array[:,1], pcd_as_numpy_array[:,0]) # azimuth
        
        # convert azimuth to degrees and make positive, round to nearest degree
        pcd_as_numpy_array_az_dist[:,3] = np.round(np.rad2deg(pcd_as_numpy_array_az_dist[:,3])) % 360
        pcd_as_numpy_array_az_dist[:,4] = np.sqrt(pcd_as_numpy_array[:,0]**2 + pcd_as_numpy_array[:,1]**2) # distance

        # Remove all NaN points
        pcd_as_numpy_array_az_dist = pcd_as_numpy_array_az_dist[~np.isnan(pcd_as_numpy_array_az_dist).any(axis=1)]

        ''' This next part of the code gets rid of all of the points that are not nearest to us at each azimuth
        ... could be useful for obstacle avoidance simplification so I will leave it here for now, commented out'''
        # # Get the farthest protruding points at each azimuth
        # pcd_as_numpy_array_az_dist_shortest = np.zeros((360, 5))
        # for i in range(360):
        #     # make a mini array of all the points at the current azimuth
        #     pcd_as_numpy_array_az_dist_shortest_mini = pcd_as_numpy_array_az_dist[np.where(pcd_as_numpy_array_az_dist[:,3] == i)]
        #     if pcd_as_numpy_array_az_dist_shortest_mini.shape[0] == 0:
        #         # if there are no points at this azimuth, skip to the next azimuth
        #         continue
        #     # get the index of the point with the shortest distance
        #     shortest_index = np.argmin(pcd_as_numpy_array_az_dist_shortest_mini[:,4])
        #     # add this to the az dist shortest array at azimuth i
        #     pcd_as_numpy_array_az_dist_shortest[i,:] = pcd_as_numpy_array_az_dist_shortest_mini[shortest_index,:]

        # # Remove all zero only rows
        # pcd_as_numpy_array_az_dist_shortest = pcd_as_numpy_array_az_dist_shortest[~np.all(pcd_as_numpy_array_az_dist_shortest == 0, axis=1)]
        
        self.flat_cloud = pcd_as_numpy_array_az_dist

        # You could plot this here if you wanted to look at it
        
        return 
    
def main(args=None):
    rclpy.init(args=args)

    pcf = PCFlattening()

    rclpy.spin(pcf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
