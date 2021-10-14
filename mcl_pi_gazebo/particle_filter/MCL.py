#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, PoseArray, Point32, PolygonStamped
from sensor_msgs.msg import PointCloud
from laser_scan_get_map import MapClientLaserScanSubscriber 
from particle_filter import ParticleFilter
from particlesintersection import RobotFusion
from matplotlib import pyplot as plt
from sklearn.neighbors import NearestNeighbors as KNN
import tf_conversions

def main():
    rospy.init_node('ParticleFilter', anonymous = True)
    PF_l = ParticleFilter(Np=300)
    r = rospy.Rate(5)
    plt.ion()
    fig = plt.figure()
    # For debug: draw the occupancy map and check if it's correct
    # plt.imshow(PF_l.scan.occupancy_grid)
    # fig.canvas.draw()
    while not rospy.is_shutdown():
        r.sleep()

        PF_l.pub()

        mean = np.mean(PF_l.particles,axis=0)

        # M = PF_l.scan.loction_based(mean)
        # plt.imshow(-M+PF_l.scan.occupancy_grid)
        # fig.canvas.draw()

    rospy.spin()


if __name__ == "__main__":
    main()