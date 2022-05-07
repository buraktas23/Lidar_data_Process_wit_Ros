#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarMvrs:
    def __init__(self):
        self.sub = rospy.Subscriber('spur/laser/scan', LaserScan, self.lidar_data)


    def lidar_data(self,lidar_data):
        self.min_rad = lidar_data.angle_min
        self.max_rad = lidar_data.angle_max
        self.step = lidar_data.angle_increment
        self.lidar_rng = lidar_data.ranges
        # rospy.loginfo('lidar data : {0}'.format(self.min_rad))
        # rospy.loginfo('lidar data : {0}'.format(len(self.lidar_rng)))
        self.obs_coordinates()


    def lidar_data_process(self):
        self.deg_new_x = list()
        self.deg_new_y = list()
        self.lid_new_x = list()
        self.lid_new_y = list()
        self.leng = np.arange(self.min_rad,self.max_rad,self.step)
        self.lidar_ang = np.transpose(self.leng)
        self.lidar_rng_new = list()

        for i in range(len(self.lidar_rng)):
            self.lidar_rng_new.append(self.lidar_rng[i])
            if len(self.lidar_rng_new)== len(self.lidar_rng):
                self.lidar_rng_new.append(self.lidar_rng[0])

        for i in range(len(self.lidar_ang)):
            self.deg_new_x.append(math.cos(self.lidar_ang[i]))
            self.deg_new_y.append(math.sin(self.lidar_ang[i]))

            self.lid_new_x.append(self.lidar_rng_new[i]*self.deg_new_x[i])
            self.lid_new_y.append(self.lidar_rng_new[i]*self.deg_new_y[i])

        # rospy.loginfo('length of lidar: {0}'.format(len(self.leng)))
        # rospy.loginfo('lid_new_x: {0}'.format(len(self.lid_new_x)))

    def obs_coordinates(self):
        self.lidar_data_process()

        self.xn_new = list()
        self.yn_new = list()
        self.dist = list()

        self.xn = self.lid_new_x
        self.yn = self.lid_new_y

        for i in range(len(self.xn)):
            if not np.isinf(self.xn[i]) and not np.isinf(self.yn[i]):
                self.xn_new.append(-self.xn[i])
                self.yn_new.append(-self.yn[i])


        for i in range(len(self.xn_new)):
            self.dist.append(np.hypot(self.xn_new[i],self.yn_new[i]))

        self.min_i = self.dist.index(min(self.dist))

        self.nearest_point = np.array([[self.xn_new[self.min_i]],[self.yn_new[self.min_i]]])
        rospy.loginfo('Nearest Point fron of X coordinate : {0}'.format(self.nearest_point[0]))
        rospy.loginfo('Nearest Point fron of Y coordinate : {0}'.format(self.nearest_point[1]))


def main(args):
    rospy.init_node('mavros_lidar', anonymous=True)
    lid = LidarMvrs()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('jhdkjg')


if __name__ == '__main__':
    main(sys.argv)



