#!/usr/bin/env python
# Author: Leo Zhang

import rospy
import numpy as np
import math
from micro_doppler_pkg.msg import MicroDoppler
from ti_mmwave_rospkg.msg import RadarScan

class micro_doppler_signature:
    def __init__(self, host_velocity=0):
        self.frame_id = 'micro_doppler'
        self.host_velocity = host_velocity
        self.time_domain_bins = 25

        while not rospy.has_param('/ti_mmwave/doppler_vel_resolution'):
            continue
        self.nd = rospy.get_param("/ti_mmwave/numLoops")
        self.ntx = rospy.get_param("/ti_mmwave/num_TX")
        self.fs = rospy.get_param("/ti_mmwave/f_s")
        self.fc = rospy.get_param("/ti_mmwave/f_c")
        self.PRI = rospy.get_param("/ti_mmwave/PRI")
        self.tfr = rospy.get_param("/ti_mmwave/t_fr")
        self.max_range = rospy.get_param("/ti_mmwave/max_range")
        self.vrange = rospy.get_param("/ti_mmwave/range_resolution")
        self.max_vel = rospy.get_param("/ti_mmwave/max_doppler_vel")
        self.vvel = rospy.get_param("/ti_mmwave/doppler_vel_resolution")

        self.mds_array = np.zeros((self.nd, self.time_domain_bins))
        self.prev_id = 0
        self.mds_cur = np.zeros(self.nd) 

        self.listener_publisher()

    def listener_publisher(self):
        rospy.init_node('micro_doppler_node')
        self.sub_ = rospy.Subscriber('/ti_mmwave/radar_scan', RadarScan, self.ti_doppler_parser)
        self.pub_ = rospy.Publisher('/ti_mmwave/micro_doppler', MicroDoppler, queue_size=100)
        rospy.spin()

    def ti_doppler_parser(self, radar):
        
        if radar.target_id < self.prev_id:
            self.micro_doppler()
        self.prev_id = radar.target_id
        self.mds_cur[radar.doppler_bin] += 10**(radar.intensity/10) - 1

    def micro_doppler(self):
        temp1 = np.delete(self.mds_array, 0, 1)
        temp2 = np.transpose([self.mds_cur])
        self.mds_array = np.append(temp1, temp2, axis=1)
        mds_list = self.mds_array.flatten().tolist()
        mds_msg = MicroDoppler()
        mds_msg.header.frame_id = self.frame_id
        mds_msg.header.stamp = rospy.Time.now()
        mds_msg.time_domain_bins = self.time_domain_bins
        mds_msg.num_chirps = self.nd
        mds_msg.micro_doppler_array = mds_list
        self.pub_.publish(mds_msg)

        self.mds_cur = np.zeros(self.nd)
        
if __name__ == '__main__':
    micro_doppler_signature(host_velocity=0)
    
    
