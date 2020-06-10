#!/usr/bin/env python

#MIT License

#Copyright (c) 2020 Filippo Grazioli

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import rospy
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf


class Plotter:
    def __init__(self):
        mpl.rcParams['legend.fontsize'] = 10
        self.fig = plt.figure()
        self.plot3D = self.fig.add_subplot(111, projection='3d')

        position = np.empty(shape=[0, 3])
        gt_position = np.empty(shape=[0, 3])
        self.position_dict = {'prediction': position, 'gt': gt_position}

        orientation = np.empty(shape=[0, 4])
        gt_orientation = np.empty(shape=[0, 4])
        self.orientation_dict = {'prediction': orientation, 'gt': gt_orientation}

        self.tfListener = tf.TransformListener()

        self.callback_count = 0

    def odom_callback(self, pwcs):
        self.callback_count += 1

        (trans, rot) = self.tfListener.lookupTransform('/world', '/imu_link', rospy.Time(0))

        new_position = np.expand_dims(np.array([pwcs.pose.pose.position.x,
                                                pwcs.pose.pose.position.y,
                                                pwcs.pose.pose.position.z]), axis=0)
        self.position_dict['prediction'] = np.append(self.position_dict['prediction'], new_position, axis=0)

        new_orientation = np.expand_dims(np.array([pwcs.pose.pose.orientation.x,
                                                   pwcs.pose.pose.orientation.y,
                                                   pwcs.pose.pose.orientation.z,
                                                   pwcs.pose.pose.orientation.w]), axis=0)
        self.orientation_dict['prediction'] = np.append(self.orientation_dict['prediction'], new_orientation, axis=0)

        new_gt_position = np.expand_dims(np.array(trans), axis=0)
        self.position_dict['gt'] = np.append(self.position_dict['gt'], new_gt_position, axis=0)

        new_gt_orientation = np.expand_dims(np.array(rot), axis=0)
        self.orientation_dict['gt'] = np.append(self.orientation_dict['gt'], new_gt_orientation, axis=0)

        if self.callback_count % 100 == 0:  # only update the plot after 100 callbacks
            self.plot3D.clear()
            self.plot3D.set_title('Position')
            self.plot3D.plot(self.position_dict['prediction'][:, 0],
                             self.position_dict['prediction'][:, 1],
                             self.position_dict['prediction'][:, 2], 'c1', label='predicted position')
            self.plot3D.plot(self.position_dict['gt'][:, 0],
                             self.position_dict['gt'][:, 1],
                             self.position_dict['gt'][:, 2], 'r1', label='gt position')
            self.plot3D.legend()

            #ToDo: plot orientation


if __name__ == '__main__':
    print('Plotter node starting...')
    rospy.init_node('plotter', anonymous=True)
    plotter = Plotter()

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('kf_odom/odom', PoseWithCovarianceStamped, plotter.odom_callback)
            plt.show()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    rospy.spin()

