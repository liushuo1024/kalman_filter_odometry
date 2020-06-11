#!/usr/bin/env python

# MIT License

# Copyright (c) 2020 Filippo Grazioli

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D


class Plotter:
    def __init__(self):
        mpl.rcParams['legend.fontsize'] = 10
        gs = GridSpec(3, 2)
        self.fig = plt.figure()
        self.plot_position = self.fig.add_subplot(gs[:, 0], projection='3d')
        self.plot_roll = self.fig.add_subplot(gs[0, 1])
        self.plot_pitch = self.fig.add_subplot(gs[1, 1])
        self.plot_yaw = self.fig.add_subplot(gs[2, 1])

        position = np.empty(shape=[0, 3])
        gt_position = np.empty(shape=[0, 3])
        self.position_dict = {'prediction': position, 'gt': gt_position}

        orientation = np.empty(shape=[0, 3])
        gt_orientation = np.empty(shape=[0, 3])
        self.orientation_dict = {'prediction': orientation, 'gt': gt_orientation}

        self.tfListener = tf.TransformListener()

        self.callback_count = 0

    def odom_callback(self, pwcs):
        self.callback_count += 1

        (trans, rot) = self.tfListener.lookupTransform('/world', '/imu_link', rospy.Time(0))

        # Predicted position
        new_position = np.expand_dims(np.array([pwcs.pose.pose.position.x,
                                                pwcs.pose.pose.position.y,
                                                pwcs.pose.pose.position.z]), axis=0)
        self.position_dict['prediction'] = np.append(self.position_dict['prediction'], new_position, axis=0)

        # Predicted orientation - convert to roll, pitch and yaw because it is more intuitive to visualise
        rpy = tf.transformations.euler_from_quaternion((pwcs.pose.pose.orientation.x,
                                                        pwcs.pose.pose.orientation.y,
                                                        pwcs.pose.pose.orientation.z,
                                                        pwcs.pose.pose.orientation.w))
        new_orientation = np.expand_dims(np.array(rpy), axis=0)
        self.orientation_dict['prediction'] = np.append(self.orientation_dict['prediction'], new_orientation, axis=0)

        # Gt position
        new_gt_position = np.expand_dims(np.array(trans), axis=0)
        self.position_dict['gt'] = np.append(self.position_dict['gt'], new_gt_position, axis=0)

        # Gt orientation - convert to roll, pitch and yaw because it is more intuitive to visualise
        rpy = tf.transformations.euler_from_quaternion(rot)
        new_gt_orientation = np.expand_dims(np.array(rpy), axis=0)
        self.orientation_dict['gt'] = np.append(self.orientation_dict['gt'], new_gt_orientation, axis=0)

        if self.callback_count % 100 == 0:  # only update the plot after 100 callbacks
            self.plot_position.clear()
            self.plot_position.set_title('Position XYZ (rotate to refresh)')
            self.plot_position.plot(self.position_dict['prediction'][:, 0],
                                    self.position_dict['prediction'][:, 1],
                                    self.position_dict['prediction'][:, 2], 'c1', label='predicted position')
            self.plot_position.plot(self.position_dict['gt'][:, 0],
                                    self.position_dict['gt'][:, 1],
                                    self.position_dict['gt'][:, 2], 'r1', label='gt position')
            self.plot_position.legend(loc='lower right')

            self.plot_roll.clear()
            self.plot_roll.set_title('Roll [rad]')
            self.plot_roll.plot(self.orientation_dict['prediction'][:, 0], 'c1', label='predicted roll')
            self.plot_roll.plot(self.orientation_dict['gt'][:, 0], 'r1', label='gt roll')
            self.plot_roll.legend(loc='lower right')

            self.plot_pitch.clear()
            self.plot_pitch.set_title('Pitch [rad]')
            self.plot_pitch.plot(self.orientation_dict['prediction'][:, 1], 'c1', label='predicted pitch')
            self.plot_pitch.plot(self.orientation_dict['gt'][:, 1], 'r1', label='gt pitch')
            self.plot_pitch.legend(loc='lower right')

            self.plot_yaw.clear()
            self.plot_yaw.set_title('Yaw [rad]')
            self.plot_yaw.plot(self.orientation_dict['prediction'][:, 2], 'c1', label='predicted yaw')
            self.plot_yaw.plot(self.orientation_dict['gt'][:, 2], 'r1', label='gt yaw')
            self.plot_yaw.legend(loc='lower right')

if __name__ == '__main__':
    print('Plotter node starting...')
    rospy.init_node('plotter', anonymous=True)
    plotter = Plotter()

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('kf_odom/odom', PoseWithCovarianceStamped, plotter.odom_callback)
            plotter.plot_position.set_title('Position XYZ (rotate to refresh)')
            plotter.plot_roll.set_title('Roll [rad]')
            plotter.plot_pitch.set_title('Pitch [rad]')
            plotter.plot_yaw.set_title('Yaw [rad]')
            plt.show()  # cannot place this in the callback, rotate the plot to update it
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    rospy.spin()
