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


mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
plot3D = fig.add_subplot(111, projection='3d')

position = np.empty(shape=[0, 3])
gt_position = np.empty(shape=[0, 3])
position_dict = {"prediction": position, "gt": gt_position}

orientation = np.empty(shape=[0, 4])
gt_orientation = np.empty(shape=[0, 4])
orientation_dict = {"prediction": orientation, "gt": gt_orientation}

tfListener = tf.TransformListener()


def plotter():
    print("Plotter starting...")
    rospy.init_node('plotter', anonymous=True)

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("kf_odom/odom", PoseWithCovarianceStamped, odom_callback, (position_dict, orientation_dict))
            plot3D.legend()
            plt.show()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    rospy.spin()


def odom_callback(pwcs, args):
    (trans, rot) = tfListener.lookupTransform('/world', '/imu_link', rospy.Time(0))
    
    new_position = np.expand_dims(np.array([pwcs.pose.pose.position.x,
                                            pwcs.pose.pose.position.y,
                                            pwcs.pose.pose.position.z]), axis=0)
    position_dict["prediction"] = np.append(args[0].get("prediction"), new_position, axis=0)

    new_orientation = np.expand_dims(np.array([pwcs.pose.pose.orientation.x,
                                               pwcs.pose.pose.orientation.y,
                                               pwcs.pose.pose.orientation.z,
                                               pwcs.pose.pose.orientation.w]), axis=0)
    orientation_dict["prediction"] = np.append(args[1].get("prediction"), new_orientation, axis=0)

    new_gt_position = np.expand_dims(np.array(trans), axis=0)
    position_dict["gt"] = np.append(args[0].get("gt"), new_gt_position, axis=0)

    new_gt_orientation = np.expand_dims(np.array(rot), axis=0)
    orientation_dict["gt"] = np.append(args[1].get("gt"), new_gt_orientation, axis=0)

    plot3D.plot(position_dict["prediction"][:, 0],
                position_dict["prediction"][:, 1],
                position_dict["prediction"][:, 2], label='predicted position')
    plot3D.plot(position_dict["gt"][:, 0],
                position_dict["gt"][:, 1],
                position_dict["gt"][:, 2], label='gt position')
    #ToDo: plot orientation
    #ToDo: fix proble with non-displayed labels
    pass


if __name__ == "__main__":
    try:
        plotter()
    except rospy.ROSInterruptException:
        pass
