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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

tfListener = tf.TransformListener()

def plotter():
    print("Plotter starting...")
    rospy.init_node('plotter', anonymous=True)

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("kf_odom/odom", PoseWithCovarianceStamped, odomCallback)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    rospy.spin()

def odomCallback(poseWithCovariance):
    (trans,rot) = tfListener.lookupTransform('/world', '/imu_link', rospy.Time(0))
    
    pass

if __name__ == "__main__":
    try:
        plotter()
    except rospy.ROSInterruptException:
        pass
