#!/usr/bin/env python
#Convert DVL messege to Odometry for robot_localization package
from __future__ import division


import rospy
#from scipy.spatial.transform import Rotation as R
# msgs type
import numpy as np
from nav_msgs.msg import Odometry
from underwater_sensor_msgs.msg import DVL
from sensor_msgs.msg import Imu


def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z
        
def quat_NED2ENU(iquat):
        a  = np.array([0.707, 0.707, 0, 0 ])
        b = np.array([[[iquat[3], -iquat[2], iquat[1], -iquat[0]], \
                    [iquat[2], iquat[3], -iquat[0], -iquat[1]], \
                    [-iquat[1], iquat[0], iquat[3], -iquat[2]], \
                    [iquat[0], iquat[1], iquat[2], iquat[3]]]])

        c = np.array([[[0, 0, -0.707, 0.707], \
                    [0 , 0, 0.707, 0.707], \
                    [0.707, -0.707, 0, 0], \
                    [-0.707, -0.707, 0 ,0]]])
        return (a.dot(b)).dot(c)[0][0]      
def qNED2ENU(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    q.x, q.y, q.z, q.w = y, x, -z, w
        
class DVL2Odometry():
    def __init__(self):
        self.DVLsub_topic = rospy.get_param('DVLsub_topic', '/teledyne_explorer/DVL')
        self.DVLpub_topic = rospy.get_param('DVLpub_topic', '/BlueRov2/DVL')
        self.DVLsub = rospy.Subscriber(self.DVLsub_topic, DVL, self.forward)
        self.IMUsub = rospy.Subscriber("/BlueRov2/imu/data", Imu, self.ImuCB)
        self.DVLpub = rospy.Publisher(self.DVLpub_topic, Odometry, queue_size=10)
        self.IMUpub = rospy.Publisher("/BlueRov2/imu/data/ENU", Imu, queue_size=10)

    def forward(self, dvl):
        odo = Odometry()
        odo.header = dvl.header
        odo.twist.twist.linear.x = 0.0 if abs(dvl.bi_x_axis) > 32.0 else dvl.bi_x_axis #Could potentially be: dvl_data.wi_x_axis
        odo.twist.twist.linear.y = 0.0 if abs(dvl.bi_y_axis) > 32.0 else dvl.bi_y_axis
        odo.twist.twist.linear.z = 0.0 if abs(dvl.bi_z_axis) > 32.0 else dvl.bi_z_axis
        self.DVLpub.publish(odo)
        
    def ImuCB(self, msg):
#        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_NED2ENU([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        msg.orientation = qNED2ENU(msg.orientation)
#        r = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.IMUpub.publish(msg)
#        print(r)
        
if __name__ == '__main__':
    try:
        rospy.init_node('DVL_2_Odometry_converter', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    n = DVL2Odometry()
    rospy.spin()
