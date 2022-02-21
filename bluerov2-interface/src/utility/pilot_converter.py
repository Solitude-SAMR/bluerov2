#!/usr/bin/env python
#Convert DVL messege to Odometry for robot_localization package
from __future__ import division


import rospy
#from scipy.spatial.transform import Rotation as R
# msgs type
import numpy as np
from nav_msgs.msg import Odometry
from underwater_sensor_msgs.msg import DVL
from auv_msgs.msg import NavigationStatus
from sensor_msgs.msg import Imu


def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

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

class BlueRovToPilot():
    def __init__(self):
        self.Odo_sub_topic = rospy.get_param('odometry/filtered', '/odometry/filtered')
        self.Nav_sts_pub_topic = rospy.get_param('navigation_status', '/nav/nav_sts')
        self.OdoSub = rospy.Subscriber(self.Odo_sub_topic, Odometry, self.OdoCB)
        self.navSts = rospy.Publisher(self.Nav_sts_pub_topic, NavigationStatus, queue_size=10)
        
        
    def OdoCB(self, msg):
        nav_sts = NavigationStatus()
        nav_sts.header = msg.header
        #nav_sts.global_position = Todo
        #nav_sts.origin = Todo
        N, E, D = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z #TODO: is depth = -z
        nav_sts.position.north, nav_sts.position.east, nav_sts.position.depth = N, E, D
        #nav_sts.altitude = #from DVL?
        nav_sts.body_velocity.x = msg.twist.twist.linear.x
        nav_sts.body_velocity.y = msg.twist.twist.linear.y
        nav_sts.body_velocity.z = msg.twist.twist.linear.z
        #nav_sts.seafloor_velocity = #Todo
        r, p, y = quaternion_to_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        nav_sts.orientation.x, nav_sts.orientation.y, nav_sts.orientation.z = r, p, y
        nav_sts.orientation_rate.x = msg.twist.twist.angular.x 
        nav_sts.orientation_rate.y = msg.twist.twist.angular.y 
        nav_sts.orientation_rate.z = msg.twist.twist.angular.z
        self.navSts.publish(nav_sts)
#        nav_sts.position_variance = #todo
#        nav_sts.orientation_variance = #todo
        
        
    
        
        
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
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_NED2ENU([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        #r = quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.IMUpub.publish(msg)
#        print(r)
        
if __name__ == '__main__':
    try:
        rospy.init_node('DVL_2_Odometry_converter', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    d2o = DVL2Odometry()
    pilot = BlueRovToPilot()
    rospy.spin()
