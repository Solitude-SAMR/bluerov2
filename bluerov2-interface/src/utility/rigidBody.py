#!/usr/bin/env python

"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

c = lambda a : np.cos(a)
s = lambda a : np.sin(a)

def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        R = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        P = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Y = math.atan2(t3, t4)

        return R, P, Y
        
def R(quat):
    """
    pose = [x, y, z, r, p, y]
    """
    r, p, y = quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
    cr, sr, cp, sp, cy, sy = c(r), s(r), c(p), s(p), c(y), s(y)
    return np.array([[cy*cr+sy*sp*sr, sy*cr-cy*sp*sr,  cp*sr, 0.],
                  [-sy*cp,         cy*cp,           sp,    0.],
                  [sy*sp*cr-cy*sr, -cy*sp*cr-sy*sr, cp*cr, 0.],
                  [0.,              0.,               0.,     1.]])
def T(v3):
    """
    pose = [x, y, z, r, p, y]
    """       
    return np.array([v3.x, v3.y, v3.z, 1.])
    
    
def RT(quat, v3):
    return R(quat)*T(v3)


#T = np.array([[0, 0, 0, vx],[0, 0, 0, vz],[0, 0, 0, vy],[0, 0, 0, 1]])
class nav():
    def __init__(self, initialPos=np.array([0., 0., 0.]), initialOri=np.array([0.,0.,0.,1.])):
        self.state = Odometry()
        self.lastT = None
        self.state.pose.pose.position = Point(initialPos[0], initialPos[1], initialPos[2])
        self.state.pose.pose.orientation = Quaternion(initialOri[0], initialOri[1], initialOri[2], initialOri[3])
        self.state.header.frame_id = "odom"
        self.subImu = rospy.Subscriber("/bluerov2/imu/data/ENU", Imu, self.ImuCB, tcp_nodelay=True, queue_size=1)
        self.subDVL = rospy.Subscriber("/bluerov2/DVL", Odometry, self.OdoCB, tcp_nodelay=True, queue_size=1)
        self.subDepth = rospy.Subscriber("/bluerov2/depth", Odometry, self.DepthCB, tcp_nodelay=True, queue_size=1)
        self.pubPose = rospy.Publisher("/odometry/raw", Odometry, tcp_nodelay=True, queue_size=1)
        self.depth = initialPos[2]
        
    def ImuCB(self, msg):
        self.state.pose.pose.orientation = msg.orientation
        self.state.header.stamp = msg.header.stamp
        self.pubPose.publish(self.state)
        
    def DepthCB(self, msg):
        self.depth = msg.pose.pose.position.z
#        self.state.pose.pose.position.z = msg.pose.pose.position.z
#        self.state.header.stamp = msg.header.stamp
#        self.pubPose.publish(self.state)
        
    def OdoCB(self, msg):
        if self.lastT != None:
            if (msg.header.stamp.to_sec() < self.lastT.to_sec()): return #interpolated data, cannot be handeled yet
            dt = msg.header.stamp - self.lastT
            dt = dt.to_sec()
            self.state.header.stamp = msg.header.stamp
            deltaPos = RT(self.state.pose.pose.orientation, msg.twist.twist.linear)
            self.state.pose.pose.position.x += deltaPos[0][0]*dt
            self.state.pose.pose.position.y += deltaPos[1][1]*dt
            self.state.pose.pose.position.z = self.depth
        
            self.lastT = msg.header.stamp
            self.pubPose.publish(self.state)
        else:
            self.lastT = msg.header.stamp
    
if __name__ == '__main__':
    try:
        rospy.init_node('RAW_DATA_NAV', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    navNode = nav(initialPos=np.array([0,0,-0.42999997735]), initialOri=np.array([-0.0197868556729, 0.01251406715950, -0.920387348775, 0.38953189585])) #bluerovbag

#    navNode = nav(initialPos=np.array([-6.20296475742, 9.56097354031, 0.0373533627977]), initialOri=np.array([0.0197710896301, 0.0173543297972 , 0.492502134858, 0.869913546516]))
    rospy.spin()

