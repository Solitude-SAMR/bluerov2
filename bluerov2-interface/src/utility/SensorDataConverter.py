#!/usr/bin/env python
#Convert DVL messege to Odometry for robot_localization package
from __future__ import division

from copy import copy
import rospy
#from scipy.spatial.transform import Rotation as R
# msgs type
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from underwater_sensor_msgs.msg import DVL
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped
from scipy.interpolate import interp1d

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
        
def qNED2ENU(quat):
        #print "-----------"
        #print "q1 : ", quat
        iquat = [-quat.x, -quat.y, quat.z, quat.w]
        a  = np.array([0.707, 0.707, 0, 0 ])
        b = np.array([[[iquat[3], -iquat[2], iquat[1], -iquat[0]], \
                    [iquat[2], iquat[3], -iquat[0], -iquat[1]], \
                    [-iquat[1], iquat[0], iquat[3], -iquat[2]], \
                    [iquat[0], iquat[1], iquat[2], iquat[3]]]])

        c = np.array([[[0, 0, -0.707, 0.707], \
                    [0 , 0, 0.707, 0.707], \
                    [0.707, -0.707, 0, 0], \
                    [-0.707, -0.707, 0 ,0]]])
        q = (a.dot(b)).dot(c)[0][0]    
        #print "q2 : ", Quaternion(-q[0], q[1], q[2], q[3])
        return Quaternion(-q[0], q[1], q[2], q[3])
        
#def qNED2ENU(q):
#    return Quaternion(q.x, q.y, -q.z, q.w)
        
class SensorDataConverter():
    def __init__(self):
        self.lastDVL = None
        self.lastIMU = None
        self.lastDepth = None
        
        self.log = False
        self.logfile = '/home/orca/your_file.txt'
        
        self.DVLHistory = []
        self.maxDVLhistorySize = 25
        self.interpolateOnNextValidDVL = False
        self.validDVLsamples = 0
        self.x = []
        self.xt = []
        self.ix = []
        self.ixt = []
        self.interpolate = True
        
        self.DVLsub_topic = rospy.get_param('DVL_TOPIC_IN', '/teledyne_explorer/DVL')
        self.IMUsub_topic = rospy.get_param('IMU_TOPIC_IN', '/BlueRov2/imu/data')
#        self.DEPTHsub_topic = rospy.get_param('DEPTH_TOPIC_IN', '/BlueRov2/Depth')
        self.DVLpub_topic = rospy.get_param('DVLpub_topic', '/BlueRov2/DVL')
        
        self.DVLsub = rospy.Subscriber(self.DVLsub_topic, DVL, self.dvlCB)
        self.DVLpub = rospy.Publisher(self.DVLpub_topic, Odometry, queue_size=10)
        
        self.IMUsub = rospy.Subscriber("/BlueRov2/imu/data", Imu, self.ImuCB)
        self.IMUpub = rospy.Publisher("/BlueRov2/imu/data/ENU", Imu, queue_size=10)
        
        
#        self.depthsub = rospy.Subscriber(self.DEPTHsub_topic, Odometry, self.depthCB)
#        self.DepthPub = rospy.Publisher("/BlueRov2/Depth2", Odometry, queue_size=10)
        
#        self.subOrb = rospy.Subscriber("/orb_slam2_stereo/pose", PoseStamped, self.orbCB, tcp_nodelay=True, queue_size=1)
#        self.Orbpub = rospy.Publisher("/orb_slam2_stereo/odometry", Odometry, queue_size=10)
        self.firsstOrb = None
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
#        rospy.Timer(rospy.Duration(0.5), self.lostSensorTimer)
        
    def lostSensorTimer(self, event):
        t_now = rospy.Time.now().to_sec()
        if self.lastDVL != None:
            if t_now - self.lastDVL.header.stamp.to_sec() > 1.0: #this need to be figured out so it doesnt send messages while waiting for data to interpolate
                print "publishing zero DVL message, DVL seems to be lost"
                dvlMsg = Odometry()
                dvlMsg.header.stamp = rospy.Time.now()
                dvlMsg.header.frame_id = self.lastDVL.header.frame_id
                dvlMsg.twist.twist.linear.x =  0.0
                dvlMsg.twist.twist.linear.y =  0.0
                dvlMsg.twist.twist.linear.z =  0.0
                dvlMsg.twist.covariance = np.diag([0.01, 0.01, 0.01, 0.0, 0.0, 0.0]).flatten() #double variance as it is interpolated
                self.DVLpub.publish(dvlMsg)
        if self.lastDepth != None:
            if t_now - self.lastDepth.header.stamp.to_sec() > 1.0: #this need to be figured out so it doesnt send messages while waiting for data to interpolate
                print "publishing zero Depth message, Depth seems to be lost"
                depthMsg = Odometry()
                depthMsg.header.stamp = rospy.Time.now()
                depthMsg.header.frame_id = self.lastDepth.header.frame_id
                depthMsg.twist.twist.linear.z =  self.lastDepth.pose.pose.position.z
                depthMsg.twist.covariance = np.diag([0.01, 0.01, 0.01, 0.0, 0.0, 0.0]).flatten() 
                self.DepthPub.publish(depthMsg) 
        if self.lastIMU != None:
            if t_now - self.lastIMU.header.stamp.to_sec() > 1.0: #this need to be figured out so it doesnt send messages while waiting for data to interpolate
                print "publishing zero IMU message, IMU seems to be lost"
                imuMsg = copy(self.lastIMU)
                imuMsg.header.stamp = rospy.Time.now()
                imuMsg.header.stamp = rospy.Time.now()
                imuMsg.header.frame_id = self.lastIMU.header.frame_id
                imuMsg.angular_velocity.x = 0.
                imuMsg.angular_velocity.y = 0.
                imuMsg.angular_velocity.z = 0.
                self.IMUpub.publish(imuMsg)    
                
    
    def depthCB(self, msg):
        self.lastDepth = msg
        nm = Odometry()
        nm.header = msg.header
        nm.pose.pose.position.z = msg.pose.pose.position.z
        nm.pose.covariance = np.diag([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]).flatten()
        self.DepthPub.publish(nm)
        
    
    def orbCB(self, msg):
        nm = Odometry()
        nm.header = msg.header
#        if self.firsstOrb == None:
#            self.firsstOrb = msg.header.stamp
#        t = msg.header.stamp - self.firsstOrb
#        if t.to_sec() >= 30 and t.to_sec() <= 50:
#            None
#        else:
        nm.pose.pose = msg.pose
        nm.pose.covariance = [ 0.3,  0.,  0.,  0.,  0.,  0.,
                               0.,  0.3,  0.,  0.,  0.,  0.,
                               0.,  0.,  0.3,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.1,  0.,  0.,
                               0.,  0.,  0.,  0.,  0.1,  0.,
                               0.,  0.,  0.,  0.,  0.,  0.1]
        self.Orbpub.publish(nm)
        
    def dvlInterpolate(self):
        x = []
        y = []
        z = []
        t = []
        newT = []
        interpolateT = []
#        t_0 = self.DVLHistory[0].header.stamp.to_sec()
        
        for i in range(len(self.DVLHistory)):
            msg = self.DVLHistory[i]
            newT.append(msg.header.stamp.to_sec())
            if msg.bi_error > -32.:
                x.append(msg.bi_y_axis)
                y.append(-msg.bi_x_axis)
                z.append(msg.bi_z_axis)
                t.append(msg.header.stamp.to_sec())
            else:
                interpolateT.append(i)
        print "DVLHistory ", len(self.DVLHistory), " t    : ", len(t), " tnew : ", len(newT), " inter : ", len(interpolateT), "::::::"
        print "t: ", t[0], "->", t[-1], ".... ", "newt: ", newT[0], "->", newT[-1]
        print ""
        interpolationmode = ['linear', 'cubic', 'quadratic', 'slinear'][1]
        fx = interp1d(t, x, kind=interpolationmode)
        fy = interp1d(t, y, kind=interpolationmode)
        fz = interp1d(t, z, kind=interpolationmode)
        x_n = fx(newT)
        y_n = fy(newT)
        z_n = fz(newT)
        res = []
        
        for i in interpolateT:
            odo = Odometry()
            odo.twist.twist.linear.x =  x_n[i] 
            odo.twist.twist.linear.y =  y_n[i]
            odo.twist.twist.linear.z =  z_n[i]
            odo.twist.covariance = np.diag([0.04, 0.04, 0.04, 0.0, 0.0, 0.0]).flatten() #double variance as it is interpolated
            odo.header = self.DVLHistory[i].header
            res.append(odo)
        return res
        
    def dvlCB(self, msg):
        if msg.bi_error > -32.:
            odo = Odometry()
            odo.header = msg.header
            odo.twist.twist.linear.x =  msg.bi_y_axis #Maybe this should be handeled from a TF
            odo.twist.twist.linear.y =  -msg.bi_x_axis
            odo.twist.twist.linear.z =  msg.bi_z_axis
            odo.twist.covariance = np.diag([0.01, 0.01, 0.01, 0.0, 0.0, 0.0]).flatten()
            self.DVLpub.publish(odo)
        else:
            odo = Odometry()
            odo.header = msg.header
            odo.twist.twist.linear.x =  0.0 #Maybe this should be handeled from a TF
            odo.twist.twist.linear.y =  0.0
            odo.twist.twist.linear.z =  0.0
            odo.twist.covariance = np.diag([0.01, 0.01, 0.01, 0.0, 0.0, 0.0]).flatten()
            self.DVLpub.publish(odo)
#            self.DVLHistory.append(msg)
#            self.lastDVL = odo
##           odo.child_frame_id = dvl.header.frame_id
#            if self.interpolateOnNextValidDVL == True and self.validDVLsamples > 4 and self.interpolate:
#                newMsgs = self.dvlInterpolate()
#                print "publishing ", len(newMsgs), " new messages"
#                for nm in newMsgs:
#                    self.DVLpub.publish(nm)
#                    self.ixt.append(nm.header.stamp.to_sec())
#                    self.ix.append(nm.twist.twist.linear.x)
#                    
#                self.DVLHistory[:] = [msg]
#                self.validDVLsamples = 1
#                self.DVLpub.publish(odo)
#                self.interpolateOnNextValidDVL = False
#                
#                self.xt.append(odo.header.stamp.to_sec())
#                self.x.append(odo.twist.twist.linear.x)
#                if self.log:
#                    with open(self.logfile, 'w') as f: #just for debugging (plotting using interPlotTest.py)
#                        print >> f,""
#                        print >> f,"ix = ", self.ix
#                        print >> f,""
#                        print >> f,"ixt = ", self.ixt
#                        print >> f,""
#                        print >> f, "x = ", self.x
#                        print >> f,""
#                        print >> f,"xt = ", self.xt
#            else:
#                self.DVLpub.publish(odo)
#                self.interpolateOnNextValidDVL = False
#                self.validDVLsamples += 1
#                
#                self.xt.append(odo.header.stamp.to_sec())
#                self.x.append(odo.twist.twist.linear.x)
#            
#            if len(self.DVLHistory) > self.maxDVLhistorySize:
#                self.DVLHistory.pop(0)
#            
#        else:
#            self.interpolateOnNextValidDVL = True
#            if self.validDVLsamples>0 and len(self.DVLHistory) > 0: #cant have a non valid package to start with
#                self.DVLHistory.append(msg)

        
    def ImuCB(self, msg):

        orientation = qNED2ENU(msg.orientation)
        msg.orientation = orientation
        msg.orientation_covariance = np.diag([0.05, 0.05, 0.05]).flatten()
#        msg.orientation_covariance = [1.294e-2, 0, 0, 0, 1.324e-2, 0, 0, 0, 2.012e-2]
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = -gz
        msg.angular_velocity_covariance = np.diag([0.025, 0.025, 0.025]).flatten()
#        msg.linear_acceleration.z = -msg.linear_acceleration.z
#        msg.linear_acceleration.x = -msg.linear_acceleration.x
#        msg.linear_acceleration.y = -msg.linear_acceleration.y
        msg.linear_acceleration_covariance = np.diag([0.1, 0.1, 0.1]).flatten()
#        msg.angular_velocity_covariance = [6.235e-4, 0, 0, 0, 7.026e-4, 0, 0, 0, 6.218e-4]
#        new_angular_x(EAST_ENU)=old_angular_y(NED_EAST)
#        new_angular_y(NORTH_ENU)=old_angular_x(NED_NORTH)
#        new_angular_z(UP_ENU)=-old_angular_z(NED_DOWN)
        self.lastIMU = msg
        self.IMUpub.publish(msg)
#        print(r)
        
if __name__ == '__main__':
    try:
        rospy.init_node('Sensor_conv', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    n = SensorDataConverter()
    rospy.spin()
