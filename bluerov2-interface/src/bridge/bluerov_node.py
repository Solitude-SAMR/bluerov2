#!/usr/bin/env python

from __future__ import division

import json
import math
import re
import rospy
#import sys
import time
#import pyrr
import numpy as np

from bridge import Bridge

try:
    from pubs import Pubs
    from subs import Subs
except:
    from bluerov.pubs import Pubs
    from bluerov.subs import Subs

# msgs type
from geometry_msgs.msg import TwistStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_srvs.srv import SetBool, SetBoolResponse #Trigger, TriggerResponse, 
import tf

def mapRanges(value, inMin, inMax, outMin, outMax):
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    valueScaled = float(value - inMin) / float(inSpan)
    return outMin + (valueScaled * outSpan)

class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', joy_mode='Body',joy=True,baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.pub = Pubs()
        self.sub = Subs()
        
        self.thrusterRanges = [1100., 1900.] #pwm Todo, test and move to parameter
        self.thrusterInputRanges = [-3.0, 3.0]  #m/s, r/s Todo, test and move to parameter
        
        self.flightmode=0
        self.joy_var=[1500 for _ in range(9)]
        self.joy_var[6]=1100
        self.set_mode('manual')
        self.ROV_name = 'bluerov2'
        self.model_base_link = 'base_link' #JONATAN CHANGED FROM /base_link
        self.joy_mode=joy_mode
        self.camera=1500
        self.lights=1000
	self.pilot_sensitivity=25
        self.starttime=time.time()

        self.pub_topics = [
            [
                self._create_battery_msg,
                '/battery',
                BatteryState,
                1
            ],
            [
                self._create_ROV_state,
                '/state',
                String,
                1
            ],
            [
                self._create_raw_msg,
                '/raw',
                String,
                1
            ],
            [
                self._create_imu_msg,
                '/imu/data',
                Imu,
                1
            ],
#            [
#                self._create_odometry_msg,
#                '/odometry',
#                Odometry,
#                1
#            ],
            [
                self._create_depth_msg,
                '/depth',
                Odometry,
                1
            ],
        ]
        self.sub_topics= [
            [
                self._setpoint_velocity_cmd_vel_callback,
                '/setpoint_velocity/cmd_vel',
                TwistStamped,
                1
            ],
            [
                self._set_servo_callback,
                '/servo{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],#SetBool
            [
                self._set_rc_channel_callback,
                '/rc_channel{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],
            [
                self._set_mode_callback,
                '/mode/set',
                String,
                1
            ],
            [
                self._arm_callback,
                '/arm',
                Bool,
                1
            ],
#            [
#                self._joy_callback,
#                '/joy',
#                Joy,
#                1
#            ],
        ]

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

        for topic in self.sub_topics:
            if len(topic) <= 4:
                callback, topic_name, msg, queue = topic
                self._sub_subscribe_topic(topic_name, msg, queue, callback)
            else:
                callback, topic_name, msg, queue, arg = topic
                for name in arg:
                    self._sub_subscribe_topic(topic_name.format(name), msg, queue, callback)
#        _ = rospy.Subscriber('/pilot/forces', Vector6Stamped, self._pilot_callback)
        
        
        _ = rospy.Subscriber('/bluerov2/cmd_vel', Twist, self._cmd_vel_callback)
#        _ = rospy.Subscriber('/bluerov2/Light', UInt16, self._light_callback)
#        _ = rospy.Subscriber('/bluerov2/cameraTilt', UInt16, self._camera_tilt_callback)
        
        _ = rospy.Service('/bluerov2/arm', SetBool, self._handle_Arm)
        _ = rospy.Service('/bluerov2/disarm', SetBool, self._handle_Disarm)
        
        self.IMU_ENU_pub = rospy.Publisher("/bluerov2/imu/data/ENU", Imu, queue_size=10)
        
        _ = rospy.Service('/bluerov2/setmode/manual', SetBool, self._setModeManual)
        _ = rospy.Service('/bluerov2/setmode/alt_hold', SetBool, self._setModeAltHold)
        _ = rospy.Service('/bluerov2/setmode/stabalize', SetBool, self._setModeStabalize)
        
        self.arm_throttle(False) #disarm at start
    @staticmethod
    def _callback_from_topic(topic):
        """ Create callback function name

        Args:
            topic (str): Topic name

        Returns:
            str: callback namem
        """
        return topic.replace('/', '_') + '_callback'

    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def _sub_subscribe_topic(self, topic, msg, queue_size=1, callback=None):
        """ Subscribe to a topic using the subscriber

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
            callback (None, optional): Callback function
        """
        self.sub.subscribe_topic(self.ROV_name + topic, msg, queue_size, callback)

    def _set_servo_callback(self, msg, topic):
        """ Set servo from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            None: Description
        """
        paths = topic.split('/')
        servo_id = None
        for path in paths:
            if 'servo' in path:
                servo_id = int(re.search('[0-9]', path).group(0)) + 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_servo_pwm(servo_id, msg.data)

    def _set_rc_channel_callback(self, msg, topic):
        """ Set RC channel from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            TYPE: Description
        """
        paths = topic.split('/')
        channel_id = None
        for path in paths:
            if 'rc_channel' in path:
                channel_id = int(re.search('[0-9]', path).group(0)) - 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_rc_channel_pwm(channel_id, msg.data)
    
    def mapRanges(self, value):
        return np.clip(mapRanges(value, 
                         self.thrusterInputRanges[0], 
                         self.thrusterInputRanges[1], 
                         self.thrusterRanges[0],
                         self.thrusterRanges[1]), 
                         self.thrusterRanges[0],
                         self.thrusterRanges[1])
#        return mapRanges(value, -85., 85., 1400., 1600.)
    
    def _handle_Arm(self, req):
        self.arm_throttle(True)
        return SetBoolResponse(True, "")

    def _handle_Disarm(self, req):
        self.arm_throttle(False)
        return SetBoolResponse(True, "")
        
    def _setModeManual(self, req):
        self.set_mode('manual')
        return SetBoolResponse(True, "")

    def _setModeAltHold(self, req):
        self.set_mode('alt_hold')
        return SetBoolResponse(True, "")

    def _setModeStabalize(self, req):
        self.set_mode('stabilize')
        return SetBoolResponse(True, "")
    

    def _pilot_callback(self, msg):
        """Listen to /pilot/forces to set vx,vy,vz,vr,vp,vyaw
            TODO: figure out value ranges and conversions
                    BlueRov in range 1000-2000, 1500=0.0
        """
        override = []
        override.append(self.mapRanges(msg.values[4]))  		#Pitch
        override.append(self.mapRanges(msg.values[3]))		#Roll
        override.append(self.mapRanges(msg.values[2]))  	     #throttle
        override.append(self.mapRanges(msg.values[5])	)         #Yaw
        override.append(self.mapRanges(msg.values[0]))         #Forward
        override.append(self.mapRanges(msg.values[1]))         #Lateral
        override.append(self.lights) #light strength
        override.append(self.camera)
#        print(override)
        self.set_rc_channels_pwm(override)
       
    def _cmd_vel_callback(self, msg):
        override = []
        override.append(self.mapRanges(msg.angular.y))  		#Pitch
        override.append(self.mapRanges(msg.angular.x))		#Roll
        override.append(self.mapRanges(msg.linear.z))  	     #throttle
        override.append(3000-self.mapRanges(msg.angular.z))         #Yaw
        override.append(self.mapRanges(msg.linear.x))         #Forward
        override.append(3000-self.mapRanges(msg.linear.y))         #Lateral
        override.append(65535)                          #light strength
        override.append(65535)                          # camera servo tilt
#        print(override)
        self.set_rc_channels_pwm(override)
        
#    def _camera_tilt_callback(self, msg):
#        if msg.data > 2000:
#            msg.data = 2000
#        elif msg.data < 1100:
#            msg.data = 1100
#        self.camera = msg.data
#        self.set_rc_channel_pwm(7, self.camera)
#        
#    def _light_callback(self, msg):
#        if msg.data > 2000:
#            msg.data = 2000
#        elif msg.data < 1100:
#            msg.data = 1100
#        self.lights = msg.data
#        self.set_servo_pwm(9, self.lights)
#        self.set_rc_channel_pwm(6, self.lights)
        
        
#    def _joy_callback(self, msg, _ ):
#        """ Set RC channel from joy topic
#
#        Args:
#            msg (TYPE): ROS Joy message
#
#        Returns:
#            TYPE: Description
#	"""
#        # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
#        joy=[msg.axes[i] for i in range(6)]
#        buttons=msg.buttons
# 
#        override=[]
#
#        if self.joy_mode=="Global":
#            attitude_data = self.get_data()['ATTITUDE']
#            orientation = pyrr.Vector3([attitude_data[i] for i in ['roll', 'pitch', 'yaw']])
#            attitude_quat=pyrr.quaternion.create_from_eulers(orientation)
#            inv_attitude_quat=pyrr.quaternion.inverse(attitude_quat)
#            if self.flightmode == 0:  #Forward/Lateral
#                thrust_vector=Vector3([joy[1],joy[0],joy[3]])
#            else:                     #Pitch/Roll
#                thrust_vector=Vector3([0,0,joy[3]])
#            thrust_vector=pyrr.quaternion.apply.to.vector(inv_attitude_quat,thrust_vector)
#            if self.flightmode == 0:#Forward/Lateral
#                joy[1]=thrust_vector[0]
#                joy[0]=thrust_vector[1]
#                joy[3]=thrust_vector[2]
#            else:                   #Pitch/Roll
#                joy[3]=thrust_vector[2]
#
#        for val in joy:
#            newval=int(val*self.pilot_sensitivity*4 + 1500)
#            override.append(newval)
#	
#        override.append(1100)
#        override.append(1500)
#
#        override1=override[:]
#
#        if buttons[1]==1:
#            self.set_mode('manual')
#        if buttons[0]==1:
#            if  self.flightmode==0:
#                print("Flight Mode is pitch/yaw") 
#                self.flightmode=1
#            else:
#                self.flightmode=0
#                print("Flight Mode is forward/lateral")  
#        if buttons[8]==1:
#            self.set_mode('alt_hold')
#        if buttons[3]==1:
#            self.set_mode('stabilize')
#        if buttons[4]==1:
#            if self.camera>1100:
#                self.camera-=50
#                override[7]=1100
#            
#        if buttons[5]==1:
#            if self.camera<1900:
#                self.camera+=50
#                override[7]=1900
#        if buttons[6]==1:
#            self.arm_throttle(False)
#	if buttons[7]==1:
#            self.arm_throttle(True)
#	if buttons[9]==1:
#            self.joy_mode="Body"
#            print("Joystick Mode is relative to Body axis")
#	if buttons[10]==1:
#            self.joy_mode="Global"
#            print("Joystick Mode is relative to Global axis")
#        if override[6]<1500:
#            if self.lights<1900:
#                self.lights+=80
#        if override[6]>1500:
#            if self.lights>1100:
#                self.lights-=80
#        if override[7]>1500:
#            #Increase Sensitivity
#            if self.pilot_sensitivity<100:
#                self.pilot_sensitivity+=25
#        if override[7]<1500:
#            #Decrease Sensitivity
#            if self.pilot_sensitivity>25:
#                self.pilot_sensitivity-=25
#        if self.flightmode==0:
#            override[0]=1500  		#Pitch
#            override[1]=1500		#Roll
#            override[2]=override1[4]  	#throttle
#            override[3]=override1[3]	#Yaw
#            override[4]=3000-override1[1]    #Forward
#            override[5]=3000-override1[0]    #Lateral
#
#
#        else:
#            override[0]=override1[4]
#            override[1]=override1[3]
#            override[2]=1500#override1[3]
#            override[3]=1500#override1[2]
#            override[4]=3000-override1[1]    #Forward
#            override[5]=3000-override1[0]    #Lateral
#        override[6]=self.lights
#        self.joy_var=override[:]
#        self.set_rc_channels_pwm(override)


    def _set_mode_callback(self, msg, _):
        """ Set ROV mode from topic

        Args:
            msg (TYPE): Topic message
            _ (TYPE): Description
        """
        self.set_mode(msg.data)

    def _arm_callback(self, msg, _):
        """ Set arm state from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        self.arm_throttle(msg.data)

    def _setpoint_velocity_cmd_vel_callback(self, msg, _):
        """ Set angular and linear velocity from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        params = [
            None,
            None,
            None,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            None,
            None,
            None,
            None,
            None,
            ]
        self.set_position_target_local_ned(params)

        #http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
        params = [
            None,
            None,
            None,
            None,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
            None,
            ]
        self.set_attitude_target(params)

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_odometry_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')
        #if 'LOCAL_POSITION_NED' not in self.get_data():
        #    raise Exception('no LOCAL_POSITION_NED data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()
        self._create_header(msg)
        msg.header.frame_id = 'odom'
        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)

#        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
#        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
#        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
#        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]
        self.pub.set_data('/odometry', msg)



    def _create_depth_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available

        if 'alt' not in self.get_data()['VFR_HUD']:
            raise Exception('no alt data')


        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.z = self.get_data()['VFR_HUD']['alt']
        msg.pose.covariance = np.diag([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]).flatten()
        self.pub.set_data('/depth', msg)

    def qNED2ENU(self, quat):
#        print "-----------"
#        print "q1 : ", quat
        iquat = [quat.x, quat.y, quat.z, quat.w]
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
#        print "q2 : ", Quaternion(-q[0], q[1], q[2], q[3])
        return Quaternion(-q[0], q[1], q[2], q[3])
    
    def _create_imu_msg(self):
        """ Create imu message from ROV data

        Raises:
            Exception: No data available
        """
        
        # Check if data is available
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: move all msgs creating to msg
        msg = Imu()

        self._create_header(msg)
        msg.header.frame_id = 'imu_link'
        #http://mavlink.org/messages/common#SCALED_IMU
        imu_data = None
        for i in ['', '2', '3']:
            try:
                imu_data = self.get_data()['SCALED_IMU{}'.format(i)]
                break
            except Exception as e:
                pass

        if imu_data is None:
            raise Exception('no SCALED_IMUX data')

        acc_data = [imu_data['{}acc'.format(i)]  for i in ['x', 'y', 'z']]
        gyr_data = [imu_data['{}gyro'.format(i)] for i in ['x', 'y', 'z']]
#        mag_data = [imu_data['{}mag'.format(i)]  for i in ['x', 'y', 'z']] #TODO: use?

        #http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        msg.linear_acceleration.x = acc_data[0]/100
        msg.linear_acceleration.y = -acc_data[1]/100
        msg.linear_acceleration.z = -acc_data[2]/100
        msg.linear_acceleration_covariance = np.diag([6.235e-4, 7.026e-4, 6.218e-4]).flatten()

        msg.angular_velocity.x = gyr_data[0]/1000
        msg.angular_velocity.y = -gyr_data[1]/1000
        msg.angular_velocity.z = -gyr_data[2]/1000
        msg.angular_velocity_covariance = np.diag([0.025, 0.025, 0.025]).flatten()

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        
#        orientation[0] = -orientation[0]
        orientation[1] = -orientation[1]
        orientation[2] = -orientation[2]
        
        q = tf.transformations.quaternion_from_euler(*orientation)
        
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.orientation_covariance = np.diag([1.294e-5, 1.324e-5, 2.012e-5]).flatten()

        self.pub.set_data('/imu/data', msg)
        
        
#        msgENU = Imu()
#
#        self._create_header(msgENU)
#        msgENU.header.frame_id = 'imu_link'
#        msgENU.linear_acceleration.x = acc_data[1]/100
#        msgENU.linear_acceleration.y = acc_data[0]/100
#        msgENU.linear_acceleration.z = -acc_data[2]/100
#        msgENU.linear_acceleration_covariance = np.diag([6.235e-4, 6.235e-4, 6.218e-4]).flatten()
#
#        msgENU.angular_velocity.x = gyr_data[1]/1000
#        msgENU.angular_velocity.y = gyr_data[0]/1000
#        msgENU.angular_velocity.z = -gyr_data[2]/1000
#        msgENU.angular_velocity_covariance = np.diag([0.025, 0.025, 0.025]).flatten()
#
#        #http://mavlink.org/messages/common#ATTITUDE
#        orientationENU[0] = orientation[1]
#        orientationENU[1] = orientation[0]
#        orientationENU[2] = -orientation[2]
#        print "ENU" , np.rad2deg(orientationENU)
#        
#        q = tf.transformations.quaternion_from_euler(*orientation)
#        
#        msgENU.orientation.x = q[0]
#        msgENU.orientation.y = q[1]
#        msgENU.orientation.z = q[2]
#        msgENU.orientation.w = q[3]
#        msgENU.orientation_covariance = np.diag([1.294e-5, 1.324e-5, 2.012e-5]).flatten()
#        
#        self.IMU_ENU_pub.publish(msgENU)
#        odomMsg = Odometry()
#        self._create_header(odomMsg)
#        odomMsg.header.frame_id = 'imu_link'
#        odomMsg.pose.pose.orientation = msg.orientation
#        self.pubOdomImu.publish(odomMsg)
        #self.pub.set_data('/odometry', odomMsg)

    def _create_battery_msg(self):
        """ Create battery message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SYS_STATUS' not in self.get_data():
            raise Exception('no SYS_STATUS data')

        if 'BATTERY_STATUS' not in self.get_data():
            raise Exception('no BATTERY_STATUS data')

        bat = BatteryState()
        self._create_header(bat)

        #http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery']/1000
        if bat.voltage < 12.5:
            rospy.logwarn("Battery is LOW : %s", bat.voltage)
        bat.current = self.get_data()['SYS_STATUS']['current_battery']/100
        bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        self.pub.set_data('/battery', bat)


    def _create_raw_msg(self):
        s = self.get_data()
        
        string = String()
        string.data = str(json.dumps(s, ensure_ascii=False))

        self.pub.set_data('/raw', string)
        
    def _create_ROV_state(self):
        """ Create ROV state message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        rc_channel_msg = self.get_data()['RC_CHANNELS']
        rc_channel = [rc_channel_msg['chan{}_raw'.format(i+1)] for i in range(9)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(8)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = (rc_channel[6] - 1100) / 8
        #need to check
        camera_angle =self.camera - 1500

        # Create angle from pwm
        camera_angle = 45*camera_angle/400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)

        state = {
            'Gain': self.pilot_sensitivity,
            'motor': motor_throttle,
            'light': light_on,
            'camera_angle': camera_angle,
            'mode': mode,
            'arm': arm
        }

        string = String()
        string.data = str(json.dumps(state, ensure_ascii=False))

        self.pub.set_data('/state', string)

    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
	if (time.time()-self.starttime)>1 :
            self.starttime=time.time()
	    
#            self.set_rc_channels_pwm(self.joy_var)
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    bluerov = BlueRov(device='udp:localhost:14550')

    while not rospy.is_shutdown():
        bluerov.publish()
