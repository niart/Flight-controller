#!/usr/bin/env python


"""
This is an isolated pilot, it does not use the drone framework whatsoever.

"""

import rospy
from math import radians, degrees, isinf
import mavros
import mavros.utils as mavUI
import mavros.setpoint as mavSP
import mavros.command as mavCMD

from PID2 import PID
from lidarProcessing import calcYaw
from lidarProcessing import lidarOperations

from mavros_msgs.msg import AttitudeTarget, State

import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64, Bool, Header, Float32)

from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion, Vector3

from sensor_msgs.msg import LaserScan, Range


mavros.set_namespace('mavros')  # needed to operate!


pilotEnable = '/onboard/isolated/llp_enable'
# This is the "pilot topic" for desired setpoints
targetWP_Atti = '/onboard/setpoint/lidarLandingAtti'

# these are the LiDAR inputs
LiDAR100_Sub = '/LiDAR/VU8_100deg/scan'          # type LaserScan
LiDAR48_Sub = '/LiDAR/VU8_48deg/scan'
Altimeter_Sub = '/mavros/distance_sensor/lidarlite_pub'

# needed for sanity check
onB_StateSub = '/onboard/state'

class IsolatedPilot():
    def __init__(self):
        rospy.init_node('lidarLandingPilot')

        self.lidarProc = lidarOperations()

        self.enable = False
        self.ready = False

        self.rate = rospy.Rate(20)

        self._msg_LiDAR_100 = None
        self._msg_LiDAR_48 = None
        self._msg_Altimeter = None
        self._msg_feedbackATII = AttitudeTarget()
        self.msgOutput = AttitudeTarget()

        self._msg_MAVROS_state = mavros_msgs.msg.State()

        self.powerLinePresent = 0   # 0 = False, 1 = 48Deg, 2 = 100Deg, 3 = both
        
        self.exampleThrottle = 0.675474524498       # Value based off GPS hover 
        self.controllerThrottle = PID(0.4, 0.02, 0.0001, 0.05)   # create instance of PID Controller
        self.controllerYaw = PID(0.1,0,0,0.2)

        self.controllerThrottle.updateSetpoint(10.0) # 3meters setpoint
        self.controllerYaw.updateSetpoint(0)        # 0 Degrees desired - drone is aligned 

        # Publishers
        self.setpointATTI_Pub = rospy.Publisher(mavros.get_topic(
            'setpoint_raw', 'attitude'), AttitudeTarget, queue_size=1)
        self.setpointThrottle_Pub = mavSP.get_pub_attitude_throttle(
            queue_size=2)

        self.debugErrorPub = rospy.Publisher('/onboard/debug/altError', Float32, queue_size=1)
        # needed to enable the pilot
        rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_attitude'), mavros_msgs.msg.AttitudeTarget, self._cb_AttitudeFeedback)

        rospy.Subscriber(onB_StateSub, String, self.sanityCheck)
        rospy.Subscriber(pilotEnable, Bool, self.onEnable)
        rospy.Subscriber('/mavros/state',
                         mavros_msgs.msg.State, self._cb_mavrosState)
        # LiDAR subscribers
        rospy.Subscriber(LiDAR100_Sub, LaserScan, self._lidarUpdate_100)
        rospy.Subscriber(LiDAR48_Sub, LaserScan,
                         self._lidarUpdate_48)
        rospy.Subscriber(Altimeter_Sub, Range, self._altimeterUpdate)
        self._mavrosHandshake()
        self.ready = True
        rospy.loginfo('lidarLander Initialised.')

    def _cb_mavrosState(self, msg):
        self._msg_MAVROS_state = msg

    def _mavrosHandshake(self):
        rospy.loginfo('lidarLander: Waiting for MAVROS Connection.')
        for i in range(0, 3):
            print('.'),
            if self._msg_MAVROS_state.connected:
                rospy.loginfo("lidarLander: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self._msg_MAVROS_state.connected:
            errorMsg = "MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)

        rospy.loginfo('lidarLander: waiting for LidarMessages')

        msg100 = False
        msg48 = False
        msgAlt = False

        for i in range(1, 60):
            if not (self._msg_LiDAR_100 is None):
                msg100 = True
            if not (self._msg_LiDAR_48 is None):
                msg48 = True

            if not (self._msg_Altimeter is None):
                msgAlt = True

            rospy.sleep(1)

            if msg100 and msg48 and msgAlt:
                break

    def sanityCheck(self, msg):
        if msg.data != 'isolate':
            self.enable = False
            rospy.logwarn('LLP Disable: The autonomy framework has taken over')

    # IMPORTANT Function: enables/ disables the pilot.
    def onEnable(self, msg):
        if self.ready:
            if msg.data == False:
                if self.enable:
                    rospy.logwarn("Disabling lidarPilot system!")
            else:
                if self.lidarProc.powerLinePresent != 3:
                    rospy.logwarn("LLP: unable - both lidars not detecting.")
                    pass
                else:
                    if not self.enable:
                        rospy.logwarn("Enabling lidarPilot system!")
                    self.controllerThrottle.ITerm = 0  # Reset Iterm to avoid integral windup

            self.enable = msg

        else:
            rospy.logwarn("lidarLandingPilot Unable. System is not ready")       

    def onStateChange(self, msg):
        if msg.data == 'lidar_land':
            if not self.enable:
                print('lidar Landing pilot enabled')
            self.enable = True

        else:
            if self.enable:
                print('lidar landing disabled')
            self.enable = False

    # Required Function, ensures the outgoing message header has a time of "now"

    def _pubMsg(self, msg, topic):
        msg.header.stamp = rospy.Time.now()
        topic.publish(msg)

        self.rate.sleep()

# Message callbacks

    def _lidarUpdate_100(self, msg):
        if self._msg_LiDAR_100 == None:
            rospy.loginfo('VU8_100 Message received.')
        self._msg_LiDAR_100 = msg
        self.lidarProc._msg_LiDAR_100 = msg

        pass

    def _lidarUpdate_48(self, msg):
        if self._msg_LiDAR_48 == None:
            rospy.loginfo('VU8_48 Message received.')
        self._msg_LiDAR_48 = msg
        self.lidarProc._msg_LiDAR_48 = msg

        pass

    def _altimeterUpdate(self, msg):
        if self._msg_Altimeter == None:
            rospy.loginfo('AltimeterReading_acquired')

        self._msg_Altimeter = msg
        pass
    
    def _cb_AttitudeFeedback(self,msg):
        self._msg_feedbackATII = msg

# Simple Functions
    '''
    provide a lidarMessage to give a yes/no whether an object is within the LiDARs FOV
    call: result = self.isDetected(self._msg_LiDAR_100)
    '''

    def generateATTI_Msg(self, roll, pitch, yaw, throttle=0.7):

        (_, _, yawF) = euler_from_quaternion([self._msg_feedbackATII.orientation.x, self._msg_feedbackATII.orientation.y, self._msg_feedbackATII.orientation.z, self._msg_feedbackATII.orientation.w])
        yawAlt = degrees(yawF) - yaw 

        if yawAlt != 'nan':
            outMsg = AttitudeTarget()
            outMsg.header = Header()
            outMsg.header.frame_id = "base_footprint"   #"att_pose"  # "base_footprint"
            outMsg.body_rate = Vector3()
            outMsg.orientation = Quaternion(
                *quaternion_from_euler(radians(roll), radians(pitch), radians(yawAlt)))
            # outMsg.thrust = throttle/55 + 0.62
            outMsg.thrust=throttle 
            outMsg.type_mask = 7  # ignore body rate
            
            outMsg.header.stamp = rospy.Time.now()
        else: 
            rospy.logwarn('Has lidar lost the powerline')
        return outMsg

# Control algorithms

    def yaw_controller(P=1, I=1, D=0.01, T=10):
        pid = PID2.PID(P, I, D)
        pid.output = 0
        pid.SetPoint = calcYaw.Yaw
        error = pid.update(self.error)
        pid.setSampleTime(0.1)
        while (error > 0):  # here the while condition should be "error > 0", but I always failed by writing so.
            for i in range(1, T):
                pid.SetPoint = calcYaw.Yaw  # this should be the outcome of calculation program

                pid.output = pid.update(pid.output)
                print('input', pid.SetPoint)
                print('output', pid.output)
                error = pid.SetPoint - pid.output
                print('error', error)

        pass
            
    def run(self):

        while not rospy.is_shutdown():
            self.lidarProc.analyse()
            # self.lidarProc.powerLinePresent = 3
            throttleResponse = self.controllerThrottle.update(self._msg_Altimeter.range)
            if self.enable:
                outputThrottle = self.exampleThrottle + throttleResponse/100
                # print self._msg_feedbackATII.thrust, throttleResponse, outputThrottle 
                desiredYaw = 0
                if self.lidarProc.powerLinePresent == 3:
                    
                    calculatedYaw = self.lidarProc.calcYaw()
                    desiredYaw = self.controllerYaw.update(calculatedYaw)
                    print (calculatedYaw, desiredYaw)
                self.msgOutput = self.generateATTI_Msg(0, 0, desiredYaw, self.exampleThrottle)


                self._pubMsg(self.msgOutput, self.setpointATTI_Pub) # uncomment once calculateControl is complete
                self.rate.sleep()
                if not self.enable:
                    continue
if __name__ == "__main__":
    IP = IsolatedPilot()
    IP.run()
