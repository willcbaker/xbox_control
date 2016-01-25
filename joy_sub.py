#!/usr/bin/env python

#Created by William Baker: VSGC GRA 2015
#this is a GUI for camera views and controller input, with additional buttons and stuff for later use
#must run as root
#sudo su


import rospy, os, xboxdrvmod, math
from sensor_msgs.msg import Joy
from std_msgs.msg import Byte, ByteMultiArray, Int16, UInt16MultiArray, MultiArrayDimension

			
#axis 0: (Left-x)  	steering
#axis 1: (left-y) 	N/A
#axis 2: (right-x)	web-x
#axis 3: (right-y)	web-y
#axis 4: (RT)		forward
#axis 5: (LT)		backward
#axis 6: (D-x)		kinect-x
#axis 7: (D-y)		kinect-y
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
AXIS_X1 = 0
AXIS_Y1 = 1
AXIS_X2 = 2
AXIS_Y2 = 3
AXIS_RT = 4
AXIS_LT = 5
AXIS_DX = 6
AXIS_DY = 7

CAM_PAN = 0
CAM_TILT = 1

KINECT = 0
WEBCAM = 1
STEERING = 2
DRIVING = 3

PAN_KINECT   =  0xC0
TILT_KINECT  =  0x30
PAN_WEBCAM   =  0x0C
TILT_WEBCAM  =  0x03

DEFAULT_DRIVING = 0
DEFAULT_STEERING = 90
DEFAULT_KINECT_PAN=90
DEFAULT_KINECT_TILT=145
DEFAULT_WEBCAM_PAN=90
DEFAULT_WEBCAM_TILT=70

DEADZONE = 0.08

#basic function to scale values from input range to output range
def scale(x, in_min, in_max, out_min, out_max):
	value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	return value#constrain(value,out_min,out_max)
	
def constrain(value, out_min,out_max):
	value = min(value,out_max)
	value = max(value, out_min)
	return value
	
#class PanTilt:
	
class NasaBot():
	def __init__(self,deadzone = 0.05, axes_gain = [1,1,-5,5,1,1,-5,5] ):
		self.joy_sub = rospy.Subscriber('joy', Joy , self.callback_joystick, queue_size=10)
		#self.sonar_sub = rospy.Subscriber('sonar_array', UInt16MultiArray , self.callback_sonar, queue_size=10)
		self.button_sub = rospy.Subscriber('buttons', Byte , self.callback_buttons, queue_size=10)
		#self.joy_rum_sub = rospy.Subscriber('joy_act', Joy , self.callback_joystick_action, queue_size=10)
		
		#publishers
		self.throttle_pub = rospy.Publisher('throttle', Int16, queue_size=10)
		self.steering_pub = rospy.Publisher('steering', Int16, queue_size=10)
		self.leds_pub = rospy.Publisher('leds', UInt16MultiArray, queue_size=10)
		self.pantilt_pub = rospy.Publisher('pan_tilt', UInt16MultiArray, queue_size=10)
		
		self.deadzone = deadzone
		self.axes_gain=axes_gain
		
		self.kinect   = [DEFAULT_KINECT_PAN,DEFAULT_KINECT_TILT]
		self.webcam   = [DEFAULT_WEBCAM_PAN,DEFAULT_WEBCAM_TILT]
		self.steering = DEFAULT_STEERING
		self.driving  = DEFAULT_DRIVING
		self.delta = [(0,0),(0,0),0,0]
		self.last = [self.kinect,self.webcam,self.steering,self.driving]
	def callback_buttons(self,message):
		print message.data
	def callback_joystick(self,message):
		#rospy.loginfo('Joystick Message')
		'''
		if message.buttons[BUTTON_A]:
			self.next_sphero()
		if message.buttons[1]:
			self.active_sphero.set_heading(90)
		if message.buttons[2]:
			self.active_sphero.set_heading(270)
		if message.buttons[3]:
			self.active_sphero.set_angular_velocity(200)
		'''
		axes = []#
		for axis in message.axes:
			axes.append(self.apply_deadzone(axis))
		
		#axis 0: (Left-x)  	steering
		self.delta[STEERING] = scale(axes[AXIS_X1],-1,1,0,180)
		
		#axis 1: (left-y) 	N/A
		
		#axis 2: (right-x)	web-x
		#axis 3: (right-y)	web-y
		self.delta[WEBCAM] = (scale(axes[AXIS_X2],-1,1,-self.axes_gain[AXIS_X2],self.axes_gain[AXIS_X2]),
			scale(axes[AXIS_Y2],-1,1,-self.axes_gain[AXIS_Y2],self.axes_gain[AXIS_Y2]))
		#axis 4: (RT)		forward
		#axis 5: (LT)		backward
		self.delta[DRIVING] = scale(axes[AXIS_RT],1,-1,0,255)
		if self.delta[DRIVING] < 1:
			self.delta[DRIVING] = scale(axes[AXIS_LT],-1,1,-255,0)
		#axis 6: (D-x)		kinect-x
		#axis 7: (D-y)		kinect-y
		self.delta[KINECT] = (scale(axes[6],-1,1,-self.axes_gain[6],self.axes_gain[6]),
			scale(axes[7],-1,1,-self.axes_gain[7],self.axes_gain[7]))
		
	def update(self):
		if self.delta[KINECT][CAM_PAN]:
			self.kinect[CAM_PAN] = constrain(self.kinect[CAM_PAN] + self.delta[KINECT][CAM_PAN],0,180)
			msg = UInt16MultiArray()
			msg.layout.dim = [MultiArrayDimension('data',2,2)]
			msg.data = [PAN_KINECT,self.kinect[CAM_PAN]]
			self.pantilt_pub.publish(msg)
		if self.delta[KINECT][CAM_TILT]:
			self.kinect[CAM_TILT] = constrain(self.kinect[CAM_TILT] + self.delta[KINECT][CAM_TILT],0,180)
			msg = UInt16MultiArray()
			msg.layout.dim = [MultiArrayDimension('data',1,2)]
			msg.data = [TILT_KINECT,self.kinect[CAM_TILT]]
			self.pantilt_pub.publish(msg)
		if self.delta[WEBCAM][CAM_PAN]:
			self.webcam[CAM_PAN] = constrain(self.webcam[CAM_PAN] + self.delta[WEBCAM][CAM_PAN],0,180)
			msg = UInt16MultiArray()
			msg.layout.dim = [MultiArrayDimension('data',1,2)]
			msg.data = [PAN_WEBCAM,self.webcam[CAM_PAN]]
			self.pantilt_pub.publish(msg)
		if self.delta[WEBCAM][CAM_TILT]:
			self.webcam[CAM_TILT] = constrain(self.webcam[CAM_TILT] + self.delta[WEBCAM][CAM_TILT],0,180)
			msg = UInt16MultiArray()
			msg.layout.dim = [MultiArrayDimension('data',1,2)]
			msg.data = [TILT_WEBCAM,self.webcam[CAM_TILT]]
			self.pantilt_pub.publish(msg)
		
		if self.delta[DRIVING] != self.driving:
			self.driving = self.delta[DRIVING]
			self.throttle_pub.publish(self.driving)
		if self.delta[STEERING] != self.steering:
			self.steering = self.delta[STEERING]
			self.steering_pub.publish(self.steering)
		
	def apply_deadzone(self,val):
		if val > 0 and val > self.deadzone:
			return val
		if val < 0 and val < -self.deadzone:
			return val
		return 0
        
if __name__ == '__main__':
	rospy.init_node('joystick_control')
	#subprocess = os.popen('sudo xboxdrv -d -D -v --dbus session','w',65536)
	#subprocess.write('nasa\n')

	bot = NasaBot(DEADZONE)
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		bot.update()
   		r.sleep()
