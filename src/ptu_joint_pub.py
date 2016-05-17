#!/usr/bin/env python	

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState as DynamixelJointState
from dynamixel_controllers.srv import SetSpeed, SetTorqueLimit, TorqueEnable
from std_srvs.srv import Empty

DEFAULT_FREQ = 2.0
MAX_FREQ = 500.0

class Robospect_ptu:	
			
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
		self._pan_command_topic_name = args['pan_command_topic_name'] 
		self._tilt_command_topic_name = args['tilt_command_topic_name'] 
		self._pan_state_topic_name = args['pan_state_topic_name'] 
		self._tilt_state_topic_name = args['tilt_state_topic_name'] 
		self._pan_joint_name = args['pan_joint_name'] 
		self._tilt_joint_name = args['tilt_joint_name'] 
		self._pan_offset = args['pan_offset'] 
		self._tilt_offset = args['tilt_offset'] 
		self._pan_controller_name = args['pan_controller_name']
		self._tilt_controller_name = args['tilt_controller_name']
		self._tilt_max_limit = args['tilt_max_limit']
		self._tilt_min_limit = args['tilt_min_limit']
		self._pan_max_limit = args['pan_max_limit']
		self._pan_min_limit = args['pan_min_limit']
		
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
		
		self.real_freq = 0.0		
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		
								
		self.pan_joint_value = 0.0
		self.tilt_joint_value = 0.0	
		
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()	
		if self.running:
			return 0			
		self.running = True	
		
		self.configureMotors()
			
		self.loop()	
		return 0	
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		# Publishers
		self.joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
		self.pan_command_pub = rospy.Publisher(self._pan_command_topic_name, Float64, queue_size=5)
		self.tilt_command_pub = rospy.Publisher(self._tilt_command_topic_name, Float64, queue_size=5)
		# Subscribers
		self.pan_state_sub = rospy.Subscriber(self._pan_state_topic_name, DynamixelJointState, self.PanCb, queue_size = 5)
		self.tilt_state_sub = rospy.Subscriber(self._tilt_state_topic_name, DynamixelJointState, self.TiltCb, queue_size = 5)	
		
		self.pan_command_offset_sub = rospy.Subscriber('~pan_controller/command', Float64, self.PanCommandOffsetCb, queue_size = 10)
		self.tilt_command_offset_sub = rospy.Subscriber('~tilt_controller/command', Float64, self.TiltCommandOffsetCb, queue_size = 10)
		#Services 
		self.pan_set_speed_srv = rospy.ServiceProxy(self._pan_controller_name+'/set_speed', SetSpeed)
		#print self._pan_controller_name+'/set_speed'
		self.pan_set_torque_limit_srv = rospy.ServiceProxy(self._pan_controller_name+'/set_torque_limit', SetTorqueLimit)
		self.pan_enable_torque_srv = rospy.ServiceProxy(self._pan_controller_name+'/torque_enable', TorqueEnable)
		self.tilt_set_speed_srv = rospy.ServiceProxy(self._tilt_controller_name+'/set_speed', SetSpeed)
		self.tilt_set_torque_limit_srv = rospy.ServiceProxy(self._tilt_controller_name+'/set_torque_limit', SetTorqueLimit)
		self.tilt_enable_torque_srv = rospy.ServiceProxy(self._tilt_controller_name+'/torque_enable', TorqueEnable)
		
		self.ros_initialized = True		
		return 0
		
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		self.ros_initialized = False
		return 0
		
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			self.Loop()
			
			t2 = time.time()
			tdiff = (t2 - t1)	
			t_sleep = self.time_sleep - tdiff
			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0							
		
	def loop(self):	
		
		_joints_dict = {}
		
		_joints_dict[self._pan_joint_name] = {'value': self.pan_joint_value}	
		_joints_dict[self._tilt_joint_name] = {'value': self.tilt_joint_value}	
		
			
		_joints = JointState()
			
		"""
		for i in _joints_dict:
			_joints.name.append(i)
			_joints.position.append(_joints_dict[i]['value'])
			_joints.velocity.append(0.0)
			_joints.effort.append(0.0)
		
		
		"""
		_joints.name = [self._pan_joint_name, self._tilt_joint_name]
		#_joints.position = [self.pan_joint_value, _joints_dict[self._tilt_joint_name]['value']]
		_joints.position = [0, 0]
		
		_joints.velocity = [0, 0]
		_joints.effort = [0, 0]
		freq = 10.0
		t_sleep = 1/freq
		while not rospy.is_shutdown():
			_joints.header.stamp = rospy.Time.now()
			_joints.position[0] = self.pan_joint_value
			_joints.position[1] = self.tilt_joint_value
			self.joint_state_publisher.publish(_joints)
			rospy.sleep(t_sleep)
			#print(_joints)
		
		
		self.joint_state_publisher.unregister()

	#Subscribers Callbacks

	def PanCb(self, msg):
		
		self.pan_joint_value = msg.current_pos - self._pan_offset
		#print 'panCb: %lf'%self.pan_joint_value

	def TiltCb(self,msg):
		#print 'tiltCb'
		self.tilt_joint_value = msg.current_pos - self._tilt_offset
		#print msg
	
	def PanCommandOffsetCb(self,msg):
		target_value = msg.data + self._pan_offset 
		if target_value > self._pan_max_limit:
			target_value = self._pan_max_limit
		elif target_value < self._pan_min_limit:
			target_value = self._pan_min_limit
		#rospy.loginfo('robospect_ptu_joint_pub: pan = %lf', target_value)
		self.pan_command_pub.publish(target_value)
	
	def TiltCommandOffsetCb(self,msg):
		target_value = msg.data + self._tilt_offset 
		if target_value > self._tilt_max_limit:
			target_value = self._tilt_max_limit
		elif target_value < self._tilt_min_limit:
			target_value = self._tilt_min_limit
		self.tilt_command_pub.publish(target_value)
		#rospy.loginfo('robospect_ptu_joint_pub: pan = %lf', target_value)
		
	def configureMotors(self):
		speed = SetSpeed()
		set_torque = SetTorqueLimit()
		enable_torque = TorqueEnable()
		speed.speed = 0.75
		set_torque.torque_limit = 1.0
		enable_torque.torque_enable = True
		
		self.pan_set_speed_srv.call(speed.speed)
		self.pan_set_torque_limit_srv.call(set_torque.torque_limit)
		self.pan_enable_torque_srv.call(enable_torque.torque_enable)
		speed.speed = 0.3
		self.tilt_set_speed_srv.call(speed.speed)
		self.tilt_set_torque_limit_srv.call(set_torque.torque_limit)
		self.tilt_enable_torque_srv.call(enable_torque.torque_enable)
		
def main():

	rospy.init_node("robospect_ptu_joint_filtered_pub")
	
	rospy.sleep(4)
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'pan_command_topic_name': '/pan_controller/command',
	  'tilt_command_topic_name': '/tilt_controller/command',
	  'pan_state_topic_name': '/pan_controller/state',
	  'tilt_state_topic_name': '/tilt_controller/state',
	  'pan_joint_name': 'pan_joint',
	  'tilt_joint_name': 'tilt_joint',
	  'pan_offset': 0.0,
	  'tilt_offset': 0.0,
	  'pan_controller_name': 'pan_controller',
	  'tilt_controller_name': 'tilt_controller',
	  'tilt_max_limit': 1.3,
	  'tilt_min_limit': -0.8,
	  'pan_max_limit': 3.92,
	  'pan_min_limit': -0.78,
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))
			
	
	rc_node = Robospect_ptu(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()

if __name__ == "__main__":
	main()
