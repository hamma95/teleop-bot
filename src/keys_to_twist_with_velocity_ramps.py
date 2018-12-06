#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

key_mapping = { 'z': [ 0, 1], 'x': [0, -1],
				'q': [-1, 0], 'd': [1, 0],
				's': [ 0, 0] }


g_vel_scales = [0.1, 0.1] # default to very slow
g_vel_ramps = [1, 1] # units: meters per second^2



def vel_ramp(v_now,v_target,t_prev,t_now,ramp_rate):

	step = ramp_rate*(t_now-t_prev).to_sec() 
	sign = 1.0 if (v_target > v_now) else -1.0
	error=math.fabs(v_target-v_now)

	if (step > error) :
		return v_target
	else:
		return v_now + sign*step

def ramped_twist(now,target,t_prev,t_now,ramps):

	t=Twist()
	t.linear.x = vel_ramp(now.linear.x,target.linear.x,t_prev,t_now,ramps[1])
	t.angular.z = vel_ramp(now.angular.z,target.angular.z,t_prev,t_now,ramps[0])
	return t






def send_twist():
	global g_last_twist_send_time, g_target_twist, g_last_twist,\
	g_vel_scales, g_vel_ramps, g_twist_pub

	t_now = rospy.Time.now()
	g_last_twist = ramped_twist(g_last_twist, g_target_twist,g_last_twist_send_time, t_now, g_vel_ramps)
	g_last_twist_send_time = t_now
	g_twist_pub.publish(g_last_twist)


def keys_cb(msg):
	global g_target_twist,g_vel_scales

	if len(msg.data)==0 or not key_mapping.has_key(msg.data[0]):
		return #unknown key
	else:
		vels = key_mapping[msg.data[0]]
		g_target_twist.linear.x= vels[1]*g_vel_scales[0]
		g_target_twist.angular.z = vels[0]*g_vel_scales[1]

def fetch_param(name,default):

	if rospy.has_param(name):
		return rospy.get_param(name)
	else:
		rospy.logwarn("%s does not exist; using %.1f" % (name,default))
		return default



if __name__ == '__main__':


	rospy.init_node("keys_to_twist_with_velocity_ramps")
	g_twist_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
	rospy.Subscriber("keys",String,keys_cb)
	g_last_twist_send_time = rospy.Time.now()
	g_last_twist=Twist()
	g_target_twist=Twist()
	g_vel_scales[0]=fetch_param('~angular_scale',0.1)
	g_vel_scales[1]=fetch_param('~linear_scale',0.1)
	g_vel_ramps[0]=fetch_param('~angular_accel',1)
	g_vel_ramps[1]=fetch_param('~linear_accel',1)
	rate=rospy.Rate(20)

	while not rospy.is_shutdown():
		send_twist()
		rate.sleep()





    



























