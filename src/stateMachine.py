#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import time
import dlib
import pathPlanner as planner
import Astar as astar

FLIGHT_TIME = 10.0
#STATES = {0: "first_turn", 1: "second_turn", 2: "run_Astar", 3: "micro_plan", 4: "action", 5: "land" }
STATE = 0
READ_INTERVAL = 2.0
PI = 3.1415926535897
angular_speed = 45*2*PI/360 	#45 degrees per second
linear_speed = 0.5



def rotate(clockwise,angle):
	global drone_pub
	vel_msg = Twist()
	if clockwise:
		vel_msg.angular.z = -abs(angular_speed) 
	else:
		vel_msg.angular.z = abs(angular_speed) 

	t0 = time.time()
	current_angle = 0
	relative_angle = (angle-angle/2)*2*PI/360 #POSE

	while(current_angle < relative_angle):
		drone_pub.publish(vel_msg)
		t1 = time.time()
		current_angle = angular_speed*(t1-t0)

	#Forcing our robot to stop
	vel_msg.angular.z = 0
	drone_pub.publish(vel_msg)
	return

def x_translate(direction,length):
	######### 0.5 Length = 50 inches #####
	global drone_pub
	#forward = 1
	vel_msg = Twist()
	if direction:
		vel_msg.linear.x = abs(linear_speed) 
	else:
		vel_msg.linear.x = -abs(linear_speed) 

	t0 = time.time()
	current_linear = 0
	relative_linear = length - length*0.6 

	while(current_linear < relative_linear):
		drone_pub.publish(vel_msg)
		t1 = time.time()
		current_linear = linear_speed*(t1-t0)

	#Forcing our robot to stop
	vel_msg.linear.x = 0
	drone_pub.publish(vel_msg)
	return

def y_translate(direction,length):
	global drone_pub
	#forward = 1
	vel_msg = Twist()
	if direction:
		vel_msg.linear.y = abs(linear_speed) 
	else:
		vel_msg.linear.y = -abs(linear_speed) 

	t0 = rospy.Time.now().to_sec()
	current_linear = 0
	relative_linear = length - length*0.6 

	while(current_linear < relative_linear):
		drone_pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_linear = linear_speed*(t1-t0)

	#Forcing our robot to stop
	vel_msg.linear.y = 0
	drone_pub.publish(vel_msg)
	return



def main():
	#initialize ros nodes
	global drone_pub, STATE
	rospy.init_node("statemachine", anonymous=True)
	drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
	takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
	landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)
	#ar_tags = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, planner.average_ar_tag)
	world_map = planner.WorldMap()

	time.sleep(1.)
    #takeoff and start run timer
	takeoff_pub.publish(Empty())
	time.sleep(2.)
	print ("Takeoff! Flight time is %f." % FLIGHT_TIME)
		
	last_call = time.time()
	start_time = time.time()
	print(last_call)
	#while loop for states and flight time
	while not rospy.is_shutdown() and time.time() - start_time < FLIGHT_TIME:
		if STATE == 0:
			if time.time() - last_call > READ_INTERVAL:
				print ("Rotate 0")
				#turn clockwise 45 deg to read ar tags
				rotate(1,45)
				last_call = time.time()
				print(last_call)
				STATE = 1

		if STATE == 1:
			if time.time() - last_call > READ_INTERVAL:
				print ("Rotate 1")
				#turn to counter-clockwise 45 deg to read ar tags
				rotate(0,90)
				last_call = time.time()
				print(last_call)
				STATE = 2

		if STATE == 2:
			if time.time() - last_call > READ_INTERVAL:
				print ("Rotate 2")
				#return to forward and load into map
				rotate(1,45)
				last_call = time.time()
				print(last_call)
				time.sleep(1.)
				x_translate(1,0.5)
				time.sleep(1.)
				#x_translate(0,0.5)
				time.sleep(10.)
				# print ("Add to map")
				# data = planner.averageAreaCoor #([lowtuple], [hightuple])
				# for index in range(0,6):
				# 	if data[0][0][index] != -1.0:
				# 		if index == 4:
				# 			world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
				# 		else:
				# 			world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)

				# world_map.print_obs_map()
				# path = astar.path(astar.a_star(world_map, (0,0), (data[0][0][4],data[0][1][4])), (0,0), (data[0][0][4],data[0][1][4]))
				# print path
				# STATE = 3

    	if STATE == 3:
    	 	temp = 0 

    	if STATE == 5:
    		temp = 0



	landing_pub.publish(Empty())
	print ("Shutdown")

if __name__ == '__main__':
	main()
