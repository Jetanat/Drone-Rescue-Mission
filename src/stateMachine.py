#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import time
import dlib
import pathPlanner as planner
import Astar as astar

FLIGHT_TIME = 25.0
STATE = 1
READ_INTERVAL = 3.0
PI = 3.1415926535897
angular_speed = 45*2*PI/360 	#45 degrees per second
linear_speed = 0.5
MOCAP_ERROR = 0.1

mocap_x = 0
mocap_y = 0
#mocap_z = 0
mocap_theta = 0

current_path = 0


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
	global drone_pub
	#forward = 1
	vel_msg = Twist()
	if direction:
		vel_msg.linear.x = abs(linear_speed) 
	else:
		vel_msg.linear.x = -abs(linear_speed) 

	print(pose_x)
	t0 = time.time()
	current_linear = 0
	relative_linear = length - length*0.65 
	print(relative_linear)

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

	t0 = time.time()
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


def set_pose_destination(pose_x,pose_y):
	global mocap_x, mocap_y, mocap_theta
	x = pose_x - mocap_x
	y = pose_y - mocap_y
	# break move commands into respective functions
	# moves should be in meters 
	if x > 0:
		print("x forward")
		#x_translate(1,x)
		time.sleep(.1)

	if x < 0:
		print("x backward")
		#x_translate(0,x)
		time.sleep(.1)

	if y > 0:
		print("y forward")
		#y_translate(1,y)
		time.sleep(.1)

	if y < 0:
		print("y backward")
		#y_translate(0,y)
		time.sleep(.1)

	# read mocap data and see if the drone moved far enough
	# in each direction and if not, then rerun this move function
	# error_x = x - mocap_x
	# error_y = y - mocap_y

	# if abs(error_x) > MOCAP_ERROR: 
	# 	set_pose_destination(error_x, 0)

	# if abs(error_y) > MOCAP_ERROR: 
	# 	set_pose_destination(0, error_y)
	mocap_x = mocap_x + x
	mocap_y = mocap_y + y
	return


def mirco_plan(path):
	global current_path, STATE
	#current ith path
	i = current_path

	#current x, y coords
	#x,y are not -1
	if len(path)-1-i >= 0:
		#if path[i][0] != -1 and path[i][1] != -1:
		cur_x = path[i][0]
		cur_y = path[i][1]

	#next x,y coords
	#if we have one extra path ahead
	#and x,y are not -1
	if len(path)-1-i >= 1:
		#if path[i+1][0] != -1 and path[i+1][1] != -1:
		next_x = path[i+1][0]
		next_y = path[i+1][1]

	# checking x coord
	if next_x == -10 and next_y == -10:
		print("Arrived destination")
		STATE = 4
		return

	print("set_pose_dest", next_x,next_y)
	set_pose_destination(next_x,next_y)
	current_path = current_path + 1 
	return

def update_pose(data):
	global mocap_x, mocap_y, mocap_theta #,mocap_z
	mocap_x = data.pose.position.x
	mocap_y = data.pose.position.y
	#mocap_z = data.pose.position.z
	mocap_theta = data.pose.position.theta

def main():
	#initialize ros nodes
	global drone_pub, STATE, current_path, FLIGHT_TIME
	rospy.init_node("statemachine", anonymous=True)
	drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
	takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
	landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)
	ar_tags = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, planner.return_area_detected)
	#movo = rospy.Subscriber("/vrpn_client_node/movo/pose", PoseStamped, update_pose)
	world_map = planner.WorldMap()

	time.sleep(1.)
    #takeoff and start run timer
	#takeoff_pub.publish(Empty())
	#time.sleep(3.)
	print ("Takeoff! Flight time is %f." % FLIGHT_TIME)
	
	path = []
	last_call = time.time()
	start_time = time.time()
	print(last_call)
	#while loop for states and flight time
	while not rospy.is_shutdown() and time.time() - start_time < FLIGHT_TIME:
		if STATE == 0:
			if time.time() - last_call > READ_INTERVAL:
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				print(data)
				print(data[0][0][0])
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)
				#world_map.print_obs_map()
				print ("NOW Rotate 0")
				#turn clockwise 50 deg to read ar tags
				#rotate(1,50)
				#print(pose_theta)
				last_call = time.time()
				print(last_call)
				STATE = 1

		if STATE == 1:
			if time.time() - last_call > READ_INTERVAL:
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)
				#world_map.print_obs_map()
				print ("NOW Rotate 1")
				#turn to counter-clockwise 50 deg to read ar tags
				#rotate(0,100)
				#print(pose_theta)
				last_call = time.time()
				print(last_call)
				STATE = 2

		if STATE == 2:
			if time.time() - last_call > READ_INTERVAL:
				#return to forward and load into map
				print ("Add to map")
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)

				world_map.print_obs_map()
				ar_tags.unregister()
				print ("NOW Rotate 2")
				#rotate(1,50)
				#print(pose_theta)
				last_call = time.time()
				print(last_call)
				time.sleep(1.)
				#landing_pub.publish(Empty())
				print ("Shutdown")
				a_star_out, cost_so_far = astar.a_star(world_map, (0,0), (data[0][0][4],data[0][1][4]))
				i,j = world_map._world_to_map(data[0][0][4],data[0][1][4])
				#print(a_star_out)
				path = astar.path(a_star_out, (0,0), world_map._map_to_world(i,j))
				print (path)
				print("Done with A_Star")
				#print(path[1][1])
					# if path[1][1]>path[1][0]: #traversing y initially
				time.sleep(1.)
				STATE = 3
		
		if STATE == 3:
			mirco_plan(path)
		
		if STATE == 4:
			print("DONE!")
			FLIGHT_TIME = 0
    		#landing_pub.publish(Empty())
			#time.sleep(3.)


	#landing_pub.publish(Empty())
	print ("Shutdown")

if __name__ == '__main__':
	main()
