#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import tf
import time
import dlib
import pathPlanner as planner
import Astar as astar

FLIGHT_TIME = 25.0
STATE = 0
READ_INTERVAL = 2.0
PI = 3.1415926535897
angular_speed = 45*2*PI/360 	#~45 degrees per second
linear_speed = 0.5

mocap_x = 0
mocap_y = 0
#mocap_theta = 0
current_path = 0

# rotate takes in two arguments, a clockwise flag and an angle
# to move to. Clockwise is a bool and the angle should be an int
# between 0 and 360. It will publish velocities to the drone until 
# the timing approximation believes it moved to the correct theta.
def rotate(clockwise,angle):
	global drone_pub
	#creating the movement message
	vel_msg = Twist()		
	if clockwise:
		vel_msg.angular.z = -abs(angular_speed)
	else:
		vel_msg.angular.z = abs(angular_speed)

	t0 = time.time()
	current_angle = 0
	#pose to move to with hardcoded angle tuning
	relative_angle = (angle/2)*2*PI/360 
	#while loop of publishing to the drone
	while(current_angle < relative_angle):
		drone_pub.publish(vel_msg)
		t1 = time.time()
		current_angle = angular_speed*(t1-t0)

	#Forcing our robot to stop before returning
	vel_msg.angular.z = 0
	drone_pub.publish(vel_msg)
	return

# x_translate takes in two arguments, a direction flag and a length
# to move to. Direction is a bool with forward as True and the length
# should be in meters. It will publish velocities to the drone until 
# the timing approximation believes it moved to the correct x.
def x_translate(direction,length):
	global drone_pub
	#forward = 1
	#creating the movement message
	vel_msg = Twist()
	if direction:
		vel_msg.linear.x = abs(linear_speed) 
	else:
		vel_msg.linear.x = -abs(linear_speed) 

	t0 = time.time()
	current_linear = 0
	#pose to move to with hardcoded distance corrections
	relative_linear = length - length*0.65 
	print(relative_linear)
	#while loop for publishing to the drone
	while(current_linear < relative_linear):
		drone_pub.publish(vel_msg)
		t1 = time.time()
		current_linear = linear_speed*(t1-t0)

	#Forcing our robot to stop before returning
	vel_msg.linear.x = 0
	drone_pub.publish(vel_msg)
	return

# y_translate takes in two arguments, a direction flag and a length
# to move to. Direction is a bool with forward as True and the length
# should be in meters. It will publish velocities to the drone until 
# the timing approximation believes it moved to the correct y.
def y_translate(direction,length):
	global drone_pub
	#forward = 1
	#creating the movement message
	vel_msg = Twist()
	if direction:
		vel_msg.linear.y = abs(linear_speed) 
	else:
		vel_msg.linear.y = -abs(linear_speed)

	t0 = time.time()
	current_linear = 0
	#pose to move to with hardcoded distance corrections
	relative_linear = length - length*0.65 
	#while loop for publishing to the drone
	while(current_linear < relative_linear):
		drone_pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_linear = linear_speed*(t1-t0)

	#Forcing our robot to stop before returning
	vel_msg.linear.y = 0
	drone_pub.publish(vel_msg)
	return

# set_pose_destination takes in two arguments, an x and y pose in meters.
# The function will call the appropriate movement functions based on the 
# sign of the passed poses. It also checks current pose before assigning
# movement commands.
def set_pose_destination(pose_x,pose_y):
	global mocap_x, mocap_y # mocap_theta
	# read current poses and create actual movement command
	x = pose_x - mocap_x
	y = pose_y - mocap_y
	#theta = pose_theta - mocap_theta
	print(x,y)

	# break move commands into respective functions
	# moves should be in meters. sleep commands are for
	# safety and to not publish to the drone too rapidly
	if x > 0:
		print("x forward")
		x_translate(1,x)
		time.sleep(.5)

	if x < 0:
		print("x backward")
		x_translate(0,x)
		time.sleep(.5)

	if y > 0:
		print("y forward")
		y_translate(1,y)
		time.sleep(.5)

	if y < 0:
		print("y backward")
		y_translate(0,y)
		time.sleep(.5)
	
	time.sleep(.5)
	return

# micro_plan takes an array of poses and iterates along that path,
# passing the commands to set_pose_destination until it reaches the
# goal which was hard coded to be (-10,-10) so it would be outside
# of any possible map pose.
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

	# checking next coord
	if next_x == -10 and next_y == -10:
		print("Arrived destination")
		STATE = 4
		return

	print("set_pose_dest", next_x,next_y)
	set_pose_destination(next_x,next_y)
	current_path = current_path + 1 
	return

# update_pose is ROS subscriber function to read the motion capture
# data and save it to the global positions.
def update_pose(data):
	global mocap_x, mocap_y #mocap_theta, mocap_z
	# axis was flipped and offset from how we were moving the drone
	# so the mapings were inverted here instead of creating a static TF
	mocap_x = -data.pose.position.y + 1
	mocap_y = data.pose.position.x
	#mocap_z = data.pose.orientation.z  #didn't read theta because of quaternions
	return


def main():
	#initialize ros nodes
	global drone_pub, STATE, current_path, FLIGHT_TIME
	rospy.init_node("statemachine", anonymous=True)
	drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
	takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
	landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)
	ar_tags = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, planner.return_area_detected)
	movo = rospy.Subscriber("/vrpn_client_node/RigidBody01/pose", PoseStamped, update_pose)
	#br = tf.TransformBroadcaster() #for fixing some AR tag rotations
	world_map = planner.WorldMap()

	time.sleep(1.)
    #takeoff and start run timer
	takeoff_pub.publish(Empty())
	time.sleep(3.)
	print ("Takeoff! Flight time is %f." % FLIGHT_TIME)
	
	path = []
	last_call = time.time()
	start_time = time.time()
	print(last_call)
	landing = []
	#while loop for states and flight time
	while not rospy.is_shutdown() and time.time() - start_time < FLIGHT_TIME:
		#br.sendTransform((0.0,0.0,0.0),tf.transformations.quaternion_from_euler(0.0,0.0,-0.70), rospy.Time.now(),"fixed","odom")
		if STATE == 0:
			if time.time() - last_call > READ_INTERVAL:
				#reset drone to origin and read the data
				set_pose_destination(0,0)
				time.sleep(.5)
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				print(data)
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							#save the landing data if found
							landing = (data[0][0][index],data[0][1][index])
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)
				#world_map.print_obs_map()
				print ("NOW Rotate 0")
				#turn clockwise 50 deg to read ar tags on the right
				time.sleep(.5)
				rotate(1,65) # initial drift of the drone added into this
				last_call = time.time()
				print(last_call)
				STATE = 1

		if STATE == 1:
			if time.time() - last_call > READ_INTERVAL:
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				print(data)
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							#save the landing data if found
							landing = (data[0][0][index],data[0][1][index])
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)
				#world_map.print_obs_map()
				print ("NOW Rotate 1")
				#turn to counter-clockwise 50 deg to read ar tags on the left
				time.sleep(.5)
				rotate(0,100)
				last_call = time.time()
				print(last_call)
				STATE = 2

		if STATE == 2:
			if time.time() - last_call > READ_INTERVAL:
				#return to forward and load into map
				print ("Add to map")
				data = planner.currentAreaCoor #([lowtuple], [hightuple])
				print(data)
				for index in range(0,6):
					if data[0][0][index] != "NO_DETECTION":
						if index == 4:
							landing = (data[0][0][index],data[0][1][index])
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 3)
						else:
							world_map.set_feature((data[0][0][index],data[0][1][index]), (data[1][0][index],data[1][1][index]), 1)

				#unsuscribe from reading the AR_Tags and print the map to terminal
				ar_tags.unregister()
				world_map.print_obs_map()
				#return to (0,0) and 0 theta for proper movement in map
				print ("NOW Rotate 2")
				set_pose_destination(0,0)
				time.sleep(.5)
				rotate(1,50)
				last_call = time.time()
				print(last_call)
				#testing and printing conditions before movement
				print(landing)
				landing = (round(landing[0],1), round(landing[1],1)) 
				print(landing)
				#run A* algorithim in map
				a_star_out, cost_so_far = astar.a_star(world_map, (0,0), landing)
				#print(a_star_out)
				# for key,value in a_star_out:
				# 	print(key)
				#recreate the path A* created
				path = astar.path(a_star_out, (0,0), landing)
				print (path)
				print("Done with A_Star")
				time.sleep(3.)
				STATE = 3
		
		if STATE == 3:
			#path planner based on the created path
			mirco_plan(path)
		
		if STATE == 4:
			print("DONE!")
			time.sleep(1.)
			landing_pub.publish(Empty())
			FLIGHT_TIME = 0
			#time.sleep(3.)


	landing_pub.publish(Empty())
	print ("Shutdown")

if __name__ == '__main__':
	main()
