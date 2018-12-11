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

FLIGHT_TIME = 15.0
STATE = 2
READ_INTERVAL = 3.0
PI = 3.1415926535897
angular_speed = 45*2*PI/360 	#45 degrees per second
linear_speed = 0.5
MOCAP_ERROR = 0.1

mocap_x = 0
mocap_y = 0
mocap_z = 0
mocap_theta = 0

pose_x = 0
pose_y = 0
pose_z = 1
pose_theta = 0

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

# def mirco_plan(path):
# 	global STATE, current_path
#     i = current_path
#     	#current x, y coords
#     cur_x = path[i][0]
#     cur_y = path[i][1]

#     #next x,y coords
#     if path[i+1][0] != -1 and path[i+1][1] != -1:
#         next_x = path[i+1][0]
#         next_y = path[i+1][1]
    
#     #next 2 of x, y
#     if not path[i+2][0] and not path[i+2][1]:
#         next2_x = path[i+2][0]
#         next2_y = path[i+2][1]

#     if next_y != next2_y:
#         if next_y < next2_y:
#             print("set_pose_dest((x, y), -90.0)")
            
#         elif next_y > next2_y:
#             print("set_pose_dest((x, y), 90.0)")
        
#     elif next_y == next2_y:
#     	print("set_pose_dest((x, y), 0.0)")

#     STATE = 5
#     i+=1

# all in meters and 
def set_pose_destination(x,y,theta):
	if x > 0:
		x_translate(1,x)
		time.sleep(.2)

	if x < 0:
		x_translate(0,x)
		time.sleep(.2)

	if y > 0:
		y_translate(1,y)
		time.sleep(.2)

	if y < 0:
		y_translate(0,y)
		time.sleep(.2)

	if theta < 0:
		rotate(1,angle)
		time.sleep(.2)

	if theta > 0:
		rotate(0,angle)
		time.sleep(.2)

	error_x = mocap_x - pose_x
	error_y = mocap_y - pose_y
	error_theta = mocap_theta - pose_theta

	if abs(error_x) > MOCAP_ERROR: 
		set_pose_destination(error_x, 0, 0)

	if abs(error_y) > MOCAP_ERROR: 
		set_pose_destination(0, error_y, 0)

	if abs(error_theta) > MOCAP_ERROR:
		set_pose_destination(0, 0, error_theta) 


def main():
	#initialize ros nodes
	global drone_pub, STATE, current_path
	rospy.init_node("statemachine", anonymous=True)
	drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
	takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
	landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)
	ar_tags = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, planner.return_area_detected)
	world_map = planner.WorldMap()

	time.sleep(1.)
    #takeoff and start run timer
	#takeoff_pub.publish(Empty())
	#time.sleep(3.)
	print ("Takeoff! Flight time is %f." % FLIGHT_TIME)
		
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
				#turn clockwise 45 deg to read ar tags
				#rotate(1,45)
				print(pose_theta)
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
				#turn to counter-clockwise 45 deg to read ar tags
				#rotate(0,90)
				print(pose_theta)
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
				#rotate(1,45)
				print(pose_theta)
				last_call = time.time()
				print(last_call)
				time.sleep(1.)
				#landing_pub.publish(Empty())
				print ("Shutdown")
				a_star_out = astar.a_star(world_map, (0,0), (data[0][0][4],data[0][1][4]))
				#path = astar.path(a_star_out, (0,0), (data[0][0][4],data[0][1][4]))
				#print path
				time.sleep(10.)
				STATE = 3

    	if STATE == 3:
    		temp = 0
    	 	micro_plan(path) 

    	if STATE == 5:
    		temp = 0

	#landing_pub.publish(Empty())
	print ("Shutdown")

if __name__ == '__main__':
	main()
