#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import time
import pathPlanner as planner
import Astar as astar

FLIGHT_TIME = 25
STATES = {0: "first_turn", 1: "second_turn", 2: "run_Astar", 3: "temp", 4: "land" }
STATE = STATES[0]
READ_INTERVAL = 1
PI = 3.1415926535897
angular_speed = 45*2*PI/360 	#45 degrees per second
relative_angle = angle*2*PI/360

def rotate():
	return

def main():
	global drone_pub
	drone_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
	takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
	landing_pub = rospy.Publisher("/bebop/land", Empty, queue_size=1)
    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, return_area_detected)
    world_map = planner.WorldMap()

    #takeoff and start run timer
	takeoff_pub.publish(Empty())
	print "Takeoff! Flight time is %f." % FLIGHT_TIME
	time.sleep(1.)
	last_call = time.time()
	start_time = time.time()

	while not rospy.is_shutdown() and time.time() - start_time < FLIGHT_TIME:
		if STATE == "first_turn":
			arTagAverage()
			if time.time() - last_call > READ_INTERVAL:
				last_call = time.time()
				vel_msg = Twist()
				vel_msg.angular.z = abs(angular_speed) 
				t0 = rospy.Time.now().to_sec()
    			current_angle = 0

    			while(current_angle < relative_angle):
        			velocity_publisher.publish(vel_msg)
        			t1 = rospy.Time.now().to_sec()
        			current_angle = angular_speed*(t1-t0)


    			#Forcing our robot to stop
    			vel_msg.angular.z = 0
    			velocity_publisher.publish(vel_msg)
    		else:
    			STATE = "second_turn"

    	if STATE == "second_turn":

    	if STATE == "run_Astar":

    	if STATE == "temp":

    	if STATE == "land":
    		break

    landing_pub.publish(Empty())
    print "Shutdown"

if __name__ == '__main__':
	main()
