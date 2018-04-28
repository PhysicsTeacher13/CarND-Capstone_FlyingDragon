#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
	self.count =0;
        self.pose = None
        self.waypoints = None
        self.waypoint_tree = None

        self.final_waypoints = None
        self.final_waypoints_2d = None
        self.final_waypoint_tree = None

        self.light_tree = None

        self.waypoints_2d = None
        self.lights_waypoints_2d = None
	self.traffic_light_idxs = []
	self.stop_line_idxs = []

        self.camera_image = None
	self.has_image = False
        self.lights = []
	self.IMAGECAPTURE = False
	self.image_dir = "images/"
	self.currentNextLight = None
	self.foundStopLinesIdxs = False


	rospy.loginfo("Starting TL")
	sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
#        sub3 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

# get rid of the spin to try to improve latency
#        rospy.spin()
	self.loop()

    def loop(self):
# run at 10hz

	    rate = rospy.Rate(1)
	    while not rospy.is_shutdown():
 #                   rospy.loginfo("loop")
		    if self.has_image:                    
			rospy.loginfo("loop2")
		        light_wp, state = self.process_traffic_lights()

			if self.state != state:
			    self.state_count = 0
			    self.state = state
			elif self.state_count >= STATE_COUNT_THRESHOLD:
			    self.last_state = self.state
			    light_wp = light_wp if state == TrafficLight.RED else -1
			    self.last_wp = light_wp
			    self.upcoming_red_light_pub.publish(Int32(light_wp))
			else:
			    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
			self.state_count += 1
		    rate.sleep()	

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
	rospy.loginfo("waypoints_cb")

	if not self.waypoints_2d:
# run this only once
	        self.waypoints = waypoints
		rospy.loginfo('way_call 2')
# create a KD tree of the base waypoints
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)


    def final_waypoints_cb(self, waypoints):
#	rospy.loginfo("final_waypoints_cb")

	if not self.final_waypoints_2d:
# run this only once
	        self.final_waypoints = waypoints
		rospy.loginfo('way_call 2: {}'. format(self.final_waypoints.waypoints[0]))
# create a KD tree of the base waypoints
		self.final_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
		self.final_waypoint_tree = KDTree(self.final_waypoints_2d)




    def traffic_cb(self, msg):
#	rospy.loginfo("traffic_cb")
# get the light status on every received msg
	self.lights = msg.lights
	if not self.lights_waypoints_2d and self.waypoints_2d:
		rospy.loginfo("traffic_cb2")
#		rospy.loginfo("traffic_cb: {}" .format(msg.lights))
#		self.lights = msg.lights


		if not self.foundStopLinesIdxs:
			rospy.loginfo("traffic_cb2")
			foundStopLinesIdxs = True
			self.lights_waypoints_2d = [[light.pose.pose.position.x, light.pose.pose.position.y] for light in self.lights]
	#		self.lights_tree = KDTree(self.lights_waypoints_2d)
			

	# just want to set up the waypoints etc once
	
			for x,y in self.lights_waypoints_2d:
				rospy.loginfo("traffic_cb: x:{0}, y:{1}" .format(x,y))
				self.traffic_light_idxs.append(self.get_closest_waypoint(x,y))


			stop_line_positions = self.config['stop_line_positions']
			for i, line_pos in enumerate(stop_line_positions):
				# Get stop line waypoint index
				#line = stop_line_positions[i]
				#temp_light_wp_idx = self.traffic_light_idxs[i]

				rospy.loginfo("traffic_cb2 calling get closest waypoint")
				self.stop_line_idxs.append(self.get_closest_waypoint(line_pos[0], line_pos[1]))

			rospy.loginfo("traffic_cb: should have all the idx of lines and lights")
			rospy.loginfo("traffic_cb: traffic_light_idxs: {}" .format(self.traffic_light_idxs))
			rospy.loginfo("traffic_cb: stop_line_idxs: {}" .format(self.stop_line_idxs))



    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

# this runs at the camera capture rate. might be worth considering controlling the rate for efficency
	rospy.loginfo("image_cb")
        self.has_image = True
        self.camera_image = msg


# image capture (training data)
	if (self.IMAGECAPTURE):
		self.count = self.count+1
		filename = self.image_dir +'img_{0}_{1}.png'.format(self.count, state)
		img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		cv2.imwrite(filename , img)



    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

	if self.waypoint_tree:
       		closest_idx = self.waypoint_tree.query([x, y], 1)[1]
	        return closest_idx
	else:
		rospy.loginfo("get_closest_waypoint: no waypoint_tree " )
		return -1


    def get_closest_light_wp(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
#	rospy.loginfo("get_closest_light_wp\t pose: {0},{1}" .format(x,y))
#	rospy.loginfo("light_tree:" .format(self.light_tree))
	if self.light_tree:
# query the light KD tree
	        closest_idx = self.light_tree.query([x, y], 1)[1]
	else:
		closest_idx = -1  
		rospy.loginfo("get_closest_light_wp: no light tree" )
	return closest_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For testing, just return the light state
        return light.state

        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.loginfo(" process_traffic_lights" )
        # light = None
        closest_light = None
        line_wp_idx = None
	light_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']


        if(self.pose):
# get the idx of the nearest waypoint to the cars pose
# this was causing the simulator and vehicle control to run REALLY slow. Abandoned for now (10902 waypoints!))
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
#	    car_wp_idx = 200

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)




# iterate through the array of lights. Used instead of a KDtree. 
# iterate through and find the closest waypoint
# note that currently check for closest waypint in front of vehicle. There is the issue where the vehicle has just
# crossed the line due to the lights changing on approach

# instead of finding the nearest of the base waypoints to the cars position, find the nearest base waypoint to tl position. 
# that only needs to be done once, then we have to work with 8 points instead of 10902

#            rospy.loginfo(" lights: {}" .format(self.lights))
	    
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
		temp_light_wp_idx = self.traffic_light_idxs[i]

                temp_line_wp_idx = self.stop_line_idxs[i]

#	        rospy.loginfo("light: {0}, line: {1}".format(temp_line_wp_idx, line))
# line[0], line[] are the x and y values of the pose of the lights
# gets the 
#		rospy.loginfo("light: {}".format(light))
#          	rospy.loginfo(" light idx: {0}, light idx: {1}, closest light state: {2}" .format(temp_line_wp_idx, temp_light_wp_idx, self.get_light_state(light)))
#                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_line_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
		    self.currentNextLight = closest_light
                    line_wp_idx = temp_line_wp_idx
                    light_wp_idx = temp_light_wp_idx

#                i = i+1

        if closest_light:
            state = self.get_light_state(closest_light)
            rospy.loginfo("closest light idx: {0},closest light idx: {1}, closest light state: {2}, car wp: {3}" .format(line_wp_idx, light_wp_idx, state, car_wp_idx))
#  	        if self.currentNextLight != closest_light:
#            rospy.loginfo("closest light: {}" .format(closest_light.pose))

            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
