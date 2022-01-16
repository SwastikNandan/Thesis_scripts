import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
import math
import cv2
import time
import numpy as np
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from mavros_msgs.srv import SetMode, CommandBool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Header
from sensor_msgs.msg import Range
import subprocess
import multiprocessing

class OffboardControl:

    """ Controller for PX4-UAV offboard mode """
    def __init__(self):
        self.curr_pose1 = PoseStamped()
        self.flag = "False"
        self.is_ready_to_fly1 = False
        self.hover_loc1 = [-6 + 12, 0, 3, 0, 0, 0, 0] # Hovers 3meter above at this location 
        self.mode1 = "HOVER"
        self.mode2 = "HOVER"
        self.waypointIndex = 0
        self.sim_ctr = 1
        self.arm1 = True
        self.range = 0
        self.dist_threshold = 0.04
        self.detect_interval = 1
        self.keypoints = []

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub1 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback1)
        self.state_sub1 = rospy.Subscriber('uav1/mavros/state', State, callback=self.state_callback1)
        self.vel_pub1 = rospy.Publisher('uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.controller()

    def pose_callback1(self, msg):
        self.curr_pose1 = msg

    def state_callback1(self, msg):
        if msg.mode == 'OFFBOARD':
            self.is_ready_to_fly1 = True
        else:
            self.take_off1()

    def get_descent(self,x,y,z):
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp = rospy.Time.from_sec(time.time())
        des_vel.coordinate_frame = 8
        des_vel.type_mask = 3527
        des_vel.velocity.x = x
        des_vel.velocity.y = y
        des_vel.velocity.z = z
        return des_vel

    def set_offboard_mode1(self):
        rospy.wait_for_service('uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm1(self):
        rospy.wait_for_service('uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm1 = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def take_off1(self):
        self.set_offboard_mode1()
        self.set_arm1()

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def descent1(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            err_x = self.hover_loc1[0] - self.curr_pose1.pose.position.x 
            err_y = self.hover_loc1[1] - self.curr_pose1.pose.position.y 
            err_z = self.hover_loc1[2] - self.curr_pose1.pose.position.z
            print(self.curr_pose1.pose.position.x, self.curr_pose1.pose.position.y, self.curr_pose1.pose.position.z)

            x_change = err_x * (-0.05)
            y_change = err_y * (-0.05)
            z_change = err_z * (0.2)
            #print("x_change:",x_change, "y_change:", y_change, "z_change", z_change)
            des = self.get_descent(x_change, y_change, z_change)
            self.vel_pub1.publish(des)

    def hover1(self):
        """ hover at height mentioned in location
        set mode as HOVER to make it work
        """
        location = self.hover_loc1
        loc = [location,
        location,
        location,
        location,
        location]
        #print(loc)

        rate = rospy.Rate(20)
        shape = len(loc)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose1)
        waypoint_index = 0
        sim_ctr = 1

        print("I am in hover1")

        while self.mode1 == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                print("HOVER COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose1.pose.position.x
            curr_y = self.curr_pose1.pose.position.y
            curr_z = self.curr_pose1.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < self.dist_threshold:
                waypoint_index += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def controller(self):

        print("Controller")
        while not rospy.is_shutdown():
            if self.mode1 == "HOVER":
                self.hover1()
                #print("I am here")
                #p1 = multiprocessing.Process(target = self.hover1)
                #p1.start()
                #p2 = multiprocessing.Process(target = self.hover2)
                #p2.start()
                #p1.join()
                #p2.join()

if __name__ == "__main__":
    OffboardControl()