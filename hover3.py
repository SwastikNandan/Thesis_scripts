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
import random
import sys

class OffboardControl:

    """ Controller for PX4-UAV offboard mode """
    def __init__(self):
        self.curr_pose2 = PoseStamped()
        self.curr_pose1 = PoseStamped()
        self.curr_pose1_1 = PoseStamped()
        self.curr_pose2_1 = PoseStamped()
        self.completion_tag = "Not_done"
        self.flag = "False"
        self.is_ready_to_fly2 =False
        self.hover_loc2 = [2 , 0, 5, 0, 0, 0, 0]
        self.mode2 = "HOVER"
        self.direction = None
        self.arm2 = True
        self.dist_threshold = 0.1
        self.step_size = 1
        print("a is:", a)
        print("b is:", b)
        print("c is:", c)
        self.alpha = float(a)
        self.beta = float(b)
        self.gamma = float(c)
        self.keypoints = []

        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
        self.pose_sub1 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback2)
        self.pose_sub2 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback1)
        self.state_sub2 = rospy.Subscriber('uav1/mavros/state', State, callback=self.state_callback2)
        self.vel_pub1 = rospy.Publisher('uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.controller()

    def pose_callback1(self, msg):
        self.curr_pose1_1 = msg
        self.curr_pose1.pose.position.x = self.curr_pose1_1.pose.position.x - 6
        self.curr_pose1.pose.position.y = self.curr_pose1_1.pose.position.y 
        self.curr_pose1.pose.position.z = self.curr_pose1_1.pose.position.z
        self.curr_pose1.pose.orientation.x = self.curr_pose1_1.pose.orientation.x
        self.curr_pose1.pose.orientation.y = self.curr_pose1_1.pose.orientation.y
        self.curr_pose1.pose.orientation.z = self.curr_pose1_1.pose.orientation.z
        self.curr_pose1.pose.orientation.w = self.curr_pose1_1.pose.orientation.w

    def pose_callback2(self, msg):
        self.curr_pose2_1 = msg
        self.curr_pose2.pose.position.x = self.curr_pose2_1.pose.position.x - 12
        self.curr_pose2.pose.position.y = self.curr_pose2_1.pose.position.y 
        self.curr_pose2.pose.position.z = self.curr_pose2_1.pose.position.z
        self.curr_pose2.pose.orientation.x = self.curr_pose2_1.pose.orientation.x
        self.curr_pose2.pose.orientation.y = self.curr_pose2_1.pose.orientation.y
        self.curr_pose2.pose.orientation.z = self.curr_pose2_1.pose.orientation.z
        self.curr_pose2.pose.orientation.w = self.curr_pose2_1.pose.orientation.w

    def state_callback2(self, msg):
        if msg.mode == 'OFFBOARD':
            self.is_ready_to_fly2 = True
        else:
            self.take_off2()

    def set_offboard_mode2(self):
        rospy.wait_for_service('uav1/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm2(self):
        rospy.wait_for_service('uav1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm2 = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)


    def take_off2(self):
        self.set_offboard_mode2()
        self.set_arm2()

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def hover2(self):
        """ hover at height mentioned in location
        set mode as HOVER to make it work
        """
        self.completion_tag = "Not_done"
        location = self.hover_loc2
        x_ = location[0]
        y_ = location[1]
        z_ = location[2]
        destination_pose = [x_ , y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        #print(destination_pose)

        rate = rospy.Rate(20)
        shape = len(loc)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("HOVER COUNTER: " + str(sim_ctr))
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

            curr_x = self.curr_pose2.pose.position.x + 12
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            #print("The current pose is:", [curr_x, curr_y, curr_z])
            #print("The destination pose is:", [des_x, des_y, des_z])
            if dist < self.dist_threshold:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                self.mode2 = "Go_straight"
                break
            rate.sleep()

    def go_straight(self, step):
        #print("I am in go_straight")
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ + step, y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_straight" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def go_back(self, step):
        #print("I am in go_back")
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ - step, y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_back" and not rospy.is_shutdown():
            #print("I am in the while loop")
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0
            #print("I am here 1")
            #print(self.des_pose)

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.2:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def go_left(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_ + step, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_left" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def go_right(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_ - step, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_right" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def go_up(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_ + step, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_up" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def go_down(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_ - step, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Go_down" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def repulsion(self, step, axis_name):
        #print("I am in repulsion")
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_, 0, 0, 0]
        if axis_name == '-x':
            destination_pose = [x_ - step , y_, z_ , 0, 0, 0]

        if axis_name == 'x':
            destination_pose = [x_ + step, y_, z_, 0, 0, 0]

        if axis_name == '-y':
            destination_pose = [x_ , y_ - step, z_, 0, 0, 0]

        if axis_name == 'y':
            destination_pose = [x_ , y_ + step, z_, 0, 0, 0]

        if axis_name == '-z':
            destination_pose = [x_ , y_, z_ - step, 0, 0, 0]

        if axis_name == 'z':
            destination_pose = [x_ , y_, z_ + step, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Avoid_collision" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()

    def hit_boundary(self, step, axis_name):
        #print("I am in hit boundary")
        #print("The direction is:", axis_name)
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_, 0, 0, 0]
        if axis_name == '-x':
            destination_pose = [x_ - step, y_, z_ , 0, 0, 0]

        if axis_name == 'x':
            destination_pose = [x_ + step, y_, z_, 0, 0, 0]

        if axis_name == '-y':
            destination_pose = [x_ , y_ - step, z_, 0, 0, 0]

        if axis_name == 'y':
            destination_pose = [x_ , y_ + step, z_, 0, 0, 0]

        if axis_name == '-z':
            destination_pose = [x_ , y_, z_ - step, 0, 0, 0]

        if axis_name == 'z':
            destination_pose = [x_ , y_, z_ + step, 0, 0, 0]

        #print("The current pose is:", [x_ , y_, z_])

        #print("The destination pose is:", destination_pose)
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "Bounce_back" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            #print("The distance is:", dist)
            if dist < 0.1:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                break
            rate.sleep()



    def mode_decider(self):
        print("I am in mode decider")
        mode_ = None
        robot2_position = self.curr_pose2
        robot1_position = self.curr_pose1
        x1 = robot1_position.pose.position.x
        y1 = robot1_position.pose.position.y
        z1 = robot1_position.pose.position.z
        x2 = robot2_position.pose.position.x
        y2 = robot2_position.pose.position.y
        z2 = robot2_position.pose.position.z
        #print("The position of robot1 is:", (x1, y1, z1))
        #print("The position of robot2 is:", (x2, y2, z2))
        x_diff = x2 - x1
        y_diff = y2 - y1
        z_diff = z2 - z1
        d = {}
        d[1] = abs(x_diff)
        d[2] = abs(y_diff)
        d[3] = abs(z_diff)
        min_key = min(d, key=d.get)
        #min_val = min(d.values())
        dist = math.sqrt((x_diff)*(x_diff) + (y_diff)*(y_diff) + (z_diff)*(z_diff))
        if dist < 3:
            #print("The d array is:", d)
            #print("The min value is:", min_val)
            mode_ = "Avoid_collision"
            dir_ = 'x'
            if min_key == 1:
                if x_diff < 0:
                    dir_ = '-x'
                else:
                    dir_ = 'x'
            if min_key == 2:
                if y_diff < 0:
                    dir_ = '-y'
                else:
                    dir_ = 'y'
            if min_key == 3:
                if z_diff < 0:
                    dir_ = '-z'
                else:
                    dir_ = 'z'
            return [mode_, dir_]
        if x2 > 10:
            mode_ = "Bounce_back"
            dir_ = '-x'
            return [mode_, dir_]
        if x2 < -10:
            mode_ = "Bounce_back"
            dir_ = 'x'
            return [mode_, dir_]

        if y2 > 10:
            mode_ = "Bounce_back"
            dir_ = '-y'
            return [mode_, dir_]

        if y2 < -10:
            mode_ = "Bounce_back"
            dir_ = 'y'
            return [mode_, dir_]

        if z2 > 4:
            mode_ = "Bounce_back"
            dir_ = '-z'
            return [mode_, dir_]

        if z2 < 2:
            mode_ = "Bounce_back"
            #print("Bouncing upward")
            dir_ = 'z'
            return [mode_, dir_]

        random_value1 = random.uniform(0, 1)
        random_value2 = random.uniform(0, 1)
        #random_value1 = 0.20
        #random_value2 = 0.80
        #print("The value of the random number 1 is:", random_value1)
        mode_ = None
        if random_value1 <= self.alpha and random_value2 <= self.gamma:
            mode_ = "Go_straight"
            return [mode_, None]
        if random_value1 <= self.alpha and random_value2 > self.gamma: 
            mode_ = "Go_back"
            return [mode_, None]
        if random_value1 <= self.beta and random_value1 > self.alpha and random_value2 <= self.gamma:
            mode_ = "Go_left"
            return [mode_, None]
        if random_value1 <= self.beta and random_value1 > self.alpha and random_value2 > self.gamma:
            mode_ = "Go_right"
            return [mode_, None]
        if random_value1 > self.beta and random_value2 <= self.gamma:
            mode_ = "Go_up"
            return [mode_, None]
        if random_value1 > self.beta and random_value2 > self.gamma:
            mode_ = "Go_down"
        print("The mode is:", mode_)
        return [mode_, None]


    def controller(self):

        print("Controller")
        while not rospy.is_shutdown():
            if self.mode2 == "HOVER":
                self.hover2()
                arr = self.mode_decider()
                #print("I amm here")
                self.mode2 = arr[0]
                #self.mode2 = "HOVER"
                self.direction = arr[1]

            if self.mode2 == "Go_straight":
                self.go_straight(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Go_back":
                self.go_back(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Go_left":
                self.go_left(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Go_right":
                self.go_right(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Go_up":
                self.go_up(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Go_down":
                self.go_down(1)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Avoid_collision":
                self.repulsion(1,self.direction)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

            if self.mode2 == "Bounce_back":
                self.hit_boundary(1,self.direction)
                arr = self.mode_decider()
                self.mode2 = arr[0]
                self.direction = arr[1]

if __name__ == "__main__":

    # total arguments
    a = sys.argv[1]
    b = sys.argv[2]
    c = sys.argv[3]
    OffboardControl()
