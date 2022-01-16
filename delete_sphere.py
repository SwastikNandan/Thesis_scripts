import rospy
import collections
import rospkg
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import DeleteModel
from rosgraph_msgs.msg import Clock
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np

class RobotActionsServer:
    def __init__(self, sphere_dict1, dc):
        self.deleted_queue = dc
        self.sim_time = float()
        self.sphere_dict = sphere_dict1
        self.uav0_location = (0.0, 0.0, 0.0)
        self.uav1_location = (4.0, 4.0, 4.0)
        rospy.init_node('delete_spheres', anonymous=True)
        self.pose_sub_0 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_0)
        self.pose_sub_1 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_1)
        self.simtime = rospy.Subscriber('/clock', Clock, callback= self.clock_back )
        rospy.wait_for_service('/gazebo/delete_model')
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)
        while not rospy.is_shutdown():
            self.delete_spheres()

    def clock_back(self, t):
        self.sim_time = t.clock.secs
        #print("The sim time is:", self.sim_time)
        #print("The length of the deleted queue is:", len(self.deleted_queue))
        if len(self.deleted_queue) > 0:
            sphere_name = self.deleted_queue[-1][0]
            last_time = self.deleted_queue[-1][1]
            elem_cood = self.deleted_queue[-1][2]

            if self.sim_time >= last_time + 2:
                #print("add sphere")
                self.deleted_queue.pop()
                self.sphere_dict[elem_cood] = sphere_name



    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = 'uav' + str(uavID) + '/mavros/'
        return mav_topic_string

    def pose_callback_0(self, msg):
        curr_pose = msg
        uav0_location_x = curr_pose.pose.position.x
        uav0_location_y = curr_pose.pose.position.y
        uav0_location_z = curr_pose.pose.position.z
        self.uav0_location = (uav0_location_x - 6, uav0_location_y, uav0_location_z)
        #print("The pose of uav1 is:",self.uav0_location)

    def pose_callback_1(self, msg):
        curr_pose = msg
        uav1_location_x = curr_pose.pose.position.x
        uav1_location_y = curr_pose.pose.position.y
        uav1_location_z = curr_pose.pose.position.z
        self.uav1_location = (uav1_location_x - 12, uav1_location_y, uav1_location_z)
        #print("The pose of uav2 is:", self.uav1_location)

    def delete_spheres(self):
    	key_list = self.sphere_dict.keys()
        uav_loc0 = np.array(self.uav0_location)
        uav_loc1 = np.array(self.uav1_location)
    	for elem in key_list:
            #print(elem)
            if np.linalg.norm(elem - uav_loc0) < 3:
                sphere_name = self.sphere_dict[elem]
                #print(sphere_name)
                self.execute_delete(sphere_name, elem)
                j = self.sphere_dict.pop(elem)
                #print("Just popped", j)
                #print("Sphere dict is:", self.sphere_dict)

            elif np.linalg.norm(elem - uav_loc1) < 2:
                sphere_name = self.sphere_dict[elem]
                self.execute_delete(sphere_name, elem)
                j = self.sphere_dict.pop(elem)
                #print("Just popped", j)
                #print("Sphere dict is:", self.sphere_dict)

    def execute_delete(self, sphere_name, sphere_coordinates):
        #model_name_1 = str(sphere_name)
        #model_name = model_name_1[:-1]
        model_name1 = sphere_name
        #print(model_name1)
        try:
            del_sphere = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            #print("Trying to delete:", model_name1)
            resp = del_sphere(model_name1)
            #print(self.deleted_queue,"________________________________________________\n")
    	except rospy.ServiceException, e:
        	print "Service call failed: %s" % e
        self.deleted_queue.appendleft((model_name1, self.sim_time, sphere_coordinates))

if __name__ == "__main__":
    sphere_dict = {}
    sphere_dict1 = {}
    deleted_queue = collections.deque()
    root_path = '/root/catkin_ws/src/cps_challenge_2020'
    with open(root_path+'/worlds/sphere_dict.txt') as f:
        for line in f:
            (key, value) = line.split(":")
            sphere_dict[key] = value
        #print(sphere_dict)
    for elem, value in sphere_dict.items():
        elem3 = elem[1:-1]
        elem2 = elem3.split(',')
        elem = (float(elem2[0]), float(elem2[1]), float(elem2[2]))
        val1 = str(value)
        model_name = val1[:-1]
        sphere_dict1[elem] = model_name
    #print(sphere_dict1)
    RobotActionsServer(sphere_dict1, deleted_queue)