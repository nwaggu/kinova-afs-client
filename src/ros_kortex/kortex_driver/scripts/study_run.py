#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import math
import actionlib
from enum import Enum
import copy
import numpy as np 

from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from test_grid import DiscreteGrid, DualGrid
from test_patterns import TraversalPaths

grid = DualGrid((0.545, 0.17),(0.155,-0.725), 5, 2, (-0.87, 0.211),(-0.284, -0.684), 3)
#print("Cell Heigh",grid.grid_two.cell_height)
#print("Cell Width", grid.grid_two.cell_width)

#grid = DualGrid((0.418, -0.095),(0.165,-0.6), 5, 2, (-0.339, 0.224),(-0.483, -0.546), 3)
#grid = TraversalPaths((-0.55,0.6), (-0.26, -0.563), 5, 3)
# update = {4:[math.radians(133), math.radians(-8.47) ,math.radians(6)], 3:[math.radians(157), 0 ,math.radians(4.7)] , 
#           8:[math.radians(157), 0 ,math.radians(4.7)], 9:[math.radians(157), 0 ,math.radians(4.7)], 
#           5:[math.radians(-167), math.radians(-25) ,math.radians(63)],
#           6:[math.radians(176), math.radians(13) ,math.radians(63)],
#           23:[math.radians(-115), math.radians(23) ,math.radians(120)], 22:[math.radians(-130), math.radians(15) ,math.radians(109)],
#           21:[math.radians(-138), math.radians(9) ,math.radians(94)],20:[math.radians(-139), math.radians(0) ,math.radians(85)],
#           14:[math.radians(134), math.radians(6) ,math.radians(5)], 19:[math.radians(-51), math.radians(163) ,math.radians(161)],
#           13:[math.radians(163), math.radians(6) ,math.radians(30)], 18:[math.radians(-176), math.radians(40) ,math.radians(56.6)],
#             24:[math.radians(-86.4), math.radians(-168.5) ,math.radians(129.2)]
#           }

GRID_POS = grid.to_dict_fix(height=0.1)
GRID_POS[24] = [-0.769, -0.636, 0.247]
update_grid = {
    4: 0.186,
    8: 0.183,
    18:0.318,
    19: 0.33,
    23: 0.33,
    17:0.188,
    10:0.231,
    7:0.169,
    6:0.169,
    5:0.169

}

angle={}
for i in range(len(GRID_POS)):
    if i in update_grid:
        GRID_POS[i] = [GRID_POS.get(i)[0], GRID_POS.get(i)[1], update_grid.get(i)]
    angle[i] = None
update = {4:[math.radians(133), math.radians(-8.47) ,math.radians(6)], 3:[math.radians(157), 0 ,math.radians(4.7)] , 
          8:[math.radians(173), math.radians(8.4) ,math.radians(36)], 9:[math.radians(157), 0 ,math.radians(4.7)], 
          
          6:[math.radians(176), math.radians(13) ,math.radians(63)],
          23:[math.radians(-115), math.radians(23) ,math.radians(120)], 22:[math.radians(-130), math.radians(15) ,math.radians(109)],
          21:[math.radians(-138), math.radians(9) ,math.radians(94)],20:[math.radians(-139), math.radians(0) ,math.radians(85)],
          14:[math.radians(-163), math.radians(28) ,math.radians(105)], 19:[math.radians(-108), math.radians(7) ,math.radians(128)],
          13:[math.radians(163), math.radians(6) ,math.radians(30)], 18:[math.radians(-121), math.radians(10) ,math.radians(111)],
            24:[math.radians(-86.4), math.radians(-168.5) ,math.radians(129.2)],
            23:[math.radians(-104), math.radians(3) ,math.radians(113)],
            17:[math.radians(-180), math.radians(3.2) ,math.radians(100)],
            10:[math.radians(169.7), math.radians(-5.4) ,math.radians(89)]
          }

for key in update:
    angle[key] = update[key]

print(angle)

print(GRID_POS)
#GRID_POS = [[0.35000000000000003, -0.24, 0.2], [0.35000000000000003, -0.12, 0.2], [0.35000000000000003, 0.0, 0.2], [0.35000000000000003, 0.12, 0.2], [0.35000000000000003, 0.24000000000000005, 0.2], [0.6500000000000001, -0.24, 0.2], [0.6500000000000001, -0.12, 0.2], [0.6500000000000001, 0.0, 0.2], [0.6500000000000001, 0.12, 0.2], [0.6500000000000001, 0.24000000000000005, 0.2]]
rospy.loginfo("Complete GRID setup")

class Action(Enum):
    OPEN = ('open',6)
    CLOSE = ('close', 5)
    UP = (0.1, 0, 0, 1)
    DOWN = (-0.1, 0, 0, 2)
    LEFT = (0, 0.1, 0, 3)
    RIGHT = (0, -0.1, 0, 4)

INDEX_TO_ACTION = {e.value[-1]: e for e in Action}

class ArmGridClient:
    def __init__(self):
        try:
            rospy.init_node('example_waypoint_action_python')

            self.HOME_ACTION_IDENTIFIER = 10314
            self.last_gripper_pos = 0
            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.loginfo(clear_faults_full_name)
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)
            
            #Setup Admittance
            
            admittance_name = '/' + self.robot_name + "/base/set_admittance"
            rospy.loginfo(admittance_name)
            rospy.wait_for_service(admittance_name)
            self.set_admittance = rospy.ServiceProxy(admittance_name, SetAdmittance)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.loginfo(read_action_full_name)
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
            
            
            #Setup EE Pose service
            get_measured_ee_pose = '/' + self.robot_name + "/base/get_measured_cartesian_pose"
            rospy.loginfo(get_measured_ee_pose)
            rospy.wait_for_service(get_measured_ee_pose)
            self.ee_pose = rospy.ServiceProxy(get_measured_ee_pose, GetMeasuredCartesianPose)
          

            #Setup Open Close Gripper
            send_gripper = '/' + self.robot_name + "/base/send_gripper_command"
            rospy.loginfo(send_gripper)
            rospy.wait_for_service(send_gripper)
            self.send_gripper = rospy.ServiceProxy(send_gripper, SendGripperCommand)

            #Setup checking the Gripper
            get_gripper = '/' + self.robot_name + "/base/get_measured_gripper_movement"
            rospy.loginfo(get_gripper)
            rospy.wait_for_service(get_gripper)
            self.get_gripper = rospy.ServiceProxy(get_gripper, GetMeasuredGripperMovement)
            


            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True


    """
    Update Stored Joint States
    """
    def joint_callback(self, msg):
        self.joint_states = msg


    """
    Activates the Arm's compliance
    mode 4: None
    mode 1: Cartesian
    mode 2: Joint-based 
    mode 3: Null-Space
    """
    def setAdmittance(self, mode=0):
        try:
            req = SetAdmittanceRequest()
            # admit = Admittance()
            # admit.admittance_mode = mode
            req.input.admittance_mode = mode
            self.set_admittance(req)
            rospy.loginfo(req)
            return True
        except:
            rospy.logerr("Failed to Set Compliance")
            return False

    """
    Gets the grid index for the current location of the end effector
    """
    def getCartesianGridIndex(self): 
        #Get End Effector position
        current_position = self.ee_pose().output
        ee = np.array([current_position.x, current_position.y])
        min_index = grid.augmented_get_index_from_position(ee)
        return min_index

    """
    Starts Compliance Mode, Empty [req]e
    Returns A Forward Response Object That Can Be Ignored
    """
    def startFeedbackInformation(self, req):
        try:
            self.last_gripper_pos = round(self.getGripperPos(),1)
            req = self.setAdmittance(2)
            rospy.loginfo(req)
        except:
            rospy.loginfo("Failed to start admittance")
        #ee = self.ee_pose()
        #index = self.getCartesianGridIndex()
        
        return ForwardResponse(0, 0, 0)

    """
    Ends Compliance Mode 
    Input: Empty Request
    Returns: FindIndexResponse with the final grid [index] and [grip] differential (self.getGripperPos())
    """
    def endDemoFeedback(self, req):
        grip = 0
        try:
            if (self.last_gripper_pos - round(self.getGripperPos(),1)) > 0.1:
                grip = 1
            elif (self.last_gripper_pos - round(self.getGripperPos(),1)) < -0.1:
                grip = -1
            self.setAdmittance(4)
        except:
            rospy.loginfo("Failed to end admittance")
        index = self.getCartesianGridIndex()
        rospy.loginfo("State Index: ")
        rospy.loginfo(index)
        
        return FindIndexResponse(index, grip)


    """
    Gets the current ee position
    DEPRECATED
    """
    def serviceGridPosition(self, req):
        current_pose = self.ee_pose()
        return ForwardResponse(current_pose.output.x, current_pose.output.y, current_pose.output.z)


    """
    
    """
    def trajectoryFollowing(self, req):
        indexes = list(req.indicies)
        rospy.loginfo(indexes)
        #grid.visualize(path_points=indexes)
        #Setup to limit joint angles to be sent once per location

        waypoints = []
        #Loop through indicies until we've reached every point
        if len(indexes)!=0:
            for i in indexes:
                print(i)
                waypoints.append(GRID_POS.get(i))
                rospy.loginfo(GRID_POS.get(i))
            rospy.loginfo(waypoints)
            self.sendWaypoints(waypoints,indicies=indexes)
        return SendStateResponse(True)


    """
    Move to requested position [x,y,z]
    """
    def goToState(self, req):
        rospy.loginfo("Sending current request:")
        rospy.loginfo(req)
        current_pose = self.ee_pose().output
        height = 0.35
        index = list(req.indicies)[0]
        pos=GRID_POS.get(index)
        current_index = grid.augmented_get_index_from_position([current_pose.x, current_pose.y])

        current_grid = grid.checkDistance(current_pose.x, current_pose.y)
        target_grid =  grid.checkDistance(pos[0],pos[1])
        
        self.openGripper(None)
        if current_index >= 20 or current_index==19 or current_index==14:
            self.clear_faults()
            self.home_the_robot(identifier=10315)
            current_pose = self.ee_pose().output
        #Check what grid im transitioning to. 
        if current_grid==target_grid:
            res = self.sendWaypoints([[current_pose.x, current_pose.y, height], [pos[0],pos[1], height], [pos[0],pos[1],pos[2]]],indicies=[0, index, index], intial_pose=(math.radians(current_pose.theta_x),math.radians(current_pose.theta_y),math.radians(current_pose.theta_z)))
            rospy.loginfo("Result:")
            rospy.loginfo(res)
        
        else:
            if target_grid == 0:
                self.clear_faults()
                self.home_the_robot()
            else:
                self.clear_faults()
                self.home_the_robot(identifier=10315)
            current_pose = self.ee_pose().output
            res = self.sendWaypoints([[current_pose.x, current_pose.y, height], [pos[0],pos[1], height], [pos[0],pos[1],pos[2]]],indicies=[0, index, index],intial_pose=(math.radians(current_pose.theta_x),math.radians(current_pose.theta_y),math.radians(current_pose.theta_z)))
            rospy.loginfo("Result:")
            rospy.loginfo(res)

        return SendStateResponse(True)
    

    """
    Sends a set of actions (see enum above) to the arm
    """
    def sendActions(self, actions):
        self.last_action_notif_type = None
        goal = FollowCartesianTrajectoryGoal()
        current_pose = self.ee_pose()
        for act in actions:
            assert(isinstance(act, Action))
            displacement = act.value
            new_pose = Pose()
            new_pose = copy.deepcopy(current_pose)
            new_pose.x += displacement[0]
            new_pose.y += displacement[1]
            new_pose.z += displacement[2]
            goal.trajectory.append(self.FillCartesianWaypoint(new_pose.x,  new_pose.y, new_pose.z,  0, 0, 0, 1))
            current_pose = new_pose
            # goal.trajectory.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, math.radians(90), 0, math.radians(90), 0))

        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            self.client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            self.client.wait_for_result()
            return True


    #GRIPPER COMMANDS
    def getGripperPos(self):
        req = GetMeasuredGripperMovementRequest()
        req.input.mode = GripperMode.GRIPPER_POSITION
        try:
            msg = self.get_gripper(req)
        except:
            rospy.logerr("Failed To Get Gripper Information")
        
        gripper = msg.output 
        gripper_pos = gripper.finger[0].value
        
        return gripper_pos





    def closeGripper(self, requ):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 1.0
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")
        current_pose = self.ee_pose().output
        # Call the service 
        try:
            index = grid.augmented_get_index_from_position([current_pose.x,current_pose.y])
            self.sendWaypoints([[current_pose.x,current_pose.y,current_pose.z-0.05]], indicies=[index])
            self.send_gripper(req)
            self.sendWaypoints([[current_pose.x,current_pose.y,current_pose.z]], indicies=[index])
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return TriggerResponse(False)
        else:
            time.sleep(0.5)
            return TriggerResponse(True)
        

    def openGripper(self, requ):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 0
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return TriggerResponse(False)
        else:
            time.sleep(0.5)
            return TriggerResponse(True)
    

    """
    Callback action topic
    """
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
        rospy.loginfo(self.last_action_notif_type)

    """
    Wait script for some actions
    """
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                #rospy.loginfo("stuck????")
                time.sleep(10)
                return True


    """
    Creates a new point for Cartesian planner
    """
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()
        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        return cartesianWaypoint




    """Home The Robot with an ACTION, care for objects"""
    def home_the_robot(self,identifier=None):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        
        if identifier is None:
            req.input.identifier = self.HOME_ACTION_IDENTIFIER
        else:
            req.input.identifier = identifier
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()
            
    """
    Can be used to guage severity of warnings for ACTIONS (web app)
    """
    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True




    

    """
    Send a set of waypoints as a 2D array.
    e.g. waypoints  = [[0.4,0.1,0.4], [0.5, 0.2, 0.2]]
    """
    def sendWaypoints(self, waypoints, indicies=None, intial_pose=None):
        self.last_action_notif_type = None
        goal = FollowCartesianTrajectoryGoal()
        count = 0
        for point in waypoints:
            if count!= len(waypoints)-1:
                blending = 0
            else:
                blending = 0

            if indicies is not None:
                index = indicies[count]
                if intial_pose is not None and count==0:
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  intial_pose[0], intial_pose[1], intial_pose[2], 0))
                elif angle[index] is not None: 
                    if index >= 20: blending=0
                    #rospy.loginfo(angle[index])
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  angle[index][0], angle[index][1], angle[index][2], blending)) 
                else:    
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  math.radians(180), 0, math.radians(90), blending))
            else:    
                goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  math.radians(180), 0, math.radians(90), blending))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, math.radians(90), 0, math.radians(90), 0))
            count+=1
        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            self.client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            self.client.wait_for_result()
            return True


    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except:
            pass
        
        success &= self.example_clear_faults()
        #self.home_the_robot(10314)

        self.client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)
        self.client.wait_for_server()

        rospy.Subscriber('joint_states', JointState, self.joint_callback)
        rospy.Service('robot/startCompliance', Forward, self.startFeedbackInformation)
        rospy.Service('robot/endCompliance', FindIndex, self.endDemoFeedback)
        rospy.Service('robot/GridPosition', Forward, self.serviceGridPosition)
        rospy.Service('robot/SendTraj', SendState, self.trajectoryFollowing)

        rospy.Service('robot/goToState', SendState, self.goToState)
        rospy.Service('robot/closeGripper', Trigger, self.closeGripper)
        rospy.Service('robot/openGripper', Trigger, self.openGripper)
        rospy.loginfo("All Services Setup.")
        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
        #path = grid.generate_zigzag_path()
        ss = SendState()
        list_test = [6]
        for i in range(9):
            ss.indicies = [list_test[i]]
            self.goToState(ss)
            time.sleep(8)
        # list_test = [20, 15, 10, 11, 16, 21, 22, 17, 12, 13, 18,23, 24,19,14, 9, 4, 3, 8, 7, 2, 1, 6,5,0]
        # list_test.reverse()
        # ss.indicies = list_test
        # #rospy.loginfo(ss.indicies)
        # self.trajectoryFollowing(ss)
        #trig = Trigger
        #self.closeGripper()
        rospy.spin()


if __name__ == "__main__":
    try:
        ex = ArmGridClient()
        ex.main()
    except:
        print("End")
