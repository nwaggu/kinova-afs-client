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

"""
GRID AND LOCKED ANGLE SETUP
"""
grid = DualGrid((0.545, 0.17),(0.155,-0.725), 5, 2, (-0.87, 0.211),(-0.284, -0.684), 3)

#Outputs All Grid States To A Dictionary
GRID_POS = grid.to_dict_fix(height=0.2)

#INSERT CHANGES TO CELL HEIGHTS HERE
update_grid_height = {
    4: 0.27,
    9:0.25,
    8: 0.25,
    18:0.38,
    19: 0.33,
    23: 0.33,
    17:0.2, 
    10:0.3,
    7:0.24,
    6:0.24,
    5:0.16,
    11:0.27,
    15:0.3,
    12:0.25

}

#Apply Grid Height Changes
ANGLE={}
for i in range(len(GRID_POS)):
    if i in update_grid_height:
        GRID_POS[i] = [GRID_POS.get(i)[0], GRID_POS.get(i)[1], update_grid_height.get(i)]
    ANGLE[i] = None

#Changes the default angle at a cell
update_angle = {4:[math.radians(133), math.radians(-8.47) ,math.radians(6)], 3:[math.radians(157), 0 ,math.radians(4.7)] , 
          8:[math.radians(173), math.radians(8.4) ,math.radians(36)], 9:[math.radians(157), 0 ,math.radians(4.7)], 
          
          6:[math.radians(176), math.radians(13) ,math.radians(63)],
          23:[math.radians(-115), math.radians(23) ,math.radians(120)], 22:[math.radians(-130), math.radians(15) ,math.radians(109)],
          21:[math.radians(-138), math.radians(9) ,math.radians(94)],20:[math.radians(-139), math.radians(0) ,math.radians(85)],
          14:[math.radians(-163), math.radians(28) ,math.radians(105)], 19:[math.radians(-162), math.radians(-80) ,math.radians(158)],
          13:[math.radians(163), math.radians(6) ,math.radians(30)], 18:[math.radians(-162), math.radians(-80) ,math.radians(158)],
            24:[math.radians(17), math.radians(-102) ,math.radians(15)],
            23:[math.radians(-162), math.radians(-80) ,math.radians(158)],
            17:[math.radians(-180), math.radians(0) ,math.radians(90)],
            10:[math.radians(169.7), math.radians(-5.4) ,math.radians(89)],
            95:[math.radians(-3.7), math.radians(-118) ,math.radians(1.77)], 
            96:[math.radians(-3.7), math.radians(-118) ,math.radians(1.77)],
            97:[math.radians(-93), math.radians(-42) ,math.radians(69)],
            98:[math.radians(166),math.radians(-7.2),math.radians(176)],
            99:[math.radians(-175),math.radians(-2.9),math.radians(41.6)]
          }

#Update Angles
for key in update_angle:
    ANGLE[key] = update_angle[key]

#Hard update grid positions
GRID_POS[24] = [-0.695, -0.6, 0.35]
#Indexes >90 indicate transition locations
GRID_POS[95] = [-0.409, -0.561, 0.38]
GRID_POS[96] = [-0.322, -0.396, 0.397]
GRID_POS[97] = copy.deepcopy(GRID_POS[17])
GRID_POS[97][2] = 0.3
GRID_POS[98] = [0.039, 0.35, 0.13]
GRID_POS[99] = [0, -0.3, 0.19]

class AFSClient:
    def __init__(self):
        try:
            rospy.init_node('example_waypoint_action_python')

            self.HOME_ACTION_IDENTIFIER = 10314
            self.last_gripper_pos = 0
            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)


            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            #rospy.loginfo(clear_faults_full_name)
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)
            
            #Setup Admittance
            admittance_name = '/' + self.robot_name + "/base/set_admittance"
            #rospy.loginfo(admittance_name)
            rospy.wait_for_service(admittance_name)
            self.set_admittance = rospy.ServiceProxy(admittance_name, SetAdmittance)

            #Setup Calling Web Actions
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            #rospy.loginfo(read_action_full_name)
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
            
            
            #Setup EE Pose service
            get_measured_ee_pose = '/' + self.robot_name + "/base/get_measured_cartesian_pose"
            #rospy.loginfo(get_measured_ee_pose)
            rospy.wait_for_service(get_measured_ee_pose)
            self.ee_pose = rospy.ServiceProxy(get_measured_ee_pose, GetMeasuredCartesianPose)
          
            #Setup Open Close Gripper
            send_gripper = '/' + self.robot_name + "/base/send_gripper_command"
            #rospy.loginfo(send_gripper)
            rospy.wait_for_service(send_gripper)
            self.send_gripper = rospy.ServiceProxy(send_gripper, SendGripperCommand)

            #Setup checking the Gripper
            get_gripper = '/' + self.robot_name + "/base/get_measured_gripper_movement"
            #rospy.loginfo(get_gripper)
            rospy.wait_for_service(get_gripper)
            self.get_gripper = rospy.ServiceProxy(get_gripper, GetMeasuredGripperMovement)
            

            #Setup Sending Web App Actions
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            #Action Notification Setup (Unused)
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            #Get Product Configuration
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


    def startFeedbackInformation(self, req):
        """
        Starts Compliance Mode, Empty [req]
        Returns A Forward Response Object That Can Be Ignored
        """
        try:
            self.last_gripper_pos = round(self.getGripperPos(),1)
            req = self.setAdmittance(2)
            rospy.loginfo(req)
        except:
            rospy.loginfo("Failed to start admittance")
        
        return ForwardResponse(0, 0, 0)


    def endDemoFeedback(self, req):
        """
        Ends Compliance Mode 
        
        Input: Empty Request
        Returns: FindIndexResponse with the final grid [index] and [grip] differential (self.getGripperPos())
        """
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




    def trajectoryFollowing(self, req):
        """
        Service For Sending Trajectories
        Input: 
            [SendState]: Srv Object
        Returns:
            [SendStateResponse]: Whether or not the trajectories succeeded.
        """
        indexes = list(req.indicies)
        rospy.loginfo("Sending the Following Trajectory: %s", indexes)
        #grid.visualize(path_points=indexes)
        #Setup to limit joint angles to be sent once per location
        transitionTable = {}
        pickupLocations = {100:16, 101:4, 102:17, 103:10}
        waypoints = []
        #Loop through indicies until we've reached every point
        
        prev_index = 0
        offset = 0
        current_index = None 

        #Create segmented trajs based on 
        segments = []
        current_segment = []
        count = 0
        for index in indexes:
            current_segment.append(index)
            if index >= 100:
                segments.append(current_segment)
                current_segment = []
            elif count == len(indexes)-1:
                segments.append(current_segment)
            count+=1
        
        rospy.loginfo("Splitting PickUp Segments %s", segments)
        count = 0
        for segment in segments:
            copy_index = copy.deepcopy(segment)
            if len(segment)!=0:
                for i in range(len(segment)):
                    current_index = segment[i]
                    
                    #Check If Prev and Current Index Require A Transition
                    pairing = (prev_index, current_index)
                    if 5 in pairing and 10 in pairing:
                        waypoints.append(GRID_POS.get(98))
                        copy_index.insert(i+offset,98)
                        offset+=1

                    elif 6 in pairing and 11 in pairing:
                        waypoints.append(GRID_POS.get(99))
                        copy_index.insert(i+offset,99)
                        offset+=1
                    
                    #Transitions between 16 and 17 (Too Low)
                    elif (17 in pairing and 18 in pairing) or (102 in pairing and 18 in pairing):
                        waypoints.append(GRID_POS.get(97))
                        copy_index.insert(i+offset,97)
                        offset+=1
                    
                    elif (13 in pairing and 18 in pairing):
                        waypoints.append(GRID_POS.get(96))
                        copy_index.insert(i+offset,96)
                        offset+=1

                    elif 14 in pairing and 19 in pairing:
                        waypoints.append(GRID_POS.get(95))
                        copy_index.insert(i+offset,95)
                        offset+=1

                    if current_index in pickupLocations.keys():
                        waypoints.append(GRID_POS.get(pickupLocations.get(current_index)))
                        prev_index = pickupLocations.get(current_index)
                        copy_index[i+offset] = pickupLocations.get(current_index)
                    else:
                        waypoints.append(GRID_POS.get(current_index))
                        prev_index = segment[i]
                    
                    rospy.logdebug("Trajectory %s Sent: %s", count, waypoints) 
                    
                    #rospy.loginfo(copy_index)

                self.sendWaypoints(waypoints,indicies=copy_index)
                waypoints=[]
                offset=0
                if current_index == 101 or current_index ==103:
                    self.openGripper(None)
                elif current_index == 100:
                    self.pickUp(distance=1)
                elif current_index == 102:
                    self.pickUp(None)
                count+=1

                


        return SendStateResponse(True)



    def goToState(self, req):
        """
        Move to requested position [x,y,z]
        """
        rospy.loginfo(req)
        #Get the current position 
        current_pose = self.ee_pose().output
        height = 0.35

        #Extract the desired index position
        target_index = list(req.indicies)[0]
        #Convert index to position
        pos=GRID_POS.get(target_index)
        #Get the current grid index the EE is in 
        current_index = grid.augmented_get_index_from_position([current_pose.x, current_pose.y])
        
        #Open the Gripper
        self.openGripper(None)

        # #If Index >=20 or 19 or 14, reset to home position before moving
        if current_index >= 20 or current_index==19 or current_index==14:
            self.clear_faults()
            self.take_an_action(identifier=10315)
            current_pose = self.ee_pose().output
            current_index = grid.augmented_get_index_from_position([current_pose.x, current_pose.y])
        
        #If we're on the correct grid, move the EE up to [height] and move to the position, then move back down
        if current_index<=9 and target_index<= 9 or current_index>9 and target_index>9:
            if current_index<=9:
                self.clear_faults()
                self.take_an_action()
            else:
                self.clear_faults()
                self.take_an_action(10315)
            current_pose = self.ee_pose().output
            self.clear_faults()
            res = self.sendWaypoints([[current_pose.x, current_pose.y, height], [pos[0],pos[1], height], 
                                      [pos[0],pos[1],pos[2]]],indicies=[0, target_index, target_index], 
                                      intial_pose=(math.radians(current_pose.theta_x),math.radians(current_pose.theta_y),
                                                   math.radians(current_pose.theta_z)))
            rospy.loginfo("Result:")
            rospy.loginfo(res)
        
        # #If we're NOT on the correct grid, align to the correct grid before moving
        else:
            if current_index<=9:
                self.clear_faults()
                self.take_an_action()
            else:
                self.clear_faults()
                self.take_an_action(10315)
            
            if target_index<=9:
                self.clear_faults()
                self.take_an_action()
            else:
                self.clear_faults()
                self.take_an_action(identifier=10315)
            
            current_pose = self.ee_pose().output
            res = self.sendWaypoints([[current_pose.x, current_pose.y, height], [pos[0],pos[1], height], 
                                        [pos[0],pos[1],pos[2]]],indicies=[0, target_index, target_index],
                                        intial_pose=(math.radians(current_pose.theta_x),math.radians(current_pose.theta_y),
                                                    math.radians(current_pose.theta_z)))

        
        gripper_index = list(req.indicies)[1]
        if gripper_index:
            self.closeGripper(None)
        else:
            self.openGripper(None)

        return SendStateResponse(True)
    


    def getGripperPos(self):
        """
        Get The Current Gripper State
        Returns:
            [Float]: Value Between 0 to 1, 1 is closed, 0 is open. -99 if invalid
        """
        req = GetMeasuredGripperMovementRequest()
        req.input.mode = GripperMode.GRIPPER_POSITION
        try:
            msg = self.get_gripper(req)
        except:
            rospy.logerr("Failed To Get Gripper Information")
            return -99
        
        gripper = msg.output 
        gripper_pos = gripper.finger[0].value
        
        return gripper_pos


    def pickUp(self, distance=None):
        """
        Moves Down Then Closes the Gripper, Then Moves Back Up
        Input:
            [requ]: Empty Request
        Returns:
            [TriggerResponse]: A message  type, True if success, False otherwise
        """
        # Initialize the request
        # Close the gripper
        rospy.loginfo("Sending Pickup Request...")
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 1.0
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        current_pose = self.ee_pose().output
        # Call the service 
        
        #Anmount of distance to move down for each object
        if distance is not None:
            dist = 0.18
        else:
            dist = 0.12

        #Move Down Then PickUp Then Move Back Up
        try:
            #Grab the current Index To Feed Orientation As Were Moving Down
            index = grid.augmented_get_index_from_position([current_pose.x,current_pose.y])
            self.sendWaypoints([[current_pose.x,current_pose.y,current_pose.z-dist]], indicies=[index])
            self.send_gripper(req)
            self.sendWaypoints([[current_pose.x,current_pose.y,current_pose.z]], indicies=[index])
        except rospy.ServiceException:
            rospy.logerr("Gripper or Waypoint Following Failed: Try Tightening the Gripper or Setting blend=0 in sendWaypoints")
            return TriggerResponse(False)
        else:
            time.sleep(0.5)
            return TriggerResponse(True)
        

    def openGripper(self, requ):
        """
        Opens the Gripper.

        Input:
            [requ]: Empty Request
        Returns:
            [TriggerResponse]: A message  type, True if success, False otherwise
        """
        rospy.loginfo("Sending Open Gripper Request...")
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 0
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION
        
        # Call the service 
        try:
            self.send_gripper(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand: Have You Checked If The Gripper is Tightened?")
            return TriggerResponse(False)
        else:
            time.sleep(0.5)
            return TriggerResponse(True)
    

    def closeGripper(self, requ):
        """
        Opens the Gripper.

        Input:
            [requ]: Empty Request
        Returns:
            [TriggerResponse]: A message  type, True if success, False otherwise
        """
        # Initialize the request
        # Close the gripper
        rospy.loginfo("Sending Close Gripper Request...")
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = 1
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # Call the service 
        try:
            self.send_gripper(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand: Have You Checked If The Gripper is Tightened?")
            return TriggerResponse(False)
        else:
            time.sleep(0.5)
            return TriggerResponse(True)


    def cb_action_topic(self, notif):
        """
        Callback action topic
        """
        self.last_action_notif_type = notif.action_event


    def wait_for_action_end_or_abort(self):
        """
        Wait script for some actions
        """
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)



    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        """
        Creates a new point for Cartesian planner
        """
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





    def take_an_action(self,identifier=None):
        """
        Takes an action based on an identifier. If [identifier] is None. Homes the robot.
        Note: INFERS ANY ACTION WILL TAKE 10 SECONDS, [self.last_action_notif_type] never updates
        Input: 
            [identifier]: integer, maps to Web App Actions, edit or export Action in Web App To View
        Returns:
            [bool]: Whether or not the action was successfully completed
        """
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        
        if identifier is None:
            req.input.identifier = self.HOME_ACTION_IDENTIFIER
            rospy.loginfo("Sending the Robot to HOME AFS 1")
        else:
            req.input.identifier = identifier
            rospy.loginfo("Sending the Robot to HOME AFS 2")
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            req.input.handle.action_type = ActionType.REACH_POSE
            
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()
            

    def subscribe_to_a_robot_notification(self):
        """
        Can be used to guage severity of warnings for ACTIONS (web app)
        """
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
        """
        Clears any faults occuring with the system. Should be run before moving the robot anywhere.
        """
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared Faults Successfully")
            rospy.sleep(2.5)
            return True




    


    def sendWaypoints(self, waypoints, indicies=None, intial_pose=None):
        """
        Send a Cartesian trajectory for the arm to follow.
        Inputs: 
            [waypoints], a 2D array of X,Y,Z coords, e.g. waypoints  = [[0.4,0.1,0.4], [0.5, 0.2, 0.2]]
            [indicies]: (Optional) For specificying the final angle at a posiiton based on the given index, pulls from ANGLE variable for angle
            [initial_pose]: (Optional) Only important for the goToState function. Feeds current EE angle for smoother transitions

        """
        pickupLoc = {100:16, 101:17, 102:4, 103:10}
        self.last_action_notif_type = None
        goal = FollowCartesianTrajectoryGoal()
        #We keep a count for tracking the [indicies]
        count = 0
        #For every waypoint we've asked for
        for point in waypoints:
            #Add blending (leeway) to non-target points if needed
            if count!= len(waypoints)-1:
                blending = 0
            #Never add blending to the final point
            else:
                blending = 0

            #If we've specified indicies (and thus angles)
            if indicies is not None:
                

                #Get the current index in the list (corresponds to the waypoint order as well)
                index = indicies[count]
                #If we're in a pick state its the last state till we reset
                #In the case we have a specific intial pose and its the first waypoint 
                if intial_pose is not None and count==0:
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  intial_pose[0], intial_pose[1], intial_pose[2], 0))
                #Otherwise check that our index has a special angle requirement (in ANGLE)
                elif ANGLE[index] is not None: 
                    #For indexes greater than 20, blending fails spectacularly, remove it
                    if index >= 20: blending=0
                    #Add the trajectory point to the list
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  ANGLE[index][0], ANGLE[index][1], ANGLE[index][2], blending)) 
                #If we have no angle requirement, we assume we can stand straight up
                else:    
                    #Add the trajectory point to the list
                    goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  math.radians(180), 0, math.radians(90), blending))
            #Again if we have no angle requirements, we add and stand straight up 
            else:    
                goal.trajectory.append(self.FillCartesianWaypoint(point[0],  point[1], point[2],  math.radians(180), 0, math.radians(90), blending))
            #Increment the count
            count+=1
        
        # Call the service, send the trajectories
        #rospy.loginfo("Sending Cartesian Goal To Action Server")
        try:
            self.client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            self.client.wait_for_result()
            #rospy.loginfo("Completed Cartesian Goal In Action Server")
            return True

    def calibration_16(self, req):
        
        pose = self.ee_pose().output
        GRID_POS[16][0] = pose.x
        GRID_POS[16][1] = pose.y

        return TriggerResponse(True)

    def calibration_17(self, req):
        
        pose = self.ee_pose().output
        GRID_POS[17][0] = pose.x
        GRID_POS[17][1] = pose.y

        return TriggerResponse(True)

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except:
            pass
        
        success &= self.example_clear_faults()
        self.subscribe_to_a_robot_notification()
        self.take_an_action()

        self.client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("All Kinova Services Setup... Wait For Study Services.")
        rospy.Subscriber('joint_states', JointState, self.joint_callback)
        rospy.Service('robot/startCompliance', Forward, self.startFeedbackInformation)
        rospy.Service('robot/endCompliance', FindIndex, self.endDemoFeedback)
        rospy.Service('robot/SendTraj', SendState, self.trajectoryFollowing)
        rospy.Service('robot/goToState', SendState, self.goToState)
        rospy.Service('robot/calibration_1', Trigger, self.calibration_16)
        rospy.Service('robot/calibration_2', Trigger, self.calibration_17)
        #rospy.Service('robot/closeGripper', Trigger, self.pickUp)
        #rospy.Service('robot/openGripper', Trigger, self.openGripper)
        rospy.loginfo("All Study Services Setup")
        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
        
        self.openGripper(None)
        #path = grid.generate_zigzag_path()
        ss = SendState()
        print("----------------------------------------------------")
        rospy.loginfo("Calibration Time: Activate Admittance in Web App. Move End Effector Over Plush Toy and Run Call Service \'robot/calibration_1\', Align End Effector Over White Box and Run Call Service \'robot/calibration_2\', Press Enter To Skip...")
        input()
        rospy.loginfo("Calibration Setup.")

        # IN CASE OF FAILURE ON SENDING TRAJECTORY (Copy paste output into ss.indicies):
        ss.indicies = [0, 0]
        self.goToState(ss)
        ss.indicies = [0, 1, 2, 7, 12, 102, 17, 16, 15, 103, 10, 11, 100, 16, 11, 6, 1, 2, 3, 101, 4]
        self.trajectoryFollowing(ss)

        rospy.spin()


if __name__ == "__main__":
    try:
        ex = AFSClient()
        ex.main()
    except:
        print("End")
