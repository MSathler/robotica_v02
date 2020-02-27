#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy
import tf

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0

viapoints=[]

viapoints_marker=MarkerArray()
count =0

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )

    print '+++++++++++',feedback.menu_entry_id,'+++++++++++++++++++++'


    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id==1:
            group.set_position_target([feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z])
            group.set_pose_target(feedback.pose) #give for 6 or more dof arms
            plan = group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            #pub.publish(feedback.pose)
            	
            
        elif feedback.menu_entry_id==2:
            listener.waitForTransform('/tool0','/link_1',rospy.Time(), rospy.Duration(1.0))
            (trans,rot)=listener.lookupTransform('link_1','tool0',rospy.Time())
            print trans,rot
  
            feedback.pose.position.x=trans[0]
            feedback.pose.position.y=trans[1]
            feedback.pose.position.z=trans[2]
            feedback.pose.orientation.x=rot[0]
            feedback.pose.orientation.y=rot[1]
            feedback.pose.orientation.z=rot[2]
            feedback.pose.orientation.w=rot[3]
            server.setPose( feedback.marker_name, feedback.pose )
            
        elif feedback.menu_entry_id==4:
            viapoints.append(feedback.pose)
            print 'Count: ',len(viapoints)
            marker= Marker()
            marker.header.frame_id = "link_1"
            marker.type=Marker.SPHERE
            marker.action=marker.ADD
            marker.scale.x=0.005
            marker.scale.y=0.005
            marker.scale.z=0.005
            marker.color.r=1
            marker.color.g=1
            marker.color.b=0
            marker.color.a=1.0

            marker.pose=feedback.pose
            
            viapoints_marker.markers.append(marker)
            id=0
            for m in viapoints_marker.markers:
		        m.id=id
		        id+=1
            
            
            marker_pub.publish(viapoints_marker)
        
        elif feedback.menu_entry_id==5:
            if len(viapoints)>0:
               (plan, fraction)=group.compute_cartesian_path(viapoints, 0.01, 0.0)
	       print viapoints
               display_traj=moveit_msgs.msg.DisplayTrajectory()
               display_traj.trajectory_start = robot.get_current_state()
               display_traj.trajectory.append(plan)
               display_traj_pub.publish(display_traj)
               group.execute(plan,wait=True)			
            
        elif feedback.menu_entry_id==6:
            if len(viapoints_marker.markers)>0:
		        viapoints[:]=[]
		        for m in viapoints_marker.markers:
		            m.action=Marker().DELETE
		        marker_pub.publish(viapoints_marker)
		        print 'Cleared all viapoints'
            else:
                print 'No via points set'
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 0.15

    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


def makeChessPieceMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 0.3

    int_marker.name = "chess_piece"
    int_marker.description = "Chess Piece\n(2D Move + Alignment)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

    # set different callback for POSE_UPDATE feedback
    server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )
    menu_handler.apply( server, int_marker.name )

if __name__=="__main__":
    rospy.init_node("simple_marker")
    listener=tf.TransformListener()
    br = TransformBroadcaster()
    display_traj_pub=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    marker_pub= rospy.Publisher('/via_points',MarkerArray,queue_size=40)


    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    listener.waitForTransform('/link_1','/tool0',rospy.Time(), rospy.Duration(1.0))
    print listener.frameExists('link_1')
    print listener.frameExists('tool0')
    (trans,rot)=listener.lookupTransform('tool0','link_1',rospy.Time())
    print trans,rot

    server = InteractiveMarkerServer("simple_marker")

    menu_handler.insert( "Execute Motion",callback=processFeedback )
    menu_handler.insert( "Move to endeff", callback=processFeedback )
    ViaPoints=menu_handler.insert( "Via Points")
    menu_handler.insert( "Add via-point", parent=ViaPoints ,callback=processFeedback )
    menu_handler.insert( "Execute Trajectory", parent=ViaPoints ,callback=processFeedback )
    menu_handler.insert( "Reset via-points", parent=ViaPoints, callback=processFeedback )
  
    position = Point( 1, 1, 0)
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, True )

    #position = Point( -1, -1, 0)
    #makeChessPieceMarker( position )

    

    server.applyChanges()

    rospy.spin()


