import numpy as np
import webcam_teleop_interface as wt
import simple_ik as si
import argparse
import gripper_to_goal as gg
import goal_from_teleop as gt
import dex_teleop_parameters as dt
import pprint as pp
import loop_timer as lt
import scipy.spatial


if __name__ == '__main__':

    args = dt.get_arg_parser().parse_args()
    use_fastest_mode = args.fast
    manipulate_on_ground = args.ground
    left_handed = args.left
    using_stretch_2 = args.stretch_2
    slide_lift_range = args.slide_lift_range
        
    # The 'default', 'slow', 'fast', and 'max' options are defined by
    # Hello Robot. The 'fastest_stretch_2' option has been specially tuned for
    # this application.
    #
    # WARNING: 'fastest_stretch_*' have velocities and accelerations that exceed
    # the factory 'max' values defined by Hello Robot.
    if use_fastest_mode:
        if using_stretch_2:
            robot_speed = 'fastest_stretch_2'
        else: 
            robot_speed = 'fastest_stretch_3'
    else:
        robot_speed = 'slow'
    print('running with robot_speed =', robot_speed)

    lift_middle = dt.get_lift_middle(manipulate_on_ground)
    center_configuration = dt.get_center_configuration(lift_middle)
    starting_configuration = dt.get_starting_configuration(lift_middle)
    
    if left_handed: 
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='left', visualize_detections=False)
    else:
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=False)
    
    # Initialize IK
    simple_ik = si.SimpleIK()

    # Define the center position for the wrist that corresponds with
    #the teleop origin.
    center_wrist_position = simple_ik.fk_rotary_base(center_configuration)

    gripper_to_goal = gg.GripperToGoal(robot_speed, starting_configuration, dt.robot_allowed_to_move, using_stretch_2)

    goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, center_wrist_position, slide_lift_range=slide_lift_range)


    loop_timer = lt.LoopTimer()
    print_timing = False
    print_goal = True
    
    from robot import ros_node
    import geometry_msgs.msg

    pub = ros_node.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose", 10)
    
    while True:
        loop_timer.start_of_iteration()
        markers = webcam_aruco_detector.process_next_frame()
        goal_dict = goal_from_markers.get_goal_dict(markers)
        if goal_dict:
            msg = geometry_msgs.msg.PoseStamped()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.pose.position.x = goal_dict["wrist_position"][0]
            msg.pose.position.y = goal_dict["wrist_position"][1]
            msg.pose.position.z = goal_dict["wrist_position"][2]

            # Use the gripper pose marker's orientation to directly control the robot's wrist yaw, pitch, and roll. 
            aruco_rotation = np.zeros((3, 3))

            aruco_rotation[:,0] = goal_dict["gripper_x_axis"]
            aruco_rotation[:,1] = goal_dict["gripper_y_axis"]
            aruco_rotation[:,2] = goal_dict["gripper_z_axis"]

            r = scipy.spatial.transform.Rotation.from_matrix(aruco_rotation)
            # capital letters represent intrinsic
            # rotations, lowercase letters represent
            # extrinsic rotations
            # ypr = r.as_euler('ZXY', degrees=False)
            (x, y, z, w) = r.as_quat()

            msg.pose.orientation.x = x
            msg.pose.orientation.y = y
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w

            pub.publish(msg)
            if print_goal:
                print('goal_dict =')
                pp.pprint(goal_dict)
            gripper_to_goal.update_goal(**goal_dict)
        loop_timer.end_of_iteration()
        if print_timing: 
            loop_timer.pretty_print()




##############################################################
## NOTES
##############################################################

#######################################
#
# Overview
#
# Dexterous teleoperation uses a marker dictionary representing either
# a real or virtual ArUco marker specified with respect to the
# camera's frame of reference. The marker's position controls the
# robot's wrist position via inverse kinematics (IK). The marker's
# orientation directly controls the joints of the robot's dexterous
# wrist.
#
#######################################

#######################################
#
# The following coordinate systems are important to this teleoperation
# code
#
#######################################

#######################################
# ArUco Coordinate System
#
# Origin in the middle of the ArUco marker.
#
# x-axis
# right side when looking at marker is pos
# left side when looking at marker is neg

# y-axis
# top of marker is pos
# bottom of marker is neg

# z-axis
# normal to marker surface is pos
# pointing into the marker surface is neg
#
#######################################

#######################################
# Camera Coordinate System
#
# Camera on the floor looking with the top of the camer facing away
# from the person.
#
# This configuration matches the world frame's coordinate system with
# a different origin that is mostly just translated along the x and y
# axes.
#
# Origin likely at the optical cemter of a pinhole
# model of the camera.
#
# The descriptions below describe when the robot's mobile base is at
# theta = 0 deg.
#
# x-axis
# human left is pos / robot forward is pos
# human right is neg / robot backward is neg

# y-axis
# human arm extended is neg / robot arm extended is neg
# human arm retracted is pos / robot arm retracted is pos

# z-axis
# up is positive for person and the robot
# down is negative for person and the robot
#
#######################################

#######################################
# IK World Frame Coordinate System
#
# Origin at the axis of rotation of the mobile
# base on the floor.
#
# x-axis
# human/robot left is pos
# human/robot right is neg

# y-axis
# human/robot forward is neg
# human/robot backward is pos

# z-axis
# human/robot up is pos
# human/robot down is neg
#
#######################################

#######################################
# Robot Wrist Control

# wrist yaw
#     - : deployed direction
#     0 : straight out parallel to the telescoping arm
#     + : stowed direction

# wrist pitch
#     - : up
#     0 : horizontal
#     + : down

# wrist roll
#     - : 
#     0 : horizontal
#     + :
#
#######################################

##############################################################
