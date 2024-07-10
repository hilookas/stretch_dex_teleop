import rclpy.node
import sensor_msgs.msg

class Robot():
    def __init__(self):
        rclpy.init()
        self.node = rclpy.node.Node("teleop")

        global ros_node
        ros_node = self.node

        self.joint_state_publisher = self.node.create_publisher(sensor_msgs.msg.JointState, "/joint_states", 10)

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/base.py
    def base_reset_odometry(self):
        """
        Reset X/Y/Theta to report 0
        """
        print("base_reset_odometry")

    def base_get_theta(self):
        # self.robot.base.status['theta']
        print("base_get_status_theta")
        return 0
    
    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/base.py
    def base_rotate_by(self, x_r, v_r=None, a_r=None, stiffness=None, contact_thresh_N=None, contact_thresh=None):
        """
        Incremental rotation of the base
        x_r: desired motion (radians)
        v_r: velocity for trapezoidal motion profile (rad/s)
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_N: (deprecated) effort to stop at (units of pseudo_N)
        contact_thresh: effort to stop at (units of effort_pct (-100, 100))
        """
        print(f"base_rotate_by: {x_r}")

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/prismatic_joint.py
    def lift_move_to(self, x_m, v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None,
                req_calibration=True, contact_thresh_pos=None, contact_thresh_neg=None):
        """
        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """
        print(f"lift_move_to: {x_m}")

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.name = [ "joint_lift" ]
        msg.position = [ float(x_m) ]

        self.joint_state_publisher.publish(msg)
        

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/prismatic_joint.py
    def arm_move_to(self, x_m, v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None,
                req_calibration=True, contact_thresh_pos=None, contact_thresh_neg=None):
        """
        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """
        print(f"arm_move_to: {x_m}")

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.name = [ "joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3",  ]
        msg.position = [ float(x_m)/4, float(x_m)/4, float(x_m)/4, float(x_m)/4 ]

        self.joint_state_publisher.publish(msg)

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_yaw_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_yaw_move_to: {x_r}")

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.name = [ "joint_wrist_yaw" ]
        msg.position = [ float(x_r) ]

        self.joint_state_publisher.publish(msg)

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_pitch_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_pitch_move_to: {x_r}")

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.name = [ "joint_wrist_pitch" ]
        msg.position = [ float(x_r) ]

        self.joint_state_publisher.publish(msg)

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_roll_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_roll_move_to: {x_r}")

        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        msg.name = [ "joint_wrist_roll" ]
        msg.position = [ float(x_r) ]

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def gripper_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"gripper_move_to: {x_r}")

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py
    def push_command(self):
        print("push_command!")
