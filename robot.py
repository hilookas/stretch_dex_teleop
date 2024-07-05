
class Robot():
    def __init__(self):
        pass

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

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_yaw_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_yaw_move_to: {x_r}")

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_pitch_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_pitch_move_to: {x_r}")

    # https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py
    def wrist_roll_move_to(self, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        print(f"wrist_roll_move_to: {x_r}")

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
