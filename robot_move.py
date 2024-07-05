from functools import partial
from robot import Robot


class RobotMove:
    def __init__(self, robot: Robot, speed='default'):
        
        self.available_speeds = ['default', 'slow', 'fast', 'max', 'fastest_stretch_2', 'fastest_stretch_3']
        self.create_all_commands(robot)

        if speed not in self.all_speeds.keys():
            print('WARNING: RobotMove create_commands called with unrecognized speed = \'' + speed + '\'')
            print('         Using \'default\' speed instead.')
            speed = 'default'
        
        self.speed = speed
        self.move_to_functions = self.all_speeds[self.speed]['move_to_functions']
        self.move_to_settings = self.all_speeds[self.speed]['move_to_settings']
        

    def print_settings(self):
        print(self.move_to_settings)

    def create_all_commands(self, robot):
        self.all_speeds = {}
        for speed in self.available_speeds:
            
            if (speed == 'fastest_stretch_2') or (speed == 'fastest_stretch_3'):

                # # These were the "max" values on Stretch 2040 right after
                # # a software and firmware upgrade on December 5, 2023.
                # max_parameters = {
                #     'joint_mobile_base_rotate_by': (None, {'v_r': 0.3, 'a_r': 0.3}),
                #     'joint_lift': (None, {'v_m': 0.15, 'a_m': 0.3}),
                #     'joint_arm_l0': (None, {'v_m': 0.15, 'a_m': 0.3}),
                #     'joint_wrist_yaw': (None, {'v_r': 3.0, 'a_r': 10}),
                #     'joint_wrist_pitch': (None, {'v_r': 3.0, 'a_r': 10.0}),
                #     'joint_wrist_roll': (None, {'v_r': 4.5, 'a_r': 12}),
                #     'stretch_gripper':  (None, {'v_r': 8, 'a_r': 12})
                #  }

                if speed == 'fastest_stretch_2': 
                    custom_parameters = {
                        'joint_mobile_base_rotate_by': (None, {'v_r': 0.3, 'a_r': 0.5}),
                        'joint_lift': (None, {'v_m': 0.2, 'a_m': 1.0}),
                        'joint_arm_l0': (None, {'v_m': 0.18, 'a_m': 1.0}),
                        'joint_wrist_yaw': (None, {'v_r': 3.0, 'a_r': 10}),
                        'joint_wrist_pitch': (None, {'v_r': 3.0, 'a_r': 10.0}),
                        'joint_wrist_roll': (None, {'v_r': 4.5, 'a_r': 12}),
                        'stretch_gripper':  (None, {'v_r': 12, 'a_r': 18})
                    }
                elif speed == 'fastest_stretch_3':
                    #'joint_lift': (None, {'v_m': 0.13, 'a_m': 1.0}),
                    custom_parameters = {
                        'joint_mobile_base_rotate_by': (None, {'v_r': 0.3, 'a_r': 0.5}),
                        'joint_lift': (None, {'v_m': 0.2, 'a_m': 1.0}),
                        'joint_arm_l0': (None, {'v_m': 0.18, 'a_m': 1.0}),
                        'joint_wrist_yaw': (None, {'v_r': 3.0, 'a_r': 10}),
                        'joint_wrist_pitch': (None, {'v_r': 3.0, 'a_r': 10.0}),
                        'joint_wrist_roll': (None, {'v_r': 4.5, 'a_r': 12}),
                        'stretch_gripper':  (None, {'v_r': 12, 'a_r': 18})
                    }

                joint_functions = {
                    'joint_mobile_base_rotate_by': robot.base_rotate_by,
                    'joint_lift': robot.lift_move_to, 
                    'joint_arm_l0': robot.arm_move_to,
                    'joint_wrist_yaw': robot.wrist_yaw_move_to,
                    'joint_wrist_pitch': robot.wrist_pitch_move_to,
                    'joint_wrist_roll': robot.wrist_roll_move_to,
                    'stretch_gripper' : robot.gripper_move_to
                    }

                f = joint_functions
                p = custom_parameters

                move_to_functions = { j: ((partial(f[j], **p[j][1]))
                                               if (p[j][0] is None) else
                                                   (partial(f[j], p[j][0], **p[j][1])))
                                           for j in joint_functions.keys()}

                move_to_settings = custom_parameters

            else: 

                base_rot_v = 0.3
                base_rot_a = 0.5

                lift_v = 0.2
                lift_a = 1.0

                arm_v = 0.18
                arm_a = 1.0

                yaw_v = 3.0
                yaw_a = 10

                pitch_v = 3.0
                pitch_a = 10

                roll_v = 4.5
                roll_a = 12

                gripper_v = 12
                gripper_a = 18

                # # stretch gripper move_to
                # def move_to(self,pct, v_r=None, a_r=None):
                #     """
                #     pct: commanded absolute position (Pct).
                #     v_r: velocity for trapezoidal motion profile (rad/s).
                #     a_r: acceleration for trapezoidal motion profile (rad/s^2)
                #     """

                move_to_functions = {
                    'joint_mobile_base_rotate_by': partial(robot.base_rotate_by, v_r=base_rot_v, a_r=base_rot_a),
                    'joint_lift': partial(robot.lift_move_to, v_m=lift_v, a_m=lift_a), 
                    'joint_arm_l0': partial(robot.arm_move_to, v_m=lift_v, a_m=lift_a), 
                    'joint_wrist_yaw': partial(robot.wrist_yaw_move_to, v_r=yaw_v, a_r=yaw_a),
                    'joint_wrist_pitch': partial(robot.wrist_pitch_move_to, v_r=pitch_v, a_r=pitch_a),
                    'joint_wrist_roll': partial(robot.wrist_roll_move_to, v_r=roll_v, a_r=roll_a),
                    'stretch_gripper' : partial(robot.gripper_move_to, v_r=gripper_v, a_r=gripper_a)
                }

                move_to_settings = {
                    'joint_mobile_base_rotate_by': (None, {'v_r':base_rot_v, 'a_r':base_rot_a}),
                    'joint_lift': (None, {'v_m':lift_v, 'a_m':lift_a}), 
                    'joint_arm_l0': (None, {'v_m':lift_v, 'a_m':lift_a}), 
                    'joint_wrist_yaw': (None, {'v_r':yaw_v, 'a_r':yaw_a}),
                    'joint_wrist_pitch': (None, {'v_r':pitch_v, 'a_r':pitch_a}),
                    'joint_wrist_roll': (None, {'v_r':roll_v, 'a_r':roll_a}),
                    'stretch_gripper' : (None, {'v_r':gripper_v, 'a_r':gripper_a})
                }

                
            self.all_speeds[speed] = {'move_to_functions': move_to_functions, 'move_to_settings': move_to_settings}
                
        

    def clip_configuration(self, config, min_config, max_config):
        pass
        
    def to_configuration(self, config, valid_joints=None, speed=None):
        if speed is None:
            move_to_functions = self.move_to_functions
        else:
            if speed not in self.all_speeds.keys():
                print('WARNING: RobotMove create_commands called with unrecognized speed = \'' + speed + '\'')
                print('         Using \'default\' speed instead.')
                speed = 'default'
            move_to_functions = self.all_speeds[speed]['move_to_functions']
            
        for j in config.keys():
            if valid_joints is None:
                move_to_functions[j](config[j])
            elif j in valid_joints: 
                move_to_functions[j](config[j])
            

                
if __name__ == '__main__':
    
    robot = stretch_body.robot.Robot()
    robot.startup()
    
    robot_move = RobotMove(robot, speed='slow')
    robot_move.print_settings()

    starting_configuration = {
        'joint_mobile_base_rotate_by': 0.0,
        'joint_lift': 0.7,
        'joint_arm_l0': 0.01,
        'joint_wrist_yaw': 0.9 * 3.14,
        'joint_wrist_pitch': 0.0,
        'joint_wrist_roll': 0.0
    }
    
    robot_move.to_configuration(starting_configuration, speed='slow')
    robot.push_command()
    robot.wait_command()
    robot.stop()
    
