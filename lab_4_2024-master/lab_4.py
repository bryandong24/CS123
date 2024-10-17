import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)


Kp = 3
Kd = 0.1


def rotation_x(angle):
   return np.array([
                   [1, 0, 0, 0],
                   [0, np.cos(angle), -np.sin(angle), 0],
                   [0, np.sin(angle), np.cos(angle), 0],
                   [0, 0, 0, 1]
               ])




def rotation_y(angle):
   return np.array([
                   [np.cos(angle), 0, np.sin(angle), 0],
                   [0, 1, 0, 0],
                   [-np.sin(angle), 0, np.cos(angle), 0],
                   [0, 0, 0, 1]
               ])


def rotation_z(angle):
   return np.array([
                   [np.cos(angle), -np.sin(angle), 0, 0],
                   [np.sin(angle), np.cos(angle), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]
               ])


def translation(x, y, z):
   return np.array([
                   [1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]
               ])


class InverseKinematics(Node):


   def __init__(self):
       super().__init__('inverse_kinematics')
       self.joint_subscription = self.create_subscription(
           JointState,
           'joint_states',
           self.listener_callback,
           10)
       self.joint_subscription  # prevent unused variable warning


       self.command_publisher = self.create_publisher(
           Float64MultiArray,
           '/forward_command_controller/commands',
           10
       )


       self.joint_positions = None
       self.joint_velocities = None
       self.target_joint_positions = None
       self.counter = 0
       self.current_target = 0 #WE ADDED THIS. IT WAS IN OUR OLD LAB


       # Trotting gate positions, already implemented
       touch_down_position = np.array([0.05, 0.0, -0.14]) #INCREASE X VALUE TO MAKE TRIANGLE HAVE LONGER BASE
       stand_position_1 = np.array([0.025, 0.0, -0.14])
       stand_position_2 = np.array([0.0, 0.0, -0.14])
       stand_position_3 = np.array([-0.025, 0.0, -0.14])
       liftoff_position = np.array([-0.05, 0.0, -0.14]) #DECREASE X VALUE TO MAKE TRIANGLE HAVE LONGER BASE
       mid_swing_position = np.array([0.0, 0.0, -0.05]) #INCREASE Z VALUE IF YOU WANT A HIGHER GAIT
      
       ## trotting
       # TODO: Implement each leg’s trajectory in the trotting gait.
       ## trotting
       # TODO: Implement each leg's trajectory in the trotting gait.
       rf_ee_offset = np.array([0.06, -0.09, 0])
       rf_ee_triangle_positions = np.array([
           touch_down_position,
           liftoff_position,
           mid_swing_position
       ]) + rf_ee_offset


       lf_ee_offset = np.array([0.06, 0.09, 0])
       lf_ee_triangle_positions = np.array([
           touch_down_position,
           liftoff_position,
           mid_swing_position
       ]) + lf_ee_offset


       rb_ee_offset = np.array([-0.11, -0.09, 0])
       rb_ee_triangle_positions = np.array([
           touch_down_position,
           liftoff_position,
           mid_swing_position
       ]) + rb_ee_offset


       lb_ee_offset = np.array([-0.11, 0.09, 0])
       lb_ee_triangle_positions = np.array([
           touch_down_position,
           liftoff_position,
           mid_swing_position
       ]) + lb_ee_offset


       # DELIVERABLE: Alternative gaits for Pupper
       # 1. Bounding gait: Front legs move together, then back legs. Useful for high-speed locomotion on flat terrain.
       # 2. Crawling gait: Move one leg at a time. Useful for precise foot placement on very rough terrain.
       # 3. Galloping gait: Similar to horse gallop. Useful for maximum speed on open terrain.






       self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
       self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.lb_leg_fk]


       self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
       print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
       print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')




       self.pd_timer_period = 1.0 / 200  # 200 Hz
       self.ik_timer_period = 1.0 / 100   # 10 Hz
       self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
       self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)




   def fr_leg_fk(self, theta):
       # Already implemented in Lab 2
       T_RF_0_1 = translation(0.07500, -0.04450, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
       T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
       T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
       T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
       return T_RF_0_ee[:3, 3]


   def fl_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_LF_0_1 = translation(0.07500, 0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_LF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
       T_LF_2_3 = translation(0, 0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
       T_LF_3_ee = translation(0.06231, 0.06216, 0.01800)
       T_LF_0_ee = T_LF_0_1 @ T_LF_1_2 @ T_LF_2_3 @ T_LF_3_ee
       return T_LF_0_ee[:3,3]




   def br_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_RB_0_1 = translation(-0.07500, -0.03350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_RB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
       T_RB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
       T_RB_3_ee = translation(-0.06231, -0.06216, 0.01800)
       T_RB_0_ee = T_RB_0_1 @ T_RB_1_2 @ T_RB_2_3 @ T_RB_3_ee
       return T_RB_0_ee[:3,3]


   def lb_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_LB_0_1 = translation(-0.07500, 0.03350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_LB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
       T_LB_2_3 = translation(0, 0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
       T_LB_3_ee = translation(-0.06231, 0.06216, 0.01800)
       T_LB_0_ee = T_LB_0_1 @ T_LB_1_2 @ T_LB_2_3 @ T_LB_3_ee
       return T_LB_0_ee[:3,3]


   def forward_kinematics(self, theta):
       return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])


   def listener_callback(self, msg):
       joints_of_interest = [
           'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
           'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
           'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
           'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
       ]
       self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
       self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])


   def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
       leg_forward_kinematics = self.fk_functions[leg_index]


       def cost_function(theta):
           current_ee = leg_forward_kinematics(theta)
           error = target_ee - current_ee
           l1_norm = np.abs(error)
           cost = np.sum(l1_norm**2)  # squared 2-norm of the error vector
           return cost, l1_norm
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################


       def gradient(theta, epsilon=1e-3):
           grad = np.zeros_like(theta)
           for i in range(len(theta)):
               theta_plus = theta.copy()
               theta_plus[i] += epsilon
               cost_plus, l1 = cost_function(theta_plus)


               theta_minus = theta.copy()
               theta_minus[i] -= epsilon
               cost_minus, l1 = cost_function(theta_minus)


               grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################
           return grad


       theta = np.array(initial_guess)
       learning_rate = 10 # TODO:[already done] paste lab 3 inverse kinematics here
       max_iterations = 50 # TODO: [already done] paste lab 3 inverse kinematics here
       tolerance = 1e-3 # TODO: [already done] paste lab 3 inverse kinematics here


       cost_l = []
       for _ in range(max_iterations):
           grad = gradient(theta)
           theta -= learning_rate * grad
           ################################################################################################


           cost, l1 = cost_function(theta)
           # cost_l.append(cost)
           if l1.mean() < tolerance:
               break
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################
           continue


       return theta


   def interpolate_triangle(self, t, leg_index):
       ################################################################################################
       # TODO: implement interpolation for all 4 legs here
       ################################################################################################
       #
  
       # Normalize t to repeat every 6 seconds (full gait cycle)
       cycle_time = 6.0
       t_normalized = t % cycle_time


       # Define phase shifts for each leg
       phase_shifts = [0, cycle_time/2, cycle_time/2, 0]  # FR, FL, BR, BL


       # Apply phase shift
       t_shifted = (t_normalized + phase_shifts[leg_index]) % cycle_time


       # Get the triangle positions for the current leg
       leg_positions = self.ee_triangle_positions[leg_index]


       if t_shifted < cycle_time/2.0:
           x = np.interp(t_shifted, [0, 1], [leg_positions[0][0], leg_positions[1][0]])
           y = 0
           z = np.interp(t_shifted, [0, 1], [leg_positions[0][2], leg_positions[1][2]])
       elif t_shifted < cycle_time*3.0/4.0:
           # Interpolate between touch down and stand position 1
           x = np.interp(t_shifted, [1, 2], [leg_positions[1][0], leg_positions[2][0]])
           y = 0
           z = np.interp(t_shifted, [1, 2], [leg_positions[1][2], leg_positions[2][2]])
       else:
           # Interpolate between liftoff position and mid swing position
           x = np.interp(t_shifted, [2, 3], [leg_positions[2][0], leg_positions[0][0]])
           y = 0
           z = np.interp(t_shifted, [2, 3], [leg_positions[2][2], leg_positions[0][2]])


       return np.array([x, y, z])






   def cache_target_joint_positions(self):
       # Calculate and store the target joint positions for a cycle and all 4 legs
       target_joint_positions_cache = []
       target_ee_cache = []
       for leg_index in range(4):
           target_joint_positions_cache.append([])
           target_ee_cache.append([])
           target_joint_positions = [0] * 3
           for t in np.arange(0, 1, 0.02):
               print(t)
               target_ee = self.interpolate_triangle(t, leg_index)
               target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)


               target_joint_positions_cache[leg_index].append(target_joint_positions)
               target_ee_cache[leg_index].append(target_ee)


       # (4, 50, 3) -> (50, 12)
       target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
       target_ee_cache = np.concatenate(target_ee_cache, axis=1)
      
       return target_joint_positions_cache, target_ee_cache


   def get_target_joint_positions(self):
       target_joint_positions = self.target_joint_positions_cache[self.counter]
       target_ee = self.target_ee_cache[self.counter]
       self.counter += 1
       if self.counter >= self.target_joint_positions_cache.shape[0]:
           self.counter = 0
       return target_ee, target_joint_positions


   def ik_timer_callback(self):
       if self.joint_positions is not None:
           target_ee, self.target_joint_positions = self.get_target_joint_positions()
           current_ee = self.forward_kinematics(self.joint_positions)


           self.get_logger().info(
               f'Target EE: {target_ee}, \
               Current EE: {current_ee}, \
               Target Angles: {self.target_joint_positions}, \
               Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
               Current Angles: {self.joint_positions}')


   def pd_timer_callback(self):
       if self.target_joint_positions is not None:
           torques = Kp*(self.target_joint_positions - self.joint_positions) + Kd*(0 - self.joint_velocities) #WE SWITCHED CURRENT WITH TARGET
           command_msg = Float64MultiArray()
           command_msg.data = torques.tolist()
           self.command_publisher.publish(command_msg)


def main():
   rclpy.init()
   inverse_kinematics = InverseKinematics()
  
   try:
       rclpy.spin(inverse_kinematics)
   except KeyboardInterrupt:
       print("Program terminated by user")
   finally:
       # Send zero torques
       zero_torques = Float64MultiArray()
       zero_torques.data = [0.0] * 12
       inverse_kinematics.command_publisher.publish(zero_torques)
      
       inverse_kinematics.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()





