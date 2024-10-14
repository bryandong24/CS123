import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import pi
np.set_printoptions(precision=3, suppress=True)

Kp = 3
Kd = 0.1

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

       self.pd_timer_period = 1.0 / 200  # 200 Hz
       self.ik_timer_period = 1.0 / 20   # 10 Hz
       self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
       self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

       self.joint_positions = None
       self.joint_velocities = None
       self.target_joint_positions = None

       self.ee_triangle_positions = np.array([
           [0.05, 0.0, -0.12],  # Touchdown
           [-0.05, 0.0, -0.12], # Liftoff
           [0.0, 0.0, -0.06]    # Mid-swing
       ])

       center_to_rf_hip = np.array([0.07500, -0.08350, 0])
       self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
       self.current_target = 0
       self.t = 0

   def listener_callback(self, msg):
       joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
       self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
       self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

   def forward_kinematics(self, theta1, theta2, theta3):
       def rotation_x(angle):
           # rotation about the x-axis implemented for you
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
       
       # RGB -> x,y,z

       # T_0_1 (base_link to leg_front_r_1)
       T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(pi/2) @ rotation_z(theta1)

       # T_1_2 (leg_front_r_1 to leg_front_r_2)
       T_1_2 = translation(0, 0, 0.039) @ rotation_y(3*pi/2) @ rotation_z(theta2) # Changed Z translation from 0.00039 to 0.039

       # T_2_3 (leg_front_r_2 to leg_front_r_3)
       T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(pi/2) @rotation_z(theta3)

       # T_3_ee (leg_front_r_3 to end-effector)
       T_3_ee = translation(0.06231, -.06216, 0.018)

       # Compute the final transformation
       T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

       # Extract the end-effector position
       end_effector_position = T_0_ee[:3, 3]

       return end_effector_position


   def inverse_kinematics(self, target_ee, initial_guess=[0, 0, 0]):
       def cost_function(theta):
           # Compute the cost function and the L1 norm of the error
           current_ee = self.forward_kinematics(*theta)
           error = target_ee - current_ee
           l1_norm = np.abs(error)
           cost = np.sum(l1_norm**2)  # squared 2-norm of the error vector
           return cost, l1_norm

       def gradient(theta, epsilon=1e-3):
           # TODO: Implement the gradient computation            
           # Compute the gradient of the cost function using finite differences
           grad = np.zeros_like(theta)
           for i in range(len(theta)):
               theta_plus = theta.copy()
               theta_plus[i] += epsilon
               cost_plus, l1 = cost_function(theta_plus)
               
               theta_minus = theta.copy()
               theta_minus[i] -= epsilon
               cost_minus, l1 = cost_function(theta_minus)
               
               grad[i] = (cost_plus - cost_minus) / (2 * epsilon)
           return grad

       theta = np.array(initial_guess)
       learning_rate = 10 # TODO: Set the learning rate
       max_iterations = 50 # TODO: Set the maximum number of iterations
       tolerance = 1e-3 # TODO: Set the tolerance for the L1 norm of the error

       cost_l = []
       for _ in range(max_iterations):
           grad = gradient(theta)

           # Update the theta (parameters) using the gradient and the learning rate
           ################################################################################################
           # TODO: Implement the gradient update
           # TODO (BONUS): Implement the (quasi-)Newton's method for faster convergence
           theta -= learning_rate * grad
           ################################################################################################

           cost, l1 = cost_function(theta)
           # cost_l.append(cost)
           if l1.mean() < tolerance:
               break

       # print(f'Cost: {cost_l}')

       return theta

   def interpolate_triangle(self, t):
       # Interpolate between the three triangle positions in the self.ee_triangle_positions
       # based on the current time t
       t_normalized = t % 3  # Normalize t to repeat every 3 seconds
       
       if t_normalized < 1:
           # Interpolate between vertex 1 and 2
           x = np.interp(t_normalized, [0,1], [self.ee_triangle_positions[0][0], self.ee_triangle_positions[1][0]])
           y = 0
           z = np.interp(t_normalized, [0,1], [self.ee_triangle_positions[0][2], self.ee_triangle_positions[1][2]])
           return np.array([x, y, z])
       elif t_normalized < 2:
           # Interpolate between vertex 2 and 3
           x = np.interp(t_normalized, [1,2], [self.ee_triangle_positions[1][0], self.ee_triangle_positions[2][0]])
           y = 0
           z = np.interp(t_normalized, [1,2], [self.ee_triangle_positions[1][2], self.ee_triangle_positions[2][2]])
           return np.array([x, y, z])
       else:
           # Interpolate between vertex 3 and 1
           x = np.interp(t_normalized, [2,3], [self.ee_triangle_positions[2][0], self.ee_triangle_positions[0][0]])
           y = 0
           z = np.interp(t_normalized, [2,3], [self.ee_triangle_positions[2][2], self.ee_triangle_positions[0][2]])
           return np.array([x, y, z])


   def ik_timer_callback(self):
       if self.joint_positions is not None:
           target_ee = self.interpolate_triangle(self.t)
           self.target_joint_positions = self.inverse_kinematics(target_ee, self.joint_positions)
           current_ee = self.forward_kinematics(*self.joint_positions)

           # update the current time for the triangle interpolation
           ################################################################################################
           # TODO: Implement the time update
           ################################################################################################
           self.t += self.ik_timer_period
           
           self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {self.forward_kinematics(*self.target_joint_positions)}, Current Angles: {self.joint_positions}')

   def pd_timer_callback(self):
       if self.target_joint_positions is not None:
            torques = Kp*(target_ee - current_ee) + Kd*(0 - joint_velocities)
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
       zero_torques.data = [0.0, 0.0, 0.0]
       inverse_kinematics.command_publisher.publish(zero_torques)
       
       inverse_kinematics.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
   main()