from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time

IMAGE_WIDTH = 1400

# TODO: Add your new constants here
#ALL CONSTANTS WITH TODO NEXT TO THEM ARE NOT SET YET 1 IS A RANDOM CONSTANT
TIMEOUT = 1 #TODO threshold in timer_callback
SEARCH_YAW_VEL = .5 #TODO searching constant
TRACK_FORWARD_VEL = .25 #TODO tracking constant
KP = 1 #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    TRACK = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = 1 # TODO
        self.center_x = 0
        self.time = 0

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """

        if msg.detections:
            best_idx = -1
            closest_x = 2
            for dct_idx in range(len(msg.detections)):
                x = ((((msg.detections[dct_idx].bbox).center).position).x)
                x_val_norm = (2*x - IMAGE_WIDTH)/IMAGE_WIDTH
                if abs(x_val_norm) < closest_x:
                    closest_x = x_val_norm
                    best_idx = dct_idx
            
            # import pdb; pdb.set_trace()
            # breakpoint()
            print("closest to center position is: ")            
            print(closest_x)
            self.center_x = closest_x
            self.time = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**-9)
            print("timestamp:")
            print(self.time)

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        # import pdb; pdb.set_trace()
        if (time.time() - self.time) > TIMEOUT: # Part 3.2
            #breakpoint()
            self.state = State.SEARCH
        else:
            self.state = State.TRACK

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH: #part 3.1
            yaw_command = SEARCH_YAW_VEL * -np.sign(self.center_x)
            pass

        elif self.state == State.TRACK:
            K_p = 1.5
            yaw_command = -K_p*self.center_x
            forward_vel_command = TRACK_FORWARD_VEL

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
