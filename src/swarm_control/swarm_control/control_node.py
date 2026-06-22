#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MotherBrainNode(Node):
    def __init__(self):
        super().__init__('mother_brain_node')

        
        self.formation_offsets = {
            'child_1': [-1.5,  1.5, 0.0],  # 1.5m behind, 1.5m left
            'child_2': [-1.5, -1.5, 0.0],  # 1.5m behind, 1.5m right
            'child_3': [-3.0,  0.0, 0.0]   # 3.0m straight behind
        }

        
        self.mother_state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.children_state = {
            'child_1': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'child_2': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'child_3': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }

        
        self.create_subscription(Odometry, '/model/mother/odometry', self.mother_odom_callback, 10)
        
        
        for child in self.formation_offsets.keys():
            self.create_subscription(
                Odometry, 
                f'/model/{child}/odometry', 
                lambda msg, name=child: self.child_odom_callback(msg, name), 
                10
            )

        
        self.child_pubs = {}
        for child in self.formation_offsets.keys():
            self.child_pubs[child] = self.create_publisher(Twist, f'/model/{child}/cmd_vel', 10)

        
        self.timer = self.create_timer(0.05, self.master_control_loop)
        self.get_logger().info('Mother Brain is online. Taking control of the swarm.')

    
    def mother_odom_callback(self, msg):
        self.mother_state['x'] = msg.pose.pose.position.x
        self.mother_state['y'] = msg.pose.pose.position.y
        self.mother_state['z'] = msg.pose.pose.position.z

        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.mother_state['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def child_odom_callback(self, msg, child_name):
        self.children_state[child_name]['x'] = msg.pose.pose.position.x
        self.children_state[child_name]['y'] = msg.pose.pose.position.y
        self.children_state[child_name]['z'] = msg.pose.pose.position.z

    
    def master_control_loop(self):
        Kp = 2.0  

        
        xm = self.mother_state['x']
        ym = self.mother_state['y']
        zm = self.mother_state['z']
        yaw = self.mother_state['yaw']

        
        for child, offset in self.formation_offsets.items():
            
            
            target_x = xm + (offset[0] * math.cos(yaw) - offset[1] * math.sin(yaw))
            target_y = ym + (offset[0] * math.sin(yaw) + offset[1] * math.cos(yaw))
            target_z = zm + offset[2] #+ 0.5  # Maintain 0.5m flight altitude

        
            err_x = target_x - self.children_state[child]['x']
            err_y = target_y - self.children_state[child]['y']
            err_z = target_z - self.children_state[child]['z']

            
            cmd = Twist()
            cmd.linear.x = Kp * err_x
            cmd.linear.y = Kp * err_y
            cmd.linear.z = Kp * err_z

            #Safety Limits (Don't command the children to fly too fast)
            cmd.linear.x = max(min(cmd.linear.x, 2.5), -2.5)
            cmd.linear.y = max(min(cmd.linear.y, 2.5), -2.5)
            cmd.linear.z = max(min(cmd.linear.z, 1.5), -1.5)

            # Broadcast the command to the child
            self.child_pubs[child].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotherBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()