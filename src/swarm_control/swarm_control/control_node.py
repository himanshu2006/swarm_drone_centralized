#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class MotherBrainNode(Node):
    def __init__(self):
        super().__init__('mother_brain_node')

        self.FORMATIONS = {
            'triangle': {
                'child_1': [-1.5,  1.5, 0.0, 0.0],  # Back left
                'child_2': [-1.5, -1.5, 0.0, 0.0],  # Back right
                'child_3': [-3.0,  0.0, 0.0, 0.0]   # Straight back
            },
            'line': {
                'child_1': [0.0,  2.5, 0.0, 0.0],  # Far Left Wing
                'child_2': [0.0, -2.5, 0.0, 0.0],  # Far Right Wing
                'child_3': [0.0,  5.0, 0.0, 0.0]   # Outer Left Wing
            },
            'column': {
                'child_1': [-2.0, 0.0, 0.0, 0.0],  # Single file 1st
                'child_2': [-4.0, 0.0, 0.0, 0.0],  # Single file 2nd
                'child_3': [-6.0, 0.0, 0.0, 0.0]   # Single file 3rd
            },
            'diamond': {
                'child_1': [-2.0,  2.0, 0.0, 0.0],  # Left Corner
                'child_2': [-2.0, -2.0, 0.0, 0.0],  # Right Corner
                'child_3': [-4.0,  0.0, 0.0, 0.0]   # Rear Point
            },
            'pyramid': {
                'child_1': [-1.5,  1.5,  1.0, 0.0],  # High Left Tier
                'child_2': [-1.5, -1.5,  1.0, 0.0],  # High Right Tier
                'child_3': [-3.0,  0.0,  2.0, 0.0]   # Peak Rear Tier
            }
        }

        
        self.formation_offsets = {
            'child_1': list(self.FORMATIONS['triangle']['child_1']),
            'child_2': list(self.FORMATIONS['triangle']['child_2']),
            'child_3': list(self.FORMATIONS['triangle']['child_3'])
        }

        self.mother_state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        
       
        self.children_state = {
            'child_1': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'child_2': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
            'child_3': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        }

        self.create_subscription(Odometry, '/model/mother/odometry', self.mother_odom_callback, 10)
        
        for child in self.formation_offsets.keys():
            self.create_subscription(
                Odometry, 
                f'/model/{child}/odometry', 
                lambda msg, name=child: self.child_odom_callback(msg, name), 
                10
            )

        for child in self.formation_offsets.keys():
            self.create_subscription(
                Twist,
                f'/model/mother/move_{child}',
                lambda msg, name=child: self.modify_offset_callback(msg, name),
                10
            )

        self.create_subscription(
            String,
            '/swarm/select_formation',
            self.formation_switch_callback,
            10
        )

        # self.mother_pub = self.create_publisher(Twist, '/model/mother/cmd_vel', 10)
        # self.mother_target_z = 0.5

        self.child_pubs = {}
        for child in self.formation_offsets.keys():
            self.child_pubs[child] = self.create_publisher(Twist, f'/model/{child}/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.master_control_loop)
        self.get_logger().info('Swarm Brain Started.')


    def formation_switch_callback(self, msg):
        fmt_name = msg.data.lower().strip()
        if fmt_name in self.FORMATIONS:
            preset = self.FORMATIONS[fmt_name]
            for child in self.formation_offsets.keys():
                self.formation_offsets[child] = list(preset[child])
    
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
        
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.children_state[child_name]['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def modify_offset_callback(self, msg, child_name):
        dt = 0.05 
        
        
        current_yaw = self.formation_offsets[child_name][3]
        
        
        local_move_x = msg.linear.x * dt
        local_move_y = msg.linear.y * dt
        
        
        rotated_move_x = (local_move_x * math.cos(current_yaw)) - (local_move_y * math.sin(current_yaw))
        rotated_move_y = (local_move_x * math.sin(current_yaw)) + (local_move_y * math.cos(current_yaw))
        
        
        self.formation_offsets[child_name][0] += rotated_move_x
        self.formation_offsets[child_name][1] += rotated_move_y
        
        
        self.formation_offsets[child_name][2] += msg.linear.z * dt
        self.formation_offsets[child_name][3] += msg.angular.z * dt

    def master_control_loop(self):
        Kp = 2.5
        Kp_yaw = 2.0 
        
        xm, ym, zm, yaw = self.mother_state['x'], self.mother_state['y'], self.mother_state['z'], self.mother_state['yaw']

        for child, offset in self.formation_offsets.items():
            
            
            target_x = xm + (offset[0] * math.cos(yaw) - offset[1] * math.sin(yaw))
            target_y = ym + (offset[0] * math.sin(yaw) + offset[1] * math.cos(yaw))
            target_z = zm + offset[2] + 0.5 
            
            
            target_yaw = yaw + offset[3] 

            
            err_x_global = target_x - self.children_state[child]['x']
            err_y_global = target_y - self.children_state[child]['y']
            err_z_global = target_z - self.children_state[child]['z']
            
            
            child_yaw = self.children_state[child]['yaw']
            err_x_local = (err_x_global * math.cos(child_yaw)) + (err_y_global * math.sin(child_yaw))
            err_y_local = (-err_x_global * math.sin(child_yaw)) + (err_y_global * math.cos(child_yaw))
            
            
            err_yaw = target_yaw - child_yaw
            

            err_yaw = math.atan2(math.sin(err_yaw), math.cos(err_yaw))

            cmd = Twist()
            
            cmd.linear.x = Kp * err_x_local
            cmd.linear.y = Kp * err_y_local
            cmd.linear.z = Kp * err_z_global
            cmd.angular.z = Kp_yaw * err_yaw

            
            cmd.linear.x = max(min(cmd.linear.x, 2.5), -2.5)
            cmd.linear.y = max(min(cmd.linear.y, 2.5), -2.5)
            cmd.linear.z = max(min(cmd.linear.z, 1.5), -1.5)
            cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0) 

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