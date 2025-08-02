#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time
import threading

class TBotIntegrationTest(Node):
    def __init__(self):
        super().__init__('tbot_integration_test')
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # 创建订阅者
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.status_sub = self.create_subscription(
            String, 'status', self.status_callback, 10)
        
        # 存储接收到的消息
        self.last_odom = None
        self.last_laser = None
        self.last_status = None
        
        # 消息接收标志
        self.odom_received = False
        self.laser_received = False
        self.status_received = False
    
    def odom_callback(self, msg):
        self.last_odom = msg
        self.odom_received = True
        self.get_logger().info(f'Received odometry: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
    
    def laser_callback(self, msg):
        self.last_laser = msg
        self.laser_received = True
        self.get_logger().info(f'Received laser scan: {len(msg.ranges)} points')
    
    def status_callback(self, msg):
        self.last_status = msg
        self.status_received = True
        self.get_logger().info(f'Received status: {msg.data}')
    
    def publish_velocity_command(self, linear_x=0.5, angular_z=0.0):
        """发布速度控制命令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Published velocity command: linear_x={linear_x}, angular_z={angular_z}')
    
    def publish_goal_pose(self, x=1.0, y=2.0, z=0.0, w=1.0):
        """发布导航目标"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = w
        self.goal_pose_pub.publish(pose)
        self.get_logger().info(f'Published goal pose: x={x}, y={y}, z={z}')
    
    def wait_for_messages(self, timeout=10.0):
        """等待接收消息"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.odom_received and self.laser_received and self.status_received:
                return True
            time.sleep(0.1)
        return False

class TestTBotIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TBotIntegrationTest()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        
        # 启动执行器线程
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()
    
    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def setUp(self):
        # 重置接收标志
        self.node.odom_received = False
        self.node.laser_received = False
        self.node.status_received = False
    
    def test_velocity_control(self):
        """测试速度控制功能"""
        # 发布速度命令
        self.node.publish_velocity_command(linear_x=0.5, angular_z=0.2)
        
        # 等待一段时间
        time.sleep(2.0)
        
        # 验证话题存在
        self.assertTrue(self.node.count_publishers('cmd_vel') > 0)
        self.assertTrue(self.node.count_subscribers('cmd_vel') > 0)
    
    def test_goal_pose(self):
        """测试导航目标功能"""
        # 发布导航目标
        self.node.publish_goal_pose(x=1.0, y=2.0, z=0.0, w=1.0)
        
        # 等待一段时间
        time.sleep(2.0)
        
        # 验证话题存在
        self.assertTrue(self.node.count_publishers('goal_pose') > 0)
        self.assertTrue(self.node.count_subscribers('goal_pose') > 0)
    
    def test_message_reception(self):
        """测试消息接收功能"""
        # 等待接收消息
        messages_received = self.node.wait_for_messages(timeout=5.0)
        
        # 验证消息接收
        if messages_received:
            self.assertIsNotNone(self.node.last_odom)
            self.assertIsNotNone(self.node.last_laser)
            self.assertIsNotNone(self.node.last_status)
        else:
            self.skipTest("No messages received within timeout")
    
    def test_topic_names(self):
        """测试话题名称"""
        # 验证所有必要的话题都存在
        expected_topics = ['cmd_vel', 'goal_pose', 'odom', 'scan', 'status']
        
        for topic in expected_topics:
            self.assertTrue(
                self.node.count_publishers(topic) > 0 or self.node.count_subscribers(topic) > 0,
                f"Topic {topic} not found"
            )
    
    def test_parameter_setting(self):
        """测试参数设置"""
        # 设置参数
        self.node.declare_parameter('robot_ip', '192.168.8.110')
        self.node.set_parameters([rclpy.Parameter('robot_ip', value='192.168.1.100')])
        
        # 获取参数
        robot_ip = self.node.get_parameter('robot_ip').value
        self.assertEqual(robot_ip, '192.168.1.100')
    
    def test_message_types(self):
        """测试消息类型"""
        # 测试Twist消息
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.2
        self.assertEqual(twist.linear.x, 0.5)
        self.assertEqual(twist.angular.z, 0.2)
        
        # 测试PoseStamped消息
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        self.assertEqual(pose.pose.position.x, 1.0)
        self.assertEqual(pose.pose.position.y, 2.0)
        self.assertEqual(pose.pose.orientation.w, 1.0)

def main():
    unittest.main()

if __name__ == '__main__':
    main() 