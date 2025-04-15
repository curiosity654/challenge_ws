#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math


class NavigationClient(Node):
    """
    ROS2节点，用于向TurtleBot3发送导航到特定位置的指令
    """

    def __init__(self):
        super().__init__('navigation_client')
        
        # 声明参数
        self.declare_parameter('x_pose', 1.0)
        self.declare_parameter('y_pose', 1.0)
        self.declare_parameter('yaw', 1.57)  # 约90度，弧度制
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', -1.57)
        self.declare_parameter('set_initial_pose', True)
        
        # 创建ActionClient，用于发送导航请求
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 创建发布者，用于设置AMCL的初始位姿
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            qos_profile
        )
        
        # 日志输出
        self.get_logger().info('导航客户端已初始化')
        
    def set_initial_pose(self, x, y, yaw=0.0):
        """
        设置AMCL的初始位姿（优化：等待AMCL订阅并多次发布）
        
        Args:
            x (float): 初始位置的x坐标（米）
            y (float): 初始位置的y坐标（米）
            yaw (float): 初始朝向（弧度）
        """
        self.get_logger().info(f'设置初始位姿: x={x}, y={y}, yaw={yaw}')
        
        # 创建初始位姿消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        initial_pose.pose.pose.position.x = float(x)
        initial_pose.pose.pose.position.y = float(y)
        initial_pose.pose.pose.position.z = 0.0
        
        # 设置朝向（四元数）
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 设置协方差（使用默认值）
        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                initial_pose.pose.covariance[i] = 0.25  # x, y, z 方差
            elif i == 21 or i == 28 or i == 35:
                initial_pose.pose.covariance[i] = 0.06853891945200942  # 旋转方差
            else:
                initial_pose.pose.covariance[i] = 0.0
        
        # 优化1：等待AMCL订阅initialpose
        timeout = 10  # 最多等待10秒
        waited = 0
        while self._initial_pose_pub.get_subscription_count() == 0 and waited < timeout:
            self.get_logger().info('等待AMCL订阅initialpose...')
            rclpy.spin_once(self, timeout_sec=0.5)
            waited += 0.5
        if self._initial_pose_pub.get_subscription_count() == 0:
            self.get_logger().warn('AMCL仍未订阅initialpose，可能会导致初始定位失败')
        else:
            self.get_logger().info('AMCL已订阅initialpose，开始发布初始位姿')
        
        # 优化2：多次发布初始位姿（模仿RViz行为）
        for i in range(10):
            self._initial_pose_pub.publish(initial_pose)
            self.get_logger().info(f'第{i+1}次发布初始位姿')
            rclpy.spin_once(self, timeout_sec=0.2)
        
        self.get_logger().info('初始位姿发布完成，等待AMCL处理...')
        rclpy.spin_once(self, timeout_sec=2.0)
    
    def send_goal(self, x, y, yaw=0.0):
        """
        发送导航目标到Nav2
        
        Args:
            x (float): 目标位置的x坐标（米）
            y (float): 目标位置的y坐标（米）
            yaw (float): 目标朝向（弧度）
        """
        # 等待ActionServer可用
        self.get_logger().info('等待导航服务器...')
        self._action_client.wait_for_server()
        
        # 创建目标位姿消息
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置目标位置
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 设置目标朝向（四元数）
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'发送导航目标: x={x}, y={y}, yaw={yaw}')
        
        # 发送目标并获取结果
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # 添加目标响应回调
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        处理目标请求的响应
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
            
        self.get_logger().info('导航目标被接受')
        
        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        处理导航结果
        """
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # 成功
            self.get_logger().info('导航成功完成!')
        else:
            self.get_logger().error(f'导航失败，状态码: {status}')
    
    def feedback_callback(self, feedback_msg):
        """
        处理导航反馈
        """
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose
        
        # 计算到目标的距离
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        
        self.get_logger().info(f'当前位置: x={x:.2f}, y={y:.2f}')


def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    # 创建导航客户端节点
    nav_client = NavigationClient()
    
    try:
        # 从参数获取目标位置
        x_pose = nav_client.get_parameter('x_pose').value
        y_pose = nav_client.get_parameter('y_pose').value
        yaw = nav_client.get_parameter('yaw').value
        
        # 获取是否需要设置初始位姿
        set_initial_pose = nav_client.get_parameter('set_initial_pose').value
        
        if set_initial_pose:
            # 从参数获取初始位姿
            initial_x = nav_client.get_parameter('initial_x').value
            initial_y = nav_client.get_parameter('initial_y').value
            initial_yaw = nav_client.get_parameter('initial_yaw').value
            
            # 设置初始位姿
            nav_client.get_logger().info(f'设置初始位姿: x={initial_x}, y={initial_y}, yaw={initial_yaw}')
            nav_client.set_initial_pose(initial_x, initial_y, initial_yaw)
        
        # 发送导航目标
        nav_client.get_logger().info(f'从参数获取的目标: x={x_pose}, y={y_pose}, yaw={yaw}')
        nav_client.send_goal(x_pose, y_pose, yaw)
        
        # 保持节点运行，处理回调
        rclpy.spin(nav_client)
        
    except KeyboardInterrupt:
        nav_client.get_logger().info('用户中断导航')
    
    # 清理资源
    nav_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
