#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import time


class WaypointNavigator(Node):
    """
    ROS2节点，用于控制TurtleBot3依次走过多个路径点
    """

    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # 声明参数
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
        
        # 导航状态
        self._is_navigating = False
        self._current_waypoint_index = 0
        
        # 日志输出
        self.get_logger().info('路径点导航器已初始化')
        
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
    
    def navigate_to_waypoints(self, waypoints):
        """
        依次导航到多个路径点
        
        Args:
            waypoints (list): 路径点列表，每个路径点是一个包含x, y, yaw的元组
        """
        if not waypoints:
            self.get_logger().error('路径点列表为空')
            return
            
        self._waypoints = waypoints
        self._current_waypoint_index = 0
        self._is_navigating = False
        
        # 等待ActionServer可用
        self.get_logger().info('等待导航服务器...')
        self._action_client.wait_for_server()
        
        # 开始导航到第一个路径点
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        """
        导航到下一个路径点
        """
        if self._is_navigating:
            self.get_logger().warn('正在导航中，无法开始新的导航任务')
            return
            
        if self._current_waypoint_index >= len(self._waypoints):
            self.get_logger().info('所有路径点导航完成！')
            return
            
        # 获取当前路径点
        waypoint = self._waypoints[self._current_waypoint_index]
        x, y, yaw = waypoint
        
        self.get_logger().info(f'开始导航到路径点 {self._current_waypoint_index+1}/{len(self._waypoints)}: x={x}, y={y}, yaw={yaw}')
        
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
        
        # 发送目标并获取结果
        self._is_navigating = True
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
            self._is_navigating = False
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
        
        self._is_navigating = False
        
        if status == 4:  # 成功
            self.get_logger().info(f'路径点 {self._current_waypoint_index+1}/{len(self._waypoints)} 导航成功完成!')
            
            # 导航到下一个路径点
            self._current_waypoint_index += 1
            
            # 在导航到下一个路径点之前稍微等待一下
            time.sleep(1.0)
            
            # 导航到下一个路径点
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f'导航失败，状态码: {status}')
            
            # 尝试重新导航到当前路径点
            self.get_logger().info(f'尝试重新导航到路径点 {self._current_waypoint_index+1}/{len(self._waypoints)}')
            time.sleep(2.0)
            self.navigate_to_next_waypoint()
    
    def feedback_callback(self, feedback_msg):
        """
        处理导航反馈
        """
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose
        
        # 计算到目标的距离
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y
        
        # 每5秒输出一次当前位置（减少日志量）
        current_time = self.get_clock().now().to_msg().sec
        if current_time % 5 == 0:
            self.get_logger().info(f'当前位置: x={x:.2f}, y={y:.2f}，导航到路径点 {self._current_waypoint_index+1}/{len(self._waypoints)}')


def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    # 创建路径点导航器节点
    waypoint_navigator = WaypointNavigator()
    
    try:
        # 设置初始位姿（如果需要）
        if waypoint_navigator.get_parameter('set_initial_pose').value:
            initial_x = waypoint_navigator.get_parameter('initial_x').value
            initial_y = waypoint_navigator.get_parameter('initial_y').value
            initial_yaw = waypoint_navigator.get_parameter('initial_yaw').value
            
            waypoint_navigator.get_logger().info(f'从参数获取的初始位姿: x={initial_x}, y={initial_y}, yaw={initial_yaw}')
            waypoint_navigator.set_initial_pose(initial_x, initial_y, initial_yaw)
        
        # 定义6个路径点 (x, y, yaw)
        waypoints = [
            (-1.54, -1.2, -1.57),      # 路径点1
            (-0.59, -2.43, 0.25),     # 路径点2
            (-2.90, -1.77, -0.28),     # 路径点3
            (-2.46, -2.7, -1.57),    # 路径点4
            (-2.56, -0.65, 1.57),     # 路径点5
            (-1.93, -0.1, 0.0),       # 路径点6
            (0.0, 0.0, -1.57)       # 回到起点
        ]
        
        # 开始导航
        waypoint_navigator.get_logger().info(f'开始导航，共 {len(waypoints)} 个路径点')
        waypoint_navigator.navigate_to_waypoints(waypoints)
        
        # 保持节点运行，直到所有路径点导航完成
        rclpy.spin(waypoint_navigator)
        
    except KeyboardInterrupt:
        waypoint_navigator.get_logger().info('用户中断操作')
    finally:
        # 清理资源
        waypoint_navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
