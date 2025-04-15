#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class InitialPoseNode(Node):
    """
    ROS2节点，用于设置AMCL的初始位姿
    """

    def __init__(self):
        super().__init__('initial_pose_node')
        
        # 声明参数
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', -1.57)
        self.declare_parameter('wait_for_map_timeout', 30.0)  # 等待map坐标系的超时时间（秒）
        
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
        
        # 创建TF监听器，用于检查map坐标系是否可用
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 日志输出
        self.get_logger().info('初始位姿节点已初始化')
        
    def wait_for_map_frame(self, timeout=30.0):
        """
        等待map坐标系可用
        
        Args:
            timeout (float): 超时时间（秒）
            
        Returns:
            bool: 如果map坐标系可用返回True，否则返回False
        """
        self.get_logger().info(f'等待map坐标系可用，超时时间: {timeout}秒')
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # 尝试获取map到base_link的转换，检查map坐标系是否可用
                # 如果map坐标系不可用，会抛出异常
                self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info('map坐标系已可用')
                return True
            except TransformException as ex:
                # 如果是因为找不到坐标系而失败，继续等待
                self.get_logger().info(f'等待map坐标系: {ex}')
                rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().error(f'等待map坐标系超时（{timeout}秒）')
        return False
    
    def set_initial_pose(self, x, y, z=0.0, yaw=0.0):
        """
        设置AMCL的初始位姿（优化：等待AMCL订阅并多次发布）
        
        Args:
            x (float): 初始位置的x坐标（米）
            y (float): 初始位置的y坐标（米）
            z (float): 初始位置的z坐标（米）
            yaw (float): 初始朝向（弧度）
        """
        self.get_logger().info(f'设置初始位姿: x={x}, y={y}, z={z}, yaw={yaw}')
        
        # 首先等待map坐标系可用
        timeout = self.get_parameter('wait_for_map_timeout').value
        if not self.wait_for_map_frame(timeout):
            self.get_logger().error('无法设置初始位姿：map坐标系不可用')
            return
        
        # 创建初始位姿消息
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # 设置位置
        initial_pose.pose.pose.position.x = float(x)
        initial_pose.pose.pose.position.y = float(y)
        initial_pose.pose.pose.position.z = float(z)
        
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


def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    # 创建初始位姿节点
    initial_pose_node = InitialPoseNode()
    
    try:
        # 从参数获取初始位姿
        initial_x = initial_pose_node.get_parameter('initial_x').value
        initial_y = initial_pose_node.get_parameter('initial_y').value
        initial_z = initial_pose_node.get_parameter('initial_z').value
        initial_yaw = initial_pose_node.get_parameter('initial_yaw').value
        
        # 设置初始位姿
        initial_pose_node.get_logger().info(f'从参数获取的初始位姿: x={initial_x}, y={initial_y}, z={initial_z}, yaw={initial_yaw}')
        initial_pose_node.set_initial_pose(initial_x, initial_y, initial_z, initial_yaw)
        
        # 任务完成后关闭节点
        initial_pose_node.get_logger().info('初始位姿设置完成，关闭节点')
        
    except KeyboardInterrupt:
        initial_pose_node.get_logger().info('用户中断操作')
    finally:
        # 清理资源
        initial_pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
