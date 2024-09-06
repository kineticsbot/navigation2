
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from tf_transformations import quaternion_from_matrix
from tf_transformations import euler_matrix
from tf_transformations import inverse_matrix
import tf_transformations

from tf2_ros import TransformBroadcaster




class FakeLocalization(Node):
    def __init__(self,node_name='fake_localization',namespace=''):
        super().__init__(node_name=node_name,namespace=namespace)
        self.cmd_vel_ = Twist()
        self.cmd_vel_.linear.x = 0.0
        self.cmd_vel_.linear.y = 0.0
        self.cmd_vel_.linear.z = 0.0
        self.cmd_vel_.angular.x = 0.0
        self.cmd_vel_.angular.y = 0.0
        self.cmd_vel_.angular.z = 0.0

        self.x_ = 0
        self.y_ = 0
        self.theta_ = 0

        self.map_x_ = 0
        self.map_y_ = 0
        self.map_theta_ = 0

        self.br = TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.timer = self.create_timer(0.1, self.odom_callback)
        self.last_time = self.get_clock().now()

        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            10
        )
 
        pass

    def init_pose_callback(self,msg):
        self.map_x_ = msg.pose.pose.position.x
        self.map_y_ = msg.pose.pose.position.y
        self.map_theta_ = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
        pass

    def odom_callback(self):

       # 获取当前时间并计算时间差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
        self.last_time = current_time 

        # 从cmd_vel消息中获取线速度和角速度
        vx = self.cmd_vel_.linear.x
        vy = self.cmd_vel_.linear.y  # 一般情况下，vy在常见的差动驱动机器人中为0
        v_theta = self.cmd_vel_.angular.z

        # 使用tf_transformations库进行变换计算
        # 创建初始的位姿变换矩阵
        odom_to_base = euler_matrix(0, 0, self.theta_)
        odom_to_base[0, 3] = self.x_
        odom_to_base[1, 3] = self.y_

        # 创建移动变换矩阵
        movement = euler_matrix(0, 0, v_theta * dt)
        movement[0, 3] = vx * dt
        movement[1, 3] = vy * dt

        # 计算新的位姿
        new_odom_to_base = tf_transformations.concatenate_matrices(odom_to_base, movement)

        # 提取更新后的位姿
        self.x_ = new_odom_to_base[0, 3]
        self.y_ = new_odom_to_base[1, 3]
        ori =  quaternion_from_matrix(new_odom_to_base)
        self.theta_ = euler_from_quaternion(ori)[2]

        map_to_base = euler_matrix(0, 0, self.map_theta_)
        map_to_base[0, 3] = self.map_x_
        map_to_base[1, 3] = self.map_y_
        new_map_to_base = tf_transformations.concatenate_matrices(map_to_base, movement)
        self.map_x_ = new_map_to_base[0, 3]
        self.map_y_ = new_map_to_base[1, 3]
        ori_map =  quaternion_from_matrix(new_map_to_base)
        self.map_theta_ = euler_from_quaternion(ori_map)[2]

        # 创建并发布新的Odometry消息

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.position.z = 0.
        odom.pose.pose.orientation.x = ori[0]
        odom.pose.pose.orientation.y = ori[1]
        odom.pose.pose.orientation.z = ori[2]
        odom.pose.pose.orientation.w = ori[3]
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.
        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = v_theta
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.
        t.transform.rotation.x = ori[0]
        t.transform.rotation.y = ori[1]
        t.transform.rotation.z = ori[2]
        t.transform.rotation.w = ori[3]
        self.br.sendTransform(t)

        # 计算map 和 odom的变换
                                 

        t_map_odom = tf_transformations.concatenate_matrices(new_map_to_base,inverse_matrix(new_odom_to_base))

        t_map_odom_stamped = TransformStamped()
        t_map_odom_stamped.header.stamp = self.get_clock().now().to_msg()
        t_map_odom_stamped.header.frame_id = 'map'
        t_map_odom_stamped.child_frame_id = 'odom'
        t_map_odom_stamped.transform.translation.x = t_map_odom[0,3]
        t_map_odom_stamped.transform.translation.y = t_map_odom[1,3]
        t_map_odom_stamped.transform.translation.z = 0.
        q_map_odom = quaternion_from_matrix(t_map_odom)
        t_map_odom_stamped.transform.rotation.x = q_map_odom[0]
        t_map_odom_stamped.transform.rotation.y = q_map_odom[1]
        t_map_odom_stamped.transform.rotation.z = q_map_odom[2]
        t_map_odom_stamped.transform.rotation.w = q_map_odom[3]

        self.br.sendTransform(t_map_odom_stamped)


        pass



    def cmd_vel_callback(self,msg):
        self.cmd_vel_ = msg
        pass



def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalization()
    rclpy.spin(node)
    rclpy.shutdown()