import numpy as np
import random
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf_transformations as tf


class FakePerceptionNode(Node):
    def __init__(self,node_name='fake_perception',namespace=''):
        super().__init__(node_name=node_name,namespace=namespace)
        
        self.x_pose = 0.0  # Current x position of the robot
        self.y_pose = 0.0  # Current y position of the robot
        self.yaw_pose = 0.0  # Yaw angle of the robot
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize an empty numpy array for the point cloud, with shape (n_points, 3)
        # Initially empty (0 points), but later will be populated with x, y, z coordinates
        self.pcd = np.empty((0, 3))
        
        # Publisher for point cloud
        self.pcd_publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        # Timer for periodically calling the perception_callback
        self.timer = self.create_timer(0.1, self.perception_callback)

    def perception_callback(self):

        # Lookup the transformation between 'map' and 'base_link' frames
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return

        translation = [transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z]
        rotation = [transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w]

        transformation_matrix = tf.concatenate_matrices(
            tf.translation_matrix(translation),
            tf.quaternion_matrix(rotation)
        )
        self.x_pose = translation[0]
        self.y_pose = translation[1]
        self.yaw_pose = tf.euler_from_quaternion(rotation)[2]
            
        # Calculate point distances and filter points within a radius of 5.0 units
        distance_sqr = (self.pcd[:, 0] - self.x_pose) ** 2 + (self.pcd[:, 1] - self.y_pose) ** 2
        filtered_points = self.pcd[distance_sqr < 5.**2]
        
        # If fewer than 20 points, add random points
        added_points = np.empty((0, 3))
        while len(filtered_points) < 10:
            random_x = random.uniform(-5, 5)
            random_y = random.uniform(-5, 5)
            if random_x**2 + random_y**2 > 5**2:
                continue
            new_point = np.array([[random_x,
                                   random_y,
                                   0.0]])  # Assuming Z is 0
            filtered_points = np.vstack([filtered_points, new_point])
            # self.pcd = np.vstack([self.pcd, new_point])  # Also add to the point cloud
            added_points = np.vstack([added_points, new_point])
            # print(f'pose: {self.x_pose}, {self.y_pose}, {self.yaw_pose}')

        points_in_map = np.empty((0, 3))
        for point in added_points:
            # 将点扩展为齐次坐标 (x, y, z, 1)
            point_homogeneous = np.append(point, 1.0)
            # 进行坐标变换
            transformed_point = np.dot(transformation_matrix, point_homogeneous)
            # 只保留 (x, y, z) 部分
            # points_in_map.append(transformed_point[:3])
            points_in_map = np.vstack([points_in_map, transformed_point[:3]])
        
        self.pcd = np.vstack([self.pcd,points_in_map])
        print(len(self.pcd))
        map_to_base_link_matrix = np.linalg.inv(transformation_matrix)

        points_in_base = []
        for point in self.pcd:
            # 将点扩展为齐次坐标 (x, y, z, 1)
            point_homogeneous = np.append(point, 1.0)
            # 进行坐标变换
            transformed_point = np.dot(map_to_base_link_matrix, point_homogeneous)
            # 只保留 (x, y, z) 部分
            points_in_base.append(transformed_point[:3])

            
            # # transform self.pcd to base_link frame using numpy
            # transformed_point = np.zeros_like(self.pcd)
            # transformed_point[:, 0] = self.pcd[:, 0] - self.x_pose
            # transformed_point[:, 1] = self.pcd[:, 1] - self.y_pose
            # transformed_point[:, 2] = self.pcd[:, 2]
            # # rotate the point cloud
            # cos_yaw = np.cos(self.yaw_pose)
            # sin_yaw = np.sin(self.yaw_pose)
            # transformed_point2 = np.zeros_like(transformed_point)
            # transformed_point2[:, 0] = transformed_point[:, 0] * cos_yaw - transformed_point[:, 1] * sin_yaw
            # transformed_point2[:, 1] = transformed_point[:, 0] * sin_yaw + transformed_point[:, 1] * cos_yaw

        # Convert the filtered points to a PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points_in_base)
            
        # Publish the point cloud
        self.pcd_publisher.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()