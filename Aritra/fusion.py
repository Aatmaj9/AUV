import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose

class LidarMasked(Node):
    def __init__(self):
        super().__init__('LidarMasked')
        self.get_logger().info('LidarMasked node started...')
        self.lidar_sub = self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_cam_uv, 10)
        self.uv_xyz_pub = self.create_publisher(PoseArray, '/uv_xyz', 10)
        
        self.min_d = 0.0
        self.max_d = 25
        self.min_z = -1.04
        self.max_z = 2

        self.f_x, self.f_y, self.c_x, self.c_y = [762.72232, 762.72231, 1280.0 / 2, 720.0 / 2]
        self.K_left = np.array([[self.f_x, 0.0, self.c_x],
                                [0.0, self.f_y, self.c_y],
                                [0.0, 0.0, 1]])

        self.image_shape = (720, 1280, 3)

    def lidar_cam_uv(self, pc_msg: PointCloud2):
        points = list(pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))
        point = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        self.lidar_stamp = pc_msg.header.stamp

        D = np.linalg.norm(point, axis=1)
        D_mask = (D > self.min_d) & (D < self.max_d)
        Z_mask = (point[:, 2] > self.min_z) & (point[:, 2] < self.max_z)
        point = point[D_mask & Z_mask]

        # Voxelization
        voxel_size = 0.1
        min_points = 0
        max_bounds, min_bounds = np.max(point, axis=0), np.min(point, axis=0)
        voxel_dim = np.floor((max_bounds - min_bounds+1) / voxel_size).astype(int)
        
        voxel_grid = np.zeros(voxel_dim, dtype=int)
        voxel_indices = np.ceil((point - min_bounds) / voxel_size).astype(int)
        unique_voxel_indices, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
        voxel_grid[voxel_indices[:, 0], voxel_indices[:, 1], voxel_indices[:, 2]] = 1
        cloud_numpy_vox = (voxel_indices - 0.5) * voxel_size + min_bounds
        cloud_numpy_unq , point_counts = np.unique(cloud_numpy_vox,axis=0,return_counts=True)
        point_mask = point_counts>min_points
        cloud_numpy_unq = cloud_numpy_unq[point_mask]

        # Clustering
        dbs_buoy = DBSCAN(eps = 0.5, min_samples = 3)
        cloud_lables = dbs_buoy.fit_predict(cloud_numpy_unq)
        unq_cloud_lables = np.unique(cloud_lables)
        point_labels = cloud_lables[inverse_indices]
        
        outliers_mask = point_labels == -1
        point_labels = point_labels[~outliers_mask]
        point = point[~outliers_mask]

        # Get singlepoint from each cluster
        selected_points = []
        for label in unq_cloud_lables:
            if label == -1:
                continue
            cluster_points = cloud_numpy_unq[cloud_lables == label]
            D_points = np.linalg.norm(cluster_points, axis=1)
            nearest_point_idx = np.argmin(D_points)
            selected_points.append(cluster_points[nearest_point_idx])

        point = np.array(selected_points)  # Shape (N, 3)

        lidar_camera = np.array([-0.05, -0.1, 0.3])
        just_rot = np.array([
            [0.966, 0.000, -0.259],
            [0.000, 1.000, 0.000],
            [0.259, 0.000, 0.966]
        ])

        trans = just_rot @ lidar_camera
        T_lidar_camera = np.array([
            [0.966, 0.000, -0.259, trans[0]],
            [0.000, 1.000, 0.000, trans[1]],
            [0.259, 0.000, 0.966, trans[2]],
            [0.000, 0.000, 0.000, 1.000]
        ])

        point_T = point.T
        ones_row = np.ones((1, point_T.shape[1]))
        point_hom = np.vstack((point_T, ones_row))
        cam_points = (T_lidar_camera @ point_hom).T

        trans_points = np.zeros((len(cam_points), 3))
        trans_points[:, 2] = cam_points[:, 0]
        trans_points[:, 0] = -cam_points[:, 1]
        trans_points[:, 1] = -cam_points[:, 2]

        valid_z = trans_points[:, 2] > 0
        trans_points = trans_points[valid_z]

        uv = (self.K_left @ trans_points.T).T
        uv[:, 0] /= uv[:, 2]
        uv[:, 1] /= uv[:, 2]
        UV = np.int32(uv[:, :2])
        #print(UV)

        self.Lidar_cam_filter(UV, trans_points)
        print(trans_points)
    def Lidar_cam_filter(self, UV, xyz_points):
        Us = UV[:, 0].astype(int)
        Vs = UV[:, 1].astype(int)

        mask = (
            (Us >= 0) & (Us < self.image_shape[1]) &
            (Vs >= 0) & (Vs < self.image_shape[0])
        )

        # Apply the mask correctly for xyz_points
        filtered_xyz_points = xyz_points[mask]  # Apply mask to the points

        # Log the filtered results (optional)
        print(filtered_xyz_points)

        msg = PoseArray()
        msg.header.stamp = self.lidar_stamp
        msg.header.frame_id = "xyz_link"  # Use appropriate frame

        # Loop through the valid UV points and corresponding xyz points
        for uv, pt in zip(zip(Us[mask], Vs[mask]), filtered_xyz_points):
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = pt[2]
            pose.orientation.x = float(uv[0])
            pose.orientation.y = float(uv[1])
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.uv_xyz_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = LidarMasked()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()