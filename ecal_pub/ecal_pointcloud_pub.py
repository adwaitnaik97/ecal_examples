import sys
import numpy as np
import open3d as o3d
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher

import proto_messages.pointcloud_pb2 as pointcloud_pb2

def generate_random_points(num_points=100, scale=10.0):
    points = np.random.rand(num_points, 3) * scale
    return points

def pointclouds_from_datasets():
    pcd_dataset = o3d.data.PCDPointCloud()
    pcd_dataset_filepath = o3d.io.read_point_cloud(pcd_dataset.path)
    pcd_array = np.asarray(pcd_dataset_filepath.points)
    return pcd_array

if __name__ == "__main__":

    ##points = generate_random_points(1000, 1.0)

    pcd_array_dash = pointclouds_from_datasets()
    print(pcd_array_dash)

    ecal_core.initialize(sys.argv, "Pointcloud Publisher")

    pointcloud_pub = ProtoPublisher("pointcloud", pointcloud_pb2.PointCloud)

    while ecal_core.ok():
        pointcloud = pointcloud_pb2.PointCloud()
        pointxyzi = pointcloud.points.add()
        
        pointcloud.header = "Pointcloud Publisher"

        num_points = 10000
        scale = 1.0

        ##random_points = generate_random_points(num_points, scale)

        """
        for point in random_points:
            pointxyzi.x, pointxyzi.y, pointxyzi.z = point
            pointxyzi.i = 1.0
            pointcloud.points.append(pointxyzi)
        """
 
        for point in pcd_array_dash:
            pointxyzi.x, pointxyzi.y, pointxyzi.z = point
            pointxyzi.i = 1.0
            pointcloud.points.append(pointxyzi)
            ## only uncomment the line below if pointxyzi doesn't exists.
            ## pointcloud.points.extend(pointxyzi)

        pointcloud_pub.send(pointcloud)

    ecal_core.finalize()