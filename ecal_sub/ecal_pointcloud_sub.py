import sys
import time
import open3d as o3d
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

##import proto_messages.pointcloud_pb2 as pointcloud_pb2

import proto_messages.leroy_pointlcoud_pb2 as leroy_pointcloud_pb2

def rotate_view(vis):
    ctr = vis.get_view_control()
    ctr.rotate(10.0, 0.0)
    return False

def callback(topic_name, point_cloud_message, time):
    points = []
    print(len(points))
    for point in point_cloud_message.points:
        points.append([point.x, point.y, point.z])
    
    if points: 
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        o3d.visualization.draw_geometries([pcd])
                
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "Pointcloud Subscriber")
    pointcloud_sub = ProtoSubscriber("", leroy_pointcloud_pb2.PointCloud)
    pointcloud_sub.set_callback(callback)
 
    try:
        while ecal_core.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    ecal_core.finalize()