import sys
import time
import numpy as np
import open3d as o3d
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import proto_messages.leroy_pointlcoud_pb2 as leroy_pointcloud_pb2

def pointcloud_callback(topic_name, lane_point_cloud_message, time):    
    points = []

    for point in lane_point_cloud_message.points:
        points.append([point.x, point.y, point.z])
    
    if points:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.paint_uniform_color([0.706, 0.706, 0])
        o3d.visualization.draw_geometries([pcd])

def marker_callback(topic_name, color_message, time):
    marker_color = []
    #marker_text = ""
    marker_pose = []
    marker_dict = {}  # Initialize the dictionary to store marker_pose as keys and marker_text as values
    markersArray = color_message.markers

    for color in markersArray:
        marker_color.append([(color.color.r)/255.0, (color.color.g)/255.0, (color.color.b)/255.0])

    for pose in markersArray:
        marker_pose.append([pose.pose.x, pose.pose.y, pose.pose.z])

    if marker_pose:
        pcd_markers = o3d.geometry.PointCloud()
        pcd_markers.points = o3d.utility.Vector3dVector(np.array(marker_pose))
        pcd_markers.paint_uniform_color([1, 0.706, 1])
        o3d.visualization.draw_geometries([pcd_markers])
    
    """
    for text in markersArray:
        marker_text += str(text.text) + " "
        pcd_text = o3d.t.geometry.TriangleMesh.create_text(marker_text[0], 0.5, 0.1)
        pcd_text.compute_vertex_normals()
        pcd_text.paint_uniform_color([0, 0, 1])
        o3d.visualization.draw_geometries([pcd_text])
    """
      
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "Pointcloud Subscriber")
    lane_pointcloud_sub = ProtoSubscriber("lane_pointcloud", leroy_pointcloud_pb2.PointCloud)
    lane_id_markers_sub = ProtoSubscriber("lane_id_markers", leroy_pointcloud_pb2.MarkerArray)
    lane_pointcloud_sub.set_callback(pointcloud_callback)
    lane_id_markers_sub.set_callback(marker_callback)
 
    try:
        while ecal_core.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    ecal_core.finalize()