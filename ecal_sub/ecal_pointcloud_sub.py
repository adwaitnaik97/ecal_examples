import sys
import time

import numpy as np
import open3d as o3d

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

import proto_messages.leroy_pointlcoud_pb2 as leroy_pointcloud_pb2

# Define the timing decorator
import functools

def measure_execution_time(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"{func.__name__} executed in {execution_time:.4f} seconds")
        return result
    return wrapper

# Callback function for lane_pointcloud topic
@measure_execution_time
def lane_pointcloud_cb(topic_name, lane_point_cloud_message, time):    
    points = []

    for point in lane_point_cloud_message.points:
        points.append([point.x, point.y, point.z])
    
    if points:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.paint_uniform_color([0.706, 0.706, 0])
        o3d.visualization.draw_geometries([pcd])

# Callback function for lane_id_markers topic
@measure_execution_time
def lane_id_markers_cb(topic_name, color_message, time):
    marker_color = []
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

# Callback function for junctions_pointcloud topic
@measure_execution_time
def junctions_pointcloud_cb(topic_name, junctions_point_cloud_message, time):
    points = []

    for point in junctions_point_cloud_message.points:
        points.append([point.x, point.y, point.z])
    
    if points:
        junctions_pcd = o3d.geometry.PointCloud()
        junctions_pcd.points = o3d.utility.Vector3dVector(np.array(points))
        junctions_pcd.paint_uniform_color([0.305, 0.706, 0])
        o3d.visualization.draw_geometries([junctions_pcd])

# Callback function for junction_pointcloud topic
@measure_execution_time
def junction_pointcloud_cb(topic_name, junction_point_cloud_message, time):
    points = []

    for point in junction_point_cloud_message.points:
        points.append([point.point.x, point.point.y, point.point.z])
    
    if points:
        junction_pcd = o3d.geometry.PointCloud()
        junction_pcd.points = o3d.utility.Vector3dVector(np.array(points))
        junction_pcd.paint_uniform_color([0.305, 0.706, 0])
        o3d.visualization.draw_geometries([junction_pcd])

# Callback function for junction_id_markers topic
@measure_execution_time
def junction_id_markers_cb(topic_name, color_message, time):
    junction_marker_color = []
    junction_marker_pose = []
    junction_marker_dict = {}  # Initialize the dictionary to store marker_pose as keys and marker_text as values
    markersArray = color_message.markers

    for color in markersArray:
        junction_marker_color.append([(color.color.r)/255.0, (color.color.g)/255.0, (color.color.b)/255.0])

    for pose in markersArray:
        junction_marker_pose.append([pose.pose.x, pose.pose.y, pose.pose.z])

    if junction_marker_pose:
        junction_pcd_markers = o3d.geometry.PointCloud()
        junction_pcd_markers.points = o3d.utility.Vector3dVector(np.array(junction_marker_pose))
        junction_pcd_markers.paint_uniform_color([1, 0.706, 1])
        o3d.visualization.draw_geometries([junction_pcd_markers])

# Callback function for waypoint_pointcloud topic
@measure_execution_time
def waypoint_pointclouds_cb(topic_name, waypoint_point_cloud_message, time):
    points = []

    for point in waypoint_point_cloud_message.points:
        points.append([point.x, point.y, point.z])
    
    if points:
        waypoint_pcd = o3d.geometry.PointCloud()
        waypoint_pcd.points = o3d.utility.Vector3dVector(np.array(points))
        waypoint_pcd.paint_uniform_color([0.205, 0.205, 0])
        o3d.visualization.draw_geometries([waypoint_pcd])

# Main function      
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "Pointcloud Subscriber")
    lane_pointcloud_sub = ProtoSubscriber("lane_pointcloud", leroy_pointcloud_pb2.PointCloud)
    lane_id_markers_sub = ProtoSubscriber("lane_id_markers", leroy_pointcloud_pb2.MarkerArray)
    junctions_pointcloud_sub = ProtoSubscriber("junctions_pointcloud", leroy_pointcloud_pb2.PointCloud)
    junction_pointcloud_sub = ProtoSubscriber("junction_pointcloud", leroy_pointcloud_pb2.OntologyLanePointCloud)
    junction_id_markers_sub = ProtoSubscriber("junction_id_markers", leroy_pointcloud_pb2.MarkerArray)
    waypoint_pointclouds_sub = ProtoSubscriber("waypoint_pointcloud", leroy_pointcloud_pb2.PointCloud)

    lane_pointcloud_sub.set_callback(lane_pointcloud_cb)
    lane_id_markers_sub.set_callback(lane_id_markers_cb)
    junctions_pointcloud_sub.set_callback(junctions_pointcloud_cb)
    junction_pointcloud_sub.set_callback(junction_pointcloud_cb)
    junction_id_markers_sub.set_callback(junction_id_markers_cb)
    waypoint_pointclouds_sub.set_callback(waypoint_pointclouds_cb)
 
    try:
        while ecal_core.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    ecal_core.finalize()