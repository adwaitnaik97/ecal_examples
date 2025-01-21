import sys
import time
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import open3d as o3d
import numpy as np

##import proto_messages.pointcloud_pb2 as pointcloud_pb2

import proto_messages.leroy_pointlcoud_pb2 as leroy_pointcloud_pb2

#visualize = None

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
        #global visualize
        #visualize.clear_geometries()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        ##visualize.add_geometry(pcd)
        
        o3d.visualization.draw_geometries([pcd])
        
        #visualize.poll_events()
        #visualize.update_renderer()
        
if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "Pointcloud Subscriber")
    pointcloud_sub = ProtoSubscriber("", leroy_pointcloud_pb2.PointCloud)
    pointcloud_sub.set_callback(callback)

    #visualize = o3d.visualization.Visualizer()
    #visualize.create_window(window_name="Pointcloud Visualization")
 
    try:
        while ecal_core.ok():
            #visualize.poll_events()  # Process events for the visualization window
            #visualize.update_renderer()  # Update the renderer to reflect changes
            time.sleep(0.1)  # Reduce sleep time to make the visualization smoother
    except KeyboardInterrupt:
        pass
    ecal_core.finalize()