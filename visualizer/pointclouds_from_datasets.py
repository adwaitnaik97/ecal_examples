import numpy as np
import open3d as o3d

if __name__ == "__main__":

    print("Visualize data from PCD dataset.")
    pcd_dataset = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(pcd_dataset.path)
    ##print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

