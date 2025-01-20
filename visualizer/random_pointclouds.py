import numpy as np
import open3d as o3d

def generate_random_points(num_points, scale=1.0):
    points = np.random.rand(num_points, 3)*scale
    return points

if __name__ == "__main__":

    num_points = 10000
    scale=10.0
    random_points = generate_random_points(num_points, scale)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(random_points)

    colors = np.random.rand(num_points, 3)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([point_cloud])
