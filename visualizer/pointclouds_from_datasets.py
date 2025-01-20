import jax.numpy as jnp
import open3d as o3d

if __name__ == "__main__":

    print("Visualize data from PCD dataset.")
    pcd_dataset = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(pcd_dataset.path)
    print(pcd)
    print(jnp.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Visualize data from PLY dataset.")
    ply_dataset = o3d.data.PLYPointCloud()
    ply = o3d.io.read_point_cloud(ply_dataset.path)
    print(ply)
    print(jnp.asarray(ply.points))
    o3d.visualization.draw_geometries([ply])
