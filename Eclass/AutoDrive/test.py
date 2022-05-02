import numpy as np
import open3d as o3d
from scipy.io import loadmat

def main():
    dataset = "20220331T160645_ProjectData.mat"
    datas = loadmat(dataset)

    # Gen datas
    time_iter, = datas['Ibeo_X_stack'][0].shape
    xyz = []
    for t in range(time_iter):
        x = datas["Ibeo_X_stack"][0][t]
        y = datas["Ibeo_Y_stack"][0][t]
        z = datas["Ibeo_Z_stack"][0][t]
        x_y_z = np.vstack( (np.vstack((x, y)), z))
        x_y_z = x_y_z.T
        xyz.append(x_y_z)

    # Setting O3D
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # Init PCD for o3d
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # Push datas to PCD and Vis
    for t in range(time_iter):
        pcd.points = o3d.utility.Vector3dVector(xyz[t])
        mask = vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        print(mask)

    vis.destroy_window()

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

if __name__ == "__main__":
    main()