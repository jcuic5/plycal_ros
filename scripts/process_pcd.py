import numpy as np
import glob, os
import open3d as o3d

if __name__=='__main__':
    data_dir = '../plycal/data/pointcloud_bin'
    save_dir = '../plycal/data/pointcloud'
    file_paths = glob.glob(os.path.join(data_dir, '*.bin'))
    for file_path in file_paths:
        # pcd = np.genfromtxt(file_path, delimiter=",", skip_header=1)
        pcd = np.fromfile(file_path, dtype=np.float32)
        pcd = pcd.reshape(-1, 6)
        pcd_total = pcd.copy()

        inds = (pcd[:, 0] < 5) * (pcd[:, 0] > 0) * \
                    (pcd[:, 1] < 2.5) * (pcd[:, 1] > -2.5) * \
                        (pcd[:, 2] < 2) * (pcd[:, 2] > -2)
        pcd = pcd[inds]

        pcd_temp = pcd_total[~inds]
        inds = (pcd_temp[:, 0] > 0)
                    # (pcd_temp[:, 1] < 2.5) * (pcd_temp[:, 1] > -2.5)
        pcd_temp = pcd_temp[inds]

        max_z = np.amax(pcd[:, 2])
        inds = pcd[:, 2] > max_z - 0.594
        pcd = pcd[inds]
        
        pcd_cat = np.vstack((pcd, pcd_temp))
        file_name = file_path.split('/')[-1]
        file_name = f"{file_name.split('.')[0]}.pcd"
        save_path = os.path.join(save_dir, file_name)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_cat[:, :3])
        o3d.io.write_point_cloud(save_path, pcd)
        # np.savetxt(save_path, pcd_cat, delimiter=",")