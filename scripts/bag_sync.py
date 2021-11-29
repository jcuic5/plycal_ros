#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, \
                            CompressedImage, \
                            PointCloud2, \
                            PointField
from std_msgs.msg import Header
import message_filters
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import os
import threading
import cv2
# import open3d as o3d


img_topic_raw = "/ecam_v4l2/image_raw"
pcd_topic_raw = "/velodyne_points_B"
img_topic_syn = "/ecam_v4l2/image_raw/syn"
pcd_topic_syn = "/velodyne_points_B/syn"

# calib_filename = "calibration_simrod_ecam_1280x720.txt"
calib_filename = "calibration.txt"
img_topic_debug = "/ecam_v4l2/image_debug"
pcd_topic_filtered = "/velodyne_points_B/filtered"

PAUSE = False
KEY_LOCK = threading.Lock()


def handle_keyboard():
    global KEY_LOCK, PAUSE
    key = raw_input('Press [ENTER] to pause and pick points\n')
    # key = input('Press [ENTER] to pause and pick points\n')
    with KEY_LOCK: PAUSE = True

def start_keyboard_handler():
    keyboard_t = threading.Thread(target=handle_keyboard)
    keyboard_t.daemon = True
    keyboard_t.start()

def extend_matrix(mat):
    mat = np.concatenate([mat, np.array([[0., 0., 0., 1.]])], axis=0)
    return mat

def read_calib_file(calib_path):
    """
    Read in a calibration file and parse into a dictionary.
    """
    calib_info = {}
    with open(calib_path, 'r') as f:
        lines = f.readlines()
    P0 = np.array([float(info) for info in lines[0].split(' ')[1:13]
                    ]).reshape([3, 4])
    P1 = np.array([float(info) for info in lines[1].split(' ')[1:13]
                    ]).reshape([3, 4])
    P2 = np.array([float(info) for info in lines[2].split(' ')[1:13]
                    ]).reshape([3, 4])
    P3 = np.array([float(info) for info in lines[3].split(' ')[1:13]
                    ]).reshape([3, 4])
    if extend_matrix:
        P0 = extend_matrix(P0)
        P1 = extend_matrix(P1)
        P2 = extend_matrix(P2)
        P3 = extend_matrix(P3)
    R0_rect = np.array([
        float(info) for info in lines[4].split(' ')[1:10]
    ]).reshape([3, 3])
    if extend_matrix:
        rect_4x4 = np.zeros([4, 4], dtype=R0_rect.dtype)
        rect_4x4[3, 3] = 1.
        rect_4x4[:3, :3] = R0_rect
    else:
        rect_4x4 = R0_rect

    Tr_velo_to_cam = np.array([
        float(info) for info in lines[5].split(' ')[1:13]
    ]).reshape([3, 4])
    Tr_imu_to_velo = np.array([
        float(info) for info in lines[6].split(' ')[1:13]
    ]).reshape([3, 4])
    if extend_matrix:
        Tr_velo_to_cam = extend_matrix(Tr_velo_to_cam)
        Tr_imu_to_velo = extend_matrix(Tr_imu_to_velo)
    calib_info['P0'] = P0
    calib_info['P1'] = P1
    calib_info['P2'] = P2
    calib_info['P3'] = P3
    calib_info['R0_rect'] = rect_4x4
    calib_info['Tr_velo_to_cam'] = Tr_velo_to_cam
    calib_info['Tr_imu_to_velo'] = Tr_imu_to_velo

    return calib_info

class Synchronizer:

    def __init__(self, calib_path=None, save_path=None):
        self.img_sub = rospy.Subscriber(img_topic_raw, Image, self.img_raw_callback, queue_size=10) 
        self.pcd_sub = rospy.Subscriber(pcd_topic_raw, PointCloud2, self.pcd_raw_callback, queue_size=10)
        self.img_pub = rospy.Publisher(img_topic_syn, Image, queue_size=10)
        self.pcd_pub = rospy.Publisher(pcd_topic_syn, PointCloud2, queue_size=10)

        self.img_debug_pub = rospy.Publisher(img_topic_debug, Image, queue_size=10)
        self.pcd_filtered_pub = rospy.Publisher(pcd_topic_filtered, PointCloud2, queue_size=10)

        if calib_path is not None:
            self.lidar2img = self.init_calib(calib_path)
            img_syn_sub = message_filters.Subscriber(img_topic_syn, Image)
            pcd_syn_sub = message_filters.Subscriber(pcd_topic_syn, PointCloud2)
            self.ts = message_filters.ApproximateTimeSynchronizer([img_syn_sub, pcd_syn_sub], 10, 0.1, False)
            self.ts.registerCallback(self.ts_callback)
        
        self.save_path = save_path
        self.save_count = 0

    def init_calib(self, calib_path):
        calib = read_calib_file(calib_path)
        self.calib = calib
        rect = calib['R0_rect'].astype(np.float32)
        Trv2c = calib['Tr_velo_to_cam'].astype(np.float32)
        P2 = calib['P2'].astype(np.float32)

        # # Transform the calib as we want (from x-left/y-back to x-front/y-right)
        # T_vp2cam = Trv2c
        # T_v2vp = t3d.axangles.axangle2aff([0, 0, 1], -math.pi/2)
        # T_v2cam = T_vp2cam @ T_v2vp # vp: velodyne^{\prime}
        # Trv2c = T_v2cam

        return np.dot(np.dot(P2, rect), Trv2c)

    def img_raw_callback(self, msg):
        # im = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        #                                 msg.height, msg.width, -1)
        # im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        # image_np = np.array(im)

        new_msg = msg
        new_msg.header.stamp = rospy.Time.now()
        # new_msg.encoding = 'mono8'
        # new_msg.step /= 3
        # new_msg.data = image_np.tobytes()
        # rospy.loginfo(f'Image synchronized timestamp: {new_msg.header.stamp}')
        self.img_pub.publish(new_msg)

    def pcd_raw_callback(self, msg):
        new_msg = msg
        new_msg.header.stamp = rospy.Time.now()
        # rospy.loginfo(f'Pointcloud synchronized timestamp: {new_msg.header.stamp}')
        self.pcd_pub.publish(new_msg)

    def lidar2img_projection(self, pcd, image_shape):
        # project points from velo coordinate to camera coordinate
        points = pcd[:, :3]
        num_points = points.shape[0]
        pts_4d = np.hstack((points, np.ones((points.shape[0], 1))))
        pts_2d = np.dot(pts_4d, self.lidar2img.T)

        pts_2d[:, 2] = np.clip(pts_2d[:, 2], a_min=1e-5, a_max=None)
        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]

        # print(image_shape)
        h, w = image_shape
        mask = (pts_2d[:, 0] > 0) * (pts_2d[:, 0] < w) * \
                (pts_2d[:, 1] > 0) * (pts_2d[:, 1] < h)

        return pcd[mask], pts_2d[mask], pcd[~mask]

    def ts_callback(self, img_msg, pcd_msg):
        # rospy.loginfo('Entering time synchronizer callback function...')
        #
        # Parse image and pointcloud
        #
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                                        img_msg.height, img_msg.width, -1)
        # image = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        image_np = np.array(im)
        #NOTE: in case of CompressedImage:
        # np_arr = np.fromstring(img_msg.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gen = []
        for p in pc2.read_points(
                pcd_msg,
                field_names=("x", "y", "z", "intensity", "ring", "time"),
                skip_nans=True):
            gen.append(np.array([p[0], p[1], p[2], p[3], p[4], p[5]])) #p[3]/100
        pcd = np.array(gen, dtype=np.float32)
        # rospy.loginfo('Pointcloud read shape: %s', pcd.shape)
        #
        # Draw pointcloud projection on image and publish it
        #
        inds = (pcd[:, 0] < 5) * (pcd[:, 0] > 0) * \
                    (pcd[:, 1] < 2.5) * (pcd[:, 1] > -2.5) * \
                        (pcd[:, 2] < 2) * (pcd[:, 2] > -2)
        #NOTE: filter the checkerboard out to more clearly see the quality
        pcd = pcd[inds]
        pcd, pts_2d, pcd_non_fov = \
            self.lidar2img_projection(pcd, image_np.shape[:2]) #(720, 1280)
        vis_size = 2
        grids_x = np.clip(pts_2d[:, 0].astype(np.int), a_min=1, a_max=image_np.shape[1]-2)
        grids_y = np.clip(pts_2d[:, 1].astype(np.int), a_min=1, a_max=image_np.shape[0]-2)
        for i in range(-vis_size, vis_size):
            for j in range(-vis_size, vis_size):
                gs_x, gs_y = grids_x + i, grids_y + j
                gs_x, gs_y = gs_x.tolist(), gs_y.tolist()
                image_np[gs_y, gs_x] = np.array([0, 255, 0])
        img_debug_msg = img_msg
        img_debug_msg.header.stamp = rospy.Time.now()
        img_debug_msg.data = image_np.tobytes()
        self.img_debug_pub.publish(img_debug_msg)
        #
        # [Optional] Also publish the filtered pointcloud
        #
        points = []
        for i in range(pcd.shape[0]):
            points.append(pcd[i].tolist())
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 16, PointField.UINT16, 1),
            PointField('time', 18, PointField.FLOAT32, 1),
        ]
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pcd_msg.header.frame_id
        pcd_msg_filtered = pc2.create_cloud(header, fields, points)
        self.pcd_filtered_pub.publish(pcd_msg_filtered)

        global PAUSE
        if self.save_path and PAUSE:
            save_path = os.path.join(self.save_path, 
                                     'image_orig', 
                                     str(self.save_count)+'.png') #.zfill(4)
            cv2.imwrite(save_path, image_np)
            save_path = os.path.join(self.save_path, 
                                     'pointcloud_bin', 
                                     str(self.save_count)+'.bin') #.zfill(4)
            pcd.tofile(save_path, "")
            # np.savetxt(save_path, pcd, delimiter=",")
            # o3d_pcd = o3d.geometry.PointCloud()
            # o3d_pcd.points = o3d.utility.Vector3dVector(pcd[:, :3])
            # o3d.io.write_point_cloud(save_path, o3d_pcd)
            rospy.loginfo('Image and pointcloud pair no.{} is saved'.format(self.save_count))
            self.save_count += 1

            # Resume listener
            with KEY_LOCK: PAUSE = False
            start_keyboard_handler()

if __name__=='__main__':
    rospy.init_node('bag_sync', anonymous=False)
    file_path = os.path.dirname(os.path.abspath(__file__))
    calib_path = '{}/{}'.format(file_path, calib_filename)
    save_path = '{}/../plycal/data/'.format(file_path)
    start_keyboard_handler()
    syner = Synchronizer(calib_path, save_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")