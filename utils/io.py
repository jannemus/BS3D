import os
import sys
import yaml
import glob
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

# Read camera poses from text file.
# The format should be RTAB-Map's "RGBD-SLAM".
def readPosesTXT(fpath):
    
    if os.path.isfile(fpath) == False:
        print("Cannot find poses: '%s'"%fpath)
        sys.exit()

    data = np.loadtxt(fpath, dtype='float_', delimiter=' ')
    
    N = data.shape[0]
    eye = np.eye(4, dtype=np.float32)
    eye = np.expand_dims(eye, axis=0)
    poses = np.repeat(eye, N, axis=0)
    
    if data.shape[1] == 8:
    
        # Assuming RGBD-SLAM format
        timestamps = data[:,0]
        qts = data[:,4:]
        tvecs = data[:,1:4]
        
        for i, (qt, t) in enumerate(zip(qts,tvecs)):
            rot = Rotation.from_quat(qt)
            R = rot.as_matrix()
            poses[i,:3,:3] = R
            poses[i,:3,3] = t
            
        return poses, timestamps

    else:
        print("Check the format of the camera poses: '%s'" %fpath)
        sys.exit()
        

def writePosesTXT(poses, timestamps, fpath):

    print("Writing poses (.txt): %s" %fpath)
    
    if type(poses) is list:
    
        with open(fpath, 'w') as f:

            for i, pose in enumerate(poses):
        
                f.write("%.6f" %timestamps[i])
        
                rot = Rotation.from_matrix(pose[:3,:3])
                qt = rot.as_quat().ravel()
                tvec = pose[:3,3].ravel()
                vals = np.concatenate((tvec,qt))
                for val in vals:
                    f.write(" %.6f"%val)
                f.write("\n")
    
    else:
        N = poses.shape[0]

        with open(fpath, 'w') as f:

            for i in range(N):
        
                f.write("%.6f" %timestamps[i])
        
                rot = Rotation.from_matrix(poses[i,:3,:3])
                qt = rot.as_quat().ravel()
                tvec = poses[i,:3,3].ravel()
                vals = np.concatenate((tvec,qt))
                for val in vals:
                    f.write(" %.6f"%val)
                f.write("\n")

    
def writePosesLOG(poses, intrinsics, fpath):

    print("Writing poses (.log): %s" %fpath)
    
    num_poses = poses.shape[0]
    camera_params = []
    
    for i, pose in enumerate(poses):
        params = o3d.camera.PinholeCameraParameters()
        params.intrinsic = intrinsics
        params.extrinsic = pose
        camera_params.append(params)
    
    trajectory = o3d.camera.PinholeCameraTrajectory()
    trajectory.parameters = camera_params
    o3d.io.write_pinhole_camera_trajectory(fpath, trajectory)


def readCalibration(fpath):

    if os.path.isfile(fpath) == False:
        print("Cannot find calibration file: '%s'"%fpath)
        sys.exit()
    
    with open(fpath) as infile:
        for i in range(2):
            _ = infile.readline()
        dictionary = yaml.safe_load(infile)

    try:
        K = np.array(dictionary["camera_matrix"]["data"])
        K = K.reshape((3,3))
        width = dictionary["image_width"]
        height = dictionary["image_height"]
        depth2color = np.array(dictionary["depth_to_color"]["data"])
        depth2color = depth2color.reshape((4,4))
    except:
        print("Error while reading '%s'" %fpath)
        sys.exit()
        
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, K[0,0], K[1,1], K[0,2], K[1,2])

    return intrinsics, depth2color


def readTimestamps(fpath):

    if os.path.isfile(fpath) == False:
        print("Cannot find timestamps: '%s'"%fpath)
        sys.exit()

    timestamps = np.loadtxt(fpath, dtype='float_', delimiter=' ')
    
    if timestamps.shape[0] < 1:
        print("Check timestamps: %s" %fpath)
        sys.exit()

    return timestamps
    
    
def getPaths(stamps, data_dir, prefix):

    session_dirs = sorted(glob.glob(data_dir + "/*/"))
    paths_all = []
    
    for session_dir in session_dirs:
    
        files_dir = os.path.join(session_dir, prefix)
        paths_all.extend(glob.glob(files_dir + "/*"))
        
    stamps_all = [float(os.path.splitext(os.path.basename(path))[0]) for path in paths_all]
    stamps_all = np.array(stamps_all)
    
    # Timestamps of poses and images may not be exactly the same
    stamps_all = np.around(stamps_all, decimals=3)
    stamps = np.around(stamps, decimals=3)
    
    paths_out = [path for path, stamp in zip(paths_all, stamps_all) if stamp in stamps]
    
    return paths_out

def readIMU(data_dir):

    print("Reading IMU data...")
    session_dirs = sorted(glob.glob(data_dir + "/*/"))
    
    imu = []
    for session_dir in session_dirs:
        imu_path = os.path.join(session_dir, "imu", "imu.csv")
        imu_data = np.loadtxt(imu_path, delimiter=',')
        
        #print("Filtering out extra IMU samples...")
        #imu_data = np.array([row for row in imu_data if np.any(np.abs(stamps - row[0]) <= 10.0)])
        
        imu.append(imu_data)
        
    imu = np.concatenate(imu, axis=0)
    
    #print("Filtering out extra IMU samples...")
    #diffs = np.abs(imu[:, 0:1] - stamps)
    #keep_mask = np.any(diffs <= 10.0, axis=1)
    #imu = imu[keep_mask]
    #imu = np.array([row for row in imu if np.any(np.abs(stamps - row[0]) <= 10.0)])
    #print("OK!")
    
    imu_calib_path = os.path.join(session_dirs[0], "imu", "calibration.yaml")
    print("OK!")
    
    return imu, imu_calib_path