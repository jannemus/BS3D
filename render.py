import os
import sys
import glob
import cv2
import numpy as np
import open3d as o3d
from tqdm import tqdm
from shutil import copyfile
from argparse import ArgumentParser

import utils.io as io

def parse_arguments():

    if len(sys.argv) <= 1:
        print("Render depth and normal maps from mesh.")
        print("Usage: render.py <dataset_dir>\n"
              "\n"
              "Optional arguments:\n"
              "output_name : name of the output folder.\n"
              "start_stamp : rendering starts from 'start-stamp'.\n"
              "end_stamp   : rendering ends at 'end-stamp'.\n" 
              "frame_skip  : render every 'frame-skip' frame.\n"
              "max_depth   : maximum depth to render.\n")
        sys.exit(1)

    parser = ArgumentParser()
    
    # Positional argument for the dataset directory
    parser.add_argument("dataset_dir", type=str, help="Path to the dataset directory.")

    # Optional arguments
    parser.add_argument("--output_name", type=str, default="rendered", help="Name of the output folder.")
    parser.add_argument("--start_stamp", type=float, default=-1, help="First timestamp to render. If negative, start from the beginning.")
    parser.add_argument("--end_stamp", type=float, default=-1, help="Last timestamp to render. If negative, process to the end.")
    parser.add_argument("--frame_skip", type=int, default=1, help="Render every 'frame-skip' frame. If one, render every frame.")
    parser.add_argument("--max_depth", type=float, default=30.0, help="Maximum depth to render.")
    
    args = parser.parse_args()
    
    return args

if __name__ == '__main__':

    args = parse_arguments()

    dataset_dir = sys.argv[1]
    if not os.path.isdir(dataset_dir):
        print("Dataset not found!")
        print(dataset_dir)
        sys.exit()
        
    data_dir = os.path.join(dataset_dir, "preprocessed")
    session_dirs = glob.glob(data_dir + "/*/")
    if len(session_dirs) < 1:
        print("No sessions found!")
        sys.exit()

    mesh_path = os.path.join(dataset_dir, "mesh", "mesh.ply")
    if not os.path.exists(mesh_path):
        print("Mesh not found!")
        print(mesh_path)
        sys.exit()

    # Define path to poses and calibration    
    poses_path = os.path.join(dataset_dir, "rtabmap", "poses.txt")
    calib_depth_path = os.path.join(dataset_dir, "rtabmap", "calib_depth.yaml")
    calib_color_path = os.path.join(dataset_dir, "rtabmap", "calib_color.yaml")
    
    # Read poses, calibration, and IMU
    poses, timestamps = io.readPosesTXT(poses_path)
    intrinsics_depth, depth2color = io.readCalibration(calib_depth_path)
    intrinsics_color, depth2color = io.readCalibration(calib_color_path)
    imu, imu_calib_path = io.readIMU(data_dir)
    
    # Keep data only in given range
    t0 = args.start_stamp
    t1 = args.end_stamp
    t0 = np.min(timestamps) if t0 < 0 else t0
    t1 = np.max(timestamps) if t1 < 0 else t1
    mask = (timestamps >= t0) & (timestamps <= t1)
    
    timestamps = timestamps[mask]
    timestamps = timestamps[::args.frame_skip]
    poses = poses[mask]
    poses = poses[::args.frame_skip]
    
    mask = (imu[:,0] >= t0) & (imu[:,0] <= t1)
    imu = imu[mask,:]
    
    # Define paths to images
    color_paths = io.getPaths(timestamps, data_dir, prefix="color_cam/rgb")
    depth_paths = io.getPaths(timestamps, data_dir, prefix="color_cam/depth")
    infrared_paths = io.getPaths(timestamps, data_dir, prefix="depth_cam/infrared")
    cloud_paths = io.getPaths(timestamps, data_dir, prefix="depth_cam/clouds")
    depth_raw_paths = io.getPaths(timestamps, data_dir, prefix="depth_cam/depth")
    color_trans_paths = io.getPaths(timestamps, data_dir, prefix="depth_cam/rgb")

    output_dir = os.path.join(dataset_dir, args.output_name)
    #output_dir = "D:/BS3D/datasets/campus"
    
    # Prepare output folders
    color_ccam_dir = os.path.join(output_dir, "color_cam", "color")
    depth_ccam_dir = os.path.join(output_dir, "color_cam", "depth")
    depth_render_ccam_dir = os.path.join(output_dir, "color_cam", "depth_render")
    normal_render_ccam_dir = os.path.join(output_dir, "color_cam", "normal_render")
    depth_dcam_dir = os.path.join(output_dir, "depth_cam", "depth")
    color_dcam_dir = os.path.join(output_dir, "depth_cam", "color")
    depth_render_dcam_dir = os.path.join(output_dir, "depth_cam", "depth_render")
    normal_render_dcam_dir = os.path.join(output_dir, "depth_cam", "normal_render")
    infrared_dcam_dir = os.path.join(output_dir, "depth_cam", "infrared")
    clouds_dcam_dir = os.path.join(output_dir, "depth_cam", "clouds")
    imu_dir = os.path.join(output_dir, "imu")
    mesh_dir = os.path.join(output_dir, "mesh")
    
    output_dirs = [color_ccam_dir, color_dcam_dir, depth_ccam_dir, depth_dcam_dir, 
                   depth_render_ccam_dir, depth_render_dcam_dir,
                   normal_render_ccam_dir, normal_render_dcam_dir, 
                   infrared_dcam_dir, clouds_dcam_dir, imu_dir, mesh_dir]

    for outdir in output_dirs:
        if not os.path.exists(outdir):
            os.makedirs(outdir)
    
    # Read mesh and initialize rendering
    print("Loading PLY...")
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    
    scene = o3d.t.geometry.RaycastingScene()
    tri_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    mesh_id = scene.add_triangles(tri_mesh)
    
    # Render depths and normals for color camera
    rays_orig = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        intrinsics_color.intrinsic_matrix, np.eye(4), 
        intrinsics_color.width, intrinsics_color.height)
    V = (rays_orig[:,:,3:]).numpy()
    poses_color = []

    print("Rendering (raycast, color camera)...")
    
    for i, pose in tqdm(enumerate(poses)):

        pose_color = pose.dot(np.linalg.inv(depth2color))
        poses_color.append(pose_color)

        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            intrinsics_color.intrinsic_matrix, np.linalg.inv(pose_color), 
            intrinsics_color.width, intrinsics_color.height)
            
        result = scene.cast_rays(rays)
        D = result['t_hit'].numpy()
        D_inf = D > args.max_depth
        
        N = result['primitive_normals'].numpy()
        R = np.linalg.inv(pose_color[:3,:3])
        N_t = np.zeros_like(N)
        N_t[:,:,0] = R[0,0]*N[:,:,0] + R[0,1]*N[:,:,1] + R[0,2]*N[:,:,2]
        N_t[:,:,1] = R[1,0]*N[:,:,0] + R[1,1]*N[:,:,1] + R[1,2]*N[:,:,2]
        N_t[:,:,2] = R[2,0]*N[:,:,0] + R[2,1]*N[:,:,1] + R[2,2]*N[:,:,2]

        N_invalid = (V[:,:,0]*N_t[:,:,0] + V[:,:,1]*N_t[:,:,1] + V[:,:,2]*N_t[:,:,2]) > 0
        D[D_inf | N_invalid] = 0
        
        # Save rendered depth
        basename = os.path.basename(depth_paths[i])
        depth_uint16 = (D*1e3).astype(np.uint16)
        outfile = os.path.join(depth_render_ccam_dir, basename)
        cv2.imwrite(outfile, depth_uint16)
        
        # Save rendered normals
        N_t = ((N_t + 1.0) * 0.5)*255
        N_t[D_inf | N_invalid] = 0
        N_t_uint8 = N_t.astype(np.uint8)
        outfile = os.path.join(normal_render_ccam_dir, basename)
        cv2.imwrite(outfile, N_t_uint8)

        # Copy color image
        basename = os.path.basename(color_paths[i])
        outfile = os.path.join(color_ccam_dir, basename)
        copyfile(color_paths[i], outfile)
        
        # Copy depth map
        basename = os.path.basename(depth_paths[i])
        outfile = os.path.join(depth_ccam_dir, basename)
        copyfile(depth_paths[i], outfile)

    # Render depths and normals for depth (infrared) camera
    rays_orig = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        intrinsics_depth.intrinsic_matrix, np.eye(4), 
        intrinsics_depth.width, intrinsics_depth.height)
    V = (rays_orig[:,:,3:]).numpy()

    print("Rendering (raycast, infrared camera)...")
    
    for i, pose in tqdm(enumerate(poses)):

        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            intrinsics_depth.intrinsic_matrix, np.linalg.inv(pose), 
            intrinsics_depth.width, intrinsics_depth.height)
            
        result = scene.cast_rays(rays)
        D = result['t_hit'].numpy()
        D_inf = D > args.max_depth
        
        N = result['primitive_normals'].numpy()
        R = np.linalg.inv(pose[:3,:3])
        N_t = np.zeros_like(N)
        N_t[:,:,0] = R[0,0]*N[:,:,0] + R[0,1]*N[:,:,1] + R[0,2]*N[:,:,2]
        N_t[:,:,1] = R[1,0]*N[:,:,0] + R[1,1]*N[:,:,1] + R[1,2]*N[:,:,2]
        N_t[:,:,2] = R[2,0]*N[:,:,0] + R[2,1]*N[:,:,1] + R[2,2]*N[:,:,2]

        N_invalid = (V[:,:,0]*N_t[:,:,0] + V[:,:,1]*N_t[:,:,1] + V[:,:,2]*N_t[:,:,2]) > 0
        D[D_inf | N_invalid] = 0
        
        # Save rendered depth
        basename = os.path.basename(depth_raw_paths[i])
        depth_uint16 = (D*1e3).astype(np.uint16)
        outfile = os.path.join(depth_render_dcam_dir, basename)
        cv2.imwrite(outfile, depth_uint16)
        
        # Save rendered normals
        N_t = ((N_t + 1.0) * 0.5)*255
        N_t[D_inf | N_invalid] = 0
        N_t_uint8 = N_t.astype(np.uint8)
        outfile = os.path.join(normal_render_dcam_dir, basename)
        cv2.imwrite(outfile, N_t_uint8)

        # Copy infrared image
        basename = os.path.basename(infrared_paths[i])
        outfile = os.path.join(infrared_dcam_dir, basename)
        #ir = cv2.imread(infrared_paths[i], cv2.IMREAD_ANYDEPTH)
        #ir_norm = 0.004*ir.astype(np.float32)
        #ir_norm = np.clip(ir_norm,0,1)**0.6
        #cv2.imwrite(outfile, (255*ir_norm).astype(np.uint8))
        copyfile(infrared_paths[i], outfile)
        
        # Copy point cloud
        basename = os.path.basename(cloud_paths[i])
        outfile = os.path.join(clouds_dcam_dir, basename)
        copyfile(cloud_paths[i], outfile)
        
        # Copy raw depth map
        basename = os.path.basename(depth_raw_paths[i])
        outfile = os.path.join(depth_dcam_dir, basename)
        copyfile(depth_raw_paths[i], outfile)
        
        # Copy transformed color image
        basename = os.path.basename(color_trans_paths[i])
        outfile = os.path.join(color_dcam_dir, basename)
        copyfile(color_trans_paths[i], outfile)
    
    # Write poses
    outfile = os.path.join(output_dir, "depth_cam", "poses.txt")
    io.writePosesTXT(poses, timestamps, outfile)
    outfile = os.path.join(output_dir, "color_cam", "poses.txt")
    io.writePosesTXT(poses_color, timestamps, outfile)
    
    # Write IMU
    outfile = os.path.join(imu_dir, "imu.csv")
    np.savetxt(outfile, imu, fmt='%.9f', delimiter=',')
    copyfile(imu_calib_path, os.path.join(imu_dir, "calibration.yaml"))
    
    # Copy calibration
    outfile = os.path.join(output_dir, "color_cam", "calibration.yaml")
    copyfile(calib_color_path, outfile)
    outfile = os.path.join(output_dir, "depth_cam", "calibration.yaml")
    copyfile(calib_depth_path, outfile)

    # Crop and save mesh
    translations = poses[:, :3, 3]
    min_bound = np.min(translations, axis=0) - 10.0
    max_bound = np.max(translations, axis=0) + 10.0
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    mesh_cropped = mesh.crop(bbox)
    outfile = os.path.join(mesh_dir, "mesh.ply")
    o3d.io.write_triangle_mesh(outfile, mesh_cropped)