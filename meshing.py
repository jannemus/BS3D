import os
import sys
import glob
import numpy as np
import open3d as o3d
from tqdm import tqdm
from sklearn.cluster import KMeans
from argparse import ArgumentParser

import utils.io as io

def parse_arguments():

    if len(sys.argv) <= 1:
        print("Surface reconstruction using scalable TSDF fusion.")
        print("Usage: meshing.py <dataset_dir>\n"
              "\n"
              "Optional arguments:\n"
              "frame_skip  : render every 'frame-skip' frame.\n"
              "max_depth   : maximum depth to render.\n")
        sys.exit(1)

    parser = ArgumentParser()
    
    # Positional argument for the dataset directory
    parser.add_argument("dataset_dir", type=str, help="Path to the dataset directory.")

    # Optional arguments
    parser.add_argument("--frame_skip", type=int, default=1, help="Render every 'frame-skip' frame. If one, render every frame.")
    parser.add_argument("--frames_in_segment", type=int, default=1500, help="Approximate number of frames in each segment.")
    parser.add_argument("--voxel_length", type=float, default=1/128, help="Voxel length parameter for TSDF fusion.")
    parser.add_argument("--sdf_trunc", type=float, default=0.05, help="SDF truncation parameter for TSDF fusion.")
    parser.add_argument("--mesh_decimation_factor", type=float, default=0.5, help="Parameter for reducing the mesh size.")
    parser.add_argument("--max_depth", type=float, default=5.0, help="Maximum depth.")
    
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
        
    # Define path to poses and calibration    
    poses_path = os.path.join(dataset_dir, "rtabmap", "poses.txt")
    calib_depth_path = os.path.join(dataset_dir, "rtabmap", "calib_depth.yaml")
    calib_color_path = os.path.join(dataset_dir, "rtabmap", "calib_color.yaml")
    
    # Read poses and calibration
    poses, timestamps = io.readPosesTXT(poses_path)
    intrinsics_depth, depth2color = io.readCalibration(calib_depth_path)
    intrinsics_color, depth2color = io.readCalibration(calib_color_path)
    poses = poses[::args.frame_skip]
    timestamps = timestamps[::args.frame_skip]
    
    # Define paths to raw depth images
    depth_paths = io.getPaths(timestamps, data_dir, prefix="depth_cam/depth")

    # Prepare output folders
    mesh_dir = os.path.join(dataset_dir, "mesh")
    segments_dir = os.path.join(mesh_dir, "segments")
    if not os.path.exists(segments_dir):
        os.makedirs(segments_dir)
    
    ''' Create sparse point cloud if it does not exist ''' 
    
    sparse_cloud_path = os.path.join(mesh_dir, "sparse.npz")
    if not os.path.exists(sparse_cloud_path):
    
        stride = 128
        points = []
        view_ids = []
        
        print("Creating sparse cloud from %d depth maps..." %len(depth_paths))
        for i, (dpath, pose) in tqdm(enumerate(zip(depth_paths,poses))):
        
            depth = o3d.io.read_image(dpath)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(depth, intrinsics_depth, 
                extrinsic=np.linalg.inv(pose), depth_scale=1000.0, depth_trunc=1000.0, stride=stride, 
                project_valid_depth_only=True)
            
            pts = np.asarray(pcd.points)
            points.append(pts)
            
            ids = np.ones(pts.shape, dtype=np.int32) * i
            view_ids.append(ids)
        
        points = np.concatenate(points, axis=0).astype(np.float32)
        view_ids = np.concatenate(view_ids, axis=0)
        
        np.savez(sparse_cloud_path, points=points, view_ids=view_ids)
    else:
        print("Sparse cloud found.")
        
    # Cluster sparse point cloud
    with np.load(sparse_cloud_path) as data:
        pts = data['points']
        view_ids = data['view_ids']
        
    # Cluster points using K-means
    num_poses = len(poses)
    num_segments = int(np.ceil(num_poses/args.frames_in_segment))
    print("Clustering %d points -> %d fragments" %(pts.shape[0], num_segments))
    kmeans = KMeans(n_clusters=num_segments, random_state=0).fit(pts)
    segment_ids = kmeans.labels_
    
    # Surface reconstruction
    for idx in range(num_segments):
    
    
        pts_seg = pts[segment_ids==idx,:]
        view_ids_seg = view_ids[segment_ids==idx,:]
        view_ids_seg = np.unique(view_ids_seg)
        
        print("Creating TSDF mesh (%d/%d) ..." %(idx+1,num_segments))
        print("Number of views = %d" %(view_ids_seg.shape[0]))
        
        color_type = o3d.pipelines.integration.TSDFVolumeColorType.NoColor
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length = args.voxel_length,
            sdf_trunc = args.sdf_trunc,
            color_type=color_type)
        
        for view_idx in tqdm(view_ids_seg):
            
            pose = poses[view_idx]
            stamp = timestamps[view_idx]
            dpath = depth_paths[view_idx]
            
            depth = o3d.io.read_image(dpath)
            image = o3d.geometry.Image(np.ones_like(depth, dtype=np.uint8))
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    image, depth, depth_trunc=args.max_depth, convert_rgb_to_intensity=True)
                
            volume.integrate(rgbd, intrinsics_depth, np.linalg.inv(pose))
        
        print("Extracting triangle mesh...")
        mesh = volume.extract_triangle_mesh()
        mesh.paint_uniform_color(np.array([0.8,0.8,0.8]))
        
        # Crop
        pcd_seg = o3d.geometry.PointCloud()
        pcd_seg.points = o3d.utility.Vector3dVector(pts_seg)
        bbox = pcd_seg.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)
        
        mesh.vertex_normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        mesh.compute_vertex_normals()
        
        print("Writing...")
        mesh_path = os.path.join(segments_dir, "%02d.ply"%idx)
        o3d.io.write_triangle_mesh(mesh_path, mesh)

    
    ''' Simplify, clean, and merge segments '''
    
    mesh_paths = glob.glob(segments_dir + "/*.ply")
    
    decimation_factor = args.mesh_decimation_factor
    mesh_merged = o3d.geometry.TriangleMesh()

    for idx, mesh_path in enumerate(mesh_paths):
    
        print("Simplifying and merging (%d/%d) ..." %(idx+1,len(mesh_paths)))
    
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        num_triangles = np.asarray(mesh.triangles).shape[0]
        
        print("Vertex clustering...")
        mesh = mesh.simplify_vertex_clustering(voxel_size=0.02, contraction=o3d.geometry.SimplificationContraction.Average)
        
        if np.asarray(mesh.triangles).shape[0] > decimation_factor*num_triangles:
            print("Quadratic decimation...")
            # Without vertex clustering, this would be very slow
            mesh = mesh.simplify_quadric_decimation(int(decimation_factor*num_triangles))

        # Remove small isolated components
        triangle_clusters, cluster_n_triangles, cluster_area = (
        mesh.cluster_connected_triangles())
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        triangles_to_remove = cluster_n_triangles[triangle_clusters] < 50
        mesh.remove_triangles_by_mask(triangles_to_remove)
        
        mesh_merged += mesh

    mesh_path = os.path.join(mesh_dir, "mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh_merged)
