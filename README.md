# BS3D: Building-scale 3D Reconstruction from RGB-D Images

Implementation of the reconstruction framework presented in: <br>
*BS3D: Building-scale 3D Reconstruction from RGB-D Images* [[arXiv](https://arxiv.org/pdf/2301.01057.pdf)] <br> <br>
The BS3D dataset is provided in [Section 2](#2-bs3d-dataset).

# 1. Reconstruction framework

Follow these instructions to reconstruct your environment. This repository includes a template dataset `datasets/mydataset` with necessary configuration files and folder structure.

## 1.1 Prerequisites

This software has been tested on **Windows 10**, but it should be compatible with Ubuntu 18.04.

- Install Azure Kinect SDK from [here](https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download) (version 1.4.1, latest)
- Install RTAB-Map from [here](https://github.com/introlab/rtabmap/releases) (version 0.20.16, latest)
- Install Preprocess-MKV (instructions below)

Clone the repository:

    git clone https://github.com/jannemus/BS3D.git
    cd BS3D

## 1.2 Install Preprocess-MKV
Preprocess-MKV is needed for extracting and processing the MKV files captured using Azure Kinect. Make sure you have installed the Azure Kinect SDK (see prerequisites). You also need [OpenCV 4.3.0](https://opencv.org/) (or later) and [CMake 3.18.2](https://cmake.org/download/) (or later).

In the following example, Visual Studio 2017 is used to compile Preprocess-MKV. Open the Visual Studio command prompt (Start -> VS2015 x64 Native Tools Command Prompt). To compile:

    mkdir preprocess\build
    cd preprocess\build
    cmake -G"Visual Studio 15 2017 Win64" ..
    cmake --build . --config Release --target install

## 1.3 Data capture
Azure Kinect SDK includes a recorder application (k4arecorder.exe) that is called from `record.py`. Record one or more sequences by running:

    python record.py output.mkv

Put your recordings (e.g. A1.mkv, A2.mkv, ...) to the *mydataset/mkv* folder.

**Capturing tips**
- To encourage loop closure detection, start and end the recording from a view that has plenty of visual features (corners etc.).
- During recording, it is good to revisit  locations, especially those that are rich in visual features.
- Although Azure Kinect has a fairly good depth range and FoV, avoid pointing the camera towards a view that has insufficient geometry (e.g. large and completely empty lobby or corridor).

## 1.4 Preprocess MKVs
Extract images (color, depth, infrared), inertial measurements, point clouds, and calibration information from the MKV files using `preprocess.py`. The code will also undistort the images and perform color-to-depth alignment (C2D). The command:

    python preprocess.py datasets/mydataset

will process all MKV files and write data to *mydataset/preprocessed/\*/*, where * is the name of the MKV file. RTAB-Map configuration files will also be written to *mydataset/rtabmap/*.

**Note** If you just want to extract data, you can provide arguments `--undistort false` and `--c2d false`.

## 1.5 Single-session mapping
Launch RTAB-Map and load configuration from *mydataset/rtabmap/\*/single-session-config.ini*, where * is the session name. <br>
`Preferences -> Load settings (*.ini)`
This will automatically set paths to calibration, color images and depth maps.

Initialize database `File -> New database` and press start. After the reconstruction has finished, check that the map looks good. If it does, close the database (.db) to save it to *mydataset/rtabmap/\*/map.db* If you have multiple sessions, process and save each of them. Make sure you name each database *map.db*.

If you only have a single session, export camera poses to *mydataset/rtabmap/poses.txt* <br>
`File -> Export poses -> RGBD-SLAM format (*.txt) -> Frame: Camera` <br>
After that, continue to Sec. 1.7 Surface reconstruction.

## 1.6 Multi-session mapping

In RTAB-Map, load configuration *mydataset/rtabmap/multi-session-config.ini* <br>

Select all single-session databases: <br>
`Preferences -> Source -> Database > [...] button`

**Note** that the order in which the databases are processed matters (A1.db, A2.db, ..., C3.db). For example, the sequence C3.db should overlap at least one of the earlier sequences (A1.db, A2.db, ...).

Initialize database `File -> New database` and press start. After the reconstruction has finished, check that the map looks good. If it does, export camera poses to *mydataset/rtabmap/poses.txt* <br>
`File -> Export poses -> RGBD-SLAM format (*.txt) -> Frame: Camera`

**Optionally** you can perform post-processing to detect more loop closures: <br>
`Tools -> Post-processing -> OK (default settings)` <br>
after which you need to export poses again.

## 1.7 Surface reconstruction
Perform surface reconstruction using TSDF fusion:

    python surface_reconstruction.py datasets/mydataset configs/meshing_config.yaml
    
The output mesh (.ply) will be written to *mydataset/reconstruction/mesh*.

## 1.8 Render images
Render depth maps and surface normals from the mesh:

    python render.py datasets/mydataset configs/render_config.yaml

The output data will be written to *mydataset/reconstruction*.


# 2. BS3D dataset

## 2.1 Data description
An overview of the data is shown in the table below. Download links and details are given in the following sections.

| Data                 | Resolution |   Format   | Description                                            |
| :------------------- | :--------- |:-----------|:-------------------------------------------------------|
| color/\*.jpg         | 720 x 1280 | 24-bit JPG | Color images (undistorted). Filename represents timestamp in seconds. |
| depth/\*.png         | 720 x 1280 | 16-bit PNG | Sensor depth in millimeters (invalid depth equals 0). Filename represents timestamp in seconds.|
| normal_render/\*.png | 720 x 1280 | 24-bit PNG | Surface normals rendered from mesh. Invalid normal equals (0,0,0). Filename represents timestamp in seconds. |
| depth_render/\*.png  | 720 x 1280 | 16-bit PNG | Depth rendered from mesh in millimeters (invalid depth equals 0). Filename represents timestamp in seconds. |
| infrared/\*.png      | 512 x 512  | 16-bit PNG | Infrared images. Note that some normalization is needed at least for visualization. |
| depth_raw/\*.png     | 512 x 512  | 16-bit PNG | Raw sensor depth in millimeters (invalid depth equals 0). Filename represents timestamp in seconds. |
| calibration/calib_color.yaml |    | YAML        | Color camera intrinsics and extrinsics between color and infrared camera.|
| calibration/calib_infrared.yaml | | YAML        | Infrared camera intrinsics and extrinsics between color and infrared camera.|
| poses/poses_color.txt |           | TXT        | Color camera poses (camera-to-world) in the RGBD SLAM format: timestamp, tx, ty, tz, qx, qy, qz, qw |
| poses/poses_infrared.txt |        | TXT        | Infrared camera poses (camera-to-world) in the RGBD SLAM format: timestamp, tx, ty, tz, qx, qy, qz, qw |
| imu/imu.csv          |            | CSV        | Accelerometer and gyroscope readings sampled at 1.6 kHz. Format: timestamp (s), w_xyz (rad/s) a_xyz (m/s^2)|
| imu/calibration.yaml |            | YAML       | IMU-camera extrinsics (e.g. between gyro and color camera). |
| mesh/mesh.ply        |            | PLY        | Triangle mesh created from raw depth maps (simplified). |
| laserscan/gt.ply     |            | PLY        | Ground truth point cloud obtained using a laser scanner. Note that this is not always available.

## 2.2 Raw recordings
If you need the original recordings (.mkv), those can be downloaded from [here](https://unioulu-my.sharepoint.com/:f:/g/personal/jannemus_univ_yo_oulu_fi/EkPCovSyBWlOmKkDU_fEN-oBG7jV-ESG2RlcIC4m9gqi3w). There are 47 recordings (6.7GB - 11.6 GB each) which you can extract using [process-mkv.exe](#-1.4-Process-MKVs). You may want to discard a few seconds at the beginning/end of the recording if device is stationary.

## 2.3 Complete reconstruction (2 Hz)
The reconstruction of the complete environment is available [here](https://unioulu-my.sharepoint.com/:f:/g/personal/jannemus_univ_yo_oulu_fi/EkPCovSyBWlOmKkDU_fEN-oBG7jV-ESG2RlcIC4m9gqi3w). Table above shows the data format. For this data, lasers scans were not captured.

<img src="images/overview.jpg" width="500">


## 2.4 Reconstruction (30 Hz)
The sequences used in the visual-inertial odometry experiments are available below. For this data, lasers scans were not captured. Click the sequence to download.

| Sequence  | Duration (s) | Length (m) | Dimensions (m)    | 
|:----------|:-------------|:-----------|:------------------|
| [cafeteria](https://unioulu) | 200          | 90.0       | 12.4 x 15.7 x 0.8 |
| [central](https://unioulu)   | 242          | 155.0      | 25.5 x 42.1 x 5.3 |
| [dining](https://unioulu)    | 192          | 109.2      | 33.8 x 25.0 x 5.5 |
| [corridor](https://unioulu)  | 174          | 77.6       | 31.1 x 4.7 x 2.4  |
| [foobar](https://unioulu)    | 75           | 37.1       | 5.4 x 14.4 x 0.6  |
| [hub](https://unioulu)       | 124          | 52.3       | 11.4 x 5.9 x 0.7  |
| [juice](https://unioulu)     | 103          | 42.7       | 6.3 x 8.6 x 0.5   |
| [lounge](https://unioulu)    | 222          | 94.2       | 14.4 x 10.3 x 1.1 | 
| [study](https://unioulu)     | 87           | 40.0       | 5.6 x 9.8 x 0.6   |
| [waiting](https://unioulu)   | 139          | 60.1       | 9.8 x 6.7 x 0.9   |


## 2.5 Reconstruction (30 Hz) with laser scans
We scanned a lobby and corridors using FARO 3D X 130 laser scanner as shown in the figure below. The FARO point cloud can be downloaded [here](https://unioulu-my.sharepoint.com), Azure Kinect MKVs [here](https://unioulu-my.sharepoint.com), and the reconstruction (2 Hz) [here](https://unioulu-my.sharepoint.com).

<img src="images/laserscan.jpg" width="400">

## Citation
If you use this repository in your research, please consider citing:

    @article{mustaniemi2023bs3d,
      title={BS3D: Building-scale 3D Reconstruction from RGB-D Images},
      author={Mustaniemi, Janne and Kannala, Juho and Rahtu, Esa and Liu, Li and Heikkil{\"a}, Janne},
      journal={arXiv preprint arXiv:2301.01057},
      year={2023}
    }
