#include "kinect.hpp"
#include "util.h"
#include "happly.h"

#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

#include <sys/stat.h>

// Constructor
kinect::kinect(int argc, char* argv[])
    : is_quit(true),
      is_color(false),
      is_depth(false),
      is_infrared(false),
	  is_imu(false),
	  is_undistort(false)
{
    // Initialize
    initialize(argc, argv);
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize(int argc, char* argv[])
{
    // Initialize Parameter
    initialize_parameter(argc, argv);

    // Initialize Playback
    initialize_playback();

	// Initialize Camera Calibration
	initialize_calibration();

    // Initialize Save
    initialize_save();

	// Write Depth Camera Calibration File (.yaml)
	std::string fname = "calibration_" + std::to_string(size_depth.height) + "x" + std::to_string(size_depth.width) + ".yaml";
	std::string file_path = output_dir.generic_string() + "/depth_cam/" + fname;
	write_calibration(K_depth, D_depth, R_depth, P_depth, size_depth.width, size_depth.height,
		file_path, fname);

	// Write Color Camera Calibration File (.yaml)
	fname = "calibration_" + std::to_string(size_color.height) + "x" + std::to_string(size_color.width) + ".yaml";
	file_path = output_dir.generic_string() + "/color_cam/" + fname;
	write_calibration(K_color, D_color, R_color, P_color, size_color.width, size_color.height,
		file_path, fname);

	// Write Depth Camera (2x resolution) Calibration File (.yaml) 
	fname = "calibration_" + std::to_string(2 * size_depth.height) + "x" + std::to_string(2 * size_depth.width) + ".yaml";
	file_path = output_dir.generic_string() + "/depth_cam/" + fname;
	cv::Mat K_depth2 = 2 * K_depth.clone();
	K_depth2.at<double>(2, 2) = 1.0f;
	cv::Mat P_depth2 = 2 * P_depth.clone();
	P_depth2.at<double>(2, 2) = 1.0f;
	write_calibration(K_depth2, D_depth, R_depth, P_depth2, 2 * size_depth.width, 2 * size_depth.height,
		file_path, fname);

	// Write IMU Data (.csv)
	write_imu();
}

// Initialize Parameter
void kinect::initialize_parameter(int argc, char* argv[])
{
    // Create Command Line Parser
	const std::string keys =
        "{input         |      | path to input mkv file (required) }"
		"{output        |      | path to output folder (required) }"
		"{sequence_idx  |      | sequence index used to offset timestamps (optional) }"
        "{undistort     | true | correct radial distortion (optional) }"
		"{c2d           | true | transform color-to-depth (optional) }"
        "{export_clouds | true | export point clouds (optional) }"
        "{help          |      | show help message }";

    cv::CommandLineParser parser(argc, argv, keys);

    if(parser.has("help")){
        parser.printMessage();
        std::exit(EXIT_SUCCESS);
    }

    // Check Parsing Error
    if(!parser.check()){
		parser.printMessage();
        parser.printErrors();
        throw std::runtime_error("Invalid commandline arguments!");
    }

    // Get MKV File Path (Required)
    if(!parser.has("input")){
		parser.printMessage();
        throw std::runtime_error("Input MKV path not provided!");
    }
    else{
        mkv_file = parser.get<cv::String>("input").c_str();
        if(!filesystem::is_regular_file(mkv_file) || mkv_file.extension() != ".mkv"){
            throw std::runtime_error("Input mkv file not found!");
        }
    }
	
	// Output directory (Required)
    if(!parser.has("output")){
		parser.printMessage();
        throw std::runtime_error("Output directory not provided!");
    }
    else{
        output_dir = parser.get<cv::String>("output").c_str();
    }

	// Sequence index (Option)
	if (!parser.has("sequence_idx")) {
		stamp_offset = 0.0;
	}
	else {
		int sequence = parser.get<int>("sequence_idx");
		stamp_offset = (double)(sequence*1e6);
	}

	// Get Undistort Flag (Option)
	if (!parser.has("undistort")) {
		is_undistort = true;
	}
	else {
		is_undistort = parser.get<bool>("undistort");
	}
	
	// Get c2d Flag (Option)
	if (!parser.has("c2d")) {
		is_c2d = true;
	}
	else {
		is_c2d = parser.get<bool>("c2d");
	}

	// Export point clouds (Option)
	if (!parser.has("export_clouds")) {
		is_export_clouds = true;
	}
	else {
		is_export_clouds = parser.get<bool>("export_clouds");
	}


}

// Initialize Playback
inline void kinect::initialize_playback()
{
    if(!filesystem::is_regular_file(mkv_file) || !filesystem::exists(mkv_file)){
        throw k4a::error("Failed to found file path!");
    }

    // Open Playback
    playback = k4a::playback::open(mkv_file.generic_string().c_str());

    // Get Record Configuration
    record_configuration = playback.get_record_configuration();

    // Get Calibration
    calibration = playback.get_calibration();

    // Create Transformation
    transformation = k4a::transformation(calibration);
}

// Initialize Camera Calibration
inline void kinect::initialize_calibration()
{
	// Color Camera Calibration
	size_color = cv::Size(calibration.color_camera_calibration.resolution_width, calibration.color_camera_calibration.resolution_height);
	K_color = cv::Mat::eye(3, 3, CV_64FC1);
	K_color.at<double>(0, 0) = calibration.color_camera_calibration.intrinsics.parameters.param.fx;
	K_color.at<double>(1, 1) = calibration.color_camera_calibration.intrinsics.parameters.param.fy;
	K_color.at<double>(0, 2) = calibration.color_camera_calibration.intrinsics.parameters.param.cx;
	K_color.at<double>(1, 2) = calibration.color_camera_calibration.intrinsics.parameters.param.cy;
	D_color = cv::Mat::zeros(1, 8, CV_64FC1);
	D_color.at<double>(0, 0) = calibration.color_camera_calibration.intrinsics.parameters.param.k1;
	D_color.at<double>(0, 1) = calibration.color_camera_calibration.intrinsics.parameters.param.k2;
	D_color.at<double>(0, 2) = calibration.color_camera_calibration.intrinsics.parameters.param.p1;
	D_color.at<double>(0, 3) = calibration.color_camera_calibration.intrinsics.parameters.param.p2;
	D_color.at<double>(0, 4) = calibration.color_camera_calibration.intrinsics.parameters.param.k3;
	D_color.at<double>(0, 5) = calibration.color_camera_calibration.intrinsics.parameters.param.k4;
	D_color.at<double>(0, 6) = calibration.color_camera_calibration.intrinsics.parameters.param.k5;
	D_color.at<double>(0, 7) = calibration.color_camera_calibration.intrinsics.parameters.param.k6;
	R_color = cv::Mat::eye(3, 3, CV_64FC1);
	P_color = cv::Mat::eye(3, 4, CV_64FC1);

	dist_coeffs_color = D_color.clone();
	intrinsics_color = K_color.clone();

	cv::Rect ROI_color;
	cv::Mat K_color_new = cv::getOptimalNewCameraMatrix(K_color, D_color, size_color, 0, size_color, &ROI_color, true);
	cv::initUndistortRectifyMap(K_color, D_color, R_color, K_color_new, size_color, CV_32FC1, rect_color.first, rect_color.second);

	if (is_undistort) {
		K_color = K_color_new;
		D_color = cv::Mat::zeros(1, 8, CV_64FC1);
	}

	P_color.at<double>(0, 0) = K_color.at<double>(0, 0);
	P_color.at<double>(1, 1) = K_color.at<double>(1, 1);
	P_color.at<double>(0, 2) = K_color.at<double>(0, 2);
	P_color.at<double>(1, 2) = K_color.at<double>(1, 2);

	// Depth Camera Calibration
	size_depth = cv::Size(calibration.depth_camera_calibration.resolution_width, calibration.depth_camera_calibration.resolution_height);
	K_depth = cv::Mat::eye(3, 3, CV_64FC1);
	K_depth.at<double>(0, 0) = calibration.depth_camera_calibration.intrinsics.parameters.param.fx;
	K_depth.at<double>(1, 1) = calibration.depth_camera_calibration.intrinsics.parameters.param.fy;
	K_depth.at<double>(0, 2) = calibration.depth_camera_calibration.intrinsics.parameters.param.cx;
	K_depth.at<double>(1, 2) = calibration.depth_camera_calibration.intrinsics.parameters.param.cy;
	D_depth = cv::Mat::zeros(1, 8, CV_64FC1);
	D_depth.at<double>(0, 0) = calibration.depth_camera_calibration.intrinsics.parameters.param.k1;
	D_depth.at<double>(0, 1) = calibration.depth_camera_calibration.intrinsics.parameters.param.k2;
	D_depth.at<double>(0, 2) = calibration.depth_camera_calibration.intrinsics.parameters.param.p1;
	D_depth.at<double>(0, 3) = calibration.depth_camera_calibration.intrinsics.parameters.param.p2;
	D_depth.at<double>(0, 4) = calibration.depth_camera_calibration.intrinsics.parameters.param.k3;
	D_depth.at<double>(0, 5) = calibration.depth_camera_calibration.intrinsics.parameters.param.k4;
	D_depth.at<double>(0, 6) = calibration.depth_camera_calibration.intrinsics.parameters.param.k5;
	D_depth.at<double>(0, 7) = calibration.depth_camera_calibration.intrinsics.parameters.param.k6;
	R_depth = cv::Mat::eye(3, 3, CV_64FC1);
	P_depth = cv::Mat::eye(3, 4, CV_64FC1);

	cv::Rect ROI_depth;
	K_depth_rect = cv::getOptimalNewCameraMatrix(K_depth, D_depth, size_depth, 0, size_depth, &ROI_depth, true);
	cv::initUndistortRectifyMap(K_depth, D_depth, R_depth, K_depth_rect, size_depth, CV_32FC1, rect_depth.first, rect_depth.second);

	if (is_undistort) {
		K_depth = K_depth_rect;
		D_depth = cv::Mat::zeros(1, 8, CV_64FC1);
	}
	P_depth.at<double>(0, 0) = K_depth.at<double>(0, 0);
	P_depth.at<double>(1, 1) = K_depth.at<double>(1, 1);
	P_depth.at<double>(0, 2) = K_depth.at<double>(0, 2);
	P_depth.at<double>(1, 2) = K_depth.at<double>(1, 2);

	// Extrinsics
	k4a_calibration_extrinsics_t* depth_to_color_ext;
	depth_to_color_ext = &calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];

	depth_to_color_trans = cv::Mat::eye(4, 4, CV_64FC1);
	depth_to_color_trans.at<double>(0, 0) = depth_to_color_ext->rotation[0];
	depth_to_color_trans.at<double>(0, 1) = depth_to_color_ext->rotation[1];
	depth_to_color_trans.at<double>(0, 2) = depth_to_color_ext->rotation[2];
	depth_to_color_trans.at<double>(1, 0) = depth_to_color_ext->rotation[3];
	depth_to_color_trans.at<double>(1, 1) = depth_to_color_ext->rotation[4];
	depth_to_color_trans.at<double>(1, 2) = depth_to_color_ext->rotation[5];
	depth_to_color_trans.at<double>(2, 0) = depth_to_color_ext->rotation[6];
	depth_to_color_trans.at<double>(2, 1) = depth_to_color_ext->rotation[7];
	depth_to_color_trans.at<double>(2, 2) = depth_to_color_ext->rotation[8];
	depth_to_color_trans.at<double>(0, 3) = depth_to_color_ext->translation[0] / 1000.0f;
	depth_to_color_trans.at<double>(1, 3) = depth_to_color_ext->translation[1] / 1000.0f;
	depth_to_color_trans.at<double>(2, 3) = depth_to_color_ext->translation[2] / 1000.0f;

}

// Initialize Save
void kinect::initialize_save()
{
    // Create root directory
	filesystem::remove_all(output_dir);
    if(!filesystem::create_directories(output_dir)){
        throw std::runtime_error("Failed to create output directory!");
    }

	if (!record_configuration.color_track_enabled) {
		throw std::runtime_error("Color track is not enabled");
	}
	if (!record_configuration.depth_track_enabled) {
		throw std::runtime_error("Depth track is not enabled");
	}
	if (!record_configuration.ir_track_enabled) {
		throw std::runtime_error("IR track is not enabled");
	}
	if (!record_configuration.imu_track_enabled) {
		throw std::runtime_error("IMU track is not enabled");
	}

	if (record_configuration.depth_mode != k4a_depth_mode_t::K4A_DEPTH_MODE_WFOV_2X2BINNED) {
		std::cout << "WARNING! Depth mode is not WFOV_2X2BINNED" << std::endl;
	}
	
	if (record_configuration.color_resolution != k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P) {
		std::cout << "WARNING! Color resolution is not 720P" << std::endl;
	}
	
    // Create Sub Directories
    std::vector<std::string> names;
	names.push_back("color_cam");
	names.push_back("depth_cam");

	path_color = "color_cam/rgb";
	path_depth = "depth_cam/depth";
	path_ir = "depth_cam/infrared";
	path_color_trans = "depth_cam/rgb";
	path_depth_trans = "color_cam/depth";
	path_clouds = "depth_cam/clouds";

	names.push_back(path_color);
	names.push_back(path_depth);
	names.push_back(path_ir);
	names.push_back(path_color_trans);
	names.push_back(path_depth_trans);
	names.push_back(path_clouds);

	if (record_configuration.imu_track_enabled) {
		path_imu = "imu/";
		names.push_back(path_imu);
		is_imu = true;
	}

    for(const std::string& name : names){
        filesystem::path sub_directory = output_dir.generic_string() + "/" + name;
        if(!filesystem::create_directories(sub_directory)){
            throw std::runtime_error("Failed to create sub directory (" + name + ")");
        }
    }

	// Timestamps
	std::string timestamps_color_path = output_dir.generic_string() + "/color_cam/timestamps.txt";
	timestamps_color_file.open(timestamps_color_path.c_str());
	timestamps_color_file.precision(6);
	timestamps_color_file.setf(std::ios::fixed, std::ios::floatfield);

	std::string timestamps_depth_path = output_dir.generic_string() + "/depth_cam/timestamps.txt";
	timestamps_depth_file.open(timestamps_depth_path.c_str());
	timestamps_depth_file.precision(6);
	timestamps_depth_file.setf(std::ios::fixed, std::ios::floatfield);

    // Start Threads
    color_thread = std::thread(&kinect::export_color, this);
    depth_thread = std::thread(&kinect::export_depth, this);
    infrared_thread = std::thread(&kinect::export_infrared, this);
	color_trans_thread = std::thread(&kinect::export_color_trans, this);
	depth_trans_thread = std::thread(&kinect::export_depth_trans, this);
	clouds_thread = std::thread(&kinect::export_cloud, this);
}

// Finalize
void kinect::finalize()
{
    // Destroy Transformation
    transformation.destroy();

    // Close Playback
    playback.close();

    // Join Threads
    is_quit = true;
    if(color_thread.joinable()){
        color_thread.join();
    }
	if (color_trans_thread.joinable()) {
		color_trans_thread.join();
	}
    if(depth_thread.joinable()){
        depth_thread.join();
    }
	if (depth_trans_thread.joinable()) {
		depth_trans_thread.join();
	}
    if(infrared_thread.joinable()){
        infrared_thread.join();
    }
	if(clouds_thread.joinable()){
        clouds_thread.join();
    }

	// Close Timestamp Files
	timestamps_color_file.close();
	timestamps_depth_file.close();

    // Close Window
    cv::destroyAllWindows();
}

// Export color image
void kinect::export_color()
{
    assert(record_configuration.color_format == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG);

	std::chrono::microseconds lenght = playback.get_recording_length();
	double length_s = (1e-6)*lenght.count();

    is_quit = false;
    uint64_t index = 0;

    while(!(is_quit && color_queue.empty())){
        // Pop Queue
		std::pair<cv::Mat, int64_t> data;
        bool result = color_queue.try_pop(data);
        if(!result){
            std::this_thread::yield();
            continue;
        }
		double timestamp = 1e-6*(data.second);

		std::string fname = cv::format("%s/%s/%.6f.jpg", output_dir.generic_string().c_str(), path_color.c_str(), timestamp + stamp_offset);
		cv::imwrite(fname, data.first);

		// Write Timestamp
		timestamps_color_file << timestamp + stamp_offset << std::endl;

		index++;
		if (index % 50 == 0) {
			std::cout << "Progress: " << timestamp << " s / " << length_s << " s \r";
			std::cout.flush();
		}
    }
}


// Export transformed color image
void kinect::export_color_trans()
{
	assert(record_configuration.color_format == k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_MJPG);

	is_quit = false;

	while (!(is_quit && color_trans_queue.empty())) {
		// Pop Queue
		std::pair<cv::Mat, int64_t> data;
		bool result = color_trans_queue.try_pop(data);
		if (!result) {
			std::this_thread::yield();
			continue;
		}
		double timestamp = 1e-6*(data.second);

		std::string fname = cv::format("%s/%s/%.6f.jpg", output_dir.generic_string().c_str(), path_color_trans.c_str(), timestamp + stamp_offset);
		cv::imwrite(fname, data.first);
	}
}


// Export depth map
void kinect::export_depth()
{
    is_quit = false;

    while(!(is_quit && depth_queue.empty())){
        // Pop Queue
		std::pair<cv::Mat, int64_t> data;
        bool result = depth_queue.try_pop(data);
        if(!result){
            std::this_thread::yield();
            continue;
        }
		double timestamp = 1e-6*(data.second);

		cv::Mat depth = data.first;
		std::string fname = cv::format("%s/%s/%.6f.png", output_dir.generic_string().c_str(), path_depth.c_str(), timestamp + stamp_offset);
		cv::imwrite(fname, depth);

		// Write Timestamp
		timestamps_depth_file << timestamp + stamp_offset << std::endl;
    }
}

// Export transformed depth map
void kinect::export_depth_trans()
{
	is_quit = false;

	while (!(is_quit && depth_trans_queue.empty())) {
		// Pop Queue
		std::pair<cv::Mat, int64_t> data;
		bool result = depth_trans_queue.try_pop(data);
		if (!result) {
			std::this_thread::yield();
			continue;
		}
		double timestamp = 1e-6*(data.second);

		cv::Mat depth = data.first;

		std::string fname = cv::format("%s/%s/%.6f.png", output_dir.generic_string().c_str(), path_depth_trans.c_str(), timestamp + stamp_offset);
		cv::imwrite(fname, depth);
	}
}

// Export infrared image
void kinect::export_infrared()
{
    is_quit = false;

    while(!(is_quit && infrared_queue.empty())){
        // Pop Queue
		std::pair<cv::Mat, int64_t> data;
        bool result = infrared_queue.try_pop(data);
        if(!result){
            std::this_thread::yield();
            continue;
        }
		double timestamp = 1e-6*(data.second);

		cv::Mat infrared = data.first;

		std::string fname = cv::format("%s/%s/%.6f.png", output_dir.generic_string().c_str(), path_ir.c_str(), timestamp + stamp_offset);
		cv::imwrite(fname, infrared);
    }
}

// Export point cloud (infrared intensity)
void kinect::export_cloud()
{
	is_quit = false;

	while (!(is_quit && cloud_queue.empty())) {

		// Pop Queue
		std::tuple<k4a::image, k4a::image, int64_t> data;
		bool result = cloud_queue.try_pop(data);
		if (!result) {
			std::this_thread::yield();
			continue;
		}

		// Write Point Cloud
		k4a::image cloud_image = std::get<0>(data);
		k4a::image ir_image_t = std::get<1>(data);
		double timestamp = 1e-6*(std::get<2>(data));

		std::string fname = cv::format("%s/%s/%.6f.ply", output_dir.generic_string().c_str(), path_clouds.c_str(), timestamp + stamp_offset);

		int width = cloud_image.get_width_pixels();
		int height = ir_image_t.get_height_pixels();

		int16_t *point_cloud_image_data = (int16_t *)(void *)cloud_image.get_buffer();
		uint16_t *ir_image_data = (uint16_t *)(void *)ir_image_t.get_buffer();

		// Write PLY (binary)
		std::vector<int16_t> property_x;
		std::vector<int16_t> property_y;
		std::vector<int16_t> property_z;
		std::vector<uint16_t> property_v;

		for (int i = 0; i < width * height; i++)
		{
			int16_t x = point_cloud_image_data[3 * i + 0];
			int16_t y = point_cloud_image_data[3 * i + 1];
			int16_t z = point_cloud_image_data[3 * i + 2];
			uint16_t v = ir_image_data[i];

			if (z > 0) {
				property_x.push_back(x);
				property_y.push_back(y);
				property_z.push_back(z);
				property_v.push_back(v);
			}
		}

		// Create an empty object
		happly::PLYData plyOut;

		plyOut.addElement("vertex", property_x.size());

		plyOut.getElement("vertex").addProperty<int16_t>("x", property_x);
		plyOut.getElement("vertex").addProperty<int16_t>("y", property_y);
		plyOut.getElement("vertex").addProperty<int16_t>("z", property_z);
		plyOut.getElement("vertex").addProperty<uint16_t>("ir", property_v);

		plyOut.write(fname, happly::DataFormat::Binary);
		//plyOut.write(fname, happly::DataFormat::ASCII);
	}
}

// Run
void kinect::run()
{
    // Main Loop
    while(true){
        // Update
        update();

        // Wait Key
        constexpr int32_t delay = 1;
        const int32_t key = cv::waitKey(delay);
        if(key == 'q'){
            break;
        }
    }
}

// Update
void kinect::update()
{
    // Update Frame
    update_frame();

	update_images();

    // Update Transformation
    update_transformation();

	// Update Undistorted
	if (is_undistort) {
		update_undistort();
	}

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    bool result = playback.get_next_capture(&capture);

    if(!result){
        // EOF
		while (!color_queue.empty() || !depth_queue.empty() || !infrared_queue.empty() ||
			    !depth_trans_queue.empty() || !cloud_queue.empty()) {
			continue;
		}
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(1000ms);
        std::exit(EXIT_SUCCESS);
    }
}


// Update Images
inline void kinect::update_images()
{
	// Get Images
	color_image = capture.get_color_image();
	depth_image= capture.get_depth_image();
	infrared_image = capture.get_ir_image();

	if (!color_image.handle() ||
		!depth_image.handle() ||
		!infrared_image.handle()) {
		return;
	}

	color = k4a::get_mat(color_image);
	depth = k4a::get_mat(depth_image);
	infrared = k4a::get_mat(infrared_image);

	if(!is_undistort) {
		color_queue.push(std::make_pair(color, color_image.get_device_timestamp().count()));
		depth_queue.push(std::make_pair(depth, depth_image.get_device_timestamp().count()));
		infrared_queue.push(std::make_pair(infrared, infrared_image.get_device_timestamp().count()));
	}
}


// Update Transformation
inline void kinect::update_transformation()
{
	if (!depth_image.handle() || !color_image.handle() || !infrared_image.handle()) {
		return;
	}

	// Transform Depth Image to Color Camera
	depth_image_trans = transformation.depth_image_to_color_camera(depth_image);
	depth_trans = k4a::get_mat(depth_image_trans);

	if (is_c2d) {
		// Resolution of the transformed color image will be twice the resolution of raw depth
		cv::Size size_target = cv::Size(2 * depth.cols, 2 * depth.rows);

		// Two parameters that affect the speed
		int downscale_factor = 4;
		float inpaint_radius = 3.0f;

		// Scale depth camera intrinsics
		K_depth_rect.convertTo(K_depth, CV_32FC1);
		cv::Mat K_depth_scaled = cv::Mat::ones(K_depth.size(), CV_32FC1);
		cv::Mat mulmask = cv::Mat::ones(K_depth.size(), CV_32FC1) * 2;
		mulmask.at<float>(mulmask.rows - 1, mulmask.cols - 1) = 1;
		cv::multiply(K_depth, mulmask, K_depth_scaled);

		// Rectify raw depth map
		cv::Mat depth_rect(depth.size(), depth.type());
		cv::remap(depth, depth_rect, rect_depth.first, rect_depth.second, cv::INTER_NEAREST);

		// Downsample depth map for speed
		cv::Mat depth_down;
		cv::resize(depth_rect, depth_down, cv::Size(), 1.0 / downscale_factor, 1.0 / downscale_factor, cv::INTER_NEAREST);

		// Create binary mask from the downsampled depth map
		cv::Mat mask_down;
		cv::threshold(depth_down, mask_down, 0.0f, 255.0f, cv::THRESH_BINARY_INV);
		mask_down.convertTo(mask_down, CV_8U);

		//int64_t start_ticks = cv::getTickCount();

		// Inpaint depth map
		cv::Mat depth_down_inpaint;
		cv::inpaint(depth_down, mask_down, depth_down_inpaint, inpaint_radius, cv::INPAINT_TELEA);

		//int64_t end_ticks = cv::getTickCount();
		//double time_elapsed = static_cast<double>(end_ticks - start_ticks) / cv::getTickFrequency();
		//std::cout << "Inpainting operation took " << time_elapsed << " seconds." << std::endl;

		// Upsample depth map.
		cv::Mat depth_inpaint;
		cv::resize(depth_down_inpaint, depth_inpaint, size_target, 0, 0, cv::INTER_LINEAR);

		// Depth map to point cloud
		//start_ticks = cv::getTickCount();

		cv::Mat K_inv;
		cv::invert(K_depth_scaled, K_inv);
		std::vector<cv::Point> non_zero_points;
		cv::findNonZero(depth_inpaint, non_zero_points);
		cv::Mat pts2D = cv::Mat::ones(3, static_cast<int>(non_zero_points.size()), CV_32FC1);

		for (int i = 0; i < non_zero_points.size(); ++i) {
			pts2D.at<float>(0, i) = static_cast<float>(non_zero_points[i].x) - 0.5f;
			pts2D.at<float>(1, i) = static_cast<float>(non_zero_points[i].y) - 0.5f;
		}
		cv::Mat pts3D = K_inv * pts2D;

		for (int i = 0; i < non_zero_points.size(); ++i) {
			float depth_value = depth_inpaint.at<uint16_t>(non_zero_points[i].y, non_zero_points[i].x);
			//pts3D.col(i) = depth_value * pts3D.col(i); // Super slow
			pts3D.at<float>(0, i) = depth_value * pts3D.at<float>(0, i);
			pts3D.at<float>(1, i) = depth_value * pts3D.at<float>(1, i);
			pts3D.at<float>(2, i) = depth_value * pts3D.at<float>(2, i);
		}

		//end_ticks = cv::getTickCount();
		//time_elapsed = static_cast<double>(end_ticks - start_ticks) / cv::getTickFrequency();
		//std::cout << "Depth map uprojection took " << time_elapsed << " seconds." << std::endl;

		//start_ticks = cv::getTickCount();

		// Project points to color camera frame
		cv::Mat se3 = cv::Mat(3, 3, CV_32FC1, &calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation);
		cv::Mat rvec = cv::Mat(3, 1, CV_32FC1);
		cv::Rodrigues(se3, rvec);
		cv::Mat tvec = cv::Mat(3, 1, CV_32F, &calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation);

		std::vector<cv::Point2f> pts2D_t;
		cv::projectPoints(pts3D.t(), rvec, tvec, intrinsics_color, dist_coeffs_color, pts2D_t);

		//end_ticks = cv::getTickCount();
		//time_elapsed = static_cast<double>(end_ticks - start_ticks) / cv::getTickFrequency();
		//std::cout << "Point cloud projection took " << time_elapsed << " seconds." << std::endl;

		// Transform color image
		//start_ticks = cv::getTickCount();

		cv::Mat mapx = cv::Mat::zeros(depth_inpaint.size(), CV_32F);
		cv::Mat mapy = cv::Mat::zeros(depth_inpaint.size(), CV_32F);

		for (size_t i = 0; i < non_zero_points.size(); ++i) {
			mapx.at<float>(non_zero_points[i].y, non_zero_points[i].x) = pts2D_t[i].x;
			mapy.at<float>(non_zero_points[i].y, non_zero_points[i].x) = pts2D_t[i].y;
		}

		//cv::remap(color, color_trans, mapx, mapy, cv::INTER_LINEAR);

		// Convert maps to fixed-point representation
		cv::Mat mapx_fixed, mapy_fixed;
		cv::convertMaps(mapx, mapy, mapx_fixed, mapy_fixed, CV_16SC2);
		cv::remap(color, color_trans, mapx_fixed, mapy_fixed, cv::INTER_LINEAR);

		color_trans_queue.push(std::make_pair(color_trans, color_image.get_device_timestamp().count()));

		//end_ticks = cv::getTickCount();
		//time_elapsed = static_cast<double>(end_ticks - start_ticks) / cv::getTickFrequency();
		//std::cout << "Remapping operation took " << time_elapsed << " seconds." << std::endl << std::endl;

	}

	// Point cloud
	point_cloud_image = transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_DEPTH);
	cloud_queue.push(std::make_tuple(point_cloud_image, infrared_image, depth_image.get_device_timestamp().count()));

	if (!is_undistort) {
		depth_trans_queue.push(std::make_pair(depth_trans, depth_image.get_device_timestamp().count()));
		//color_trans_queue.push(std::make_pair(color_trans, color_image.get_device_timestamp().count()));
	}
}


// Update Undistort
inline void kinect::update_undistort()
{
	if (!depth_image.handle() || !color_image.handle() || !infrared_image.handle()) {
		return;
	}

	// Undistort Original Color Image
	cv::Mat color_rect(color.size(), color.type());
	cv::remap(color, color_rect, rect_color.first, rect_color.second, cv::INTER_LINEAR);
	color_queue.push(std::make_pair(color_rect, color_image.get_device_timestamp().count()));

	// Undistort Original Depth Image
	cv::Mat depth_rect(depth.size(), depth.type());
	cv::remap(depth, depth_rect, rect_depth.first, rect_depth.second, cv::INTER_NEAREST);
	depth_queue.push(std::make_pair(depth_rect, depth_image.get_device_timestamp().count()));

	// Undistort Infrared Image
	cv::Mat infrared_rect(infrared.size(), infrared.type());
	cv::remap(infrared, infrared_rect, rect_depth.first, rect_depth.second, cv::INTER_LINEAR);
	infrared_queue.push(std::make_pair(infrared_rect, infrared_image.get_device_timestamp().count()));

	// Undistort Transformed Color Image
	//cv::Mat color_trans_rect(color_trans.size(), color_trans.type());
	//cv::remap(color_trans, color_trans_rect, rect_depth.first, rect_depth.second, cv::INTER_LINEAR);
	//color_trans_queue.push(std::make_pair(color_trans_rect, color_image.get_device_timestamp().count()));
	color_trans_queue.push(std::make_pair(color_trans, color_image.get_device_timestamp().count()));

	// Undistort Transformed Depth Image
	cv::Mat depth_trans_rect(depth_trans.size(), depth_trans.type());
	cv::remap(depth_trans, depth_trans_rect, rect_color.first, rect_color.second, cv::INTER_NEAREST);
	depth_trans_queue.push(std::make_pair(depth_trans_rect, depth_image.get_device_timestamp().count()));
}

// Write Color Camera Calibration (.yaml) File
void kinect::write_calibration(cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat P, int width, int height,
		std::string file_path, std::string name)
{
	cv::FileStorage fs(file_path, cv::FileStorage::WRITE);

	fs << "camera_name" << name;
	fs << "image_width" << width;
	fs << "image_height" << height;

	fs << "camera_matrix" << "{";
	fs << "rows" << K.rows;
	fs << "cols" << K.cols;
	fs << "data" << std::vector<double>((double*)K.data, ((double*)K.data) + (K.rows*K.cols));
	fs << "}";

	fs << "distortion_coefficients" << "{";
	fs << "rows" << D.rows;
	fs << "cols" << D.cols;
	fs << "data" << std::vector<double>((double*)D.data, ((double*)D.data) + (D.rows*D.cols));
	fs << "}";

	fs << "rectification_matrix" << "{";
	fs << "rows" << R.rows;
	fs << "cols" << R.cols;
	fs << "data" << std::vector<double>((double*)R.data, ((double*)R.data) + (R.rows*R.cols));
	fs << "}";

	fs << "projection_matrix" << "{";
	fs << "rows" << P.rows;
	fs << "cols" << P.cols;
	fs << "data" << std::vector<double>((double*)P.data, ((double*)P.data) + (P.rows*P.cols));
	fs << "}";

	fs << "depth_to_color" << "{";
	fs << "rows" << depth_to_color_trans.rows;
	fs << "cols" << depth_to_color_trans.cols;
	fs << "data" << std::vector<double>((double*)depth_to_color_trans.data,
		((double*)depth_to_color_trans.data) + (depth_to_color_trans.rows*depth_to_color_trans.cols));
	fs << "}";

	fs.release();
}

// Write IMU (.csv) File
void kinect::write_imu()
{
	if (!is_imu) {
		return;
	}
	std::string calib_file_path = output_dir.generic_string() + "/" + path_imu + "/calibration.yaml";
	cv::FileStorage fs(calib_file_path, cv::FileStorage::WRITE);

	// Gyro-to-depth
	k4a_calibration_extrinsics_t* ext;
	ext = &calibration.extrinsics[K4A_CALIBRATION_TYPE_GYRO][K4A_CALIBRATION_TYPE_DEPTH];
	float params1[16] = { ext->rotation[0], ext->rotation[1], ext->rotation[2], ext->translation[0] / 1000.0f,
					     ext->rotation[3], ext->rotation[4], ext->rotation[5], ext->translation[1] / 1000.0f,
					     ext->rotation[6], ext->rotation[7], ext->rotation[8], ext->translation[2] / 1000.0f,
					     0.0f, 0.0f, 0.0f, 1.0f };
	cv::Mat extrinsics = cv::Mat(4, 4, CV_32F, params1);
	fs << "gyro2depth" << "{";
	fs << "rows" << 4;
	fs << "cols" << 4;
	fs << "data" << std::vector<float>((float*)extrinsics.data,
		((float*)extrinsics.data) + (extrinsics.rows*extrinsics.cols));
	fs << "}";

	// Gyro-to-color
	ext = &calibration.extrinsics[K4A_CALIBRATION_TYPE_GYRO][K4A_CALIBRATION_TYPE_COLOR];
	float params2[16] = { ext->rotation[0], ext->rotation[1], ext->rotation[2], ext->translation[0] / 1000.0f,
						 ext->rotation[3], ext->rotation[4], ext->rotation[5], ext->translation[1] / 1000.0f,
						 ext->rotation[6], ext->rotation[7], ext->rotation[8], ext->translation[2] / 1000.0f,
						 0.0f, 0.0f, 0.0f, 1.0f };
	extrinsics = cv::Mat(4, 4, CV_32F, params2);
	fs << "gyro2color" << "{";
	fs << "rows" << 4;
	fs << "cols" << 4;
	fs << "data" << std::vector<float>((float*)extrinsics.data,
		((float*)extrinsics.data) + (extrinsics.rows*extrinsics.cols));
	fs << "}";

	// Acc-to-dept
	ext = &calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH];
	float params3[16] = { ext->rotation[0], ext->rotation[1], ext->rotation[2], ext->translation[0] / 1000.0f,
						 ext->rotation[3], ext->rotation[4], ext->rotation[5], ext->translation[1] / 1000.0f,
						 ext->rotation[6], ext->rotation[7], ext->rotation[8], ext->translation[2] / 1000.0f,
						 0.0f, 0.0f, 0.0f, 1.0f };
	extrinsics = cv::Mat(4, 4, CV_32F, params3);
	fs << "acc2depth" << "{";
	fs << "rows" << 4;
	fs << "cols" << 4;
	fs << "data" << std::vector<float>((float*)extrinsics.data,
		((float*)extrinsics.data) + (extrinsics.rows*extrinsics.cols));
	fs << "}";

	// Acc-to-color
	ext = &calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR];
	float params4[16] = { ext->rotation[0], ext->rotation[1], ext->rotation[2], ext->translation[0] / 1000.0f,
						 ext->rotation[3], ext->rotation[4], ext->rotation[5], ext->translation[1] / 1000.0f,
						 ext->rotation[6], ext->rotation[7], ext->rotation[8], ext->translation[2] / 1000.0f,
						 0.0f, 0.0f, 0.0f, 1.0f };
	extrinsics = cv::Mat(4, 4, CV_32F, params4);
	fs << "acc2color" << "{";
	fs << "rows" << 4;
	fs << "cols" << 4;
	fs << "data" << std::vector<float>((float*)extrinsics.data,
		((float*)extrinsics.data) + (extrinsics.rows*extrinsics.cols));
	fs << "}";

	fs.release();

	std::ofstream csv_file;
	std::string file_path = output_dir.generic_string() + "/" + path_imu + "/imu.csv";
	csv_file.open(file_path.c_str());
	csv_file.precision(9);
	csv_file.setf(std::ios::fixed, std::ios::floatfield);

	csv_file <<  "#timestamp[s], w_x[rad s^-1], w_y[rad s^-1], w_z[rad s^-1], a_x[m s^-2], a_y[m s^-2], a_z[m s^-2]" << std::endl;

	k4a_imu_sample_t imu_sample;
	while (playback.get_next_imu_sample(&imu_sample)) {
		double stamp = (1e-6)*imu_sample.gyro_timestamp_usec;
		csv_file << stamp + stamp_offset << ", ";
		csv_file << imu_sample.gyro_sample.xyz.x << ", ";
		csv_file << imu_sample.gyro_sample.xyz.y << ", ";
		csv_file << imu_sample.gyro_sample.xyz.z << ", ";
		csv_file << imu_sample.acc_sample.xyz.x << ", ";
		csv_file << imu_sample.acc_sample.xyz.y << ", ";
		csv_file << imu_sample.acc_sample.xyz.z << std::endl;
	}
	csv_file.close();
}
