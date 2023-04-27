#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <opencv2/opencv.hpp>

#if __has_include(<concurrent_queue.h>)
#include <concurrent_queue.h>
#else
#include <tbb/concurrent_queue.h>
namespace concurrency = tbb;
#endif

#include <thread>
#include <atomic>
#include <fstream>

#if __has_include(<filesystem>)
//#include <filesystem>
//namespace filesystem = std::filesystem;
//#else
#include <experimental/filesystem>
#if _WIN32
namespace filesystem = std::experimental::filesystem::v1;
#else
namespace filesystem = std::experimental::filesystem;
#endif
#endif

class kinect
{
private:
    // Kinect
    k4a::playback playback;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_record_configuration_t record_configuration;

	bool is_color;
	bool is_depth;
	bool is_infrared;

    // Color image
    k4a::image color_image;
    cv::Mat color;
	std::string path_color;

	// Color image (transformed)
	k4a::image color_image_trans;
	cv::Mat color_trans;
	std::string path_color_trans;
	
    // Depth image
    k4a::image depth_image;
    cv::Mat depth;
	std::string path_depth;

	// Depth map (transformed)
	k4a::image depth_image_trans;
	cv::Mat depth_trans;
	std::string path_depth_trans;

    // Infrared image
    k4a::image infrared_image;
    cv::Mat infrared;
	std::string path_ir;

	// Point cloud (depth frame)
	k4a::image point_cloud_image;
	std::string path_clouds;

	// IMU
	bool is_imu;
	std::string path_imu;

	// Timestamps
	std::ofstream timestamps_color_file;
	std::ofstream timestamps_depth_file;

	// Calibration
	cv::Mat K_color, K_depth, K_depth_rect;
	cv::Mat P_color, P_depth;
	cv::Mat D_color, D_depth;
	cv::Mat R_color, R_depth;
	cv::Size size_color, size_depth;
	cv::Mat depth_to_color_trans;
	std::pair<cv::Mat, cv::Mat> rect_color, rect_depth;

	cv::Mat dist_coeffs_color;
	cv::Mat intrinsics_color;

    // Thread
    std::atomic_bool is_quit;
    std::thread color_thread;
	std::thread color_trans_thread;
	std::thread depth_thread;
	std::thread depth_trans_thread;
	std::thread infrared_thread;
	std::thread clouds_thread;

	double stamp_offset;

	concurrency::concurrent_queue<std::pair<cv::Mat, int64_t>> color_queue;
	concurrency::concurrent_queue<std::pair<cv::Mat, int64_t>> color_trans_queue;
	concurrency::concurrent_queue<std::pair<cv::Mat, int64_t>> depth_queue;
	concurrency::concurrent_queue<std::pair<cv::Mat, int64_t>> depth_trans_queue;
    concurrency::concurrent_queue<std::pair<cv::Mat, int64_t>> infrared_queue;
	concurrency::concurrent_queue<std::tuple<k4a::image, k4a::image, int64_t>> cloud_queue;

    filesystem::path mkv_file;
    filesystem::path output_dir;
	bool is_undistort;
	bool is_c2d;
	bool is_export_clouds;


public:
    // Constructor
    kinect(int argc, char* argv[]);

    // Destructor
    ~kinect();

    // Run
    void run();

    // Update
    void update();

private:
    // Initialize
    void initialize(int argc, char* argv[]);

    // Initialize Parameter
    void initialize_parameter(int argc, char* argv[]);

    // Initialize Playback
    void initialize_playback();

	// Initialize Camera Calibration
	void initialize_calibration();

    // Initialize Save
    void initialize_save();

    // Finalize
    void finalize();

    // Export images/clouds
    void export_color();
	void export_color_trans();
    void export_depth();
	void export_depth_trans();
    void export_infrared();
	void export_cloud();

    // Update Frame
    void update_frame();

	void update_images();

    // Update Transformation
    void update_transformation();

	// Update Undistort
	void update_undistort();

	// Write Camera Calibration (.yaml) Files
	void write_color_calibration();
	void write_depth_calibration();
	void write_calibration(cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat P, int width, int height,
		std::string file_path, std::string name);

	// Write IMU (.csv) File
	void write_imu();

};

#endif // __KINECT__
