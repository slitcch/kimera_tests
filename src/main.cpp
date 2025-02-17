/**
 * @file main.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @author Moses Turner (mosesturner@protonmail.com)
 * @brief OAK-D/Kimera Experiments main file.
 * 
 * @version 0.1
 * @date 2021-01-16
 * 
 * @copyright Copyright (c) Duncan R Hamill 2021
 * @copyright Moses Turner, 2021.
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <iostream>
#include <chrono>
#include <thread>
#include <future>


#include <kimera-vio/common/vio_types.h>
#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/frontend/Frame.h>
#include <kimera-vio/utils/Statistics.h>
#include <kimera-vio/utils/Timer.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>



// #include "depthai/depthai.hpp"
// #include "util.h"

#include <librealsense2/rs.hpp>
// #include <librealsense2/h/rs_types.h>
// #include <librealsense2/h/rs_pipeline.h>
// #include <librealsense2/h/rs_option.h>
// #include <librealsense2/h/rs_frame.h>

/* -------------------------------------------------------------------------
 * CONSTANTS
 * ------------------------------------------------------------------------- */

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

// This defines a variable called FLAGS_params_folder_path, for some reason.
DEFINE_string(
    params_folder_path,
    "params/t26x",
    "Path to the folder containing the yaml files with the VIO parameters.");

uint64_t frame_id = 0; // not a good way of doing this...

/* -------------------------------------------------------------------------
 * STRUCTS
 * ------------------------------------------------------------------------- */

// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;


VIO::Timestamp timestamp_rs_to_vio(rs2::frame& frame) {
	double ts_rs = frame.get_timestamp() - (double)1616190000000.0f;

	int64_t ts_vio = (int64_t)(ts_rs * 1000.0f * 1000.0f * 1000.0f);
	
	// std::cout << ts_rs << " " << ts_vio << "\n";

    return (VIO::Timestamp)ts_vio;
}

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

bool cango0 = false;
bool cango1 = false;

int main(int argc, char *argv[]) {

    std::cout << "OAK-D/Kimera-VIO Experiment\n" << std::endl;

    // Init Google log and flags, which are used by Kimera
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Parse VIO parameters from gflags.
    VIO::VioParams vio_params(FLAGS_params_folder_path);

    // Create the VIO pipeline
    VIO::Pipeline vio_pipeline(vio_params);

		// Shoutout to Damien Rompapas (https://github.com/HyperLethalVector) for the Realsense code I'm copying:
		rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile myProf; 
    std::vector<std::string> serials;
    uint32_t dev_q; 
    rs2::context ctx; 
    cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE, 1);
    cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE, 2);
    // cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		int64_t already = 0;

		auto profile = pipe.start(cfg, [&](rs2::frame frame)
		{
			if (rs2::pose_frame fs = frame.as<rs2::pose_frame>())
			{
				cango0 = true;
				// std::cout << "hey dog1\n";

				rs2_pose pose = fs.get_pose_data();
				
				// First 3 elements correspond to acceleration data [m/s^2]
				// while the 3 last correspond to angular velocities [rad/s].
				VIO::ImuAccGyr reading;
				reading(0) = pose.acceleration.x;
				reading(1) = pose.acceleration.y;
				reading(2) = pose.acceleration.z;
				reading(3) = pose.angular_acceleration.x;
				reading(4) = pose.angular_acceleration.y;
				reading(5) = pose.angular_acceleration.z;
				VIO::Timestamp time = timestamp_rs_to_vio(frame);
				//VIO::ImuAccGyr(pose.acceleration.x, pose.acceleration.y, pose.acceleration.z,
													//  pose.angular_acceleration.x, pose.angular_acceleration.y, pose.angular_acceleration.z)
				if (time >= already){
				vio_pipeline.fillSingleImuQueue(VIO::ImuMeasurement(
            timestamp_rs_to_vio(frame),reading
            
        ));} else {
					// std::cout << "what the heck\n";
				}
			}
			else if (auto fs = frame.as<rs2::frameset>())
			{
				// rs2::video_frame frame_left = fs.get_fisheye_frame(1).get_data();
				// rs2::video_frame frame_right = fs.get_fisheye_frame(2);
				if (cango0){
				cango1 = true;
				// std::cout << frame_id << " hey dog\n";
				cv::Mat fisheye_left  = cv::Mat(cv::Size(848,800), CV_8UC1, (void*)fs.get_fisheye_frame(1).get_data(), cv::Mat::AUTO_STEP);
				cv::Mat fisheye_right = cv::Mat(cv::Size(848,800), CV_8UC1, (void*)fs.get_fisheye_frame(2).get_data(), cv::Mat::AUTO_STEP);

				cv::imshow("video",fisheye_left);
				cv::imshow("video1",fisheye_right);
				cv::waitKey(1);

				VIO::Timestamp time = timestamp_rs_to_vio(frame);

				vio_pipeline.fillLeftFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            (VIO::Timestamp)time,
            (VIO::CameraParams&)vio_params.camera_params_[0],
            (cv::Mat&)fisheye_left
        ));
        vio_pipeline.fillRightFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            (VIO::Timestamp)time,
            (VIO::CameraParams&)vio_params.camera_params_[1],
            (cv::Mat&)fisheye_right
        ));
				frame_id++; // BAD
			}}
		} );

		while(!(cango0 && cango1)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;

    // Now for the main loop
			auto handle_pipeline = std::async(std::launch::async, &VIO::Pipeline::spin, &vio_pipeline);
			std::cout << "refasdfsafdfasdfa\n";
			vio_pipeline.spinViz(); // runs forever
			// Above two lines will only do what you want if you have parallel_run: 1 in PipelineParams.yaml

    // Shutdown the VIO pipeline
    vio_pipeline.shutdown();

    return EXIT_SUCCESS;
}