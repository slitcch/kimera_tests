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
#include <time.h>

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

#define U_1_000_000_000 (1000000000)


static inline uint64_t
os_timespec_to_ns(const struct timespec *spec)
{
	uint64_t ns = 0;
	ns += (uint64_t)spec->tv_sec * U_1_000_000_000;
	ns += (uint64_t)spec->tv_nsec;
	return ns;
}

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
    cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		int64_t already = 0;

		auto profile = pipe.start(cfg);



		std::this_thread::sleep_for(std::chrono::milliseconds(300));

    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;
			// Now for the main loop

		while (true) {
			// XXX: Get the most correct timestamp
			struct timespec ts;
			int ret = clock_gettime(CLOCK_MONOTONIC, &ts);
			if (ret != 0) {
				std::cout << "the gay pirate assassins are here, run.\n";
				return 1;
			}
			int64_t the_time = os_timespec_to_ns(&ts);

			rs2::frameset frames = pipe.wait_for_frames();

			std::cout << "size is " << frames.size() << "\n";

			rs2::video_frame fisheye_left = frames.get_fisheye_frame(1);
			rs2::video_frame fisheye_right = frames.get_fisheye_frame(2);

			// rs2::frame has a bool operator that returns false if the frame is empty
			// ie. if this is giving us an IMU value and not fisheye frames.
			if (fisheye_left && fisheye_right){
				std::cout << "Fishy.\n";



			cv::Mat fisheye_left  = cv::Mat(cv::Size(848,800), CV_8UC1, (void*)frames.get_fisheye_frame(1).get_data(), cv::Mat::AUTO_STEP);
			cv::Mat fisheye_right = cv::Mat(cv::Size(848,800), CV_8UC1, (void*)frames.get_fisheye_frame(2).get_data(), cv::Mat::AUTO_STEP);

cv::imshow("video",fisheye_left);
				cv::imshow("video1",fisheye_right);
				cv::waitKey(1);

			vio_pipeline.fillLeftFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            (VIO::Timestamp)the_time,
            (VIO::CameraParams&)vio_params.camera_params_[0],
            (cv::Mat&)fisheye_left
        ));
        vio_pipeline.fillRightFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            (VIO::Timestamp)the_time,
            (VIO::CameraParams&)vio_params.camera_params_[1],
            (cv::Mat&)fisheye_right
        ));
				frame_id++;
			}

			rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
			rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);

			if (gyro_frame && gyro_frame.get_profile().stream_type() == RS2_STREAM_GYRO && accel_frame && accel_frame.get_profile().stream_type() == RS2_STREAM_ACCEL) { // Same deal as the fisheye frames. These better be in sync or so help me god
				rs2_vector gyro_vec = gyro_frame.get_motion_data();
				rs2_vector accel_vec = accel_frame.get_motion_data();

				// First 3 elements correspond to acceleration data [m/s^2]
				// while the 3 last correspond to angular velocities [rad/s].
				VIO::ImuAccGyr reading;
				reading(0) = accel_vec.x;
				reading(1) = accel_vec.y;
				reading(2) = accel_vec.z;
				reading(1) = gyro_vec.x;
				reading(2) = gyro_vec.y;
				reading(3) = gyro_vec.z;
				std::cout << "Wow, this is really an accelerating experience!\n";
				std::cout << reading(0) << " " << reading(1) << " " << reading(2) << " " << reading(3) << " " << reading(4) << " " << reading(5) << " " << reading(6) << "\n";

				vio_pipeline.fillSingleImuQueue(VIO::ImuMeasurement( the_time,reading));
			
			}





			vio_pipeline.spin();
			// std::cout << "refasdfsafdfasdfa\n";
			vio_pipeline.spinViz(); 
    
		}
		// unreachable for now
	vio_pipeline.shutdown();

    return EXIT_SUCCESS;
    
}