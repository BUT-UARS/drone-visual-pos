#ifndef VPS_CONFIG_H
#define VPS_CONFIG_H

//#define MAVLINK_ENABLE 1
#define LOG_ENABLE 1
//#define USE_BARO_FOR_ALT 1
//#define ROTATION_COMPENSATION_ENABLE

//coefficients to account for orientation relative to the flight controller
const float altitude_baro_coeff=1;
const float output_x_coeff=1;
const float output_y_coeff=1;
const float output_z_coeff=1;

const float rot_x_coeff=1;
const float rot_y_coeff=1;
const float rot_z_coeff=1;
const float rot_x_offset=1;
const float rot_y_offset=1;
const float rot_z_offset=1;
//camera settings
const  int image_width = 1640;
const  int image_height = 1232;
const float framerate=30;

//camera paremeters, it is recommended to use meters as the distance unit, values should be replaced by ones obtained by calibration

//left camera intrinsic matrix
cv::Mat M1 = (cv::Mat_<double>(3,3) << 1, 0, 1,
        0, 1, image_width/2,
        0, 0, image_height/2);

//left camera distorsion coefficients
cv::Mat D1 = (cv::Mat_<double>(1,14) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

//right camera intrinsic matrix
cv::Mat M2 = (cv::Mat_<double>(3,3) << 1, 0, 1,
        0, 1, image_width/2,
        0, 0, image_height/2);

//right camera distorsion coefficients
cv::Mat D2 = (cv::Mat_<double>(1,14) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

//rotation matrix
cv::Mat R=(cv::Mat_<double>(3,3)<< 1, 0, 0,
       0, 1, 0,
        0, 0, 1);

//translation vector
cv::Mat T = (cv::Mat_<double>(3,1)<<0.1,
        0,
        0);

static const char *const mavlink_path = "/dev/serial/by-id/YOUR_FLIGHT_CONTROLLER";

static const int number_of_keypoints = 600;

static const double scale_factor_orb = 1.3;

static const int pyramid_levels_orb = 8;

static const int edge_threshold_orb = 59;

static const int patch_size_orb = 59;

static const int fast_threshold_orb = 20;

static const float match_threshold=30;

static const char *const log_path = "/home/user/log.csv";


#endif //VPS_CONFIG_H
