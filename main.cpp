#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mocap/mocap.h>

#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/imgcodecs.hpp>

#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <jetson-utils/gstCamera.h>
#include <jetson-utils/cudaGrayscale.h>
#include <jetson-utils/cudaMappedMemory.h>

#include <fstream>
#include <iostream>

#include <memory>
#include <thread>

#include "config.h"
using namespace cv;
using namespace std;
using namespace mavsdk;
struct refpoint
{
    Point3f coords;
    bool isfresh = true;
    unsigned int desc_index=UINT16_MAX;
};

float calcMedian(std::vector<float> vector);
Mat calcRotationMatrix();


float pitch_deg=0; //x
float roll_deg=0;   //y
float yaw_deg=0;    //z
float altitude_baro=0;


int main(int argc, char** argv)
{
    videoOptions opt_l;
    opt_l.width  = image_width;
    opt_l.height = image_height;
    opt_l.frameRate = framerate;
    opt_l.zeroCopy = true;
    opt_l.resource = "csi://0";

    videoOptions opt_r;
    opt_r.width  = image_width;
    opt_r.height = image_height;
    opt_r.frameRate = framerate;
    opt_r.zeroCopy = true;
    opt_r.resource = "csi://1";


    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;


    auto imageSize = Size(image_width * 2, image_height * 2);
    cv::stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, Size(image_width, image_height));
    Mavsdk mavsdk;

    videoSource * camera_input_l = gstCamera::Create(opt_l);
    videoSource * camera_input_r = gstCamera::Create(opt_r);

    uchar3* image_l = NULL;
    uchar3* image_r = NULL;
    uint8_t* image_l_g = NULL;
    uint8_t*  image_r_g = NULL;

    if (!camera_input_l||!camera_input_r) {
        std::cerr << "Failed to create input stream" << std::endl;
        return 1;
    }

    cv::cuda::GpuMat desc_l;
    vector<KeyPoint> cpu_kp_l;

    cv::cuda::GpuMat desc_r;
    vector<KeyPoint> cpu_kp_r;

    cv::cuda::GpuMat desc_l_past;

    vector <Point2f> points_l;
    vector <Point2f> points_r;
    vector<DMatch> matches_lr;
    vector<DMatch> matches_pc;


    vector <Point3f> triangpoints;
    Mat triangpoints_homogenous;
    Mat triangpoints_correct;

    float dist_lr_arr[number_of_keypoints]={0 };
    unsigned int index_lr_arr[number_of_keypoints]={0};

    float dist_pc_arr[number_of_keypoints]={0};
    unsigned int index_pc_arr[number_of_keypoints]={0};


    unsigned int index_point_arr[number_of_keypoints]={0};


    unsigned int arr_lr_counter = 0;
    unsigned int arr_pc_counter = 0;
    unsigned int index_point_counter=0;

    vector<refpoint> refpoints;

    vector <float> position_candidates_x;
    vector <float> position_candidates_y;
    vector <float> position_candidates_z;

    Point3f   true_position;



    auto orb = cuda::ORB::create(number_of_keypoints, scale_factor_orb, pyramid_levels_orb, edge_threshold_orb, 0, 2, cv::ORB::HARRIS_SCORE, patch_size_orb,fast_threshold_orb, true);
    auto matcher_gpu = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);

    std::chrono::high_resolution_clock::time_point loop_start_time;
    std::chrono::high_resolution_clock::time_point loop_end_time;
    std::chrono::milliseconds loop_duration_ms;

    cv::cuda::GpuMat im_l_g;
    cv::cuda::GpuMat im_r_g;



    if( !cudaAllocMapped(&image_l_g, image_width, image_height))
        std::cerr << "Allocation error"<< std::endl;
    if( !cudaAllocMapped(&image_r_g, image_width, image_height))
        std::cerr << "Allocation error"<< std::endl;

    std::fill(dist_lr_arr, dist_lr_arr + number_of_keypoints, FLT_MAX);
    std::fill(index_lr_arr, index_lr_arr + number_of_keypoints, UINT16_MAX);

    std::fill(dist_pc_arr, dist_pc_arr + number_of_keypoints, FLT_MAX);
    std::fill(index_pc_arr, index_pc_arr + number_of_keypoints, UINT16_MAX);
    std::fill(index_point_arr, index_point_arr + number_of_keypoints, UINT16_MAX);


    true_position.x=0;
    true_position.y=0;
    true_position.z=0;



#ifdef LOG_ENABLE
    std::ofstream myfile;
    myfile.open (log_path);
#endif
#ifdef MAVLINK_ENABLE
    ConnectionResult connection_result = mavsdk.add_serial_connection(mavlink_path);


    if (connection_result != ConnectionResult::Success) {
        cerr<<"Connection failed"<<endl;
        return 1;
    }


        auto system = mavsdk.first_autopilot(5.0);

        auto telemetry = Telemetry{system};
        auto action = Action{system};
        auto mocap = Mocap{system};
        telemetry.set_rate_position(20.0);
        telemetry.set_rate_attitude(20.0);

        telemetry.subscribe_attitude_euler([](Telemetry::EulerAngle eulerAngle) {
            pitch_deg=eulerAngle.pitch_deg;
            roll_deg=eulerAngle.roll_deg;
            yaw_deg=eulerAngle.yaw_deg;
        });
        telemetry.subscribe_position([](Telemetry::Position position) {
            altitude_baro=position.relative_altitude_m;
        });
#endif

    while (camera_input_l && camera_input_r)
    {
#ifdef MAVLINK_ENABLE
    if (telemetry.in_air())
    {
#endif
        loop_start_time = std::chrono::high_resolution_clock::now();

        camera_input_l->Capture(&image_l, 100);
        camera_input_r->Capture(&image_r, 100);

        cudaRGB8ToGray8(image_l,image_l_g,image_width,image_height);
        cudaRGB8ToGray8(image_r,image_r_g,image_width,image_height);
        cv::cuda::GpuMat im_l(image_height, image_width, CV_8UC1, image_l_g);
        cv::cuda::GpuMat im_r(image_height, image_width, CV_8UC1, image_r_g);


        if(im_l.size().width>1&&im_l.size().height>1&&im_r.size().width>1&&im_r.size().height>1)
        {
            orb->detectAndCompute(im_l, noArray(), cpu_kp_l, desc_l);
            orb->detectAndCompute(im_r, noArray(), cpu_kp_r, desc_r);

            matcher_gpu->match(desc_r,desc_l,matches_lr);

            matcher_gpu->match(desc_l_past,desc_l,matches_pc);
            for (arr_lr_counter = 0; arr_lr_counter < matches_lr.size(); arr_lr_counter++ ) {
                if (matches_lr[arr_lr_counter].distance < match_threshold) {

                    if (matches_lr[arr_lr_counter].distance < dist_lr_arr[matches_lr[arr_lr_counter].trainIdx]) {
                        dist_lr_arr[matches_lr[arr_lr_counter].trainIdx] = matches_lr[arr_lr_counter].distance;
                        index_lr_arr[matches_lr[arr_lr_counter].trainIdx] = matches_lr[arr_lr_counter].queryIdx;

                    }

                }
            }
            for (arr_pc_counter = 0; arr_pc_counter < matches_pc.size(); ++arr_pc_counter)
            {
                if (matches_pc[arr_pc_counter].distance < match_threshold) {

                    if (matches_pc[arr_pc_counter].distance < dist_pc_arr[matches_pc[arr_pc_counter].trainIdx]) { // dist_pc_arr[index of left descriptor]=distance of left descriptor match
                        dist_pc_arr[matches_pc[arr_pc_counter].trainIdx] = matches_pc[arr_pc_counter].distance;     // index_pc_arr[index of left descriptor]=index of past left descriptor
                        index_pc_arr[matches_pc[arr_pc_counter].trainIdx] = matches_pc[arr_pc_counter].queryIdx;    // index_lr_arr [index of left descriptor]=index of right descriptor

                    }

                }
            }
            for (int j = 0; j < arr_lr_counter; ++j) {//create refpoints form valid matches
                if(dist_lr_arr[j]<match_threshold)
                {
                    points_l.push_back(cpu_kp_l[j].pt);
                    points_r.push_back(cpu_kp_r[index_lr_arr[j]].pt);
                    index_point_arr[j]=index_point_counter;// index_point_arr[index of left descriptor] = index of point
                    index_point_counter++;
                }
            }
            if (!points_l.empty() && !points_r.empty())
            {
                undistortPoints(points_l, points_l, M1, D1, R1, P1);
                undistortPoints(points_r, points_r, M2, D2, R2, P2);


                cout << "Number of refpoints: " << points_l.size() << endl;

                triangulatePoints(P1, P2, points_l, points_r, triangpoints_homogenous);
                cv::sfm::homogeneousToEuclidean(triangpoints_homogenous,triangpoints_correct);
#ifdef ROTATION_COMPENSATION_ENABLE
                Mat rotmat= calcRotationMatrix();
                triangpoints_correct=rotmat*triangpoints_correct;
#endif
                for (int i = 0; i <refpoints.size(); ++i) {
                    refpoints[i].isfresh = false;
                }
                for (int i  = 0; i  < arr_pc_counter; ++i ) {
                    if (dist_pc_arr[i]<match_threshold&&dist_lr_arr[i]<match_threshold) //for each point that was already detected previously
                    {
                        for (int j = 0; j < refpoints.size(); ++j) {
                            if (refpoints[j].desc_index==index_pc_arr[i])//if the point is matching the refpoint, calcluate candidate for drone position
                            {

                                position_candidates_x.push_back(triangpoints_correct.at<float>(0,index_point_arr[i])+refpoints[j].coords.x);
                                position_candidates_y.push_back(triangpoints_correct.at<float>(1,index_point_arr[i])+refpoints[j].coords.y);
                                position_candidates_z.push_back(triangpoints_correct.at<float>(2,index_point_arr[i])+refpoints[j].coords.z);
                                refpoints[j].desc_index=i;
                                refpoints[j].isfresh= true;
                            }
                        }
                    }
                }
                if(!position_candidates_x.empty())
                {

                    true_position.x= calcMedian(position_candidates_x);
                    true_position.y= calcMedian(position_candidates_y);
#ifndef USE_BARO_FOR_ALT
                    true_position.z= calcMedian(position_candidates_z);
#else
                    true_position.z=altitude_baro*altitude_baro_coeff;
#endif
                    //true_position.x=cv::mean(position_candidates_x)[0];
                    //true_position.y=cv::mean(position_candidates_y)[0];
                    //true_position.z=cv::mean(position_candidates_z)[0];
                    cout<<"True position: "<<true_position<<endl;
#ifdef MAVLINK_ENABLE
                    if (system)
                    {
                        mavsdk::Mocap::VisionPositionEstimate estimate;
                        estimate.position_body.x_m=true_position.x*output_x_coeff;
                        estimate.position_body.y_m=true_position.y*output_y_coeff;
                        estimate.position_body.z_m=true_position.z*output_z_coeff;
                        mocap.set_vision_position_estimate(estimate);
                    }
#endif
#ifdef LOG_ENABLE
                        myfile<<true_position.x<<","<<true_position.y<<","<<true_position.z<<"\n";
#endif
                }


                for (int i = 0; i <refpoints.size(); ++i) {
                    if(!refpoints[i].isfresh)
                    {
                        refpoints.erase(refpoints.begin()+i);//delete old points that were not detected this loop
                    }
                }

                for (int i  = 0; i  < arr_lr_counter; ++i ) {
                    if (dist_pc_arr[i]>=match_threshold&&dist_lr_arr[i]<match_threshold) //insert new points into the refpoints vector
                    {
                        refpoint temprefpoint;
                        Point3_<float> temppoint;
                        temppoint.x = triangpoints_correct.at<float>(0,index_point_arr[i]);
                        temppoint.y = triangpoints_correct.at<float>(1,index_point_arr[i]);
                        temppoint.z = triangpoints_correct.at<float>(2,index_point_arr[i]);
                        temprefpoint.coords=true_position-temppoint;
                        temprefpoint.desc_index=i;
                        temprefpoint.isfresh = true;
                        refpoints.push_back(temprefpoint);

                    }
                }
            }


        }


        //reset arrays
        std::fill(dist_lr_arr, dist_lr_arr + arr_lr_counter, FLT_MAX);
        std::fill(index_lr_arr, index_lr_arr + arr_lr_counter, UINT16_MAX);

        std::fill(dist_pc_arr, dist_pc_arr + arr_pc_counter, FLT_MAX);
        std::fill(index_pc_arr, index_pc_arr + arr_pc_counter, UINT16_MAX);

        std::fill(index_point_arr, index_point_arr + index_point_counter, UINT16_MAX);


        //empty variables
        desc_l.copyTo(desc_l_past);
        points_l.clear();
        points_r.clear();
        index_point_counter=0;
        position_candidates_x.clear();
        position_candidates_y.clear();
        position_candidates_z.clear();
        loop_end_time = std::chrono::high_resolution_clock::now();
        loop_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
        std::cout << "Loop duration: " <<loop_duration_ms.count() << "ms " << std::endl;
#ifdef MAVLINK_ENABLE
    } else
    {
        true_position.x=0;
        true_position.y=0;
        true_position.z=0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
#endif
    }
    camera_input_l->Close();
    camera_input_r->Close();
#ifdef LOG_ENABLE
    myfile.close();
#endif
    return 0;
}

float calcMedian(std::vector<float> vector)
{
    auto vector_middle = vector.begin() + floor(vector.size() * 0.5);
    std::nth_element(vector.begin(), vector_middle, vector.end());
    return *vector_middle;

}

Mat calcRotationMatrix()
{
    // x axis
    Mat rotx = (Mat_<float>(3,3) <<
                                 1,0,0,
            0,cos(pitch_deg*0.017453292),-sin(pitch_deg*0.017453292),
            0,sin(pitch_deg*0.017453292),cos(pitch_deg*0.017453292));

    // y axis
    Mat roty = (Mat_<float>(3,3) <<
                                 cos(roll_deg*0.017453292),0,sin(roll_deg*0.017453292),
            0,1,0,
            -sin(roll_deg*0.017453292),0,cos(roll_deg*0.017453292));

    // z axis
    Mat rotz = (Mat_<float>(3,3) <<
                                 cos(yaw_deg*0.017453292),-sin(yaw_deg*0.017453292),0,
            sin(yaw_deg*0.017453),cos(yaw_deg*0.017453292),0,
            0,0,1);

    return rotx*roty*rotz;
}
