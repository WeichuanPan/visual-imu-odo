#ifndef cameraNodes_H
#define cameraNodes_H
#include<iostream>
#include<algorithm>
#include<stdio.h>
#include<vector>
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
// #include<opencv2/xfeatures2d/nonfree.hpp>
#include<cv_bridge/cv_bridge.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<boost/thread/thread.hpp>
#include<Eigen/Core>
#include<sophus/se3.h>
// #include "camera.h"
using namespace std;

static int resolution = 752 * 480;
//left camera
static double xi_left = 1.1806148576800861;
static double fu_left = 701.1307622240085;
static double fv_left = 703.1080314857786;
static double pu_left = 378.0566680943181;
static double pv_left = 188.15668023579335;
static double time_shift_left = -0.0005059423461297469;
static double disc_k1_left = 0.0625246105784006;
static double disc_k2_left = -0.7277659355916754;
static double disc_p1_left = 0.01915675463326762;
static double disc_p2_left = 0.0005569274456103805;

static double transformation_imu2leftcam[4][4] = {-0.024071935625889607, 0.9980119780708341, -0.058246317843825085, 0.051256501943867105, 
-0.996011524257566, -0.018934933034799872, 0.08719238416907608, 7.348205590327135e-05,
0.0859161536693952, 0.06011289327680473, 0.9944872319947342, -0.08451592055528777,
0.0, 0.0, 0.0, 1.0};

// right camera

static double xi_right = 1.2953701844954368;
static double fu_right =  710.3844549898763;
static double fv_right = 710.4285739826983;
static double pu_right = 384.83110508006024;
static double pv_right = 221.0616863891682;
static double time_shift_right =  -0.001120711829094518;
static double disc_k1_right = 0.19750895382402686;
static double disc_k2_right = -0.4042070280819853;
static double disc_p1_right = 0.011211379044141559;
static double disc_p2_right =  -0.016373313680729314;
static double transformation_imu2rightcam[4][4] = {-0.03296710147108256, 0.9979855313483119, -0.054203776990382514, -0.07126213498781858, 
-0.9985746045635804, -0.030611841565605535, 0.04372269749971461, -0.005316181574240802,
0.04197534206274719, 0.055567925779086244, 0.9975721910134252, -0.09707538255818604,
0.0, 0.0, 0.0, 1.0};

static double transformation_camleft2camright[4][4] = {0.9999522665943039, 0.009212667234026889, 0.0032544270388869035, -0.12224181635294094, 
-0.009060047090136025, 0.998983733383689, -0.0441521911292456, -0.008656765709317879,
-0.0036578791178589524, 0.04412059833257028, 0.9990195156868245, -0.012458080521374603,
0.0, 0.0, 0.0, 1.0};
//t_imu = t_cam + time_shift
// Camera camera_left(fu_left, fv_left, pu_left, pv_left, xi_left, disc_k1_left, disc_k2_left, disc_p1_left, disc_p2_left, 0);
struct s_camera
{
    double cx, cy, fx, fy, scale, k1, k2, p1, p2, xi;

};
struct FRAME
{
    cv::Mat rgb, depth;
    cv::Mat desp;
    vector<cv::KeyPoint> kp;
};
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};
void left_imageCallback(const sensor_msgs::ImageConstPtr& msg);
void right_imageCallback(const sensor_msgs::ImageConstPtr& msg);
void last_now_imageCallback(const sensor_msgs::ImageConstPtr& msg);
void dmatch_TransformationMatrix(const cv::Mat, const std::vector<cv::KeyPoint>, Sophus::SE3);
// RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, Camera camera_left);

bool comp(cv::DMatch, cv::DMatch);
cv::Point3f point2dTo3d(const cv::Point3f, const s_camera);
void dmatchs2point2f(const vector<cv::DMatch> dmatches, const vector<cv::KeyPoint> src, const vector<cv::KeyPoint> dst,
    vector<cv::Point2f> &src_pt2f, vector<cv::Point2f> &dst_pt2f);

void poseEstimation_2d2d(vector<cv::Point2f> points1, vector<cv::Point2f> points2, cv::Mat &R, cv::Mat &t, Eigen::Matrix3d K);
bool comp(cv::DMatch match1, cv::DMatch match2);


void couple_grey_depth(const vector<cv::Point2d> grey, const vector<cv::Point2f> depth, vector<cv::Point3f> &gd);
#endif