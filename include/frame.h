#ifndef FRAME_H
#define FRMAE_H
#include<opencv2/core.hpp>
#include<ros/ros.h>
class systemParam
{
    public:
        double dT_Cam0_2_Imu[9];
        double dIntrinsicsLeft[5], dIntrinsicsRight[5];
        double dDistortionCoeffsLeft[5], dDistortionCoeffsRight[5];
        double pT_CamLeft2Imu[4][4], pT_CamRight2Imu[4][4];
        double dTimeshiftCamLeft2Imu, dTimeshiftCamRight2Imu;

        double kMatrixLeft[3][3], kMatrixRight[3][3];
        double gyroNoise, gyroRandom, accNoise, accRandom;
        
    public:
        systemParam(ros::NodeHandle nh);
};

class image
{
    public:
        int height, width;
        cv::Mat cvMatImage;
        Eigen::Matrix3d K;
        vector<cv::KeyPoint> kpNow, kpLast;
        cv::Mat descriptors;
        int64_t timestamp;
        double intrinsics[5];
    public:
        image(sensor_msgs::ImageConstPtr);
        void getK();
};

class frame
{
    public: 
        double time_stamp;
        image leftImage, rightImage;
        ros::NodeHandle nh;

        Sophus::SE3 T_c_w;
        cv::Mat image_grey, image_depth;    
        cv::Mat descriptors;
        std::shared_ptr<std::vector<cv::KeyPoint>> p_keyPoints;

    frame();

    
    frame(int pixel_row_num, int pixel_col_num, Mynteye camera_params, cv::Mat image_grey):pixel_row_num(pixel_row_num), pixel_col_num(pixel_col_num),\
    camera_params(camera_params), image_grey(image_grey) {}
    frame(int pixel_row_num, int pixel_col_num, Mynteye camera_params, cv::Mat image_grey, cv::Mat image_depth):pixel_row_num(pixel_row_num), pixel_col_num(pixel_col_num),\
    camera_params(camera_params), image_grey(image_grey), image_depth(image_depth) {}
    
};

void get_pose_estimation_2d2d(const std::vector<cv::KeyPoint> keypoints1, const std::vector<cv::KeyPoint> keypoints2, \
const std::vector<cv::DMatch> matches, cv::Mat* p_R, cv::Mat* p_t, Mynteye mynt_params);

//keypoint class 将cv::KeyPoint转换为基础数据，用以ros的自定义的发布
// class keypoint
// {
//     public:
//         float pt[2];//x,y的坐标
//         float size;
//         float angle;
//         float response;
//         int octave;
//         int class_id;
//     keypoint(cv::KeyPoint kp):pt(kp.pt[0], kp.pt[1]), size(kp.size), angle(kp.angle), response(kp.response), octave(kp.octave),class_id(class_id) {};
//     keypoint();
// };



#endif