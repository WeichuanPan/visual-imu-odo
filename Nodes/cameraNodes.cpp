#include "../include/cameraNodes.h"
static cv::Mat last_image;
static cv::Mat now_image;
static std::vector<cv::KeyPoint> last_keypoints;
static cv::Mat last_descriptors;
static int frame_cnt = 0;
void left_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        // cv::Mat image_left = cv_bridge::toCvShare(msg,"bgr8")->image;
        cv::Mat image_left = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        std::vector<cv::KeyPoint> keypoints1;
        cv::Mat descriptors1;
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(40);

        // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        cout<<"\n Here:13 \n"<<endl;

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        cout<<"Here:16"<<endl;
        detector->detect(image_left, keypoints1);
        cout<<"\n Here:18"<<endl;

        // descriptor->compute(image_left, keypoints1, descriptors1);
        cv::Mat outimg1;
        
        cv::drawKeypoints(image_left, keypoints1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cout<<outimg1;
        cv::imshow("Leftview",outimg1);
        // cv::waitKey(30);
    // cv::destroyWindow("Leftview");
    return;
}

void right_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // cv::namedWindow("RightView", 1);
    try 
    {
        cv::Mat image_right = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        std::vector<cv::KeyPoint> keypoints1;
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create();
        detector->detect(image_right, keypoints1);
        cv::Mat outimage1;
        cv::drawKeypoints(image_right, keypoints1, outimage1);
        cv::imshow("RightView", outimage1);
    }
    catch(cv_bridge::Exception&e)
    {
        ROS_ERROR("Could not covert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }
    // cv::destroyWindow("RightView");
    return;
}


int main(int argc, char** argv)
{
    // ros::init(argc,argv,"image_listener");
    // ros::NodeHandle nh_left;
    // cv::startWindowThread();
    // cv::namedWindow("Leftview",0);
    // cv::namedWindow("RightView",1);
    // image_transport::ImageTransport it_left(nh_left);
    // image_transport::Subscriber sub = it_left.subscribe("/mynteye/left/image_raw", 1, left_imageCallback);
    // ros::Duration(0.5).sleep();
    // ros::NodeHandle nh_right;
    // image_transport::ImageTransport it_right(nh_right);
    // image_transport::Subscriber subLeft = it_right.subscribe("/mynteye/right/image_raw", 1, right_imageCallback);
    //  ros::Duration(0.5).sleep();
    // ros::spin();
    // cv::destroyWindow("LeftView");
    // cv::destroyWindow("RightView");
    // return 0;

    ros::init(argc, argv, "LastAndNow");
    ros::NodeHandle nh;
    cv::startWindowThread();
    cv::namedWindow("LastWindow",0);
    cv::namedWindow("NowWindow", 1);
    cv::namedWindow("MatchWindow",2);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/mynteye/left/image_raw", 1, last_now_imageCallback);
    ros::spin();
    cv::destroyWindow("LastWindow");
    cv::destroyWindow("NowWindow");
    cv::destroyWindow("MatchWindow");
    return 0;
}

void last_now_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(frame_cnt == 0)
    {
        frame_cnt = 1;
        last_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::Ptr<cv::FeatureDetector> lastDetector = cv::FastFeatureDetector::create();
        // cv::Ptr<cv::DescriptorExtractor> lastDescriptor = cv::FastFeatureDetector::create();
        cv::Ptr<cv::ORB> lastDescriptor = cv::ORB::create();
        lastDetector->detect(last_image, last_keypoints);
        lastDescriptor->compute(last_image, last_keypoints, last_descriptors);
        return;
    }
    
    cv::Mat now_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    std::vector<cv::KeyPoint> now_keypoints;
    cv::Mat now_descriptors;
    cv::Mat match_image;
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::FeatureDetector> nowDetector = cv::FastFeatureDetector::create();
    // cv::Ptr<cv::DescriptorExtractor> nowDescriptor = cv::FastFeatureDetector::create();
    cv::Ptr<cv::ORB> nowDescriptor = cv::ORB::create();

    cv::BFMatcher nowMatcher;
  
    nowDetector->detect(now_image, now_keypoints);
    nowDescriptor->compute(now_image, now_keypoints, now_descriptors);
    nowMatcher.match(last_descriptors, now_descriptors, matches);
    cv::drawMatches(last_image, last_keypoints, now_image, now_keypoints, matches, match_image);
    // std::vector<cv::KeyPoint> last_kp, now_kp;
    // sort(matches.begin(), matches.end());
    // for(int i = 0; i < 50; i ++)
    // {
    //     cv::DMatch dmatches;
    //     dmathces
    // }
    cv::imshow("LastWindow", last_image);
    cv::imshow("NowWindow", now_image);
    cv::imshow("MatchWindow", match_image);
    vector<cv::Point2f> lastImagePt2f, nowImagePt2f;
    dmatchs2point2f(matches, last_keypoints, now_keypoints, lastImagePt2f, nowImagePt2f);
    cv::Mat R, t;
    Eigen::Matrix3d K;
//     static double fu_left = 701.1307622240085;
// static double fv_left = 703.1080314857786;
// static double pu_left = 378.0566680943181;
// static double pv_left = 188.15668023579335;
    K(0,0) = 701.1307622240085;
    K(0,1) = 0;
    K(0,2)= 378.0566680943181;
    K(1,0) = 0;
    K(1,1) = 703.1080314857786;
    K(1,2) = 188.15668023579335;
    K(2,0) = 0;
    K(2,1) = 0;
    K(2,2) = 1;
    poseEstimation_2d2d(lastImagePt2f, nowImagePt2f, R, t, K);
    cout<<endl<<"R Matrix = "<<R<<endl;

    last_image = now_image.clone();
    last_keypoints = now_keypoints;
    last_descriptors = now_descriptors.clone();
    return;
}

// RESULT_OF_PNP estimateMotion(vector<cv::KeyPoint> kp0, vector<cv::KeyPoint> kp1, 
// vector<cv::DMatch> dmatches, Camera camera_left)
// {

//     double camera_matrix[3][3] = {
//         camera_left.fx, 0, camera_left.cx,
//         0, camera_left.fy, camera_left.cy,
//         0, 0, 1
//     };
//     vector<cv::Point3f> pts_obj;
//     vector<cv::Point2f> pts_img;
//     cv::Mat rvec, tvec, inliers;
//     cv::Mat cameraMatrix(3,3, cv::CV_64F, camera_matrix);
//     cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);
//     RESULT_OF_PNP result;
//     result.rvec = rvec;
//     result.tvec = tvec;
//     result.inliers = inliers.rows;
//     return result;
// }

// cv::Point3f point2dTo3d(const cv::Point3f point, const s_camera camera_left)//5.17
// {
//     cv::Point3f p;
//     p.z = double(point.z)/camera_left.scale;
//     p.x = (point.x - camera_left.cx) * p.z/camera.fx;
//     p.y = (point.y - camera.cy) * p.z / camera.fy;
//     return p;
// }


//从大到小排序
bool comp(cv::DMatch match1, cv::DMatch match2)
{
    return match1.distance > match2.distance;
}


void dmatchs2point2f(const vector<cv::DMatch> dmatches, const vector<cv::KeyPoint> src, const vector<cv::KeyPoint> dst,
    vector<cv::Point2f> &src_pt2f, vector<cv::Point2f> &dst_pt2f)
{

    // sort(dmatches.begin(), dmatches.end(), comp);
    vector<cv::DMatch> goodMatches;
    for( int i = 0; i < 50; i++)
    {
        goodMatches.push_back(dmatches[i]);
    }

    for(int i = 0; i < goodMatches.size(); i++)
    {
        src_pt2f.push_back(src[goodMatches[i].queryIdx].pt);
        dst_pt2f.push_back(dst[goodMatches[i].trainIdx].pt);
    }
}

// void couple_grey_depth(const vector<cv::Point2d> grey, const vector<cv::Point2f> depth, vector<cv::Point3f> &gd)
// {

// }

void poseEstimation_2d2d(vector<cv::Point2f> points1, vector<cv::Point2f> points2, cv::Mat &R, cv::Mat &t, Eigen::Matrix3d K)
{
    cv::Mat fundamentalMatrix;
    // Eigen::Matrix3d fundamentalMatrix3d, essentialMatrix;
    // fundamentalMatrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);
    // for(int i = 0; i < 3; i ++)
    // {
    //     for(int j = 0; j < 3; j++)
    //     {
    //         fundamentalMatrix3d[i][j] = fundamentalMatrix.at<double>(i,j)
    //     }
    // }
    // essentialMatrix = K.transpose() * fundamentalMatrix3d * K.inverse();
    int focal_length =(int) K(1,1);
    cv::Point2d principal_point(K(0,2), K(1,2));
    cv::Mat essentialMatrixMat;
    essentialMatrixMat = cv::findEssentialMat(points1, points2, focal_length, principal_point, cv::RANSAC);
    //     for(int i = 0; i < 3; i ++)
    // {
    //     for(int j = 0; j < 3; j++)
    //     {
    //         essentialMatrixMat.at<double>(i,j) = essentialMatrix[i][j];
    //     }
    // }
    cv::recoverPose(essentialMatrixMat, points1, points2, R, t, focal_length, principal_point);
}
