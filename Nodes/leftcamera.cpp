#include<../include/cameraNodes.h>

void leftImageCallback(const sensor_msgs::ImageConstPtr msg)
{
    cv::Mat leftImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leftCamera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("leftImage", 1, leftImageCallback);
    
}

