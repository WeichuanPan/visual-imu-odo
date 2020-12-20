#ifndef INS_H
#define INS_H
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/Dense>
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<iostream>
using namespace std;
#define GRAVITY_ACCELERATION 9.81

typedef struct
{
    double timestamp;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyroData;
    Eigen::Vector3d accData;
    // geometry_msgs::Vector3 gyroData;
    // geometry_msgs::Vector3 acceleratorData;
} imuRawData;


class INS {
    public:
        INS();
        ~INS() {};
        INS(ros::NodeHandle nh);
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // imuRawData imuRaw;
        imuRawData imu_raw;
        double timestamp;
        double timestampBack;
        double deltaTime;
        ros::NodeHandle nh;
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rMatrixBack;
        Eigen::Matrix3d rMatrix;
        Eigen::Vector3d positionBack;
        Eigen::Vector3d position;
        Eigen::Vector3d velocityBack;
        Eigen::Vector3d velocity;
    public:
        void showData();
        void INSNodeHandleCallback(const sensor_msgs::Imu::ConstPtr &msgs);
        void poseUpdate();
        void velocityUpdate();
    // ins():id(0),time(0), orientation(Eigen::Vector4d(0, 0, 0, 1)),
    // position(Eigen::Vector3d::Zero()),
    // velocity(Eigen::Vector3d::Zero()),
    // gyro_bias(Eigen::Vector3d::Zero()),
    // acc_bias(Eigen::Vector3d::Zero()) {}

    //  ins(const LLI& new_id): id(new_id), time(0),
    // orientation(Eigen::Vector4d(0, 0, 0, 1)),
    // position(Eigen::Vector3d::Zero()),
    // velocity(Eigen::Vector3d::Zero()),
    // gyro_bias(Eigen::Vector3d::Zero()),
    // acc_bias(Eigen::Vector3d::Zero()) {}
    
};
Eigen::Matrix3d skewSymMatrix(Eigen::Vector3d v3d);
#endif