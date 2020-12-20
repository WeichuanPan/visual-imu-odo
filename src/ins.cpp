#include "ins.h"
#include<ros/ros.h>
using namespace std;
#define PI 3.1415926
INS::INS(ros::NodeHandle nh)
{
    this->nh = nh;
}

void INS::showData()
{
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("/mynteye/imu/data_raw",1000, &INS::INSNodeHandleCallback, this);
    
    // cout<<rMatrix<<endl;
    ros::spin();
    return;
}

void INS::INSNodeHandleCallback(const sensor_msgs::Imu::ConstPtr &msgs)
{
    sensor_msgs::Imu imuData;
    imuData = *msgs;
    this->deltaTime = imuData.header.stamp.sec + imuData.header.stamp.nsec*1e-9 - this->timestamp;
    this->timestamp = imuData.header.stamp.sec + imuData.header.stamp.nsec*1e-9;
    // this->quaternion.x = imuData.orientation.x;
    // this->quaternion.y = imuData.orientation.y;
    // this->quaternion.z = imuData.orientation.z;
    // this->quaternion.w = imuData.orientation.w;
    // this->imuData.timestamp = imuData.header.timestamp;
    this->imu_raw.gyroData(0) = (double)imuData.angular_velocity.x;
    this->imu_raw.gyroData(1) = (double)imuData.angular_velocity.y;
    this->imu_raw.gyroData(2) = (double)imuData.angular_velocity.z;
    this->imu_raw.accData(0) = (double)imuData.linear_acceleration.x;
    this->imu_raw.accData(1) = (double)imuData.linear_acceleration.y;
    this->imu_raw.accData(2) = (double)imuData.linear_acceleration.z;
    this->poseUpdate();
    this->velocityUpdate();
    this->positionUpdate();
    // cout<<"rMatrix:"<<this->rMatrix<<endl;
    // this->imu_raw.acceleratorData = imuData.linear_acceleration;
    // printf("\n%ld\n", imuData.header.stamp);
    // cout<<imuData.header.stamp<<endl;
    return;
}

void INS::poseUpdate()
{
    this->rMatrixBack = this->rMatrix;
    Eigen::Quaterniond qOld(this->rMatrixBack);
    // this->querternion = Eigen::Quterniond(this->rMatrixBack);
    Eigen::Vector4d q(qOld.w(), qOld.x(), qOld.y(),qOld.z());
    double deltaTheta =pow(this->imu_raw.gyroData(0) * this->deltaTime,2) + pow(this->imu_raw.gyroData(1) * this->deltaTime,2) + \
    pow(this->imu_raw.gyroData(2) * this->deltaTime,2);
    //二阶四元数更新
    q(0) = q(0)*(1.0 - deltaTheta/8.0) + 0.5 * (-q(1) *this->imu_raw.gyroData(0) - q(2) * this->imu_raw.gyroData(1) - q(3) * this->imu_raw.gyroData(2));
    q(1) = q(1)*(1.0 - deltaTheta/8.0) + 0.5 * (q(0) *this->imu_raw.gyroData(0) + q(2) * this->imu_raw.gyroData(2) - q(3) * this->imu_raw.gyroData(1));
    q(2) = q(2)*(1.0 - deltaTheta/8.0) + 0.5 * (q(0) *this->imu_raw.gyroData(1) -q(1) * this->imu_raw.gyroData(2) + q(3) * this->imu_raw.gyroData(0));
    q(3) = q(3)*(1.0 - deltaTheta/8.0) + 0.5 * (q(0) *this->imu_raw.gyroData(2) + q(1) * this->imu_raw.gyroData(1) -q(2) * this->imu_raw.gyroData(0));
    Eigen::Quaterniond nowQ(q(0), q(1),q(2),q(3));
    this->rMatrix = nowQ.toRotationMatrix();

    // //速度更新
    // Eigen::Matrix3d midRotationMatrix = (this->rMatrixBack + this->rMatrix) * 0.5;
    // Eigen::Omega
    // Eigen::Vector3d velocity;
    // for(int i = 0; i ++; i < 3)
    // {

    // }
    // Eigen::Matrix3d skewSymMatrix = Eigen::Matrix3d::Zero();
    // skewSymMatrix(0,1) = (double)-imu_raw.gyroData(2);
    // skewSymMatrix(0,2) = (double)imu_raw.gyroData(1);
    // skewSymMatrix(1,0) = (double)imu_raw.gyroData(2);
    // skewSymMatrix(1,2) = (double)-imu_raw.gyroData(0);
    // skewSymMatrix(2,0) = (double)-imu_raw.gyroData(1);
    // skewSymMatrix(2,1) = (double)imu_raw.gyroData(0);

    // Eigen::Vector3d omegaIeN;
    // double lat = 31.347584 * PI/180;
    // double lon = 121.481363 * PI / 180;
    // omegaIeN(0) = 0.0;
    // omegaIeN(1) = 7.29e-5* cos(lat);
    // omegaIeN(2) = 7.29e-5*sin(lat);
    // // this->rMatrix = (Eigen::Matrix3d::Identity() + skewSymMatrix* this->deltaTime) * this->rMatrixBack;
    // cout<<"\n Q rotationMatrix:\t" <<nowRmatrix<<"\n rMatrix:\t"<<this->rMatrix<<endl;
    return;
}

void INS::velocityUpdate()
{
    this->velocityBack = this->velocity;
    Eigen::Matrix3d midRotationMatrix = (this->rMatrixBack + this->rMatrix) * 0.5;
    Eigen::Vector3d omegaIE_N;
    double lat = 31.347584 * PI/180;
    double lon = 121.481363 * PI / 180;
    omegaIE_N(0) = 0.0;
    omegaIE_N(1) = 7.29e-5* cos(lat);
    omegaIE_N(2) = 7.29e-5*sin(lat);
    Eigen::Vector3d omegaEN_N;
    omegaEN_N(0) = -this->position(1) / (RM + this->position(2));
    omegaEN_N(1) = this->position(0) / (RN + this->position(2));
    omegaEN_N(2) = this->position(0) /(RN + this->position(2)) * tan(lat);
    Eigen::Vector3d velocityDot;
    Eigen::Vector3d vec_IN;
    vec_IN = 2.0 * omegaIE_N + omegaEN_N;
    Eigen::Matrix3d skewMatrix = skewSymMatrix(vec_IN);
    Eigen::Vector3d velocity;
    velocityDot = this->rMarix * this->imuRaw.accData - skewMatrix *  this->velocity + volicityG;
    this->velocity = this->velocity + velocityDot * (this->deltaTime);
}
void INS::positionUpdate()
{
    this->positionBack = this->position;
    this->position = this->position + 0.5 * (this->velocityBack + this->velocity) * this->deltaTime;
}
Eigen::Matrix3d skewSymMatrix(Eigen::Vector3d v3d)
{
    Eigen::Matrix3d skewSymMatrix = Eigen::Matrix3d::Zero();
    skewSymMatrix(0,1) = (double)-imu_raw.gyroData(2);
    skewSymMatrix(0,2) = (double)imu_raw.gyroData(1);
    skewSymMatrix(1,0) = (double)imu_raw.gyroData(2);
    skewSymMatrix(1,2) = (double)-imu_raw.gyroData(0);
    skewSymMatrix(2,0) = (double)-imu_raw.gyroData(1);
    skewSymMatrix(2,1) = (double)imu_raw.gyroData(0);
    return skewSymMatrix;
}
    