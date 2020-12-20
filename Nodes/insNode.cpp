#include "insNode.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "insNode");
    ros::NodeHandle nh;
    INS insRT(nh);
    insRT.rMatrix = Eigen::Matrix3d::Identity();
    insRT.rMatrixBack = Eigen::Matrix3d::Identity();
    insRT.showData();
    return 0;
}