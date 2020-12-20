#include "../include/frame.h"

void systemParam::systemParam(ros::NodeHandle nh)
{

    vector<double> transMatrixCam0ToImu;
    xmlRpc::XmlRpcValue paramList;

    nh.getParam("cam0/T_cm_imu", paramList);
    for(int i = 0; i < paramList.size(); i++)
    {
        XmlRpc::XmlRpcValue tmpValue = paramList[i];
        if(tmpValue.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        dT_Cam0_2_Imu[i] = double(tmpValue);
    }
    vector<double> cam0Intrinsics;
    vector<int> cam0Resolution;
    vector<double> cam0DistortionCoeffs;

    nh.getParam(cam0 + "/distortion_coeffs", cam0DistortionCoeffs);
    nh.getParam(cam0 + "/intrinsics", cam0Intrinsics);
    nh.getParam(cam0 + "/resolution", cam0Resolution);
    nh.getParam(cam0 + "/timeshift_cam_imu", cam0TimeshiftCam2Imu);


    vector<double> cam1Intrinsics;
    vector<int> cam1Resolution;
    vector<double> cam1DistortionCoeffs;
    vector<double> transMatrixCam1ToImu;
    Xml::XmlRpcValue paramListRight;

    nh.getParam(cam1 + "/T_cm_imu", paramListRight);

    for(int i = 0; i < paramListRight.size(); i++)
    {
        XmlRpc::XmlRpcValue tmpValueRight = paramListRight[i];
        if(tmpValueRight.getType() == XmlRpcValue::TypeDouble)
        {
            transMatrixCam1ToImu.push_back(double(tmpValueRight));
        }
    }
    nh.getParam(cam1 + "/distortion_coeffs", cam1DistortionCoeffs);
    nh.getParam(cam1 + "/intrinsics", cam1Intrinsics);
    nh.getParam(cam1 + "/resolution", cam1Resolution);
    nh.getParam(cam1 + "/timeshift_cam_imu", cam1TimeshiftCam2Imu);


    nh.getParam(imu + "/Gyr/avg-axis/gyr_n", gyroNoise);
    nh.getParam(imu+ "/Gyr/avg-axis/gyr_w", gyroRandom);
    nh.getParam(imu+"/Acc/avg-axis/acc_n", accNoise);
    nh.getParam(imu + "/Acc/avg-axis/acc_w", accRandom);
}
