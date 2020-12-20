#ifndef CAMERA_H
#define CAMERA_H
#include  "base_include.h"
class Camera
{
    public:
    float fx, fy, cx, cy, z, k1, k2, p1, p2, xi;
    float roll, pitch, yaw;
    Eigen::Matrix3d rotationMatrix;
    Eigen::Matrix4d translationMatrix;
    Sophus::SE3 translationSe;
    Camera();
    Camera(float fx, float fy, float cx, float cy, float xi, float dist_k1, float dist_k2, float dist_p1, float dist_p2, float depth1 = 0.0):
    fx(fx), fy(fy), cx(cx), cy(cy), xi(xi), z(depth1), k1(dist_k1), k2(dist_k2), p1(dist_p1), p2(dist_p2) {};
    virtual Eigen::Matrix3d getK();
    virtual Eigen::Matrix3d get_Kmatrix_inv() const;
    Eigen::Vector3d getCameraFromWorld(const Eigen::Vector3d p_w);
    Eigen::Vector3d getWorldFromCamera(const Eigen::Vector3d p_c);
    Eigen::Vector3d getPixelFromCamera(const Eigen::Vector3d p_c);
    Eigen::Vector3d getCameraFromPixel(const Eigen::Vector3d p_p);
    Eigen::Vector3d getWorldFromPixel(const Eigen::Vector3d p_p);
};

Eigen::Matrix3d Camera::getK() 
{
    rotationMatrix(0,0) = fx;
    rotationMatrix(0,1) = 0;
    rotationMatrix(0,2) = cx;
    rotationMatrix(1,0) = 0;
    rotationMatrix(1,1) = fy;
    rotationMatrix(1,2) = cy;
    rotationMatrix(2,0) = 0;
    rotationMatrix(2,1) = 0;
    rotationMatrix(2,2) = 1;
}

Eigen::Matrix3d Camera::get_Kmatrix_inv() const
{
    Eigen::Matrix3d inv_matrix;
    inv_matrix = rotationMatrix.inverse();
    return inv_matrix;
}
//omnidirectional camera model: intrinsics vector:[xi, fu, fv, pu, pv], fu, fv:focal-length, pu, pv->principal point
// xi: mirror parameter
//distortion models:distortion_coeffs:[k1,k2,r1,r2]
class Mynteye:public Camera
{
    public:
        double mynt_fx, mynt_fy, mynt_cx, mynt_cy, mynt_depth, mynt_dist_k1, mynt_dist_k2, mynt_dist_r1,\
        mynt_dist_r2;
        float mynt_roll, mynt_pitch, mynt_yaw;
    Mynteye(double mynt_fx, double mynt_fy, double mynt_cx, double mynt_cy, double mynt_depth, double mynt_dist_k1,\
    double mynt_dist_k2, double mynt_dist_r1, double mynt_dist_r2): mynt_fx(mynt_fx), mynt_fy(mynt_fy), \
    mynt_cx(mynt_cx), mynt_cy(mynt_cy), mynt_depth(mynt_depth), mynt_dist_k1(mynt_dist_k1), mynt_dist_k2(mynt_dist_k2),\
     mynt_dist_r1(mynt_dist_r1), mynt_dist_r2(mynt_dist_r2) {};
    virtual Eigen::Matrix3d getK() const;
    cv::Mat Eigen2Mat(Eigen::Matrix3d k_matrix) const;
};

Eigen::Matrix3d Mynteye::getK() const
{
    Eigen::Matrix3d kMatrix;
        kMatrix(0,0) = mynt_fx;
        kMatrix(0,1) = 0;
        kMatrix(0,2) = mynt_cx;
        kMatrix(1,0) = 0;
        kMatrix(1,1) = mynt_fy;
        kMatrix(1,2) = mynt_cy;
        kMatrix(2,0) = 0;
        kMatrix(2,1) = 0;
        kMatrix(2,2) = 1;
        return kMatrix;
    //     K(0,0) = fx;
    // K(0,1) = 0;
    // K(0,2) = cx;
    // K(1,0) = 0;
    // K(1,1) = fy;
    // K(1,2) = cy;
    // K(2,0) = 0;
    // K(2,1) = 0;
    // K(2,2) = 1;
}
cv::Mat Mynteye::Eigen2Mat(Eigen::Matrix3d k_matrix) const
{
    // cv::Mat  kMatrix(cv::Mat_<double>(3,3), k_matrix(0,0), k_matrix(0,1), k_matrix(0,2), k_matrix(1,0), k_matrix(1,1), k_matrix(1,2),\
    // k_matrix(2,0), k_matrix(2,1), k_matrix(2,2));
    cv::Mat_<double> kMatrix(3,3);
    kMatrix(0,0) = k_matrix(0,0);
    kMatrix(0,1) = k_matrix(0,1);
    kMatrix(0,2) = k_matrix(0,2);
    kMatrix(1,0) = k_matrix(1,0);
    kMatrix(1,1) = k_matrix(1,1);
    kMatrix(1,2) = k_matrix(1,2);
    kMatrix(2,0) = k_matrix(2,0);
    kMatrix(2,1) = k_matrix(2,1);
    kMatrix(2,2) = k_matrix(2,2);
    return kMatrix;
    // (k_matrix(0,0), k_matrix(0,1), k_matrix(0,2), k_matrix(1,0), k_matrix(1,1), k_matrix(1,2),\
    // k_matrix(2,0), k_matrix(2,1), k_matrix(2,2));
}

#endif

// class Camera
// {
//     public:
//         typedef std::shared_ptr<Camera> Ptr;
//         float fx_, fy_, cx_, cy_, depth_scale_;

//         Camera();
//         Camera(float fx, float fy, float cx, float cy, float depth_scale = 0):fx_(fx), fy_(fy),cx_(cx),cy_(cy),depth_scale_(depth_scale) {}
//         Eigen::Vector3d w2c(const Eigen::Vector3d & p_w, const Sophus::SE3& T_c_w);
//         Eigen::Vector3d c2w(const Eigen::Vector3d & p_c, const Sophus::SE3& T_c_w);
//         Eigen::Vector3d c2p(const Eigen::Vector3d& p_c);
//         Eigen::Vector3d p2c(const Eigen::Vector3d& p_p, double depth=1);
//         Eigen::Vector3d p2w(const Eigen::Vector3d& p_p, const Sophus::SE3& T_c_w, double depth = 1);
//         Eigen::Vector3d w2p(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w);
// }