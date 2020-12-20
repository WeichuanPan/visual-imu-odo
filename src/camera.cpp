#include "../include/myslam/camera.h"


Camera::Camera() {}
Eigen::Matrix3d Camera::getK()
{
    Eigen::Matrix3d K;
    K(0,0) = fx;
    K(0,1) = 0;
    K(0,2) = cx;
    K(1,0) = 0;
    K(1,1) = fy;
    K(1,2) = cy;
    K(2,0) = 0;
    K(2,1) = 0;
    K(2,2) = 1;
    return K;
}

Eigen::Vector3d Camera::getCameraFromWorld(const Eigen::Vector3d p_w)
{
    Eigen::Vector3d cameraP;
    cameraP = translationSe * p_w;
    return cameraP;
}

Eigen::Vector3d Camera::getWorldFromCamera(const Eigen::Vector3d p_c)
{
    return translationSe.inverse() * p_c;
}

Eigen::Vector3d Camera::getPixelFromCamera(const Eigen::Vector3d p_c)
{
    Eigen::Vector3d p_p;
    
    Eigen::Matrix3d K = getK();
    p_p = K * p_c;
    return p_p;
}

Eigen::Vector3d Camera::getCameraFromPixel(const Eigen::Vector3d p_p)
{
    Eigen::Vector3d p_c;
    Eigen::Matrix3d K = getK();
    p_c = K.inverse() * p_c;
    return p_c;
}
Eigen::Vector3d Camera::getWorldFromPixel(const Eigen::Vector3d p_p)
{
    Eigen::Vector3d w;
    Eigen::Vector3d c = getCameraFromPixel(p_p);
    w = getWorldFromCamera(c);
    return c;
}



// Eigen::Vector3d Camera::w2c(const Eigen::Vector3d& positionW, const Sophus::SE3& T_c_w)
// {
//     return T_c_w * positionW;
// }

// Eigen::Vector3d Camera::c2p(const Eigen::Vector3d& positionC) {
//      Eigen::Vector3d pixelPosition(
//         fx_ * positionC(0,0)/positionC(2,0) + cx_, 
//         fy_ * positionC(1,0)/positionC(2,0) + cy_,
//         1
//     );
//     return pixelPosition;
// }

// Eigen::Vector3d Camera::p2c(const Eigen::Vector3d & p_p, double depth)
// {
//     Eigen::Vector3d cameraPosition;
//     cameraPosition =  Eigen::Vector3d(
//         (p_p(0,0) - cx_) * depth/fx_,
//         (p_p(1,0) - cy_) * depth/fy_,
//         depth
//     );
//     return cameraPosition;

// }


// Eigen::Vector3d Camera::w2p(const Eigen::Vector3d& p_w, const Sophus::SE3& T_c_w)
// {
//     Eigen::Vector3d pixelPosition;
//     pixelPosition = c2p(w2c(p_w, T_c_w));
//     return pixelPosition;
// }

// Eigen::Vector3d Camera::p2w(const Eigen::Vector3d& p_p, const Sophus::SE3& T_c_w, double depth)
// {
//     return c2w(p2c(p_p, depth), T_c_w);
// }
