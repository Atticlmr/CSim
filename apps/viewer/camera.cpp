#include "camera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace csim::viewer {

std::array<float, 16> columnMajor(const Matrix4& matrix) {
    std::array<float, 16> result{};
    for (std::size_t row=0; row<4; ++row)
        for (std::size_t col=0; col<4; ++col) {
            result[col*4+row] = static_cast<float>(matrix(row,col));
            if (!std::isfinite(result[col*4+row])) throw std::overflow_error("Camera matrix cannot be rendered");
        }
    return result;
}
void Camera::setViewport(int width, int height) { width_=std::max(width,0); height_=std::max(height,0); }
float Camera::aspectRatio() const {
    return width_ && height_ ? static_cast<float>(width_)/static_cast<float>(height_) : 1.0F;
}
math::Vector3 Camera::position() const {
    return target_ + math::Vector3{std::cos(pitch_)*std::cos(yaw_),
        std::cos(pitch_)*std::sin(yaw_), std::sin(pitch_)}*distance_;
}
void Camera::setTarget(math::Vector3 target) {
    if (!target.isFinite()) throw std::invalid_argument("Camera target must be finite");
    target_=target;
}
void Camera::reset(math::Vector3 target, double distance) {
    if (!std::isfinite(distance) || distance<=0) throw std::invalid_argument("Camera distance must be positive");
    setTarget(target); distance_=std::clamp(distance,0.5,500.0); yaw_=-0.85; pitch_=0.4;
}
void Camera::orbit(double dx, double dy) {
    if (!std::isfinite(dx) || !std::isfinite(dy)) return;
    yaw_=std::remainder(yaw_-std::clamp(dx,-2000.0,2000.0)*0.006,2*std::acos(-1.0));
    pitch_=std::clamp(pitch_+std::clamp(dy,-2000.0,2000.0)*0.006,-1.45,1.45);
}
void Camera::pan(double dx, double dy) {
    if (!std::isfinite(dx) || !std::isfinite(dy)) return;
    const auto forward=(target_-position()).normalized();
    const auto right=forward.cross({0,0,1}).normalized();
    const auto up=right.cross(forward);
    const double scale=distance_*0.0015;
    target_ += (right*(-std::clamp(dx,-2000.0,2000.0))+up*std::clamp(dy,-2000.0,2000.0))*scale;
}
void Camera::zoom(double wheel) {
    if (std::isfinite(wheel)) distance_=std::clamp(distance_*std::exp(-std::clamp(wheel,-20.0,20.0)*0.12),0.5,500.0);
}
Matrix4 Camera::view() const {
    const auto eye=position();
    const auto forward=(target_-eye).normalized();
    const auto right=forward.cross({0,0,1}).normalized();
    const auto up=right.cross(forward);
    return Matrix4{right.x,right.y,right.z,-right.dot(eye),
                   up.x,up.y,up.z,-up.dot(eye),
                   -forward.x,-forward.y,-forward.z,forward.dot(eye), 0,0,0,1};
}
Matrix4 Camera::projection() const {
    constexpr double near_plane=0.05, far_plane=2000;
    const double f=1/std::tan(std::acos(-1.0)/8);
    return Matrix4{f/aspectRatio(),0,0,0, 0,f,0,0,
        0,0,(far_plane+near_plane)/(near_plane-far_plane),2*far_plane*near_plane/(near_plane-far_plane),
        0,0,-1,0};
}

} // namespace csim::viewer
