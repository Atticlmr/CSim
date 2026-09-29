#pragma once

#include <csim/math/matrix.hpp>
#include <array>

namespace csim::viewer {

using Matrix4 = math::Matrix<4, 4>;
std::array<float, 16> columnMajor(const Matrix4& matrix);
// Z-up orbit camera. Matrix calculations stay in double until GPU upload.
class Camera {
public:
    void setViewport(int width, int height);
    int viewportWidth() const { return width_; }
    int viewportHeight() const { return height_; }
    float aspectRatio() const;
    math::Vector3 position() const;
    const math::Vector3& target() const { return target_; }
    void setTarget(math::Vector3 target);
    void orbit(double dx, double dy);
    void pan(double dx, double dy);
    void zoom(double wheel);
    void reset(math::Vector3 target = {0, 0, 3}, double distance = 11);
    Matrix4 view() const;
    Matrix4 projection() const;
    Matrix4 viewProjection() const { return projection() * view(); }
private:
    math::Vector3 target_{0, 0, 3};
    double yaw_ = -0.85, pitch_ = 0.4, distance_ = 11;
    int width_ = 1280, height_ = 720;
};

} // namespace csim::viewer
