#pragma once

#include <csim/math/rotation.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace csim::math {

// Hamilton quaternion (w, x, y, z). Raw arithmetic does not enforce unit length.
// For attitude q_WB, rotate(v_B) gives v_W. Default construction is identity.
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static constexpr Quaternion identity() noexcept { return {}; }

    constexpr Quaternion& operator+=(const Quaternion& other) noexcept {
        w += other.w;
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Quaternion& operator-=(const Quaternion& other) noexcept {
        w -= other.w;
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Quaternion& operator*=(double scalar) noexcept {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Quaternion& operator/=(double scalar) {
        if (scalar == 0.0) {
            throw std::domain_error("Quaternion division by zero");
        }
        w /= scalar;
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    constexpr double dot(const Quaternion& other) const noexcept {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    constexpr double squaredNorm() const noexcept { return dot(*this); }

    double norm() const { return std::hypot(std::hypot(w, x), std::hypot(y, z)); }

    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    constexpr Quaternion conjugated() const noexcept { return {w, -x, -y, -z}; }

    Quaternion normalized() const {
        const double scale = normalizationScale();
        const Quaternion scaled{w / scale, x / scale, y / scale, z / scale};
        const double length = scaled.norm();
        return {scaled.w / length, scaled.x / length, scaled.y / length, scaled.z / length};
    }

    // General inverse, not just the conjugate of a unit quaternion.
    Quaternion inverse() const {
        const double scale = normalizationScale();
        const Quaternion scaled{w / scale, x / scale, y / scale, z / scale};
        const double squared = scaled.squaredNorm();
        const Quaternion result{(scaled.w / squared) / scale, (-scaled.x / squared) / scale,
                                (-scaled.y / squared) / scale, (-scaled.z / squared) / scale};
        if (!result.isFinite()) {
            throw std::overflow_error("Quaternion inverse is not representable");
        }
        return result;
    }

    static Quaternion fromAxisAngle(const Vector3& axis, double angle) {
        if (!std::isfinite(angle)) {
            throw std::invalid_argument("Rotation angle must be finite");
        }
        const Vector3 unit = axis.normalized();
        const double s = std::sin(0.5 * angle);
        return Quaternion{std::cos(0.5 * angle), unit.x * s, unit.y * s, unit.z * s}.normalized();
    }

    static Quaternion fromRollPitchYaw(double roll, double pitch, double yaw);

    static Quaternion fromRotationMatrix(const Matrix3& matrix,
                                         double tolerance = rotationValidationTolerance) {
        if (!isRotationMatrix(matrix, tolerance)) {
            throw std::invalid_argument("Quaternion requires a proper rotation matrix");
        }
        // Four candidates for 4*w*w, 4*x*x, 4*y*y, 4*z*z. Select the largest
        // component to avoid dividing by a near-zero scalar at half-turns.
        const std::array<double, 4> candidates{
            1.0 + matrix(0, 0) + matrix(1, 1) + matrix(2, 2),
            1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2),
            1.0 - matrix(0, 0) + matrix(1, 1) - matrix(2, 2),
            1.0 - matrix(0, 0) - matrix(1, 1) + matrix(2, 2)};
        const auto largest = std::max_element(candidates.begin(), candidates.end());
        const auto index = static_cast<std::size_t>(largest - candidates.begin());
        const double component = 0.5 * std::sqrt(*largest);
        const double divisor = 4.0 * component;
        Quaternion result;
        if (index == 0) {
            result = {component, (matrix(2, 1) - matrix(1, 2)) / divisor,
                      (matrix(0, 2) - matrix(2, 0)) / divisor,
                      (matrix(1, 0) - matrix(0, 1)) / divisor};
        } else if (index == 1) {
            result = {(matrix(2, 1) - matrix(1, 2)) / divisor, component,
                      (matrix(0, 1) + matrix(1, 0)) / divisor,
                      (matrix(0, 2) + matrix(2, 0)) / divisor};
        } else if (index == 2) {
            result = {(matrix(0, 2) - matrix(2, 0)) / divisor,
                      (matrix(0, 1) + matrix(1, 0)) / divisor, component,
                      (matrix(1, 2) + matrix(2, 1)) / divisor};
        } else {
            result = {(matrix(1, 0) - matrix(0, 1)) / divisor,
                      (matrix(0, 2) + matrix(2, 0)) / divisor,
                      (matrix(1, 2) + matrix(2, 1)) / divisor, component};
        }
        return result.normalized();
    }

    // Rotation queries normalize a copy; stored components are never changed.
    Matrix3 toRotationMatrix() const {
        const Quaternion q = normalized();
        return Matrix3{
            1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y),
            2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x),
            2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)};
    }

    Vector3 rotate(const Vector3& vector) const {
        const Matrix3 rotation = toRotationMatrix();
        if (!vector.isFinite()) {
            throw std::invalid_argument("Rotated vector must be finite");
        }
        const double scale = std::max({std::abs(vector.x), std::abs(vector.y), std::abs(vector.z)});
        if (scale == 0.0) {
            return {};
        }
        const Vector3 result = (rotation * (vector / scale)) * scale;
        if (!result.isFinite()) {
            throw std::overflow_error("Rotated vector is not representable");
        }
        return result;
    }

    // dq_WB/dt = 0.5 * q_WB * (0, Omega_B). No normalization of q or dq.
    Quaternion derivativeBodyRate(const Vector3& angular_velocity_B) const;

private:
    double normalizationScale() const {
        if (!isFinite()) {
            throw std::domain_error("Quaternion must be finite and nonzero");
        }
        const double scale = std::max({std::abs(w), std::abs(x), std::abs(y), std::abs(z)});
        if (scale == 0.0) {
            throw std::domain_error("Quaternion must be finite and nonzero");
        }
        return scale;
    }
};

constexpr Quaternion operator+(Quaternion left, const Quaternion& right) noexcept {
    return left += right;
}

constexpr Quaternion operator-(Quaternion left, const Quaternion& right) noexcept {
    return left -= right;
}

constexpr Quaternion operator-(const Quaternion& q) noexcept { return {-q.w, -q.x, -q.y, -q.z}; }

constexpr Quaternion operator*(Quaternion value, double scalar) noexcept { return value *= scalar; }

constexpr Quaternion operator*(double scalar, Quaternion value) noexcept { return value *= scalar; }

inline Quaternion operator/(Quaternion value, double scalar) { return value /= scalar; }

// q_WA * q_AB: apply q_AB first, then q_WA. This product is not commutative.
constexpr Quaternion operator*(const Quaternion& a, const Quaternion& b) noexcept {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quaternion Quaternion::fromRollPitchYaw(double roll, double pitch, double yaw) {
    return (fromAxisAngle({0, 0, 1}, yaw) * fromAxisAngle({0, 1, 0}, pitch)
            * fromAxisAngle({1, 0, 0}, roll)).normalized();
}

inline Quaternion Quaternion::derivativeBodyRate(const Vector3& angular_velocity_B) const {
    if (!isFinite() || !angular_velocity_B.isFinite()) {
        throw std::invalid_argument("Quaternion and body angular velocity must be finite");
    }
    const Quaternion result = (0.5 * *this) * Quaternion{0, angular_velocity_B.x,
                                                       angular_velocity_B.y, angular_velocity_B.z};
    if (!result.isFinite()) {
        throw std::overflow_error("Quaternion derivative is not representable");
    }
    return result;
}

} // namespace csim::math
