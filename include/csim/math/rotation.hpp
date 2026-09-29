#pragma once

#include <csim/math/matrix.hpp>

#include <cmath>
#include <stdexcept>

namespace csim::math {

inline constexpr double rotationValidationTolerance = 1e-10;

// [v]_x * u = v cross u. Raw algebra follows the Vector3 floating-point policy.
constexpr Matrix3 skew(const Vector3& v) {
    return Matrix3{0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0};
}

// Right-handed rotations acting on column vectors. All angles are in radians.
inline Matrix3 rotationX(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("Rotation angle must be finite");
    }
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Matrix3{1, 0, 0, 0, c, -s, 0, s, c};
}

inline Matrix3 rotationY(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("Rotation angle must be finite");
    }
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Matrix3{c, 0, s, 0, 1, 0, -s, 0, c};
}

inline Matrix3 rotationZ(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("Rotation angle must be finite");
    }
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Matrix3{c, -s, 0, s, c, 0, 0, 0, 1};
}

inline Matrix3 rotationFromRollPitchYaw(double roll, double pitch, double yaw) {
    return rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
}

// Checks max-entry error in R^T R - I and absolute error in det(R) - 1.
// Invalid tolerance throws; an invalid matrix returns false.
inline bool isRotationMatrix(const Matrix3& matrix,
                             double tolerance = rotationValidationTolerance) {
    if (!std::isfinite(tolerance) || tolerance < 0.0 || tolerance >= 1.0) {
        throw std::invalid_argument("Rotation tolerance must be finite and in [0, 1)");
    }
    if (!matrix.isFinite()) {
        return false;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            // Valid rotation entries are bounded by one; also avoid overflow below.
            if (std::abs(matrix(i, j)) > 1.0 + tolerance) {
                return false;
            }
        }
    }
    const Matrix3 gram = matrix.transposed() * matrix;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            if (std::abs(gram(i, j) - (i == j ? 1.0 : 0.0)) > tolerance) {
                return false;
            }
        }
    }
    const double determinant =
        matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1))
        - matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0))
        + matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
    return std::abs(determinant - 1.0) <= tolerance;
}

} // namespace csim::math
