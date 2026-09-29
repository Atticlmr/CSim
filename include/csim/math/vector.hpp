#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace csim::math {

// Components must share a coordinate frame and compatible physical units.
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vector3& operator+=(const Vector3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Vector3& operator-=(const Vector3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vector3& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3& operator/=(double scalar) {
        if (scalar == 0.0) {
            throw std::domain_error("Vector3 division by zero");
        }
        // Divide directly: the reciprocal may overflow for tiny scalars.
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    constexpr double dot(const Vector3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    constexpr Vector3 cross(const Vector3& other) const noexcept {
        return {y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x};
    }

    // May overflow or underflow even when norm() is representable.
    constexpr double squaredNorm() const noexcept { return dot(*this); }

    double norm() const { return std::hypot(x, y, z); }

    bool isFinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    // Reject zero/non-finite inputs. No implicit tolerance for tiny vectors.
    Vector3 normalized() const {
        if (!isFinite()) {
            throw std::domain_error("Cannot normalize a non-finite Vector3");
        }
        const double scale = std::max({std::abs(x), std::abs(y), std::abs(z)});
        if (scale == 0.0) {
            throw std::domain_error("Cannot normalize a zero Vector3");
        }
        // Scaling also permits normalization when the original norm overflows.
        const Vector3 scaled{x / scale, y / scale, z / scale};
        const double length = scaled.norm();
        return {scaled.x / length, scaled.y / length, scaled.z / length};
    }
};

constexpr Vector3 operator+(Vector3 left, const Vector3& right) noexcept {
    return left += right;
}

constexpr Vector3 operator-(Vector3 left, const Vector3& right) noexcept {
    return left -= right;
}

constexpr Vector3 operator-(const Vector3& value) noexcept {
    return {-value.x, -value.y, -value.z};
}

constexpr Vector3 operator*(Vector3 value, double scalar) noexcept {
    return value *= scalar;
}

constexpr Vector3 operator*(double scalar, Vector3 value) noexcept {
    return value *= scalar;
}

inline Vector3 operator/(Vector3 value, double scalar) {
    return value /= scalar;
}

} // namespace csim::math
