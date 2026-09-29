#pragma once

#include <csim/math/vector.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace csim::math {

// Fixed dimensions, row-major storage. Vectors are multiplied as columns.
template <std::size_t Rows, std::size_t Cols>
class Matrix {
    static_assert(Rows > 0 && Cols > 0, "Matrix dimensions must be positive");

public:
    static constexpr std::size_t rows = Rows;
    static constexpr std::size_t cols = Cols;

    constexpr Matrix() = default;

    // An explicit initializer list must supply every element, row by row.
    constexpr explicit Matrix(std::initializer_list<double> values) {
        if (values.size() != Rows * Cols) {
            throw std::invalid_argument("Matrix initializer must match its dimensions");
        }
        std::size_t index = 0;
        for (double value : values) {
            values_[index++] = value;
        }
    }

    constexpr double& operator()(std::size_t row, std::size_t col) {
        checkIndex(row, col);
        return values_[row * Cols + col];
    }

    constexpr const double& operator()(std::size_t row, std::size_t col) const {
        checkIndex(row, col);
        return values_[row * Cols + col];
    }

    static constexpr Matrix identity() {
        static_assert(Rows == Cols, "Identity requires a square matrix");
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i) {
            result(i, i) = 1.0;
        }
        return result;
    }

    constexpr Matrix<Cols, Rows> transposed() const {
        Matrix<Cols, Rows> result;
        for (std::size_t row = 0; row < Rows; ++row) {
            for (std::size_t col = 0; col < Cols; ++col) {
                result(col, row) = (*this)(row, col);
            }
        }
        return result;
    }

    constexpr Matrix& operator+=(const Matrix& other) noexcept {
        for (std::size_t i = 0; i < values_.size(); ++i) {
            values_[i] += other.values_[i];
        }
        return *this;
    }

    constexpr Matrix& operator-=(const Matrix& other) noexcept {
        for (std::size_t i = 0; i < values_.size(); ++i) {
            values_[i] -= other.values_[i];
        }
        return *this;
    }

    constexpr Matrix& operator*=(double scalar) noexcept {
        for (double& value : values_) {
            value *= scalar;
        }
        return *this;
    }

    Matrix& operator/=(double scalar) {
        if (scalar == 0.0) {
            throw std::domain_error("Matrix division by zero");
        }
        // Direct division avoids overflowing a tiny scalar's reciprocal.
        for (double& value : values_) {
            value /= scalar;
        }
        return *this;
    }

    bool isFinite() const {
        for (double value : values_) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return true;
    }

private:
    static constexpr void checkIndex(std::size_t row, std::size_t col) {
        if (row >= Rows || col >= Cols) {
            throw std::out_of_range("Matrix index out of range");
        }
    }

    std::array<double, Rows * Cols> values_{};
};

template <std::size_t Rows, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator+(Matrix<Rows, Cols> left,
                                      const Matrix<Rows, Cols>& right) noexcept {
    return left += right;
}

template <std::size_t Rows, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator-(Matrix<Rows, Cols> left,
                                      const Matrix<Rows, Cols>& right) noexcept {
    return left -= right;
}

template <std::size_t Rows, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator-(Matrix<Rows, Cols> value) noexcept {
    return value *= -1.0;
}

template <std::size_t Rows, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator*(Matrix<Rows, Cols> value, double scalar) noexcept {
    return value *= scalar;
}

template <std::size_t Rows, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator*(double scalar, Matrix<Rows, Cols> value) noexcept {
    return value *= scalar;
}

template <std::size_t Rows, std::size_t Cols>
Matrix<Rows, Cols> operator/(Matrix<Rows, Cols> value, double scalar) {
    return value /= scalar;
}

template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
constexpr Matrix<Rows, Cols> operator*(const Matrix<Rows, Inner>& left,
                                      const Matrix<Inner, Cols>& right) {
    Matrix<Rows, Cols> result;
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
            double value = 0.0;
            for (std::size_t k = 0; k < Inner; ++k) {
                value += left(row, k) * right(k, col);
            }
            result(row, col) = value;
        }
    }
    return result;
}

using Matrix3 = Matrix<3, 3>;

constexpr Vector3 operator*(const Matrix3& matrix, const Vector3& vector) {
    return {
        matrix(0, 0) * vector.x + matrix(0, 1) * vector.y + matrix(0, 2) * vector.z,
        matrix(1, 0) * vector.x + matrix(1, 1) * vector.y + matrix(1, 2) * vector.z,
        matrix(2, 0) * vector.x + matrix(2, 1) * vector.y + matrix(2, 2) * vector.z,
    };
}

} // namespace csim::math
