#pragma once

#include <csim/math/matrix.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace csim::math {

// Includes matrices rejected by the configured numerical pivot threshold.
class SingularMatrixError : public std::domain_error {
public:
    using std::domain_error::domain_error;
};

// Partial row pivoting: P * A = L * U, with a unit diagonal in L.
template <std::size_t N>
class PartialPivLU {
public:
    static constexpr double defaultTolerance() noexcept {
        return static_cast<double>(N) * std::numeric_limits<double>::epsilon();
    }

    explicit PartialPivLU(const Matrix<N, N>& matrix,
                          double relative_pivot_tolerance = defaultTolerance())
        : factors_(matrix), relative_pivot_tolerance_(relative_pivot_tolerance) {
        if (!std::isfinite(relative_pivot_tolerance_) ||
            relative_pivot_tolerance_ < 0.0 || relative_pivot_tolerance_ >= 1.0) {
            throw std::invalid_argument("LU relative pivot tolerance must be finite and in [0, 1)");
        }
        if (!matrix.isFinite()) {
            throw std::invalid_argument("LU matrix must contain only finite values");
        }
        for (std::size_t row = 0; row < N; ++row) {
            permutation_[row] = row;
            for (std::size_t col = 0; col < N; ++col) {
                matrix_scale_ = std::max(matrix_scale_, std::abs(matrix(row, col)));
            }
        }
        if (matrix_scale_ == 0.0) {
            throw SingularMatrixError("Cannot factor a zero matrix");
        }

        for (std::size_t col = 0; col < N; ++col) {
            std::size_t pivot_row = col;
            double pivot_size = std::abs(factors_(col, col));
            for (std::size_t row = col + 1; row < N; ++row) {
                const double candidate = std::abs(factors_(row, col));
                if (candidate > pivot_size) {
                    pivot_row = row;
                    pivot_size = candidate;
                }
            }
            // Comparing a ratio avoids underflow in tolerance * matrix_scale.
            // With tolerance zero, only an exactly zero pivot is rejected.
            if (pivot_size == 0.0 ||
                (relative_pivot_tolerance_ > 0.0 &&
                 pivot_size / matrix_scale_ <= relative_pivot_tolerance_)) {
                throw SingularMatrixError("LU pivot is zero or below the relative threshold");
            }
            if (pivot_row != col) {
                // Swap the complete packed row, including earlier L multipliers.
                for (std::size_t j = 0; j < N; ++j) {
                    std::swap(factors_(col, j), factors_(pivot_row, j));
                }
                std::swap(permutation_[col], permutation_[pivot_row]);
            }
            for (std::size_t row = col + 1; row < N; ++row) {
                factors_(row, col) /= factors_(col, col);
                for (std::size_t j = col + 1; j < N; ++j) {
                    factors_(row, j) = finiteResult(
                        factors_(row, j) - factors_(row, col) * factors_(col, j));
                }
            }
        }
    }

    Matrix<N, N> lower() const {
        auto result = Matrix<N, N>::identity();
        for (std::size_t row = 0; row < N; ++row) {
            for (std::size_t col = 0; col < row; ++col) {
                result(row, col) = factors_(row, col);
            }
        }
        return result;
    }

    Matrix<N, N> upper() const {
        Matrix<N, N> result;
        for (std::size_t row = 0; row < N; ++row) {
            for (std::size_t col = row; col < N; ++col) {
                result(row, col) = factors_(row, col);
            }
        }
        return result;
    }

    // The permuted row i comes from original row permutation()[i].
    const std::array<std::size_t, N>& permutation() const noexcept { return permutation_; }

    Matrix<N, N> permutationMatrix() const {
        Matrix<N, N> result;
        for (std::size_t row = 0; row < N; ++row) {
            result(row, permutation_[row]) = 1.0;
        }
        return result;
    }

    double matrixScale() const noexcept { return matrix_scale_; }
    double relativePivotTolerance() const noexcept { return relative_pivot_tolerance_; }

    template <std::size_t RhsCols>
    Matrix<N, RhsCols> solve(const Matrix<N, RhsCols>& rhs) const {
        if (!rhs.isFinite()) {
            throw std::invalid_argument("LU right-hand side must contain only finite values");
        }
        Matrix<N, RhsCols> solution;
        for (std::size_t col = 0; col < RhsCols; ++col) {
            // L * y = P * b; L has an implicit unit diagonal.
            for (std::size_t row = 0; row < N; ++row) {
                double value = rhs(permutation_[row], col);
                for (std::size_t j = 0; j < row; ++j) {
                    value -= factors_(row, j) * solution(j, col);
                }
                solution(row, col) = finiteResult(value);
            }
            // U * x = y. Count down without unsigned index underflow.
            for (std::size_t remaining = N; remaining > 0; --remaining) {
                const std::size_t row = remaining - 1;
                double value = solution(row, col);
                for (std::size_t j = row + 1; j < N; ++j) {
                    value -= factors_(row, j) * solution(j, col);
                }
                solution(row, col) = finiteResult(value / factors_(row, row));
            }
        }
        return solution;
    }

    template <std::size_t Size = N>
    std::enable_if_t<Size == 3 && Size == N, Vector3> solve(const Vector3& rhs) const {
        const auto result = solve(Matrix<3, 1>{rhs.x, rhs.y, rhs.z});
        return {result(0, 0), result(1, 0), result(2, 0)};
    }

private:
    static double finiteResult(double value) {
        if (!std::isfinite(value)) {
            throw std::overflow_error("LU arithmetic produced a non-finite result");
        }
        return value;
    }

    Matrix<N, N> factors_;
    std::array<std::size_t, N> permutation_{};
    double matrix_scale_ = 0.0;
    double relative_pivot_tolerance_;
};

} // namespace csim::math
