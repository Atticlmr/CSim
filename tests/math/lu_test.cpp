#include <csim/math/lu.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace {

using csim::math::Matrix;
using csim::math::Matrix3;
using csim::math::PartialPivLU;
using csim::math::SingularMatrixError;
using csim::math::Vector3;

template <typename Solver, typename Rhs, typename = void>
struct CanSolve : std::false_type {};

template <typename Solver, typename Rhs>
struct CanSolve<Solver, Rhs, std::void_t<decltype(std::declval<Solver>().solve(std::declval<Rhs>()))>>
    : std::true_type {};

static_assert(CanSolve<PartialPivLU<3>, Vector3>::value);
static_assert(!CanSolve<PartialPivLU<2>, Vector3>::value);
static_assert(!CanSolve<PartialPivLU<3>, Matrix<2, 1>>::value);
static_assert(!std::is_constructible_v<PartialPivLU<3>, Matrix<3, 2>>);
static_assert(std::is_same_v<decltype(std::declval<PartialPivLU<3>>().solve(Matrix<3, 2>{})), Matrix<3, 2>>);

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const std::string& message,
          double relative_tolerance = 1e-12, double absolute_tolerance = 1e-14) {
    check(std::isfinite(actual) && std::isfinite(expected), message + ": non-finite value");
    const double tolerance = std::max(absolute_tolerance,
        relative_tolerance * std::max(std::abs(actual), std::abs(expected)));
    check(std::abs(actual - expected) <= tolerance, message);
}

template <std::size_t Rows, std::size_t Cols>
void nearMatrix(const Matrix<Rows, Cols>& actual, const Matrix<Rows, Cols>& expected,
                const std::string& message) {
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
            near(actual(row, col), expected(row, col), message);
        }
    }
}

template <typename Exception, typename Function>
void throws(Function function, const std::string& message) {
    try {
        function();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

// Independently accumulate a normwise scaled residual for each RHS column.
template <std::size_t N, std::size_t Cols>
void checkResidual(const Matrix<N, N>& a, const Matrix<N, Cols>& x,
                   const Matrix<N, Cols>& b) {
    long double matrix_norm = 0;
    for (std::size_t row = 0; row < N; ++row) {
        long double row_sum = 0;
        for (std::size_t j = 0; j < N; ++j) {
            row_sum += std::abs(static_cast<long double>(a(row, j)));
        }
        matrix_norm = std::max(matrix_norm, row_sum);
    }
    for (std::size_t col = 0; col < Cols; ++col) {
        long double residual = 0;
        long double x_norm = 0;
        long double b_norm = 0;
        for (std::size_t row = 0; row < N; ++row) {
            long double product = 0;
            for (std::size_t j = 0; j < N; ++j) {
                product += static_cast<long double>(a(row, j)) * x(j, col);
            }
            residual = std::max(residual, std::abs(product - b(row, col)));
            x_norm = std::max(x_norm, std::abs(static_cast<long double>(x(row, col))));
            b_norm = std::max(b_norm, std::abs(static_cast<long double>(b(row, col))));
        }
        const long double scale = matrix_norm * x_norm + b_norm;
        check(std::isfinite(residual) && std::isfinite(scale), "finite residual metric");
        check(scale == 0 ? residual == 0 : residual / scale <= 1e-14L,
              "scaled solve residual");
    }
}

void factorizationAndPermutation() {
    const Matrix3 a{0, 2, 1, 1, 1, 0, 2, 0, 1};
    const PartialPivLU<3> lu(a);
    check(lu.permutation() == std::array<std::size_t, 3>{2, 0, 1}, "non-self-inverse permutation");
    nearMatrix(lu.lower(), Matrix3{1, 0, 0, 0, 1, 0, 0.5, 0.5, 1}, "lower after two pivots");
    nearMatrix(lu.upper(), Matrix3{2, 0, 1, 0, 2, 1, 0, 0, -1}, "upper factors");
    nearMatrix(lu.permutationMatrix() * a, lu.lower() * lu.upper(), "P A = L U");
    nearMatrix(lu.permutationMatrix().transposed() * lu.permutationMatrix(), Matrix3::identity(),
               "permutation orthogonality");
    nearMatrix(a, Matrix3{0, 2, 1, 1, 1, 0, 2, 0, 1}, "factorization preserves input");

    const PartialPivLU<2> negative(Matrix<2, 2>{-3, 1, 1, 1});
    check(negative.permutation()[0] == 0, "pivot uses absolute magnitude");
    nearMatrix(negative.solve(Matrix<2, 1>{-2, 2}), Matrix<2, 1>{1, 1}, "negative pivot solve");
}

void knownAndMultipleRhs() {
    const Matrix3 a{0, 2, 1, 1, 1, 0, 2, 0, 1};
    const PartialPivLU<3> lu(a);
    const Matrix<3, 2> b{3, 4, 3, -1, 1, 2};
    const Matrix<3, 2> expected{1, -1, 2, 0, -1, 4};
    const auto x = lu.solve(b);
    nearMatrix(x, expected, "two right-hand sides");
    checkResidual(a, x, b);
    nearMatrix(b, Matrix<3, 2>{3, 4, 3, -1, 1, 2}, "solve preserves RHS");
    const auto vector = lu.solve(Vector3{3, 3, 1});
    near(vector.x, 1, "Vector3 x");
    near(vector.y, 2, "Vector3 y");
    near(vector.z, -1, "Vector3 z");
    nearMatrix(lu.solve(Matrix<3, 1>{4, -1, 2}), Matrix<3, 1>{-1, 0, 4}, "reuse factorization");
    nearMatrix(lu.solve(Matrix<3, 1>{}), Matrix<3, 1>{}, "zero RHS");
    checkResidual(a, lu.solve(Matrix<3, 1>{}), Matrix<3, 1>{});
    nearMatrix(lu.permutationMatrix() * a, lu.lower() * lu.upper(), "solve preserves factors");
}

template <std::size_t N>
void checkDenseSystem() {
    Matrix<N, N> a;
    Matrix<N, 2> expected;
    for (std::size_t row = 0; row < N; ++row) {
        expected(row, 0) = static_cast<double>(row) - 2.0;
        expected(row, 1) = 1.0 / static_cast<double>(row + 1);
        for (std::size_t col = 0; col < N; ++col) {
            a(row, col) = row == col ? static_cast<double>(N + 2)
                : static_cast<double>((row + 2 * col) % 3) - 1.0;
        }
    }
    for (std::size_t col = 0; col < N; ++col) {
        std::swap(a(0, col), a(N - 1, col));
    }
    const Matrix<N, 2> b = a * expected;
    const PartialPivLU<N> lu(a);
    nearMatrix(lu.solve(b), expected, "dense system with known solution");
    checkResidual(a, lu.solve(b), b);
    nearMatrix(lu.permutationMatrix() * a, lu.lower() * lu.upper(), "dense factor reconstruction");
}

void dimensionsAndDenseSystems() {
    nearMatrix(PartialPivLU<1>(Matrix<1, 1>{4}).solve(Matrix<1, 1>{8}),
               Matrix<1, 1>{2}, "one-dimensional solve");
    const Matrix<3, 2> b{1, 2, 3, 4, 5, 6};
    nearMatrix(PartialPivLU<3>(Matrix3::identity()).solve(b), b, "identity solve");
    checkDenseSystem<2>();
    checkDenseSystem<3>();
    checkDenseSystem<5>();
    checkDenseSystem<8>();
}

void scaleHandling() {
    const Matrix3 base{0, 2, 1, 1, 1, 0, 2, 0, 1};
    const Matrix<3, 1> rhs{3, 3, 1};
    for (double scale : {1e-200, -1e-200, 1.0, 1e200, -1e200}) {
        const auto a = base * scale;
        const auto b = rhs * scale;
        const PartialPivLU<3> lu(a);
        const auto x = lu.solve(b);
        nearMatrix(x, Matrix<3, 1>{1, 2, -1}, "uniform scaling");
        checkResidual(a, x, b);
        near(lu.matrixScale(), 2 * std::abs(scale), "matrix scale", 1e-12, 0);
    }
    const double minimum = std::numeric_limits<double>::denorm_min();
    if (minimum > 0) {
        const Matrix<2, 2> a{0, minimum, minimum, 0};
        nearMatrix(PartialPivLU<2>(a).solve(Matrix<2, 1>{-minimum, minimum}),
                   Matrix<2, 1>{1, -1}, "subnormal pivots without reciprocal overflow");
    }
    const double maximum = std::numeric_limits<double>::max();
    nearMatrix(PartialPivLU<1>(Matrix<1, 1>{maximum}).solve(Matrix<1, 1>{maximum}),
               Matrix<1, 1>{1}, "largest finite scalar equation");
}

void singularMatricesAndTolerance() {
    throws<SingularMatrixError>([] { (void)PartialPivLU<2>(Matrix<2, 2>{}); }, "zero matrix rejected");
    throws<SingularMatrixError>([] { (void)PartialPivLU<2>(Matrix<2, 2>{0, 1, 0, 2}); },
                                 "zero pivot column rejected");
    throws<SingularMatrixError>([] { (void)PartialPivLU<2>(Matrix<2, 2>{1, 2, 2, 4}); },
                                 "dependent rows rejected");
    throws<SingularMatrixError>([] { (void)PartialPivLU<3>(Matrix3{1, 0, 2, 0, 1, 3, 1, 1, 5}); },
                                 "late zero pivot rejected");
    const double delta = 2 * std::numeric_limits<double>::epsilon();
    const Matrix<2, 2> nearly_singular{1, 1, 1, 1 + delta};
    throws<SingularMatrixError>([&] { (void)PartialPivLU<2>(nearly_singular); },
                                 "default relative pivot check");
    const PartialPivLU<2> exact_only(nearly_singular, 0.0);
    const Matrix<2, 1> b{2, 2 + delta};
    nearMatrix(exact_only.solve(b), Matrix<2, 1>{1, 1}, "explicit zero threshold");
    checkResidual(nearly_singular, exact_only.solve(b), b);
    const Matrix<2, 2> unequal{1, 0, 0, 1e-18};
    throws<SingularMatrixError>([&] { (void)PartialPivLU<2>(unequal); }, "scale disparity policy");
    nearMatrix(PartialPivLU<2>(unequal, 0.0).solve(Matrix<2, 1>{2, 3e-18}),
               Matrix<2, 1>{2, 3}, "explicit policy override");
    throws<SingularMatrixError>([] {
        (void)PartialPivLU<2>(Matrix<2, 2>{1, 0, 0, 1e-7}, 1e-6);
    }, "custom pivot threshold");
    throws<SingularMatrixError>([] { (void)PartialPivLU<1>(Matrix<1, 1>{0}, 0.0); },
                                 "zero threshold still rejects exact zero");
}

void invalidInputs() {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double tolerance : {-1.0, 1.0, 2.0, inf, nan}) {
        throws<std::invalid_argument>([&] { (void)PartialPivLU<2>(Matrix<2, 2>::identity(), tolerance); },
                                      "invalid tolerance rejected");
    }
    const PartialPivLU<3> lu(Matrix3::identity());
    near(lu.relativePivotTolerance(), PartialPivLU<3>::defaultTolerance(), "default tolerance", 0, 0);
    for (double invalid : {inf, -inf, nan}) {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 3; ++col) {
                auto a = Matrix3::identity();
                a(row, col) = invalid;
                throws<std::invalid_argument>([&] { (void)PartialPivLU<3>(a); }, "non-finite matrix rejected");
            }
            for (std::size_t col = 0; col < 2; ++col) {
                Matrix<3, 2> b;
                b(row, col) = invalid;
                throws<std::invalid_argument>([&] { (void)lu.solve(b); }, "non-finite RHS rejected");
            }
        }
        throws<std::invalid_argument>([&] { (void)lu.solve(Vector3{1, invalid, 2}); },
                                      "non-finite Vector3 RHS rejected");
    }
    nearMatrix(lu.solve(Matrix<3, 1>{1, 2, 3}), Matrix<3, 1>{1, 2, 3}, "reuse after rejected RHS");
}

void arithmeticOverflow() {
    const double maximum = std::numeric_limits<double>::max();
    throws<std::overflow_error>([&] { (void)PartialPivLU<2>(Matrix<2, 2>{maximum, maximum, -maximum, maximum}); },
                                "factor growth overflow reported");
    const PartialPivLU<1> tiny(Matrix<1, 1>{1e-200});
    throws<std::overflow_error>([&] { (void)tiny.solve(Matrix<1, 1>{maximum}); }, "division overflow reported");
    const PartialPivLU<2> lower(Matrix<2, 2>{1, 0, -1, 1});
    throws<std::overflow_error>([&] { (void)lower.solve(Matrix<2, 1>{maximum, maximum}); },
                                "forward substitution overflow reported");
    const PartialPivLU<2> upper(Matrix<2, 2>{1, 1, 0, 1});
    throws<std::overflow_error>([&] { (void)upper.solve(Matrix<2, 1>{maximum, -maximum}); },
                                "back substitution overflow reported");
}

} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"factorization and permutation", factorizationAndPermutation},
        {"known and multiple RHS", knownAndMultipleRhs},
        {"dimensions and dense systems", dimensionsAndDenseSystems},
        {"scale handling", scaleHandling},
        {"singular matrices and tolerance", singularMatricesAndTolerance},
        {"invalid inputs", invalidInputs},
        {"arithmetic overflow", arithmeticOverflow},
    };
    int failures = 0;
    for (const auto& test : tests) {
        try {
            test.run();
            std::cout << "PASS: " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n';
        }
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
