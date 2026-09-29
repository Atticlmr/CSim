#include <csim/math/matrix.hpp>

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
using csim::math::Vector3;

template <typename Left, typename Right, typename = void>
struct CanMultiply : std::false_type {};

template <typename Left, typename Right>
struct CanMultiply<Left, Right, std::void_t<decltype(std::declval<Left>() * std::declval<Right>())>>
    : std::true_type {};

static_assert(CanMultiply<Matrix<2, 3>, Matrix<3, 4>>::value);
static_assert(!CanMultiply<Matrix<2, 3>, Matrix<2, 3>>::value);
static_assert(!CanMultiply<Matrix<2, 3>, Vector3>::value);
static_assert(!CanMultiply<Vector3, Matrix3>::value);
static_assert(std::is_same_v<decltype(Matrix<2, 3>{} * Matrix<3, 4>{}), Matrix<2, 4>>);
static_assert(std::is_same_v<decltype(Matrix<2, 3>{}.transposed()), Matrix<3, 2>>);

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const std::string& message) {
    check(std::isfinite(actual) && std::isfinite(expected), message + ": non-finite value");
    const double tolerance = std::max(1e-14, 1e-12 * std::max(std::abs(actual), std::abs(expected)));
    check(std::abs(actual - expected) <= tolerance, message);
}

template <std::size_t Rows, std::size_t Cols>
void nearMatrix(const Matrix<Rows, Cols>& actual, const Matrix<Rows, Cols>& expected,
                const std::string& message) {
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
            near(actual(row, col), expected(row, col),
                 message + " (" + std::to_string(row) + ", " + std::to_string(col) + ")");
        }
    }
}

void nearVector(const Vector3& actual, const Vector3& expected, const std::string& message) {
    near(actual.x, expected.x, message + " (x)");
    near(actual.y, expected.y, message + " (y)");
    near(actual.z, expected.z, message + " (z)");
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

void constructionAndAccess() {
    const Matrix<2, 3> zero;
    nearMatrix(zero, Matrix<2, 3>{0, 0, 0, 0, 0, 0}, "default is zero");
    constexpr Matrix<2, 3> values{1, 2, 3, 4, 5, 6};
    static_assert(values.rows == 2 && values.cols == 3);
    static_assert(values(0, 2) == 3 && values(1, 0) == 4);
    Matrix<2, 3> copy = values;
    copy(1, 2) = -6;
    near(copy(1, 2), -6, "mutable access");
    near(values(1, 2), 6, "copy owns its storage");
    nearMatrix(Matrix<1, 1>{7}, Matrix<1, 1>{7}, "one-element matrix");
    throws<std::invalid_argument>([] { (void)Matrix<2, 2>{1, 2, 3}; }, "short initializer rejected");
    throws<std::invalid_argument>([] { (void)Matrix<2, 2>{1, 2, 3, 4, 5}; }, "long initializer rejected");
    throws<std::invalid_argument>([] { (void)Matrix<2, 2>{1}; }, "scalar initializer is not a fill");
    throws<std::out_of_range>([&] { copy(2, 0) = 1; }, "mutable row bound");
    throws<std::out_of_range>([&] { copy(0, 3) = 1; }, "mutable column bound");
    throws<std::out_of_range>([&] { (void)values(2, 0); }, "const row bound");
    throws<std::out_of_range>([&] { (void)values(0, 3); }, "const column bound");
    throws<std::out_of_range>([&] { (void)values(std::numeric_limits<std::size_t>::max(), 0); },
                              "index checked before offset calculation");
    near(copy(1, 2), -6, "failed write preserves data");
}

void arithmetic() {
    const Matrix<2, 2> a{1, -2, 3, 4};
    const Matrix<2, 2> b{5, 6, -7, 8};
    nearMatrix(a + b, Matrix<2, 2>{6, 4, -4, 12}, "addition");
    nearMatrix(a - b, Matrix<2, 2>{-4, -8, 10, -4}, "subtraction");
    nearMatrix(-a, Matrix<2, 2>{-1, 2, -3, -4}, "negation");
    nearMatrix(a * 2.0, Matrix<2, 2>{2, -4, 6, 8}, "right scaling");
    nearMatrix(-2.0 * a, Matrix<2, 2>{-2, 4, -6, -8}, "left scaling");
    nearMatrix(a * 0.0, Matrix<2, 2>{}, "zero scaling");
    nearMatrix(a / -2.0, Matrix<2, 2>{-0.5, 1, -1.5, -2}, "division");
    Matrix<2, 2> value = a;
    value += b;
    value -= a;
    value *= 2.0;
    value /= 4.0;
    nearMatrix(value, Matrix<2, 2>{2.5, 3, -3.5, 4}, "compound operations");
    value += value;
    nearMatrix(value, b, "self addition");
    value -= value;
    nearMatrix(value, Matrix<2, 2>{}, "self subtraction");
    nearMatrix(a, Matrix<2, 2>{1, -2, 3, 4}, "value operators preserve input");
}

void products() {
    constexpr Matrix<2, 3> a{1, 2, 3, -1, 0, 4};
    constexpr Matrix<3, 2> b{2, 1, 0, -2, 3, 5};
    constexpr auto result = a * b;
    static_assert(result(0, 0) == 11 && result(1, 1) == 19);
    nearMatrix(result, Matrix<2, 2>{11, 12, 10, 19}, "rectangular product");
    const Matrix<2, 2> c{1, 2, 3, 4};
    const Matrix<2, 2> d{0, 1, -1, 0};
    nearMatrix(c * d, Matrix<2, 2>{-2, 1, -4, 3}, "ordered product");
    nearMatrix(d * c, Matrix<2, 2>{3, 4, -1, -2}, "reverse product differs");
    auto square = c;
    square = square * square;
    nearMatrix(square, Matrix<2, 2>{7, 10, 15, 22}, "self product assignment");
    nearMatrix(Matrix<1, 3>{1, 2, 3} * Matrix<3, 1>{4, 5, 6}, Matrix<1, 1>{32}, "row times column");
    nearMatrix(a * Matrix<3, 2>{}, Matrix<2, 2>{}, "right zero product");
    nearMatrix(Matrix<2, 3>{} * b, Matrix<2, 2>{}, "left zero product");
}

void identityAndTranspose() {
    constexpr auto identity = Matrix3::identity();
    static_assert(identity(0, 0) == 1 && identity(0, 1) == 0 && identity(2, 2) == 1);
    nearMatrix(identity, Matrix3{1, 0, 0, 0, 1, 0, 0, 0, 1}, "identity values");
    nearMatrix(Matrix<1, 1>::identity(), Matrix<1, 1>{1}, "one-element identity");
    constexpr Matrix<2, 3> a{1, 2, 3, -1, 0, 4};
    constexpr auto transposed = a.transposed();
    static_assert(transposed(2, 1) == 4);
    nearMatrix(transposed, Matrix<3, 2>{1, -1, 2, 0, 3, 4}, "rectangular transpose");
    nearMatrix(transposed.transposed(), a, "double transpose");
    nearMatrix(Matrix<2, 2>::identity() * a, a, "left identity");
    nearMatrix(a * identity, a, "right identity");
    const Matrix<3, 2> b{2, 1, 0, -2, 3, 5};
    nearMatrix((a * b).transposed(), b.transposed() * a.transposed(), "product transpose order");
}

void vectorTransforms() {
    constexpr Matrix3 rz{0, -1, 0, 1, 0, 0, 0, 0, 1};
    constexpr Matrix3 rx{1, 0, 0, 0, 0, -1, 0, 1, 0};
    constexpr Vector3 ex{1, 0, 0};
    constexpr auto rotated_axis = rz * ex;
    static_assert(rotated_axis.x == 0 && rotated_axis.y == 1 && rotated_axis.z == 0);
    const Vector3 v{2, -3, 5};
    nearVector(rz * v, {3, 2, 5}, "positive yaw on column vector");
    nearVector((rz * rx) * v, {5, 2, -3}, "rightmost transform applied first");
    nearVector((rx * rz) * v, {3, -5, 2}, "reverse transform composition");
    nearVector(rz.transposed() * (rz * v), v, "rotation round trip");
    near((rz * v).norm(), v.norm(), "rotation preserves length");
    nearMatrix(rz.transposed() * rz, Matrix3::identity(), "rotation orthogonality");
    nearVector(Matrix3::identity() * v, v, "identity vector transform");
    nearVector(Matrix3{} * v, {}, "zero vector transform");
    const Matrix3 inertia{2, 0, 0, 0, 3, 0, 0, 0, 4};
    nearVector(inertia * Vector3{1, -2, 3}, {2, -6, 12}, "diagonal inertia times angular velocity");
}

void invalidAndExtremeInputs() {
    for (double zero : {0.0, -0.0}) {
        Matrix<2, 2> a{1, 2, 3, 4};
        throws<std::domain_error>([&] { (void)(a / zero); }, "division by zero rejected");
        throws<std::domain_error>([&] { a /= zero; }, "compound division by zero rejected");
        nearMatrix(a, Matrix<2, 2>{1, 2, 3, 4}, "failed division preserves input");
    }
    const double minimum = std::numeric_limits<double>::denorm_min();
    if (minimum > 0.0) {
        nearMatrix(Matrix<2, 2>{minimum, 0, -minimum, minimum} / minimum,
                   Matrix<2, 2>{1, 0, -1, 1}, "tiny scalar division");
    }
    for (double invalid : {std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        for (std::size_t row = 0; row < 2; ++row) {
            for (std::size_t col = 0; col < 2; ++col) {
                Matrix<2, 2> a;
                a(row, col) = invalid;
                check(!a.isFinite(), "detect non-finite element");
            }
        }
    }
    check(Matrix3{}.isFinite(), "zero matrix is finite");
    check(Matrix<1, 1>{std::numeric_limits<double>::max()}.isFinite(), "maximum value is finite");
}

} // namespace

int main() {
    struct Test {
        const char* name;
        void (*run)();
    };
    const Test tests[] = {
        {"construction and access", constructionAndAccess},
        {"arithmetic", arithmetic},
        {"products", products},
        {"identity and transpose", identityAndTranspose},
        {"vector transforms", vectorTransforms},
        {"invalid and extreme inputs", invalidAndExtremeInputs},
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
