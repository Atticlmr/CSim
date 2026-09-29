#include <csim/math/vector.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using csim::math::Vector3;

void check(bool condition, const std::string& message) {
    // Deliberately independent of assert/NDEBUG so Release tests stay active.
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

void nearVector(const Vector3& actual, const Vector3& expected, const std::string& message) {
    near(actual.x, expected.x, message + " (x)");
    near(actual.y, expected.y, message + " (y)");
    near(actual.z, expected.z, message + " (z)");
}

template <typename Function>
void domainError(Function function, const std::string& message) {
    try {
        function();
    } catch (const std::domain_error&) {
        return;
    }
    throw std::runtime_error(message);
}

void constructionAndArithmetic() {
    nearVector(Vector3{}, {0.0, 0.0, 0.0}, "default vector");
    constexpr Vector3 a{1.0, -2.0, 3.0};
    constexpr Vector3 b{-4.0, 5.0, 2.0};
    constexpr Vector3 sum = a + b;
    static_assert(sum.x == -3.0 && sum.y == 3.0 && sum.z == 5.0);
    nearVector(sum, {-3.0, 3.0, 5.0}, "addition");
    nearVector(a - b, {5.0, -7.0, 1.0}, "subtraction");
    nearVector(-a, {-1.0, 2.0, -3.0}, "negation");
    nearVector(a * 2.0, {2.0, -4.0, 6.0}, "right scaling");
    nearVector(-2.0 * a, {-2.0, 4.0, -6.0}, "left scaling");
    nearVector(a * 0.0, {}, "zero scaling");
    nearVector(a / -2.0, {-0.5, 1.0, -1.5}, "division");

    Vector3 value = a;
    value += b;
    value -= a;
    value *= 2.0;
    value /= 4.0;
    nearVector(value, {-2.0, 2.5, 1.0}, "compound operations");
    value += value;
    nearVector(value, b, "self addition");
    value -= value;
    nearVector(value, {}, "self subtraction");
    nearVector(a, {1.0, -2.0, 3.0}, "value operators preserve operands");
}

void dotAndCross() {
    const Vector3 a{1.0, -2.0, 3.0};
    const Vector3 b{-4.0, 5.0, 2.0};
    near(a.dot(b), -8.0, "dot product");
    nearVector(a.cross(b), {-19.0, -14.0, -3.0}, "cross product");
    nearVector(b.cross(a), {19.0, 14.0, 3.0}, "cross antisymmetry");
    nearVector(a.cross(a), {}, "parallel cross product");
    nearVector(a.cross({}), {}, "zero cross product");
    near(a.cross(b).dot(a), 0.0, "cross orthogonal to first operand");
    near(a.cross(b).dot(b), 0.0, "cross orthogonal to second operand");
    const Vector3 ex{1.0, 0.0, 0.0};
    const Vector3 ey{0.0, 1.0, 0.0};
    const Vector3 ez{0.0, 0.0, 1.0};
    nearVector(ex.cross(ey), ez, "right-handed x cross y");
    nearVector(ey.cross(ez), ex, "right-handed y cross z");
    nearVector(ez.cross(ex), ey, "right-handed z cross x");
}

void normsAndNormalization() {
    const Vector3 v{3.0, 4.0, 12.0};
    near(v.squaredNorm(), 169.0, "squared norm");
    near(v.norm(), 13.0, "norm");
    nearVector(v.normalized(), {3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0}, "normalization");
    near(v.normalized().norm(), 1.0, "unit length");
    nearVector(v, {3.0, 4.0, 12.0}, "normalization preserves input");
    nearVector(Vector3{0.0, 0.0, -7.0}.normalized(), {0.0, 0.0, -1.0}, "negative axis");
    near(Vector3{}.norm(), 0.0, "zero norm");
    near(Vector3{}.squaredNorm(), 0.0, "zero squared norm");
}

void extremeMagnitudes() {
    const Vector3 huge{3e200, 4e200, 0.0};
    near(huge.norm(), 5e200, "norm avoids intermediate overflow", 1e-12, 0.0);
    nearVector(huge.normalized(), {0.6, 0.8, 0.0}, "large normalization");
    const Vector3 tiny{3e-200, -4e-200, 0.0};
    near(tiny.norm(), 5e-200, "norm avoids intermediate underflow", 1e-12, 0.0);
    nearVector(tiny.normalized(), {0.6, -0.8, 0.0}, "tiny normalization");

    const double maximum = std::numeric_limits<double>::max();
    const Vector3 beyond_norm_range{maximum, -maximum, maximum};
    const double unit = 1.0 / std::sqrt(3.0);
    nearVector(beyond_norm_range.normalized(), {unit, -unit, unit}, "normalize beyond norm range");

    const double minimum = std::numeric_limits<double>::denorm_min();
    if (minimum > 0.0) {
        const Vector3 subnormal{minimum, 0.0, 0.0};
        nearVector(subnormal.normalized(), {1.0, 0.0, 0.0}, "subnormal normalization");
        nearVector(subnormal / minimum, {1.0, 0.0, 0.0}, "division avoids reciprocal overflow");
    }
}

void invalidInputs() {
    domainError([] { (void)Vector3{}.normalized(); }, "zero normalization must fail");
    domainError([] { (void)Vector3{-0.0, 0.0, -0.0}.normalized(); }, "signed zero must fail");
    for (const double zero : {0.0, -0.0}) {
        Vector3 v{1.0, 2.0, 3.0};
        domainError([&] { (void)(v / zero); }, "division by zero must fail");
        domainError([&] { v /= zero; }, "compound division by zero must fail");
        nearVector(v, {1.0, 2.0, 3.0}, "division failure preserves input");
    }
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double invalid : {infinity, -infinity, nan}) {
        for (const Vector3 v : {Vector3{invalid, 1.0, 2.0}, Vector3{1.0, invalid, 2.0},
                               Vector3{1.0, 2.0, invalid}}) {
            check(!v.isFinite(), "detect non-finite component");
            domainError([&] { (void)v.normalized(); }, "non-finite normalization must fail");
        }
    }
    check(Vector3{}.isFinite(), "zero vector is finite");
    check(Vector3{1.0, -2.0, 3.0}.isFinite(), "ordinary vector is finite");
}

} // namespace

int main() {
    struct Test {
        const char* name;
        void (*run)();
    };
    const Test tests[] = {
        {"construction and arithmetic", constructionAndArithmetic},
        {"dot and cross", dotAndCross},
        {"norms and normalization", normsAndNormalization},
        {"extreme magnitudes", extremeMagnitudes},
        {"invalid inputs", invalidInputs},
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
