#include <csim/math/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {
using namespace csim::math;
constexpr double pi = 3.14159265358979323846;

static_assert(std::is_aggregate_v<Quaternion>);
constexpr Quaternion qi{0, 1, 0, 0};
constexpr Quaternion qj{0, 0, 1, 0};
constexpr Quaternion qk = qi * qj;
static_assert(qk.w == 0 && qk.x == 0 && qk.y == 0 && qk.z == 1);
static_assert((qj * qi).z == -1);
static_assert(Quaternion{}.w == 1);
static_assert((2 * qi + qj - qi).x == 1);
static_assert(qi.conjugated().x == -1);
static_assert(skew(Vector3{1, 2, 3})(0, 1) == -3);

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(double actual, double expected, const std::string& message, double tolerance = 1e-12) {
    check(std::isfinite(actual) && std::isfinite(expected), message + ": non-finite value");
    check(std::abs(actual - expected) <= tolerance * std::max({1.0, std::abs(actual), std::abs(expected)}),
          message);
}

void nearVector(const Vector3& a, const Vector3& b, const std::string& message) {
    near(a.x, b.x, message + " x");
    near(a.y, b.y, message + " y");
    near(a.z, b.z, message + " z");
}

void nearQuaternion(const Quaternion& a, const Quaternion& b, const std::string& message) {
    near(a.w, b.w, message + " w");
    near(a.x, b.x, message + " x");
    near(a.y, b.y, message + " y");
    near(a.z, b.z, message + " z");
}

void nearMatrix(const Matrix3& a, const Matrix3& b, const std::string& message, double tolerance = 1e-12) {
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            near(a(i, j), b(i, j), message, tolerance);
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

void rawArithmetic() {
    nearQuaternion(Quaternion{}, {1, 0, 0, 0}, "identity default");
    nearQuaternion(Quaternion::identity(), {}, "identity factory");
    nearQuaternion(qi * qi, {-1, 0, 0, 0}, "i squared");
    nearQuaternion(qi * qj * qk, {-1, 0, 0, 0}, "i j k");
    const Quaternion q{1, -2, 3, -4};
    const Quaternion p{-2, 1, 4, 3};
    nearQuaternion(q * p, {0, 30, 0, 0}, "general Hamilton product");
    nearQuaternion(q * q.conjugated(), {30, 0, 0, 0}, "product with conjugate");
    nearQuaternion((q * p).conjugated(), p.conjugated() * q.conjugated(), "conjugate reverses order");
    nearQuaternion(q + p, {-1, -1, 7, -1}, "addition");
    nearQuaternion(q - p, {3, -3, -1, -7}, "subtraction");
    nearQuaternion(-q, {-1, 2, -3, 4}, "negation");
    nearQuaternion(q / -2, {-0.5, 1, -1.5, 2}, "division");
    Quaternion value = q;
    value += p;
    value -= p;
    value *= 2;
    value /= 2;
    nearQuaternion(value, q, "compound arithmetic");
    value -= value;
    nearQuaternion(value, {0, 0, 0, 0}, "raw zero differs from identity");
    near(q.dot(p), -4, "dot product");
    near(q.squaredNorm(), 30, "squared norm");
    near(q.norm(), std::sqrt(30.0), "norm");
    nearQuaternion(q, {1, -2, 3, -4}, "operands preserved");
}

void principalAxesAndSkew() {
    const Vector3 ex{1, 0, 0}, ey{0, 1, 0}, ez{0, 0, 1};
    nearVector(rotationX(pi / 2) * ey, ez, "positive roll");
    nearVector(rotationY(pi / 2) * ez, ex, "positive pitch");
    nearVector(rotationZ(pi / 2) * ex, ey, "positive yaw");
    nearVector(rotationZ(-pi / 2) * ex, -ey, "negative yaw");
    nearMatrix(rotationZ(pi / 2), Matrix3{0, -1, 0, 1, 0, 0, 0, 0, 1}, "known yaw matrix");
    nearVector(Quaternion::fromAxisAngle({0, 0, 7}, pi / 2).rotate(ex), ey, "axis is normalized");
    nearVector(Quaternion::fromAxisAngle({0, 0, -7}, pi / 2).rotate(ex), -ey, "negative axis");
    nearQuaternion(Quaternion::fromAxisAngle(ez, 0), {}, "zero angle");
    const Vector3 a{1, 2, -3}, b{-4, 5, 6};
    nearVector(skew(a) * b, a.cross(b), "skew cross product");
    nearMatrix(skew(a).transposed(), -skew(a), "skew antisymmetric");
}

void compositionAndEuler() {
    const auto q_WA = Quaternion::fromAxisAngle({0, 0, 1}, pi / 2);
    const auto q_AB = Quaternion::fromAxisAngle({1, 0, 0}, pi / 2);
    nearVector((q_WA * q_AB).rotate({0, 1, 0}), {0, 0, 1}, "right rotation acts first");
    nearVector((q_AB * q_WA).rotate({0, 1, 0}), {-1, 0, 0}, "composition is noncommutative");
    nearMatrix((q_WA * q_AB).toRotationMatrix(), rotationZ(pi / 2) * rotationX(pi / 2),
               "matrix composition order");
    for (double pitch : {-pi / 2, -0.4, 0.0, 0.6, pi / 2}) {
        const double roll = 0.37, yaw = -0.81;
        const auto q = Quaternion::fromRollPitchYaw(roll, pitch, yaw);
        const Matrix3 matrix = rotationFromRollPitchYaw(roll, pitch, yaw);
        nearMatrix(q.toRotationMatrix(), matrix, "roll pitch yaw agrees across representations");
        const Vector3 v{1, 2, 3};
        nearVector(q.rotate(v), rotationZ(yaw) * (rotationY(pitch) * (rotationX(roll) * v)),
                   "Euler sequence");
    }
}

void normalizationAndInverse() {
    const Quaternion base{1, -2, 3, -4};
    const auto unit = base.normalized();
    for (double scale : {1e-300, -1e-300, 1.0, -1.0, 1e300, -1e300}) {
        const auto q = base * scale;
        near(q.normalized().norm(), 1, "unit length at all scales");
        near(std::abs(q.normalized().dot(unit)), 1, "same orientation at all scales");
        nearQuaternion(q * q.inverse(), {}, "right inverse at all scales");
        nearQuaternion(q.inverse() * q, {}, "left inverse at all scales");
        nearMatrix(q.toRotationMatrix(), unit.toRotationMatrix(), "rotation ignores nonzero scale");
    }
    nearQuaternion(unit.inverse(), unit.conjugated(), "unit inverse equals conjugate");
    nearQuaternion(base, {1, -2, 3, -4}, "queries preserve components");
    const double maximum = std::numeric_limits<double>::max();
    nearQuaternion(Quaternion{maximum, -maximum, maximum, -maximum}.normalized(),
                   {0.5, -0.5, 0.5, -0.5}, "normalization beyond norm range");
    near(Quaternion{3e-200, 4e-200, 0, 0}.norm() / 1e-200, 5, "tiny norm");
    near(Quaternion{3e200, 4e200, 0, 0}.norm() / 1e200, 5, "large norm");
    const double minimum = std::numeric_limits<double>::denorm_min();
    if (minimum > 0) {
        nearQuaternion(Quaternion{0, minimum, 0, 0}.normalized(), qi, "subnormal normalization");
        nearQuaternion(Quaternion{0, minimum, 0, 0} / minimum, qi, "subnormal scalar division");
        throws<std::overflow_error>([&] { (void)Quaternion{minimum, 0, 0, 0}.inverse(); },
                                    "unrepresentable inverse rejected");
    }
}

// Independent Rodrigues construction, including exact and near half-turns.
void conversionRoundTrips() {
    const Vector3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 2, 3}, {-4, 2, -1}, {1, 1, 1}};
    const double angles[] = {0, 1e-12, -1e-8, 0.7, -1.3, pi / 2, pi - 1e-12, pi, pi + 1e-12, 2 * pi};
    for (const auto& axis : axes) {
        const Vector3 unit = axis.normalized();
        const Matrix3 k = skew(unit);
        for (double angle : angles) {
            const Matrix3 reference = Matrix3::identity() + std::sin(angle) * k
                                      + (1 - std::cos(angle)) * (k * k);
            const auto q = Quaternion::fromAxisAngle(axis, angle);
            nearMatrix(q.toRotationMatrix(), reference, "Rodrigues agreement");
            const auto recovered = Quaternion::fromRotationMatrix(reference);
            near(recovered.norm(), 1, "converted quaternion normalized");
            near(std::abs(q.dot(recovered)), 1, "round trip up to quaternion sign");
            nearMatrix(recovered.toRotationMatrix(), reference, "matrix round trip");
            nearMatrix(reference.transposed() * reference, Matrix3::identity(), "orthogonality");
            const Vector3 c0{reference(0, 0), reference(1, 0), reference(2, 0)};
            const Vector3 c1{reference(0, 1), reference(1, 1), reference(2, 1)};
            const Vector3 c2{reference(0, 2), reference(1, 2), reference(2, 2)};
            near(c0.cross(c1).dot(c2), 1, "proper determinant");
            check(isRotationMatrix(reference), "SO(3) validation");
        }
    }
    // Exact half-turn matrices exercise each vector-dominant conversion branch.
    nearQuaternion(Quaternion::fromRotationMatrix(Matrix3{1, 0, 0, 0, -1, 0, 0, 0, -1}), qi, "x half turn");
    nearQuaternion(Quaternion::fromRotationMatrix(Matrix3{-1, 0, 0, 0, 1, 0, 0, 0, -1}), qj, "y half turn");
    nearQuaternion(Quaternion::fromRotationMatrix(Matrix3{-1, 0, 0, 0, -1, 0, 0, 0, 1}), qk, "z half turn");
}

void vectorRotation() {
    const auto q = Quaternion::fromAxisAngle({1, -2, 3}, 1.2);
    for (const Vector3 v : {Vector3{}, Vector3{1, -2, 7}, Vector3{-3, 1, -5}}) {
        const Vector3 rotated = q.rotate(v);
        nearVector(rotated, q.toRotationMatrix() * v, "matrix and quaternion action");
        near(rotated.norm(), v.norm(), "rotation preserves length");
        nearVector(q.inverse().rotate(rotated), v, "inverse action");
        nearVector((-q).rotate(v), rotated, "q and minus q same rotation");
        const Quaternion pure{0, v.x, v.y, v.z};
        const Quaternion product = q * pure * q.conjugated();
        near(product.w, 0, "sandwich scalar part");
        nearVector(rotated, {product.x, product.y, product.z}, "Hamilton sandwich agreement");
    }
    const double maximum = std::numeric_limits<double>::max();
    nearVector(Quaternion{}.rotate({maximum, -maximum, maximum}), {maximum, -maximum, maximum},
               "finite components beyond vector norm range");
    nearVector(q.rotate({1e300, -2e300, 3e300}) / 1e300, q.rotate({1, -2, 3}), "large vector");
    nearVector(q.rotate({1e-300, -2e-300, 3e-300}) / 1e-300, q.rotate({1, -2, 3}), "tiny vector");
    throws<std::overflow_error>([&] {
        (void)Quaternion::fromAxisAngle({0, 0, 1}, pi / 4).rotate({maximum, maximum, 0});
    }, "unrepresentable rotation rejected");
}

void bodyRateKinematics() {
    nearQuaternion(Quaternion{}.derivativeBodyRate({1, 2, 3}), {0, 0.5, 1, 1.5}, "identity derivative");
    const auto q = Quaternion::fromAxisAngle({1, 0, 0}, pi / 2);
    const double s = std::sqrt(0.5);
    nearQuaternion(q.derivativeBodyRate({0, 0, 2}), {0, 0, -s, s}, "body rate multiplies on right");
    nearQuaternion(q.derivativeBodyRate({}), {0, 0, 0, 0}, "zero angular velocity");
    const auto initial = Quaternion::fromRollPitchYaw(0.3, -0.2, 0.8);
    const Vector3 omega{0.4, -0.7, 1.3};
    const auto derivative = initial.derivativeBodyRate(omega);
    near(initial.dot(derivative), 0, "quaternion tangent derivative");
    nearQuaternion((initial * 2).derivativeBodyRate(omega), derivative * 2, "derivative does not normalize");
    const double h = 1e-5;
    const Matrix3 finite_difference = ((initial + derivative * h).toRotationMatrix()
                                      - (initial - derivative * h).toRotationMatrix()) / (2 * h);
    nearMatrix(finite_difference, initial.toRotationMatrix() * skew(omega), "Rdot = R skew(Omega_B)", 2e-9);
    const double maximum = std::numeric_limits<double>::max();
    throws<std::overflow_error>([&] {
        (void)Quaternion{maximum, maximum, maximum, maximum}.derivativeBodyRate({4, 4, 4});
    }, "derivative overflow reported");
}

void invalidInputsAndValidation() {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const Quaternion zero{0, 0, 0, 0};
    for (const Quaternion q : {zero, Quaternion{-0.0, 0.0, -0.0, 0.0}}) {
        throws<std::domain_error>([&] { (void)q.normalized(); }, "zero normalization");
        throws<std::domain_error>([&] { (void)q.inverse(); }, "zero inverse");
        throws<std::domain_error>([&] { (void)q.toRotationMatrix(); }, "zero matrix conversion");
        throws<std::domain_error>([&] { (void)q.rotate({}); }, "zero is not an attitude");
    }
    for (double invalid : {inf, -inf, nan}) {
        for (const Quaternion q : {Quaternion{invalid, 1, 2, 3}, Quaternion{1, invalid, 2, 3},
                                   Quaternion{1, 2, invalid, 3}, Quaternion{1, 2, 3, invalid}}) {
            check(!q.isFinite(), "non-finite quaternion");
            throws<std::domain_error>([&] { (void)q.normalized(); }, "non-finite normalization");
            throws<std::domain_error>([&] { (void)q.inverse(); }, "non-finite inverse");
            throws<std::domain_error>([&] { (void)q.toRotationMatrix(); }, "non-finite matrix conversion");
            throws<std::invalid_argument>([&] { (void)q.derivativeBodyRate({}); }, "non-finite derivative input");
        }
        throws<std::invalid_argument>([&] { (void)rotationX(invalid); }, "non-finite roll");
        throws<std::invalid_argument>([&] { (void)rotationY(invalid); }, "non-finite pitch");
        throws<std::invalid_argument>([&] { (void)rotationZ(invalid); }, "non-finite yaw");
        throws<std::invalid_argument>([&] { (void)Quaternion::fromAxisAngle({1, 0, 0}, invalid); }, "non-finite angle");
        throws<std::domain_error>([&] { (void)Quaternion::fromAxisAngle({1, invalid, 0}, 0); }, "non-finite axis");
        throws<std::invalid_argument>([&] { (void)Quaternion{}.rotate({0, invalid, 0}); }, "non-finite vector");
        throws<std::invalid_argument>([&] { (void)Quaternion{}.derivativeBodyRate({invalid, 0, 0}); }, "non-finite rate");
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                auto matrix = Matrix3::identity();
                matrix(i, j) = invalid;
                check(!isRotationMatrix(matrix), "non-finite matrix");
                throws<std::invalid_argument>([&] { (void)Quaternion::fromRotationMatrix(matrix); }, "non-finite matrix rejected");
            }
        }
    }
    throws<std::domain_error>([] { (void)Quaternion::fromAxisAngle({}, 0); }, "zero axis even for zero angle");
    for (double scalar : {0.0, -0.0}) {
        Quaternion q{1, 2, 3, 4};
        throws<std::domain_error>([&] { q /= scalar; }, "zero scalar division");
        nearQuaternion(q, {1, 2, 3, 4}, "failed division preserves input");
    }
    for (const Matrix3& invalid : {Matrix3{}, Matrix3::identity() * 2,
                                  Matrix3{-1, 0, 0, 0, 1, 0, 0, 0, 1},
                                  Matrix3{1, 0.1, 0, 0, 1, 0, 0, 0, 1},
                                  Matrix3::identity() * std::numeric_limits<double>::max()}) {
        check(!isRotationMatrix(invalid), "invalid rotation matrix");
        throws<std::invalid_argument>([&] { (void)Quaternion::fromRotationMatrix(invalid); }, "invalid rotation rejected");
    }
    for (double tolerance : {-1.0, 1.0, inf, nan}) {
        throws<std::invalid_argument>([&] { (void)isRotationMatrix(Matrix3::identity(), tolerance); }, "invalid tolerance");
        throws<std::invalid_argument>([&] { (void)Quaternion::fromRotationMatrix(Matrix3::identity(), tolerance); }, "invalid conversion tolerance");
    }
    check(isRotationMatrix(Matrix3::identity(), 0), "exact identity with zero tolerance");
    auto perturbed = Matrix3::identity();
    perturbed(0, 0) += 1e-8;
    check(!isRotationMatrix(perturbed), "default tolerance rejects perturbation");
    check(isRotationMatrix(perturbed, 1e-6), "explicit tolerance accepts perturbation");
    nearMatrix(Quaternion::fromRotationMatrix(perturbed, 1e-6).toRotationMatrix(), Matrix3::identity(),
               "conversion normalizes accepted near-rotation");
}
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"raw quaternion arithmetic", rawArithmetic},
        {"principal axes and skew", principalAxesAndSkew},
        {"composition and Euler inputs", compositionAndEuler},
        {"normalization and inverse", normalizationAndInverse},
        {"conversion round trips", conversionRoundTrips},
        {"vector rotation", vectorRotation},
        {"body rate kinematics", bodyRateKinematics},
        {"invalid inputs and validation", invalidInputsAndValidation},
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
