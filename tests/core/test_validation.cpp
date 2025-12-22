#include <gtest/gtest.h>
#include <limits>
#include <vulcan/core/Validation.hpp>

namespace vulcan::tests {

using namespace vulcan::validation;

TEST(ValidationTest, IsFinite) {
    EXPECT_TRUE(is_finite(0.0));
    EXPECT_TRUE(is_finite(1e10));
    EXPECT_TRUE(is_finite(-1e10));

    double inf = std::numeric_limits<double>::infinity();
    double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(is_finite(inf));
    EXPECT_FALSE(is_finite(-inf));
    EXPECT_FALSE(is_finite(nan));
}

TEST(ValidationTest, Clamp) {
    EXPECT_DOUBLE_EQ(clamp(0.5, 0.0, 1.0), 0.5);
    EXPECT_DOUBLE_EQ(clamp(-0.5, 0.0, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(clamp(1.5, 0.0, 1.0), 1.0);
}

TEST(ValidationTest, AssertFinite) {
    EXPECT_NO_THROW(assert_finite(0.0, "x"));

    double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(assert_finite(inf, "x"), ValidationError);
}

TEST(ValidationTest, AssertPositive) {
    EXPECT_NO_THROW(assert_positive(1.0, "x"));
    EXPECT_NO_THROW(assert_positive(1e-10, "x"));

    EXPECT_THROW(assert_positive(0.0, "x"), ValidationError);
    EXPECT_THROW(assert_positive(-1.0, "x"), ValidationError);
}

TEST(ValidationTest, AssertUnitQuaternion) {
    Eigen::Vector4d q_identity(1.0, 0.0, 0.0, 0.0); // w, x, y, z
    EXPECT_NO_THROW(assert_unit_quaternion(q_identity));

    Eigen::Vector4d q_bad(1.0, 0.1, 0.0, 0.0);
    EXPECT_THROW(assert_unit_quaternion(q_bad), ValidationError);
}

} // namespace vulcan::tests
