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

TEST(ValidationTest, AssertNonNegative) {
    EXPECT_NO_THROW(assert_non_negative(0.0, "x"));
    EXPECT_NO_THROW(assert_non_negative(1.0, "x"));

    EXPECT_THROW(assert_non_negative(-0.001, "x"), ValidationError);
}

TEST(ValidationTest, AssertInRange) {
    // Value in range
    EXPECT_NO_THROW(assert_in_range(0.5, "x", 0.0, 1.0));
    EXPECT_NO_THROW(assert_in_range(0.0, "x", 0.0, 1.0));
    EXPECT_NO_THROW(assert_in_range(1.0, "x", 0.0, 1.0));

    // Value out of range
    EXPECT_THROW(assert_in_range(-0.1, "x", 0.0, 1.0), ValidationError);
    EXPECT_THROW(assert_in_range(1.1, "x", 0.0, 1.0), ValidationError);
}

TEST(ValidationTest, AssertAtMost) {
    EXPECT_NO_THROW(assert_at_most(0.5, "x", 1.0));
    EXPECT_NO_THROW(assert_at_most(1.0, "x", 1.0));

    EXPECT_THROW(assert_at_most(1.1, "x", 1.0), ValidationError);
}

TEST(ValidationTest, AssertAtLeast) {
    EXPECT_NO_THROW(assert_at_least(1.0, "x", 0.0));
    EXPECT_NO_THROW(assert_at_least(0.0, "x", 0.0));

    EXPECT_THROW(assert_at_least(-0.1, "x", 0.0), ValidationError);
}

TEST(ValidationTest, AssertUnitQuaternion) {
    Eigen::Vector4d q_identity(1.0, 0.0, 0.0, 0.0); // w, x, y, z
    EXPECT_NO_THROW(assert_unit_quaternion(q_identity));

    Eigen::Vector4d q_bad(1.0, 0.1, 0.0, 0.0);
    EXPECT_THROW(assert_unit_quaternion(q_bad), ValidationError);
}

TEST(ValidationTest, ErrorMessagesIncludeValues) {
    // Verify error messages include actual values for debugging
    try {
        assert_positive(-5.0, "altitude");
        FAIL() << "Expected ValidationError";
    } catch (const ValidationError &e) {
        std::string msg = e.what();
        EXPECT_NE(msg.find("-5"), std::string::npos)
            << "Error should include value";
        EXPECT_NE(msg.find("altitude"), std::string::npos)
            << "Error should include name";
    }

    try {
        assert_in_range(15.0, "temperature", 0.0, 10.0);
        FAIL() << "Expected ValidationError";
    } catch (const ValidationError &e) {
        std::string msg = e.what();
        EXPECT_NE(msg.find("15"), std::string::npos)
            << "Error should include value";
        EXPECT_NE(msg.find("[0"), std::string::npos)
            << "Error should include min";
        EXPECT_NE(msg.find("10]"), std::string::npos)
            << "Error should include max";
    }
}

} // namespace vulcan::tests
