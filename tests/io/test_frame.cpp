/**
 * @file test_frame.cpp
 * @brief Unit tests for Frame data container
 */

#include <gtest/gtest.h>
#include <vulcan/io/Frame.hpp>

using namespace vulcan::io;

class FrameTest : public ::testing::Test {
  protected:
    void SetUp() override {
        schema_.add_vec3("position", SignalLifecycle::Dynamic, "m")
            .add_vec3("velocity", SignalLifecycle::Dynamic, "m/s")
            .add_quat("attitude")
            .add_int32("phase", SignalLifecycle::Dynamic, "enum")
            .add_double("mass", SignalLifecycle::Static, "kg");
    }

    TelemetrySchema schema_;
};

// =============================================================================
// Basic Operations
// =============================================================================

TEST_F(FrameTest, Construction) {
    Frame frame(schema_);
    EXPECT_EQ(frame.size_bytes(), schema_.frame_size_bytes());
    EXPECT_EQ(frame.time(), 0.0);
}

TEST_F(FrameTest, SetGetTime) {
    Frame frame(schema_);
    frame.set_time(1.234);
    EXPECT_DOUBLE_EQ(frame.time(), 1.234);
}

TEST_F(FrameTest, SetGetDouble) {
    Frame frame(schema_);
    frame.set("mass", 100.5);
    EXPECT_DOUBLE_EQ(frame.get_double("mass"), 100.5);
}

TEST_F(FrameTest, SetGetInt32) {
    Frame frame(schema_);
    frame.set("phase", int32_t{3});
    EXPECT_EQ(frame.get_int32("phase"), 3);
}

TEST_F(FrameTest, SetGetVec3) {
    Frame frame(schema_);
    Eigen::Vector3d pos(1.0, 2.0, 3.0);
    frame.set("position", pos);

    auto result = frame.get_vec3("position");
    EXPECT_DOUBLE_EQ(result.x(), 1.0);
    EXPECT_DOUBLE_EQ(result.y(), 2.0);
    EXPECT_DOUBLE_EQ(result.z(), 3.0);
}

TEST_F(FrameTest, SetGetQuaternion) {
    Frame frame(schema_);
    Eigen::Quaterniond q(0.5, 0.5, 0.5, 0.5);
    q.normalize();

    frame.set("attitude", q);

    auto result = frame.get_quat("attitude");
    EXPECT_NEAR(result.w(), q.w(), 1e-15);
    EXPECT_NEAR(result.x(), q.x(), 1e-15);
    EXPECT_NEAR(result.y(), q.y(), 1e-15);
    EXPECT_NEAR(result.z(), q.z(), 1e-15);
}

TEST_F(FrameTest, Clear) {
    Frame frame(schema_);
    frame.set_time(1.0);
    frame.set("position", Eigen::Vector3d(1, 2, 3));

    frame.clear();

    EXPECT_DOUBLE_EQ(frame.time(), 0.0);
    auto pos = frame.get_vec3("position");
    EXPECT_DOUBLE_EQ(pos.x(), 0.0);
    EXPECT_DOUBLE_EQ(pos.y(), 0.0);
    EXPECT_DOUBLE_EQ(pos.z(), 0.0);
}

// =============================================================================
// Type Safety
// =============================================================================

TEST_F(FrameTest, TypeMismatchThrows) {
    Frame frame(schema_);

    // Try to set double signal with int32
    EXPECT_THROW(frame.set("mass", int32_t{100}), std::runtime_error);

    // Try to get double signal as int32
    frame.set("mass", 100.0);
    EXPECT_THROW(frame.get_int32("mass"), std::runtime_error);
}

TEST_F(FrameTest, SignalNotFoundThrows) {
    Frame frame(schema_);
    EXPECT_THROW(frame.set("nonexistent", 1.0), std::runtime_error);
    EXPECT_THROW(frame.get_double("nonexistent"), std::runtime_error);
}

// =============================================================================
// Raw Buffer Access
// =============================================================================

TEST_F(FrameTest, RawBufferAccess) {
    Frame frame(schema_);
    frame.set("position", Eigen::Vector3d(1, 2, 3));

    const std::byte *data = frame.data();
    EXPECT_NE(data, nullptr);

    // Verify we can read position.x from raw buffer
    double pos_x;
    std::memcpy(&pos_x, data + schema_.offset("position.x"), sizeof(double));
    EXPECT_DOUBLE_EQ(pos_x, 1.0);
}

// =============================================================================
// Int64 Signals
// =============================================================================

class FrameInt64Test : public ::testing::Test {
  protected:
    void SetUp() override {
        schema_.add_int64("timestamp", SignalLifecycle::Static)
            .add_int64("counter", SignalLifecycle::Dynamic)
            .add_double("value");
    }

    TelemetrySchema schema_;
};

TEST_F(FrameInt64Test, SetGetInt64) {
    Frame frame(schema_);
    frame.set("timestamp", int64_t{1234567890123456789LL});
    EXPECT_EQ(frame.get_int64("timestamp"), 1234567890123456789LL);
}

TEST_F(FrameInt64Test, Int64LargeValues) {
    Frame frame(schema_);

    // Test max int64
    frame.set("counter", std::numeric_limits<int64_t>::max());
    EXPECT_EQ(frame.get_int64("counter"), std::numeric_limits<int64_t>::max());

    // Test min int64
    frame.set("counter", std::numeric_limits<int64_t>::min());
    EXPECT_EQ(frame.get_int64("counter"), std::numeric_limits<int64_t>::min());
}

TEST_F(FrameInt64Test, Int64TypeMismatchThrows) {
    Frame frame(schema_);

    // Try to set int64 signal with double
    EXPECT_THROW(frame.set("timestamp", 1.0), std::runtime_error);

    // Try to set int64 signal with int32
    EXPECT_THROW(frame.set("timestamp", int32_t{100}), std::runtime_error);

    // Try to get double as int64
    frame.set("value", 1.0);
    EXPECT_THROW(frame.get_int64("value"), std::runtime_error);

    // Try to get int64 as double
    frame.set("timestamp", int64_t{100});
    EXPECT_THROW(frame.get_double("timestamp"), std::runtime_error);

    // Try to get int64 as int32
    EXPECT_THROW(frame.get_int32("timestamp"), std::runtime_error);
}

// =============================================================================
// Additional Error Paths
// =============================================================================

TEST_F(FrameTest, GetInt32FromDoubleThrows) {
    Frame frame(schema_);
    frame.set("mass", 100.0);
    EXPECT_THROW(frame.get_int32("mass"), std::runtime_error);
}

TEST_F(FrameTest, SetInt32ToDoubleThrows) {
    Frame frame(schema_);
    EXPECT_THROW(frame.set("mass", int32_t{100}), std::runtime_error);
}

TEST_F(FrameTest, GetDoubleFromInt32Throws) {
    Frame frame(schema_);
    frame.set("phase", int32_t{1});
    EXPECT_THROW(frame.get_double("phase"), std::runtime_error);
}

TEST_F(FrameTest, SetDoubleToInt32Throws) {
    Frame frame(schema_);
    EXPECT_THROW(frame.set("phase", 1.0), std::runtime_error);
}

TEST_F(FrameTest, GetNonexistentSignalThrows) {
    Frame frame(schema_);
    EXPECT_THROW(frame.get_double("nonexistent"), std::runtime_error);
    EXPECT_THROW(frame.get_int32("nonexistent"), std::runtime_error);
    EXPECT_THROW(frame.get_int64("nonexistent"), std::runtime_error);
}
