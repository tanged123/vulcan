/**
 * @file test_frame_serializer.cpp
 * @brief Unit tests for FrameSerializer binary serialization
 */

#include <gtest/gtest.h>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/FrameSerializer.hpp>

using namespace vulcan::io;

class FrameSerializerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        schema_.add_vec3("position", SignalLifecycle::Dynamic, "m")
            .add_int32("phase", SignalLifecycle::Dynamic)
            .add_double("mass", SignalLifecycle::Static, "kg");
    }

    TelemetrySchema schema_;
};

// =============================================================================
// Basic Serialization
// =============================================================================

TEST_F(FrameSerializerTest, SerializeDynamicSize) {
    FrameSerializer serializer(schema_);

    // Dynamic: time (8) + position.x,y,z (24) + phase (8) = 40 bytes
    EXPECT_EQ(serializer.dynamic_size_bytes(), 40);

    // Static: mass (8) = 8 bytes
    EXPECT_EQ(serializer.static_size_bytes(), 8);
}

TEST_F(FrameSerializerTest, SerializeDeserializeRoundTrip) {
    FrameSerializer serializer(schema_);

    // Create and populate source frame
    Frame src(schema_);
    src.set_time(1.234);
    src.set("position", Eigen::Vector3d(1, 2, 3));
    src.set("phase", int32_t{5});
    src.set("mass", 100.0);

    // Serialize dynamic signals
    auto bytes = serializer.serialize(src);
    EXPECT_EQ(bytes.size(), serializer.dynamic_size_bytes());

    // Deserialize into new frame
    Frame dst(schema_);
    serializer.deserialize(bytes, dst);

    // Verify dynamic signals
    EXPECT_DOUBLE_EQ(dst.time(), 1.234);
    auto pos = dst.get_vec3("position");
    EXPECT_DOUBLE_EQ(pos.x(), 1.0);
    EXPECT_DOUBLE_EQ(pos.y(), 2.0);
    EXPECT_DOUBLE_EQ(pos.z(), 3.0);
    EXPECT_EQ(dst.get_int32("phase"), 5);

    // Static signals are NOT included in dynamic serialization
    // (mass will be 0 in dst since it wasn't serialized)
}

TEST_F(FrameSerializerTest, SerializeStaticsRoundTrip) {
    FrameSerializer serializer(schema_);

    Frame src(schema_);
    src.set("mass", 100.0);

    auto bytes = serializer.serialize_statics(src);
    EXPECT_EQ(bytes.size(), 8);

    Frame dst(schema_);
    serializer.deserialize_statics(bytes, dst);

    EXPECT_DOUBLE_EQ(dst.get_double("mass"), 100.0);
}

TEST_F(FrameSerializerTest, FullFrameRoundTrip) {
    FrameSerializer serializer(schema_);

    Frame src(schema_);
    src.set_time(1.234);
    src.set("position", Eigen::Vector3d(1, 2, 3));
    src.set("phase", int32_t{5});
    src.set("mass", 100.0);

    // Serialize both static and dynamic
    auto statics = serializer.serialize_statics(src);
    auto dynamics = serializer.serialize(src);

    // Deserialize both
    Frame dst(schema_);
    serializer.deserialize_statics(statics, dst);
    serializer.deserialize(dynamics, dst);

    // Verify everything
    EXPECT_DOUBLE_EQ(dst.time(), 1.234);
    auto pos = dst.get_vec3("position");
    EXPECT_DOUBLE_EQ(pos.x(), 1.0);
    EXPECT_DOUBLE_EQ(pos.y(), 2.0);
    EXPECT_DOUBLE_EQ(pos.z(), 3.0);
    EXPECT_EQ(dst.get_int32("phase"), 5);
    EXPECT_DOUBLE_EQ(dst.get_double("mass"), 100.0);
}

// =============================================================================
// Error Handling
// =============================================================================

TEST_F(FrameSerializerTest, DeserializeTooSmallThrows) {
    FrameSerializer serializer(schema_);
    Frame frame(schema_);

    std::vector<std::byte> tiny(4);
    EXPECT_THROW(serializer.deserialize(std::span(tiny), frame),
                 std::runtime_error);
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F(FrameSerializerTest, AllDynamicSchema) {
    TelemetrySchema all_dynamic;
    all_dynamic.add_double("a").add_double("b");

    FrameSerializer serializer(all_dynamic);
    EXPECT_EQ(serializer.static_size_bytes(), 0);
    EXPECT_EQ(serializer.dynamic_size_bytes(), 24); // 8 (time) + 16 (signals)
}

TEST_F(FrameSerializerTest, AllStaticSchema) {
    TelemetrySchema all_static;
    all_static.add_double("a", SignalLifecycle::Static)
        .add_double("b", SignalLifecycle::Static);

    FrameSerializer serializer(all_static);
    EXPECT_EQ(serializer.static_size_bytes(), 16);
    EXPECT_EQ(serializer.dynamic_size_bytes(), 8); // Just time
}
