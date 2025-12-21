/**
 * @file test_telemetry_schema.cpp
 * @brief Unit tests for TelemetrySchema
 */

#include <gtest/gtest.h>
#include <vulcan/io/TelemetrySchema.hpp>

using namespace vulcan::io;

// =============================================================================
// Schema Building
// =============================================================================

TEST(TelemetrySchema, AddDoubleSignal) {
    TelemetrySchema schema;
    schema.add_double("altitude", SignalLifecycle::Dynamic, "m");

    EXPECT_EQ(schema.signal_count(), 1);
    EXPECT_TRUE(schema.has_signal("altitude"));

    const auto &sig = schema.signal("altitude");
    EXPECT_EQ(sig.name, "altitude");
    EXPECT_EQ(sig.type, SignalType::Double);
    EXPECT_EQ(sig.lifecycle, SignalLifecycle::Dynamic);
    EXPECT_EQ(sig.unit, "m");
    EXPECT_EQ(sig.offset, 0);
}

TEST(TelemetrySchema, AddInt32Signal) {
    TelemetrySchema schema;
    schema.add_int32("gnc.phase", SignalLifecycle::Dynamic, "enum");

    EXPECT_EQ(schema.signal_count(), 1);

    const auto &sig = schema.signal("gnc.phase");
    EXPECT_EQ(sig.type, SignalType::Int32);
    EXPECT_EQ(sig.semantic, "enum");
}

TEST(TelemetrySchema, AddInt64Signal) {
    TelemetrySchema schema;
    schema.add_int64("timestamp", SignalLifecycle::Static);

    const auto &sig = schema.signal("timestamp");
    EXPECT_EQ(sig.type, SignalType::Int64);
    EXPECT_EQ(sig.lifecycle, SignalLifecycle::Static);
}

TEST(TelemetrySchema, AddVec3Expands) {
    TelemetrySchema schema;
    schema.add_vec3("position", SignalLifecycle::Dynamic, "m");

    EXPECT_EQ(schema.signal_count(), 3);
    EXPECT_TRUE(schema.has_signal("position.x"));
    EXPECT_TRUE(schema.has_signal("position.y"));
    EXPECT_TRUE(schema.has_signal("position.z"));

    EXPECT_EQ(schema.signal("position.x").unit, "m");
    EXPECT_EQ(schema.signal("position.y").unit, "m");
    EXPECT_EQ(schema.signal("position.z").unit, "m");
}

TEST(TelemetrySchema, AddQuatExpands) {
    TelemetrySchema schema;
    schema.add_quat("attitude");

    EXPECT_EQ(schema.signal_count(), 4);
    EXPECT_TRUE(schema.has_signal("attitude.w"));
    EXPECT_TRUE(schema.has_signal("attitude.x"));
    EXPECT_TRUE(schema.has_signal("attitude.y"));
    EXPECT_TRUE(schema.has_signal("attitude.z"));
}

TEST(TelemetrySchema, ChainedAdd) {
    TelemetrySchema schema;
    schema.add_vec3("position", SignalLifecycle::Dynamic, "m")
        .add_vec3("velocity", SignalLifecycle::Dynamic, "m/s")
        .add_int32("phase");

    EXPECT_EQ(schema.signal_count(), 7);
}

// =============================================================================
// Offset Calculation
// =============================================================================

TEST(TelemetrySchema, OffsetsAreSequential) {
    TelemetrySchema schema;
    schema.add_double("a").add_double("b").add_double("c");

    EXPECT_EQ(schema.offset("a"), 0);
    EXPECT_EQ(schema.offset("b"), 8);
    EXPECT_EQ(schema.offset("c"), 16);
}

TEST(TelemetrySchema, FrameSizeBytes) {
    TelemetrySchema schema;
    schema
        .add_vec3("position") // 3 * 8 = 24 bytes
        .add_int32("phase");  // 8 bytes (padded)

    EXPECT_EQ(schema.frame_size_bytes(), 32);
}

TEST(TelemetrySchema, DynamicStaticSplit) {
    TelemetrySchema schema;
    schema.add_double("dynamic_signal", SignalLifecycle::Dynamic)
        .add_double("static_signal", SignalLifecycle::Static)
        .add_double("another_dynamic", SignalLifecycle::Dynamic);

    auto dynamic = schema.dynamic_signals();
    auto statc = schema.static_signals();

    EXPECT_EQ(dynamic.size(), 2);
    EXPECT_EQ(statc.size(), 1);
    EXPECT_EQ(schema.dynamic_frame_size_bytes(), 16);
    EXPECT_EQ(schema.static_frame_size_bytes(), 8);
}

// =============================================================================
// Validation
// =============================================================================

TEST(TelemetrySchema, DuplicateNameThrows) {
    TelemetrySchema schema;
    schema.add_double("altitude");

    EXPECT_THROW(schema.add_double("altitude"), std::runtime_error);
}

TEST(TelemetrySchema, SignalNotFoundThrows) {
    TelemetrySchema schema;
    schema.add_double("altitude");

    EXPECT_THROW(schema.signal("nonexistent"), std::runtime_error);
    EXPECT_THROW(schema.offset("nonexistent"), std::runtime_error);
}

TEST(TelemetrySchema, EmptySchemaValidateFails) {
    TelemetrySchema schema;
    EXPECT_THROW(schema.validate(), std::runtime_error);
}

// =============================================================================
// JSON Serialization
// =============================================================================

TEST(TelemetrySchema, ToJsonContainsSignals) {
    TelemetrySchema schema;
    schema.add_double("altitude", SignalLifecycle::Dynamic, "m")
        .add_int32("phase", SignalLifecycle::Static, "enum");

    std::string json = schema.to_json();

    EXPECT_TRUE(json.find("\"altitude\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"double\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"dynamic\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"m\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"phase\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"int32\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"static\"") != std::string::npos);
}
