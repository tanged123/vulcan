#include <gtest/gtest.h>

#include <vulcan/coordinates/FrameRegistry.hpp>

#include <string>

namespace {

using vulcan::BuiltinFrame;
using vulcan::FRAME_BODY;
using vulcan::FRAME_ECEF;
using vulcan::FRAME_ECI;
using vulcan::FRAME_ENU;
using vulcan::FRAME_NED;
using vulcan::FRAME_WIND;
using vulcan::FrameID;
using vulcan::FrameRegistry;

TEST(FrameRegistry, BuiltinTreeStructureAndDepths) {
    const FrameRegistry registry;

    EXPECT_TRUE(registry.has_frame(FRAME_ECI));
    EXPECT_TRUE(registry.has_frame(FRAME_ECEF));
    EXPECT_TRUE(registry.has_frame(FRAME_NED));
    EXPECT_TRUE(registry.has_frame(FRAME_BODY));
    EXPECT_TRUE(registry.has_frame(FRAME_WIND));

    EXPECT_EQ(registry.parent_of(FRAME_ECI), FRAME_ECI);
    EXPECT_EQ(registry.parent_of(FRAME_ECEF), FRAME_ECI);
    EXPECT_EQ(registry.parent_of(FRAME_NED), FRAME_ECEF);
    EXPECT_EQ(registry.parent_of(FRAME_BODY), FRAME_NED);
    EXPECT_EQ(registry.parent_of(FRAME_WIND), FRAME_BODY);

    EXPECT_EQ(registry.depth_of(FRAME_ECI), 0);
    EXPECT_EQ(registry.depth_of(FRAME_ECEF), 1);
    EXPECT_EQ(registry.depth_of(FRAME_NED), 2);
    EXPECT_EQ(registry.depth_of(FRAME_BODY), 3);
    EXPECT_EQ(registry.depth_of(FRAME_WIND), 4);
}

TEST(FrameRegistry, PathAndLcaQueries) {
    const FrameRegistry registry;

    const auto ned_to_body = registry.find_path(FRAME_NED, FRAME_BODY);
    ASSERT_TRUE(ned_to_body.is_valid());
    EXPECT_EQ(ned_to_body.lca, FRAME_NED);
    EXPECT_EQ(ned_to_body.length(), 1);
    ASSERT_EQ(ned_to_body.frames.size(), 2U);
    EXPECT_EQ(ned_to_body.frames[0], FRAME_NED);
    EXPECT_EQ(ned_to_body.frames[1], FRAME_BODY);

    const auto ned_to_enu = registry.find_path(FRAME_NED, FRAME_ENU);
    ASSERT_TRUE(ned_to_enu.is_valid());
    EXPECT_EQ(ned_to_enu.lca, FRAME_ECEF);
    ASSERT_EQ(ned_to_enu.frames.size(), 3U);
    EXPECT_EQ(ned_to_enu.frames[0], FRAME_NED);
    EXPECT_EQ(ned_to_enu.frames[1], FRAME_ECEF);
    EXPECT_EQ(ned_to_enu.frames[2], FRAME_ENU);

    const auto body_to_eci = registry.find_path(FRAME_BODY, FRAME_ECI);
    ASSERT_TRUE(body_to_eci.is_valid());
    EXPECT_EQ(body_to_eci.lca, FRAME_ECI);
    ASSERT_EQ(body_to_eci.frames.size(), 4U);
    EXPECT_EQ(body_to_eci.frames[0], FRAME_BODY);
    EXPECT_EQ(body_to_eci.frames[1], FRAME_NED);
    EXPECT_EQ(body_to_eci.frames[2], FRAME_ECEF);
    EXPECT_EQ(body_to_eci.frames[3], FRAME_ECI);
}

TEST(FrameRegistry, UserFrameRegistrationAndPath) {
    FrameRegistry registry;

    const FrameID sensor = registry.register_frame("Sensor", FRAME_BODY);
    EXPECT_FALSE(sensor.is_builtin());
    EXPECT_EQ(registry.parent_of(sensor), FRAME_BODY);
    EXPECT_EQ(registry.depth_of(sensor), registry.depth_of(FRAME_BODY) + 1);

    const auto sensor_to_enu = registry.find_path(sensor, FRAME_ENU);
    ASSERT_TRUE(sensor_to_enu.is_valid());
    ASSERT_EQ(sensor_to_enu.frames.front(), sensor);
    ASSERT_EQ(sensor_to_enu.frames.back(), FRAME_ENU);
}

TEST(FrameRegistry, MaxDepthGuard) {
    FrameRegistry registry;
    FrameID parent = FRAME_WIND;

    // Depth(FRAME_WIND) is 4, so 28 extra levels is still valid (depth 32).
    for (int i = 0; i < 28; ++i) {
        parent = registry.register_frame("Deep_" + std::to_string(i), parent);
    }

    EXPECT_THROW(
        {
            const auto id = registry.register_frame("TooDeep", parent);
            static_cast<void>(id);
        },
        vulcan::CoordinateError);
}

TEST(FrameID, BuiltinConversion) {
    const FrameID body = BuiltinFrame::Body;
    EXPECT_TRUE(body.is_builtin());
    EXPECT_EQ(body, FRAME_BODY);
}

} // namespace
