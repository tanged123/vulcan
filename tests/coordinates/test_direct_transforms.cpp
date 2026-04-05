#include <gtest/gtest.h>

#include <vulcan/coordinates/FrameContext.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

namespace {

using vulcan::FRAME_BODY;
using vulcan::FRAME_NED;
using vulcan::Quantity;
using vulcan::Vec3;
using vulcan::units::rad;

TEST(DirectTransforms, ParentChildChainLengthIsOne) {
    vulcan::FrameContext<double> ctx;
    ctx.set_ecef(0.3);
    ctx.set_ned(Quantity<rad>(0.2), Quantity<rad>(-0.4));
    ctx.set_body_euler(0.4, -0.1, 0.2);

    const auto body_to_ned = ctx.chain(FRAME_BODY, FRAME_NED);
    const auto ned_to_body = ctx.chain(FRAME_NED, FRAME_BODY);
    EXPECT_EQ(body_to_ned.length(), 1);
    EXPECT_EQ(ned_to_body.length(), 1);
}

TEST(DirectTransforms, ParentChildPositionRoundtrip) {
    vulcan::FrameContext<double> ctx;
    ctx.set_ecef(0.0);
    ctx.set_ned(Quantity<rad>(0.1), Quantity<rad>(0.2));
    ctx.set_body_euler(0.0, 0.0, 0.0);

    Vec3<double> p_body;
    p_body << 10.0, -2.0, 5.0;

    const auto p_ned = ctx.transform_position(p_body, FRAME_BODY, FRAME_NED);
    const auto p_back = ctx.transform_position(p_ned, FRAME_NED, FRAME_BODY);

    EXPECT_NEAR(p_back(0), p_body(0), 1e-12);
    EXPECT_NEAR(p_back(1), p_body(1), 1e-12);
    EXPECT_NEAR(p_back(2), p_body(2), 1e-12);
}

} // namespace
