#include <gtest/gtest.h>

#include <vulcan/coordinates/FrameContext.hpp>

#include <janus/janus.hpp>

namespace {

using vulcan::FRAME_BODY;
using vulcan::FRAME_ECI;
using vulcan::Vec3;

TEST(SymbolicFrameContext, BuildsAndEvaluatesBodyToECIChain) {
    using Scalar = janus::SymbolicScalar;

    Scalar gmst = janus::sym("gmst");
    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");
    Scalar yaw = janus::sym("yaw");
    Scalar pitch = janus::sym("pitch");
    Scalar roll = janus::sym("roll");
    Scalar vx = janus::sym("vx");
    Scalar vy = janus::sym("vy");
    Scalar vz = janus::sym("vz");

    vulcan::FrameContext<Scalar> sym_ctx;
    sym_ctx.set_ecef(gmst);
    sym_ctx.set_ned(lon, lat);
    sym_ctx.set_body_euler(yaw, pitch, roll);

    Vec3<Scalar> v_body;
    v_body << vx, vy, vz;

    const auto v_eci = sym_ctx.transform(v_body, FRAME_BODY, FRAME_ECI);
    EXPECT_FALSE(v_eci(0).is_constant());

    janus::Function f("frame_ctx_body_to_eci",
                      {gmst, lon, lat, yaw, pitch, roll, vx, vy, vz},
                      {v_eci(0), v_eci(1), v_eci(2)});

    const double gmst_val = 0.25;
    const double lon_val = 0.1;
    const double lat_val = 0.3;
    const double yaw_val = 0.2;
    const double pitch_val = -0.05;
    const double roll_val = 0.1;
    const Vec3<double> v_body_val = [] {
        Vec3<double> v;
        v << 12.0, -3.5, 0.8;
        return v;
    }();

    auto result = f({gmst_val, lon_val, lat_val, yaw_val, pitch_val, roll_val,
                     v_body_val(0), v_body_val(1), v_body_val(2)});

    vulcan::FrameContext<double> num_ctx;
    num_ctx.set_ecef(gmst_val);
    num_ctx.set_ned(lon_val, lat_val);
    num_ctx.set_body_euler(yaw_val, pitch_val, roll_val);
    const auto expected = num_ctx.transform(v_body_val, FRAME_BODY, FRAME_ECI);

    EXPECT_NEAR(result[0](0, 0), expected(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), expected(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), expected(2), 1e-10);
}

} // namespace
