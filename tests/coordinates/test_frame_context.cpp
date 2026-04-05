#include <gtest/gtest.h>

#include <vulcan/coordinates/FrameContext.hpp>
#include <vulcan/coordinates/FramePrimitives.hpp>
#include <vulcan/coordinates/FrameTransforms.hpp>
#include <vulcan/coordinates/FrameVehicle.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

#include <memory>

namespace {

using vulcan::FRAME_BODY;
using vulcan::FRAME_ECI;
using vulcan::FRAME_NED;
using vulcan::Quantity;
using vulcan::Vec3;
using vulcan::units::rad;

struct OffsetAngleModel final : vulcan::EarthRotationModel {
    double offset_rad = 0.0;

    explicit OffsetAngleModel(double offset) : offset_rad(offset) {}

    [[nodiscard]] double gmst(double t_seconds) const override {
        // Deliberately different from ecef_to_eci_angle so the test verifies
        // the override path is used.
        return t_seconds;
    }

    [[nodiscard]] double ecef_to_eci_angle(double t_seconds) const override {
        return t_seconds + offset_rad;
    }
};

TEST(FrameContext, EndToEndMatchesLegacyComposition) {
    const double gmst = 0.42;
    const double lon = -77.0367 * vulcan::constants::angle::deg2rad;
    const double lat = 38.8951 * vulcan::constants::angle::deg2rad;
    const double yaw = 15.0 * vulcan::constants::angle::deg2rad;
    const double pitch = -2.0 * vulcan::constants::angle::deg2rad;
    const double roll = 5.0 * vulcan::constants::angle::deg2rad;

    vulcan::FrameContext<double> ctx;
    ctx.set_ecef(gmst);
    ctx.set_ned(Quantity<rad>(lon), Quantity<rad>(lat));
    ctx.set_body_euler(yaw, pitch, roll);

    const auto eci = vulcan::CoordinateFrame<double>::eci(gmst);
    const auto ned = vulcan::CoordinateFrame<double>::ned(lon, lat);
    const auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

    Vec3<double> v_body;
    v_body << 100.0, -20.0, 5.0;

    const auto v_ned_expected = ned.from_ecef(body.to_ecef(v_body));
    const auto v_ned_ctx = ctx.transform(v_body, FRAME_BODY, FRAME_NED);
    EXPECT_NEAR(v_ned_ctx(0), v_ned_expected(0), 1e-12);
    EXPECT_NEAR(v_ned_ctx(1), v_ned_expected(1), 1e-12);
    EXPECT_NEAR(v_ned_ctx(2), v_ned_expected(2), 1e-12);

    const auto v_eci_expected = eci.from_ecef(body.to_ecef(v_body));
    const auto v_eci_ctx =
        vulcan::transform(v_body, FRAME_BODY, FRAME_ECI, ctx);
    EXPECT_NEAR(v_eci_ctx(0), v_eci_expected(0), 1e-12);
    EXPECT_NEAR(v_eci_ctx(1), v_eci_expected(1), 1e-12);
    EXPECT_NEAR(v_eci_ctx(2), v_eci_expected(2), 1e-12);
}

TEST(FrameContext, CustomFrameRegistrationAndTransform) {
    const double gmst = 0.1;
    const double lon = 0.3;
    const double lat = 0.5;
    const double yaw = 0.2;
    const double pitch = -0.1;
    const double roll = 0.4;

    vulcan::FrameContext<double> ctx;
    ctx.set_ecef(gmst);
    ctx.set_ned(Quantity<rad>(lon), Quantity<rad>(lat));
    ctx.set_body_euler(yaw, pitch, roll);

    const auto q_body_to_ned =
        janus::Quaternion<double>::from_euler(roll, pitch, yaw);
    const auto q_sensor_to_body = janus::Quaternion<double>::from_euler(
        0.0, 10.0 * vulcan::constants::angle::deg2rad, 0.0);

    const auto sensor_id = ctx.add_frame(
        "Sensor", FRAME_BODY,
        std::make_shared<vulcan::QuaternionProvider<double>>(q_sensor_to_body));

    const auto sensor_to_ned = ctx.chain(sensor_id, FRAME_NED);
    EXPECT_EQ(sensor_to_ned.length(), 2);

    Vec3<double> v_sensor;
    v_sensor << 1.0, 0.0, 0.0;

    const auto v_ned_expected =
        q_body_to_ned.rotate(q_sensor_to_body.rotate(v_sensor));
    const auto v_ned_ctx = ctx.transform(v_sensor, sensor_id, FRAME_NED);
    EXPECT_NEAR(v_ned_ctx(0), v_ned_expected(0), 1e-12);
    EXPECT_NEAR(v_ned_ctx(1), v_ned_expected(1), 1e-12);
    EXPECT_NEAR(v_ned_ctx(2), v_ned_expected(2), 1e-12);
}

TEST(FrameContext, UsesEarthRotationModelEcefToEciAngleOverride) {
    const double t_seconds = 0.3;
    const double offset = 0.25;
    OffsetAngleModel model(offset);

    vulcan::FrameContext<double> ctx;
    ctx.set_ecef(model, t_seconds);

    Vec3<double> v_ecef;
    v_ecef << 5.0, -2.0, 1.0;

    const auto v_eci_ctx = ctx.transform(v_ecef, vulcan::FRAME_ECEF, FRAME_ECI);
    const auto v_eci_expected =
        vulcan::CoordinateFrame<double>::eci(t_seconds + offset)
            .from_ecef(v_ecef);

    EXPECT_NEAR(v_eci_ctx(0), v_eci_expected(0), 1e-12);
    EXPECT_NEAR(v_eci_ctx(1), v_eci_expected(1), 1e-12);
    EXPECT_NEAR(v_eci_ctx(2), v_eci_expected(2), 1e-12);
}

TEST(FrameContext, InvalidIdentityIdsThrow) {
    vulcan::FrameContext<double> ctx;

    vulcan::Vec3<double> v;
    v << 1.0, 2.0, 3.0;

    const vulcan::FrameID invalid(999U);
    EXPECT_THROW(
        {
            const auto out = ctx.transform(v, invalid, invalid);
            static_cast<void>(out);
        },
        vulcan::CoordinateError);
    EXPECT_THROW(
        {
            const auto out = ctx.transform_position(v, invalid, invalid);
            static_cast<void>(out);
        },
        vulcan::CoordinateError);
    EXPECT_THROW(
        {
            const auto ch = ctx.chain(invalid, invalid);
            static_cast<void>(ch);
        },
        vulcan::CoordinateError);
}

} // namespace
