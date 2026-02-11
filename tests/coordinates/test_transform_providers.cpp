#include <gtest/gtest.h>

#include <vulcan/coordinates/FramePrimitives.hpp>
#include <vulcan/coordinates/FrameVehicle.hpp>
#include <vulcan/coordinates/providers/ECEFProvider.hpp>
#include <vulcan/coordinates/providers/EarthProviders.hpp>
#include <vulcan/coordinates/providers/VehicleProviders.hpp>
#include <vulcan/core/Constants.hpp>

namespace {

using vulcan::CoordinateFrame;
using vulcan::Vec3;

TEST(TransformProviders, ECEFProviderMatchesCoordinateFrameECI) {
    const double gmst = 0.73;
    const vulcan::ECEFProvider<double> provider(gmst);
    const auto eci = CoordinateFrame<double>::eci(gmst);

    Vec3<double> v_ecef;
    v_ecef << 1200.0, -3400.0, 5600.0;

    const auto v_eci_provider = provider.to_parent(v_ecef);
    const auto v_eci_expected = eci.from_ecef(v_ecef);
    EXPECT_NEAR(v_eci_provider(0), v_eci_expected(0), 1e-12);
    EXPECT_NEAR(v_eci_provider(1), v_eci_expected(1), 1e-12);
    EXPECT_NEAR(v_eci_provider(2), v_eci_expected(2), 1e-12);

    const auto v_back = provider.from_parent(v_eci_provider);
    EXPECT_NEAR(v_back(0), v_ecef(0), 1e-12);
    EXPECT_NEAR(v_back(1), v_ecef(1), 1e-12);
    EXPECT_NEAR(v_back(2), v_ecef(2), 1e-12);
}

TEST(TransformProviders, NEDProviderMatchesCoordinateFrame) {
    const double lon = -77.0367 * vulcan::constants::angle::deg2rad;
    const double lat = 38.8951 * vulcan::constants::angle::deg2rad;
    const vulcan::NEDProvider<double> provider(lon, lat);
    const auto ned = CoordinateFrame<double>::ned(lon, lat);

    Vec3<double> v_ned;
    v_ned << 120.0, -33.0, 4.0;

    const auto v_ecef_provider = provider.to_parent(v_ned);
    const auto v_ecef_expected = ned.to_ecef(v_ned);
    EXPECT_NEAR(v_ecef_provider(0), v_ecef_expected(0), 1e-12);
    EXPECT_NEAR(v_ecef_provider(1), v_ecef_expected(1), 1e-12);
    EXPECT_NEAR(v_ecef_provider(2), v_ecef_expected(2), 1e-12);
}

TEST(TransformProviders, BodyProviderMatchesBodyFrameEuler) {
    const double lon = 0.4;
    const double lat = 0.3;
    const double yaw = 0.8;
    const double pitch = -0.2;
    const double roll = 0.1;

    const auto ned = CoordinateFrame<double>::ned(lon, lat);
    const auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);
    const vulcan::BodyProvider<double> provider(yaw, pitch, roll);

    Vec3<double> v_body;
    v_body << 12.0, -7.0, 5.5;

    const auto v_ned_provider = provider.to_parent(v_body);
    const auto v_ned_expected = ned.from_ecef(body.to_ecef(v_body));
    EXPECT_NEAR(v_ned_provider(0), v_ned_expected(0), 1e-12);
    EXPECT_NEAR(v_ned_provider(1), v_ned_expected(1), 1e-12);
    EXPECT_NEAR(v_ned_provider(2), v_ned_expected(2), 1e-12);
}

TEST(TransformProviders, WindProviderRoundtrip) {
    const double alpha = 8.0 * vulcan::constants::angle::deg2rad;
    const double beta = -3.0 * vulcan::constants::angle::deg2rad;
    const vulcan::WindProvider<double> provider(alpha, beta);

    Vec3<double> v_wind;
    v_wind << 80.0, 2.0, -1.5;

    const auto v_body = provider.to_parent(v_wind);
    const auto v_wind_back = provider.from_parent(v_body);

    EXPECT_NEAR(v_wind_back(0), v_wind(0), 1e-12);
    EXPECT_NEAR(v_wind_back(1), v_wind(1), 1e-12);
    EXPECT_NEAR(v_wind_back(2), v_wind(2), 1e-12);
}

} // namespace
