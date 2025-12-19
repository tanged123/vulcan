#include <gtest/gtest.h>
#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/Units.hpp>

#include <janus/janus.hpp>

// ============================================
// Coordinate Frame Placeholder Tests
// ============================================
TEST(CoordinateFrames, WGS84Constants) {
    // Verify WGS84 constants are defined correctly
    EXPECT_NEAR(vulcan::constants::wgs84::a, 6378137.0, 1.0);
    EXPECT_NEAR(vulcan::constants::wgs84::f, 1.0 / 298.257223563, 1e-12);

    // Verify semi-minor axis calculation
    double b_expected =
        vulcan::constants::wgs84::a * (1.0 - vulcan::constants::wgs84::f);
    EXPECT_NEAR(vulcan::constants::wgs84::b, b_expected, 1.0);
}

TEST(CoordinateFrames, EarthConstants) {
    // Verify Earth gravitational parameter
    EXPECT_NEAR(vulcan::constants::earth::mu, 3.986004418e14, 1e8);

    // Verify Earth angular velocity
    EXPECT_NEAR(vulcan::constants::earth::omega, 7.292115e-5, 1e-10);
}

TEST(UnitConversions, AngularConversions) {
    // Degrees to radians
    double rad = vulcan::units::deg_to_rad(180.0);
    EXPECT_NEAR(rad, vulcan::constants::angle::pi, 1e-10);

    // Radians to degrees
    double deg = vulcan::units::rad_to_deg(vulcan::constants::angle::pi);
    EXPECT_NEAR(deg, 180.0, 1e-10);
}

TEST(UnitConversions, LengthConversions) {
    // Feet to meters
    double m = vulcan::units::ft_to_m(1.0);
    EXPECT_NEAR(m, 0.3048, 1e-6);

    // Nautical miles to meters
    double nm_m = vulcan::units::nm_to_m(1.0);
    EXPECT_NEAR(nm_m, 1852.0, 0.1);
}

// ============================================
// EarthModel Tests
// ============================================
TEST(EarthModel, WGS84_Constants) {
    auto model = vulcan::EarthModel::WGS84();

    // Primary parameters
    EXPECT_DOUBLE_EQ(model.a, vulcan::constants::wgs84::a);
    EXPECT_DOUBLE_EQ(model.f, vulcan::constants::wgs84::f);
    EXPECT_DOUBLE_EQ(model.omega, vulcan::constants::wgs84::omega);
    EXPECT_DOUBLE_EQ(model.mu, vulcan::constants::wgs84::mu);

    // Derived parameters
    EXPECT_NEAR(model.b, vulcan::constants::wgs84::b, 0.001);
    EXPECT_NEAR(model.e2, vulcan::constants::wgs84::e2, 1e-15);
    EXPECT_NEAR(model.e_prime2, vulcan::constants::wgs84::e_prime2, 1e-15);
}

TEST(EarthModel, Spherical_NoFlattening) {
    auto model = vulcan::EarthModel::Spherical();

    // Spherical means no flattening
    EXPECT_DOUBLE_EQ(model.f, 0.0);

    // Semi-major = semi-minor for a sphere
    EXPECT_DOUBLE_EQ(model.a, model.b);

    // Zero eccentricity for a sphere
    EXPECT_DOUBLE_EQ(model.e2, 0.0);
    EXPECT_DOUBLE_EQ(model.e_prime2, 0.0);

    // Uses mean Earth radius
    EXPECT_DOUBLE_EQ(model.a, vulcan::constants::earth::R_mean);
}

TEST(EarthModel, DerivedQuantities) {
    // Test derived quantity formulas with a custom model
    double a = 6378137.0;
    double f = 1.0 / 298.257223563;

    vulcan::EarthModel model(a, f, 7.292115e-5, 3.986004418e14);

    // b = a * (1 - f)
    double b_expected = a * (1.0 - f);
    EXPECT_NEAR(model.b, b_expected, 1e-10);

    // e² = 2f - f²
    double e2_expected = 2.0 * f - f * f;
    EXPECT_NEAR(model.e2, e2_expected, 1e-15);

    // e'² = e² / (1 - e²)
    double e_prime2_expected = e2_expected / (1.0 - e2_expected);
    EXPECT_NEAR(model.e_prime2, e_prime2_expected, 1e-15);
}

// ============================================
// Earth Rotation Model Tests
// ============================================
TEST(EarthRotation, ConstantOmega_ZeroTime) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // At t=0, angle should be theta0 (default 0)
    EXPECT_DOUBLE_EQ(rotation.gmst(0.0), 0.0);
}

TEST(EarthRotation, ConstantOmega_OneDay) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // After one sidereal day (~86164.1 seconds), should complete one rotation
    // Sidereal day = 2π / ω
    double sidereal_day =
        2.0 * vulcan::constants::angle::pi / vulcan::constants::wgs84::omega;
    double angle = rotation.gmst(sidereal_day);

    EXPECT_NEAR(angle, 2.0 * vulcan::constants::angle::pi, 1e-6);
}

TEST(EarthRotation, ConstantOmega_QuarterDay) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // After ~6 hours, should rotate ~90 degrees
    double quarter_sidereal_day =
        0.5 * vulcan::constants::angle::pi / vulcan::constants::wgs84::omega;
    double angle = rotation.gmst(quarter_sidereal_day);

    EXPECT_NEAR(angle, vulcan::constants::angle::pi / 2.0, 1e-6);
}

TEST(EarthRotation, ConstantOmega_WithInitialAngle) {
    double theta0 = vulcan::constants::angle::pi / 4.0; // 45 degrees
    vulcan::ConstantOmegaRotation rotation(vulcan::constants::wgs84::omega,
                                           theta0);

    // At t=0, angle should be theta0
    EXPECT_DOUBLE_EQ(rotation.gmst(0.0), theta0);

    // After some time, angle should be theta0 + omega*t
    double t = 3600.0; // 1 hour
    double expected = theta0 + vulcan::constants::wgs84::omega * t;
    EXPECT_NEAR(rotation.gmst(t), expected, 1e-10);
}

TEST(EarthRotation, GMSTRotation_Basic) {
    vulcan::GMSTRotation rotation;

    // GMST at J2000.0 (t=0) should be ~280 degrees (known value)
    // The formula gives GMST in radians, normalized to [0, 2π)
    double gmst_0 = rotation.gmst(0.0);

    // Just verify it returns a reasonable angle (0 to 2π)
    EXPECT_GE(gmst_0, 0.0);
    EXPECT_LT(gmst_0, 2.0 * vulcan::constants::angle::pi);
}

TEST(EarthRotation, PolymorphicInterface) {
    // Test that we can use EarthRotationModel polymorphically
    std::unique_ptr<vulcan::EarthRotationModel> rotation =
        std::make_unique<vulcan::ConstantOmegaRotation>(
            vulcan::constants::wgs84::omega);

    double angle = rotation->gmst(3600.0);
    double expected = vulcan::constants::wgs84::omega * 3600.0;
    EXPECT_NEAR(angle, expected, 1e-10);
}

// ============================================
// CoordinateFrame Tests
// ============================================
TEST(CoordinateFrame, ECEF_Identity) {
    auto ecef = vulcan::CoordinateFrame<double>::ecef();

    // ECEF is the identity frame
    EXPECT_TRUE(ecef.is_valid());

    // Basis vectors should be standard basis
    EXPECT_NEAR(ecef.x_axis(0), 1.0, 1e-10);
    EXPECT_NEAR(ecef.x_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(ecef.x_axis(2), 0.0, 1e-10);

    EXPECT_NEAR(ecef.y_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(ecef.y_axis(1), 1.0, 1e-10);
    EXPECT_NEAR(ecef.y_axis(2), 0.0, 1e-10);

    EXPECT_NEAR(ecef.z_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(ecef.z_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(ecef.z_axis(2), 1.0, 1e-10);

    // Origin at Earth center
    EXPECT_NEAR(ecef.origin(0), 0.0, 1e-10);
    EXPECT_NEAR(ecef.origin(1), 0.0, 1e-10);
    EXPECT_NEAR(ecef.origin(2), 0.0, 1e-10);
}

TEST(CoordinateFrame, ECEF_Roundtrip) {
    auto ecef = vulcan::CoordinateFrame<double>::ecef();

    vulcan::Vec3<double> v;
    v << 1000.0, 2000.0, 3000.0;

    // ECEF to local and back should be identity
    auto v_local = ecef.from_ecef(v);
    auto v_back = ecef.to_ecef(v_local);

    EXPECT_NEAR(v_back(0), v(0), 1e-10);
    EXPECT_NEAR(v_back(1), v(1), 1e-10);
    EXPECT_NEAR(v_back(2), v(2), 1e-10);
}

TEST(CoordinateFrame, ECI_ZeroTime) {
    // At t=0 (gmst=0), ECI should align with ECEF
    auto eci = vulcan::CoordinateFrame<double>::eci(0.0);

    EXPECT_TRUE(eci.is_valid());

    // Should be same as ECEF
    EXPECT_NEAR(eci.x_axis(0), 1.0, 1e-10);
    EXPECT_NEAR(eci.y_axis(1), 1.0, 1e-10);
    EXPECT_NEAR(eci.z_axis(2), 1.0, 1e-10);
}

TEST(CoordinateFrame, ECI_QuarterRotation) {
    // At gmst = π/2, ECI X should point to ECEF -Y
    double gmst = vulcan::constants::angle::pi / 2.0;
    auto eci = vulcan::CoordinateFrame<double>::eci(gmst);

    EXPECT_TRUE(eci.is_valid());

    // ECI X-axis should point toward ECEF (0, -1, 0)
    EXPECT_NEAR(eci.x_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(eci.x_axis(1), -1.0, 1e-10);
    EXPECT_NEAR(eci.x_axis(2), 0.0, 1e-10);

    // ECI Y-axis should point toward ECEF (1, 0, 0)
    EXPECT_NEAR(eci.y_axis(0), 1.0, 1e-10);
    EXPECT_NEAR(eci.y_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(eci.y_axis(2), 0.0, 1e-10);

    // Z-axis unchanged
    EXPECT_NEAR(eci.z_axis(2), 1.0, 1e-10);
}

TEST(CoordinateFrame, ECI_ECEF_Transform) {
    double gmst = vulcan::constants::angle::pi / 4.0; // 45 degrees
    auto eci = vulcan::CoordinateFrame<double>::eci(gmst);

    // A vector pointing along ECEF X
    vulcan::Vec3<double> v_ecef;
    v_ecef << 1.0, 0.0, 0.0;

    // Transform to ECI
    auto v_eci = eci.from_ecef(v_ecef);

    // In ECI, this should be rotated by -gmst about Z
    double c = std::cos(gmst);
    double s = std::sin(gmst);
    EXPECT_NEAR(v_eci(0), c, 1e-10);
    EXPECT_NEAR(v_eci(1), s, 1e-10);
    EXPECT_NEAR(v_eci(2), 0.0, 1e-10);
}

TEST(CoordinateFrame, NED_Equator) {
    // NED at equator, prime meridian (lon=0, lat=0)
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);

    EXPECT_TRUE(ned.is_valid());

    // North should point to +Z (ECEF)
    EXPECT_NEAR(ned.x_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(ned.x_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(ned.x_axis(2), 1.0, 1e-10);

    // East should point to +Y (ECEF)
    EXPECT_NEAR(ned.y_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(ned.y_axis(1), 1.0, 1e-10);
    EXPECT_NEAR(ned.y_axis(2), 0.0, 1e-10);

    // Down should point to -X (ECEF)
    EXPECT_NEAR(ned.z_axis(0), -1.0, 1e-10);
    EXPECT_NEAR(ned.z_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(ned.z_axis(2), 0.0, 1e-10);
}

TEST(CoordinateFrame, NED_NorthPole) {
    // NED at North Pole (lon=0, lat=π/2)
    double lat = vulcan::constants::angle::pi / 2.0;
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, lat);

    EXPECT_TRUE(ned.is_valid());

    // At North Pole, Down should point to -Z (ECEF)
    EXPECT_NEAR(ned.z_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(ned.z_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(ned.z_axis(2), -1.0, 1e-10);
}

TEST(CoordinateFrame, ENU_Equator) {
    // ENU at equator, prime meridian
    auto enu = vulcan::CoordinateFrame<double>::enu(0.0, 0.0);

    EXPECT_TRUE(enu.is_valid());

    // East should point to +Y (ECEF)
    EXPECT_NEAR(enu.x_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(enu.x_axis(1), 1.0, 1e-10);
    EXPECT_NEAR(enu.x_axis(2), 0.0, 1e-10);

    // North should point to +Z (ECEF)
    EXPECT_NEAR(enu.y_axis(0), 0.0, 1e-10);
    EXPECT_NEAR(enu.y_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(enu.y_axis(2), 1.0, 1e-10);

    // Up should point to +X (ECEF)
    EXPECT_NEAR(enu.z_axis(0), 1.0, 1e-10);
    EXPECT_NEAR(enu.z_axis(1), 0.0, 1e-10);
    EXPECT_NEAR(enu.z_axis(2), 0.0, 1e-10);
}

TEST(CoordinateFrame, DCM_Orthonormal) {
    auto ned = vulcan::CoordinateFrame<double>::ned(0.5, 0.3);
    auto dcm = ned.dcm();

    // DCM should be orthonormal: R^T * R = I
    auto eye = dcm.transpose() * dcm;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            EXPECT_NEAR(eye(i, j), expected, 1e-10);
        }
    }
}

TEST(CoordinateFrame, TransformVector_Roundtrip) {
    auto ned = vulcan::CoordinateFrame<double>::ned(0.5, 0.3);
    auto ecef = vulcan::CoordinateFrame<double>::ecef();

    vulcan::Vec3<double> v;
    v << 100.0, 200.0, 300.0;

    // ECEF -> NED -> ECEF should be identity
    auto v_ned = vulcan::transform_vector(v, ecef, ned);
    auto v_back = vulcan::transform_vector(v_ned, ned, ecef);

    EXPECT_NEAR(v_back(0), v(0), 1e-10);
    EXPECT_NEAR(v_back(1), v(1), 1e-10);
    EXPECT_NEAR(v_back(2), v(2), 1e-10);
}

TEST(CoordinateFrame, Symbolic_ECI_Evaluation) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic GMST
    Scalar gmst = janus::sym("gmst");

    // Create ECI frame symbolically
    auto eci = vulcan::CoordinateFrame<Scalar>::eci(gmst);

    // Verify symbolic expressions were created
    EXPECT_FALSE(eci.x_axis(0).is_constant());

    // Create function to evaluate ECI basis vectors
    janus::Function f("eci_basis", {gmst},
                      {eci.x_axis(0), eci.x_axis(1), eci.x_axis(2),
                       eci.y_axis(0), eci.y_axis(1), eci.y_axis(2)});

    // Test at gmst = π/4 (45 degrees)
    double test_gmst = vulcan::constants::angle::pi / 4.0;
    auto result = f({test_gmst});

    // Compare against direct numeric computation
    auto eci_numeric = vulcan::CoordinateFrame<double>::eci(test_gmst);

    EXPECT_NEAR(result[0](0, 0), eci_numeric.x_axis(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), eci_numeric.x_axis(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), eci_numeric.x_axis(2), 1e-12);
    EXPECT_NEAR(result[3](0, 0), eci_numeric.y_axis(0), 1e-12);
    EXPECT_NEAR(result[4](0, 0), eci_numeric.y_axis(1), 1e-12);
    EXPECT_NEAR(result[5](0, 0), eci_numeric.y_axis(2), 1e-12);
}

TEST(CoordinateFrame, Symbolic_NED_Evaluation) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic coordinates
    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");

    // Create NED frame symbolically
    auto ned = vulcan::CoordinateFrame<Scalar>::ned(lon, lat);

    // Verify symbolic expressions
    EXPECT_FALSE(ned.x_axis(0).is_constant());

    // Create function to evaluate all NED basis vectors
    janus::Function f("ned_basis", {lon, lat},
                      {ned.x_axis(0), ned.x_axis(1), ned.x_axis(2),
                       ned.y_axis(0), ned.y_axis(1), ned.y_axis(2),
                       ned.z_axis(0), ned.z_axis(1), ned.z_axis(2)});

    // Test at Washington DC (approx 38.9°N, 77.0°W)
    double test_lon = -77.0 * vulcan::constants::angle::deg2rad;
    double test_lat = 38.9 * vulcan::constants::angle::deg2rad;
    auto result = f({test_lon, test_lat});

    // Compare against direct numeric computation
    auto ned_numeric = vulcan::CoordinateFrame<double>::ned(test_lon, test_lat);

    EXPECT_NEAR(result[0](0, 0), ned_numeric.x_axis(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), ned_numeric.x_axis(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), ned_numeric.x_axis(2), 1e-12);
    EXPECT_NEAR(result[3](0, 0), ned_numeric.y_axis(0), 1e-12);
    EXPECT_NEAR(result[4](0, 0), ned_numeric.y_axis(1), 1e-12);
    EXPECT_NEAR(result[5](0, 0), ned_numeric.y_axis(2), 1e-12);
    EXPECT_NEAR(result[6](0, 0), ned_numeric.z_axis(0), 1e-12);
    EXPECT_NEAR(result[7](0, 0), ned_numeric.z_axis(1), 1e-12);
    EXPECT_NEAR(result[8](0, 0), ned_numeric.z_axis(2), 1e-12);
}

TEST(CoordinateFrame, Symbolic_Transform_Evaluation) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic inputs
    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");
    Scalar vx = janus::sym("vx");
    Scalar vy = janus::sym("vy");
    Scalar vz = janus::sym("vz");

    // Create symbolic NED frame
    auto ned = vulcan::CoordinateFrame<Scalar>::ned(lon, lat);

    // Create symbolic ECEF vector
    vulcan::Vec3<Scalar> v_ecef;
    v_ecef << vx, vy, vz;

    // Transform to NED
    auto v_ned = ned.from_ecef(v_ecef);

    // Create function
    janus::Function f("ecef_to_ned", {lon, lat, vx, vy, vz},
                      {v_ned(0), v_ned(1), v_ned(2)});

    // Test with concrete values
    double test_lon = 0.5;
    double test_lat = 0.3;
    vulcan::Vec3<double> test_v;
    test_v << 100.0, 200.0, 300.0;

    auto result = f({test_lon, test_lat, test_v(0), test_v(1), test_v(2)});

    // Compare against direct numeric computation
    auto ned_numeric = vulcan::CoordinateFrame<double>::ned(test_lon, test_lat);
    auto v_ned_numeric = ned_numeric.from_ecef(test_v);

    EXPECT_NEAR(result[0](0, 0), v_ned_numeric(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), v_ned_numeric(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), v_ned_numeric(2), 1e-10);
}

TEST(CoordinateFrame, Symbolic_Roundtrip_Evaluation) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic GMST and vector
    Scalar gmst = janus::sym("gmst");
    Scalar vx = janus::sym("vx");
    Scalar vy = janus::sym("vy");
    Scalar vz = janus::sym("vz");

    // Create symbolic ECI frame
    auto eci = vulcan::CoordinateFrame<Scalar>::eci(gmst);

    // Create symbolic ECEF vector
    vulcan::Vec3<Scalar> v_ecef;
    v_ecef << vx, vy, vz;

    // Roundtrip: ECEF -> ECI -> ECEF
    auto v_eci = eci.from_ecef(v_ecef);
    auto v_back = eci.to_ecef(v_eci);

    // Create function
    janus::Function f("eci_roundtrip", {gmst, vx, vy, vz},
                      {v_back(0), v_back(1), v_back(2)});

    // Test with concrete values
    double test_gmst = 1.234;
    vulcan::Vec3<double> test_v;
    test_v << 1000.0, 2000.0, 3000.0;

    auto result = f({test_gmst, test_v(0), test_v(1), test_v(2)});

    // Roundtrip should return original vector
    EXPECT_NEAR(result[0](0, 0), test_v(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), test_v(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), test_v(2), 1e-10);
}
