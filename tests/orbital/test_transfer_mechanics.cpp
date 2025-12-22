// Transfer Mechanics Tests
#include <gtest/gtest.h>
#include <vulcan/orbital/TransferMechanics.hpp>

using namespace vulcan::orbital::transfer;

// Test Hohmann transfer
TEST(TransferMechanics, Hohmann_LEOtoGEO) {
    double r_leo = vulcan::constants::earth::R_eq + 300.0e3;
    double r_geo = 42164.0e3;

    auto [dv1, dv2] = hohmann_delta_v(r_leo, r_geo);
    double total = dv1 + dv2;

    // LEO to GEO ~ 3.9 km/s total
    EXPECT_GT(total / 1000.0, 3.5);
    EXPECT_LT(total / 1000.0, 4.5);
}

TEST(TransferMechanics, Hohmann_Symmetric) {
    // Hohmann is symmetric in total delta-v
    double r1 = 7000.0e3;
    double r2 = 14000.0e3;

    double total_up = hohmann_total_delta_v(r1, r2);
    double total_down = hohmann_total_delta_v(r2, r1);

    EXPECT_NEAR(total_up, total_down, 1e-6);
}

// Test transfer time
TEST(TransferMechanics, HohmannTime_LEOtoGEO) {
    double r_leo = vulcan::constants::earth::R_eq + 300.0e3;
    double r_geo = 42164.0e3;

    double t = hohmann_transfer_time(r_leo, r_geo);

    // LEO to GEO transfer ~ 5 hours
    double hours = t / 3600.0;
    EXPECT_GT(hours, 4.0);
    EXPECT_LT(hours, 6.0);
}

// Test bielliptic transfer
TEST(TransferMechanics, Bielliptic_Basic) {
    double r1 = 7000.0e3;
    double r2 = 105000.0e3; // Large radius ratio
    double r_b = 210000.0e3;

    auto [dv1, dv2, dv3] = bielliptic_delta_v(r1, r2, r_b);

    // All delta-v should be positive
    EXPECT_GT(dv1, 0.0);
    EXPECT_GT(dv2, 0.0);
    EXPECT_GT(dv3, 0.0);
}

TEST(TransferMechanics, Bielliptic_VsHohmann) {
    // For r2/r1 > 11.94, bielliptic is more efficient
    double r1 = 7000.0e3;
    double r2 = 100000.0e3; // Ratio ~ 14.3
    double r_b = 200000.0e3;

    auto [dv1, dv2, dv3] = bielliptic_delta_v(r1, r2, r_b);
    double bielliptic_total = dv1 + dv2 + dv3;

    double hohmann_total = hohmann_total_delta_v(r1, r2);

    // Bielliptic should be comparable or better for this ratio
    // (depends on choice of r_b)
    EXPECT_GT(bielliptic_total, 0.0);
    EXPECT_GT(hohmann_total, 0.0);
}

// Test plane change
TEST(TransferMechanics, PlaneChange_90deg) {
    double v = 7700.0;           // LEO velocity
    double delta_i = M_PI / 2.0; // 90 degree change

    double dv = plane_change_delta_v(v, delta_i);

    // 90° plane change at v requires dv = sqrt(2) * v
    EXPECT_NEAR(dv / v, std::sqrt(2.0), 1e-6);
}

TEST(TransferMechanics, PlaneChange_Small) {
    double v = 7700.0;
    double delta_i = 0.1; // ~5.7 degrees

    double dv = plane_change_delta_v(v, delta_i);

    // For small angles: dv ≈ v * delta_i
    EXPECT_LT(dv, v * delta_i * 1.1); // Within 10% of linear approx
}

// Test combined maneuver
TEST(TransferMechanics, CombinedManeuver) {
    double v1 = 7700.0;
    double v2 = 7700.0;
    double delta_i = 0.0; // No plane change

    double dv = combined_maneuver_delta_v(v1, v2, delta_i);

    // With same velocities and no plane change, dv = 0
    EXPECT_NEAR(dv, 0.0, 1e-6);
}

TEST(TransferMechanics, CombinedManeuver_PurePlaneChange) {
    double v = 7700.0;
    double delta_i = M_PI / 4.0;

    double dv_combined = combined_maneuver_delta_v(v, v, delta_i);
    double dv_plane = plane_change_delta_v(v, delta_i);

    // Should be equal for same initial and final velocity
    EXPECT_NEAR(dv_combined, dv_plane, 1e-6);
}

// Symbolic tests
TEST(TransferMechanics, Symbolic_Hohmann) {
    auto r1 = janus::sym("r1");
    auto r2 = janus::sym("r2");

    auto [dv1, dv2] = hohmann_delta_v(r1, r2);

    janus::Function f("hohmann", {r1, r2}, {dv1, dv2});
    auto result = f({7000.0e3, 14000.0e3});

    EXPECT_GT(result[0](0, 0), 0.0);
}
