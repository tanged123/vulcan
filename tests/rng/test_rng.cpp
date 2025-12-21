// RNG Core Tests
#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/rng/RNG.hpp>
#include <vulcan/rng/Seeding.hpp>

using namespace vulcan::rng;

// =============================================================================
// Construction and Seeding
// =============================================================================

TEST(RNG, ConstructWithSeed) {
    RNG rng(42);
    EXPECT_EQ(rng.seed(), 42);
}

TEST(RNG, Reproducibility) {
    RNG rng1(12345);
    RNG rng2(12345);

    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(rng1(), rng2());
    }
}

TEST(RNG, DifferentSeedsDifferentSequence) {
    RNG rng1(42);
    RNG rng2(43);

    // Very unlikely to be equal
    EXPECT_NE(rng1(), rng2());
}

TEST(RNG, Reset) {
    RNG rng(42);

    double first = rng.gaussian();
    rng.gaussian();
    rng.gaussian();

    rng.reset();
    EXPECT_DOUBLE_EQ(rng.gaussian(), first);
}

TEST(RNG, Reseed) {
    RNG rng(42);
    rng.gaussian();

    rng.reseed(100);
    EXPECT_EQ(rng.seed(), 100);

    RNG rng2(100);
    EXPECT_EQ(rng.gaussian(), rng2.gaussian());
}

// =============================================================================
// Gaussian Distribution
// =============================================================================

TEST(RNG, GaussianMean) {
    RNG rng(42);
    int N = 100000;
    double sum = 0.0;

    for (int i = 0; i < N; ++i) {
        sum += rng.gaussian();
    }

    double mean = sum / N;
    EXPECT_NEAR(mean, 0.0, 0.01);
}

TEST(RNG, GaussianVariance) {
    RNG rng(42);
    int N = 100000;
    double sum = 0.0;
    double sum_sq = 0.0;

    for (int i = 0; i < N; ++i) {
        double x = rng.gaussian();
        sum += x;
        sum_sq += x * x;
    }

    double mean = sum / N;
    double variance = sum_sq / N - mean * mean;
    EXPECT_NEAR(variance, 1.0, 0.02);
}

TEST(RNG, GaussianMeanStddev) {
    RNG rng(42);
    int N = 100000;
    double sum = 0.0;

    for (int i = 0; i < N; ++i) {
        sum += rng.gaussian(5.0, 2.0);
    }

    double mean = sum / N;
    EXPECT_NEAR(mean, 5.0, 0.02);
}

// =============================================================================
// Uniform Distribution
// =============================================================================

TEST(RNG, UniformRange) {
    RNG rng(42);

    for (int i = 0; i < 1000; ++i) {
        double x = rng.uniform();
        EXPECT_GE(x, 0.0);
        EXPECT_LT(x, 1.0);
    }
}

TEST(RNG, UniformCustomRange) {
    RNG rng(42);

    for (int i = 0; i < 1000; ++i) {
        double x = rng.uniform(-5.0, 10.0);
        EXPECT_GE(x, -5.0);
        EXPECT_LT(x, 10.0);
    }
}

TEST(RNG, UniformMean) {
    RNG rng(42);
    int N = 100000;
    double sum = 0.0;

    for (int i = 0; i < N; ++i) {
        sum += rng.uniform();
    }

    double mean = sum / N;
    EXPECT_NEAR(mean, 0.5, 0.01);
}

TEST(RNG, UniformIntRange) {
    RNG rng(42);

    for (int i = 0; i < 1000; ++i) {
        int64_t x = rng.uniform_int(5, 10);
        EXPECT_GE(x, 5);
        EXPECT_LE(x, 10);
    }
}

// =============================================================================
// Vector Generation
// =============================================================================

TEST(RNG, Gaussian3Independent) {
    RNG rng(42);
    int N = 10000;

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();

    for (int i = 0; i < N; ++i) {
        auto v = rng.gaussian3();
        sum += v;
        sum_outer += v * v.transpose();
    }

    Eigen::Vector3d mean = sum / N;
    Eigen::Matrix3d cov = sum_outer / N - mean * mean.transpose();

    // Mean should be near zero
    EXPECT_NEAR(mean.norm(), 0.0, 0.05);

    // Covariance should be approximately identity
    EXPECT_NEAR(cov(0, 0), 1.0, 0.1);
    EXPECT_NEAR(cov(1, 1), 1.0, 0.1);
    EXPECT_NEAR(cov(2, 2), 1.0, 0.1);

    // Off-diagonal should be near zero
    EXPECT_NEAR(cov(0, 1), 0.0, 0.05);
    EXPECT_NEAR(cov(0, 2), 0.0, 0.05);
    EXPECT_NEAR(cov(1, 2), 0.0, 0.05);
}

TEST(RNG, GaussianN) {
    RNG rng(42);
    auto v = rng.gaussianN<6>();
    EXPECT_EQ(v.size(), 6);
}

// =============================================================================
// Seeding Utilities
// =============================================================================

TEST(Seeding, HardwareSeed) {
    uint64_t s1 = hardware_seed();
    uint64_t s2 = hardware_seed();

    // Should produce different values (overwhelmingly likely)
    EXPECT_NE(s1, s2);
}

TEST(Seeding, SeedFromString) {
    uint64_t s1 = seed_from_string("monte_carlo_run_1");
    uint64_t s2 = seed_from_string("monte_carlo_run_2");
    uint64_t s3 = seed_from_string("monte_carlo_run_1");

    EXPECT_NE(s1, s2);
    EXPECT_EQ(s1, s3);
}

TEST(Seeding, StreamSeed) {
    uint64_t base = 42;
    uint64_t s1 = stream_seed(base, 0);
    uint64_t s2 = stream_seed(base, 1);
    uint64_t s3 = stream_seed(base, 100);

    // All different
    EXPECT_NE(s1, s2);
    EXPECT_NE(s1, s3);
    EXPECT_NE(s2, s3);

    // Reproducible
    EXPECT_EQ(stream_seed(base, 0), s1);
}

TEST(Seeding, StreamSeedIndependence) {
    // Verify streams produce uncorrelated sequences
    RNG rng1(stream_seed(42, 0));
    RNG rng2(stream_seed(42, 1));

    int N = 10000;
    double sum_xy = 0.0;
    double sum_x = 0.0, sum_y = 0.0;

    for (int i = 0; i < N; ++i) {
        double x = rng1.gaussian();
        double y = rng2.gaussian();
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
    }

    double mean_x = sum_x / N;
    double mean_y = sum_y / N;
    double cov = sum_xy / N - mean_x * mean_y;

    // Covariance should be near zero
    EXPECT_NEAR(cov, 0.0, 0.05);
}

TEST(Seeding, CreateSeedSeq) {
    auto seq = create_seed_seq({1, 2, 3, 4, 5});
    RNG rng(seq);

    // Should work and produce values
    double x = rng.gaussian();
    EXPECT_TRUE(std::isfinite(x));
}

TEST(Seeding, AdvanceSeed) {
    uint64_t s1 = 42;
    uint64_t s2 = advance_seed(s1, 0);
    uint64_t s3 = advance_seed(s1, 1);
    uint64_t s4 = advance_seed(s1, 100);

    EXPECT_EQ(s1, s2); // n=0 should be identity
    EXPECT_NE(s1, s3);
    EXPECT_NE(s3, s4);
}
