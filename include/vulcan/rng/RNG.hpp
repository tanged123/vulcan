/**
 * @file RNG.hpp
 * @brief Vulcan Random Number Generator
 *
 * Wraps std::mt19937_64 with convenience methods for common distributions
 * used in aerospace simulation and Monte Carlo analysis.
 */

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <random>

namespace vulcan::rng {

/**
 * @brief Vulcan Random Number Generator
 *
 * Provides a convenient wrapper around std::mt19937_64 with methods for
 * common distributions used in sensor simulation and Monte Carlo analysis.
 *
 * Design principles:
 * - Explicit seeding required (no auto-seeding for reproducibility)
 * - Same seed produces identical sequence across all platforms
 * - Stores seed for logging/debugging
 *
 * Example:
 * @code
 * vulcan::rng::RNG rng(42);  // Seeded for reproducibility
 *
 * double x = rng.gaussian();       // N(0,1)
 * double y = rng.uniform(0, 10);   // U[0, 10)
 * auto v = rng.gaussian3();        // 3-vector of N(0,1)
 * @endcode
 */
class RNG {
  public:
    using Engine = std::mt19937_64;
    using result_type = Engine::result_type;

    /**
     * @brief Construct with explicit seed
     * @param seed Seed value (logged for reproducibility)
     */
    explicit RNG(uint64_t seed)
        : engine_(seed), seed_(seed), normal_(0.0, 1.0), uniform_(0.0, 1.0) {}

    /**
     * @brief Construct with seed sequence
     *
     * Provides better initialization of MT19937's large state.
     *
     * @param seq Seed sequence
     */
    explicit RNG(std::seed_seq &seq)
        : engine_(seq), seed_(0), normal_(0.0, 1.0), uniform_(0.0, 1.0) {}

    // Prevent copying (each RNG should have unique state)
    RNG(const RNG &) = delete;
    RNG &operator=(const RNG &) = delete;

    // Allow moving
    RNG(RNG &&) = default;
    RNG &operator=(RNG &&) = default;

    /**
     * @brief Get raw random bits
     * @return Random 64-bit value
     */
    result_type operator()() { return engine_(); }

    /**
     * @brief Generate standard normal N(0,1)
     * @return Random sample from N(0,1)
     */
    double gaussian() { return normal_(engine_); }

    /**
     * @brief Generate normal N(mean, stddev)
     * @param mean Distribution mean
     * @param stddev Distribution standard deviation
     * @return Random sample from N(mean, stddev²)
     */
    double gaussian(double mean, double stddev) {
        return mean + stddev * normal_(engine_);
    }

    /**
     * @brief Generate uniform [0, 1)
     * @return Random sample from U[0, 1)
     */
    double uniform() { return uniform_(engine_); }

    /**
     * @brief Generate uniform [min, max)
     * @param min Minimum value (inclusive)
     * @param max Maximum value (exclusive)
     * @return Random sample from U[min, max)
     */
    double uniform(double min, double max) {
        return min + (max - min) * uniform_(engine_);
    }

    /**
     * @brief Generate uniform integer [min, max]
     * @param min Minimum value (inclusive)
     * @param max Maximum value (inclusive)
     * @return Random integer from [min, max]
     */
    int64_t uniform_int(int64_t min, int64_t max) {
        std::uniform_int_distribution<int64_t> dist(min, max);
        return dist(engine_);
    }

    /**
     * @brief Fill 3-vector with independent N(0,1)
     * @return 3D vector of independent standard normal samples
     */
    Eigen::Vector3d gaussian3() {
        return Eigen::Vector3d(normal_(engine_), normal_(engine_),
                               normal_(engine_));
    }

    /**
     * @brief Fill N-vector with independent N(0,1)
     * @tparam N Vector dimension
     * @return N-dimensional vector of independent standard normal samples
     */
    template <int N> Eigen::Vector<double, N> gaussianN() {
        Eigen::Vector<double, N> v;
        for (int i = 0; i < N; ++i) {
            v(i) = normal_(engine_);
        }
        return v;
    }

    /**
     * @brief Access underlying engine (for advanced use)
     * @return Reference to MT19937_64 engine
     */
    Engine &engine() { return engine_; }
    const Engine &engine() const { return engine_; }

    /**
     * @brief Get original seed (for logging/reproducibility)
     * @return Seed used to initialize this RNG
     * @note Returns 0 if constructed with seed_seq
     */
    uint64_t seed() const { return seed_; }

    /**
     * @brief Discard n values (for stream splitting)
     * @param n Number of values to skip
     */
    void discard(uint64_t n) { engine_.discard(n); }

    /**
     * @brief Reset to initial state with same seed
     *
     * Resets both the engine and distribution caches for full reproducibility.
     */
    void reset() {
        engine_.seed(seed_);
        normal_.reset();  // Clear Box-Muller cache
        uniform_.reset(); // Clear any distribution cache
    }

    /**
     * @brief Reseed with new value
     * @param new_seed New seed value
     */
    void reseed(uint64_t new_seed) {
        seed_ = new_seed;
        engine_.seed(new_seed);
        normal_.reset();
        uniform_.reset();
    }

  private:
    Engine engine_;
    uint64_t seed_;
    std::normal_distribution<double> normal_;
    std::uniform_real_distribution<double> uniform_;
};

} // namespace vulcan::rng
