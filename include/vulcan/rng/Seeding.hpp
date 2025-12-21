/**
 * @file Seeding.hpp
 * @brief Seed utilities for reproducible random number generation
 *
 * Provides utilities for seed management, stream splitting for parallel
 * simulations, and deterministic seed generation from strings.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <random>
#include <string_view>

namespace vulcan::rng {

/**
 * @brief Generate seed from std::random_device
 *
 * Use only for initial seeding. The returned seed should be logged
 * for reproducibility in future runs.
 *
 * @warning Not guaranteed to be truly random on all platforms.
 *          Some implementations may fall back to PRNG.
 *
 * @return Hardware-generated seed value
 */
inline uint64_t hardware_seed() {
    std::random_device rd;
    // Combine two 32-bit values for 64-bit seed
    uint64_t high = static_cast<uint64_t>(rd()) << 32;
    uint64_t low = static_cast<uint64_t>(rd());
    return high | low;
}

/**
 * @brief Create seed sequence from multiple values
 *
 * Provides robust initialization for MT19937 which benefits from
 * multiple seed values to initialize its large state.
 *
 * @param values Initializer list of seed values
 * @return seed_seq for RNG initialization
 */
inline std::seed_seq create_seed_seq(std::initializer_list<uint32_t> values) {
    return std::seed_seq(values);
}

/**
 * @brief Hash-based seed from string
 *
 * Useful for named simulation runs, e.g.,
 * seed_from_string("monte_carlo_run_42") Uses std::hash which provides
 * reasonable distribution.
 *
 * @param name String identifier for the simulation
 * @return Deterministic seed value
 */
inline uint64_t seed_from_string(std::string_view name) {
    return std::hash<std::string_view>{}(name);
}

/**
 * @brief Combine base seed with stream index for parallel streams
 *
 * Uses golden ratio hashing for good distribution of stream seeds.
 * Each stream will produce an independent sequence.
 *
 * Example:
 * @code
 * uint64_t base = 42;
 * for (int i = 0; i < 100; ++i) {
 *     RNG stream_rng(stream_seed(base, i));
 *     // Independent simulation...
 * }
 * @endcode
 *
 * @param base_seed Base seed for all streams
 * @param stream_index Index of this particular stream
 * @return Unique seed for this stream
 */
inline uint64_t stream_seed(uint64_t base_seed, uint64_t stream_index) {
    // Golden ratio constant for 64-bit
    constexpr uint64_t PHI = 0x9E3779B97F4A7C15ULL;
    // Combine using multiplication and XOR for good mixing
    return base_seed ^ (stream_index * PHI);
}

/**
 * @brief Advance seed to skip N sequences
 *
 * Useful for checkpointing: compute the seed that would result
 * from drawing N samples.
 *
 * @param seed Current seed
 * @param n Number of positions to advance
 * @return New seed value
 */
inline uint64_t advance_seed(uint64_t seed, uint64_t n) {
    // Simple linear congruential step for seed advancement
    constexpr uint64_t A = 6364136223846793005ULL;
    constexpr uint64_t C = 1442695040888963407ULL;
    for (uint64_t i = 0; i < n; ++i) {
        seed = A * seed + C;
    }
    return seed;
}

} // namespace vulcan::rng
