# RNG Utilities Library

Implementation plan for random number generation utilities to complement sensor noise models and Monte Carlo simulation.

## Background

Current sensor noise models require users to instantiate and manage their own RNG:

```cpp
std::mt19937 rng(42);
std::normal_distribution<double> dist(0.0, 1.0);
vulcan::random_walk::step(state, coeffs, dist(rng));  // User provides sample
```

This is intentional for Janus symbolic compatibility, but boilerplate-heavy for common use cases. A Vulcan RNG library can simplify this while maintaining flexibility.

---

## Proposed Changes

### RNG Module Structure

```
include/vulcan/rng/
├── RNG.hpp              # [NEW] Main RNG class with MT19937
├── Distributions.hpp    # [NEW] Gaussian, Uniform, IMU noise generators
└── Seeding.hpp          # [NEW] Seed utilities and stream management
tests/rng/
├── test_rng.cpp         # [NEW]
└── test_distributions.cpp # [NEW]
```

---

### RNG Module

#### [NEW] [RNG.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/rng/RNG.hpp)

Main RNG class wrapping `std::mt19937_64`:

```cpp
namespace vulcan::rng {

/**
 * @brief Vulcan Random Number Generator
 *
 * Wraps std::mt19937_64 with convenience methods for common distributions.
 * Seed is required at construction for reproducibility.
 */
class RNG {
public:
    using Engine = std::mt19937_64;
    using result_type = Engine::result_type;

    /// Construct with explicit seed (required for reproducibility)
    explicit RNG(uint64_t seed);

    /// Construct with seed sequence (for better initialization)
    explicit RNG(std::seed_seq& seq);

    /// Get raw random bits
    result_type operator()();

    /// Generate standard normal N(0,1)
    double gaussian();

    /// Generate normal N(mean, stddev)
    double gaussian(double mean, double stddev);

    /// Generate uniform [0, 1)
    double uniform();

    /// Generate uniform [min, max)
    double uniform(double min, double max);

    /// Generate uniform integer [min, max]
    int64_t uniform_int(int64_t min, int64_t max);

    /// Fill 3-vector with independent N(0,1)
    Eigen::Vector3d gaussian3();

    /// Access underlying engine (for advanced use)
    Engine& engine();
    const Engine& engine() const;

    /// Get current seed (for logging/reproducibility)
    uint64_t seed() const;

    /// Discard n values (for stream splitting)
    void discard(uint64_t n);

private:
    Engine engine_;
    uint64_t seed_;
    std::normal_distribution<double> normal_;
    std::uniform_real_distribution<double> uniform_;
};

} // namespace vulcan::rng
```

---

#### [NEW] [Distributions.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/rng/Distributions.hpp)

Higher-level noise generators:

```cpp
namespace vulcan::rng {

/**
 * @brief Generate IMU noise input for Allan variance model
 *
 * Fills all 18 noise channels with independent N(0,1).
 */
template <typename RNGType>
allan::IMUNoiseInput<double> generate_imu_noise(RNGType& rng);

/**
 * @brief Generate 3-axis white noise
 */
template <typename RNGType>
Eigen::Vector3d generate_noise3(RNGType& rng);

/**
 * @brief Generate correlated noise with given covariance
 *
 * @param rng Random number generator
 * @param L Lower-triangular Cholesky factor of covariance
 * @return Correlated noise vector
 */
template <typename RNGType, int N>
Eigen::Vector<double, N> generate_correlated_noise(
    RNGType& rng,
    const Eigen::Matrix<double, N, N>& L);

} // namespace vulcan::rng
```

---

#### [NEW] [Seeding.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/rng/Seeding.hpp)

Seed management utilities:

```cpp
namespace vulcan::rng {

/**
 * @brief Generate seed from std::random_device
 *
 * Use only for initial seeding; log the seed for reproducibility.
 */
uint64_t hardware_seed();

/**
 * @brief Create seed sequence from multiple values
 *
 * For robust MT19937 initialization.
 */
std::seed_seq create_seed_seq(std::initializer_list<uint32_t> values);

/**
 * @brief Hash-based seed from string
 *
 * Useful for named simulation runs.
 */
uint64_t seed_from_string(std::string_view name);

/**
 * @brief Combine base seed with stream index
 *
 * For parallel simulations with independent streams.
 * Uses golden ratio hashing.
 */
uint64_t stream_seed(uint64_t base_seed, uint64_t stream_index);

} // namespace vulcan::rng
```

---

### CMake Updates

#### [MODIFY] [CMakeLists.txt](file:///home/tanged/sources/temp/vulcan/tests/CMakeLists.txt)

```cmake
# RNG tests
add_executable(test_rng
    rng/test_rng.cpp
    rng/test_distributions.cpp
)
target_link_libraries(test_rng PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_rng)
```

---

### Main Header Updates

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/vulcan.hpp)

```cpp
// Random number generation
#include <vulcan/rng/RNG.hpp>
#include <vulcan/rng/Distributions.hpp>
#include <vulcan/rng/Seeding.hpp>
```

---

## Design Decisions

### Why `std::mt19937_64`?

1. **Portability**: Guaranteed same sequence across all C++11 implementations
2. **Reproducibility**: Essential for Monte Carlo validation
3. **Quality**: Passes all standard statistical tests
4. **No external dependencies**: Header-only

### Why Not PCG/Xoshiro?

These are faster with better statistical properties, but:
- Require external headers
- Less portable (implementation-dependent)
- MT19937 is sufficient for aerospace simulation rates

> [!NOTE]
> A future `FastRNG` class could wrap PCG for performance-critical applications.

### Seed Management

- **No default seeds**: Users must explicitly provide seeds
- **No `std::random_device` auto-seeding**: Not reproducible across runs
- **Seed logging**: Class stores seed for debugging

---

## Example Usage

```cpp
#include <vulcan/vulcan.hpp>

// Create seeded RNG
vulcan::rng::RNG rng(42);

// Generate IMU noise for sensor simulation
auto imu_state = vulcan::allan::init_state<double>();
auto imu_coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

for (int i = 0; i < 1000; ++i) {
    auto noise_input = vulcan::rng::generate_imu_noise(rng);
    auto noise = vulcan::allan::step(imu_state, imu_coeffs, noise_input);
    // Use noise.gyro, noise.accel
}

// Parallel streams for Monte Carlo
for (int run = 0; run < 100; ++run) {
    vulcan::rng::RNG stream_rng(vulcan::rng::stream_seed(42, run));
    // Independent simulation...
}
```

---

## Verification Plan

### Automated Tests

Run via:
```bash
./scripts/ci.sh
```

Or individually:
```bash
cd build && ctest -R test_rng -VV
```

#### Test Coverage

| Test File | Coverage |
|-----------|----------|
| `test_rng.cpp` | Construction, distributions, seeding |
| `test_distributions.cpp` | IMU noise, correlated noise, variance checks |

#### Specific Test Cases

1. **Reproducibility**: Same seed produces same sequence
2. **Statistical properties**: Gaussian mean/variance within tolerance
3. **Independence**: 3-vector components uncorrelated
4. **Stream splitting**: Different streams produce different sequences

---

## Out of Scope

- Hardware RNG (RDRAND)
- Cryptographically secure RNG
- Quasi-random sequences (Halton, Sobol)
- External libraries (PCG, Xoshiro)
