# Vulcan RNG Utilities User Guide

This guide covers the random number generation utilities in Vulcan, providing reproducible and convenient RNG for sensor simulation and Monte Carlo analysis.

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>

// Create seeded RNG (required for reproducibility)
vulcan::rng::RNG rng(42);

// Generate standard normal N(0,1)
double x = rng.gaussian();

// Generate uniform [0, 10)
double y = rng.uniform(0.0, 10.0);

// Generate 3D noise vector
Eigen::Vector3d noise = rng.gaussian3();
```

## The RNG Class

The `vulcan::rng::RNG` class wraps `std::mt19937_64` with convenience methods for aerospace simulation.

### Design Principles

| Principle | Rationale |
|-----------|-----------|
| **Explicit seeding** | No auto-seeding ensures reproducibility |
| **Cross-platform** | Same seed produces identical sequence everywhere |
| **Seed storage** | Original seed is saved for logging/debugging |

### Construction

```cpp
// Construct with integer seed
vulcan::rng::RNG rng(42);

// Construct with seed sequence (for robust MT19937 initialization)
std::seed_seq seq{1, 2, 3, 4};
vulcan::rng::RNG rng2(seq);
```

> [!IMPORTANT]
> RNGs are **non-copyable** to prevent accidental state sharing. Use `std::move()` if needed.

### Generating Random Numbers

#### Gaussian (Normal) Distribution

```cpp
// Standard normal N(0, 1)
double x = rng.gaussian();

// Arbitrary mean and stddev: N(μ, σ)
double y = rng.gaussian(5.0, 2.0);  // N(5, 4)

// 3D vector of independent N(0, 1)
Eigen::Vector3d v = rng.gaussian3();

// N-dimensional vector
Eigen::Vector<double, 6> w = rng.gaussianN<6>();
```

#### Uniform Distribution

```cpp
// Uniform [0, 1)
double u = rng.uniform();

// Uniform [min, max)
double u2 = rng.uniform(-5.0, 5.0);

// Uniform integer [min, max] (inclusive)
int64_t i = rng.uniform_int(1, 100);
```

### State Management

```cpp
// Get original seed (for logging)
std::cout << "Using seed: " << rng.seed() << std::endl;

// Reset to initial state (replay sequence)
rng.reset();

// Reseed with new value
rng.reseed(12345);

// Skip n values (for stream splitting)
rng.discard(1000);

// Access underlying engine (advanced use)
std::mt19937_64& engine = rng.engine();
```

## Noise Generators

Higher-level functions for sensor simulation are in `vulcan::rng::`:

### IMU Noise Generation

```cpp
vulcan::rng::RNG rng(42);

// Initialize Allan variance model
auto state = vulcan::allan::init_state<double>();
auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

// Generate all 18 IMU noise channels with one call
auto input = vulcan::rng::generate_imu_noise(rng);
auto noise = vulcan::allan::step(state, coeffs, input);

// Use noise.gyro, noise.accel
```

### White Noise Vectors

```cpp
// 3D white noise
Eigen::Vector3d w3 = vulcan::rng::generate_noise3(rng);

// N-dimensional white noise
Eigen::Vector<double, 6> w6 = vulcan::rng::generate_noiseN<6>(rng);
```

### Correlated Noise

```cpp
// Given covariance matrix P, compute Cholesky factor L
Eigen::Matrix3d P = ...;  // 3x3 covariance
Eigen::Matrix3d L = P.llt().matrixL();

// Generate correlated noise ~ N(0, P)
Eigen::Vector3d corr_noise = vulcan::rng::generate_correlated_noise<3>(rng, L);
```

### Turbulence Forcing

```cpp
double noise_u, noise_v, noise_w;
vulcan::rng::generate_turbulence_noise(rng, noise_u, noise_v, noise_w);
```

## Seeding Utilities

Functions in `vulcan::rng::` for managing seeds:

### Hardware Seed

```cpp
// Get seed from hardware RNG (non-reproducible)
uint64_t seed = vulcan::rng::hardware_seed();
std::cout << "Random seed: " << seed << std::endl;  // Log it!

vulcan::rng::RNG rng(seed);
```

> [!WARNING]
> Only use `hardware_seed()` for initial seeding. Always log the returned value for reproducibility.

### Named Seeds

```cpp
// Deterministic seed from string (useful for named experiments)
uint64_t seed = vulcan::rng::seed_from_string("monte_carlo_v2");
```

### Parallel Streams

For Monte Carlo simulations with independent parallel streams:

```cpp
uint64_t base_seed = 42;
int num_runs = 100;

for (int run = 0; run < num_runs; ++run) {
    // Each stream gets a unique, reproducible seed
    uint64_t stream = vulcan::rng::stream_seed(base_seed, run);
    vulcan::rng::RNG rng(stream);
    
    // Independent simulation...
}
```

### Seed Advancement

```cpp
// Advance seed (useful for checkpointing)
uint64_t advanced = vulcan::rng::advance_seed(base_seed, 1000);
```

## Integration with Sensor Models

### Bias Instability

```cpp
vulcan::rng::RNG rng(42);

auto bias_state = vulcan::bias_instability::init_state<double>();
auto bias_coeffs = vulcan::bias_instability::compute_coeffs(1e-5, 100.0, 0.01);

for (int i = 0; i < 10000; ++i) {
    double bias = vulcan::bias_instability::step(
        bias_state, bias_coeffs, rng.gaussian()
    );
}
```

### Random Walk

```cpp
vulcan::rng::RNG rng(42);

auto rw_state = vulcan::random_walk::init_state<double>();
auto rw_coeffs = vulcan::random_walk::compute_coeffs(1e-6, 0.01);

for (int i = 0; i < 1000; ++i) {
    vulcan::random_walk::step(rw_state, rw_coeffs, rng.gaussian());
}
```

## API Reference

### RNG.hpp

| Method | Description |
|--------|-------------|
| `RNG(uint64_t seed)` | Construct with integer seed |
| `RNG(std::seed_seq& seq)` | Construct with seed sequence |
| `gaussian()` | Sample from N(0, 1) |
| `gaussian(mean, stddev)` | Sample from N(mean, stddev²) |
| `uniform()` | Sample from U[0, 1) |
| `uniform(min, max)` | Sample from U[min, max) |
| `uniform_int(min, max)` | Sample integer from [min, max] |
| `gaussian3()` | 3D vector of N(0, 1) samples |
| `gaussianN<N>()` | N-dimensional vector of N(0, 1) |
| `seed()` | Get original seed |
| `reset()` | Reset to initial state |
| `reseed(seed)` | Reseed with new value |
| `discard(n)` | Skip n values |
| `engine()` | Access underlying MT19937_64 |

### Distributions.hpp

| Function | Description |
|----------|-------------|
| `generate_imu_noise(rng)` | 18-channel IMU noise input |
| `generate_noise3(rng)` | 3D white noise vector |
| `generate_noiseN<N>(rng)` | N-dimensional white noise |
| `generate_correlated_noise<N>(rng, L)` | Correlated noise with Cholesky factor |
| `generate_turbulence_noise(rng, u, v, w)` | 3-channel turbulence forcing |

### Seeding.hpp

| Function | Description |
|----------|-------------|
| `hardware_seed()` | Get seed from std::random_device |
| `create_seed_seq({...})` | Create seed sequence from values |
| `seed_from_string(name)` | Deterministic seed from string |
| `stream_seed(base, index)` | Unique seed for parallel stream |
| `advance_seed(seed, n)` | Advance seed by n positions |

## Example: Complete Demo

See `examples/sensors/sensor_noise_demo.cpp` for comprehensive usage demonstrating:
- IMU noise simulation with Allan variance
- Bias instability accumulation
- Random walk drift
- Symbolic graph generation

```bash
# Build and run the demo
./scripts/build.sh
./build/examples/sensor_noise_demo
```
