# Vulcan Sensor Noise Models User Guide

This guide covers sensor noise modeling utilities for simulating realistic IMU (Inertial Measurement Unit) errors in flight simulation and navigation filter development.

## Quick Start

```cpp
#include <vulcan/sensors/AllanVarianceNoise.hpp>
#include <random>

// Create consumer-grade IMU noise model at 100 Hz
auto state = vulcan::allan::init_state<double>();
auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

// Step with white noise input
std::mt19937 rng(42);
std::normal_distribution<double> dist(0.0, 1.0);

vulcan::allan::IMUNoiseInput<double> input;
for (int j = 0; j < 3; ++j) {
    input.gyro_arw(j) = dist(rng);
    input.gyro_bias(j) = dist(rng);
    input.gyro_rrw(j) = dist(rng);
    input.accel_arw(j) = dist(rng);
    input.accel_bias(j) = dist(rng);
    input.accel_rrw(j) = dist(rng);
}

auto noise = vulcan::allan::step(state, coeffs, input);
// Use noise.gyro and noise.accel (Eigen::Vector3d)
```

## Noise Model Overview

Vulcan implements five fundamental sensor noise processes:

| Noise Type | Allan Slope | Parameter | Physical Source |
|------------|-------------|-----------|-----------------|
| White Noise (ARW/VRW) | -1/2 | N | Quantization, thermal noise |
| Bias Instability | 0 | B | Electronics drift |
| Rate Random Walk | +1/2 | K | Slow environmental effects |
| First-Order Markov | varies | σ, τ | Correlated disturbances |

## Gaussian White Noise

Simple scaling of unit-variance noise:

```cpp
#include <vulcan/sensors/GaussianNoise.hpp>

double noise_sample = dist(rng);  // N(0,1)
double sigma = 0.01;

// Scale to desired variance
double output = vulcan::gaussian::apply(noise_sample, sigma);

// Convert Allan variance N parameter to discrete σ
double N = 8.7e-5;  // ARW [rad/s/√Hz]
double dt = 0.01;   // 100 Hz
double discrete_sigma = vulcan::gaussian::sigma_from_arw(N, dt);
```

## Random Walk (Drift)

Integrated white noise modeling sensor drift:

```cpp
#include <vulcan/sensors/RandomWalk.hpp>

auto state = vulcan::random_walk::init_state<double>();
double K = 1e-6;  // Rate random walk [rad/s²/√Hz]
double dt = 0.01;
auto coeffs = vulcan::random_walk::compute_coeffs(K, dt);

// Step the process
double output = vulcan::random_walk::step(state, coeffs, dist(rng));

// Expected variance grows linearly with time
double t = 100.0;
double expected_var = vulcan::random_walk::expected_variance(K, t);
```

## Bias Instability

Long-correlation-time Markov process for slow bias wandering:

```cpp
#include <vulcan/sensors/BiasInstability.hpp>

auto state = vulcan::bias_instability::init_state<double>();
double sigma_b = 4.8e-6;  // Bias instability [rad/s]
double tau = 300.0;       // Correlation time [s]
double dt = 0.01;

auto coeffs = vulcan::bias_instability::compute_coeffs(sigma_b, tau, dt);

// Step the bias process
double bias = vulcan::bias_instability::step(state, coeffs, dist(rng));
```

## First-Order Markov Process

General exponentially correlated noise:

```cpp
#include <vulcan/sensors/MarkovProcess.hpp>

double tau = 10.0;    // Correlation time [s]
double sigma = 0.1;   // Steady-state std dev
double dt = 0.01;

auto state = vulcan::markov::init_state<double>();
auto coeffs = vulcan::markov::discretize(tau, sigma, dt);

// Step the process: x[k+1] = φ·x[k] + q·w[k]
double output = vulcan::markov::step(state, coeffs, dist(rng));

// Analysis utilities
double R_tau = vulcan::markov::autocorrelation(tau, sigma, tau);  // σ²·e⁻¹
double S_dc = vulcan::markov::psd_at_frequency(tau, sigma, 0.0);  // 2·σ²·τ
```

## Allan Variance Composite Model

Complete IMU noise model combining all processes:

```cpp
#include <vulcan/sensors/AllanVarianceNoise.hpp>

// Use preset IMU grades
auto coeffs = vulcan::allan::consumer_grade_coeffs(dt);
// Or: industrial_grade_coeffs, tactical_grade_coeffs, navigation_grade_coeffs

// Or specify custom parameters
vulcan::sensors::AllanParams<double> gyro_params{
    .N = 2.9e-5,        // ARW [rad/s/√Hz]
    .B = 4.8e-6,        // Bias instability [rad/s]
    .K = 1.0e-8,        // Rate random walk [rad/s²/√Hz]
    .tau_bias = 300.0,  // Correlation time [s]
};
vulcan::sensors::AllanParams<double> accel_params{
    .N = 1.0e-3,        // VRW [m/s²/√Hz]
    .B = 1.0e-4,        // Bias instability [m/s²]
    .K = 3.0e-6,        // Rate random walk [m/s³/√Hz]
    .tau_bias = 300.0,
};

auto custom_coeffs = vulcan::allan::compute_coeffs(gyro_params, accel_params, dt);
```

### IMU Grade Presets

| Grade | Gyro ARW | Gyro Bias | Typical Application |
|-------|----------|-----------|---------------------|
| Consumer | 0.3°/√hr | 10°/hr | Smartphones, drones |
| Industrial | 0.1°/√hr | 1°/hr | Robotics, vehicles |
| Tactical | 0.01°/√hr | 0.1°/hr | Military, aerospace |
| Navigation | 0.001°/√hr | 0.01°/hr | Aircraft, submarines |

## Symbolic Computation

All noise models work with `janus::SymbolicScalar`:

```cpp
using Scalar = janus::SymbolicScalar;

auto w1 = janus::sym("w_arw");
auto w2 = janus::sym("w_bias");
auto w3 = janus::sym("w_rrw");

auto state = vulcan::allan::init_axis_state<Scalar>();
auto coeffs = vulcan::allan::compute_axis_coeffs(
    vulcan::sensors::imu_grades::industrial_gyro(), dt);

auto output = vulcan::allan::step_axis(state, coeffs, w1, w2, w3);

// Create CasADi function for optimization
janus::Function f("imu_noise", {w1, w2, w3}, {output});
```

### Graph Visualization

```cpp
janus::export_graph_html(output, "graph_imu_noise", "IMU_Noise");
```

> [!TIP]
> **Interactive Examples** - Explore the computational graphs:
> - [🔍 IMU Noise](examples/graph_imu_noise.html)

## API Reference

### NoiseTypes.hpp

| Type | Description |
|------|-------------|
| `AllanParams<Scalar>` | N, B, K, τ_bias parameters |
| `IMUNoiseSample<Scalar>` | 3-axis gyro + accel noise vectors |
| `imu_grades::consumer_gyro()` | Preset Allan parameters |
| `imu_grades::industrial_gyro()` | ... |
| `imu_grades::tactical_gyro()` | ... |
| `imu_grades::navigation_gyro()` | ... |

### GaussianNoise.hpp

| Function | Description |
|----------|-------------|
| `apply(noise, σ)` | Scale noise by σ |
| `apply_psd(noise, psd, dt)` | Scale from PSD |
| `sigma_from_arw(N, dt)` | Convert Allan N to discrete σ |

### RandomWalk.hpp

| Function | Description |
|----------|-------------|
| `init_state<Scalar>()` | Initialize to zero |
| `compute_coeffs(K, dt)` | Discretize coefficients |
| `step(state, coeffs, noise)` | Integrate one step |
| `expected_variance(K, t)` | K²·t |

### BiasInstability.hpp

| Function | Description |
|----------|-------------|
| `init_state<Scalar>()` | Initialize to zero |
| `compute_coeffs(σ_b, τ, dt)` | Discretize Markov process |
| `step(state, coeffs, noise)` | Update bias |

### MarkovProcess.hpp

| Function | Description |
|----------|-------------|
| `init_state<Scalar>()` | Initialize to zero |
| `discretize(τ, σ, dt)` | Exact discretization |
| `step(state, coeffs, noise)` | Update state |
| `autocorrelation(τ, σ, lag)` | R(lag) = σ²·exp(-\|lag\|/τ) |
| `psd_at_frequency(τ, σ, f)` | Lorentzian PSD |

### AllanVarianceNoise.hpp

| Function | Description |
|----------|-------------|
| `init_state<Scalar>()` | Initialize 6-axis IMU state |
| `compute_coeffs(gyro, accel, dt)` | From Allan params |
| `step(state, coeffs, input)` | Full 6-axis step |
| `consumer_grade_coeffs(dt)` | Preset coefficients |

## Example: Sensor Noise Demo

See `examples/sensors/sensor_noise_demo.cpp`:

```bash
./scripts/build.sh
./build/examples/sensor_noise_demo
```

Output shows:
- IMU grade parameter comparison
- Gyroscope noise time series
- Bias instability accumulation
- Random walk drift growth
- Symbolic graph generation
