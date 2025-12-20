# Phase 8: Sensor Noise Models

Implementation plan for stochastic sensor noise models suitable for IMU simulation, sensor characterization, and Kalman filter development.

## Background

Accurate sensor simulation requires modeling multiple noise processes that occur in real inertial sensors:
- **White noise** (angle/velocity random walk)
- **Random walk** (rate random walk / drift)
- **Bias instability** (1/f noise)
- **First-order Markov processes** (exponentially correlated noise)
- **Composite noise** (Allan variance parameterized)

These models must be **templated** for Janus dual-mode compatibility (`double` and `casadi::MX`).

---

## Proposed Changes

### Sensors Module Structure

```
include/vulcan/sensors/
├── NoiseTypes.hpp          # [NEW] Common types and parameters
├── GaussianNoise.hpp       # [NEW] White noise generator
├── RandomWalk.hpp          # [NEW] Integrated white noise
├── BiasInstability.hpp     # [NEW] 1/f noise model
├── MarkovProcess.hpp       # [NEW] First-order Gauss-Markov
└── AllanVarianceNoise.hpp  # [NEW] Composite IMU noise model
tests/sensors/
├── test_gaussian_noise.cpp # [NEW]
├── test_random_walk.cpp    # [NEW]
├── test_bias_instability.cpp # [NEW]
├── test_markov_process.cpp # [NEW]
└── test_allan_variance.cpp # [NEW]
```

---

### Sensors

#### [NEW] [NoiseTypes.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/NoiseTypes.hpp)

Common types and parameter structures:

```cpp
namespace vulcan::sensors {

/// Allan variance noise parameters for IMU sensors
template <typename Scalar>
struct AllanParams {
    Scalar N;          ///< Angle/velocity random walk [rad/s/√Hz] or [m/s²/√Hz]
    Scalar B;          ///< Bias instability [rad/s] or [m/s²]
    Scalar K;          ///< Rate random walk [rad/s²/√Hz] or [m/s³/√Hz]
    Scalar tau_bias;   ///< Bias instability correlation time [s]
};

/// First-order Markov process parameters
template <typename Scalar>
struct MarkovParams {
    Scalar sigma;      ///< Steady-state standard deviation
    Scalar tau;        ///< Correlation time [s]
};

/// Noise output state
template <typename Scalar>
struct NoiseState {
    Scalar value;      ///< Current noise value
    Scalar state;      ///< Internal filter state (for stateful processes)
};

} // namespace vulcan::sensors
```

---

#### [NEW] [GaussianNoise.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/GaussianNoise.hpp)

Gaussian white noise generator:

```cpp
namespace vulcan::gaussian {

/// Apply Gaussian white noise scaling
/// @param noise_input White noise sample (unit variance)
/// @param sigma Standard deviation [units]
/// @param dt Time step [s] (for power spectral density scaling)
/// @return Scaled noise value
template <typename Scalar>
Scalar apply(const Scalar& noise_input, double sigma, double dt);

/// Compute noise variance from PSD and sample rate
/// @param psd_density Power spectral density [units²/Hz]
/// @param dt Time step [s]
/// @return Variance of discrete samples
template <typename Scalar>
Scalar variance_from_psd(double psd_density, double dt);

} // namespace vulcan::gaussian
```

---

#### [NEW] [RandomWalk.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/RandomWalk.hpp)

Integrated white noise (random walk / drift):

```cpp
namespace vulcan::random_walk {

/// Random walk state
template <typename Scalar>
struct State {
    Scalar value;  ///< Accumulated random walk value
};

/// Initialize random walk state
template <typename Scalar>
State<Scalar> init_state();

/// Step the random walk
/// @param state Current state (updated in-place)
/// @param noise_input White noise sample (unit variance)
/// @param K Rate random walk coefficient [units/s/√Hz]
/// @param dt Time step [s]
/// @return Updated random walk value
template <typename Scalar>
Scalar step(State<Scalar>& state, const Scalar& noise_input, double K, double dt);

} // namespace vulcan::random_walk
```

---

#### [NEW] [BiasInstability.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/BiasInstability.hpp)

Bias instability modeled as a first-order Gauss-Markov process with long correlation time:

```cpp
namespace vulcan::bias_instability {

/// Bias instability state
template <typename Scalar>
struct State {
    Scalar bias;  ///< Current bias value
};

/// Discretized filter coefficients
struct Coeffs {
    double a;  ///< State transition coefficient exp(-dt/tau)
    double b;  ///< Input gain sqrt(1 - a²) * sigma
};

/// Compute filter coefficients
/// @param sigma_b Bias instability magnitude [units]
/// @param tau Correlation time [s] (typically very long, e.g., 100-3600s)
/// @param dt Time step [s]
Coeffs compute_coeffs(double sigma_b, double tau, double dt);

/// Initialize state
template <typename Scalar>
State<Scalar> init_state();

/// Step the bias instability process
template <typename Scalar>
Scalar step(State<Scalar>& state, const Coeffs& coeffs, const Scalar& noise_input);

} // namespace vulcan::bias_instability
```

---

#### [NEW] [MarkovProcess.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/MarkovProcess.hpp)

General first-order Gauss-Markov process:

```cpp
namespace vulcan::markov {

/// First-order Gauss-Markov state
template <typename Scalar>
struct State {
    Scalar x;  ///< Process state
};

/// Discretized process coefficients
struct Coeffs {
    double phi;   ///< State transition: exp(-dt/tau)
    double q;     ///< Process noise gain: sigma * sqrt(1 - phi²)
};

/// Compute continuous-time parameters from Allan variance
/// @param tau Correlation time [s]
/// @param sigma Steady-state standard deviation
/// @return Process noise spectral density
double process_noise_psd(double tau, double sigma);

/// Discretize first-order Markov process
/// @param tau Correlation time [s]
/// @param sigma Steady-state standard deviation
/// @param dt Time step [s]
Coeffs discretize(double tau, double sigma, double dt);

/// Initialize state
template <typename Scalar>
State<Scalar> init_state();

/// Step the Gauss-Markov process
/// @param state Current state (updated in-place)
/// @param coeffs Discretized coefficients
/// @param noise_input White noise sample (unit variance)
/// @return Current process output
template <typename Scalar>
Scalar step(State<Scalar>& state, const Coeffs& coeffs, const Scalar& noise_input);

} // namespace vulcan::markov
```

---

#### [NEW] [AllanVarianceNoise.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/sensors/AllanVarianceNoise.hpp)

Composite IMU noise model parameterized from Allan variance analysis:

```cpp
namespace vulcan::allan {

/// Combined IMU noise state (per-axis)
template <typename Scalar>
struct AxisState {
    Scalar arw_value;       ///< Angle/velocity random walk contribution
    markov::State<Scalar> bias_state;  ///< Bias instability state
    random_walk::State<Scalar> rrw_state;  ///< Rate random walk state
};

/// Pre-computed coefficients for all noise processes
struct AxisCoeffs {
    double arw_gain;           ///< White noise gain from N parameter
    markov::Coeffs bias_coeffs; ///< Bias instability coefficients
    double rrw_K;              ///< Rate random walk coefficient
};

/// 3-axis IMU noise state
template <typename Scalar>
struct IMUNoiseState {
    AxisState<Scalar> gyro[3];  ///< Gyroscope noise states (x, y, z)
    AxisState<Scalar> accel[3]; ///< Accelerometer noise states (x, y, z)
};

/// 3-axis IMU noise coefficients
struct IMUNoiseCoeffs {
    AxisCoeffs gyro;   ///< Gyroscope coefficients (assumed same for all axes)
    AxisCoeffs accel;  ///< Accelerometer coefficients
};

/// Compute coefficients from Allan variance parameters
/// @param gyro_params Allan variance parameters for gyroscope
/// @param accel_params Allan variance parameters for accelerometer
/// @param dt Time step [s]
IMUNoiseCoeffs compute_coeffs(
    const AllanParams<double>& gyro_params,
    const AllanParams<double>& accel_params,
    double dt);

/// Initialize IMU noise state
template <typename Scalar>
IMUNoiseState<Scalar> init_state();

/// Step the IMU noise model
/// @param state Current noise state (updated in-place)
/// @param coeffs Pre-computed coefficients
/// @param gyro_noise 3-element white noise for gyroscopes
/// @param accel_noise 3-element white noise for accelerometers
/// @return Noise contributions to add to ideal sensor outputs
template <typename Scalar>
IMUNoiseSample<Scalar> step(
    IMUNoiseState<Scalar>& state,
    const IMUNoiseCoeffs& coeffs,
    const Eigen::Vector<Scalar, 3>& gyro_noise,
    const Eigen::Vector<Scalar, 3>& accel_noise);

/// IMU noise output sample
template <typename Scalar>
struct IMUNoiseSample {
    Eigen::Vector<Scalar, 3> gyro;   ///< Gyroscope noise [rad/s]
    Eigen::Vector<Scalar, 3> accel;  ///< Accelerometer noise [m/s²]
};

/// Typical IMU grade parameters
namespace grades {
    /// Consumer-grade MEMS IMU (e.g., smartphone)
    AllanParams<double> consumer_gyro();
    AllanParams<double> consumer_accel();
    
    /// Industrial-grade MEMS IMU
    AllanParams<double> industrial_gyro();
    AllanParams<double> industrial_accel();
    
    /// Tactical-grade IMU
    AllanParams<double> tactical_gyro();
    AllanParams<double> tactical_accel();
    
    /// Navigation-grade IMU
    AllanParams<double> navigation_gyro();
    AllanParams<double> navigation_accel();
}

} // namespace vulcan::allan
```

---

### CMake Updates

#### [MODIFY] [CMakeLists.txt](file:///home/tanged/sources/temp/vulcan/tests/CMakeLists.txt)

Add sensor noise tests:

```cmake
# Sensor noise tests
add_executable(test_sensors
    sensors/test_gaussian_noise.cpp
    sensors/test_random_walk.cpp
    sensors/test_bias_instability.cpp
    sensors/test_markov_process.cpp
    sensors/test_allan_variance.cpp
)
target_link_libraries(test_sensors PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_sensors)
```

---

### Main Header Updates

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/vulcan.hpp)

Add sensors module include:

```cpp
// Sensor noise models
#include <vulcan/sensors/NoiseTypes.hpp>
#include <vulcan/sensors/GaussianNoise.hpp>
#include <vulcan/sensors/RandomWalk.hpp>
#include <vulcan/sensors/BiasInstability.hpp>
#include <vulcan/sensors/MarkovProcess.hpp>
#include <vulcan/sensors/AllanVarianceNoise.hpp>
```

---

## Implementation Details

### Janus Compatibility Requirements

1. **All scalar operations use `janus::` math functions** (e.g., `janus::sqrt`, `janus::exp`)
2. **No `if/else` branching on `Scalar` types** — use `janus::where` if needed
3. **Filter coefficients computed with `double`** — structural parameters
4. **State transitions templated on `Scalar`** — supports symbolic graph generation

### Discretization Approach

For first-order Markov processes, use exact discretization:
- State transition: `φ = exp(-Δt/τ)`
- Process noise: `q = σ·sqrt(1 - φ²)`

This preserves the steady-state variance `σ²` regardless of time step.

### Allan Variance Relationship

The Allan deviation slope on a log-log plot identifies noise types:

| Noise Type | Slope | Parameter |
|------------|-------|-----------|
| White noise (ARW) | -1/2 | N [°/√hr] |
| Bias instability | 0 | B [°/hr] |
| Rate random walk | +1/2 | K [°/hr/√hr] |

Parameters are extracted at characteristic τ values and converted to SI units for implementation.

---

## Verification Plan

### Automated Tests

All tests run via:
```bash
./scripts/ci.sh
```

Or individually after building:
```bash
cd build && ctest -R test_sensors -VV
```

#### Test Coverage

| Test File | Coverage |
|-----------|----------|
| `test_gaussian_noise.cpp` | Scaling, PSD variance, symbolic compatibility |
| `test_random_walk.cpp` | Integration behavior, variance growth with time |
| `test_bias_instability.cpp` | Steady-state variance, correlation time |
| `test_markov_process.cpp` | Discretization accuracy, symbolic evaluation |
| `test_allan_variance.cpp` | Composite model, grade presets, 3-axis output |

#### Specific Test Cases

1. **Variance verification**: Run multiple realizations, verify empirical variance matches expected
2. **Correlation time**: Verify autocorrelation decay matches exp(-Δt/τ)
3. **Symbolic mode**: Generate symbolic graphs, evaluate against numeric results
4. **Edge cases**: Zero time step, very long correlation times, zero noise

### Manual Verification

1. Run examples demonstrating noise time series visualization
2. Compare Allan deviation computed from simulated data against input parameters

---

## Dependencies

- **Eigen3**: For 3-axis vector operations
- **Janus**: For symbolic compatibility
- **C++ `<random>`**: For numeric-only RNG in examples/tests (not in templates)

---

## Open Questions

> [!IMPORTANT]
> 1. **Random number generation**: Should we provide RNG utilities, or expect users to supply noise samples? (Current plan: supply samples externally)
> 2. **Quantization noise**: Should we include this (slope -1 on Allan plot)? Less common in modern IMUs.
> 3. **Rate ramp**: Should we include drift rate modeling (slope +1)?

---

## Out of Scope

- Hardware-in-the-loop integration
- Real-time noise filtering/estimation
- Kalman filter implementation
- Sensor fusion algorithms
