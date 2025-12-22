# State Estimation

The estimation module provides Kalman filter implementations for navigation and sensor fusion. All functions are stateless utilities that take in a filter state and return an updated state.

## Quick Start

```cpp
#include <vulcan/estimation/Estimation.hpp>
using namespace vulcan::estimation;

// 2D position/velocity tracking
constexpr int N = 2; // State: [pos, vel]
constexpr int M = 1; // Measurement: pos only

// Initialize filter
FilterState<double, N> state;
state.x << 0.0, 0.0;
state.P = Eigen::Matrix2d::Identity() * 10.0;

// System matrices
Eigen::Matrix2d F;
F << 1, dt, 0, 1; // Constant velocity
Eigen::Matrix<double, M, N> H;
H << 1, 0; // Observe position

// Noise specification
auto Q = ProcessNoise<N>::scalar(0.01);
auto R = MeasurementNoise<M>::scalar(1.0);

// Run one step
state = kf_predict<double, N>(state, F, Q);
Eigen::Matrix<double, M, 1> z; z << measurement;
state = kf_update<double, N, M>(state, z, H, R);
```

## Filter Types

### Linear Kalman Filter (KF)

For linear systems with known dynamics:

| Function | Description |
|----------|-------------|
| `kf_predict` | Propagate state: x⁻ = Fx + Bu, P⁻ = FPFᵀ + Q |
| `kf_update` | Incorporate measurement with Kalman gain |
| `kf_step` | Combined predict + update |

### Extended Kalman Filter (EKF)

For nonlinear systems with user-provided Jacobians:

```cpp
// Nonlinear measurement (range-bearing)
auto h = [](const Eigen::Vector2d& x) {
    double r = x.norm();
    double theta = std::atan2(x(1), x(0));
    return Eigen::Vector2d(r, theta);
};

// Jacobian at current state
auto H = compute_jacobian(state.x);

state = ekf_update<double, N, M>(state, z, h, H, R);
```

### Unscented Kalman Filter (UKF)

For nonlinear systems without requiring Jacobians:

```cpp
UKFParams params;
params.alpha = 0.1; // Sigma point spread

// No Jacobian needed!
state = ukf_update<N, M>(state, z, h, R, params);
```

## Utility Functions

### Measurement Gating

Reject outlier measurements using chi-squared test:

```cpp
double threshold = chi_squared_threshold(M, 0.95);
if (passes_gate<double, M>(innovation, S, threshold)) {
    state = kf_update(...);
}
```

### Joseph Form Update

Numerically stable covariance update:

```cpp
auto P = joseph_update<N, M>(P_prior, K, H, R);
```

## Common Types

| Type | Description |
|------|-------------|
| `FilterState<Scalar, N>` | State vector `x` and covariance `P` |
| `ProcessNoise<N>` | Process noise `Q` with factory methods |
| `MeasurementNoise<M>` | Measurement noise `R` with factory methods |
| `UKFParams` | UKF tuning: alpha, beta, kappa |

## When to Use Which Filter

| Scenario | Recommended |
|----------|-------------|
| Linear system | KF |
| Nonlinear, Jacobian available | EKF |
| Nonlinear, Jacobian hard to derive | UKF |
| Highly nonlinear / multi-modal | UKF |

## Example Output

```
=== Linear Kalman Filter Demo ===
Step | True Pos | Est Pos  | Est Vel  | Pos Error
-----|----------|----------|----------|----------
   0 |     0.50 |     0.93 |     0.41 |     0.43
   1 |     1.00 |     1.37 |     2.47 |     0.37
   ...
  19 |    10.00 |     9.98 |     4.96 |     0.02
```

## See Also

- [Transfer Functions](transfer_functions.md) — Dynamic system discretization
- [Sensors](sensors.md) — Noise models for filter tuning
