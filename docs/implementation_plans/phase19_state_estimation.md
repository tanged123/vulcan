# Phase 19: State Estimation Utilities

Kalman filter implementations for navigation and sensor fusion.

## Scope

| Category | Included | Notes |
|----------|----------|-------|
| Kalman Filter (KF) | ✅ | Linear systems |
| Extended KF (EKF) | ✅ | Nonlinear, Jacobian-based |
| Unscented KF (UKF) | ✅ | Sigma-point, no Jacobians |
| Square-root variants | ✅ | Numerical stability |
| Measurement gating | ✅ | Outlier rejection |
| Multi-rate fusion | ❌ | Application-specific |
| Particle filters | ❌ | Too heavy for Vulcan scope |

---

## Proposed API

### Common Types (`include/vulcan/estimation/EstimationTypes.hpp`)

```cpp
namespace vulcan::estimation {

/// Filter state container
template <typename Scalar, int N>
struct FilterState {
    Eigen::Matrix<Scalar, N, 1> x;      ///< State estimate
    Eigen::Matrix<Scalar, N, N> P;      ///< Covariance
};

/// Process noise specification
template <int N>
struct ProcessNoise {
    Eigen::Matrix<double, N, N> Q;      ///< Process noise covariance
};

/// Measurement noise specification  
template <int M>
struct MeasurementNoise {
    Eigen::Matrix<double, M, M> R;      ///< Measurement noise covariance
};

}
```

### Kalman Filter (`include/vulcan/estimation/KalmanFilter.hpp`)

```cpp
namespace vulcan::estimation {

/// Linear Kalman filter predict step
/// x̂⁻ = F*x̂ + B*u
/// P⁻ = F*P*Fᵀ + Q
template <typename Scalar, int N, int U>
FilterState<Scalar, N> kf_predict(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<double, N, N>& F,
    const Eigen::Matrix<double, N, U>& B,
    const Eigen::Matrix<Scalar, U, 1>& u,
    const ProcessNoise<N>& noise);

/// Linear Kalman filter update step
/// K = P⁻*Hᵀ*(H*P⁻*Hᵀ + R)⁻¹
/// x̂ = x̂⁻ + K*(z - H*x̂⁻)
/// P = (I - K*H)*P⁻
template <typename Scalar, int N, int M>
FilterState<Scalar, N> kf_update(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, M, 1>& z,
    const Eigen::Matrix<double, M, N>& H,
    const MeasurementNoise<M>& noise);

/// Combined predict + update
template <typename Scalar, int N, int U, int M>
FilterState<Scalar, N> kf_step(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, U, 1>& u,
    const Eigen::Matrix<Scalar, M, 1>& z,
    const Eigen::Matrix<double, N, N>& F,
    const Eigen::Matrix<double, N, U>& B,
    const Eigen::Matrix<double, M, N>& H,
    const ProcessNoise<N>& Q,
    const MeasurementNoise<M>& R);

}
```

### Extended Kalman Filter (`include/vulcan/estimation/ExtendedKF.hpp`)

```cpp
namespace vulcan::estimation {

/// EKF predict with nonlinear dynamics
/// x̂⁻ = f(x̂, u)
/// P⁻ = F*P*Fᵀ + Q  where F = ∂f/∂x
template <typename Scalar, int N, int U, typename DynamicsFunc>
FilterState<Scalar, N> ekf_predict(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, U, 1>& u,
    DynamicsFunc f,                              // x_next = f(x, u)
    const Eigen::Matrix<double, N, N>& F,        // Jacobian ∂f/∂x
    const ProcessNoise<N>& noise);

/// EKF update with nonlinear measurement
/// K = P⁻*Hᵀ*(H*P⁻*Hᵀ + R)⁻¹
/// x̂ = x̂⁻ + K*(z - h(x̂⁻))
template <typename Scalar, int N, int M, typename MeasurementFunc>
FilterState<Scalar, N> ekf_update(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, M, 1>& z,
    MeasurementFunc h,                           // z_pred = h(x)
    const Eigen::Matrix<double, M, N>& H,        // Jacobian ∂h/∂x
    const MeasurementNoise<M>& noise);

}
```

### Unscented Kalman Filter (`include/vulcan/estimation/UnscentedKF.hpp`)

```cpp
namespace vulcan::estimation {

/// UKF tuning parameters
struct UKFParams {
    double alpha = 1e-3;   ///< Spread of sigma points
    double beta = 2.0;     ///< Prior knowledge (2 = Gaussian)
    double kappa = 0.0;    ///< Secondary scaling
};

/// Generate sigma points
template <int N>
Eigen::Matrix<double, N, 2*N+1> sigma_points(
    const Eigen::Matrix<double, N, 1>& x,
    const Eigen::Matrix<double, N, N>& P,
    const UKFParams& params);

/// UKF predict (no Jacobians needed)
template <typename Scalar, int N, int U, typename DynamicsFunc>
FilterState<Scalar, N> ukf_predict(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, U, 1>& u,
    DynamicsFunc f,
    const ProcessNoise<N>& noise,
    const UKFParams& params = {});

/// UKF update (no Jacobians needed)
template <typename Scalar, int N, int M, typename MeasurementFunc>
FilterState<Scalar, N> ukf_update(
    const FilterState<Scalar, N>& state,
    const Eigen::Matrix<Scalar, M, 1>& z,
    MeasurementFunc h,
    const MeasurementNoise<M>& noise,
    const UKFParams& params = {});

}
```

### Utilities (`include/vulcan/estimation/EstimationUtils.hpp`)

```cpp
namespace vulcan::estimation {

/// Innovation (residual) and normalized innovation squared
template <typename Scalar, int M>
Scalar normalized_innovation_squared(
    const Eigen::Matrix<Scalar, M, 1>& innovation,
    const Eigen::Matrix<Scalar, M, M>& S);

/// Chi-squared gating for outlier rejection
template <typename Scalar, int M>
bool passes_gate(const Eigen::Matrix<Scalar, M, 1>& innovation,
                 const Eigen::Matrix<Scalar, M, M>& S,
                 double threshold);

/// Joseph form covariance update (numerically stable)
template <int N, int M>
Eigen::Matrix<double, N, N> joseph_update(
    const Eigen::Matrix<double, N, N>& P,
    const Eigen::Matrix<double, N, M>& K,
    const Eigen::Matrix<double, M, N>& H,
    const Eigen::Matrix<double, M, M>& R);

}
```

---

## File Structure

```
include/vulcan/estimation/
├── Estimation.hpp          # Aggregate header
├── EstimationTypes.hpp     # Common types
├── KalmanFilter.hpp        # Linear KF
├── ExtendedKF.hpp          # EKF
├── UnscentedKF.hpp         # UKF
└── EstimationUtils.hpp     # Gating, Joseph form

tests/estimation/
├── test_kalman_filter.cpp
├── test_ekf.cpp
├── test_ukf.cpp
└── test_utils.cpp

docs/user_guides/
└── estimation.md

examples/estimation/
└── sensor_fusion_demo.cpp
```

---

## Verification Plan

### Reference Values
| Test | Source |
|------|--------|
| KF equations | Bar-Shalom, Li & Kirubarajan |
| UKF sigma points | Julier & Uhlmann (1997) |
| Joseph form | Bucy & Joseph (1968) |

### Test Cases
1. **Constant velocity tracking** — 2D position/velocity with noisy position measurements
2. **Nonlinear bearing-only** — EKF/UKF comparison
3. **Covariance consistency** — Verify P stays positive definite

### Symbolic Compatibility
- KF/EKF/UKF with `double` for runtime simulation
- **Note**: Sigma-point generation in UKF requires Cholesky, which needs care for symbolic (may be numeric-only)
