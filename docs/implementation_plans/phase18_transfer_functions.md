# Phase 18: Transfer Function Utilities

Discrete-time linear system models for actuators, filters, and frequency-domain modeling.

## Scope

| Category | Included | Notes |
|----------|----------|-------|
| First-order systems | ✅ | Time constant, discretized |
| Second-order systems | ✅ | ω_n, ζ, discretized |
| State-space | ✅ | (A, B, C, D) representation |
| Discretization | ✅ | ZOH, Tustin/bilinear |
| Rate limiters | ✅ | Actuator dynamics |
| Saturation | ✅ | Position/rate limits |
| Frequency response | ❌ | Analysis tool, not runtime |
| PID controllers | ❌ | Icarus GNC concern |

---

## Proposed API

### First-Order Systems (`include/vulcan/dynamics/FirstOrder.hpp`)

```cpp
namespace vulcan::dynamics {

/// First-order system: τ*ẏ + y = K*u
/// Discretized: y[k+1] = a*y[k] + b*u[k]
template <typename Scalar>
struct FirstOrderSystem {
    Scalar a;    ///< State coefficient: exp(-dt/τ)
    Scalar b;    ///< Input coefficient: K*(1 - a)
    Scalar tau;  ///< Time constant [s]
    Scalar K;    ///< DC gain
    Scalar dt;   ///< Sample time [s]
};

/// Create first-order system from time constant
template <typename Scalar>
FirstOrderSystem<Scalar> first_order(double tau, double K, double dt);

/// Step the system
template <typename Scalar>
Scalar step(const FirstOrderSystem<Scalar>& sys,
            const Scalar& state, const Scalar& input);

}
```

### Second-Order Systems (`include/vulcan/dynamics/SecondOrder.hpp`)

```cpp
namespace vulcan::dynamics {

/// Second-order system: ÿ + 2ζω_n*ẏ + ω_n²*y = K*ω_n²*u
/// State-space form with 2 states
template <typename Scalar>
struct SecondOrderSystem {
    Eigen::Matrix<Scalar, 2, 2> A;  ///< State matrix
    Eigen::Matrix<Scalar, 2, 1> B;  ///< Input matrix
    Eigen::Matrix<Scalar, 1, 2> C;  ///< Output matrix
    Scalar D;                        ///< Feedthrough
    Scalar omega_n;                  ///< Natural frequency [rad/s]
    Scalar zeta;                     ///< Damping ratio
    Scalar K;                        ///< DC gain
    Scalar dt;                       ///< Sample time [s]
};

/// Create second-order system
template <typename Scalar>
SecondOrderSystem<Scalar> second_order(double omega_n, double zeta,
                                        double K, double dt);

/// Step the system (state is [y, ẏ])
template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> step(const SecondOrderSystem<Scalar>& sys,
                                  const Eigen::Matrix<Scalar, 2, 1>& state,
                                  const Scalar& input);

}
```

### Actuator Models (`include/vulcan/dynamics/Actuator.hpp`)

```cpp
namespace vulcan::dynamics {

/// Rate-limited actuator with position limits
template <typename Scalar>
struct ActuatorLimits {
    Scalar pos_min;     ///< Minimum position
    Scalar pos_max;     ///< Maximum position
    Scalar rate_max;    ///< Maximum rate magnitude
};

/// Apply rate limiting
template <typename Scalar>
Scalar rate_limit(const Scalar& current, const Scalar& commanded,
                  double rate_max, double dt);

/// Apply position saturation
template <typename Scalar>
Scalar saturate(const Scalar& value, const Scalar& min, const Scalar& max);

/// Combined actuator model: second-order + rate limit + saturation
template <typename Scalar>
struct ActuatorState {
    Eigen::Matrix<Scalar, 2, 1> filter_state;  ///< Second-order filter state
    Scalar position;                            ///< Current position
};

template <typename Scalar>
ActuatorState<Scalar> actuator_step(const ActuatorState<Scalar>& state,
                                     const Scalar& command,
                                     const SecondOrderSystem<Scalar>& dynamics,
                                     const ActuatorLimits<double>& limits);

}
```

### Discretization (`include/vulcan/dynamics/Discretize.hpp`)

```cpp
namespace vulcan::dynamics {

/// Zero-order hold discretization
/// A_d = exp(A*dt), B_d = A^(-1)*(A_d - I)*B
template <typename Scalar, int N>
void discretize_zoh(const Eigen::Matrix<Scalar, N, N>& A_c,
                    const Eigen::Matrix<Scalar, N, 1>& B_c,
                    double dt,
                    Eigen::Matrix<Scalar, N, N>& A_d,
                    Eigen::Matrix<Scalar, N, 1>& B_d);

/// Tustin (bilinear) discretization
/// Preserves frequency response better than ZOH
template <typename Scalar, int N>
void discretize_tustin(const Eigen::Matrix<Scalar, N, N>& A_c,
                       const Eigen::Matrix<Scalar, N, 1>& B_c,
                       double dt,
                       Eigen::Matrix<Scalar, N, N>& A_d,
                       Eigen::Matrix<Scalar, N, 1>& B_d);

}
```

---

## File Structure

```
include/vulcan/dynamics/
├── Dynamics.hpp         # Aggregate header
├── FirstOrder.hpp       # First-order systems
├── SecondOrder.hpp      # Second-order systems
├── Actuator.hpp         # Actuator models with limits
└── Discretize.hpp       # Continuous → discrete

tests/dynamics/
├── test_first_order.cpp
├── test_second_order.cpp
├── test_actuator.cpp
└── test_discretize.cpp

docs/user_guides/
└── dynamics.md

examples/dynamics/
└── actuator_demo.cpp
```

---

## Verification Plan

### Reference Values
| Test | Source |
|------|--------|
| First-order step response | Analytical: y(t) = K*(1 - exp(-t/τ)) |
| Second-order response | Franklin, Powell & Emami-Naeini |
| ZOH discretization | Ogata, Discrete-Time Control Systems |
| Tustin transform | Åström & Wittenmark |

### Symbolic Validation
- Instantiate systems with `casadi::MX`
- Verify step functions trace properly
- Test autodiff through rate limiters using `janus::where`
