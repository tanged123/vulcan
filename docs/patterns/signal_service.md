# Signal Service Pattern

Design pattern for component model signal registration in Icarus, using Vulcan's signal primitives as the wire format foundation.

## Overview

Every Icarus component model registers its I/O interface during initialization. The **Signal Service** provides a unified API for:

| Registration | Purpose |
|--------------|---------|
| Parameters (int) | Configuration values, modes, counters |
| Parameters (double) | Physical constants, tuning values |
| Input signals | Data consumed from other components |
| Output signals | Data produced for other components / telemetry |

All signals use Vulcan's `SignalType` and `SignalLifecycle` for wire format compatibility.

## Component Init Pattern

```cpp
#include <vulcan/io/Signal.hpp>
#include <icarus/SignalService.hpp>

class RocketDynamics : public ComponentModel {
public:
    void init(SignalService& signals) override {
        // =====================================================================
        // Integer parameters (static configuration)
        // =====================================================================
        signals.bind_param("num_engines", &num_engines_)
            .unit("count")
            .info("Number of active engines");
            
        signals.bind_param("guidance_mode", &guidance_mode_)
            .semantic("enum")
            .info("0=inertial, 1=terminal, 2=landing");
        
        // =====================================================================
        // Double parameters (physical constants)
        // =====================================================================
        signals.bind_param("dry_mass", &dry_mass_)
            .unit("kg")
            .info("Vehicle dry mass excluding propellant");
            
        signals.bind_param("isp_vacuum", &isp_vacuum_)
            .unit("s")
            .info("Specific impulse in vacuum");
        
        // =====================================================================
        // Input signals (bound to member variables, updated before step)
        // =====================================================================
        signals.bind_input("thrust_body", &thrust_body_)
            .unit("N")
            .info("Thrust vector in body frame");
            
        signals.bind_input("throttle", &throttle_)
            .unit("ratio")
            .info("Throttle command [0, 1]");
            
        signals.bind_input("attitude_cmd", &attitude_cmd_);
        
        // =====================================================================
        // Output signals (bound to member variables, captured after step)
        // =====================================================================
        signals.bind_output("position_ecef", &position_)
            .unit("m")
            .info("Position in ECEF frame");
            
        signals.bind_output("velocity_ecef", &velocity_)
            .unit("m/s");
            
        signals.bind_output("attitude", &attitude_);
        
        signals.bind_output("altitude_agl", &altitude_agl_)
            .unit("m")
            .info("Altitude above ground level");
            
        signals.bind_output("flight_phase", &flight_phase_)
            .semantic("enum")
            .info("0=prelaunch, 1=powered, 2=coast, 3=terminal");
    }
    
    void step(double dt) override {
        // Inputs already populated by framework
        Eigen::Vector3d accel = thrust_body_ * throttle_ / mass();
        velocity_ += accel * dt;
        position_ += velocity_ * dt;
        // Outputs automatically captured after step()
    }

private:
    // Parameters (initialized with defaults, static after init)
    int32_t num_engines_ = 9;
    int32_t guidance_mode_ = 0;
    double dry_mass_ = 22200.0;
    double isp_vacuum_ = 311.0;
    
    // Inputs (populated before each step)
    Eigen::Vector3d thrust_body_ = Eigen::Vector3d::Zero();
    double throttle_ = 0.0;
    Eigen::Quaterniond attitude_cmd_ = Eigen::Quaterniond::Identity();
    
    // Outputs (captured after each step)
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
    double altitude_agl_ = 0.0;
    int32_t flight_phase_ = 0;
};
```

## Signal Lifecycle Mapping

| Registration Type | `SignalLifecycle` | Wire Behavior |
|-------------------|-------------------|---------------|
| `add_param_*` | `Static` | Sent once at handshake |
| `add_input_*` | `Dynamic` | Updated each tick |
| `add_output_*` | `Dynamic` | Streamed at 60Hz |

## SignalService API

```cpp
namespace icarus {

/// Builder for signal metadata (returned by bind_* methods)
class SignalBuilder {
public:
    SignalBuilder& unit(std::string_view u);
    SignalBuilder& semantic(std::string_view s);  // "enum", "boolean", etc.
    SignalBuilder& info(std::string_view description);
};

class SignalService {
public:
    // =========================================================================
    // Parameter binding (Static lifecycle)
    // Default value comes from member initializer, not bind call
    // =========================================================================
    
    SignalBuilder& bind_param(std::string_view name, int32_t* ptr);
    SignalBuilder& bind_param(std::string_view name, int64_t* ptr);
    SignalBuilder& bind_param(std::string_view name, double* ptr);
    
    // =========================================================================
    // Input binding (Dynamic lifecycle, updated before step)
    // =========================================================================
    
    SignalBuilder& bind_input(std::string_view name, double* ptr);
    SignalBuilder& bind_input(std::string_view name, int32_t* ptr);
    SignalBuilder& bind_input(std::string_view name, Eigen::Vector3d* ptr);
    SignalBuilder& bind_input(std::string_view name, Eigen::Quaterniond* ptr);
    
    // =========================================================================
    // Output binding (Dynamic lifecycle, captured after step)
    // =========================================================================
    
    SignalBuilder& bind_output(std::string_view name, double* ptr);
    SignalBuilder& bind_output(std::string_view name, int32_t* ptr);
    SignalBuilder& bind_output(std::string_view name, Eigen::Vector3d* ptr);
    SignalBuilder& bind_output(std::string_view name, Eigen::Quaterniond* ptr);
    
    // =========================================================================
    // Framework operations (called by simulation runner)
    // =========================================================================
    
    void propagate_inputs();                         // outputs → inputs
    void capture_outputs(vulcan::io::Frame& frame);  // outputs → Frame
    
    // =========================================================================
    // Schema export
    // =========================================================================
    
    vulcan::io::TelemetrySchema to_schema() const;
    std::string to_json() const;
};

} // namespace icarus
```

## Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ICARUS SIMULATION                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐                │
│  │   Guidance   │────▶│   Dynamics   │────▶│   Sensors    │                │
│  │   Component  │     │   Component  │     │   Component  │                │
│  └──────────────┘     └──────────────┘     └──────────────┘                │
│         │                    │                    │                         │
│         ▼                    ▼                    ▼                         │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     SIGNAL SERVICE (Data Backplane)                  │   │
│  │  • Routes signals between components                                 │   │
│  │  • Maintains TelemetrySchema from all registrations                  │   │
│  │  • Provides Frame buffer for current timestep                        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                        │
└────────────────────────────────────┼────────────────────────────────────────┘
                                     │
                    ┌────────────────┴────────────────┐
                    │                                 │
                    ▼                                 ▼
           ┌───────────────┐                 ┌───────────────┐
           │ VULCAN HDF5   │                 │    HERMES     │
           │ (Post-process)│                 │ (Real-time)   │
           └───────────────┘                 └───────────────┘
```

## Wire Format Compatibility

The SignalService internally translates registrations to Vulcan signals:

```cpp
// Registration
signals.add_output_vec3("position_ecef", "m");

// Becomes (internally):
schema.add_vec3("rocket.position_ecef", SignalLifecycle::Dynamic, "m");
// → position_ecef.x (double, dynamic, "m")
// → position_ecef.y (double, dynamic, "m")
// → position_ecef.z (double, dynamic, "m")
```

Component names are prefixed automatically to avoid collisions.

## Connection Validation

During simulation init, the SignalService validates that:
1. Every input has a matching output from some component
2. No duplicate signal names exist
3. Type compatibility between connected signals

```cpp
void Simulation::init() {
    // 1. All components register their signals
    for (auto& component : components_) {
        component->init(signal_service_);
    }
    
    // 2. Validate connections
    signal_service_.validate_connections();  // throws on errors
    
    // 3. Export schema for telemetry
    telemetry_schema_ = signal_service_.to_schema();
}
```

## Telemetry Selection

Not all signals need to be streamed. The SignalService supports filtering:

```cpp
// Stream only outputs (not internal connections)
auto telemetry_schema = signal_service_.to_schema(
    SignalFilter::OutputsOnly
);

// Stream specific signals by pattern
auto filtered = signal_service_.to_schema(
    SignalFilter::ByPattern("*.position_*")
);
```

## See Also

- [vulcan/io/Signal.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/Signal.hpp) - Wire format types
- [vulcan/io/TelemetrySchema.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/TelemetrySchema.hpp) - Schema builder
- [vulcan/io/Frame.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/Frame.hpp) - Data container
- [vulcan/io/FrameSerializer.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/FrameSerializer.hpp) - Binary serialization
