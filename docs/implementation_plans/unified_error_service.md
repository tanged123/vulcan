# Unified Error Service for Aerospace Tooling Stack

**Goal**: Standardize error handling across Janus, Vulcan, Icarus, and Hermes.  
**Status**: 📋 Proposal  
**Created**: 2025-12-21  

---

## Executive Summary

This plan evaluates whether to standardize error handling across your four engineering tools. **Recommendation: Yes, with caveats.**

### Tool Roles

| Tool | Purpose | Error Context |
|------|---------|---------------|
| **Janus** | Symbolic/numeric math framework | Math errors (interpolation, integration, AD) |
| **Vulcan** | Aerospace utilities library | Domain errors (atmosphere, coordinates, gravity) |
| **Icarus** | Simulation framework (planned) | Runtime errors (component init, signal routing) |
| **Hermes** | Real-time telemetry (planned) | I/O errors (serialization, transport) |

### Does Standardization Make Sense?

| Factor | Verdict | Rationale |
|--------|---------|-----------|
| **Shared dependency chain** | ✅ Yes | Vulcan→Janus, Icarus→Vulcan, Hermes→Vulcan |
| **Cross-layer error propagation** | ✅ Yes | Simulation errors may originate in Janus math |
| **Consistent debugging** | ✅ Yes | Unified error format simplifies root cause analysis |
| **Independent versioning** | ⚠️ Caution | Tight coupling could complicate releases |
| **Differing contexts** | ⚠️ Caution | Real-time (Hermes) vs batch (Janus) needs differ |

**Bottom line**: Standardize the error **hierarchy and format**, but keep tool-specific exception types for clarity.

---

## Current State Analysis

### Janus (✅ Already Standardized)

Janus has a completed error handling enhancement with a clean hierarchy:

```
std::runtime_error
 └── janus::JanusError          "[janus] ..."
      ├── janus::InvalidArgument    (input validation)
      ├── janus::RuntimeError       (eval failures)
      ├── janus::InterpolationError (grid/data issues)
      └── janus::IntegrationError   (ODE solver issues)
```

**Status**: Done. This is the reference implementation.

### Vulcan (⚠️ Ad-hoc)

Vulcan currently uses raw `std::runtime_error` throughout:

| Module | Error Pattern | Issues |
|--------|---------------|--------|
| `TelemetrySchema.hpp` | `throw std::runtime_error("Signal not found: ...")` | No namespace prefix |
| `Frame.hpp` | `throw std::runtime_error("Type mismatch: ...")` | Inconsistent format |
| `CSVExport.hpp` | `throw std::runtime_error("Failed to open: ...")` | Generic message |
| `FrameSerializer.hpp` | `throw std::runtime_error("Invalid frame data")` | Missing context |
| `TableInterpolator.hpp` | Uses `janus::InterpolationError` | ✅ Correct |

**Problem**: Mixed error types, no unified hierarchy.

### Icarus (🚧 Planned)

Per `signal_service.md`, Icarus will need errors for:
- Component initialization failures
- Signal connection validation
- Runtime simulation errors

### Hermes (🚧 Planned)

Will need errors for:
- Transport failures (connection, timeout)
- Serialization errors
- Protocol violations

---

## User Review Required

> [!IMPORTANT]
> **Key Design Decision: Where Should the Error Hierarchy Live?**
>
> **Option A**: Keep in Janus, export to dependents *(recommended)*
> - Janus is the foundation; all tools depend on it
> - Avoids duplication, single source of truth
> - Tools add domain-specific errors that derive from `janus::JanusError`
>
> **Option B**: Create independent `olympus-error` package
> - True independence between tools
> - More complexity, requires coordination
>
> **Option C**: Each tool has its own hierarchy (status quo for Vulcan)
> - Maximum flexibility
> - Inconsistent debugging experience

> [!WARNING]
> **Breaking Change Risk**
>
> Vulcan currently throws `std::runtime_error`. Changing to `vulcan::VulcanError` (deriving from `janus::JanusError`) will break code that catches specific exception types.
>
> **Mitigation**: All new errors derive from `std::runtime_error`, so `catch(std::runtime_error&)` still works.

---

## Proposed Architecture

### Unified Error Hierarchy

```
std::runtime_error
 │
 └── janus::JanusError                     "[janus] ..."
      │
      ├── [Janus domain errors - existing]
      │    ├── janus::InvalidArgument
      │    ├── janus::RuntimeError
      │    ├── janus::InterpolationError
      │    └── janus::IntegrationError
      │
      ├── vulcan::VulcanError               "[vulcan] ..."
      │    ├── vulcan::IOError               (file/stream errors)
      │    ├── vulcan::SignalError           (schema/frame errors)
      │    ├── vulcan::AtmosphereError       (out-of-range altitude, etc.)
      │    ├── vulcan::CoordinateError       (invalid LLA, etc.)
      │    └── vulcan::GravityError          (invalid model params)
      │
      ├── icarus::IcarusError               "[icarus] ..."
      │    ├── icarus::ComponentError        (init/step failures)
      │    ├── icarus::ConnectionError       (signal routing failures)
      │    └── icarus::SimulationError       (runtime simulation failures)
      │
      └── hermes::HermesError               "[hermes] ..."
           ├── hermes::TransportError        (connection failures)
           ├── hermes::ProtocolError         (invalid messages)
           └── hermes::SerializationError    (encode/decode failures)
```

### Error Message Format

Standardize on the Janus format with optional context:

```
[namespace] Context: Message
```

Examples:
```
[vulcan] Signal: Signal not found: altitude_agl
[vulcan] Frame: Type mismatch for signal: thrust (expected vec3, got double)
[icarus] Connection: No producer found for input: guidance.attitude_cmd
[hermes] Transport: Connection refused to host 192.168.1.1:8080
```

---

## Proposed Changes

### Phase 1: Vulcan Error Types

#### [NEW] `include/vulcan/core/VulcanError.hpp`

```cpp
#pragma once
/**
 * @file VulcanError.hpp
 * @brief Exception hierarchy for Vulcan aerospace library
 *
 * Derives from janus::JanusError for unified error handling across
 * the Janus/Vulcan/Icarus/Hermes toolchain.
 */

#include <janus/core/JanusError.hpp>
#include <string>

namespace vulcan {

/**
 * @brief Base exception for all Vulcan errors
 */
class VulcanError : public janus::JanusError {
  public:
    explicit VulcanError(const std::string &what)
        : janus::JanusError("[vulcan] " + what) {}
};

/**
 * @brief File/stream I/O errors
 */
class IOError : public VulcanError {
  public:
    explicit IOError(const std::string &what)
        : VulcanError("IO: " + what) {}
};

/**
 * @brief Signal schema and frame errors
 */
class SignalError : public VulcanError {
  public:
    explicit SignalError(const std::string &what)
        : VulcanError("Signal: " + what) {}
};

/**
 * @brief Atmospheric model errors (altitude out of range, etc.)
 */
class AtmosphereError : public VulcanError {
  public:
    explicit AtmosphereError(const std::string &what)
        : VulcanError("Atmosphere: " + what) {}
};

/**
 * @brief Coordinate system errors (invalid LLA, singularities)
 */
class CoordinateError : public VulcanError {
  public:
    explicit CoordinateError(const std::string &what)
        : VulcanError("Coordinate: " + what) {}
};

/**
 * @brief Gravity model errors
 */
class GravityError : public VulcanError {
  public:
    explicit GravityError(const std::string &what)
        : VulcanError("Gravity: " + what) {}
};

} // namespace vulcan
```

---

#### [MODIFY] `include/vulcan/io/TelemetrySchema.hpp`

| Line | Before | After |
|------|--------|-------|
| 161 | `throw std::runtime_error("Signal not found: " + name)` | `throw vulcan::SignalError("Signal not found: " + name)` |
| 178 | `throw std::runtime_error("Signal not found: " + name)` | `throw vulcan::SignalError("Signal not found: " + name)` |
| 229 | `throw std::runtime_error("Schema has no signals")` | `throw vulcan::SignalError("Schema has no signals")` |
| 278 | `throw std::runtime_error("Duplicate signal name: " + desc.name)` | `throw vulcan::SignalError("Duplicate signal name: " + desc.name)` |

---

#### [MODIFY] `include/vulcan/io/Frame.hpp`

Replace all `throw std::runtime_error("Type mismatch...")` with:
```cpp
throw vulcan::SignalError("Type mismatch for signal: " + signal)
```

---

#### [MODIFY] `include/vulcan/io/CSVExport.hpp`

| Line | Before | After |
|------|--------|-------|
| 94 | `throw std::runtime_error("Failed to open CSV file: " + csv_path)` | `throw vulcan::IOError("Failed to open CSV file: " + csv_path)` |

---

#### [MODIFY] `include/vulcan/io/FrameSerializer.hpp`

Replace `std::runtime_error` with `vulcan::SignalError` for data format errors.

---

### Phase 2: Icarus Error Types (Future)

#### [NEW] `include/icarus/core/IcarusError.hpp`

```cpp
#pragma once
#include <janus/core/JanusError.hpp>

namespace icarus {

class IcarusError : public janus::JanusError {
  public:
    explicit IcarusError(const std::string &what)
        : janus::JanusError("[icarus] " + what) {}
};

class ComponentError : public IcarusError {
  public:
    explicit ComponentError(const std::string &what)
        : IcarusError("Component: " + what) {}
};

class ConnectionError : public IcarusError {
  public:
    explicit ConnectionError(const std::string &what)
        : IcarusError("Connection: " + what) {}
};

class SimulationError : public IcarusError {
  public:
    explicit SimulationError(const std::string &what)
        : IcarusError("Simulation: " + what) {}
};

} // namespace icarus
```

---

### Phase 3: Hermes Error Types (Future)

#### [NEW] `include/hermes/core/HermesError.hpp`

```cpp
#pragma once
#include <janus/core/JanusError.hpp>

namespace hermes {

class HermesError : public janus::JanusError {
  public:
    explicit HermesError(const std::string &what)
        : janus::JanusError("[hermes] " + what) {}
};

class TransportError : public HermesError {
  public:
    explicit TransportError(const std::string &what)
        : HermesError("Transport: " + what) {}
};

class ProtocolError : public HermesError {
  public:
    explicit ProtocolError(const std::string &what)
        : HermesError("Protocol: " + what) {}
};

class SerializationError : public HermesError {
  public:
    explicit SerializationError(const std::string &what)
        : HermesError("Serialization: " + what) {}
};

} // namespace hermes
```

---

## Verification Plan

### Automated Tests

```bash
# Vulcan - verify existing tests still pass (backward compatible)
cd vulcan && ./scripts/test.sh

# Test new exception hierarchy
ctest --output-on-failure -R "error|Error"
```

### New Tests to Add

#### [NEW] `tests/core/test_vulcan_error.cpp`

```cpp
#include <gtest/gtest.h>
#include <vulcan/core/VulcanError.hpp>

TEST(VulcanErrorTests, BaseErrorCatchable) {
    EXPECT_THROW(throw vulcan::VulcanError("test"), janus::JanusError);
    EXPECT_THROW(throw vulcan::VulcanError("test"), std::runtime_error);
}

TEST(VulcanErrorTests, SignalErrorCatchable) {
    EXPECT_THROW(throw vulcan::SignalError("test"), vulcan::VulcanError);
    EXPECT_THROW(throw vulcan::SignalError("test"), janus::JanusError);
}

TEST(VulcanErrorTests, MessageFormat) {
    try {
        throw vulcan::SignalError("Signal not found: foo");
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("[vulcan]") != std::string::npos);
        EXPECT_TRUE(msg.find("Signal:") != std::string::npos);
    }
}
```

---

## Task Breakdown

### Phase 1: Vulcan Core (Can Do Now)
- [ ] Create `VulcanError.hpp` with hierarchy
- [ ] Add include to `vulcan.hpp`
- [ ] Refactor I/O module throws
- [ ] Add unit tests
- [ ] Update documentation

### Phase 2: Icarus (When Icarus Starts)
- [ ] Create `IcarusError.hpp`
- [ ] Integrate with SignalService
- [ ] Add tests

### Phase 3: Hermes (When Hermes Starts)
- [ ] Create `HermesError.hpp`
- [ ] Integrate with transport layer
- [ ] Add tests

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Breaking existing catch blocks | Medium | Derive from `std::runtime_error` |
| Cross-repo dependencies | Medium | Clear versioning policy |
| Over-engineering for small codebase | Low | Start minimal, expand as needed |

---

## Alternative Considered: No Standardization

**Pros**:
- Less coordination overhead
- Each tool evolves independently

**Cons**:
- Inconsistent error messages
- Difficult to trace errors across layers (e.g., Icarus → Vulcan → Janus)
- Code duplication in error handling patterns

**Verdict**: Given the shared dependency chain and the goal of building an integrated simulation stack, standardization provides more value than flexibility.

---

## Appendix: Error Handling Best Practices

### When to Throw

1. **Construction failures**: Invalid parameters that make the object unusable
2. **Precondition violations**: Invalid inputs to functions
3. **Invariant violations**: Internal consistency checks (rare)

### When NOT to Throw

1. **Expected edge cases**: Use `std::optional` or `janus::where()` for symbolic compat
2. **Performance-critical paths**: Use error codes or return values
3. **CasADi symbolic mode**: Cannot throw on symbolic values; use `janus::where()`

### Error Context Checklist

Good error messages include:
- ✅ What went wrong
- ✅ What was expected
- ✅ What was received
- ✅ How to fix it (if applicable)

```cpp
// Bad
throw vulcan::SignalError("Invalid signal");

// Good  
throw vulcan::SignalError("Signal 'altitude' has wrong type: expected vec3, got double. "
                          "Check schema definition in init().");
```
