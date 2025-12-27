# Phase 24: YAML Configuration Infrastructure

**Status:** Proposed
**Depends on:** None (new dependency: yaml-cpp)
**Consumers:** Icarus (config/routing), future Vulcan tools

---

## Overview

Add YAML parsing infrastructure to Vulcan as a shared utility layer. This provides consistent config handling across the ecosystem and avoids each project reinventing parsing logic.

---

## Design Philosophy

1. **Thin wrapper over yaml-cpp** - Don't reinvent, just provide ergonomic access
2. **Type-safe extraction** - Compile-time typed accessors for common types
3. **Good error messages** - Path-aware errors that help users fix problems
4. **Vulcan type support** - Native support for Vec3, Mat3, Quaternion
5. **No domain logic** - Generic utilities only; consumers define semantics

---

## Proposed Changes

### [NEW] YamlNode.hpp

**File:** `include/vulcan/io/YamlNode.hpp`

Thin wrapper providing type-safe access with path-aware error messages:

```cpp
#pragma once

#include <janus/core/JanusTypes.hpp>
#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

namespace vulcan {
namespace io {

/**
 * @brief Exception for YAML parsing/access errors
 */
class YamlError : public std::runtime_error {
public:
    YamlError(const std::string& path, const std::string& msg)
        : std::runtime_error("YAML error at '" + path + "': " + msg)
        , path_(path) {}

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

/**
 * @brief Wrapper around yaml-cpp node with ergonomic accessors
 *
 * Provides:
 * - Type-safe extraction with good error messages
 * - Path tracking for debugging
 * - Support for Vulcan/Janus math types
 */
class YamlNode {
public:
    // =========================================================================
    // Construction
    // =========================================================================

    /// Load from file
    static YamlNode LoadFile(const std::string& path);

    /// Parse from string
    static YamlNode Parse(const std::string& yaml_content);

    /// Wrap an existing yaml-cpp node
    explicit YamlNode(YAML::Node node, std::string path = "root");

    // =========================================================================
    // Navigation
    // =========================================================================

    /// Access child by key (map)
    YamlNode operator[](const std::string& key) const;

    /// Access child by index (sequence)
    YamlNode operator[](std::size_t index) const;

    /// Check if key exists
    bool Has(const std::string& key) const;

    /// Check if node is defined
    bool IsDefined() const { return node_.IsDefined(); }

    /// Check if node is a sequence
    bool IsSequence() const { return node_.IsSequence(); }

    /// Check if node is a map
    bool IsMap() const { return node_.IsMap(); }

    /// Get sequence size
    std::size_t Size() const { return node_.size(); }

    /// Get current path (for error messages)
    const std::string& Path() const { return path_; }

    // =========================================================================
    // Typed Accessors - Required (throws if missing)
    // =========================================================================

    /// Get required value (throws YamlError if missing or wrong type)
    template <typename T>
    T Require(const std::string& key) const;

    /// Get required value from current node
    template <typename T>
    T As() const;

    // =========================================================================
    // Typed Accessors - Optional (returns default if missing)
    // =========================================================================

    /// Get optional value with default
    template <typename T>
    T Get(const std::string& key, const T& default_value) const;

    /// Get optional value (returns nullopt if missing)
    template <typename T>
    std::optional<T> GetOptional(const std::string& key) const;

    // =========================================================================
    // Iteration
    // =========================================================================

    /// Iterate over sequence elements
    template <typename Func>
    void ForEach(Func&& func) const;

    /// Iterate over map entries: func(key, node)
    template <typename Func>
    void ForEachEntry(Func&& func) const;

    /// Convert sequence to vector
    template <typename T>
    std::vector<T> ToVector() const;

    // =========================================================================
    // Raw Access
    // =========================================================================

    /// Get underlying yaml-cpp node
    const YAML::Node& Raw() const { return node_; }

private:
    YAML::Node node_;
    std::string path_;
};

// =============================================================================
// Template Specializations for Common Types
// =============================================================================

// --- Scalars ---
template <> double YamlNode::As<double>() const;
template <> float YamlNode::As<float>() const;
template <> int YamlNode::As<int>() const;
template <> int64_t YamlNode::As<int64_t>() const;
template <> bool YamlNode::As<bool>() const;
template <> std::string YamlNode::As<std::string>() const;

// --- Janus types (double specialization) ---
template <> janus::Vec3<double> YamlNode::As<janus::Vec3<double>>() const;
template <> janus::Mat3<double> YamlNode::As<janus::Mat3<double>>() const;
template <> janus::Quaternion<double> YamlNode::As<janus::Quaternion<double>>() const;

// --- Containers ---
template <> std::vector<double> YamlNode::As<std::vector<double>>() const;
template <> std::vector<int> YamlNode::As<std::vector<int>>() const;
template <> std::vector<std::string> YamlNode::As<std::vector<std::string>>() const;

}  // namespace io
}  // namespace vulcan
```

---

### [NEW] YamlConvert.hpp

**File:** `include/vulcan/io/YamlConvert.hpp`

yaml-cpp conversion traits for Janus/Vulcan types:

```cpp
#pragma once

#include <janus/core/JanusTypes.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {

// =============================================================================
// janus::Vec3<T> - stored as [x, y, z]
// =============================================================================

template <typename T>
struct convert<janus::Vec3<T>> {
    static Node encode(const janus::Vec3<T>& v) {
        Node node;
        node.push_back(v.x());
        node.push_back(v.y());
        node.push_back(v.z());
        return node;
    }

    static bool decode(const Node& node, janus::Vec3<T>& v) {
        if (!node.IsSequence() || node.size() != 3) {
            return false;
        }
        v = janus::Vec3<T>{
            node[0].as<T>(),
            node[1].as<T>(),
            node[2].as<T>()
        };
        return true;
    }
};

// =============================================================================
// janus::Quaternion<T> - stored as [w, x, y, z] (scalar-first)
// =============================================================================

template <typename T>
struct convert<janus::Quaternion<T>> {
    static Node encode(const janus::Quaternion<T>& q) {
        Node node;
        node.push_back(q.w());
        node.push_back(q.x());
        node.push_back(q.y());
        node.push_back(q.z());
        return node;
    }

    static bool decode(const Node& node, janus::Quaternion<T>& q) {
        if (!node.IsSequence() || node.size() != 4) {
            return false;
        }
        q = janus::Quaternion<T>{
            node[0].as<T>(),  // w
            node[1].as<T>(),  // x
            node[2].as<T>(),  // y
            node[3].as<T>()   // z
        };
        return true;
    }
};

// =============================================================================
// janus::Mat3<T> - stored as [[row0], [row1], [row2]] or flat [9 elements]
// =============================================================================

template <typename T>
struct convert<janus::Mat3<T>> {
    static Node encode(const janus::Mat3<T>& m) {
        Node node;
        for (int i = 0; i < 3; ++i) {
            Node row;
            for (int j = 0; j < 3; ++j) {
                row.push_back(m(i, j));
            }
            node.push_back(row);
        }
        return node;
    }

    static bool decode(const Node& node, janus::Mat3<T>& m) {
        if (!node.IsSequence()) {
            return false;
        }

        // Nested format: [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
        if (node.size() == 3 && node[0].IsSequence()) {
            for (int i = 0; i < 3; ++i) {
                if (node[i].size() != 3) return false;
                for (int j = 0; j < 3; ++j) {
                    m(i, j) = node[i][j].as<T>();
                }
            }
            return true;
        }

        // Flat format: [r00, r01, r02, r10, r11, r12, r20, r21, r22] (row-major)
        if (node.size() == 9) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    m(i, j) = node[i * 3 + j].as<T>();
                }
            }
            return true;
        }

        return false;
    }
};

}  // namespace YAML
```

---

### [NEW] YamlFile.hpp

**File:** `include/vulcan/io/YamlFile.hpp`

Convenience functions for file operations:

```cpp
#pragma once

#include <vulcan/io/YamlNode.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace vulcan {
namespace io {

/**
 * @brief YAML file utilities
 */
class YamlFile {
public:
    /**
     * @brief Load YAML from file with !include directive resolution
     *
     * Example:
     *   components: !include components.yaml
     *   routes: !include routes/main.yaml
     */
    static YamlNode LoadWithIncludes(const std::string& path);

    /**
     * @brief Merge multiple YAML files (later files override earlier)
     *
     * Useful for config layering:
     *   base.yaml + env.yaml + local.yaml
     */
    static YamlNode MergeFiles(const std::vector<std::string>& paths);

    /**
     * @brief Check if file exists and is valid YAML
     */
    static bool IsValidYaml(const std::string& path);

    /**
     * @brief Get all YAML files in directory
     */
    static std::vector<std::filesystem::path> FindYamlFiles(
        const std::string& directory,
        bool recursive = false);
};

}  // namespace io
}  // namespace vulcan
```

---

### [NEW] YamlEnv.hpp

**File:** `include/vulcan/io/YamlEnv.hpp`

Environment variable expansion for YAML values and paths:

```cpp
#pragma once

#include <vulcan/io/YamlNode.hpp>

#include <optional>
#include <regex>
#include <string>

namespace vulcan {
namespace io {

/**
 * @brief Environment variable expansion utilities
 *
 * Supports two syntaxes:
 *   ${VAR}          - Required variable (throws if undefined)
 *   ${VAR:default}  - Optional with fallback value
 *
 * Example YAML:
 *   config_dir: ${CONFIG_BASE}/configs
 *   database:
 *     host: ${DB_HOST:localhost}
 *     port: ${DB_PORT:5432}
 */
class YamlEnv {
public:
    // =========================================================================
    // String Expansion
    // =========================================================================

    /**
     * @brief Expand environment variables in a string
     *
     * @param value String potentially containing ${VAR} or ${VAR:default}
     * @param strict If true, throws on undefined variables without defaults
     * @return Expanded string
     * @throws YamlError if strict=true and required variable is undefined
     */
    static std::string Expand(const std::string& value, bool strict = true);

    /**
     * @brief Check if string contains environment variable references
     */
    static bool ContainsEnvVars(const std::string& value);

    // =========================================================================
    // Node Expansion (Recursive)
    // =========================================================================

    /**
     * @brief Recursively expand all string values in a YAML tree
     *
     * Walks the entire tree and expands ${VAR} in all string nodes.
     * Non-string nodes are unchanged.
     *
     * @param node Root node to expand
     * @return New YamlNode with expanded values
     */
    static YamlNode ExpandAll(const YamlNode& node);

    // =========================================================================
    // File Loading with Expansion
    // =========================================================================

    /**
     * @brief Load YAML file with environment variable expansion
     *
     * Combines LoadFile + ExpandAll for convenience.
     * Also expands envvars in !include paths before resolution.
     */
    static YamlNode LoadFileWithEnv(const std::string& path);

    /**
     * @brief Expand envvars in path, then load the file
     *
     * Useful for: LoadFromEnvPath("${CONFIG_DIR}/app.yaml")
     */
    static YamlNode LoadFromEnvPath(const std::string& path_with_vars);

    // =========================================================================
    // Environment Access
    // =========================================================================

    /**
     * @brief Get environment variable value
     *
     * @param name Variable name (without $)
     * @return Value if defined, nullopt otherwise
     */
    static std::optional<std::string> GetEnv(const std::string& name);

    /**
     * @brief Get environment variable with default
     */
    static std::string GetEnv(const std::string& name,
                              const std::string& default_value);

private:
    // Regex pattern: ${VAR} or ${VAR:default}
    static const std::regex kEnvVarPattern;
};

/**
 * @brief Exception for undefined environment variables
 */
class EnvVarError : public YamlError {
public:
    EnvVarError(const std::string& var_name)
        : YamlError("env", "undefined environment variable: " + var_name)
        , var_name_(var_name) {}

    const std::string& var_name() const { return var_name_; }

private:
    std::string var_name_;
};

}  // namespace io
}  // namespace vulcan
```

---

### Environment Variable Syntax

| Syntax | Behavior | Example |
|--------|----------|---------|
| `${VAR}` | Required - throws if undefined | `${CONFIG_PATH}` |
| `${VAR:default}` | Optional - uses default if undefined | `${PORT:8080}` |
| `${VAR:}` | Optional - empty string if undefined | `${PREFIX:}` |

**Escaping:** Use `$${VAR}` to produce literal `${VAR}` (no expansion).

---

### Config Chaining via Environment Variables

Environment variables enable flexible config layering:

```yaml
# base.yaml - Core configuration
simulation:
  dt: 0.01
  max_time: 100.0

# Load environment-specific overrides
includes:
  - ${CONFIG_BASE}/environments/${ENV:development}.yaml
  - ${CONFIG_BASE}/local.yaml  # Optional user overrides
```

**Shell usage:**

```bash
# Development (default)
./sim --config base.yaml

# Production
CONFIG_BASE=/etc/myapp ENV=production ./sim --config base.yaml

# Testing with custom paths
CONFIG_BASE=./test/fixtures ENV=test ./sim --config base.yaml
```

---

### [MODIFY] YamlFile.hpp (Update)

Extend `YamlFile` to integrate environment expansion:

```cpp
// Add to YamlFile class:

/**
 * @brief Load with includes AND environment expansion
 *
 * Order of operations:
 * 1. Expand envvars in include paths
 * 2. Resolve !include directives
 * 3. Expand envvars in all string values
 */
static YamlNode LoadWithIncludesAndEnv(const std::string& path);
```

---

### [MODIFY] vulcan.hpp

Add YAML includes to main umbrella header:

```cpp
// Configuration I/O
#include <vulcan/io/YamlNode.hpp>
#include <vulcan/io/YamlConvert.hpp>
#include <vulcan/io/YamlFile.hpp>
#include <vulcan/io/YamlEnv.hpp>
```

---

## Implementation Tasks

### 24.1 Core Infrastructure

- [ ] Add yaml-cpp dependency to `flake.nix`
- [ ] Implement `YamlNode` wrapper class
- [ ] Implement `YamlError` exception with path context
- [ ] Unit tests for basic type extraction (double, int, bool, string)

### 24.2 Janus Type Support

- [ ] Implement `YamlConvert.hpp` traits for Vec3, Mat3, Quaternion
- [ ] Unit tests for Janus type round-trips
- [ ] Handle both nested and flat matrix formats

### 24.3 File Utilities

- [ ] Implement `YamlFile::LoadWithIncludes()`
- [ ] Implement `YamlFile::MergeFiles()`
- [ ] Implement `YamlFile::FindYamlFiles()`
- [ ] Unit tests for file operations

### 24.4 Environment Variable Support

- [ ] Implement `YamlEnv` class with `Expand()` and `ContainsEnvVars()`
- [ ] Implement recursive `ExpandAll()` for YAML trees
- [ ] Implement `LoadFileWithEnv()` combining includes + expansion
- [ ] Handle `$${VAR}` escape sequences
- [ ] Unit tests for envvar expansion (defined, undefined, defaults)

### 24.5 Documentation

- [ ] API documentation in headers
- [ ] Example: Loading configuration
- [ ] Example: Config chaining with environment variables
- [ ] Example: Saving and loading simulation state
- [ ] Add to `docs/user_guides/`

---

## Usage Examples

### Basic Loading

```cpp
#include <vulcan/io/YamlNode.hpp>

auto config = vulcan::io::YamlNode::LoadFile("sim.yaml");

// Required values (throws if missing)
auto name = config.Require<std::string>("name");
auto mass = config["vehicle"].Require<double>("mass");

// Optional with defaults
auto dt = config.Get<double>("dt", 0.01);
auto enabled = config.Get<bool>("debug", false);

// Janus types
auto position = config.Require<janus::Vec3<double>>("initial_position");
auto attitude = config.Require<janus::Quaternion<double>>("initial_attitude");
```

### Iterating Sequences

```cpp
auto components = config["components"];

for (std::size_t i = 0; i < components.Size(); ++i) {
    auto comp = components[i];
    auto type = comp.Require<std::string>("type");
    auto name = comp.Require<std::string>("name");
    // ...
}

// Or with ForEach
components.ForEach([](const YamlNode& comp) {
    auto type = comp.Require<std::string>("type");
    // ...
});
```

### Error Handling

```cpp
try {
    auto config = YamlNode::LoadFile("config.yaml");
    auto mass = config["vehicle"]["properties"].Require<double>("mass");
} catch (const YamlError& e) {
    // "YAML error at 'vehicle.properties.mass': expected number, got string"
    std::cerr << e.what() << std::endl;
}
```

### Writing YAML

```cpp
YAML::Emitter out;
out << YAML::BeginMap;
out << YAML::Key << "position" << YAML::Value << position;  // Uses convert<>
out << YAML::Key << "attitude" << YAML::Value << attitude;
out << YAML::EndMap;

std::ofstream file("state.yaml");
file << out.c_str();
```

---

## File Structure

```
include/vulcan/io/
├── YamlNode.hpp        # Main wrapper class
├── YamlConvert.hpp     # yaml-cpp conversion traits for Janus types
├── YamlFile.hpp        # File utilities (include, merge)
├── YamlEnv.hpp         # Environment variable expansion
└── YamlError.hpp       # Exception types (or inline in YamlNode.hpp)

src/io/
├── YamlNode.cpp        # Implementation
├── YamlFile.cpp        # File utilities implementation
└── YamlEnv.cpp         # Envvar expansion implementation

tests/io/
├── test_yaml_node.cpp      # Unit tests for basic extraction
├── test_yaml_convert.cpp   # Round-trip tests for Janus types
├── test_yaml_file.cpp      # File operation tests
└── test_yaml_env.cpp       # Envvar expansion tests

examples/io/
├── yaml_config_demo.cpp    # Usage demonstration
└── yaml_chaining_demo.cpp  # Config chaining with envvars
```

---

## Dependencies

| Dependency | Version | Notes |
|:-----------|:--------|:------|
| yaml-cpp | ≥ 0.7.0 | Add to flake.nix |

---

## Verification Plan

### Automated Tests

| Test File | Coverage |
|-----------|----------|
| `test_yaml_node.cpp` | Basic types, path tracking, error messages |
| `test_yaml_convert.cpp` | Vec3, Mat3, Quaternion round-trips |
| `test_yaml_file.cpp` | Include resolution, file merge |
| `test_yaml_env.cpp` | Envvar expansion, defaults, escaping |

**Run:** `./scripts/test.sh` (builds and runs all tests)

### Manual Verification

1. Create `examples/io/yaml_config_demo.cpp` that:
   - Loads a config file with Janus types
   - Modifies values
   - Writes back to YAML
   - Reads and verifies

2. Run example: `./build/examples/yaml_config_demo`

---

## Exit Criteria

- [ ] `YamlNode` wrapper with typed accessors
- [ ] Janus type conversions (Vec3, Mat3, Quaternion)
- [ ] Path-aware error messages
- [ ] Include directive support
- [ ] Environment variable expansion (`${VAR}` and `${VAR:default}`)
- [ ] Config chaining via envvar-enabled includes
- [ ] Unit tests for all type conversions and envvar expansion
- [ ] Icarus can load configs using Vulcan YAML utilities
