# YAML Configuration Guide

Vulcan provides a type-safe YAML configuration system with environment variable expansion and config chaining.

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>
using namespace vulcan::io;

auto config = YamlNode::LoadFile("mission.yaml");
auto name = config.Require<std::string>("name");
auto dt = config.Get<double>("dt", 0.01);  // with default
```

---

## Headers

| Header | Purpose |
|--------|---------|
| `YamlNode.hpp` | Type-safe node wrapper |
| `YamlConvert.hpp` | Janus type traits |
| `YamlFile.hpp` | Include/merge utilities |
| `YamlEnv.hpp` | Environment variables |

---

## YamlNode API

### Loading

```cpp
auto node = YamlNode::LoadFile("config.yaml");
auto node = YamlNode::Parse("key: value");
```

### Value Extraction

```cpp
// Required (throws if missing)
auto value = node.Require<std::string>("key");

// Optional with default
auto value = node.Get<int>("key", 42);

// Optional returning std::optional
auto value = node.GetOptional<double>("key");
```

### Navigation

```cpp
auto child = node["nested"]["path"];
bool exists = node.Has("key");
std::size_t len = node.Size();
```

### Iteration

```cpp
// Sequence iteration
node["items"].ForEach([](const YamlNode& item) {
    std::cout << item.As<std::string>() << "\n";
});

// Map iteration
node.ForEachEntry([](const std::string& key, const YamlNode& value) {
    std::cout << key << ": " << value.As<int>() << "\n";
});

// Convert to vector
auto vec = node["values"].ToVector<double>();
```

---

## Janus Types

```yaml
# Vec3: [x, y, z]
position: [1000.0, 2000.0, 3000.0]

# Quaternion: [w, x, y, z] (scalar-first)
orientation: [1.0, 0.0, 0.0, 0.0]

# Mat3: nested rows or flat array
inertia:
  - [100, 0, 0]
  - [0, 200, 0]
  - [0, 0, 150]
```

```cpp
auto pos = config.Require<janus::Vec3<double>>("position");
auto quat = config.Require<janus::Quaternion<double>>("orientation");
auto mat = config.Require<janus::Mat3<double>>("inertia");
```

---

## Environment Variables

### Syntax

| Pattern | Behavior |
|---------|----------|
| `${VAR}` | Required (throws if undefined) |
| `${VAR:default}` | Optional with fallback |
| `$${VAR}` | Escape (produces literal `${VAR}`) |

### Usage

```cpp
// Expand a string
std::string path = YamlEnv::Expand("${HOME}/data");

// Load file with expansion
auto config = YamlEnv::LoadFileWithEnv("config.yaml");
```

### Example Config

```yaml
# config.yaml
output_dir: ${OUTPUT_DIR:/tmp/vulcan}
log_level: ${LOG_LEVEL:info}
api_key: ${API_KEY}  # Required
```

---

## Config Chaining

### Include Directive

```yaml
# main.yaml
database: !include database.yaml
settings: !include config/settings.yaml
```

```cpp
auto config = YamlFile::LoadWithIncludes("main.yaml");
```

### File Merging

Layer multiple configs (later files override earlier):

```cpp
auto config = YamlFile::MergeFiles({
    "defaults.yaml",
    "environment.yaml",
    "local.yaml"
});
```

### Combined: Includes + Envvars

```cpp
auto config = YamlEnv::LoadWithIncludesAndEnv("${CONFIG_DIR}/main.yaml");
```

---

## Error Handling

```cpp
try {
    auto value = node.Require<int>("missing_key");
} catch (const YamlError& e) {
    // e.what() includes path: "root.missing_key: key not found"
}

try {
    YamlEnv::Expand("${UNDEFINED_VAR}");
} catch (const EnvVarError& e) {
    // e.var_name() returns "UNDEFINED_VAR"
}
```

---

## Example: Mission Config

```yaml
# mission.yaml
mission:
  name: ${MISSION_NAME:Demo Mission}
  
spacecraft: !include spacecraft.yaml

simulation:
  dt: 0.01
  max_time: 3600.0
  output_dir: ${OUTPUT_DIR:/tmp/sim}
```

```cpp
auto config = YamlEnv::LoadWithIncludesAndEnv("mission.yaml");

auto name = config["mission"].Require<std::string>("name");
auto mass = config["spacecraft"].Require<double>("mass_kg");
auto pos = config["spacecraft"].Require<janus::Vec3<double>>("position");
```
