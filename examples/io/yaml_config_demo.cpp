/**
 * @file yaml_config_demo.cpp
 * @brief Demonstrates YAML configuration loading with environment variables
 *
 * This example shows:
 * - Loading YAML config files
 * - Extracting typed values (scalars, vectors)
 * - Using Janus types (Vec3, Quaternion, Mat3)
 * - Environment variable expansion
 * - Config file chaining with !include
 */

#include <vulcan/vulcan.hpp>

#include <iostream>

using namespace vulcan::io;

int main() {
    std::cout << "=== Vulcan YAML Configuration Demo ===\n\n";

    // =========================================================================
    // 1. Basic YAML Parsing
    // =========================================================================

    std::cout << "1. Basic Parsing\n";
    std::cout << std::string(40, '-') << "\n";

    auto config = YamlNode::Parse(R"(
        name: Sample Mission
        version: 1.0
        enabled: true
        stages: [first, second, third]
        
        spacecraft:
            mass_kg: 1500.0
            position: [1000.0, 2000.0, 3000.0]
            velocity: [7.5, 0.0, 0.0]
            
        simulation:
            dt: 0.01
            max_time: 3600.0
            output_rate: 10
    )");

    std::cout << "Name:    " << config.Require<std::string>("name") << "\n";
    std::cout << "Version: " << config.Require<double>("version") << "\n";
    std::cout << "Enabled: " << std::boolalpha
              << config.Require<bool>("enabled") << "\n\n";

    // =========================================================================
    // 2. Janus Type Extraction
    // =========================================================================

    std::cout << "2. Janus Type Extraction\n";
    std::cout << std::string(40, '-') << "\n";

    auto spacecraft = config["spacecraft"];
    double mass = spacecraft.Require<double>("mass_kg");
    auto position = spacecraft.Require<janus::Vec3<double>>("position");
    auto velocity = spacecraft.Require<janus::Vec3<double>>("velocity");

    std::cout << "Mass:     " << mass << " kg\n";
    std::cout << "Position: [" << position.x() << ", " << position.y() << ", "
              << position.z() << "] m\n";
    std::cout << "Velocity: [" << velocity.x() << ", " << velocity.y() << ", "
              << velocity.z() << "] km/s\n\n";

    // =========================================================================
    // 3. Optional Values with Defaults
    // =========================================================================

    std::cout << "3. Optional Values\n";
    std::cout << std::string(40, '-') << "\n";

    auto sim = config["simulation"];
    double dt = sim.Get<double>("dt", 0.001);
    int precision = sim.Get<int>("precision", 15); // Uses default

    std::cout << "dt:        " << dt << " (from config)\n";
    std::cout << "precision: " << precision << " (default)\n\n";

    // =========================================================================
    // 4. Iteration
    // =========================================================================

    std::cout << "4. Iteration\n";
    std::cout << std::string(40, '-') << "\n";

    std::cout << "Stages: ";
    config["stages"].ForEach([](const YamlNode &stage) {
        std::cout << stage.As<std::string>() << " ";
    });
    std::cout << "\n\n";

    // =========================================================================
    // 5. Environment Variable Expansion
    // =========================================================================

    std::cout << "5. Environment Variables\n";
    std::cout << std::string(40, '-') << "\n";

    // Expand a string with environment variables
    std::string home_path = YamlEnv::Expand("${HOME}/data");
    std::cout << "Expanded path: " << home_path << "\n";

    // With default value
    std::string log_level = YamlEnv::Expand("${LOG_LEVEL:info}");
    std::cout << "Log level: " << log_level << "\n\n";

    // =========================================================================
    // 6. Quaternion and Matrix Support
    // =========================================================================

    std::cout << "6. Rotation Types\n";
    std::cout << std::string(40, '-') << "\n";

    auto rotation_config = YamlNode::Parse(R"(
        orientation: [1.0, 0.0, 0.0, 0.0]
        inertia:
            - [100, 0, 0]
            - [0, 200, 0]
            - [0, 0, 150]
    )");

    auto quat =
        rotation_config.Require<janus::Quaternion<double>>("orientation");
    auto inertia = rotation_config.Require<janus::Mat3<double>>("inertia");

    std::cout << "Quaternion: [w=" << quat.w << ", x=" << quat.x
              << ", y=" << quat.y << ", z=" << quat.z << "]\n";
    std::cout << "Inertia diagonal: [" << inertia(0, 0) << ", " << inertia(1, 1)
              << ", " << inertia(2, 2) << "] kg·m²\n\n";

    std::cout << "=== Demo Complete ===\n";
    return 0;
}
