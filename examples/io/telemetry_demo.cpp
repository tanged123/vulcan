/**
 * @file telemetry_demo.cpp
 * @brief Demonstrates the Vulcan Data I/O telemetry system
 *
 * Shows how to:
 * - Define a telemetry schema with various signal types
 * - Record simulation data to HDF5 files
 * - Read data back for post-processing
 * - Export to CSV for external tools
 * - Use the binary serializer for real-time streaming
 */

#include <vulcan/vulcan.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>

using namespace vulcan;
using namespace vulcan::io;

int main() {
    std::cout << "=== Vulcan Telemetry Demo ===\n\n";

    // =========================================================================
    // 1. Define Telemetry Schema
    // =========================================================================
    std::cout << "1. Defining telemetry schema...\n";

    TelemetrySchema schema;
    schema
        // Position and velocity (dynamic - changes every frame)
        .add_vec3("position", SignalLifecycle::Dynamic, "m")
        .add_vec3("velocity", SignalLifecycle::Dynamic, "m/s")
        // Attitude quaternion
        .add_quat("attitude", SignalLifecycle::Dynamic)
        // Flight phase (integer enum)
        .add_int32("phase", SignalLifecycle::Dynamic)
        // Mass (static - set once at the start)
        .add_double("mass", SignalLifecycle::Static, "kg")
        // Thrust magnitude
        .add_double("thrust", SignalLifecycle::Dynamic, "N");

    std::cout << "   Schema has " << schema.signal_count() << " signals\n";
    std::cout << "   Frame size: " << schema.frame_size_bytes() << " bytes\n";
    std::cout << "   Dynamic signals: " << schema.dynamic_signals().size()
              << "\n";
    std::cout << "   Static signals: " << schema.static_signals().size()
              << "\n\n";

    // =========================================================================
    // 2. Simulate and Record to HDF5
    // =========================================================================
    std::cout << "2. Simulating trajectory and recording to HDF5...\n";

    std::string hdf5_file = "/tmp/vulcan_telemetry_demo.h5";
    constexpr double dt = 0.01;    // 100 Hz
    constexpr double t_end = 10.0; // 10 seconds
    constexpr size_t n_frames = static_cast<size_t>(t_end / dt);

    {
        HDF5Writer writer(hdf5_file, schema);

        for (size_t i = 0; i <= n_frames; ++i) {
            double t = i * dt;

            Frame frame(schema);
            frame.set_time(t);

            // Simulated trajectory: vertical ascent with gravity turn
            double alt = 1000.0 * t - 4.9 * t * t; // Ballistic with initial
                                                   // velocity
            frame.set("position", Eigen::Vector3d(0, 0, std::max(0.0, alt)));
            frame.set("velocity", Eigen::Vector3d(0, 0, 1000.0 - 9.8 * t));

            // Simple attitude (pointing up)
            frame.set("attitude", Eigen::Quaterniond::Identity());

            // Flight phases: 0=prelaunch, 1=powered, 2=coast, 3=descent
            int32_t phase = (t < 0.1) ? 0 : (t < 5.0) ? 1 : (t < 8.0) ? 2 : 3;
            frame.set("phase", phase);

            // Static mass (only needs to be set once, but setting each frame
            // is fine)
            frame.set("mass", 1000.0);

            // Thrust during powered phase
            double thrust = (phase == 1) ? 15000.0 : 0.0;
            frame.set("thrust", thrust);

            writer.write_frame(frame);
        }

        std::cout << "   Wrote " << writer.frame_count() << " frames to "
                  << hdf5_file << "\n\n";
    }

    // =========================================================================
    // 3. Read HDF5 for Post-Processing
    // =========================================================================
    std::cout << "3. Reading HDF5 for analysis...\n";

    {
        HDF5Reader reader(hdf5_file);

        std::cout << "   Total frames: " << reader.frame_count() << "\n";

        // Read specific signals
        auto times = reader.times();
        auto positions = reader.read_vec3("position");
        auto phases = reader.read_int32("phase");

        // Find apogee
        double max_alt = 0;
        size_t apogee_idx = 0;
        for (size_t i = 0; i < positions.size(); ++i) {
            if (positions[i].z() > max_alt) {
                max_alt = positions[i].z();
                apogee_idx = i;
            }
        }

        std::cout << "   Apogee: " << max_alt << " m at t=" << times[apogee_idx]
                  << " s\n";
        std::cout << "   Phase at apogee: " << phases[apogee_idx] << "\n\n";

        // Read a slice (last 100 frames)
        size_t start = reader.frame_count() - 100;
        auto thrust_slice = reader.read_double("thrust", start, 100);
        std::cout << "   Last 100 thrust samples: first="
                  << thrust_slice.front() << " N, last=" << thrust_slice.back()
                  << " N\n\n";
    }

    // =========================================================================
    // 4. Export to CSV
    // =========================================================================
    std::cout << "4. Exporting to CSV...\n";

    std::string csv_file = "/tmp/vulcan_telemetry_demo.csv";
    {
        CSVExportOptions options;
        options.precision = 6;
        // Export only position signals for simplicity
        options.signals = {"position.x", "position.y", "position.z"};

        export_to_csv(hdf5_file, csv_file, options);

        std::cout << "   Exported position data to " << csv_file << "\n\n";
    }

    // =========================================================================
    // 5. Binary Serialization for Real-Time Streaming
    // =========================================================================
    std::cout << "5. Demonstrating binary serialization (for Hermes)...\n";

    {
        FrameSerializer serializer(schema);

        std::cout << "   Dynamic frame size: "
                  << serializer.dynamic_size_bytes() << " bytes\n";
        std::cout << "   Static frame size: " << serializer.static_size_bytes()
                  << " bytes\n";

        // Create a sample frame
        Frame frame(schema);
        frame.set_time(1.234);
        frame.set("position", Eigen::Vector3d(100, 200, 300));
        frame.set("velocity", Eigen::Vector3d(10, 20, 30));
        frame.set("attitude", Eigen::Quaterniond::Identity());
        frame.set("phase", int32_t{1});
        frame.set("mass", 1000.0);
        frame.set("thrust", 15000.0);

        // Serialize to binary (this would go to Hermes lock-free buffer)
        auto dynamic_bytes = serializer.serialize(frame);
        auto static_bytes = serializer.serialize_statics(frame);

        std::cout << "   Serialized dynamic data: " << dynamic_bytes.size()
                  << " bytes\n";
        std::cout << "   Serialized static data: " << static_bytes.size()
                  << " bytes\n";

        // Deserialize (simulating Hermes receiving the data)
        Frame received(schema);
        serializer.deserialize_statics(static_bytes, received);
        serializer.deserialize(dynamic_bytes, received);

        auto pos = received.get_vec3("position");
        std::cout << "   Deserialized position: (" << pos.x() << ", " << pos.y()
                  << ", " << pos.z() << ")\n\n";
    }

    // =========================================================================
    // 6. JSON Schema for Protocol Negotiation
    // =========================================================================
    std::cout << "6. JSON schema (for Hermes handshake)...\n";

    std::string json = schema.to_json();
    // Print first 500 chars
    std::cout << "   " << json.substr(0, 500) << "...\n\n";

    // Cleanup
    std::filesystem::remove(hdf5_file);
    std::filesystem::remove(csv_file);

    std::cout << "=== Demo Complete ===\n";
    return 0;
}
