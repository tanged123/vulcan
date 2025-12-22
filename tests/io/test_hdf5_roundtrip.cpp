/**
 * @file test_hdf5_roundtrip.cpp
 * @brief HDF5 write/read round-trip tests
 */

#include <cstdio>
#include <filesystem>
#include <gtest/gtest.h>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/HDF5Reader.hpp>
#include <vulcan/io/HDF5Writer.hpp>

using namespace vulcan::io;

class HDF5RoundtripTest : public ::testing::Test {
  protected:
    void SetUp() override {
        schema_.add_vec3("position", SignalLifecycle::Dynamic, "m")
            .add_vec3("velocity", SignalLifecycle::Dynamic, "m/s")
            .add_double("altitude", SignalLifecycle::Dynamic, "m")
            .add_int32("phase", SignalLifecycle::Dynamic, "enum");

        // Generate unique temp filename
        test_file_ = std::filesystem::temp_directory_path() /
                     ("test_hdf5_" + std::to_string(std::rand()) + ".h5");
    }

    void TearDown() override {
        if (std::filesystem::exists(test_file_)) {
            std::filesystem::remove(test_file_);
        }
    }

    TelemetrySchema schema_;
    std::filesystem::path test_file_;
};

// =============================================================================
// Basic Round-Trip
// =============================================================================

TEST_F(HDF5RoundtripTest, WriteReadSingleFrame) {
    // Write
    {
        HDF5Writer writer(test_file_.string(), schema_);

        Frame frame(schema_);
        frame.set_time(0.001);
        frame.set("position", Eigen::Vector3d(1, 2, 3));
        frame.set("velocity", Eigen::Vector3d(10, 20, 30));
        frame.set("altitude", 10000.0);
        frame.set("phase", int32_t{2});

        writer.write_frame(frame);
        EXPECT_EQ(writer.frame_count(), 1);
    }

    // Read
    {
        HDF5Reader reader(test_file_.string());

        EXPECT_EQ(reader.frame_count(), 1);

        auto times = reader.times();
        EXPECT_EQ(times.size(), 1);
        EXPECT_DOUBLE_EQ(times[0], 0.001);

        auto pos = reader.read_vec3("position");
        EXPECT_EQ(pos.size(), 1);
        EXPECT_DOUBLE_EQ(pos[0].x(), 1.0);
        EXPECT_DOUBLE_EQ(pos[0].y(), 2.0);
        EXPECT_DOUBLE_EQ(pos[0].z(), 3.0);

        auto alt = reader.read_double("altitude");
        EXPECT_DOUBLE_EQ(alt[0], 10000.0);

        auto phase = reader.read_int32("phase");
        EXPECT_EQ(phase[0], 2);
    }
}

TEST_F(HDF5RoundtripTest, WriteReadMultipleFrames) {
    constexpr size_t N = 100;

    // Write
    {
        HDF5Writer writer(test_file_.string(), schema_);

        for (size_t i = 0; i < N; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i) * 0.001);
            frame.set("position", Eigen::Vector3d(static_cast<double>(i),
                                                  static_cast<double>(i) * 2,
                                                  static_cast<double>(i) * 3));
            frame.set("altitude", static_cast<double>(i) * 100);
            frame.set("phase", int32_t{static_cast<int32_t>(i % 4)});
            writer.write_frame(frame);
        }

        EXPECT_EQ(writer.frame_count(), N);
    }

    // Read
    {
        HDF5Reader reader(test_file_.string());

        EXPECT_EQ(reader.frame_count(), N);

        auto times = reader.times();
        EXPECT_EQ(times.size(), N);

        for (size_t i = 0; i < N; ++i) {
            EXPECT_NEAR(times[i], static_cast<double>(i) * 0.001, 1e-15);
        }

        auto positions = reader.read_vec3("position");
        EXPECT_EQ(positions.size(), N);
        EXPECT_DOUBLE_EQ(positions[50].x(), 50.0);
        EXPECT_DOUBLE_EQ(positions[50].y(), 100.0);
        EXPECT_DOUBLE_EQ(positions[50].z(), 150.0);
    }
}

// =============================================================================
// Signal Names
// =============================================================================

TEST_F(HDF5RoundtripTest, SignalNames) {
    {
        HDF5Writer writer(test_file_.string(), schema_);
        Frame frame(schema_);
        frame.set_time(0.0);
        writer.write_frame(frame);
    }

    {
        HDF5Reader reader(test_file_.string());
        auto names = reader.signal_names();

        EXPECT_TRUE(std::find(names.begin(), names.end(), "position.x") !=
                    names.end());
        EXPECT_TRUE(std::find(names.begin(), names.end(), "altitude") !=
                    names.end());
        EXPECT_TRUE(std::find(names.begin(), names.end(), "phase") !=
                    names.end());
    }
}

// =============================================================================
// Large Dataset
// =============================================================================

TEST_F(HDF5RoundtripTest, LargeDataset) {
    constexpr size_t N = 10000;

    // Write
    {
        HDF5Writer writer(test_file_.string(), schema_);

        for (size_t i = 0; i < N; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i) * 0.001);
            frame.set("altitude", static_cast<double>(i));
            writer.write_frame(frame);
        }
    }

    // Read (slice)
    {
        HDF5Reader reader(test_file_.string());

        EXPECT_EQ(reader.frame_count(), N);

        // Read middle slice
        auto slice = reader.read_double("altitude", 5000, 100);
        EXPECT_EQ(slice.size(), 100);
        EXPECT_DOUBLE_EQ(slice[0], 5000.0);
        EXPECT_DOUBLE_EQ(slice[99], 5099.0);
    }
}

// =============================================================================
// Int64 Signals
// =============================================================================

TEST(HDF5Int64Test, WriteReadInt64) {
    TelemetrySchema schema;
    schema.add_int64("timestamp", SignalLifecycle::Dynamic)
        .add_int64("counter", SignalLifecycle::Dynamic)
        .add_double("value");

    auto test_file = std::filesystem::temp_directory_path() /
                     ("test_hdf5_int64_" + std::to_string(std::rand()) + ".h5");

    // Write
    {
        HDF5Writer writer(test_file.string(), schema);

        Frame frame(schema);
        frame.set_time(1.0);
        frame.set("timestamp", int64_t{9223372036854775807LL}); // max
        frame.set("counter", int64_t{-1234567890123456789LL});
        frame.set("value", 3.14);

        writer.write_frame(frame);
    }

    // Read
    {
        HDF5Reader reader(test_file.string());

        auto timestamps = reader.read_int64("timestamp");
        auto counters = reader.read_int64("counter");
        auto values = reader.read_double("value");

        EXPECT_EQ(timestamps.size(), 1);
        EXPECT_EQ(timestamps[0], 9223372036854775807LL);
        EXPECT_EQ(counters[0], -1234567890123456789LL);
        EXPECT_DOUBLE_EQ(values[0], 3.14);
    }

    std::filesystem::remove(test_file);
}

// =============================================================================
// Writer Methods
// =============================================================================

TEST_F(HDF5RoundtripTest, FlushDuringWrite) {
    {
        HDF5Writer writer(test_file_.string(), schema_);

        for (int i = 0; i < 10; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i));
            writer.write_frame(frame);
        }

        // Flush should not throw
        EXPECT_NO_THROW(writer.flush());

        // More frames after flush
        for (int i = 10; i < 20; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i));
            writer.write_frame(frame);
        }

        EXPECT_EQ(writer.frame_count(), 20);
    }

    // Verify all 20 frames written
    HDF5Reader reader(test_file_.string());
    EXPECT_EQ(reader.frame_count(), 20);
}

TEST_F(HDF5RoundtripTest, WriteFramesBatch) {
    {
        HDF5Writer writer(test_file_.string(), schema_);

        std::vector<Frame> frames;
        for (int i = 0; i < 5; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i));
            frame.set("altitude", static_cast<double>(i) * 100);
            frames.push_back(frame);
        }

        writer.write_frames(frames);
        EXPECT_EQ(writer.frame_count(), 5);
    }

    HDF5Reader reader(test_file_.string());
    auto altitudes = reader.read_double("altitude");
    EXPECT_EQ(altitudes.size(), 5);
    EXPECT_DOUBLE_EQ(altitudes[3], 300.0);
}

TEST_F(HDF5RoundtripTest, ExplicitClose) {
    {
        HDF5Writer writer(test_file_.string(), schema_);

        Frame frame(schema_);
        frame.set_time(1.0);
        writer.write_frame(frame);

        // Explicit close
        EXPECT_NO_THROW(writer.close());
    }

    // File should be readable
    HDF5Reader reader(test_file_.string());
    EXPECT_EQ(reader.frame_count(), 1);
}
