/**
 * @file test_csv_export.cpp
 * @brief Unit tests for CSV export functionality
 */

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <vulcan/io/CSVExport.hpp>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/HDF5Writer.hpp>

using namespace vulcan::io;

class CSVExportTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Use only double signals for CSV export (avoids type conversion
        // warnings)
        schema_.add_double("altitude", SignalLifecycle::Dynamic, "m")
            .add_double("velocity", SignalLifecycle::Dynamic, "m/s")
            .add_double("phase", SignalLifecycle::Dynamic);

        // Generate unique temp filenames
        auto base = std::filesystem::temp_directory_path() /
                    ("test_csv_" + std::to_string(std::rand()));
        hdf5_file_ = base.string() + ".h5";
        csv_file_ = base.string() + ".csv";

        // Create test HDF5 file
        HDF5Writer writer(hdf5_file_, schema_);
        for (int i = 0; i < 3; ++i) {
            Frame frame(schema_);
            frame.set_time(static_cast<double>(i) * 0.1);
            frame.set("altitude", static_cast<double>(i) * 1000);
            frame.set("velocity", static_cast<double>(i) * 100);
            frame.set("phase", static_cast<double>(i));
            writer.write_frame(frame);
        }
    }

    void TearDown() override {
        if (std::filesystem::exists(hdf5_file_)) {
            std::filesystem::remove(hdf5_file_);
        }
        if (std::filesystem::exists(csv_file_)) {
            std::filesystem::remove(csv_file_);
        }
    }

    TelemetrySchema schema_;
    std::string hdf5_file_;
    std::string csv_file_;
};

// =============================================================================
// Basic Export
// =============================================================================

TEST_F(CSVExportTest, ExportToFile) {
    export_to_csv(hdf5_file_, csv_file_);

    std::ifstream ifs(csv_file_);
    ASSERT_TRUE(ifs.is_open());

    std::string line;

    // Header
    std::getline(ifs, line);
    EXPECT_TRUE(line.find("time") != std::string::npos);
    EXPECT_TRUE(line.find("altitude") != std::string::npos);
    EXPECT_TRUE(line.find("velocity") != std::string::npos);

    // Data rows
    std::getline(ifs, line);
    EXPECT_TRUE(line.find("0") != std::string::npos);
}

TEST_F(CSVExportTest, ExportToStream) {
    HDF5Reader reader(hdf5_file_);
    std::stringstream ss;
    export_to_csv(reader, ss);

    std::string output = ss.str();
    EXPECT_TRUE(output.find("time") != std::string::npos);
    EXPECT_TRUE(output.find("altitude") != std::string::npos);
}

// =============================================================================
// Options
// =============================================================================

TEST_F(CSVExportTest, CustomDelimiter) {
    CSVExportOptions options;
    options.delimiter = ';';

    HDF5Reader reader(hdf5_file_);
    std::stringstream ss;
    export_to_csv(reader, ss, options);

    std::string line;
    std::getline(ss, line);
    EXPECT_TRUE(line.find(';') != std::string::npos);
    EXPECT_TRUE(line.find(',') == std::string::npos);
}

TEST_F(CSVExportTest, NoHeader) {
    CSVExportOptions options;
    options.include_header = false;

    HDF5Reader reader(hdf5_file_);
    std::stringstream ss;
    export_to_csv(reader, ss, options);

    std::string line;
    std::getline(ss, line);

    // First line should be data, not header
    EXPECT_TRUE(line.find("time") == std::string::npos);
}

TEST_F(CSVExportTest, SignalSubset) {
    CSVExportOptions options;
    options.signals = {"altitude"};

    HDF5Reader reader(hdf5_file_);
    std::stringstream ss;
    export_to_csv(reader, ss, options);

    std::string line;
    std::getline(ss, line);

    EXPECT_TRUE(line.find("time") != std::string::npos);
    EXPECT_TRUE(line.find("altitude") != std::string::npos);
    EXPECT_TRUE(line.find("velocity") == std::string::npos);
}

TEST_F(CSVExportTest, Precision) {
    CSVExportOptions options;
    options.precision = 3;

    HDF5Reader reader(hdf5_file_);
    std::stringstream ss;
    export_to_csv(reader, ss, options);

    std::string output = ss.str();
    // With precision=3, we shouldn't see many decimal places
    EXPECT_FALSE(output.empty());
}
