/**
 * @file CSVExport.hpp
 * @brief HDF5 to CSV export utility
 *
 * Exports HDF5 telemetry files to CSV format for use with
 * external tools like MATLAB, Excel, or Python pandas.
 */

#pragma once

#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

#include <vulcan/core/VulcanError.hpp>
#include <vulcan/io/HDF5Reader.hpp>

namespace vulcan::io {

/// CSV export options
struct CSVExportOptions {
    char delimiter = ',';             ///< Column delimiter
    int precision = 15;               ///< Decimal precision for doubles
    bool include_header = true;       ///< Include column names in first row
    std::vector<std::string> signals; ///< Signal subset (empty = all)
};

/**
 * @brief Export HDF5 reader to output stream
 * @param reader HDF5 reader
 * @param out Output stream
 * @param options Export options
 */
inline void export_to_csv(const HDF5Reader &reader, std::ostream &out,
                          const CSVExportOptions &options = {}) {
    // Determine signals to export
    std::vector<std::string> signals;
    if (options.signals.empty()) {
        signals = reader.signal_names();
    } else {
        signals = options.signals;
    }

    // Get frame count
    size_t n_frames = reader.frame_count();
    if (n_frames == 0) {
        return;
    }

    // Read all data upfront
    std::vector<double> times = reader.times();
    std::vector<std::vector<double>> signal_data;
    signal_data.reserve(signals.size());

    for (const auto &sig : signals) {
        signal_data.push_back(reader.read_double(sig));
    }

    // Set output precision
    out << std::setprecision(options.precision);

    // Write header
    if (options.include_header) {
        out << "time";
        for (const auto &sig : signals) {
            out << options.delimiter << sig;
        }
        out << "\n";
    }

    // Write data rows
    for (size_t i = 0; i < n_frames; ++i) {
        out << times[i];
        for (size_t j = 0; j < signals.size(); ++j) {
            out << options.delimiter << signal_data[j][i];
        }
        out << "\n";
    }
}

/**
 * @brief Export HDF5 file to CSV
 * @param hdf5_path Input HDF5 file path
 * @param csv_path Output CSV file path
 * @param options Export options
 */
inline void export_to_csv(const std::string &hdf5_path,
                          const std::string &csv_path,
                          const CSVExportOptions &options = {}) {
    HDF5Reader reader(hdf5_path);
    std::ofstream ofs(csv_path);
    if (!ofs) {
        throw vulcan::IOError("Failed to open CSV file: " + csv_path);
    }
    export_to_csv(reader, ofs, options);
}

} // namespace vulcan::io
