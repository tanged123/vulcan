/**
 * @file HDF5Reader.hpp
 * @brief HDF5 telemetry reader for post-simulation analysis
 *
 * Reads Frame data from HDF5 files created by HDF5Writer.
 * Supports reading individual signals, slices, and full trajectories.
 */

#pragma once

#include <Eigen/Core>
#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>
#include <memory>
#include <string>
#include <vector>

#include <vulcan/io/TelemetrySchema.hpp>

namespace vulcan::io {

/**
 * @brief HDF5 telemetry file reader
 *
 * Reads telemetry data from HDF5 files. Provides access to individual
 * signals, time vectors, and convenience methods for vector signals.
 *
 * Example:
 * @code
 * HDF5Reader reader("telemetry.h5");
 *
 * auto times = reader.times();
 * auto positions_x = reader.read_double("position.x");
 *
 * // Convenience for vector signals
 * auto positions = reader.read_vec3("position");
 *
 * std::cout << "Read " << reader.frame_count() << " frames\n";
 * @endcode
 */
class HDF5Reader {
  public:
    /**
     * @brief Open HDF5 file for reading
     * @param filename Path to HDF5 file
     */
    explicit HDF5Reader(const std::string &filename)
        : impl_(std::make_unique<Impl>(filename)) {}

    ~HDF5Reader() = default;

    // Non-copyable, movable
    HDF5Reader(const HDF5Reader &) = delete;
    HDF5Reader &operator=(const HDF5Reader &) = delete;
    HDF5Reader(HDF5Reader &&) noexcept = default;
    HDF5Reader &operator=(HDF5Reader &&) noexcept = default;

    /// Get schema from file metadata
    TelemetrySchema schema() const { return impl_->schema(); }

    /// Get number of frames in file
    size_t frame_count() const { return impl_->frame_count(); }

    /// Read all timestamps
    std::vector<double> times() const { return impl_->times(); }

    /// Read double signal
    std::vector<double> read_double(const std::string &signal) const {
        return impl_->read_double(signal);
    }

    /// Read int32 signal
    std::vector<int32_t> read_int32(const std::string &signal) const {
        return impl_->read_int32(signal);
    }

    /// Read int64 signal
    std::vector<int64_t> read_int64(const std::string &signal) const {
        return impl_->read_int64(signal);
    }

    /// Read double signal slice
    std::vector<double> read_double(const std::string &signal, size_t start,
                                    size_t count) const {
        return impl_->read_double_slice(signal, start, count);
    }

    /// Read 3-component vector signal (reads .x, .y, .z)
    std::vector<Eigen::Vector3d> read_vec3(const std::string &signal) const {
        auto x = read_double(signal + ".x");
        auto y = read_double(signal + ".y");
        auto z = read_double(signal + ".z");

        std::vector<Eigen::Vector3d> result;
        result.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            result.emplace_back(x[i], y[i], z[i]);
        }
        return result;
    }

    /// Read quaternion signal (reads .w, .x, .y, .z)
    std::vector<Eigen::Quaterniond> read_quat(const std::string &signal) const {
        auto w = read_double(signal + ".w");
        auto x = read_double(signal + ".x");
        auto y = read_double(signal + ".y");
        auto z = read_double(signal + ".z");

        std::vector<Eigen::Quaterniond> result;
        result.reserve(w.size());
        for (size_t i = 0; i < w.size(); ++i) {
            result.emplace_back(w[i], x[i], y[i], z[i]);
        }
        return result;
    }

    /// Get all signal names
    std::vector<std::string> signal_names() const {
        return impl_->signal_names();
    }

  private:
    struct Impl {
        HighFive::File file;
        mutable std::optional<TelemetrySchema> cached_schema;

        explicit Impl(const std::string &filename)
            : file(filename, HighFive::File::ReadOnly) {}

        TelemetrySchema schema() const {
            if (!cached_schema) {
                auto metadata = file.getGroup("metadata");
                std::string json;
                metadata.getAttribute("schema").read(json);
                cached_schema = TelemetrySchema::from_json(json);
            }
            return *cached_schema;
        }

        size_t frame_count() const {
            return file.getDataSet("time").getDimensions()[0];
        }

        std::vector<double> times() const {
            std::vector<double> result;
            file.getDataSet("time").read(result);
            return result;
        }

        std::vector<double> read_double(const std::string &signal) const {
            std::vector<double> result;
            file.getDataSet("signals/" + signal).read(result);
            return result;
        }

        std::vector<int32_t> read_int32(const std::string &signal) const {
            std::vector<int32_t> result;
            file.getDataSet("signals/" + signal).read(result);
            return result;
        }

        std::vector<int64_t> read_int64(const std::string &signal) const {
            std::vector<int64_t> result;
            file.getDataSet("signals/" + signal).read(result);
            return result;
        }

        std::vector<double> read_double_slice(const std::string &signal,
                                              size_t start,
                                              size_t count) const {
            auto ds = file.getDataSet("signals/" + signal);
            std::vector<double> result(count);
            ds.select({start}, {count}).read(result);
            return result;
        }

        std::vector<std::string> signal_names() const {
            auto signals = file.getGroup("signals");
            return signals.listObjectNames();
        }
    };

    std::unique_ptr<Impl> impl_;
};

} // namespace vulcan::io
