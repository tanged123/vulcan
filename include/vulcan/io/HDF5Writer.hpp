/**
 * @file HDF5Writer.hpp
 * @brief HDF5 telemetry writer for post-processing
 *
 * Writes Frame data to HDF5 files with one dataset per signal.
 * Uses chunked, extensible datasets for efficient frame-by-frame appends.
 */

#pragma once

#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>
#include <memory>
#include <string>
#include <vector>

#include <vulcan/io/Frame.hpp>
#include <vulcan/io/TelemetrySchema.hpp>

namespace vulcan::io {

/**
 * @brief HDF5 telemetry file writer
 *
 * Writes telemetry frames to HDF5 files. Each signal becomes a 1D dataset
 * that can be efficiently extended. Supports frame-by-frame logging or
 * batch writes.
 *
 * HDF5 Structure:
 * @code
 * /time                    # (N,) float64 - timestamps
 * /signals/
 *     position.x           # (N,) float64
 *     position.y           # (N,) float64
 *     position.z           # (N,) float64
 *     gnc.phase            # (N,) int32
 * /metadata/
 *     schema               # JSON string attribute
 *     created_at           # ISO 8601 timestamp
 * @endcode
 *
 * Example:
 * @code
 * HDF5Writer writer("telemetry.h5", schema);
 *
 * for (double t = 0; t < 10.0; t += 0.001) {
 *     Frame frame(schema);
 *     frame.set_time(t);
 *     frame.set("position", compute_position(t));
 *     writer.write_frame(frame);
 * }
 *
 * writer.close();  // Optional, called automatically on destruction
 * @endcode
 */
class HDF5Writer {
  public:
    /**
     * @brief Create HDF5 writer
     * @param filename Output HDF5 file path
     * @param schema Telemetry schema
     */
    HDF5Writer(const std::string &filename, const TelemetrySchema &schema)
        : impl_(std::make_unique<Impl>(filename, schema)) {}

    ~HDF5Writer() = default;

    // Non-copyable, movable
    HDF5Writer(const HDF5Writer &) = delete;
    HDF5Writer &operator=(const HDF5Writer &) = delete;
    HDF5Writer(HDF5Writer &&) noexcept = default;
    HDF5Writer &operator=(HDF5Writer &&) noexcept = default;

    /**
     * @brief Write a single frame
     * @param frame Frame to write
     */
    void write_frame(const Frame &frame) { impl_->write_frame(frame); }

    /**
     * @brief Write multiple frames
     * @param frames Vector of frames to write
     */
    void write_frames(const std::vector<Frame> &frames) {
        for (const auto &frame : frames) {
            impl_->write_frame(frame);
        }
    }

    /// Get number of frames written so far
    size_t frame_count() const { return impl_->frame_count(); }

    /// Flush buffered data to disk
    void flush() { impl_->flush(); }

    /// Close file (called automatically on destruction)
    void close() { impl_->close(); }

  private:
    struct Impl {
        HighFive::File file;
        TelemetrySchema schema;
        size_t frames_written = 0;
        static constexpr size_t CHUNK_SIZE = 1000;

        // Datasets
        std::vector<HighFive::DataSet> signal_datasets;
        HighFive::DataSet time_dataset;

        Impl(const std::string &filename, const TelemetrySchema &sch)
            : file(filename, HighFive::File::Truncate), schema(sch),
              time_dataset(setup_file()) {}

        HighFive::DataSet setup_file() {
            // Create groups
            file.createGroup("signals");
            file.createGroup("metadata");

            // Create time dataset (chunked, unlimited)
            HighFive::DataSpace time_space({0},
                                           {HighFive::DataSpace::UNLIMITED});
            HighFive::DataSetCreateProps time_props;
            time_props.add(HighFive::Chunking({CHUNK_SIZE}));
            auto time_ds =
                file.createDataSet<double>("time", time_space, time_props);

            // Create signal datasets
            for (const auto &sig : schema.signals()) {
                HighFive::DataSpace space({0},
                                          {HighFive::DataSpace::UNLIMITED});
                HighFive::DataSetCreateProps props;
                props.add(HighFive::Chunking({CHUNK_SIZE}));

                std::string path = "signals/" + sig.name;

                switch (sig.type) {
                case SignalType::Double: {
                    auto ds = file.createDataSet<double>(path, space, props);
                    if (!sig.unit.empty()) {
                        ds.createAttribute<std::string>("unit", sig.unit);
                    }
                    if (!sig.semantic.empty()) {
                        ds.createAttribute<std::string>("semantic",
                                                        sig.semantic);
                    }
                    signal_datasets.push_back(std::move(ds));
                    break;
                }
                case SignalType::Int32: {
                    auto ds = file.createDataSet<int32_t>(path, space, props);
                    if (!sig.unit.empty()) {
                        ds.createAttribute<std::string>("unit", sig.unit);
                    }
                    if (!sig.semantic.empty()) {
                        ds.createAttribute<std::string>("semantic",
                                                        sig.semantic);
                    }
                    signal_datasets.push_back(std::move(ds));
                    break;
                }
                case SignalType::Int64: {
                    auto ds = file.createDataSet<int64_t>(path, space, props);
                    if (!sig.unit.empty()) {
                        ds.createAttribute<std::string>("unit", sig.unit);
                    }
                    if (!sig.semantic.empty()) {
                        ds.createAttribute<std::string>("semantic",
                                                        sig.semantic);
                    }
                    signal_datasets.push_back(std::move(ds));
                    break;
                }
                }
            }

            // Store schema JSON as metadata
            auto metadata = file.getGroup("metadata");
            metadata.createAttribute<std::string>("schema", schema.to_json());

            // Store creation timestamp
            auto now = std::time(nullptr);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ",
                          std::gmtime(&now));
            metadata.createAttribute<std::string>("created_at",
                                                  std::string(buf));

            return time_ds;
        }

        void write_frame(const Frame &frame) {
            // Extend and write time
            time_dataset.resize({frames_written + 1});
            time_dataset.select({frames_written}, {1}).write(frame.time());

            // Write each signal
            for (size_t i = 0; i < schema.signals().size(); ++i) {
                const auto &sig = schema.signals()[i];
                auto &ds = signal_datasets[i];

                ds.resize({frames_written + 1});

                switch (sig.type) {
                case SignalType::Double: {
                    double val = frame.get_double(sig.name);
                    ds.select({frames_written}, {1}).write(val);
                    break;
                }
                case SignalType::Int32: {
                    int32_t val = frame.get_int32(sig.name);
                    ds.select({frames_written}, {1}).write(val);
                    break;
                }
                case SignalType::Int64: {
                    int64_t val = frame.get_int64(sig.name);
                    ds.select({frames_written}, {1}).write(val);
                    break;
                }
                }
            }

            ++frames_written;
        }

        size_t frame_count() const { return frames_written; }

        void flush() { file.flush(); }

        void close() {
            flush();
            // File will be closed when Impl is destroyed
        }
    };

    std::unique_ptr<Impl> impl_;
};

} // namespace vulcan::io
