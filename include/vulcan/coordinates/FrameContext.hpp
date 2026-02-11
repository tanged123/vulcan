// Vulcan Frame Context
// User-facing API for frame-tree setup and transforms
#pragma once

#include <vulcan/coordinates/FrameRegistry.hpp>
#include <vulcan/coordinates/FrameVehicle.hpp>
#include <vulcan/coordinates/TransformChain.hpp>
#include <vulcan/coordinates/providers/ECEFProvider.hpp>
#include <vulcan/coordinates/providers/EarthProviders.hpp>
#include <vulcan/coordinates/providers/VehicleProviders.hpp>
#include <vulcan/core/VulcanError.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace vulcan {

template <typename Scalar> class FrameContext {
  public:
    FrameContext() : registry_(FrameRegistry::default_aerospace()) {
        initialize_storage();
    }

    explicit FrameContext(FrameRegistry registry)
        : registry_(std::move(registry)) {
        initialize_storage();
    }

    // =========================================================================
    // Frame setup
    // =========================================================================

    void set_ecef(Scalar rotation_angle) {
        providers_[FRAME_ECEF.id] =
            std::make_shared<ECEFProvider<Scalar>>(rotation_angle);

        frames_[FRAME_ECEF.id] = CoordinateFrame<Scalar>::ecef();
        frames_[FRAME_ECI.id] = CoordinateFrame<Scalar>::eci(rotation_angle);
    }

    void set_ecef(const EarthRotationModel &model, double t_seconds) {
        providers_[FRAME_ECEF.id] =
            std::make_shared<ECEFProvider<Scalar>>(model, t_seconds);

        frames_[FRAME_ECEF.id] = CoordinateFrame<Scalar>::ecef();
        const Scalar angle = Scalar(model.ecef_to_eci_angle(t_seconds));
        frames_[FRAME_ECI.id] = CoordinateFrame<Scalar>::eci(angle);
    }

    void set_ned(Scalar lon, Scalar lat) {
        auto provider = std::make_shared<NEDProvider<Scalar>>(lon, lat);
        providers_[FRAME_NED.id] = provider;
        frames_[FRAME_NED.id] = provider->frame();
    }

    void set_enu(Scalar lon, Scalar lat) {
        auto provider = std::make_shared<ENUProvider<Scalar>>(lon, lat);
        providers_[FRAME_ENU.id] = provider;
        frames_[FRAME_ENU.id] = provider->frame();
    }

    void set_geocentric(Scalar lon, Scalar lat_gc) {
        auto provider =
            std::make_shared<GeocentricProvider<Scalar>>(lon, lat_gc);
        providers_[FRAME_GEOCENTRIC.id] = provider;
        frames_[FRAME_GEOCENTRIC.id] = provider->frame();
    }

    void set_rail(const LLA<Scalar> &origin, Scalar azimuth, Scalar elevation,
                  const EarthModel &m = EarthModel::WGS84()) {
        auto provider = std::make_shared<RailProvider<Scalar>>(origin, azimuth,
                                                               elevation, m);
        providers_[FRAME_RAIL.id] = provider;
        frames_[FRAME_RAIL.id] = provider->frame();
    }

    void set_cda(const LLA<Scalar> &origin, Scalar bearing,
                 const EarthModel &m = EarthModel::WGS84()) {
        auto provider =
            std::make_shared<CDAProvider<Scalar>>(origin, bearing, m);
        providers_[FRAME_CDA.id] = provider;
        frames_[FRAME_CDA.id] = provider->frame();
    }

    void set_body_euler(Scalar yaw, Scalar pitch, Scalar roll) {
        providers_[FRAME_BODY.id] =
            std::make_shared<BodyProvider<Scalar>>(yaw, pitch, roll);

        if (has_cached_frame(FRAME_NED)) {
            frames_[FRAME_BODY.id] =
                body_from_euler(*frames_[FRAME_NED.id], yaw, pitch, roll);
        } else {
            frames_[FRAME_BODY.id].reset();
        }
    }

    void set_body_quaternion(const janus::Quaternion<Scalar> &q) {
        providers_[FRAME_BODY.id] = std::make_shared<BodyProvider<Scalar>>(q);

        if (has_cached_frame(FRAME_NED)) {
            frames_[FRAME_BODY.id] =
                body_from_quaternion(*frames_[FRAME_NED.id], q);
        } else {
            frames_[FRAME_BODY.id].reset();
        }
    }

    void set_wind(Scalar alpha, Scalar beta) {
        providers_[FRAME_WIND.id] =
            std::make_shared<WindProvider<Scalar>>(alpha, beta);
        frames_[FRAME_WIND.id].reset();
    }

    void set_stability(Scalar alpha) {
        providers_[FRAME_STABILITY.id] =
            std::make_shared<StabilityProvider<Scalar>>(alpha);
        frames_[FRAME_STABILITY.id].reset();
    }

    /// Set a frame using CoordinateFrame math (requires parent=ECEF).
    void set_frame(FrameID id, const CoordinateFrame<Scalar> &frame) {
        if (!registry_.has_frame(id)) {
            throw CoordinateError("Cannot set unregistered frame");
        }
        if (id == FRAME_ECI) {
            frames_[FRAME_ECI.id] = frame;
            return;
        }
        if (registry_.parent_of(id) != FRAME_ECEF) {
            throw CoordinateError("set_frame() requires frame parent to be "
                                  "ECEF; use set_provider()");
        }

        providers_[id.id] =
            std::make_shared<CoordinateFrameProvider<Scalar>>(frame);
        frames_[id.id] = frame;
    }

    void set_provider(FrameID id,
                      std::shared_ptr<TransformProvider<Scalar>> provider) {
        if (!registry_.has_frame(id)) {
            throw CoordinateError("Cannot set provider for unregistered frame");
        }
        if (id == FRAME_ECI) {
            throw CoordinateError("Root frame cannot have a provider");
        }
        if (!provider) {
            throw CoordinateError("Provider cannot be null");
        }
        ensure_capacity(id);
        providers_[id.id] = std::move(provider);
        frames_[id.id].reset();
    }

    FrameID add_frame(const std::string &name, FrameID parent,
                      std::shared_ptr<TransformProvider<Scalar>> provider) {
        if (!provider) {
            throw CoordinateError("Provider cannot be null");
        }
        FrameID id = registry_.register_frame(name, parent);
        ensure_capacity(id);
        providers_[id.id] = std::move(provider);
        frames_[id.id].reset();
        return id;
    }

    // =========================================================================
    // Transform operations
    // =========================================================================

    [[nodiscard]] Vec3<Scalar> transform(const Vec3<Scalar> &v, FrameID from,
                                         FrameID to) const {
        assert_registered(from);
        assert_registered(to);
        if (from == to) {
            return v;
        }

        // Fast path: direct child -> parent
        if (registry_.parent_of(from) == to) {
            return provider_for_child(from)->to_parent(v);
        }

        // Fast path: direct parent -> child
        if (registry_.parent_of(to) == from) {
            return provider_for_child(to)->from_parent(v);
        }

        return chain(from, to).transform_vector(v);
    }

    [[nodiscard]] Vec3<Scalar> transform_position(const Vec3<Scalar> &pos,
                                                  FrameID from,
                                                  FrameID to) const {
        assert_registered(from);
        assert_registered(to);
        if (from == to) {
            return pos;
        }

        // Fast path: direct child -> parent
        if (registry_.parent_of(from) == to) {
            return provider_for_child(from)->position_to_parent(pos);
        }

        // Fast path: direct parent -> child
        if (registry_.parent_of(to) == from) {
            return provider_for_child(to)->position_from_parent(pos);
        }

        return chain(from, to).transform_position(pos);
    }

    [[nodiscard]] TransformChain<Scalar> chain(FrameID from, FrameID to) const {
        assert_registered(from);
        assert_registered(to);
        if (from == to) {
            FramePath identity_path;
            identity_path.frames = {from};
            identity_path.lca = from;
            identity_path.ascending_count = 0;
            identity_path.descending_count = 0;
            return TransformChain<Scalar>(identity_path, {}, {});
        }

        const FramePath path = registry_.find_path(from, to);
        if (!path.is_valid()) {
            throw CoordinateError("No transform path between requested frames");
        }
        return build_chain(path);
    }

    // =========================================================================
    // Backward-compatible frame access
    // =========================================================================

    [[nodiscard]] const CoordinateFrame<Scalar> &frame(FrameID id) const {
        assert_registered(id);
        if (id.id >= frames_.size() || !frames_[id.id].has_value()) {
            throw CoordinateError(
                "Requested frame is not available as CoordinateFrame");
        }
        return *frames_[id.id];
    }

    [[nodiscard]] const FrameRegistry &registry() const { return registry_; }

  private:
    FrameRegistry registry_;
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> providers_;
    std::vector<std::optional<CoordinateFrame<Scalar>>> frames_;

    void initialize_storage() {
        providers_.resize(registry_.size());
        frames_.resize(registry_.size());

        // Default aligned inertial/ECEF state at angle = 0.
        providers_[FRAME_ECEF.id] =
            std::make_shared<ECEFProvider<Scalar>>(Scalar(0));
        frames_[FRAME_ECEF.id] = CoordinateFrame<Scalar>::ecef();
        frames_[FRAME_ECI.id] = CoordinateFrame<Scalar>::eci(Scalar(0));
    }

    void ensure_capacity(FrameID id) {
        const size_t needed = static_cast<size_t>(id.id + 1);
        if (providers_.size() < needed) {
            providers_.resize(needed);
        }
        if (frames_.size() < needed) {
            frames_.resize(needed);
        }
    }

    void assert_registered(FrameID id) const {
        if (!registry_.has_frame(id)) {
            throw CoordinateError("Frame is not registered in this context");
        }
    }

    [[nodiscard]] bool has_cached_frame(FrameID id) const {
        return id.id < frames_.size() && frames_[id.id].has_value();
    }

    [[nodiscard]] std::shared_ptr<TransformProvider<Scalar>>
    provider_for_child(FrameID child) const {
        if (child.id >= providers_.size() || !providers_[child.id]) {
            throw CoordinateError("Missing provider for frame edge");
        }
        return providers_[child.id];
    }

    [[nodiscard]] TransformChain<Scalar>
    build_chain(const FramePath &path) const {
        std::vector<std::shared_ptr<TransformProvider<Scalar>>> ascending;
        std::vector<std::shared_ptr<TransformProvider<Scalar>>> descending;
        ascending.reserve(static_cast<size_t>(path.ascending_count));
        descending.reserve(static_cast<size_t>(path.descending_count));

        // Ascend source -> LCA. For edge child->parent, provider lives on
        // child.
        for (int i = 0; i < path.ascending_count; ++i) {
            const FrameID child = path.frames[static_cast<size_t>(i)];
            const FrameID parent = path.frames[static_cast<size_t>(i + 1)];
            if (registry_.parent_of(child) != parent) {
                throw CoordinateError(
                    "Invalid path topology while building chain");
            }
            ascending.push_back(provider_for_child(child));
        }

        // Descend LCA -> target. For each parent->child step, use child's
        // provider.
        const int start = path.ascending_count + 1;
        for (int i = start; i < static_cast<int>(path.frames.size()); ++i) {
            const FrameID parent = path.frames[static_cast<size_t>(i - 1)];
            const FrameID child = path.frames[static_cast<size_t>(i)];
            if (registry_.parent_of(child) != parent) {
                throw CoordinateError(
                    "Invalid path topology while building chain");
            }
            descending.push_back(provider_for_child(child));
        }

        return TransformChain<Scalar>(path, ascending, descending);
    }
};

} // namespace vulcan
