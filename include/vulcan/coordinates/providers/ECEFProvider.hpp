// Vulcan ECEF Provider
// ECEF <-> ECI transform provider with configurable Earth rotation input
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/TransformProvider.hpp>

#include <janus/math/Trig.hpp>

#include <memory>

namespace vulcan {

enum class EarthRotationFidelity {
    ConstantOmega,
    GMST,
    IAU2006,
};

/// Provider for the ECEF(child) <-> ECI(parent) edge.
template <typename Scalar>
class ECEFProvider final : public TransformProvider<Scalar> {
  public:
    explicit ECEFProvider(Scalar rotation_angle) {
        c_ = janus::cos(rotation_angle);
        s_ = janus::sin(rotation_angle);
    }

    ECEFProvider(const EarthRotationModel &model, double t_seconds) {
        const Scalar angle = Scalar(model.ecef_to_eci_angle(t_seconds));
        c_ = janus::cos(angle);
        s_ = janus::sin(angle);
    }

    /// ECEF -> ECI
    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_ecef) const override {
        Vec3<Scalar> result;
        result(0) = c_ * v_ecef(0) - s_ * v_ecef(1);
        result(1) = s_ * v_ecef(0) + c_ * v_ecef(1);
        result(2) = v_ecef(2);
        return result;
    }

    /// ECI -> ECEF
    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_eci) const override {
        Vec3<Scalar> result;
        result(0) = c_ * v_eci(0) + s_ * v_eci(1);
        result(1) = -s_ * v_eci(0) + c_ * v_eci(1);
        result(2) = v_eci(2);
        return result;
    }

  private:
    Scalar c_{};
    Scalar s_{};
};

template <typename Scalar>
[[nodiscard]] inline std::shared_ptr<ECEFProvider<Scalar>>
make_ecef_provider(Scalar angle) {
    return std::make_shared<ECEFProvider<Scalar>>(angle);
}

template <typename Scalar>
[[nodiscard]] inline std::shared_ptr<ECEFProvider<Scalar>>
make_ecef_provider(const EarthRotationModel &model, double t_seconds) {
    return std::make_shared<ECEFProvider<Scalar>>(model, t_seconds);
}

} // namespace vulcan
