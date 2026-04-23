// Vulcan Vehicle Providers
// Providers for vehicle-relative frame edges (Body/Wind/Stability)
#pragma once

#include <vulcan/coordinates/TransformProvider.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <metis/math/Quaternion.hpp>
#include <metis/math/Trig.hpp>

namespace vulcan {

/// Body(child) <-> NED(parent) provider.
template <typename Scalar>
class BodyProvider final : public TransformProvider<Scalar> {
  public:
    BodyProvider(Scalar yaw, Scalar pitch, Scalar roll)
        : q_(metis::Quaternion<Scalar>::from_euler(roll, pitch, yaw)) {}

    explicit BodyProvider(const metis::Quaternion<Scalar> &q_body_to_ned)
        : q_(q_body_to_ned) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_body) const override {
        return q_.rotate(v_body);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ned) const override {
        return q_.conjugate().rotate(v_ned);
    }

    [[nodiscard]] const metis::Quaternion<Scalar> &quaternion() const {
        return q_;
    }

  private:
    metis::Quaternion<Scalar> q_;
};

/// Wind(child) <-> Body(parent) provider.
template <typename Scalar>
class WindProvider final : public TransformProvider<Scalar> {
  public:
    WindProvider(Scalar alpha, Scalar beta) {
        const Scalar ca = metis::cos(alpha);
        const Scalar sa = metis::sin(alpha);
        const Scalar cb = metis::cos(beta);
        const Scalar sb = metis::sin(beta);

        // v_body = R * v_wind
        R_(0, 0) = ca * cb;
        R_(0, 1) = -ca * sb;
        R_(0, 2) = -sa;

        R_(1, 0) = sb;
        R_(1, 1) = cb;
        R_(1, 2) = Scalar(0);

        R_(2, 0) = sa * cb;
        R_(2, 1) = -sa * sb;
        R_(2, 2) = ca;
    }

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_wind) const override {
        return R_ * v_wind;
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_body) const override {
        return R_.transpose() * v_body;
    }

    [[nodiscard]] const Mat3<Scalar> &dcm() const { return R_; }

  private:
    Mat3<Scalar> R_;
};

/// Stability(child) <-> Body(parent) provider.
template <typename Scalar>
class StabilityProvider final : public TransformProvider<Scalar> {
  public:
    explicit StabilityProvider(Scalar alpha) {
        const Scalar ca = metis::cos(alpha);
        const Scalar sa = metis::sin(alpha);

        // v_body = R * v_stability
        R_(0, 0) = ca;
        R_(0, 1) = Scalar(0);
        R_(0, 2) = -sa;

        R_(1, 0) = Scalar(0);
        R_(1, 1) = Scalar(1);
        R_(1, 2) = Scalar(0);

        R_(2, 0) = sa;
        R_(2, 1) = Scalar(0);
        R_(2, 2) = ca;
    }

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_stability) const override {
        return R_ * v_stability;
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_body) const override {
        return R_.transpose() * v_body;
    }

  private:
    Mat3<Scalar> R_;
};

} // namespace vulcan
