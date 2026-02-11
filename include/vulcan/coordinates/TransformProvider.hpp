// Vulcan Transform Providers
// Edge-local transform interface for frame-tree execution
#pragma once

#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Quaternion.hpp>

#include <memory>
#include <utility>

namespace vulcan {

/// Interface for a frame edge transform (child <-> parent).
template <typename Scalar> struct TransformProvider {
    virtual ~TransformProvider() = default;

    [[nodiscard]] virtual Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v) const = 0;

    [[nodiscard]] virtual Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v) const = 0;

    [[nodiscard]] virtual Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos) const {
        return to_parent(pos);
    }

    [[nodiscard]] virtual Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos) const {
        return from_parent(pos);
    }
};

/// Wrap a CoordinateFrame as a provider where parent is ECEF.
template <typename Scalar>
class CoordinateFrameProvider final : public TransformProvider<Scalar> {
  public:
    explicit CoordinateFrameProvider(CoordinateFrame<Scalar> frame)
        : frame_(std::move(frame)) {}

    [[nodiscard]] Vec3<Scalar> to_parent(const Vec3<Scalar> &v) const override {
        return frame_.to_ecef(v);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v) const override {
        return frame_.from_ecef(v);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos) const override {
        return frame_.position_to_ecef(pos);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos) const override {
        return frame_.position_from_ecef(pos);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

/// Provider backed by a DCM that maps child -> parent.
template <typename Scalar>
class DCMProvider final : public TransformProvider<Scalar> {
  public:
    explicit DCMProvider(const Mat3<Scalar> &R_child_to_parent)
        : R_(R_child_to_parent) {}

    [[nodiscard]] Vec3<Scalar> to_parent(const Vec3<Scalar> &v) const override {
        return R_ * v;
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v) const override {
        return R_.transpose() * v;
    }

  private:
    Mat3<Scalar> R_;
};

/// Provider backed by a quaternion that rotates child -> parent.
template <typename Scalar>
class QuaternionProvider final : public TransformProvider<Scalar> {
  public:
    explicit QuaternionProvider(
        const janus::Quaternion<Scalar> &q_child_to_parent)
        : q_(q_child_to_parent) {}

    [[nodiscard]] Vec3<Scalar> to_parent(const Vec3<Scalar> &v) const override {
        return q_.rotate(v);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v) const override {
        return q_.conjugate().rotate(v);
    }

  private:
    janus::Quaternion<Scalar> q_;
};

} // namespace vulcan
