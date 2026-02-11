// Vulcan Earth-Local Providers
// Providers for frames that are children of ECEF
#pragma once

#include <vulcan/coordinates/FrameLocal.hpp>
#include <vulcan/coordinates/FramePrimitives.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/TransformProvider.hpp>

namespace vulcan {

template <typename Scalar>
class NEDProvider final : public TransformProvider<Scalar> {
  public:
    NEDProvider(Scalar lon, Scalar lat)
        : frame_(CoordinateFrame<Scalar>::ned(lon, lat)) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_ned) const override {
        return frame_.to_ecef(v_ned);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos_ned) const override {
        return frame_.position_to_ecef(pos_ned);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos_ecef) const override {
        return frame_.position_from_ecef(pos_ecef);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

template <typename Scalar>
class ENUProvider final : public TransformProvider<Scalar> {
  public:
    ENUProvider(Scalar lon, Scalar lat)
        : frame_(CoordinateFrame<Scalar>::enu(lon, lat)) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_enu) const override {
        return frame_.to_ecef(v_enu);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos_enu) const override {
        return frame_.position_to_ecef(pos_enu);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos_ecef) const override {
        return frame_.position_from_ecef(pos_ecef);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

template <typename Scalar>
class GeocentricProvider final : public TransformProvider<Scalar> {
  public:
    GeocentricProvider(Scalar lon, Scalar lat_gc)
        : frame_(local_geocentric(lon, lat_gc)) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_local) const override {
        return frame_.to_ecef(v_local);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos_local) const override {
        return frame_.position_to_ecef(pos_local);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos_ecef) const override {
        return frame_.position_from_ecef(pos_ecef);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

template <typename Scalar>
class RailProvider final : public TransformProvider<Scalar> {
  public:
    RailProvider(const LLA<Scalar> &origin, Scalar azimuth, Scalar elevation,
                 const EarthModel &m = EarthModel::WGS84())
        : frame_(local_rail(origin, azimuth, elevation, m)) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_local) const override {
        return frame_.to_ecef(v_local);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos_local) const override {
        return frame_.position_to_ecef(pos_local);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos_ecef) const override {
        return frame_.position_from_ecef(pos_ecef);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

template <typename Scalar>
class CDAProvider final : public TransformProvider<Scalar> {
  public:
    CDAProvider(const LLA<Scalar> &origin, Scalar bearing,
                const EarthModel &m = EarthModel::WGS84())
        : frame_(local_cda(origin, bearing, m)) {}

    [[nodiscard]] Vec3<Scalar>
    to_parent(const Vec3<Scalar> &v_local) const override {
        return frame_.to_ecef(v_local);
    }

    [[nodiscard]] Vec3<Scalar>
    from_parent(const Vec3<Scalar> &v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

    [[nodiscard]] Vec3<Scalar>
    position_to_parent(const Vec3<Scalar> &pos_local) const override {
        return frame_.position_to_ecef(pos_local);
    }

    [[nodiscard]] Vec3<Scalar>
    position_from_parent(const Vec3<Scalar> &pos_ecef) const override {
        return frame_.position_from_ecef(pos_ecef);
    }

    [[nodiscard]] const CoordinateFrame<Scalar> &frame() const {
        return frame_;
    }

  private:
    CoordinateFrame<Scalar> frame_;
};

} // namespace vulcan
