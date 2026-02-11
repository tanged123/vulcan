// Vulcan Transform Chain
// Executes a pre-resolved frame path using transform providers
#pragma once

#include <vulcan/coordinates/FrameRegistry.hpp>
#include <vulcan/coordinates/TransformProvider.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <memory>
#include <vector>

namespace vulcan {

template <typename Scalar> class TransformChain {
  public:
    TransformChain(const FramePath &path,
                   const std::vector<std::shared_ptr<TransformProvider<Scalar>>>
                       &ascending_providers,
                   const std::vector<std::shared_ptr<TransformProvider<Scalar>>>
                       &descending_providers)
        : path_(path), ascending_(ascending_providers),
          descending_(descending_providers) {}

    [[nodiscard]] Vec3<Scalar> transform_vector(const Vec3<Scalar> &v) const {
        Vec3<Scalar> result = v;

        for (const auto &provider : ascending_) {
            result = provider->to_parent(result);
        }
        for (const auto &provider : descending_) {
            result = provider->from_parent(result);
        }

        return result;
    }

    [[nodiscard]] Vec3<Scalar>
    transform_position(const Vec3<Scalar> &pos) const {
        Vec3<Scalar> result = pos;

        for (const auto &provider : ascending_) {
            result = provider->position_to_parent(result);
        }
        for (const auto &provider : descending_) {
            result = provider->position_from_parent(result);
        }

        return result;
    }

    [[nodiscard]] int length() const {
        return static_cast<int>(ascending_.size() + descending_.size());
    }

    [[nodiscard]] const FramePath &path() const { return path_; }

  private:
    FramePath path_;
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> ascending_;
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> descending_;
};

} // namespace vulcan
